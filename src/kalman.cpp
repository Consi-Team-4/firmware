// Functions available in the c code
extern "C" {
    #include "kalman.h"
}

// Based off lib/ukf_engl/ukf_engl.ino

#include "konfig.h"
#include "matrix.h"
#include "ukf.h"


#include "FreeRTOS.h"
#include "task.h"

#include "constants.h"
#include "imu.h"
#include "encoder.h"

/*
Typically, the IMU is used as U in the predict step and the angular velocity/linear acceleration is excluded from the state vector X.
I'm following that, but also keeping track of the angular velocity for later.
It will help with PID/any controller that uses velocity.


Frame of reference:
Z is up
X is forwards
Y is left (not tracking)

Using zx'y'' tait bryan angles https://upload.wikimedia.org/wikipedia/commons/thumb/e/ea/Taitbrianangles.svg/1920px-Taitbrianangles.svg.png
Note: typically, zy'x'' is used for yaw, pitch and roll (in that order). However, because of the way the suspension works, we want roll before pitch
alpha is yaw (not tracking)
beta is roll (left side up)
gamma is pitch (nose down)


X vector:
    z
    vz
    x
    vx
    beta
    gamma
    
U vector:
    IMU:
        Gx
        Gy
        Gz
        Ax
        Ay
        Az

Y vector:
    IMU:
        Ax
        Ay
        Az (use gravity to correct orientation)
    Encoder:
        position
        speed
    Zero bias loose assumptions:
        vz = 0
        z = 0
*/


// Not included in state, but want to keep track anyways
static float_prec vbeta, vgamma;
static uint64_t prevMicros;
static float_prec dt;




/* UKF initialization constant -------------------------------------------------------------------------------------- */
#define P_INIT      (10.)
#define Rv_INIT     (1e-6)
#define Rn_INIT     (0.0015)
/* P(k=0) variable -------------------------------------------------------------------------------------------------- */
float_prec UKF_PINIT_data[SS_X_LEN*SS_X_LEN] = {P_INIT, 0,
                                                0,      P_INIT};
Matrix UKF_PINIT(SS_X_LEN, SS_X_LEN, UKF_PINIT_data);
/* Rv constant ------------------------------------------------------------------------------------------------------ */
float_prec UKF_RVINIT_data[SS_X_LEN*SS_X_LEN] = {Rv_INIT, 0,
                                                 0,      Rv_INIT};
Matrix UKF_RvINIT(SS_X_LEN, SS_X_LEN, UKF_RVINIT_data);
/* Rn constant ------------------------------------------------------------------------------------------------------ */
float_prec UKF_RNINIT_data[SS_Z_LEN*SS_Z_LEN] = {Rn_INIT};
Matrix UKF_RnINIT(SS_Z_LEN, SS_Z_LEN, UKF_RNINIT_data);
/* Nonlinear & linearization function ------------------------------------------------------------------------------- */
bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
/* UKF variables ---------------------------------------------------------------------------------------------------- */
Matrix X(SS_X_LEN, 1);
Matrix Y(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
/* UKF system declaration ------------------------------------------------------------------------------------------- */
UKF UKF_IMU(X, UKF_PINIT, UKF_RvINIT, UKF_RnINIT, Main_bUpdateNonlinearX, Main_bUpdateNonlinearY);



/* ========================================= Auxiliary variables/function declaration ========================================= */
elapsedMillis timerLed, timerUKF;
uint64_t u64compuTime;
char bufferTxSer[100];



void setup() {
    /* serial to display data */
    Serial.begin(115200);
    while(!Serial) {}
    
    X.vSetToZero();
    UKF_IMU.vReset(X, UKF_PINIT, UKF_RvINIT, UKF_RnINIT);
}


void kalmanTaskFunc(void *) {
    // Wait for notification from ISR
    while (true) {
        if (ulTaskNotifyTake(true, portMAX_DELAY)) {
            // Update sensors

            imuData_t imuData;
            imuGetData(&imuData);

            float position, speed;
            encoderRead(&position, &speed);


            // Update dt (used in predict step)
            // Should be relatively constant since triggered by IMU interrupt, but good to check
            dt = 0.000001 * (imuData.micros - prevMicros);
            prevMicros = imuData.micros;

            Y[0][0] = imuData.Ax;
            Y[1][0] = imuData.Ay;
            Y[2][0] = imuData.Az;
            Y[3][0] = position;
            Y[4][0] = speed;
            Y[5][0] = 0;
            Y[6][0] = 0;


            U[0][0] = imuData.Gx;
            U[1][0] = imuData.Gy;
            U[2][0] = imuData.Gz;
            U[3][0] = imuData.Ax;
            U[4][0] = imuData.Ay;
            U[5][0] = imuData.Az;


            /* ============================= Update the Kalman Filter ============================== */
            if (!UKF_IMU.bUpdate(Y, U)) {
                // Use better reset for X?
                X.vSetToZero();
                UKF_IMU.vReset(X, UKF_PINIT, UKF_RvINIT, UKF_RnINIT);
            }
        }
    }
}


bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U)
{
    const float_prec Gx = U[0][0];
    const float_prec Gy = U[1][0];
    const float_prec Gz = U[2][0];
    const float_prec Ax = U[3][0];
    const float_prec Ay = U[4][0];
    const float_prec Az = U[5][0];

    const float_prec z = X[0][0];
    const float_prec vz = X[1][0];
    const float_prec x = X[2][0];
    const float_prec vx = X[3][0];
    const float_prec beta = X[4][0];
    const float_prec gamma = X[5][0];
    
    // Convert imu frame of reference to world frame of reference
    
    // https://en.wikipedia.org/wiki/Euler_angles
    // Inverse of a rotation matrix is its transpose, so using the transpose of the ZXY Tait-Bryan
    // Since alpha is always 0, ca = 1, sa=0

    const float_prec cb = cos(beta);
    const float_prec sb = sin(beta);
    const float_prec cg = cos(beta);
    const float_prec sg = sin(gamma);

    float_prec Rotation_data[3*3] = {
        (1*cg - 0*sb*sg),   (cg*0 + 1*sb*sg),   (-cb*sg)
        (-cb*0),            (ca*cb),            (sb)
        (1*sg + cg*0*sb),   (0*sg - 1*cg*sb),   (cb*cg)
    };
    Matrix Rotation(3, 3, Rotation_data);

    float_prec IMU_data[3*2] = {
        Ax, Gx,
        Ay, Gy, 
        Az, Gz
    };
    Matrix IMU(3, 2, IMU_data);
    
    Matrix U_Trans = Rotation * IMU;
    const float_prec az = U_Trans[2][0];
    const float_prec ax = U_Trans[0][0];
    const float_prec vbeta = U_Trans[0][0];
    const float_prec vgamma = cb*U_Trans[1][0] + sb*U_Trans[1][0];
    
    X_Next[0][0] = z + vz*dt + az*0.5*dt*dt;
    X_Next[1][0] = vz + az*dt;
    X_Next[2][0] = x + vx*dt + ax*0.5*dt*dt;
    X_Next[3][0] = vx + ax*dt;
    
    X_Next[4][0] = beta + vbeta*dt;
    X_Next[5][0] = gamma + vgamma*dt;
}

bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U)
{
    // Not sure why U is here, and it also isn't used in the example, so ?

    const float_prec z = X[0][0];
    const float_prec vz = X[1][0];
    const float_prec x = X[2][0];
    const float_prec vx = X[3][0];
    const float_prec beta = X[4][0];
    const float_prec gamma = X[5][0];


    // Convert world frame of reference to imu frame of reference
    
    // https://en.wikipedia.org/wiki/Euler_angles
    // Inverse of a rotation matrix is its transpose, so using the transpose of the ZXY Tait-Bryan
    // Since alpha is always 0, ca = 1, sa=0

    const float_prec cb = cos(beta);
    const float_prec sb = sin(beta);
    const float_prec cg = cos(beta);
    const float_prec sg = sin(gamma);

    
    // Rotation from rotation matrix in Main_bUpdateNonlinearX
    Y[0][0] = -GRAVITY * -cb*sg;
    Y[1][0] = -GRAVITY * cb;
    Y[2][0] = -GRAVITY * cb*cg;
    
    // Encoder
    Y[3][0] = x; // Will always underestimate the encoder position - if car is driving up a slope, encoder position will increase faster than x. But error should be small enough it's okay.
    Y[4][0] = vx*cg + vz*sg;
    
    Y[5][0] = z;
    Y[6][0] = vz;
}





void SPEW_THE_ERROR(char const * str)
{
    #if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
        cout << (str) << endl;
    #elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
        Serial.println(str);
    #else
        /* Silent function */
    #endif
    while(1);
}
