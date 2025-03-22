// Based off lib/ukf_engl/ukf_engl.ino

#include "konfig.h"
#include "matrix.h"
#include "ukf.h"

extern "C" {
    #include "kalman.h"

    #include "FreeRTOS.h"
    #include "task.h"
    #include "semphr.h"

    #include <stdio.h>

    #include "constants.h"
    #include "imu.h"
    #include "encoder.h"
}



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


// Initial state covariance
#define SB  (0.5*(M_PI / 180.0 * 20)) // 20 degrees ~= 2 sigma (95% confidence) for roll
#define SG  (0.5*(M_PI / 180.0 * 10)) // 10 degrees ~= 2 sigma (95% confidence) for pitch
static const float_prec PINIT_data[SS_X_LEN*SS_X_LEN] = {
    0,  0,  0,  0,  0,      0,      // Define z=0
    0,  0,  0,  0,  0,      0,      // Know vz=0 (Car is still)
    0,  0,  0,  0,  0,      0,      // Define x=0
    0,  0,  0,  0,  0,      0,      // Know vx=0 (Car is still)
    0,  0,  0,  0,  SB*SB,  0,      // beta within ~20degrees 95% of the time
    0,  0,  0,  0,  0,      SG*SG,  // ga
};
#undef SB
#undef SG
static Matrix PINIT(SS_X_LEN, SS_X_LEN, PINIT_data);

// Process noise covariance (includes IMU)
#define Favg 833.0
#define SL  (0.001/Favg) // 1mm drift/s
#define SV  (0.001/Favg) // 1mm/s drift/s
#define SA  ((M_PI / 180.0 * 0.01)/Favg) // 0.01 degree drift/s
static const float_prec Rv_data[SS_X_LEN*SS_X_LEN] = { 
    SL*SL,  0,      0,      0,      0,      0,
    0,      SV*SV,  0,      0,      0,      0,
    0,      0,      SL*SL,  0,      0,      0,
    0,      0,      0,      SV*SV,  0,      0,
    0,      0,      0,      0,      SA*SA,  0, 
    0,      0,      0,      0,      0,      SA*SA,
};
#undef Favg
#undef SL
#undef SV
#undef SA
static Matrix Rv(SS_X_LEN, SS_X_LEN, Rv_data);

// Measurement noise covariance
#define SA  (0.5*(0.010)) // 10mm/s^2 ~= 2 sigma (95% confidence) for accelerometer
#define SP  (0.5*(0.5)) // 0.5m ~= 2 sigma (95% confidence) for encoder position
#define SV  (1*(0.001)) // 1mm/s ~= 2 sigma (95% confidence) for encoder speed
#define Z   1 // Made up value to stop Z from wandering off
#define VZ  1 // Made up value to stop VZ from wandering off
static const float_prec Rn_data[SS_Z_LEN*SS_Z_LEN] = {
    SA*SA,  0,      0,      0,      0,      0,      0,
    0,      SA*SA,  0,      0,      0,      0,      0,
    0,      0,      SA*SA,  0,      0,      0,      0,
    0,      0,      0,      SP*SP,  0,      0,      0,
    0,      0,      0,      0,      SV*SV,  0,      0,
    0,      0,      0,      0,      0,      Z,      0,
    0,      0,      0,      0,      0,      0,      VZ,
};
#undef SA
#undef SP
#undef SV
#undef Z
#undef VZ
static Matrix Rn(SS_Z_LEN, SS_Z_LEN, Rn_data);

// Filter
static bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
static bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);
static UKF ukf(Matrix(SS_X_LEN, 1), PINIT, Rv, Rn, Main_bUpdateNonlinearX, Main_bUpdateNonlinearY);


// Not included in state, but want to track anyway
static float_prec vbeta, vgamma;

// Cached state for returning to other threads
static kalmanState_t cachedState;
static StaticSemaphore_t cacheMutexBuffer;
static SemaphoreHandle_t cacheMutex;


// Keep track of time between kalman filter steps
static uint64_t prevMicros;
static float_prec dt;

// FreeRTOS task
static StaticTask_t kalmanTaskBuffer;
static StackType_t kalmanStackBuffer[2000];
static TaskHandle_t kalmanTask;



static void kalmanTaskFunc(void *);

TaskHandle_t kalmanSetup() {
    cacheMutex = xSemaphoreCreateMutexStatic(&cacheMutexBuffer);
    // Lower priority (2) for kalman math
    kalmanTask = xTaskCreateStatic(kalmanTaskFunc, "kalmanTask", sizeof(kalmanStackBuffer)/sizeof(StackType_t), NULL, 2, kalmanStackBuffer, &kalmanTaskBuffer);
    return kalmanTask;
}

void kalmanGetState(kalmanState_t *buf) {
    xSemaphoreTake(cacheMutex, portMAX_DELAY);
    *buf = cachedState;
    xSemaphoreGive(cacheMutex);
}

static void kalmanTaskFunc(void *) {
    // Wait for notification from IMU code
    while (true) {
        if (ulTaskNotifyTake(true, portMAX_DELAY)) {
            // Get data
            imuData_t imuData;
            imuGetData(&imuData);

            if (isnan(imuData.Gx) || isnan(imuData.Gy) || isnan(imuData.Gz) || isnan(imuData.Ax) || isnan(imuData.Ay) || isnan(imuData.Az)) {
                printf("IMU Nan!\n");
                continue;
            }

            float position, speed;
            encoderRead(&position, &speed);

            // Update dt (used in predict step)
            // Should be relatively constant since triggered by IMU interrupt, but good to check
            dt = 0.000001 * (imuData.micros - prevMicros);
            prevMicros = imuData.micros;
            
            const float_prec Y_data[SS_Z_LEN] = {
                imuData.Ax,
                imuData.Ay,
                imuData.Az,
                position,
                speed,
                0,
                0,
            };
            Matrix Y(SS_Z_LEN, 1, Y_data);

            const float_prec U_data[SS_U_LEN] = {
                imuData.Gx,
                imuData.Gy,
                imuData.Gz,
                imuData.Ax,
                imuData.Ay,
                imuData.Az,
            };
            Matrix U(SS_U_LEN, 1, U_data);

            // Update kalman filter
            if (!ukf.bUpdate(Y, U)) {
                // Use better reset for X?
                printf("Kalman Error!\n");
                ukf.vReset(Matrix(SS_X_LEN, 1), PINIT, Rv, Rn);
            }
            

            // Save results to cache
            Matrix X = ukf.GetX();
            xSemaphoreTake(cacheMutex, portMAX_DELAY);
            cachedState.micros = prevMicros;
            cachedState.z = X[0][0];
            cachedState.vz = X[1][0];
            cachedState.x = X[2][0];
            cachedState.vx = X[3][0];
            cachedState.beta = X[4][0];
            cachedState.vbeta = vbeta;
            cachedState.gamma = X[5][0];
            cachedState.vgamma = vgamma;
            xSemaphoreGive(cacheMutex);
        }
    }
}


static bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U)
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
        (1*cg - 0*sb*sg),   (cg*0 + 1*sb*sg),   (-cb*sg),
        (-cb*0),            (1*cb),            (sb),
        (1*sg + cg*0*sb),   (0*sg - 1*cg*sb),   (cb*cg),
    };
    Matrix Rotation(3, 3, Rotation_data);

    float_prec IMU_data[3*2] = {
        Ax, Gx,
        Ay, Gy, 
        Az, Gz,
    };
    Matrix IMU(3, 2, IMU_data);
    
    Matrix U_Trans = Rotation * IMU;
    const float_prec az = U_Trans[2][0];
    const float_prec ax = U_Trans[0][0];
    // Save vbeta and vgamma to static variables to save for later
    vbeta = U_Trans[0][0];
    vgamma = cb*U_Trans[1][0] + sb*U_Trans[1][0];
    
    X_Next[0][0] = z + vz*dt + az*0.5*dt*dt;
    X_Next[1][0] = vz + az*dt;
    X_Next[2][0] = x + vx*dt + ax*0.5*dt*dt;
    X_Next[3][0] = vx + ax*dt;
    
    X_Next[4][0] = beta + vbeta*dt;
    X_Next[5][0] = gamma + vgamma*dt;

    return true; // succcess
}

static bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U)
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

    return true; // succcess
}




// Used in assertions in matrix math
void SPEW_THE_ERROR(char const * str)
{
    #if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
        cout << (str) << endl;
    #elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
        Serial.println(str);
    #else
        printf("%s\n", str);
    #endif
    while(1);
}
