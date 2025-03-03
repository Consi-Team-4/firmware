# ifndef LSM6DSOX_DEFINES_H
# define LSM6DSOX_DEFINES_H


#define LSM6DSOX_ADDRESS        0x6A

#define REG_FUNC_CFG_ACCESS     0x01 // Required for embedded functions or sensor hub functionality
#define REG_PIN_CTRL            0x02

// Sensor synchronization
#define REG_S4S_TPH_L           0x04
#define REG_S4S_TPH_H           0x05
#define REG_S4S_RR              0x06

// FIFO and batching Control
#define REG_FIFO_CTRL1          0x07
#define REG_FIFO_CTRL2          0x08
#define REG_FIFO_CTRL3          0x09
#define REG_FIFO_CTRL4          0x0A
#define REG_COUNTER_BDR_REG1    0x0B
#define REG_COUNTER_BDR_REG2    0x0C

// Controls behavior of the interrupt pins (INT1 & INT2)
#define REG_INT1_CTRL           0x0D
#define REG_INT2_CTRL           0x0E

// Read only WHO_AM_I register
#define REG_WHO_AM_I            0x0F
#define WHO_AM_I_VAL            0x6C // Value fixed at this

// Main control registers
#define REG_CTRL1_XL            0x10
#define REG_CTRL2_G             0x11
#define REG_CTRL3_C             0x12
#define REG_CTRL4_C             0x13
#define REG_CTRL5_C             0x14
#define REG_CTRL6_C             0x15
#define REG_CTRL7_G             0x16
#define REG_CTRL8_XL            0x17
#define REG_CTRL9_XL            0x18
#define REG_CTRL10_C            0x19

// Interrupt Source Registers
#define REG_ALL_INT_SRC         0x1A
#define REG_WAKE_UP_SRC         0x1B
#define REG_TAP_SRC             0x1C
#define REG_D6D_SRC             0x1D
#define REG_STATUS_REG          0x1E // Status register indicates if new data available

// Output data registers
// All are 16 bit registers, little endian (ie. address of H is <address of L> + 1)
#define REG_OUT_TEMP            0x20
#define REG_OUTX_G              0x22
#define REG_OUTY_G              0x24
#define REG_OUTZ_G              0x26
#define REG_OUTX_A              0x28
#define REG_OUTY_A              0x2A
#define REG_OUTZ_A              0x2C

// Embedded function interrupt status 
#define REG_EMB_FUNC_STATUS_MAINPAGE    0x35
#define REG_FSM_STATUS_A_MAINPAGE       0x36
#define REG_FSM_STATUS_B_MAINPAGE       0x37

// Sensor hub interrupt status
#define REG_MLC_STATUS_MAINPAGE         0x38
#define REG_STATUS_MASTER_MAINPAGE      0x39

// FIFO status
#define REG_FIFO_STATUS1        0x3A
#define REG_FIFO_STATUS2        0x3B

// Timestamp
// 32 bit register (big endian?)
#define REG_TIMESTAMP           0x40

// OIS (Optical Image Stabilization) output registers
#define REG_UI_STATUS_REG_OIS   0x49
// All are 16 bit registers, little endian (ie. address of H is <address of L> + 1)
#define REG_UI_OUTX_G_OIS       0x4A
#define REG_UI_OUTY_G_OIS       0x4C
#define REG_UI_OUTZ_G_OIS       0x4E
#define REG_UI_OUTX_A_OIS       0x50
#define REG_UI_OUTY_A_OIS       0x52
#define REG_UI_OUTZ_A_OIS       0x54

// Embedded function configuration registers
#define REG_TAP_CFG0            0x56
#define REG_TAP_CFG1            0x57
#define REG_TAP_CFG2            0x58
#define REG_TAP_THS_6D          0x59
#define REG_INT_DUR2            0x5A
#define REG_WAKE_UP_THS         0x5B
#define REG_WAKE_UP_DUR         0x5C
#define REG_FREE_FALL           0x5D

// Embedded function interrupt enable registers
#define REG_MD1_CFG             0x5E
#define REG_MD2_CFG             0x5F

// Sensor hub stuff?
#define REG_S4S_ST_CMD_CODE     0x60
#define REG_S4S_DT_REG          0x61
#define REG_I3C_BUS_AVB         0x62

#define REG_INTERNAL_FREQ_FINE  0x63

// Absolutely no clue lmao
#define REG_UI_INT_OIS          0x6F
#define REG_UI_CTRL1_OIS        0x70
#define REG_UI_CTRL2_OIS        0x71
#define REG_UI_CTRL3_OIS        0x72

// Accelerometer correction values
#define REG_X_OFS_USR           0x73
#define REG_Y_OFS_USR           0x74
#define REG_Z_OFS_USR           0x75

// FIFO
#define REG_FIFO_DATA_OUT_TAG   0x78
// All are 16 bit registers, little endian (ie. address of H is <address of L> + 1)
#define REG_FIFO_DATA_OUT_X     0x79
#define REG_FIFO_DATA_OUT_Y     0x7B
#define REG_FIFO_DATA_OUT_Z     0x7D





#endif
