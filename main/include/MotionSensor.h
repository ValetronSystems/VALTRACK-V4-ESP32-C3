/***************************************************************************
* File name :   MotionSensor.h                                             *
*                                                                          *
* Author    :   Ravi Y. Pujar                                              *
*                                                                          *
* Owner     :   Copyright (c) 2023 Valetron Systems Pvt Ltd,               *
*                all rights reserved                                       *
***************************************************************************/
#ifndef _MOTIONSENSOR_H
#define _MOTIONSENSOR_H
// #define VALTRACK_V4MF

// #ifdef VALTRACK_V4_VTS
//     #define LIS3DH_ENABLED
// #else
//     #define MMA854_ENABLED
// #endif

// #ifdef MMA854_ENABLED
//     #define ACCLEROMETER_I2C_ADDRESS 0x1D  //MMA854
// #else 
//     #define ACCLEROMETER_I2C_ADDRESS 0x19  //LIS3D
// #endif



#define REG_CTRL_REG1  0x20
#define  REG_CTRL_REG2  0x21
#define  REG_CTRL_REG3  0x22
#define  REG_CTRL_REG4  0x23
#define  REG_CTRL_REG5  0x24
#define  REG_CTRL_REG6  0x25
//#define  REG_REFERENCE  0x26
//#define  REG_STATUS_REG  0x27
//#define  REG_OUT_X_L  0x28
//#define  REG_OUT_X_H  0x29
//#define  REG_OUT_Y_L  0x2a
//#define  REG_OUT_Y_H  0x2b
//#define  REG_OUT_Z_L  0x2c
//#define  REG_OUT_Z_H  0x2d
//#define  REG_FIFO_CTRL_REG  0x2e
//#define  REG_FIFO_SRC_REG  0x2f
#define  REG_INT1_CFG  0x30
#define  REG_INT1_SRC  0x31
#define  REG_INT1_THS  0x32
#define  REG_INT1_DURATION  0x33
//#define  REG_CLICK_CFG  0x38
//#define  REG_CLICK_SRC  0x39
//#define  REG_CLICK_THS  0x3a
//#define  REG_TIME_LIMIT  0x3b
//#define  REG_TIME_LATENCY  0x3c
//#define  REG_TIME_WINDOW  0x3d
//#define  REG_INT2_CFG  0x34


enum
{
  MMA8652_STATUS_00 = 0,          // 0x00
  MMA8652_OUT_X_MSB,              // 0x01
  MMA8652_OUT_X_LSB,              // 0x02
  MMA8652_OUT_Y_MSB,              // 0x03
  MMA8652_OUT_Y_lSB,              // 0x04
  MMA8652_OUT_Z_MSB,              // 0x05
  MMA8652_OUT_Z_LSB,              // 0x06
  MMA8652_RSVD_0,                 // 0x07  
  MMA8652_RSVD_1,                 // 0x08  
  MMA8652_F_SETUP,                // 0x09
  MMA8652_TRIG_CFG,               // 0x0A
  MMA8652_SYSMOD,                 // 0x0B
  MMA8652_INT_SOURCE,             // 0x0C
  MMA8652_WHO_AM_I,               // 0x0D
  MMA8652_XYZ_DATA_CFG,           // 0x0E
  MMA8652_HP_FILTER_CUTOFF,       // 0x0F
  MMA8652_PL_STATUS,              // 0x10
  MMA8652_PL_CFG,                 // 0x11
  MMA8652_PL_COUNT,               // 0x12
  MMA8652_PL_BF_ZCOMP,            // 0x13
  MMA8652_PL_P_L_THS_REG,         // 0x14
  MMA8652_FF_MT_CFG,              // 0x15
  MMA8652_FF_MT_SRC,              // 0x16
  MMA8652_FF_MT_THS,              // 0x17
  MMA8652_FF_MT_COUNT,            // 0x18
  MMA8652_RSVD_2,                 // 0x19  
  MMA8652_RSVD_3,                 // 0x1A  
  MMA8652_RSVD_4,                 // 0x1B  
  MMA8652_RSVD_5,                 // 0x1C  
  MMA8652_TRANSIENT_CFG,          // 0x1D
  MMA8652_TRANSIENT_SRC,          // 0x1E
  MMA8652_TRANSIENT_THS,          // 0x1F
  MMA8652_TRANSIENT_COUNT,        // 0x20
  MMA8652_PULSE_CFG,              // 0x21
  MMA8652_PULSE_SRC,              // 0x22
  MMA8652_PULSE_THSX,             // 0x23
  MMA8652_PULSE_THSY,             // 0x24
  MMA8652_PULSE_THSZ,             // 0x25
  MMA8652_PULSE_TMLT,             // 0x26
  MMA8652_PULSE_LTCY,             // 0x27
  MMA8652_PULSE_WIND,             // 0x28
  MMA8652_ASLP_COUNT,             // 0x29
  MMA8652_CTRL_REG1,              // 0x2A
  MMA8652_CTRL_REG2,              // 0x2B
  MMA8652_CTRL_REG3,              // 0x2C
  MMA8652_CTRL_REG4,              // 0x2D
  MMA8652_CTRL_REG5,              // 0x2E
  MMA8652_OFF_X,                  // 0x2F
  MMA8652_OFF_Y,                  // 0x30
  MMA8652_OFF_Z                   // 0x31
};

#define INT_CFG_ASLP_MASK     0x80
#define INT_CFG_FIFO_MASK     0x40     // MMA8652 only
#define INT_CFG_TRANS_MASK    0x20     // MMA8652 only
#define INT_CFG_LNDPRT_MASK   0x10
#define INT_CFG_PULSE_MASK    0x08     // MMA8652 only
#define INT_CFG_FF_MT_MASK    0x04
#define INT_CFG_DRDY_MASK     0x01


#define SRC_ASLP_MASK         0x80
#define SRC_FIFO_MASK         0x40    // MMA8652 only
#define SRC_TRANS_MASK        0x20
#define SRC_LNDPRT_MASK       0x10
#define SRC_PULSE_MASK        0x08
#define SRC_FF_MT_MASK        0x04
#define SRC_DRDY_MASK         0x01


#define ST_MASK               0x80
#define RST_MASK              0x40
#define SMODS1_MASK           0x10
#define SMODS0_MASK           0x08
#define SLPE_MASK             0x04
#define MODS1_MASK            0x02
#define MODS0_MASK            0x01
#define SMODS_MASK            0x18
#define MODS_MASK             0x03

#endif