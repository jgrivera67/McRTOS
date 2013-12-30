/**
 * @file mma8451q_accelerometer.h
 *
 * Freescale MMA8451Q accelerometer declarations
 *
 * @author German Rivera 
 */ 
#ifndef __MMA8451Q_ACCELEROMETER_H
#define __MMA8451Q_ACCELEROMETER_H

#include <stdint.h>
#include "utils.h"

/*
 * I2C slave address for the accelerometer
 */
#define ACCELEROMETER_I2C_ADDR  UINT8_C(0x1d)

/*
 * Accelerometer registers
 */
#define ACCEL_STATUS        0x00
#define ACCEL_OUT_X_MSB     0x01
#define ACCEL_OUT_Y_MSB     0x03
#define ACCEL_OUT_Z_MSB     0x05
#define ACCEL_F_SETUP       0x09
#define ACCEL_WHO_AM_I      0x0D
#define ACCEL_XYZ_DATA_CFG  0x0E
#define ACCEL_CTRL_REG1     0x2A

/**
 * Expected value for ACCEL_WHO_AM_I register
 */
#define ACCEL_DEVICE_ID         0x1A

/* 
 * Bit-masks for ACCEL_CTRL_REG1 register fields
 */
#define ACCEL_CTRL_REG1_ACTIVE_MASK BIT(0)
#define ACCEL_CTRL_REG1_DR_MASK     MULTI_BIT_MASK(5, 3)
#define ACCEL_CTRL_REG1_DR_SHIFT    3
#define ACCEL_CTRL_REG1_DR_VALUE_800HZ  0x0 /* every 1.25 ms */
#define ACCEL_CTRL_REG1_DR_VALUE_400HZ  0x1
#define ACCEL_CTRL_REG1_DR_VALUE_200HZ  0x2
#define ACCEL_CTRL_REG1_DR_VALUE_100HZ  0x3
#define ACCEL_CTRL_REG1_DR_VALUE_50HZ   0x4
#define ACCEL_CTRL_REG1_DR_VALUE_12_5HZ 0x5
#define ACCEL_CTRL_REG1_DR_VALUE_6_25HZ 0x6
#define ACCEL_CTRL_REG1_DR_VALUE_1_56HZ 0x7 /* every 640 ms */

/* 
 * Bit-masks for ACCEL_STATUS register fields
 */
#define ACCEL_STATUS_ZYXDR_MASK BIT(3)
#define ACCEL_STATUS_ZDR_MASK   BIT(2)
#define ACCEL_STATUS_YDR_MASK   BIT(1)
#define ACCEL_STATUS_XDR_MASK   BIT(0)

/* 
 * Bit-masks for ACCEL_XYZ_DATA_CFG register fields
 */
#define ACCEL_XYZ_DATA_CFG_FS_MASK      MULTI_BIT_MASK(1, 0)
#define ACCEL_XYZ_DATA_CFG_FS_SHIFT     0
#define ACCEL_XYZ_DATA_CFG_FS_VALUE_2G  0x0
#define ACCEL_XYZ_DATA_CFG_FS_VALUE_4G  0x1
#define ACCEL_XYZ_DATA_CFG_FS_VALUE_8G  0x2

#define ACCEL_XYZ_DATA_CFG_HPF_OUT_MASK BIT(4)

#endif /* __MMA8451Q_ACCELEROMETER_H */
