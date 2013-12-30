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
#define ACCEL_CTRL_REG1    0x2A

/**
 * Expected value for ACCEL_WHO_AM_I register
 */
#define ACCEL_DEVICE_ID         0x1A

/* 
 * Bit-masks for ACCEL_CTRL_REG1 register fields
 */
#define ACCEL_CTRL_REG1_ACTIVE_MASK BIT(0)

/* 
 * Bit-masks for ACCEL_STATUS register fields
 */
#define ACCEL_STATUS_ZYXDR_MASK BIT(3)
#define ACCEL_STATUS_ZDR_MASK   BIT(2)
#define ACCEL_STATUS_YDR_MASK   BIT(1)
#define ACCEL_STATUS_XDR_MASK   BIT(0)

#endif /* __MMA8451Q_ACCELEROMETER_H */
