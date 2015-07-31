/**
 * @file fxos8700cq_accelerometer.h
 *
 * Freescale FXOS8700CQ accelerometer declarations
 *
 * @author German Rivera
 */
#ifndef __FXOS8700CQ_ACCELEROMETER_H
#define __FXOS8700CQ_ACCELEROMETER_H

#include <stdint.h>
#include <McRTOS/utils.h>

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
#define ACCEL_INT_SOURCE    0x0C
#define ACCEL_WHO_AM_I      0x0D
#define ACCEL_XYZ_DATA_CFG  0x0E
#define ACCEL_FF_MT_CFG     0x15
#define ACCEL_FF_MT_SRC     0x16
#define ACCEL_FF_MT_THS     0x17
#define ACCEL_FF_MT_COUNT   0x18
#define ACCEL_ASLP_COUNT    0x29 /* Auto Sleep Inactivity Timer register */
#define ACCEL_CTRL_REG1     0x2A
#define ACCEL_CTRL_REG2     0x2B
#define ACCEL_CTRL_REG3     0x2C
#define ACCEL_CTRL_REG4     0x2D
#define ACCEL_CTRL_REG5     0x2E

/*
 * Magnetometer Registers:
 */
#define MAGNET_OUT_X_MSB    0x33
#define MAGNET_OUT_Y_MSB    0x35
#define MAGNET_OUT_Z_MSB    0x37
#define MAGNET_CTRL_REG1    0x5B
#define MAGNET_CTRL_REG2    0x5C

/**
 * Expected value for ACCEL_WHO_AM_I register
 */
#define ACCEL_DEVICE_ID         0xC7

/*
 * Bit-masks for ACCEL_CTRL_REG1 register fields
 */
#define ACCEL_CTRL_REG1_ASLP_RATE_MASK	MULTI_BIT_MASK(7, 6)
#define ACCEL_CTRL_REG1_ASLP_RATE_SHIFT	6
#define ACCEL_CTRL_REG1_DR_MASK		MULTI_BIT_MASK(5, 3)
#define ACCEL_CTRL_REG1_DR_SHIFT	3
#define ACCEL_CTRL_REG1_LNOISE_MASK	BIT(2)
#define ACCEL_CTRL_REG1_FREAD_MASK	BIT(1)
#define ACCEL_CTRL_REG1_ACTIVE_MASK	BIT(0)

/*
 * Values for ACCEL_CTRL_REG1_ASLP_RATE field:
 */
#define ACCEL_CTRL_REG1_ASLP_RATE_20MS  0x00
#define ACCEL_CTRL_REG1_ASLP_RATE_80MS	0x01
#define ACCEL_CTRL_REG1_ASLP_RATE_160MS	0x02
#define ACCEL_CTRL_REG1_ASLP_RATE_640MS	0x03

/*
 * Values for ACCEL_CTRL_REG1_DR field:
 */
#define ACCEL_CTRL_REG1_DR_800HZ    0x0 /* every 1.25 ms */
#define ACCEL_CTRL_REG1_DR_400HZ    0x1 /* every 2.5 ms */
#define ACCEL_CTRL_REG1_DR_200HZ    0x2 /* every 5 ms */
#define ACCEL_CTRL_REG1_DR_100HZ    0x3 /* every 10 ms */
#define ACCEL_CTRL_REG1_DR_50HZ	    0x4 /* every 20 ms */
#define ACCEL_CTRL_REG1_DR_12_5HZ   0x5 /* every 80 ms */
#define ACCEL_CTRL_REG1_DR_6_25HZ   0x6 /* every 160 ms */
#define ACCEL_CTRL_REG1_DR_1_56HZ   0x7 /* every 640 ms */

/*
 * Bit-masks for ACCEL_CTRL_REG2 register fields
 */
#define ACCEL_CTRL_REG2_ST_MASK	    BIT(7)
#define ACCEL_CTRL_REG2_RST_MASK    BIT(6)
#define ACCEL_CTRL_REG2_SMODS_MASK  MULTI_BIT_MASK(4, 3)
#define ACCEL_CTRL_REG2_SMODS_SHIFT 3
#define ACCEL_CTRL_REG2_SLPE_MASK   BIT(2)
#define ACCEL_CTRL_REG2_MODS_MASK   MULTI_BIT_MASK(1, 0)
#define ACCEL_CTRL_REG2_MODS_SHIFT  0

/*
 * Values for ACCEL_CTRL_REG2_SMODS field:
 */
#define ACCEL_CTRL_REG2_SMOD_NORMAL           0x0
#define ACCEL_CTRL_REG2_SMOD_LOW_NOISE        0x1
#define ACCEL_CTRL_REG2_SMOD_HIGH_RES         0x2
#define ACCEL_CTRL_REG2_SMOD_LOW_POWER        0x3

#define ACCEL_CTRL_REG2_MOD_NORMAL            0x0
#define ACCEL_CTRL_REG2_MOD_LOW_NOISE         0x1
#define ACCEL_CTRL_REG2_MOD_HIGH_RES          0x2
#define ACCEL_CTRL_REG2_MOD_LOW_POWER         0x3

/*
 * Bit-masks for ACCEL_CTRL_REG3 register fields
 */
#define ACCEL_CTRL_REG3_WAKE_TRANS_MASK        BIT(6)
#define ACCEL_CTRL_REG3_WAKE_LNDPRT_MASK       BIT(5)
#define ACCEL_CTRL_REG3_WAKE_PULSE_MASK        BIT(4)
#define ACCEL_CTRL_REG3_WAKE_FF_MT_MASK        BIT(3)
#define ACCEL_CTRL_REG3_IPOL_MASK              BIT(1)
#define ACCEL_CTRL_REG3_PP_OD_MASK             BIT(0)

/*
 * Bit-masks for ACCEL_CTRL_REG4 register fields
 */
#define ACCEL_CTRL_REG4_INT_EN_ASLP_MASK    BIT(7)
#define ACCEL_CTRL_REG4_INT_EN_TRANS_MASK   BIT(5)
#define ACCEL_CTRL_REG4_INT_EN_LNDPRT_MASK  BIT(4)
#define ACCEL_CTRL_REG4_INT_EN_PULSE_MASK   BIT(3)
#define ACCEL_CTRL_REG4_INT_EN_FF_MT_MASK   BIT(2)
#define ACCEL_CTRL_REG4_INT_EN_DRDY_MASK    BIT(0)

/*
 * Bit-masks for ACCEL_CTRL_REG5 register fields
 */
#define ACCEL_CTRL_REG5_INT_CFG_ASLP_MASK      BIT(7)
#define ACCEL_CTRL_REG5_INT_CFG_FIFO_MASK      BIT(6)
#define ACCEL_CTRL_REG5_INT_CFG_TRANS_MASK     BIT(5)
#define ACCEL_CTRL_REG5_INT_CFG_LNDPRT_MASK    BIT(4)
#define ACCEL_CTRL_REG5_INT_CFG_PULSE_MASK     BIT(3)
#define ACCEL_CTRL_REG5_INT_CFG_FF_MT_MASK     BIT(2)
#define ACCEL_CTRL_REG5_INT_CFG_DRDY_MASK      BIT(0)

/*
 * Bit-masks for ACCEL_F_SETUP register fields
 */
#define ACCEL_F_SETUP_MODE_MASK     MULTI_BIT_MASK(7, 6)
#define ACCEL_F_SETUP_MODE_SHIFT    6
#define ACCEL_F_SETUP_WMRK_MASK     MULTI_BIT_MASK(5, 0)
#define ACCEL_F_SETUP_WMRK_SHIFT    0

/*
 * Values for ACCEL_F_SETUP_MODE field:
 */
#define ACCEL_F_SETUP_MODE_DISABLED 0x0
#define ACCEL_F_SETUP_MODE_CIRCULAR 0x1
#define ACCEL_F_SETUP_MODE_FILL	    0x2
#define ACCEL_F_SETUP_MODE_TRIGGER  0x3

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
#define ACCEL_XYZ_DATA_CFG_HPF_OUT_MASK BIT(4)
#define ACCEL_XYZ_DATA_CFG_FS_MASK      MULTI_BIT_MASK(1, 0)
#define ACCEL_XYZ_DATA_CFG_FS_SHIFT     0

/*
 * Values for the ACCEL_XYZ_DATA_CFG_FS field:
 */
#define ACCEL_XYZ_DATA_CFG_FS_2G  0x0 /* each count corresponds to 1g/4096 = 0.25mg */
#define ACCEL_XYZ_DATA_CFG_FS_4G  0x1 /* each count corresponds to 1g/2048 ??? */
#define ACCEL_XYZ_DATA_CFG_FS_8G  0x2 /* each count corresponds to 1g/1024 = 0.98mg */

/*
 * Bit-masks for ACCEL_FF_MT_CFG register fields
 */
#define ACCEL_FF_MT_CFG_ELE_MASK     BIT(7)
#define ACCEL_FF_MT_CFG_OAE_MASK     BIT(6)
#define ACCEL_FF_MT_CFG_ZEFE_MASK    BIT(5)
#define ACCEL_FF_MT_CFG_YEFE_MASK    BIT(4)
#define ACCEL_FF_MT_CFG_XEFE_MASK    BIT(3)

/*
 * Bit-masks for ACCEL_FF_MT_THS register fields
 */
#define ACCEL_FF_MT_THS_DBCNTM_MASK BIT(7)
#define ACCEL_FF_MT_THS_MASK	    MULTI_BIT_MASK(6, 0)
#define ACCEL_FF_MT_THS_SHIFT	    0

/*
 * Bit-masks for ACCEL_FF_MT_SRC register fields
 */
#define ACCEL_FF_MT_SRC_EA_MASK  BIT(7)
#define ACCEL_FF_MT_SRC_ZHE_MASK BIT(5)
#define ACCEL_FF_MT_SRC_ZHP_MASK BIT(4)
#define ACCEL_FF_MT_SRC_YHE_MASK BIT(3)
#define ACCEL_FF_MT_SRC_YHP_MASK BIT(2)
#define ACCEL_FF_MT_SRC_XHE_MASK BIT(1)
#define ACCEL_FF_MT_SRC_XHP_MASK BIT(0)

/*
 * Bit-masks for MAGNET_CTRL_REG1 register fields
 */
#define MAGNET_CTRL_REG1_OSR_MASK    MULTI_BIT_MASK(4, 2)
#define MAGNET_CTRL_REG1_OSR_SHIFT   2
#define MAGNET_CTRL_REG1_HMS_MASK    MULTI_BIT_MASK(1, 0)
#define MAGNET_CTRL_REG1_HMS_SHIFT   0

/*
 * Bit-masks for MAGNET_CTRL_REG2 register fields
 */
#define MAGNET_CTRL_REG2_HYB_AUTOINC_MASK	BIT(5)
#define MAGNET_CTRL_REG2_MAXMIN_DIS_MASK	BIT(4)
#define MAGNET_CTRL_REG2_MAXMIN_DIS_THS_MASK	BIT(3)
#define MAGNET_CTRL_REG2_MAXMIN_RST_MASK	BIT(2)
#define MAGNET_CTRL_REG2_RST_CNT_MASK		MULTI_BIT_MASK(1, 0)
#define MAGNET_CTRL_REG2_RST_CNT_SHIFT		0

#endif /* __FXOS8700CQ_ACCELEROMETER_H */
