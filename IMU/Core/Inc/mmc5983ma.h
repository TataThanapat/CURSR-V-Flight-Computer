/**
 ******************************************************************************
 * @file    mmc5983ma.h
 * @brief   Bare-metal HAL driver for MEMSIC MMC5983MA 3-axis magnetometer.
 *          Interface: I2C (7-bit addr 0x30, Fast Mode 400 kHz).
 *
 * @note    Output is 18-bit unsigned, centered on a null-field offset of
 *          131072 (2^17). Values are de-biased into a signed int32_t range.
 ******************************************************************************
 */

#ifndef MMC5983MA_H
#define MMC5983MA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"   /* Adjust to your target MCU family header. */
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Device address. HAL expects the 7-bit address left-shifted by 1.
 * ------------------------------------------------------------------------- */
#define MMC5983MA_I2C_ADDR_7BIT   0x30U
#define MMC5983MA_I2C_ADDR        (MMC5983MA_I2C_ADDR_7BIT << 1)

/* ---------------------------------------------------------------------------
 * Register Map
 * ------------------------------------------------------------------------- */
#define MMC5983MA_REG_XOUT0        0x00U  /* First byte of the 7-byte burst. */
#define MMC5983MA_REG_XYZOUT2      0x06U  /* Holds the 2 LSBs per axis.      */
#define MMC5983MA_REG_INT_CTRL0    0x09U
#define MMC5983MA_REG_INT_CTRL1    0x0AU
#define MMC5983MA_REG_INT_CTRL2    0x0BU
#define MMC5983MA_REG_PRODUCT_ID   0x2FU

/* ---------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */
#define MMC5983MA_PRODUCT_ID_VALUE 0x30U

/* Internal Control 0 (0x09) SET / RESET coil pulses. */
#define MMC5983MA_CTRL0_SET        0x08U
#define MMC5983MA_CTRL0_RESET      0x10U

/* Internal Control 1 (0x0A). */
#define MMC5983MA_CTRL1_SW_RESET   0x80U  /* Software reset.                  */
#define MMC5983MA_CTRL1_BW_100HZ   0x00U  /* 18-bit / 100 Hz measurement BW.  */

/* Internal Control 2 (0x0B): Cmm_en(bit3)=1 + Cmm_freq=100Hz(0b010).        */
#define MMC5983MA_CTRL2_CMM_100HZ  0x82U

/* 18-bit null-field midpoint (2^17). */
#define MMC5983MA_NULL_FIELD       131072

#define MMC5983MA_I2C_TIMEOUT      100U

/* ---------------------------------------------------------------------------
 * Device handle / object
 * ------------------------------------------------------------------------- */
typedef struct
{
    I2C_HandleTypeDef *hi2c;   /* Bound I2C peripheral handle. */

    /* De-biased, signed magnetic field samples (raw counts). */
    int32_t mag_x;
    int32_t mag_y;
    int32_t mag_z;
} MMC5983MA_Handle;

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

/**
 * @brief  Initialize the MMC5983MA: verify Product ID, software reset,
 *         SET/RESET degauss, then enable 100 Hz continuous measurement.
 * @param  dev   Pointer to a caller-allocated handle.
 * @param  hi2c  Configured I2C handle.
 * @retval 1 on success, 0 on failure (ID mismatch or bus error).
 */
uint8_t MMC5983MA_Init(MMC5983MA_Handle *dev, I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read the 7-byte burst, reconstruct 18-bit fields, de-bias to
 *         signed int32_t, and store in the handle.
 * @param  dev  Initialized handle.
 * @retval 1 on success, 0 on bus error.
 */
uint8_t MMC5983MA_ReadData(MMC5983MA_Handle *dev);

#ifdef __cplusplus
}
#endif

#endif /* MMC5983MA_H */
