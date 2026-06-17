/**
 ******************************************************************************
 * @file    mmc5983ma.c
 * @brief   Implementation of the bare-metal HAL driver for the MMC5983MA.
 ******************************************************************************
 */

#include "mmc5983ma.h"

/* ---------------------------------------------------------------------------
 * Register write helper (single byte to a register).
 * ------------------------------------------------------------------------- */
static uint8_t mmc_write_reg(MMC5983MA_Handle *dev, uint8_t reg, uint8_t val)
{
    HAL_StatusTypeDef st;

    st = HAL_I2C_Mem_Write(dev->hi2c, MMC5983MA_I2C_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT, &val, 1U,
                           MMC5983MA_I2C_TIMEOUT);

    return (st == HAL_OK) ? 1U : 0U;
}

/* ---------------------------------------------------------------------------
 * Register read helper (single byte from a register).
 * ------------------------------------------------------------------------- */
static uint8_t mmc_read_reg(MMC5983MA_Handle *dev, uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef st;

    st = HAL_I2C_Mem_Read(dev->hi2c, MMC5983MA_I2C_ADDR, reg,
                          I2C_MEMADD_SIZE_8BIT, val, 1U,
                          MMC5983MA_I2C_TIMEOUT);

    return (st == HAL_OK) ? 1U : 0U;
}

/* ---------------------------------------------------------------------------
 * Public: initialization.
 * ------------------------------------------------------------------------- */
uint8_t MMC5983MA_Init(MMC5983MA_Handle *dev, I2C_HandleTypeDef *hi2c)
{
    uint8_t pid = 0x00U;

    if ((dev == NULL) || (hi2c == NULL))
    {
        return 0U;
    }

    dev->hi2c  = hi2c;
    dev->mag_x = 0;
    dev->mag_y = 0;
    dev->mag_z = 0;

    /* 1) Identity check. */
    if (mmc_read_reg(dev, MMC5983MA_REG_PRODUCT_ID, &pid) == 0U)
    {
        return 0U;
    }
    if (pid != MMC5983MA_PRODUCT_ID_VALUE)
    {
        return 0U;
    }

    /* 2) Software reset, then allow the part to come back up. */
    if (mmc_write_reg(dev, MMC5983MA_REG_INT_CTRL1,
                      MMC5983MA_CTRL1_SW_RESET) == 0U)
    {
        return 0U;
    }
    HAL_Delay(20);

    /* 3) Degauss: SET pulse, settle, then RESET pulse, settle. */
    if (mmc_write_reg(dev, MMC5983MA_REG_INT_CTRL0,
                      MMC5983MA_CTRL0_SET) == 0U)
    {
        return 0U;
    }
    HAL_Delay(2);
    if (mmc_write_reg(dev, MMC5983MA_REG_INT_CTRL0,
                      MMC5983MA_CTRL0_RESET) == 0U)
    {
        return 0U;
    }
    HAL_Delay(2);

    /* 4a) Bandwidth / resolution (18-bit, 100 Hz measurement BW). */
    if (mmc_write_reg(dev, MMC5983MA_REG_INT_CTRL1,
                      MMC5983MA_CTRL1_BW_100HZ) == 0U)
    {
        return 0U;
    }

    /* 4b) Enable continuous-measurement mode at 100 Hz ODR. */
    if (mmc_write_reg(dev, MMC5983MA_REG_INT_CTRL2,
                      MMC5983MA_CTRL2_CMM_100HZ) == 0U)
    {
        return 0U;
    }

    return 1U;
}

/* ---------------------------------------------------------------------------
 * Public: read + reconstruct 18-bit magnetic field.
 *
 * Burst layout (7 bytes from XOUT0 @ 0x00):
 *   [0] Xout[17:10]   (high 8 bits)
 *   [1] Xout[9:2]     (mid  8 bits)
 *   [2] Yout[17:10]
 *   [3] Yout[9:2]
 *   [4] Zout[17:10]
 *   [5] Zout[9:2]
 *   [6] XYZOUT_2: bits[7:6]=Xout[1:0], bits[5:4]=Yout[1:0], bits[3:2]=Zout[1:0]
 *
 * Reconstruction per axis:
 *   value = (HIGH << 10) | (MID << 2) | LSB2
 * then de-bias by subtracting the 131072 null-field midpoint.
 * ------------------------------------------------------------------------- */
uint8_t MMC5983MA_ReadData(MMC5983MA_Handle *dev)
{
    uint8_t  raw[7];
    uint32_t x_u, y_u, z_u;

    if (dev == NULL)
    {
        return 0U;
    }

    if (HAL_I2C_Mem_Read(dev->hi2c, MMC5983MA_I2C_ADDR,
                         MMC5983MA_REG_XOUT0, I2C_MEMADD_SIZE_8BIT,
                         raw, 7U, MMC5983MA_I2C_TIMEOUT) != HAL_OK)
    {
        return 0U;
    }

    /* Assemble 18-bit unsigned codes. The 2 LSBs live in raw[6]. */
    x_u = ((uint32_t)raw[0] << 10) | ((uint32_t)raw[1] << 2) |
          (((uint32_t)raw[6] >> 6) & 0x03U);
    y_u = ((uint32_t)raw[2] << 10) | ((uint32_t)raw[3] << 2) |
          (((uint32_t)raw[6] >> 4) & 0x03U);
    z_u = ((uint32_t)raw[4] << 10) | ((uint32_t)raw[5] << 2) |
          (((uint32_t)raw[6] >> 2) & 0x03U);

    /* De-bias around the null-field midpoint into a signed range. */
    dev->mag_x = (int32_t)x_u - MMC5983MA_NULL_FIELD;
    dev->mag_y = (int32_t)y_u - MMC5983MA_NULL_FIELD;
    dev->mag_z = (int32_t)z_u - MMC5983MA_NULL_FIELD;

    return 1U;
}
