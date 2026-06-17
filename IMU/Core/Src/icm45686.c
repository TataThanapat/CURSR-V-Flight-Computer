/**
 ******************************************************************************
 * @file    icm45686.c
 * @brief   Implementation of the bare-metal HAL driver for the ICM-45686.
 ******************************************************************************
 */

#include "icm45686.h"

/* ---------------------------------------------------------------------------
 * Chip-select helpers (active-low). Inlined to keep the hot path tight.
 * ------------------------------------------------------------------------- */
static inline void icm_cs_low(ICM45686_Handle *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void icm_cs_high(ICM45686_Handle *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/* ---------------------------------------------------------------------------
 * Single-register write.
 *   Frame: [addr & 0x7F][data]  (bit 7 = 0 => write).
 * ------------------------------------------------------------------------- */
uint8_t ICM45686_WriteReg(ICM45686_Handle *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    HAL_StatusTypeDef st;

    tx[0] = (uint8_t)(reg & ICM45686_SPI_WRITE_MASK);
    tx[1] = value;

    icm_cs_low(dev);
    st = HAL_SPI_Transmit(dev->hspi, tx, 2U, ICM45686_SPI_TIMEOUT);
    icm_cs_high(dev);

    return (st == HAL_OK) ? 1U : 0U;
}

/* ---------------------------------------------------------------------------
 * Single-register read.
 *   Frame: [addr | 0x80] -> then clock out 1 byte (bit 7 = 1 => read).
 * ------------------------------------------------------------------------- */
uint8_t ICM45686_ReadReg(ICM45686_Handle *dev, uint8_t reg, uint8_t *value)
{
    uint8_t addr = (uint8_t)(reg | ICM45686_SPI_READ_FLAG);
    HAL_StatusTypeDef st;

    icm_cs_low(dev);
    st = HAL_SPI_Transmit(dev->hspi, &addr, 1U, ICM45686_SPI_TIMEOUT);
    if (st == HAL_OK)
    {
        st = HAL_SPI_Receive(dev->hspi, value, 1U, ICM45686_SPI_TIMEOUT);
    }
    icm_cs_high(dev);

    return (st == HAL_OK) ? 1U : 0U;
}

/* ---------------------------------------------------------------------------
 * Burst read: send address (with read flag), then stream `len` bytes.
 * ------------------------------------------------------------------------- */
static uint8_t icm_read_burst(ICM45686_Handle *dev, uint8_t reg,
                              uint8_t *buf, uint16_t len)
{
    uint8_t addr = (uint8_t)(reg | ICM45686_SPI_READ_FLAG);
    HAL_StatusTypeDef st;

    icm_cs_low(dev);
    st = HAL_SPI_Transmit(dev->hspi, &addr, 1U, ICM45686_SPI_TIMEOUT);
    if (st == HAL_OK)
    {
        st = HAL_SPI_Receive(dev->hspi, buf, len, ICM45686_SPI_TIMEOUT);
    }
    icm_cs_high(dev);

    return (st == HAL_OK) ? 1U : 0U;
}

/* ---------------------------------------------------------------------------
 * Public: initialization.
 * ------------------------------------------------------------------------- */
uint8_t ICM45686_Init(ICM45686_Handle *dev,
                      SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port,
                      uint16_t cs_pin)
{
    uint8_t who = 0x00U;

    if ((dev == NULL) || (hspi == NULL) || (cs_port == NULL))
    {
        return 0U;
    }

    /* Bind hardware resources. */
    dev->hspi    = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin  = cs_pin;

    dev->accel_x = 0; dev->accel_y = 0; dev->accel_z = 0;
    dev->gyro_x  = 0; dev->gyro_y  = 0; dev->gyro_z  = 0;

    /* Idle the bus: CS de-asserted before first transaction. */
    icm_cs_high(dev);

    /* 1) Identity check. */
    if (ICM45686_ReadReg(dev, ICM45686_REG_WHO_AM_I, &who) == 0U)
    {
        return 0U;
    }
    if (who != ICM45686_WHO_AM_I_VALUE)
    {
        return 0U;
    }

    /* 2) Force Big-Endian sensor data layout (SREG_CTRL bit 1). */
    if (ICM45686_WriteReg(dev, ICM45686_REG_SREG_CTRL,
                          ICM45686_SREG_BIG_ENDIAN) == 0U)
    {
        return 0U;
    }

    /* 3) Power management: Accel + Gyro Low-Noise mode. */
    if (ICM45686_WriteReg(dev, ICM45686_REG_PWR_MGMT0,
                          ICM45686_PWR_LOW_NOISE_ALL) == 0U)
    {
        return 0U;
    }
    /* Datasheet requires a short settling delay after exiting OFF. */
    HAL_Delay(1);

    /* 4) Full-scale + ODR configuration. */
    if (ICM45686_WriteReg(dev, ICM45686_REG_ACCEL_CONFIG0,
                          ICM45686_ACCEL_CFG_16G_100HZ) == 0U)
    {
        return 0U;
    }
    if (ICM45686_WriteReg(dev, ICM45686_REG_GYRO_CONFIG0,
                          ICM45686_GYRO_CFG_2000_100HZ) == 0U)
    {
        return 0U;
    }

    return 1U;
}

/* ---------------------------------------------------------------------------
 * Public: read accel + gyro burst.
 *
 * Burst layout (Big-Endian, 12 bytes from ACCEL_DATA_X1 @ 0x00):
 *   [0]=AX_H [1]=AX_L [2]=AY_H [3]=AY_L [4]=AZ_H [5]=AZ_L
 *   [6]=GX_H [7]=GX_L [8]=GY_H [9]=GY_L [10]=GZ_H [11]=GZ_L
 * ------------------------------------------------------------------------- */
uint8_t ICM45686_ReadSensors(ICM45686_Handle *dev)
{
    uint8_t raw[12];

    if (dev == NULL)
    {
        return 0U;
    }

    if (icm_read_burst(dev, ICM45686_REG_ACCEL_DATA_X1, raw, 12U) == 0U)
    {
        return 0U;
    }

    dev->accel_x = (int16_t)(((uint16_t)raw[0]  << 8) | raw[1]);
    dev->accel_y = (int16_t)(((uint16_t)raw[2]  << 8) | raw[3]);
    dev->accel_z = (int16_t)(((uint16_t)raw[4]  << 8) | raw[5]);
    dev->gyro_x  = (int16_t)(((uint16_t)raw[6]  << 8) | raw[7]);
    dev->gyro_y  = (int16_t)(((uint16_t)raw[8]  << 8) | raw[9]);
    dev->gyro_z  = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);

    return 1U;
}
