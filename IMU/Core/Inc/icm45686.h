/**
 ******************************************************************************
 * @file    icm45686.h
 * @brief   Bare-metal HAL driver for TDK InvenSense ICM-45686 6-axis IMU.
 *          Interface: SPI (Mode 3 / Mode 0, MSB first, <= 24 MHz).
 *          Chip-select handled manually via GPIO.
 *
 * @note    Sensor data is forced to Big-Endian output (SREG_CTRL) so that
 *          burst reads can be reconstructed as (HIGH << 8) | LOW.
 ******************************************************************************
 */

#ifndef ICM45686_H
#define ICM45686_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"   /* Adjust to your target MCU family header. */
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Register Map (User Bank 0)
 * ------------------------------------------------------------------------- */
#define ICM45686_REG_ACCEL_DATA_X1   0x00U  /* First byte of the 12-byte burst */
#define ICM45686_REG_PWR_MGMT0       0x10U
#define ICM45686_REG_ACCEL_CONFIG0   0x1AU
#define ICM45686_REG_GYRO_CONFIG0    0x1CU
#define ICM45686_REG_SREG_CTRL       0x67U
#define ICM45686_REG_WHO_AM_I        0x72U

/* ---------------------------------------------------------------------------
 * Constants
 * ------------------------------------------------------------------------- */
#define ICM45686_WHO_AM_I_VALUE      0xE9U

/* SPI R/W flag: bit 7 of the address byte. */
#define ICM45686_SPI_READ_FLAG       0x80U  /* OR into address for reads.  */
#define ICM45686_SPI_WRITE_MASK      0x7FU  /* AND into address for writes.*/

/* SREG_CTRL (0x67): bit 1 = SREG_DATA_ENDIAN_SEL (1 = Big Endian). */
#define ICM45686_SREG_BIG_ENDIAN     0x02U

/* PWR_MGMT0 (0x10): 0x0F -> Accel + Gyro both in Low-Noise mode. */
#define ICM45686_PWR_LOW_NOISE_ALL   0x0FU

/*
 * ACCEL_CONFIG0 (0x1A) / GYRO_CONFIG0 (0x1C):
 *   bits[7:4] = Full-Scale Select, bits[3:0] = ODR select.
 *   Accel FS=0x0 -> +/-16 g (widest range), ODR=0x07 -> ~100 Hz.
 *   Gyro  FS=0x0 -> +/-2000 dps (widest range), ODR=0x07 -> ~100 Hz.
 */
#define ICM45686_ACCEL_CFG_16G_100HZ 0x07U
#define ICM45686_GYRO_CFG_2000_100HZ 0x07U

/* Generic HAL transaction timeout (ms). */
#define ICM45686_SPI_TIMEOUT         100U

/* ---------------------------------------------------------------------------
 * Device handle / object
 * ------------------------------------------------------------------------- */
typedef struct
{
    SPI_HandleTypeDef *hspi;     /* Bound SPI peripheral handle.            */
    GPIO_TypeDef      *cs_port;  /* GPIO port driving the chip-select line. */
    uint16_t           cs_pin;   /* GPIO pin mask for the chip-select line. */

    /* Raw, unscaled sensor samples (sensor frame, signed 16-bit). */
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} ICM45686_Handle;

/* ---------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

/**
 * @brief  Initialize the ICM-45686: verify WHO_AM_I, set Big-Endian output,
 *         power up both sensors in Low-Noise mode, and configure FS/ODR.
 * @param  dev      Pointer to a caller-allocated handle.
 * @param  hspi     Configured SPI handle.
 * @param  cs_port  Chip-select GPIO port.
 * @param  cs_pin   Chip-select GPIO pin.
 * @retval 1 on success, 0 on failure (WHO_AM_I mismatch or bus error).
 */
uint8_t ICM45686_Init(ICM45686_Handle *dev,
                      SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *cs_port,
                      uint16_t cs_pin);

/**
 * @brief  Read the 12-byte accel+gyro burst and populate the handle's
 *         raw int16_t fields (Big-Endian reconstruction).
 * @param  dev  Initialized handle.
 * @retval 1 on success, 0 on bus error.
 */
uint8_t ICM45686_ReadSensors(ICM45686_Handle *dev);

/* Low-level register helpers (exposed for diagnostics/test code). */
uint8_t ICM45686_ReadReg(ICM45686_Handle *dev, uint8_t reg, uint8_t *value);
uint8_t ICM45686_WriteReg(ICM45686_Handle *dev, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_H */
