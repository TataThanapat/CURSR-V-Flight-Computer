/**
 ******************************************************************************
 * @file    stm32f4xx_hal_msp.c
 * @brief   MSP (MCU Support Package) init for SPI1 and I2C1.
 *          The HAL invokes these from inside HAL_xxx_Init().
 *
 *          Pin map (STM32F411CEU6):
 *            SPI1  : PA5 SCK, PA6 MISO, PA7 MOSI      (AF5)
 *            I2C1  : PB6 SCL, PB7 SDA                 (AF4, open-drain)
 *
 *          These functions MUST live here and NOT in main.c, otherwise the
 *          linker reports "multiple definition" (both are strong symbols
 *          overriding the HAL's __weak defaults).
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"

/* ---------------------------------------------------------------------------
 * Global HAL MSP init (called once by HAL_Init()).
 * ------------------------------------------------------------------------- */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

/* ---------------------------------------------------------------------------
 * SPI1 -> ICM-45686.  PA5/PA6/PA7, AF5, push-pull, very-high speed.
 * ------------------------------------------------------------------------- */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hspi->Instance == SPI1)
    {
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        __HAL_RCC_SPI1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    }
}

/* ---------------------------------------------------------------------------
 * I2C1 -> MMC5983MA.  PB6/PB7, AF4, OPEN-DRAIN.
 * External 2.2k-4.7k pull-ups to 3V3 are required for reliable 400 kHz;
 * GPIO_PULLUP here is only a weak fallback.
 * ------------------------------------------------------------------------- */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();

        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        __HAL_RCC_I2C1_CLK_ENABLE();
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_I2C1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    }
}
