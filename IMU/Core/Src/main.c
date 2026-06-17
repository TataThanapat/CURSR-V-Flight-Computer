/**
 ******************************************************************************
 * @file    main.c
 * @brief   Standalone sanity-check harness + bring-up init for the
 *          ICM-45686 (SPI1) and MMC5983MA (I2C1) on an STM32F411CEU6.
 *
 * Pin map (UFQFPN48 / "BlackPill"-style):
 *   SPI1_SCK  -> PA5  (AF5)
 *   SPI1_MISO -> PA6  (AF5)
 *   SPI1_MOSI -> PA7  (AF5)
 *   ICM_CS    -> PA4  (GPIO output, manual, push-pull, idle HIGH)
 *   I2C1_SCL  -> PB6  (AF4, open-drain, external pull-up)
 *   I2C1_SDA  -> PB7  (AF4, open-drain, external pull-up)
 *
 * Clock tree:
 *   HSE 25 MHz -> PLL (M=25, N=192, P=2) -> SYSCLK 96 MHz
 *   AHB  = 96 MHz, APB1 = 48 MHz, APB2 = 96 MHz
 *   SPI1 on APB2: 96 MHz / 8 = 12 MHz SCK (safe; <= 24 MHz max).
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"
#include "icm45686.h"
#include "mmc5983ma.h"
#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Peripheral handles (owned by this translation unit).
 * ------------------------------------------------------------------------- */
SPI_HandleTypeDef hspi1;
I2C_HandleTypeDef hi2c1;

/* ---------------------------------------------------------------------------
 * Sensor objects + debug flags at FILE SCOPE (global).
 * Globals live at a fixed address, so the debugger's Live Expressions view
 * can read them continuously while the target is free-running -- unlike
 * locals, which only resolve when the core is halted inside main().
 * ------------------------------------------------------------------------- */
ICM45686_Handle  imu = {0};
MMC5983MA_Handle mag = {0};

volatile uint8_t imu_ok    = 0U;   /* 1 = IMU enumerated at boot.        */
volatile uint8_t mag_ok    = 0U;   /* 1 = magnetometer enumerated.       */
volatile uint8_t imu_fault = 0U;   /* 1 = bad/suspect IMU read this pass.*/
volatile uint8_t mag_fault = 0U;
volatile uint8_t imu_stale = 0U;   /* 1 = value unchanged since last pass.*/
volatile uint8_t mag_stale = 0U;


/* ---------------------------------------------------------------------------
 * Board pin map.
 * ------------------------------------------------------------------------- */
#define ICM_CS_PORT     GPIOA
#define ICM_CS_PIN      GPIO_PIN_4

#define SPI1_SCK_PIN    GPIO_PIN_5
#define SPI1_MISO_PIN   GPIO_PIN_6
#define SPI1_MOSI_PIN   GPIO_PIN_7
#define SPI1_GPIO_PORT  GPIOA

#define I2C1_SCL_PIN    GPIO_PIN_6
#define I2C1_SDA_PIN    GPIO_PIN_7
#define I2C1_GPIO_PORT  GPIOB

/* Optional fault LED -- PC13 drives the on-board LED on most F411 modules
 * (active-low). Comment out if your PCB uses a different pin. */
#define FAULT_LED_PORT  GPIOC
#define FAULT_LED_PIN   GPIO_PIN_13

/* ---------------------------------------------------------------------------
 * Forward declarations.
 * ------------------------------------------------------------------------- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void Error_Trap(void);

/* ===========================================================================
 * Clock configuration: HSE 25 MHz -> 96 MHz SYSCLK.
 *
 * If your board uses an 8 MHz HSE, change PLLM to 8.
 * If you have no external crystal, switch to HSI (16 MHz): set
 *   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
 *   .HSIState = RCC_HSI_ON; PLLSource = RCC_PLLSOURCE_HSI; PLLM = 16;
 * ======================================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Voltage scaling for the target frequency. */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* HSE -> PLL: (25 / 25) * 192 / 2 = 96 MHz. */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM            = 25U;
    RCC_OscInitStruct.PLL.PLLN            = 192U;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 4U;   /* 96/4 = 48 MHz (USB/SDIO). */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Trap();
    }

    /* Bus clocks: AHB=96, APB1=48 (max 50), APB2=96 (max 100). */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Trap();
    }
}

/* ===========================================================================
 * GPIO init: clocks, CS line, fault LED. AF pins are configured inside the
 * respective peripheral MSP init below, but we enable the port clocks here.
 * ======================================================================== */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* ICM chip-select: push-pull output, idle HIGH (de-asserted). */
    HAL_GPIO_WritePin(ICM_CS_PORT, ICM_CS_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = ICM_CS_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(ICM_CS_PORT, &GPIO_InitStruct);

    /* Fault LED: push-pull output, start OFF (active-low -> drive HIGH). */
    HAL_GPIO_WritePin(FAULT_LED_PORT, FAULT_LED_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = FAULT_LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(FAULT_LED_PORT, &GPIO_InitStruct);
}

/* ===========================================================================
 * SPI1 init: Master, Mode 3 (CPOL=High, CPHA=2Edge), 8-bit, MSB first,
 * software NSS, prescaler /8 -> 12 MHz on a 96 MHz APB2.
 *
 * For Mode 0 instead: CLKPolarity = LOW, CLKPhase = 1EDGE.
 * To push to 24 MHz once stable: BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4.
 * ======================================================================== */
static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;   /* Mode 3 */
    hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;     /* Mode 3 */
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;  /* 12 MHz */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10U;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Trap();
    }
}

/* ===========================================================================
 * I2C1 init: Fast Mode 400 kHz, 7-bit addressing.
 * ======================================================================== */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000U;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0U;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0U;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Trap();
    }
}

/* NOTE: HAL_SPI_MspInit() and HAL_I2C_MspInit() are intentionally NOT defined
 * here. They live in the CubeMX-generated Core/Src/stm32f4xx_hal_msp.c to
 * avoid a "multiple definition" link error. See that file for the PA5/6/7
 * (SPI1, AF5) and PB6/7 (I2C1, AF4 open-drain) pin configuration. */

/* ---------------------------------------------------------------------------
 * Fault trap: halt and blink the fault LED so a failure is visible.
 * ------------------------------------------------------------------------- */
static void Error_Trap(void)
{
    while (1)
    {
        HAL_GPIO_TogglePin(FAULT_LED_PORT, FAULT_LED_PIN);
        HAL_Delay(100);  /* ~5 Hz blink => "something failed". */
    }
}

/* ---------------------------------------------------------------------------
 * Bus-fault heuristics (unchanged from the bring-up harness).
 * ------------------------------------------------------------------------- */
static uint8_t imu_data_is_suspect(const ICM45686_Handle *d)
{
    const int16_t v[6] = { d->accel_x, d->accel_y, d->accel_z,
                           d->gyro_x,  d->gyro_y,  d->gyro_z };
    uint8_t all_zero = 1U;
    uint8_t all_ones = 1U;
    int i;

    for (i = 0; i < 6; ++i)
    {
        if (v[i] != (int16_t)0x0000) { all_zero = 0U; }
        if (v[i] != (int16_t)0xFFFF) { all_ones = 0U; }
    }
    return (uint8_t)(all_zero || all_ones);
}

static uint8_t mag_data_is_suspect(const MMC5983MA_Handle *d)
{
    const int32_t stuck_low  = -MMC5983MA_NULL_FIELD;
    const int32_t stuck_high =  (MMC5983MA_NULL_FIELD - 1);

    uint8_t all_low  = (d->mag_x == stuck_low)  &&
                       (d->mag_y == stuck_low)  &&
                       (d->mag_z == stuck_low);
    uint8_t all_high = (d->mag_x == stuck_high) &&
                       (d->mag_y == stuck_high) &&
                       (d->mag_z == stuck_high);

    return (uint8_t)(all_low || all_high);
}

/* ===========================================================================
 * main
 * ======================================================================== */
int main(void)
{
    /* imu, mag, imu_ok, mag_ok, imu_fault/mag_fault, imu_stale/mag_stale are
     * now FILE-SCOPE globals (see top of file) for live debugger visibility.
     * Only the genuinely loop-local bookkeeping stays here. */
    int16_t prev_accel_x = 0;
    int32_t prev_mag_x   = 0;
    uint8_t first_pass   = 1U;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_I2C1_Init();

    /* --- Sensor bring-up. Failures are recorded but NON-fatal: we keep
     * running so any working sensor still streams, and the debugger can
     * observe which one is down via imu_ok / mag_ok. --- */
    imu_ok = ICM45686_Init(&imu, &hspi1, ICM_CS_PORT, ICM_CS_PIN);
    mag_ok = MMC5983MA_Init(&mag, &hi2c1);

    /* --- Primary acquisition loop. Runs regardless of init outcome. --- */
    while (1)
    {
        /* Attempt a read every pass. If init failed earlier, the read will
         * very likely fail too -> flagged. If the sensor recovers (e.g. you
         * reseat a wire), it can start reporting good data without a reset. */
        if (ICM45686_ReadSensors(&imu) == 0U)
        {
            imu_fault = 1U;
        }
        else
        {
            imu_fault = imu_data_is_suspect(&imu);
        }

        if (MMC5983MA_ReadData(&mag) == 0U)
        {
            mag_fault = 1U;
        }
        else
        {
            mag_fault = mag_data_is_suspect(&mag);
        }

        if (first_pass == 0U)
        {
            imu_stale = (imu.accel_x == prev_accel_x) ? 1U : 0U;
            mag_stale = (mag.mag_x   == prev_mag_x)   ? 1U : 0U;
        }
        first_pass   = 0U;
        prev_accel_x = imu.accel_x;
        prev_mag_x   = mag.mag_x;

        /* LED on (active-low -> RESET) if anything looks wrong this pass. */
        if (imu_fault || mag_fault || imu_stale || mag_stale ||
            (imu_ok == 0U) || (mag_ok == 0U))
        {
            HAL_GPIO_WritePin(FAULT_LED_PORT, FAULT_LED_PIN, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(FAULT_LED_PORT, FAULT_LED_PIN, GPIO_PIN_SET);
        }

        /* Latest raw samples live in the imu and mag struct fields
         * (accel/gyro and mag x/y/z). Watch them via Live Expressions or
         * the Variables view while the target is halted. imu_ok / mag_ok
         * show enumeration status. */

        HAL_Delay(10);  /* ~100 Hz polling cadence to match the ODR. */
    }
}
