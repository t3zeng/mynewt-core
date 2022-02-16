/**
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */
#include <assert.h>

#include "os/mynewt.h"

#include <hal/hal_bsp.h>
#include <bsp/bsp.h>
#include <mcu/hal_apollo3.h>
#include "am_mcu_apollo.h"

#if MYNEWT_VAL(SPI_0_MASTER) || MYNEWT_VAL(SPI_1_MASTER) || MYNEWT_VAL(SPI_2_MASTER) || MYNEWT_VAL(SPI_3_MASTER) || MYNEWT_VAL(SPI_4_MASTER) || MYNEWT_VAL(SPI_5_MASTER)
#include <hal/hal_spi.h>
#endif

#if MYNEWT_VAL(I2C_0) || MYNEWT_VAL(I2C_1) || MYNEWT_VAL(I2C_2) || MYNEWT_VAL(I2C_3) || MYNEWT_VAL(I2C_4) || MYNEWT_VAL(I2C_5)
#include <hal/hal_i2c.h>
#endif

#if MYNEWT_VAL(UART_0) || MYNEWT_VAL(UART_1)
#include "uart/uart.h"
#include "uart_hal/uart_hal.h"
#endif

#if MYNEWT_VAL(ADC_0)
#include <adc/adc.h>
#include <adc_apollo3/adc_apollo3.h>
#endif

#if MYNEWT_VAL(UART_0)
static struct uart_dev os_bsp_uart0;
static const struct apollo3_uart_cfg os_bsp_uart0_cfg = {
    .suc_pin_tx = MYNEWT_VAL(UART_0_PIN_TX),
    .suc_pin_rx = MYNEWT_VAL(UART_0_PIN_RX),
    .suc_pin_rts = MYNEWT_VAL(UART_0_PIN_RTS),
    .suc_pin_cts = MYNEWT_VAL(UART_0_PIN_CTS),
};
#endif

#if MYNEWT_VAL(UART_1)
static struct uart_dev os_bsp_uart1;
static const struct apollo3_uart_cfg os_bsp_uart1_cfg = {
    .suc_pin_tx = MYNEWT_VAL(UART_1_PIN_TX),
    .suc_pin_rx = MYNEWT_VAL(UART_1_PIN_RX),
    .suc_pin_rts = MYNEWT_VAL(UART_1_PIN_RTS),
    .suc_pin_cts = MYNEWT_VAL(UART_1_PIN_CTS),
};
#endif

/*
 * What memory to include in coredump.
 */
static const struct hal_bsp_mem_dump dump_cfg[] = {
    [0] = {
        .hbmd_start = &_ram_start,
        .hbmd_size = RAM_SIZE
    }
};

/*
 * NOTE: Our HAL expects that the SS pin, if used, is treated as a gpio line
 * and is handled outside the SPI routines.
 */

#if MYNEWT_VAL(SPI_0_MASTER)
static const struct apollo3_spi_cfg hal_bsp_spi0m_cfg = {
    .sck_pin      = MYNEWT_VAL(SPI_0_MASTER_PIN_SCK),
    .mosi_pin     = MYNEWT_VAL(SPI_0_MASTER_PIN_MOSI),
    .miso_pin     = MYNEWT_VAL(SPI_0_MASTER_PIN_MISO),
    .ss_pin       = MYNEWT_VAL(SPI_0_PIN_CS),
};
#endif

#if MYNEWT_VAL(SPI_1_MASTER)
static const struct apollo3_spi_cfg hal_bsp_spi1m_cfg = {
    .sck_pin      = MYNEWT_VAL(SPI_1_MASTER_PIN_SCK),
    .mosi_pin     = MYNEWT_VAL(SPI_1_MASTER_PIN_MOSI),
    .miso_pin     = MYNEWT_VAL(SPI_1_MASTER_PIN_MISO),
    .ss_pin       = MYNEWT_VAL(SPI_1_PIN_CS),
};
#endif

#if MYNEWT_VAL(SPI_2_MASTER)
static const struct apollo3_spi_cfg hal_bsp_spi2m_cfg = {
    .sck_pin      = MYNEWT_VAL(SPI_2_MASTER_PIN_SCK),
    .mosi_pin     = MYNEWT_VAL(SPI_2_MASTER_PIN_MOSI),
    .miso_pin     = MYNEWT_VAL(SPI_2_MASTER_PIN_MISO),
    .ss_pin       = MYNEWT_VAL(SPI_2_PIN_CS),
};
#endif

#if MYNEWT_VAL(SPI_3_MASTER)
static const struct apollo3_spi_cfg hal_bsp_spi3m_cfg = {
    .sck_pin      = MYNEWT_VAL(SPI_3_MASTER_PIN_SCK),
    .mosi_pin     = MYNEWT_VAL(SPI_3_MASTER_PIN_MOSI),
    .miso_pin     = MYNEWT_VAL(SPI_3_MASTER_PIN_MISO),
    .ss_pin       = MYNEWT_VAL(SPI_3_PIN_CS),
};
#endif

#if MYNEWT_VAL(SPI_4_MASTER)
static const struct apollo3_spi_cfg hal_bsp_spi4m_cfg = {
    .sck_pin      = MYNEWT_VAL(SPI_4_MASTER_PIN_SCK),
    .mosi_pin     = MYNEWT_VAL(SPI_4_MASTER_PIN_MOSI),
    .miso_pin     = MYNEWT_VAL(SPI_4_MASTER_PIN_MISO),
    .ss_pin       = MYNEWT_VAL(SPI_4_PIN_CS),
};
#endif

#if MYNEWT_VAL(SPI_5_MASTER)
static const struct apollo3_spi_cfg hal_bsp_spi5m_cfg = {
    .sck_pin      = MYNEWT_VAL(SPI_5_MASTER_PIN_SCK),
    .mosi_pin     = MYNEWT_VAL(SPI_5_MASTER_PIN_MOSI),
    .miso_pin     = MYNEWT_VAL(SPI_5_MASTER_PIN_MISO),
    .ss_pin       = MYNEWT_VAL(SPI_5_PIN_CS),
};
#endif

#if MYNEWT_VAL(I2C_0)
static const struct apollo3_i2c_cfg hal_bsp_i2c0m_cfg = {
    .scl_pin      = MYNEWT_VAL(I2C_0_PIN_SCL),
    .sda_pin      = MYNEWT_VAL(I2C_0_PIN_SDA),
};
#endif

#if MYNEWT_VAL(I2C_1)
static const struct apollo3_i2c_cfg hal_bsp_i2c1m_cfg = {
    .scl_pin      = MYNEWT_VAL(I2C_1_PIN_SCL),
    .sda_pin      = MYNEWT_VAL(I2C_1_PIN_SDA),
};
#endif

#if MYNEWT_VAL(I2C_2)
static const struct apollo3_i2c_cfg hal_bsp_i2c2m_cfg = {
    .scl_pin      = MYNEWT_VAL(I2C_2_PIN_SCL),
    .sda_pin      = MYNEWT_VAL(I2C_2_PIN_SDA),
};
#endif

#if MYNEWT_VAL(I2C_3)
static const struct apollo3_i2c_cfg hal_bsp_i2c3m_cfg = {
    .scl_pin      = MYNEWT_VAL(I2C_3_PIN_SCL),
    .sda_pin      = MYNEWT_VAL(I2C_3_PIN_SDA),
};
#endif

#if MYNEWT_VAL(I2C_4)
static const struct apollo3_i2c_cfg hal_bsp_i2c4m_cfg = {
    .scl_pin      = MYNEWT_VAL(I2C_4_PIN_SCL),
    .sda_pin      = MYNEWT_VAL(I2C_4_PIN_SDA),
};
#endif

#if MYNEWT_VAL(I2C_5)
static const struct apollo3_i2c_cfg hal_bsp_i2c5m_cfg = {
    .scl_pin      = MYNEWT_VAL(I2C_5_PIN_SCL),
    .sda_pin      = MYNEWT_VAL(I2C_5_PIN_SDA),
};
#endif

#if MYNEWT_VAL(ADC_0)
#define ADC_SAMPLE_BUF_SIZE 128
uint32_t g_ui32ADCSampleBuffer[ADC_SAMPLE_BUF_SIZE];

static struct adc_dev os_bsp_adc0;
static struct adc_cfg os_bsp_adc0_default_config = {
    .ADCConfig = {
        .eClock             = AM_HAL_ADC_CLKSEL_HFRC,
        .ePolarity          = AM_HAL_ADC_TRIGPOL_RISING,
        .eTrigger           = AM_HAL_ADC_TRIGSEL_SOFTWARE,
        .eReference         = AM_HAL_ADC_REFSEL_INT_1P5,
        .eClockMode         = AM_HAL_ADC_CLKMODE_LOW_LATENCY,
        .ePowerMode         = AM_HAL_ADC_LPMODE0,
        .eRepeat            = AM_HAL_ADC_REPEATING_SCAN,
    },
    .ADCSlotConfig = {
        .eMeasToAvg      = AM_HAL_ADC_SLOT_AVG_128,
        .ePrecisionMode  = AM_HAL_ADC_SLOT_14BIT,
        .eChannel        = AM_HAL_ADC_SLOT_CHSEL_SE0,
        .bWindowCompare  = false,
        .bEnabled        = true,
    },
    .ADCDMAConfig = {
        .bDynamicPriority = true,
        .ePriority = AM_HAL_ADC_PRIOR_SERVICE_IMMED,
        .bDMAEnable = true,
        .ui32SampleCount = ADC_SAMPLE_BUF_SIZE,
        .ui32TargetAddress = (uint32_t)g_ui32ADCSampleBuffer
    },
    .CLKConfig = {
        .clk_period = 10,
        .clk_on_time = 5,
        .clk_num = APOLLO3_ADC_CLOCK_3,
        .timer_ab = APOLLO3_ADC_TIMER_A,
        .timer_func = APOLLO3_ADC_TIMER_FUNC_REPEAT,
    }
};
#endif

const struct hal_flash *
hal_bsp_flash_dev(uint8_t id)
{
    if (id != 0) {
        return (NULL);
    }
    return &apollo3_flash_dev;
}

const struct hal_bsp_mem_dump *
hal_bsp_core_dump(int *area_cnt)
{
    *area_cnt = sizeof(dump_cfg) / sizeof(dump_cfg[0]);
    return dump_cfg;
}

void
hal_bsp_init(void)
{
    struct apollo3_timer_cfg timer_cfg;
    int rc;

    (void) timer_cfg;
    (void) rc;

#if MYNEWT_VAL(TIMER_0_SOURCE)
    timer_cfg.source = MYNEWT_VAL(TIMER_0_SOURCE);
    rc = hal_timer_init(0, &timer_cfg);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(TIMER_1_SOURCE)
    timer_cfg.source = MYNEWT_VAL(TIMER_1_SOURCE);
    rc = hal_timer_init(1, &timer_cfg);
    assert(rc == 0);
#endif
#if MYNEWT_VAL(TIMER_ADC_SOURCE)
    timer_cfg.source = MYNEWT_VAL(TIMER_ADC_SOURCE);
    rc = hal_timer_init(3, &timer_cfg);
    assert(rc == 0);
#endif

#if (MYNEWT_VAL(OS_CPUTIME_TIMER_NUM) >= 0)
    rc = os_cputime_init(MYNEWT_VAL(OS_CPUTIME_FREQ));
    assert(rc == 0);
#endif

#if MYNEWT_VAL(UART_0)
    rc = os_dev_create((struct os_dev *) &os_bsp_uart0, "uart0",
            OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *) &os_bsp_uart0_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(UART_1)
    rc = os_dev_create((struct os_dev *) &os_bsp_uart1, "uart1",
            OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *) &os_bsp_uart1_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_0_MASTER)
    rc = hal_spi_init(0, (void *)&hal_bsp_spi0m_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_1_MASTER)
    rc = hal_spi_init(1, (void *)&hal_bsp_spi1m_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_2_MASTER)
    rc = hal_spi_init(2, (void *)&hal_bsp_spi2m_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_3_MASTER)
    rc = hal_spi_init(3, (void *)&hal_bsp_spi3m_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_4_MASTER)
    rc = hal_spi_init(4, (void *)&hal_bsp_spi4m_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_5_MASTER)
    rc = hal_spi_init(5, (void *)&hal_bsp_spi5m_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(I2C_0)
    rc = hal_i2c_init(0, (void *)&hal_bsp_i2c0m_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(I2C_1)
    rc = hal_i2c_init(1, (void *)&hal_bsp_i2c1m_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(I2C_2)
    rc = hal_i2c_init(2, (void *)&hal_bsp_i2c2m_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(I2C_3)
    rc = hal_i2c_init(3, (void *)&hal_bsp_i2c3m_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(I2C_4)
    rc = hal_i2c_init(4, (void *)&hal_bsp_i2c4m_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(I2C_5)
    rc = hal_i2c_init(5, (void *)&hal_bsp_i2c5m_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(ADC_0)
    rc = os_dev_create(&os_bsp_adc0.ad_dev, "adc0",
                       OS_DEV_INIT_KERNEL, OS_DEV_INIT_PRIO_DEFAULT,
                       apollo3_adc_dev_init, &os_bsp_adc0_default_config);
    assert(rc == 0);
#endif
}

void
hal_bsp_deinit(void)
{
}

int
hal_bsp_hw_id_len(void)
{
    return 0;
}

int
hal_bsp_hw_id(uint8_t *id, int max_len)
{
    return 0;
}
