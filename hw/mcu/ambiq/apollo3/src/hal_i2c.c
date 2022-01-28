/*
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
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <stdbool.h>
#include "os/mynewt.h"
#include "hal/hal_i2c.h"
#include "mcu/hal_apollo3.h"
#include "mcu/cmsis_nvic.h"

#include "am_mcu_apollo.h"

/* Prevent CMSIS from breaking apollo3 macros. */
#undef GPIO
#undef IOSLAVE
#undef CLKGEN

/* Pointer array that points to am_hal_iom g_IOMhandles */
am_hal_iom_state_t          *g_IOMhandles[AM_REG_IOM_NUM_MODULES];

static am_hal_iom_config_t g_sIOMI2cConfig =
{
    .eInterfaceMode = AM_HAL_IOM_I2C_MODE,
    .ui32ClockFreq  = AM_HAL_IOM_1MHZ,
};

/*  | i2c:cfg   | scl   | sda   |
 *  |-----------+-------+-------|
 *  | 0:0       | 5     | 6     |
 *  | 1:0       | 8     | 9     |
 *  | 2:4       | 27    | 25    |
 *  | 3:4       | 42    | 43    |
 *  | 4:4       | 39    | 40    |
 *  | 5:4       | 48    | 49    |
 */
static int
hal_i2c_pin_config(int i2c_num, const struct apollo3_i2c_cfg *pins)
{
    switch (i2c_num) {
#if I2C_0
    case 0:
        if (pins->scl_pin == 5 && pins->sda_pin == 6) {
            return 0;
        } else {
            return -1;
        }
#endif
#if I2C_1
    case 1:
        if (pins->scl_pin == 8 && pins->sda_pin == 9) {
            return 0;
        } else {
            return -1;
        }
#endif
#if I2C_2
    case 2:
        if (pins->scl_pin == 27 && pins->sda_pin == 25) {
            return 4;
        } else {
            return -1;
        }
#endif
#if I2C_3
    case 3:
        if (pins->scl_pin == 42 && pins->sda_pin == 43) {
            return 4;
        } else {
            return -1;
        }
#endif
#if I2C_4
    case 4:
        if (pins->scl_pin == 39 && pins->sda_pin == 40) {
            return 4;
        } else {
            return -1;
        }
#endif
#if I2C_5
    case 5:
        if (pins->scl_pin == 48 && pins->sda_pin == 49) {
            return 4;
        } else {
            return -1;
        }
#endif
    default:
        return -1;
    }
}

int
hal_i2c_init(uint8_t i2c_num, void *usercfg)
{
    int pin_cfg;
    am_hal_gpio_pincfg_t i2c_cfg;
    struct apollo3_i2c_cfg *cfg = usercfg;

    //
    // Initialize the IOM.
    //
    am_hal_iom_initialize(i2c_num, &g_IOMHandles[i2c_num]);

    am_hal_iom_power_ctrl(g_IOMHandles[i2c_num], AM_HAL_SYSCTRL_WAKE, false);

    //
    // Set the required configuration settings for the IOM.
    //
    am_hal_iom_configure(g_IOMHandles[i2c_num], &g_sIOMI2cConfig);

    /* Configure GPIOs for I2C based on i2c_num */
    pin_cfg = hal_i2c_pin_config(i2c_num, cfg);
    if (pin_cfg == -1) {
        return SYS_EINVAL;
    }

    i2c_cfg.uFuncSel            = pin_cfg;
    i2c_cfg.ePullup             = AM_HAL_GPIO_PIN_PULLUP_1_5K;
    i2c_cfg.eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA;
    i2c_cfg.eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN;
    i2c_cfg.uIOMnum             = i2c_num;

    am_hal_gpio_pinconfig(cfg->sda_pin,  i2c_cfg);
    am_hal_gpio_pinconfig(cfg->scl_pin,  i2c_cfg);

    return 0;
}

int
hal_i2c_master_write(uint8_t i2c_num, struct hal_i2c_master_data *pdata,
                     uint32_t timeout, uint8_t last_op)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui32InstrLen    = 0;
    Transaction.ui32Instr       = 0;
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = pdata->len;
    Transaction.pui32TxBuffer   = (uint32_t *)pdata->buffer;
    Transaction.bContinue       = !(bool)last_op;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32I2CDevAddr = pdata->address;

    g_IOMHandles[i2c_num]->waitTimeout = timeout;
    if (am_hal_iom_blocking_transfer(g_IOMHandles[i2c_num], &Transaction) != AM_HAL_STATUS_SUCCESS)
        return -1;

    return 0;
}

int
hal_i2c_master_read(uint8_t i2c_num, struct hal_i2c_master_data *pdata,
                    uint32_t timeout, uint8_t last_op)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui32InstrLen    = 0;
    Transaction.ui32Instr       = 0;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = pdata->len;
    Transaction.pui32RxBuffer   = (uint32_t *)pdata->buffer;
    Transaction.bContinue       = !(bool)last_op;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32I2CDevAddr = pdata->address;

    g_IOMHandles[i2c_num]->waitTimeout = timeout;
    if (am_hal_iom_blocking_transfer(g_IOMHandles[i2c_num], &Transaction) != AM_HAL_STATUS_SUCCESS)
        return -1;

    return 0;
}