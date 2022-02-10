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

#include <assert.h>
#include "os/mynewt.h"
#include <hal/hal_bsp.h>
#include <adc/adc.h>
#include <mcu/cmsis_nvic.h>

/* Ambiq Apollo3 header */
#include "am_mcu_apollo.h"

#include "adc_apollo3/adc_apollo3.h"

/* Each slot can have a channel setting described by am_hal_adc_slot_chan_e */
static struct adc_chan_config apollo3_adc_chans[AM_HAL_ADC_MAX_SLOTS];
void *apollo3_adc_handle;

/**
 * Open the Apollo3 ADC device
 *
 * This function locks the device for access from other tasks.
 *
 * @param odev The OS device to open
 * @param wait The time in MS to wait.  If 0 specified, returns immediately
 *             if resource unavailable.  If OS_WAIT_FOREVER specified, blocks
 *             until resource is available.
 * @param arg  Argument provided by higher layer to open, in this case
 *             it can be a adc_cfg, to override the default
 *             configuration.
 *
 * @return 0 on success, non-zero on failure.
 */
static int
apollo3_adc_open(struct os_dev *odev, uint32_t wait, void *arg)
{
    int rc = 0;
    int unlock = 0;
    struct adc_dev *dev = (struct adc_dev *) odev;
    struct adc_cfg *adc_config = (struct adc_cfg *) arg;

    if (os_started()) {
        rc = os_mutex_pend(&dev->ad_lock, wait);
        if (rc != OS_OK) {
            goto err;
        }
        unlock = 1;
    }

    /* Initialize the ADC and get the handle. */
    am_hal_adc_initialize(0, &apollo3_adc_handle);

    /* Power on the ADC. */
    am_hal_adc_power_control(apollo3_adc_handle, AM_HAL_SYSCTRL_WAKE, false);

    /* Set up the ADC configuration parameters.*/
    am_hal_adc_configure(apollo3_adc_handle, &(adc_config->ADCConfig));

    /* ad_chan_count holds number of slots, each slot can configure a channel */
    for (int slot = 0; slot < dev->ad_chan_count; slot++) {
        /* Set up an ADC slot */
        am_hal_adc_configure_slot(apollo3_adc_handle, slot, &(adc_config->ADCSlotConfig));
    }

    /* Configure the ADC to use DMA for the sample transfer. */
    am_hal_adc_configure_dma(apollo3_adc_handle, &(adc_config->ADCDMAConfig));

    /* Wake up for each adc interrupt by default */
    am_hal_adc_interrupt_enable(apollo3_adc_handle, AM_HAL_ADC_INT_DERR | AM_HAL_ADC_INT_DCMP );

    /* Enable the ADC. */
    am_hal_adc_enable(apollo3_adc_handle);

err:
    if (unlock) {
        os_mutex_release(&dev->ad_lock);
    }

    return rc;
}

/**
 * Close the NRF52 ADC device.
 *
 * This function unlocks the device.
 *
 * @param odev The device to close.
 */
static int
apollo3_adc_close(struct os_dev *odev)
{
    struct adc_dev *dev;
    int rc = 0;
    int unlock = 0;

    dev = (struct adc_dev *) odev;

    if (os_started()) {
        rc = os_mutex_pend(&dev->ad_lock, OS_TIMEOUT_NEVER);
        if (rc != OS_OK) {
            goto err;
        }
        unlock = 1;
    }
    
    /* Initialize the ADC and get the handle. */
    am_hal_adc_deinitialize(&apollo3_adc_handle);

    /* Power on the ADC. */
    am_hal_adc_power_control(apollo3_adc_handle, AM_HAL_SYSCTRL_NORMALSLEEP, false);

    /* Wake up for each adc interrupt by default */
    am_hal_adc_interrupt_disable(apollo3_adc_handle, AM_HAL_ADC_INT_DERR | AM_HAL_ADC_INT_DCMP );

    /* Enable the ADC. */
    am_hal_adc_disable(apollo3_adc_handle);

err:
    if (unlock) {
        os_mutex_release(&dev->ad_lock);
    }

    return rc;
}

/**
 * Configure an ADC channel on the Nordic ADC.
 *
 * @param dev The ADC device to configure
 * @param cnum The channel on the ADC device to configure
 * @param cfgdata An opaque pointer to channel config, expected to be
 *                a adc_cfg
 *
 * @return 0 on success, non-zero on failure.
 */
static int
apollo3_adc_configure_channel(struct adc_dev *dev, uint8_t cnum, void *cfgdata)
{
    struct adc_cfg *adc_config = (struct adc_cfg *) cfgdata;
    
    if (cnum >= AM_HAL_ADC_MAX_SLOTS) {
        return OS_EINVAL;
    }

    /* Set up the ADC configuration parameters.*/
    am_hal_adc_configure(apollo3_adc_handle, &(adc_config->ADCConfig));

    /* Set up an ADC slot */
    am_hal_adc_configure_slot(apollo3_adc_handle, cnum, &(adc_config->ADCSlotConfig));

    /* Configure the ADC to use DMA for the sample transfer. */
    am_hal_adc_configure_dma(apollo3_adc_handle, &(adc_config->ADCDMAConfig));

    /* Store these values in channel definitions, for conversions to
     * milivolts.
     */
    dev->ad_chans[cnum].c_cnum = cnum;
    dev->ad_chans[cnum].c_res = adc_config->ADCSlotConfig.ePrecisionMode;
    dev->ad_chans[cnum].c_refmv = adc_config->ADCConfig.eReference;
    dev->ad_chans[cnum].c_configured = 1;

    return 0;
}

/**
 * Set buffer to read data into. Implementation of setbuffer handler.
 * Apollo3 cfg takes one buffer
 */
static int
apollo3_adc_set_buffer(struct adc_dev *dev, void *buf1, void *buf2, int buf_len)
{
    am_hal_adc_dma_config_t cfg;
    assert(dev);
    assert(buf1);

    if (buf_len <= 0) {
        return OS_EINVAL;
    }

    cfg = ((struct adc_cfg *)(dev->ad_dev.od_init_arg))->ADCDMAConfig;
    cfg.bDynamicPriority = true;
    cfg.ePriority = AM_HAL_ADC_PRIOR_SERVICE_IMMED;
    cfg.bDMAEnable = true;
    cfg.ui32TargetAddress = (uint32_t)buf1;
    cfg.ui32SampleCount = buf_len/sizeof(am_hal_adc_sample_t);

    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure_dma(apollo3_adc_handle, &cfg))
    {
        return OS_EINVAL;
    }

    return 0;
}

static int
apollo3_adc_release_buffer(struct adc_dev *dev, void *buf, int buf_len)
{
    am_hal_adc_dma_config_t cfg;
    assert(dev);
    assert(buf);

    if (buf_len <= 0) {
        return OS_EINVAL;
    }

    cfg = ((struct adc_cfg *)(dev->ad_dev.od_init_arg))->ADCDMAConfig;
    cfg.bDMAEnable = false;
    cfg.ui32TargetAddress = (uint32_t)buf;
    cfg.ui32SampleCount = buf_len/sizeof(am_hal_adc_sample_t);

    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure_dma(apollo3_adc_handle, &cfg))
    {
        return OS_EINVAL;
    }

    return 0;
}

/**
 * Trigger an ADC sample.
 */
static int
apollo3_adc_sample(struct adc_dev *dev)
{
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_sw_trigger(apollo3_adc_handle)) {
        return OS_EINVAL;
    }

    return 0;
}

/**
 * Blocking read of an ADC channel, returns result as an integer.
 */
static int
apollo3_adc_read_channel(struct adc_dev *dev, uint8_t cnum, int *result)
{
    int rc;
    int unlock = 0;
    struct adc_cfg * cfg= dev->ad_dev.od_init_arg;
    am_hal_adc_sample_t sample[cfg->ADCDMAConfig.ui32SampleCount];

    if (os_started()) {
        rc = os_mutex_pend(&dev->ad_lock, OS_TIMEOUT_NEVER);
        if (rc != OS_OK) {
            goto err;
        }
        unlock = 1;
    }

    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_samples_read(apollo3_adc_handle, true, (uint32_t *)cfg->ADCDMAConfig.ui32TargetAddress, &(cfg->ADCDMAConfig.ui32SampleCount), sample))
    {
        rc = OS_EINVAL;
        goto err;
    }

    *result = (int) sample[0].ui32Sample;
    rc = 0;

err:
    if (unlock) {
        os_mutex_release(&dev->ad_lock);
    }
    return rc;
}

static int
apollo3_adc_read_buffer(struct adc_dev *dev, void *buf, int buf_len, int off,
                      int *result)
{
    am_hal_adc_sample_t val;
    int data_off;

    data_off = off * sizeof(am_hal_adc_sample_t);
    assert(data_off < buf_len);

    val = *(am_hal_adc_sample_t *) ((uint8_t *) buf + data_off);
    *result = (int)val.ui32Sample;

    return 0;
}

static int
apollo3_adc_size_buffer(struct adc_dev *dev, int chans, int samples)
{
    return sizeof(am_hal_adc_sample_t) * chans * samples;
}

/**
 * ADC device driver functions
 */
static const struct adc_driver_funcs apollo3_adc_funcs = {
        .af_configure_channel = apollo3_adc_configure_channel,
        .af_sample = apollo3_adc_sample,
        .af_read_channel = apollo3_adc_read_channel,
        .af_set_buffer = apollo3_adc_set_buffer,
        .af_release_buffer = apollo3_adc_release_buffer,
        .af_read_buffer = apollo3_adc_read_buffer,
        .af_size_buffer = apollo3_adc_size_buffer,
};

/**
 * Callback to initialize an adc_dev structure from the os device
 * initialization callback.  This sets up a nrf52_adc_device(), so
 * that subsequent lookups to this device allow us to manipulate it.
 */
int
apollo3_adc_dev_init(struct os_dev *odev, void *arg)
{
    struct adc_dev *dev;
    dev = (struct adc_dev *) odev;

    os_mutex_init(&dev->ad_lock);

    dev->ad_chans = (void *) apollo3_adc_chans;
    dev->ad_chan_count = AM_HAL_ADC_MAX_SLOTS;
    dev->ad_dev.od_init_arg = (struct adc_cfg *) arg;

    OS_DEV_SETHANDLERS(odev, apollo3_adc_open, apollo3_adc_close);
    dev->ad_funcs = &apollo3_adc_funcs;

    return 0;
}
