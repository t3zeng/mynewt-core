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

#ifndef __ADC_APOLLO3_H__
#define __ADC_APOLLO3_H__

#include <adc/adc.h>
#include "am_mcu_apollo.h"

#ifdef __cplusplus
extern "C" {
#endif

struct adc_cfg {
    am_hal_adc_config_t ADCConfig;
    am_hal_adc_slot_config_t ADCSlotConfig;
    am_hal_adc_dma_config_t ADCDMAConfig;
};

int apollo3_adc_dev_init(struct os_dev *, void *);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_APOLLO3_H__ */
