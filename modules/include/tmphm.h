#ifndef _TMPHM_H_
#define _TMPHM_H_

/*
 * @brief Interface declaration of tmphm module.
 *
 * See implementation file for information about this module.
 *
 * MIT License
 * 
 * Copyright (c) 2021 Eugene R Schroeder
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdint.h>

#include "config.h"
#include "i2c.h"
#include "module.h"

enum tmphm_instance_id {
    TMPHM_INSTANCE_1,

    TMPHM_NUM_INSTANCES
};

struct tmphm_cfg
{
    enum i2c_instance_id i2c_instance_id;
    uint32_t i2c_addr;
    uint32_t sample_time_ms;
    uint32_t meas_time_ms;
};

struct tmphm_meas
{
    int16_t temp_deg_c_x10;
    uint16_t rh_percent_x10;
};

// Core module interface functions.
int32_t tmphm_get_def_cfg(enum tmphm_instance_id instance_id, struct tmphm_cfg* cfg);
int32_t tmphm_init(enum tmphm_instance_id instance_id, struct tmphm_cfg* cfg);
int32_t tmphm_start(enum tmphm_instance_id instance_id);
int32_t tmphm_run(enum tmphm_instance_id instance_id);

// Other APIs.
int32_t tmphm_get_last_meas(enum tmphm_instance_id instance_id,
                            struct tmphm_meas* meas, uint32_t* meas_age_ms);
#endif // _TMPHM_H_
