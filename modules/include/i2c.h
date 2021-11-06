#ifndef _I2C_H_
#define _I2C_H_

/*
 * @brief Interface declaration of i2c module.
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

enum i2c_errors {
    I2C_ERR_NONE,  // Must have value 0.
    I2C_ERR_INVALID_INSTANCE,
    I2C_ERR_BUS_BUSY,
    I2C_ERR_GUARD_TMR,
    I2C_ERR_PEC,
    I2C_ERR_TIMEOUT,
    I2C_ERR_ACK_FAIL,
    I2C_ERR_BUS_ERR,
    I2C_ERR_INTR_UNEXPECT,
};

// I2C numbering is based on the MCU hardware definition.

enum i2c_instance_id {

#if CONFIG_I2C_1_PRESENT
    I2C_INSTANCE_1,
#endif

#if CONFIG_I2C_2_PRESENT
    I2C_INSTANCE_2,
#endif

#if CONFIG_I2C_3_PRESENT
    I2C_INSTANCE_3,
#endif

    I2C_NUM_INSTANCES
};

struct i2c_cfg {
    uint32_t transaction_guard_time_ms;
};

// Core module interface functions.
int32_t i2c_get_def_cfg(enum i2c_instance_id instance_id, struct i2c_cfg* cfg);
int32_t i2c_init(enum i2c_instance_id instance_id, struct i2c_cfg* cfg);
int32_t i2c_start(enum i2c_instance_id instance_id);

// Other APIs.
int32_t i2c_reserve(enum i2c_instance_id instance_id);
int32_t i2c_release(enum i2c_instance_id instance_id);

int32_t i2c_write(enum i2c_instance_id instance_id, uint32_t dest_addr,
                  uint8_t* msg_bfr, uint32_t msg_len);
int32_t i2c_read(enum i2c_instance_id instance_id, uint32_t dest_addr,
                 uint8_t* msg_bfr, uint32_t msg_len);

int32_t i2c_get_op_status(enum i2c_instance_id instance_id);
enum i2c_errors i2c_get_error(enum i2c_instance_id instance_id);
int32_t i2c_bus_busy(enum i2c_instance_id instance_id);

#endif // _I2C_H_
