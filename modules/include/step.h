#ifndef _STEP_H_
#define _STEP_H_

/*
 * @brief Interface declaration of step module.
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

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "dio.h"

////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////

enum step_instance_id {

#if CONFIG_STEP_1_PRESENT
    STEP_INSTANCE_1,
#endif

#if CONFIG_STEP_2_PRESENT
    STEP_INSTANCE_2,
#endif

    STEP_NUM_INSTANCES
};

enum step_drive_mode
{
    STEP_DRIVE_MODE_FULL,
    STEP_DRIVE_MODE_WAVE,
    STEP_DRIVE_MODE_HALF,

    STEP_NUM_DRIVE_MODES,
};

struct step_cfg
{
    dio_port* dio_port;
    uint32_t dio_pin_a;
    uint32_t dio_pin_not_a;
    uint32_t dio_pin_b;
    uint32_t dio_pin_not_b;
    uint32_t idle_timer_ms;
    bool rev_direction;
    enum step_drive_mode drive_mode;
};

enum step_cmd_type {
    STEP_CMD_N_EACH_M,   // Move N steps, at M ms per step.
    STEP_CMD_N_IN_M,     // Move N steps, evenly over an M ms interval.
    STEP_CMD_TO_P_IN_M,  // Move to position P, evenly over an M ms interval.

    STEP_NUM_CMD,
};

struct step_motor_info
{
    uint32_t min_ms_per_step;
    uint32_t steps_per_rev;
};
    
////////////////////////////////////////////////////////////////////////////////
// Public (global) externs
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Public (global) function declarations
////////////////////////////////////////////////////////////////////////////////

// Core module interface functions.
int32_t step_get_def_cfg(enum step_instance_id instance_id, struct step_cfg* cfg);
int32_t step_init(enum step_instance_id instance_id, struct step_cfg* cfg);
int32_t step_start(enum step_instance_id instance_id);

// Other APIs.
int32_t step_queue_cmd(enum step_instance_id instance_id,
                       enum step_cmd_type cmd_type,
                       int32_t param1, int32_t param2);
int32_t step_get_free_queue_slots(enum step_instance_id instance_id);
int32_t step_set_position(enum step_instance_id instance_id,
                          int32_t step_position);
int32_t step_energize(enum step_instance_id instance_id, bool energize);
int32_t step_get_info(enum step_instance_id instance_id,
                      struct step_motor_info* info);
int32_t step_set_drive_mode(enum step_instance_id instance_id,
                            enum step_drive_mode drive_mode);


#endif // _STEP_H_
