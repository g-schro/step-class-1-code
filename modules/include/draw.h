#ifndef _DRAW_H_
#define _DRAW_H_

/*
 * @brief Interface declaration of draw module.
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

#include "step.h"

////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

enum draw_move_type {
    DRAW_MOVE_TYPE_JOINT,
    DRAW_MOVE_TYPE_CART,
};

#define DRAW_NUM_MOTORS 2
#define DRAW_NUM_LINKS 2

////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////

struct draw_cfg
{
    enum step_instance_id step_motor_instance[DRAW_NUM_MOTORS];
    uint32_t link_len_mm[DRAW_NUM_LINKS];
};

////////////////////////////////////////////////////////////////////////////////
// Public (global) externs
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Public (global) function declarations
////////////////////////////////////////////////////////////////////////////////

// Core module interface functions.
int32_t draw_get_def_cfg(struct draw_cfg* cfg);
int32_t draw_init(struct draw_cfg* cfg);
int32_t draw_start(void);
int32_t draw_run(void);

// Other APIs.
int32_t draw_move_to(float x_mm, float y_mm, enum draw_move_type move_type);
int32_t draw_jog_joint(uint32_t joint_idx, float jog_degrees);
int32_t draw_joint_calib(float theta_1_deg, float theta_2_deg);

#endif // _DRAW_H_
