#ifndef _MODDEFS_H_
#define _MODDEFS_H_

/*
 * @brief Common definitions for modules.
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

#include <limits.h>

// Error codes.
#define MOD_ERR_ARG          -1
#define MOD_ERR_RESOURCE     -2
#define MOD_ERR_STATE        -3
#define MOD_ERR_BAD_CMD      -4
#define MOD_ERR_BUF_OVERRUN  -5
#define MOD_ERR_BAD_INSTANCE -6
#define MOD_ERR_PERIPH       -7
#define MOD_ERR_NOT_RESERVED -8
#define MOD_ERR_OP_IN_PROG   -9
#define MOD_ERR_UNAVAIL      -10
#define MOD_ERR_INTERNAL     -11
#define MOD_ERR_IMPL         -12
#define MOD_ERR_INFEASIBLE   -13
#define MOD_ERR_BUSY         -14

// Get size of an array.
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

// Increment a uint16_t, saturating at the maximum value.
#define INC_SAT_U16(a) do { (a) += ((a) == UINT16_MAX ? 0 : 1); } while (0)

// Clamp a numeric value between a lower and upper limit, inclusive.
#define CLAMP(a, low, high) ((a) <= (low) ? (low) : ((a) > (high) ? (high) : (a)))

// Concatenate tokens. See CONCAT_X_TOKEN for the case that one of the arguments
// is a macro that requires expansion before concatenation.
#define CONCAT_TOKEN(a, b) a ## b

// Concatenate tokens after expansion. In other words, if a macro is passed to
// this macro as an argument, it will be expanded before concatenation.
#define CONCAT_X_TOKEN(a, b) CONCAT_TOKEN(a, b)

// Critical region start/end macros, when it is known that you are running at
// the base level.
#define CRIT_START() __disable_irq()
#define CRIT_END() __enable_irq()

// Critical region start/end macros, which work regardless of where you are
// runing in a handler, or at the base level.
#define CRIT_STATE_VAR uint32_t _primask_save
#define CRIT_BEGIN_NEST()                                   \
    do {                                                    \
        _primask_save = __get_PRIMASK(); __set_PRIMASK(1);  \
    } while (0)
#define CRIT_END_NEST() do { __set_PRIMASK(_primask_save); } while (0)

#endif // _MODDEFS_H_
