/*
 * @brief Implementation of float module.
 *
 * This module is simply used to study floating point operation.
 *
 * The following console commands are provided:
 * > float status
 * See code for details.
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

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "cmd.h"
#include "console.h"
#include "log.h"
#include "module.h"
#include "tmr.h"

#include "float.h"

////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

#if CONFIG_FLOAT_TYPE_FLOAT
    #define FLOAT_TYPE_NAME "float"
#elif CONFIG_FLOAT_TYPE_DOUBLE
    #define FLOAT_TYPE_NAME "double"
#elif CONFIG_FLOAT_TYPE_LONG_DOUBLE
    #define FLOAT_TYPE_NAME "long double"
#else
    #error No float type specified.
#endif

////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Private (static) function declarations
////////////////////////////////////////////////////////////////////////////////

static int32_t cmd_float_status(int32_t argc, const char** argv);
static int32_t cmd_float_test(int32_t argc, const char** argv);

////////////////////////////////////////////////////////////////////////////////
// Private (static) variables
////////////////////////////////////////////////////////////////////////////////

static int32_t log_level = LOG_DEFAULT;

// Data structure with console command info.
static struct cmd_cmd_info cmds[] = {
    {
        .name = "status",
        .func = cmd_float_status,
        .help = "Get module status, usage: float status",
    },
    {
        .name = "test",
        .func = cmd_float_test,
        .help = "Run test, usage: float test [<op> [<arg>]] (enter no op/arg for help)",
    }
};

// Data structure passed to cmd module for console interaction.
static struct cmd_client_info cmd_info = {
    .name = "float",
    .num_cmds = ARRAY_SIZE(cmds),
    .cmds = cmds,
    .log_level_ptr = &log_level,
};

////////////////////////////////////////////////////////////////////////////////
// Public (global) variables and externs
////////////////////////////////////////////////////////////////////////////////

void whetstone(int iterations, int only_float);

////////////////////////////////////////////////////////////////////////////////
// Public (global) functions
////////////////////////////////////////////////////////////////////////////////

/*
 * @brief Get default float configuration.
 *
 * @param[out] cfg The float configuration with defaults filled in.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t float_get_def_cfg(struct float_cfg* cfg)
{
    return 0;
}

/*
 * @brief Initialize float instance.
 *
 * @param[in] cfg The float configuration. (FUTURE)
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function initializes the float singleton module. Generally, it should
 * not access other modules as they might not have been initialized yet. An
 * exception is the log module.
 */
int32_t float_init(struct float_cfg* cfg)
{
    return 0;
}

/*
 * @brief Start float instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function starts the float singleton module, to enter normal operation.
 */
int32_t float_start(void)
{
    int32_t rc;

    rc = cmd_register(&cmd_info);
    if (rc < 0) {
        log_error("float_start: cmd error %d\n", rc);
        return MOD_ERR_RESOURCE;
    }
    return 0;
}

/*
 * @brief Run float instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * @note This function should not block.
 *
 * This function runs the float singleton module, during normal operation.
 */
int32_t float_run(void)
{
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////

static int float_test(float arg_f, double arg_d)
{
    float a_f = arg_f * arg_f;
    float b_f = arg_d * arg_d;
    float c_f = sin(arg_f);
    float d_f = sinf(arg_f);
    a_f = a_f * 1.234;
    a_f = a_f * 1.234F;
    return a_f + b_f + c_f + d_f;
}

/*
 * @brief Console command function for "float status".
 *
 * @param[in] argc Number of arguments, including "float"
 * @param[in] argv Argument values, including "float"
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: float status
 */
static int32_t cmd_float_status(int32_t argc, const char** argv)
{
    float_test(1.2F, 1.2);
    return 0;
}

/*
 * @brief Console command function for "float test".
 *
 * @param[in] argc Number of arguments, including "float"
 * @param[in] argv Argument values, including "float"
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: float test [<op> [<arg>]]
 */
static int32_t cmd_float_test(int32_t argc, const char** argv)
{
    // Handle help case.
    if (argc == 2) {
        printc("Float operations and param(s) are as follows:\n"
               "  Run whetstone, usage: float test whet [only-float]\n"
               "  Misc calcs, usage: float test calc\n");
        return 0;
    }

    if (argc != 3 && argc != 4)
        return MOD_ERR_BAD_CMD;

    if (strcasecmp(argv[2], "whet") == 0) {
        int only_float = 0;
        uint32_t start_time_ms;
        uint32_t elapsed_time_ms;

        if (argc == 4) {
            if (strcasecmp(argv[3], "only-float") == 0) {
                only_float = 1;
            } else {
                printc("Invalid whetstone arg\n");
                return MOD_ERR_BAD_CMD;
            }
        }
        start_time_ms = tmr_get_ms();
        whetstone(10, only_float);
        elapsed_time_ms = tmr_get_ms() - start_time_ms;
        printc_float("Million whetstones per second: ",
                     1000.0F/((float)elapsed_time_ms), 2, " (type "
                     FLOAT_TYPE_NAME ")\n");
    }
    else if (strcasecmp(argv[2], "calc") == 0) {
        int idx;
        float z1 = 1.F;
        float z2 = 0.F;
        float nan1 = sqrt(-1.F);
        float infp = 1.F / 0.F;
        float infn = -1.F / 0.F;
        z1 -= 1.F;
        printc("isfinite: z1=%d z2=%d nan1=%d infp=%d infn=%d\n",
               isfinite(z1), isfinite(z2), isfinite(nan1), isfinite(infp),
               isfinite(infn));
        printc("isnormal: z1=%d z2=%d nan1=%d infp=%d infn=%d\n",
               isnormal(z1), isnormal(z2), isnormal(nan1), isnormal(infp),
               isnormal(infn));
        printc("isnan: z1=%d z2=%d nan1=%d infp=%d infn=%d\n",
               isnan(z1), isnan(z2), isnan(nan1), isnan(infp),
               isnan(infn));
        z1 = 1.F;
        for (idx = 0; idx < 256; idx++) {
            bool iszero;
            z1 = z1/2.F;
            iszero = z1 == 0.F;
            if (isnormal(z1) == 1) continue;
            printc("idx=%d iszero=%d isnormal=%d\n",
                   idx, iszero, isnormal(z1));
            if (iszero) break;

        }
    } else {
        printc("Invalid command\n");
    }
        
    return 0;
}

