/*
 * @brief Implementation of console module.
 *
 * This module supports a CLI console on a ttys. Upon receiving a command it
 * passes the command string to the cmd module for execution.
 *
 * This module provides simple line discipline functions:
 * - Echoing received characters.
 * - Handling backspace/delete.
 *
 * This module recognizes a special character to enable/disable logging output
 * (i.e. toggle). This is handy to temporarily stop logging output when running
 * commands.
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

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cmd.h"
#include "config.h"
#include "console.h"
#include "log.h"
#include "module.h"
#include "ttys.h"

////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

#define PROMPT "> "

////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////

#define CONSOLE_CMD_BFR_SIZE 80

struct console_state {
    struct console_cfg cfg;
    char cmd_bfr[CONSOLE_CMD_BFR_SIZE];
    uint16_t num_cmd_bfr_chars;
    bool first_run_done;
};
////////////////////////////////////////////////////////////////////////////////
// Private (static) function declarations
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Private (static) variables
////////////////////////////////////////////////////////////////////////////////

// We initialze the default ttys instance so we can get console output
// very early after boot.

static struct console_state state = {
    .cfg.ttys_instance_id = CONFIG_CONSOLE_DFLT_TTYS_INSTANCE,
};

static int32_t log_level = LOG_DEFAULT;

////////////////////////////////////////////////////////////////////////////////
// Public (global) variables and externs
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Public (global) functions
////////////////////////////////////////////////////////////////////////////////

/*
 * @brief Get default console configuration.
 *
 * @param[out] cfg The console configuration with defaults filled in.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t console_get_def_cfg(struct console_cfg* cfg)
{
    if (cfg == NULL)
        return MOD_ERR_ARG;

    memset(cfg, 0, sizeof(*cfg));
    cfg->ttys_instance_id = CONFIG_CONSOLE_DFLT_TTYS_INSTANCE;
    return 0;
}

/*
 * @brief Initialize console module instance.
 *
 * @param[in] cfg The console configuration. (FUTURE)
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function initializes the console singleton module. Generally, it should
 * not access other modules as they might not have been initialized yet. An
 * exception is the log module.
 */
int32_t console_init(struct console_cfg* cfg)
{
    if (cfg == NULL) {
        return MOD_ERR_ARG;
    }
    log_debug("In console_init()\n");
    memset(&state, 0, sizeof(state));
    state.cfg = *cfg;
    return 0;
}

/*
 * @brief Run console instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * @note This function should not block.
 *
 * This function runs the console singleton module, during normal operation.
 */
int32_t console_run(void)
{
    char c;
    if (!state.first_run_done) {
        state.first_run_done = true;
        printc("%s", PROMPT);
    }
            
    while (ttys_getc(state.cfg.ttys_instance_id, &c)) {

        // Handle processing completed command line.
        if (c == '\n' || c == '\r') {
            state.cmd_bfr[state.num_cmd_bfr_chars] = '\0';
            printc("\n");
            cmd_execute(state.cmd_bfr);
            state.num_cmd_bfr_chars = 0;
            printc("%s", PROMPT);
            continue;
        }

        // Handle backspace/delete.
        if (c == '\b' || c == '\x7f') {
            if (state.num_cmd_bfr_chars > 0) {
                // Overwrite last character with a blank.
                printc("\b \b");
                state.num_cmd_bfr_chars--;
            }

            continue;
        }

        // Handle logging on/off toggle.
        if (c == LOG_TOGGLE_CHAR) {
            log_toggle_active();
            printc("\n<Logging %s>\n", log_is_active() ? "on" : "off");
            continue;
        }

        // Echo the character back.
        if (isprint(c)) {
            if (state.num_cmd_bfr_chars < (CONSOLE_CMD_BFR_SIZE-1)) {
                state.cmd_bfr[state.num_cmd_bfr_chars++] = c;
                printc("%c", c);
            } else {
                // No space in buffer for the character, so ring the bell.
                printc("\a");
            }
            continue;
        }
            
    }
    return 0;
}            

/*
 * @brief A printf for the console, safe to use in interrupts.
 *
 * @param[in] fmt Format string as in printf.
 *
 * @return Number of characters written as in printf.
 */
int printc(const char* fmt, ...)
{
    va_list args;
    char buf[CONFIG_CONSOLE_PRINT_BUF_SIZE];
    int rc;
    int idx;

    va_start(args, fmt);
    rc = vsnprintf(buf, CONFIG_CONSOLE_PRINT_BUF_SIZE, fmt, args);
    va_end(args);
    for (idx = 0; idx < rc; idx++) {
        ttys_putc(state.cfg.ttys_instance_id, buf[idx]);
        if (buf[idx] == '\0')
            break;
        if (buf[idx] == '\n') 
            ttys_putc(state.cfg.ttys_instance_id, '\r');
    }
    if (rc >= CONFIG_CONSOLE_PRINT_BUF_SIZE)
        printc("[!]\n");
    return rc;
}

/*
 * @brief A vprintf for the console, safe to use in interrupts.
 *
 * @param[in] fmt Format string as in vprintf.
 * @param[in] args Format arguments as in vprintf.
 *
 * @return Number of characters written as in vprintf.
 */
int vprintc(const char* fmt, va_list args)
{
    char buf[CONFIG_CONSOLE_PRINT_BUF_SIZE];
    int rc;
    int idx;

    rc = vsnprintf(buf, CONFIG_CONSOLE_PRINT_BUF_SIZE, fmt, args);
    for (idx = 0; idx < rc; idx++) {
        ttys_putc(state.cfg.ttys_instance_id, buf[idx]);
        if (buf[idx] == '\0')
            break;
        if (buf[idx] == '\n') 
            ttys_putc(state.cfg.ttys_instance_id, '\r');
    }
    if (rc >= CONFIG_CONSOLE_PRINT_BUF_SIZE)
        printc("[!]\n");
    return rc;
}

void printc_float(const char* prefix, float f, uint32_t max_frac_width,
                  const char* suffix)
{
    float unit_part;
    float frac_part;

    if (prefix != NULL)
        printc("%s", prefix);

    if (f < 0.0F) {
        printc("-");
        f = -f;
    }
    frac_part = modff(f, &unit_part);
    printc("%ld.", (int32_t)unit_part);

    while (frac_part != 0.0F && max_frac_width > 0) {
        max_frac_width--;
        unit_part = frac_part * 10.0F;
        frac_part = modff(unit_part, &unit_part);
        printc("%ld", (int32_t)unit_part);
    }

    if (suffix != NULL)
        printc("%s", suffix);
}

////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////
