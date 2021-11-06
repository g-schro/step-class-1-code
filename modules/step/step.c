/*
 * @brief Implementation of step module.
 *
 * This module drives stepper motors. Multiple instances are supported,
 * and each instance drives a single stepper motor.
 *
 * The usage model is to add motion commands. There is a motion command queue,
 * so it is possible to add motion commands "in advance" to get continuous
 * operation.
 *
 * Threading considerations:
 * - Only software running at the base level can invoke the public APIs.  For
 *   example, a motion command cannot be added from an interrupt handler.  This
 *   reduces the size and number of critical regions.
 *
 * The following console commands are provided:
 * > step status
 * > step test
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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "cmd.h"
#include "config.h"
#include "console.h"
#include "log.h"
#include "module.h"
#include "step.h"
#include "tmr.h"

////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

#define MAX_NUM_DRIVE_PATTERNS 8
#define CMD_QUEUE_SIZE 3
#define ZERO_STEP_MOVE ULONG_MAX
#define MIN_STEP_MS 3
#define STEPS_PER_REV 2048

////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////

struct motion_cmd {
    enum step_cmd_type cmd_type;
    int32_t steps;
    uint32_t ms;
};

struct step_state {
    struct step_cfg cfg;
    struct motion_cmd motion_cmds[CMD_QUEUE_SIZE];
    int32_t position;
    uint32_t cmd_steps_remaining;
    uint32_t cmd_num_steps;
    uint32_t last_step_ms;
    int32_t tmr_id;
    uint16_t drive_pattern[MAX_NUM_DRIVE_PATTERNS];
    uint16_t num_drive_patterns;
    uint16_t last_drive_pattern;
    int8_t drive_pattern_idx;
    int8_t step_delta;
    uint8_t get_cmd_idx;
    uint8_t put_cmd_idx;
    uint8_t active_cmd_idx;
    bool idle_timer_running;
};

////////////////////////////////////////////////////////////////////////////////
// Private (static) function declarations
////////////////////////////////////////////////////////////////////////////////

static enum tmr_cb_action timer_callback(int32_t tmr_id, uint32_t user_data);
static int32_t cmd_step_status(int32_t argc, const char** argv);
static int32_t cmd_step_test(int32_t argc, const char** argv);

////////////////////////////////////////////////////////////////////////////////
// Private (static) variables
////////////////////////////////////////////////////////////////////////////////

static struct step_state step_states[STEP_NUM_INSTANCES];

static int32_t log_level = LOG_DEFAULT;

// Data structure with console command info.
static struct cmd_cmd_info cmds[] = {
    {
        .name = "status",
        .func = cmd_step_status,
        .help = "Get module status, usage: step status",
    },
    {
        .name = "test",
        .func = cmd_step_test,
        .help = "Run test, usage: step test [<op> [<arg>]] (enter no op/arg for help)",
    }
};

// Data structure passed to cmd module for console interaction.
static struct cmd_client_info cmd_info = {
    .name = "step",
    .num_cmds = ARRAY_SIZE(cmds),
    .cmds = cmds,
    .log_level_ptr = &log_level,
};

////////////////////////////////////////////////////////////////////////////////
// Public (global) variables and externs
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Public (global) functions
////////////////////////////////////////////////////////////////////////////////

/*
 * @brief Get default step configuration.
 *
 * @param[in] instance_id Identifies the step instance.
 * @param[out] cfg The step configuration with defaults filled in.
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t step_get_def_cfg(enum step_instance_id instance_id, struct step_cfg* cfg)
{
    switch (instance_id) {
#if CONFIG_STEP_1_PRESENT
        case STEP_INSTANCE_1:
            cfg->dio_port = CONFIG_STEP_1_DFLT_GPIO_PORT;
            cfg->dio_pin_a = CONFIG_STEP_1_DFLT_DIO_PIN_A;
            cfg->dio_pin_not_a = CONFIG_STEP_1_DFLT_DIO_PIN_NOT_A;
            cfg->dio_pin_b = CONFIG_STEP_1_DFLT_DIO_PIN_B;
            cfg->dio_pin_not_b = CONFIG_STEP_1_DFLT_DIO_PIN_NOT_B;
            cfg->idle_timer_ms = CONFIG_STEP_1_DFLT_IDLE_TIMER_MS;
            cfg->rev_direction = CONFIG_STEP_1_DFLT_REV_DIRECTION;
            cfg->drive_mode = CONFIG_STEP_1_DFLT_DRIVE_MODE;
            break;
#endif

#if CONFIG_STEP_2_PRESENT
        case STEP_INSTANCE_2:
            cfg->dio_port = CONFIG_STEP_2_DFLT_GPIO_PORT;
            cfg->dio_pin_a = CONFIG_STEP_2_DFLT_DIO_PIN_A;
            cfg->dio_pin_not_a = CONFIG_STEP_2_DFLT_DIO_PIN_NOT_A;
            cfg->dio_pin_b = CONFIG_STEP_2_DFLT_DIO_PIN_B;
            cfg->dio_pin_not_b = CONFIG_STEP_2_DFLT_DIO_PIN_NOT_B;
            cfg->idle_timer_ms = CONFIG_STEP_2_DFLT_IDLE_TIMER_MS;
            cfg->rev_direction = CONFIG_STEP_2_DFLT_REV_DIRECTION;
            cfg->drive_mode = CONFIG_STEP_2_DFLT_DRIVE_MODE;
            break;
#endif

        default:
            return MOD_ERR_BAD_INSTANCE;
    }
    return 0;
}

/*
 * @brief Initialize step instance.
 *
 * @param[in] instance_id Identifies the step instance.
 * @param[in] cfg The step configuration. (FUTURE)
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function initializes a step module instance. Generally, it should not
 * access other modules as they might not have been initialized yet.  An
 * exception is the log module.
 */
int32_t step_init(enum step_instance_id instance_id, struct step_cfg* cfg)
{
    uint32_t rc;
    struct step_state* st;

    if (instance_id >= STEP_NUM_INSTANCES)
        return MOD_ERR_BAD_INSTANCE;

    if (cfg == NULL)
        return MOD_ERR_ARG;

    st = &step_states[instance_id];
    memset(st, 0, sizeof(*st));
    st->cfg = *cfg;

    rc = step_set_drive_mode(instance_id, cfg->drive_mode);
    st->active_cmd_idx = CMD_QUEUE_SIZE;
    return rc;
}

/*
 * @brief Start step instance.
 *
 * @param[in] instance_id Identifies the step instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function starts a step module instance, to enter normal operation.
 */
int32_t step_start(enum step_instance_id instance_id)
{
    struct step_state* st;
    int32_t result;

    if (instance_id >= STEP_NUM_INSTANCES)
        return MOD_ERR_BAD_INSTANCE;

    st = &step_states[instance_id];

    st->tmr_id = tmr_inst_get_cb(0, timer_callback, instance_id, true);
    if (st->tmr_id < 0) {
        log_error("step_start: tmr error %d\n", st->tmr_id);
        return MOD_ERR_RESOURCE;
    }

    result = cmd_register(&cmd_info);
    if (result < 0) {
        log_error("step_start: cmd error %d\n", result);
        return MOD_ERR_RESOURCE;
    }

    {
        struct dio_out_rt_cfg dio_cfg = {
            .port = st->cfg.dio_port,
            .pin_mask = (st->cfg.dio_pin_a | st->cfg.dio_pin_not_a |
                         st->cfg.dio_pin_b | st->cfg.dio_pin_not_b),
            .pull = DIO_PULL_NO,
            .init_value = 0,
            .speed = DIO_SPEED_FREQ_HIGH,
            .output_type = DIO_OUTPUT_PUSHPULL
        };
        result = dio_cfg_out(&dio_cfg);
        if (result != 0) {
            log_error("step_start: dio_cfg_oout error %d\n", result);
            return result;
        }
    }
    return 0;
}

/*
 * @brief Queue a step command to this module.
 *
 * @param[in] instance_id Identifies the step instance.
 * @param[in] cmd_type The command type.
 * @param[in] steps Command parameter (see below)
 * @param[in] ms Command parameter (see below)
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 *    cmd_type               steps              ms
 * ------------------  -------------------- --------------
 * STEP_CMD_N_EACH_M   N: Steps to move     M: Ms per step
 * STEP_CMD_N_IN_M     N: Steps to move     M: Total ms
 * STEP_CMD_TO_P_IN_M  P: Step destination  M: Total ms
 *
 * Note that this function just puts the command info in the queue.  Commands
 * are always started from the timer callback handler.
 */
int32_t step_queue_cmd(enum step_instance_id instance_id,
                       enum step_cmd_type cmd_type,
                       int32_t steps, int32_t ms)
{
    struct step_state* st;
    struct motion_cmd* cmd;
    uint8_t next_put_cmd_idx;

    if (instance_id >= STEP_NUM_INSTANCES)
        return MOD_ERR_BAD_INSTANCE;

    if (cmd_type >= STEP_NUM_CMD || ms == 0)
        return MOD_ERR_ARG;

    st = &step_states[instance_id];
    next_put_cmd_idx = st->put_cmd_idx + 1;
    if (next_put_cmd_idx >= CMD_QUEUE_SIZE)
        next_put_cmd_idx = 0;
    if (next_put_cmd_idx == st->get_cmd_idx)
        return MOD_ERR_RESOURCE;
    cmd = &st->motion_cmds[st->put_cmd_idx];
    cmd->steps = steps;
    cmd->ms = ms;
    cmd->cmd_type = cmd_type;
    log_debug("step_queue_cmd type=%d steps=%ld ms=%ld\n",
              cmd_type, steps, ms);
    st->put_cmd_idx = next_put_cmd_idx;

    // If there is no command running, cancel any idle timer, and start a timer
    // that will start the command. We block interrupts since the timer handler,
    // which can modify the timer, runs from an interrupt.

    CRIT_START();
    if (st->active_cmd_idx >= CMD_QUEUE_SIZE) {
        st->idle_timer_running = false;
        tmr_inst_start(st->tmr_id, 1);
    }
    CRIT_END();
    return 0;
}

/*
 * @brief Get number of free motion command queue slots.
 *
 * @param[in] instance_id Identifies the step instance.
 *
 * @return Number of free slots (non-negative) for success, else a "MOD_ERR"
 *         value. See code for details.
 */
int32_t step_get_free_queue_slots(enum step_instance_id instance_id)
{
    struct step_state* st;
    uint32_t ctr = 0;
    uint8_t put_idx;
    uint8_t get_idx;

    if (instance_id >= STEP_NUM_INSTANCES)
        return MOD_ERR_BAD_INSTANCE;
    st = &step_states[instance_id];
    put_idx = st->put_cmd_idx;
    get_idx = st->get_cmd_idx;
    // While loop does sanity check.
    while (ctr <= STEP_NUM_CMD) {
        put_idx += 1;
        if (put_idx >= STEP_NUM_CMD)
            put_idx = 0;
        if (put_idx == get_idx)
            break;
        ctr += 1;
    }
    return ctr <= STEP_NUM_CMD ? ctr : MOD_ERR_INTERNAL;
}

/*
 * @brief Set the logical step position.
 *
 * @param[in] instance_id Identifies the step instance.
 * @param[in] step_position Current position in steps.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function is normally called while the motor is stopped, but that is not
 * enforced.
 *
 * The step position value is arbitrary. It comes into play when a "step to
 * position" command is issued.
 */
int32_t step_set_position(enum step_instance_id instance_id,
                          int32_t step_position)
{
    if (instance_id >= STEP_NUM_INSTANCES)
        return MOD_ERR_BAD_INSTANCE;

    step_states[instance_id].position = step_position;
    return 0;
}

/*
 * @brief Engernize/de-energize the motor.
 *
 * @param[in] instance_id Identifies the step instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t step_energize(enum step_instance_id instance_id, bool energize)
{
    struct step_state* st;

    if (instance_id >= STEP_NUM_INSTANCES)
        return MOD_ERR_BAD_INSTANCE;

    st = &step_states[instance_id];
    if (st->active_cmd_idx != CMD_QUEUE_SIZE)
        return MOD_ERR_STATE;

    // Normally, drive_pattern_idx is the last pattern that was used (it is
    // incremented or decremented right before the next step is made).

    if (energize) {
        uint16_t drive_pattern = st->drive_pattern[st->drive_pattern_idx];

        dio_set_reset_outputs(st->cfg.dio_port, drive_pattern,
                              st->last_drive_pattern & (~drive_pattern));
        st->last_drive_pattern = drive_pattern;
    } else {
        dio_reset_outputs(st->cfg.dio_port,
                          st->cfg.dio_pin_a | st->cfg.dio_pin_not_a |
                          st->cfg.dio_pin_b | st->cfg.dio_pin_not_b);
    }
    return 0;
}

/*
 * @brief Get information about the motor.
 *
 * @param[in]  instance_id Identifies the step instance.
 * @param[out] info Information about the motor.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t step_get_info(enum step_instance_id instance_id,
                     struct step_motor_info* info)
{
    if (instance_id >= STEP_NUM_INSTANCES)
        return MOD_ERR_BAD_INSTANCE;
    if (info == NULL)
        return MOD_ERR_ARG;
    info->min_ms_per_step = MIN_STEP_MS;
    info->steps_per_rev =
        step_states[instance_id].cfg.drive_mode == STEP_DRIVE_MODE_HALF ?
        STEPS_PER_REV * 2 : STEPS_PER_REV;
    return 0;
}

/*
 * @brief Set the drive mode.
 *
 * @param[in] instance_id Identifies the step instance.
 * @param[in] drive_mode Driver mode.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */

int32_t step_set_drive_mode(enum step_instance_id instance_id,
                            enum step_drive_mode drive_mode)
{
    struct step_state* st;

    if (instance_id >= STEP_NUM_INSTANCES)
        return MOD_ERR_BAD_INSTANCE;

    st = &step_states[instance_id];

    switch (drive_mode)
    {
        case STEP_DRIVE_MODE_FULL:
            st->num_drive_patterns = 4;
            st->drive_pattern[0] = st->cfg.dio_pin_a | st->cfg.dio_pin_not_b;
            st->drive_pattern[1] = st->cfg.dio_pin_a | st->cfg.dio_pin_b;
            st->drive_pattern[2] = st->cfg.dio_pin_not_a | st->cfg.dio_pin_b;
            st->drive_pattern[3] = st->cfg.dio_pin_not_a | st->cfg.dio_pin_not_b;
            break;
        case STEP_DRIVE_MODE_WAVE:
            st->num_drive_patterns = 4;
            st->drive_pattern[0] = st->cfg.dio_pin_a; 
            st->drive_pattern[1] = st->cfg.dio_pin_b; 
            st->drive_pattern[2] = st->cfg.dio_pin_not_a; 
            st->drive_pattern[3] = st->cfg.dio_pin_not_b; 
            break;
        case STEP_DRIVE_MODE_HALF:
            st->num_drive_patterns = 8;
            st->drive_pattern[0] = st->cfg.dio_pin_a;
            st->drive_pattern[1] = st->cfg.dio_pin_a | st->cfg.dio_pin_b;
            st->drive_pattern[2] = st->cfg.dio_pin_b;
            st->drive_pattern[3] = st->cfg.dio_pin_not_a | st->cfg.dio_pin_b;
            st->drive_pattern[4] = st->cfg.dio_pin_not_a;
            st->drive_pattern[5] = st->cfg.dio_pin_not_a | st->cfg.dio_pin_not_b;
            st->drive_pattern[6] = st->cfg.dio_pin_not_b;
            st->drive_pattern[7] = st->cfg.dio_pin_a | st->cfg.dio_pin_not_b;
            break;
        default:
            return MOD_ERR_ARG;
    }
    if (st->drive_pattern_idx >= st->num_drive_patterns)
        st->drive_pattern_idx = 0;
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////

/*
 * @brief Step timer callback.
 *
 * @param[in] tmr_id Timer ID.
 * @param[in] user_data User callback data (is instance ID)
 *
 * @return TMR_CB_NONE/TMR_CB_RESTART
 *
 * This function handles any current motion in progress, and also handles
 * starting new commands in the qeuue.
 */
static enum tmr_cb_action timer_callback(int32_t tmr_id, uint32_t user_data)
{
    struct step_state* st;
    struct motion_cmd* cmd;
    uint32_t next_step_ms;
    int32_t tmp_steps;
    uint8_t next_get_cmd_idx;
    bool check_for_next_cmd = false;

    if (user_data >= STEP_NUM_INSTANCES)
        // Shouldn't happen.
        return TMR_CB_NONE;

    st = &step_states[user_data];
    if (st->active_cmd_idx >= CMD_QUEUE_SIZE) {
        if (st->idle_timer_running) {
            st->idle_timer_running = false;
            step_energize(user_data, false);
            return TMR_CB_NONE;
        } else {
            check_for_next_cmd = true;
        }
    } else {
        cmd = &st->motion_cmds[st->active_cmd_idx];
        log_trace("timer_callback cmd_steps_remaining=%lu\n",
                  st->cmd_steps_remaining);
        if (st->cmd_steps_remaining == ZERO_STEP_MOVE) {
            st->cmd_steps_remaining = 0;
            check_for_next_cmd = true;
        } else if (st->cmd_steps_remaining > 0) {
            // Take the step.
            uint16_t drive_pattern;
            int8_t drive_pattern_idx = st->drive_pattern_idx + st->step_delta;
            if (drive_pattern_idx < 0)
                drive_pattern_idx = st->num_drive_patterns - 1;
            else if (drive_pattern_idx >= st->num_drive_patterns)
                drive_pattern_idx = 0;
            st->drive_pattern_idx = drive_pattern_idx;
            log_trace("step phase %u\n", drive_pattern_idx);
            drive_pattern = st->drive_pattern[st->drive_pattern_idx];
            dio_set_reset_outputs(st->cfg.dio_port, drive_pattern,
                                  st->last_drive_pattern & (~drive_pattern));
            st->last_drive_pattern = drive_pattern;
            st->position += st->step_delta;

            // Check if the command is done.
            if (--st->cmd_steps_remaining > 0) {
                // Compute time until next step.
                if (cmd->cmd_type == STEP_CMD_N_EACH_M) {
                    tmr_inst_set_period(st->tmr_id, cmd->ms);
                } else {
                    // In the "N steps in M milliseconds" cases, the cumulative
                    // time for each step can be calculated as
                    // ((M * N_s + N/2) / N) with integer divides, where N_s is
                    // the step number 1, 2, 3, ..., N.
                    next_step_ms =
                        (cmd->ms *
                         (st->cmd_num_steps + 1 - st->cmd_steps_remaining) +
                         st->cmd_num_steps / 2) /
                        st->cmd_num_steps;
                    tmr_inst_set_period(st->tmr_id,
                                        next_step_ms - st->last_step_ms);
                    st->last_step_ms = next_step_ms;
                }
                return TMR_CB_RESTART;
            } else {
                check_for_next_cmd = true;
            }
        }
    }

    if (check_for_next_cmd) {
        if (st->get_cmd_idx == st->put_cmd_idx) {
            // Command queue is empty.
            st->active_cmd_idx = CMD_QUEUE_SIZE;
            if (st->cfg.idle_timer_ms > 0) {
                st->idle_timer_running = true;
                tmr_inst_set_period(st->tmr_id, st->cfg.idle_timer_ms);
                return TMR_CB_RESTART;
            } else {
                return TMR_CB_NONE;
            }
        }

        // Get next command from queue.
        next_get_cmd_idx = st->get_cmd_idx + 1;
        if (next_get_cmd_idx >= CMD_QUEUE_SIZE)
            next_get_cmd_idx = 0;
        st->active_cmd_idx = st->get_cmd_idx;
        st->get_cmd_idx = next_get_cmd_idx;
    }

    // Start next command.
    cmd = &st->motion_cmds[st->active_cmd_idx];
    if (cmd->cmd_type == STEP_CMD_TO_P_IN_M)
        tmp_steps =  cmd->steps - st->position;
    else
        tmp_steps = cmd->steps;

    st->step_delta = 1;
    if (tmp_steps == 0) {
        // Special case of a zero step move.
        st->cmd_steps_remaining = ZERO_STEP_MOVE;
    } else if (tmp_steps < 0) {
        // Step count must be positive; direction handled via delta.
        st->cmd_steps_remaining = -tmp_steps;
        st->step_delta = -1;
    } else {
        st->cmd_steps_remaining = tmp_steps;
    }
    if (st->cfg.rev_direction)
        st->step_delta  *= -1;

    st->cmd_num_steps = st->cmd_steps_remaining;

    if (cmd->cmd_type == STEP_CMD_N_EACH_M ||
        st->cmd_num_steps == ZERO_STEP_MOVE)
        next_step_ms = cmd->ms;
    else
        next_step_ms = cmd->ms / st->cmd_num_steps;
    st->last_step_ms = next_step_ms;
    log_trace("tmr period=%lu\n", next_step_ms);
    tmr_inst_set_period(st->tmr_id, next_step_ms);
    return TMR_CB_RESTART;
}

/*
 * @brief Console command function for "step status".
 *
 * @param[in] argc Number of arguments, including "step"
 * @param[in] argv Argument values, including "step"
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: step status
 */
static int32_t cmd_step_status(int32_t argc, const char** argv)
{
    uint32_t idx;
    struct step_state* st;
    struct motion_cmd* mc;
    uint8_t get_idx;
    uint8_t put_idx;

    printc("              Num   Steps Last  Pat Step Cmd\n"
           "ID  Position  Steps Rmain Ms    Idx Dlta Idx\n"
           "-- ---------- ----- ----- ----- --- ---- ---\n");
    for (idx = 0, st = step_states; idx < STEP_NUM_INSTANCES; idx++, st++) {
        printc("%2lu %10ld %5lu %5lu %5lu %3d %4d %3u\n",
               idx,
               st->position,
               st->cmd_num_steps,
               st->cmd_steps_remaining,
               st->last_step_ms,
               st->drive_pattern_idx,
               st->step_delta,
               st->active_cmd_idx);
        get_idx = st->get_cmd_idx;
        put_idx = st->put_cmd_idx;
        while (get_idx != put_idx) {
            mc = &st->motion_cmds[get_idx];
            printc("   Q%u: %d %ld %lu\n",
                   get_idx,
                   mc->cmd_type,
                   mc->steps,
                   mc->ms);
            get_idx += 1;
            if (get_idx >= CMD_QUEUE_SIZE)
                get_idx = 0;
        }
    }
    return 0;
}

/*
 * @brief Console command function for "step test".
 *
 * @param[in] argc Number of arguments, including "step"
 * @param[in] argv Argument values, including "step"
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: step test [<op> [<arg>]]
 */
static int32_t cmd_step_test(int32_t argc, const char** argv)
{
    int32_t rc;
    struct cmd_arg_val arg_vals[3];
    enum step_instance_id instance_id = 0;

    // Handle help case.
    if (argc == 2) {
        printc("Test operations and param(s) are as follows:\n"
               "  Step cmd, usage: step test cmd <instance-id> <cmd-type> <steps> <ms>\n"
               "  Free slots, usage: step test free <instance-id>\n"
               "  Energize, usage: step test energize <instance-id> <0/1>\n");
        printc("  Set mode, usage: step test mode <instance-id> <mode-idx>\n");
        printc("Cmd types:\n"
               "  %d: N steps, M ms/step\n"
               "  %d: N steps over M ms\n"
               "  %d: To position N over M ms\n",
               STEP_CMD_N_EACH_M,
               STEP_CMD_N_IN_M,
               STEP_CMD_TO_P_IN_M);
        return 0;
    }

    // Get instance ID.
    if (cmd_parse_args(argc-3, argv+3, "u+", arg_vals) != 1) {
        printc("Can't get instance ID\n");
        return MOD_ERR_BAD_CMD;
    }
    instance_id = (enum step_instance_id)arg_vals[0].val.u;
    if (instance_id >= STEP_NUM_INSTANCES) {
        printc("Bad instance\n");
        return MOD_ERR_BAD_INSTANCE;
    }

    if (strcasecmp(argv[2], "cmd") == 0) {
        rc = cmd_parse_args(argc-4, argv+4, "uiu", arg_vals);
        if (rc != 3) {
            return MOD_ERR_BAD_CMD;
        }
        rc = step_queue_cmd(instance_id,
                            arg_vals[0].val.u,
                            arg_vals[1].val.i,
                            arg_vals[2].val.u);
    } else if (strcasecmp(argv[2], "free") == 0) {
        rc = step_get_free_queue_slots(instance_id);
    } else if (strcasecmp(argv[2], "energize") == 0) {
        rc = cmd_parse_args(argc-4, argv+4, "u", arg_vals);
        if (rc != 1) {
            return MOD_ERR_BAD_CMD;
        }
        rc = step_energize(instance_id, arg_vals[0].val.u > 0);
    } else if (strcasecmp(argv[2], "mode") == 0) {
        rc = cmd_parse_args(argc-4, argv+4, "u", arg_vals);
        if (rc != 1) {
            return MOD_ERR_BAD_CMD;
        }
        rc = step_set_drive_mode(instance_id, arg_vals[0].val.u);
    } else {
        printc("Invalid operation '%s'\n", argv[2]);
        return MOD_ERR_BAD_CMD;
    }
    printc("Return code %ld\n", rc);
    return 0;
}
