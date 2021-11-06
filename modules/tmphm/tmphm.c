/*
 * @brief Implementation of tmphm module.
 *
 * This module supports that Sensirion SHT31-D temperature and humidify
 * sensor. This sensor has an I2C interface.
 *
 * This module polls the sensor at a configurable rate (e.g. once every 1000
 * ms). The module can can be queried at any time for the last measurement,
 * including the "age" (in ms) of that measurement.
 *
 * The following console commands are provided:
 * > tmphm status
 * > tmphm pm
 * > tmphm log
 * See code for details.
 *
 * @ref Senssirion Datasheet SHT3x-DIS. February 2019 - Version 6.
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
#include <stdio.h>
#include <string.h>

#include "cmd.h"
#include "console.h"
#include "i2c.h"
#include "log.h"
#include "module.h"
#include "tmr.h"

#include "tmphm.h"

////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////

enum states {
    STATE_IDLE,
    STATE_RESERVE_I2C,
    STATE_WRITE_MEAS_CMD,
    STATE_WAIT_MEAS,
    STATE_READ_MEAS_VALUE
};

#define I2C_MSG_BFR_LEN 6

// Per-instance tmphm state information.
struct tmphm_state {
    struct tmphm_cfg cfg;
    struct tmphm_meas last_meas;
    int32_t tmr_id;
    uint32_t i2c_op_start_ms;
    uint32_t last_meas_ms;
    uint8_t msg_bfr[I2C_MSG_BFR_LEN];
    bool got_meas;
    enum states state;
};

// Performance measurements for i2c. Currently these are common to all
// instances.  A future enhancement would be to make them per-instance.

enum tmphm_u16_pms {
    CNT_RESERVE_FAIL,
    CNT_WRITE_INIT_FAIL,
    CNT_WRITE_OP_FAIL,
    CNT_READ_INIT_FAIL,
    CNT_READ_OP_FAIL,
    CNT_TASK_OVERRUN,
    CNT_CRC_FAIL,

    NUM_U16_PMS
};

////////////////////////////////////////////////////////////////////////////////
// Private (static) function declarations
////////////////////////////////////////////////////////////////////////////////

static enum tmr_cb_action tmr_callback(int32_t tmr_id, uint32_t user_data);
static int32_t cmd_tmphm_status(int32_t argc, const char** argv);
static int32_t cmd_tmphm_test(int32_t argc, const char** argv);
static uint8_t crc8(const uint8_t *data, int len);

////////////////////////////////////////////////////////////////////////////////
// Private (static) variables
////////////////////////////////////////////////////////////////////////////////

static struct tmphm_state tmphm_states[TMPHM_NUM_INSTANCES];

static int32_t log_level = LOG_INFO;

// Storage for performance measurements.
static uint16_t cnts_u16[NUM_U16_PMS];

// Names of performance measurements.
static const char* cnts_u16_names[NUM_U16_PMS] = {
    "reserve fail",
    "write init fail",
    "write op fail",
    "read init fail",
    "read op fail",
    "task overrun",
    "crc error",
};

// Data structure with console command info.
static struct cmd_cmd_info cmds[] = {
    {
        .name = "status",
        .func = cmd_tmphm_status,
        .help = "Get module status, usage: tmphm status",
    },
    {
        .name = "test",
        .func = cmd_tmphm_test,
        .help = "Run test, usage: tmphm test [<op> [<arg>]] (enter no op/arg for help)",
    }
};

// Data structure passed to cmd module for console interaction.
static struct cmd_client_info cmd_info = {
    .name = "tmphm",
    .num_cmds = ARRAY_SIZE(cmds),
    .cmds = cmds,
    .log_level_ptr = &log_level,
    .num_u16_pms = NUM_U16_PMS,
    .u16_pms = cnts_u16,
    .u16_pm_names = cnts_u16_names,
};

const char sensor_i2c_cmd[2] = {0x2c, 0x06 };

////////////////////////////////////////////////////////////////////////////////
// Public (global) variables and externs
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Public (global) functions
////////////////////////////////////////////////////////////////////////////////

/*
 * @brief Get default tmphm configuration.
 *
 * @param[in] instance_id Identifies the tmphm instance.
 * @param[out] cfg The tmphm configuration with defaults filled in.
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t tmphm_get_def_cfg(enum tmphm_instance_id instance_id, struct tmphm_cfg* cfg)
{
    if (instance_id == TMPHM_INSTANCE_1) {
        cfg->i2c_instance_id = CONFIG_TMPHM_1_DFLT_I2C_INSTANCE;
        cfg->i2c_addr = CONFIG_TMPHM_1_DFLT_I2C_ADDR;
    } else {
        return MOD_ERR_ARG;
    }
    cfg->sample_time_ms = CONFIG_TMPHM_DFLT_SAMPLE_TIME_MS;
    cfg->meas_time_ms = CONFIG_TMPHM_DFLT_MEAS_TIME_MS;
    return 0;
}
/*
 * @brief Initialize tmphm instance.
 *
 * @param[in] instance_id Identifies the tmphm instance.
 * @param[in] cfg The tmphm configuration. (FUTURE)
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function initializes a tmphm module instance. Generally, it should not
 * access other modules as they might not have been initialized yet.  An
 * exception is the log module.
 */
int32_t tmphm_init(enum tmphm_instance_id instance_id, struct tmphm_cfg* cfg)
{
    if (instance_id >= TMPHM_NUM_INSTANCES)
        return MOD_ERR_BAD_INSTANCE;

    tmphm_states[instance_id].cfg = *cfg;
    return 0;
}

/*
 * @brief Start tmphm instance.
 *
 * @param[in] instance_id Identifies the tmphm instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function starts a tmphm module instance, to enter normal operation.
 */
int32_t tmphm_start(enum tmphm_instance_id instance_id)
{
    struct tmphm_state* st;
    int32_t rc;

    if (instance_id >= TMPHM_NUM_INSTANCES)
        return MOD_ERR_BAD_INSTANCE;

    rc = cmd_register(&cmd_info);
    if (rc < 0) {
        log_error("tmphm_start: cmd error %d\n", rc);
        return MOD_ERR_RESOURCE;
    }

    st = &tmphm_states[instance_id];

    st->tmr_id = tmr_inst_get_cb(st->cfg.sample_time_ms, tmr_callback,
                                 (uint32_t)instance_id, false);
    if (st->tmr_id < 0)
        return MOD_ERR_RESOURCE;

    return 0;
}

/*
 * @brief Run tmphm instance.
 *
 * @param[in] instance_id Identifies the tmphm instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * @note This function should not block.
 *
 * This function runs a tmphm module instance, during normal operation.
 */
int32_t tmphm_run(enum tmphm_instance_id instance_id)
{
    struct tmphm_state* st;
    int32_t rc;

    if (instance_id >= TMPHM_NUM_INSTANCES)
        return MOD_ERR_BAD_INSTANCE;

    st = &tmphm_states[instance_id];

    switch (st->state) {
        case STATE_IDLE:
            break;

        case STATE_RESERVE_I2C:
            rc = i2c_reserve(st->cfg.i2c_instance_id);
            if (rc == 0) {
                memcpy(st->msg_bfr, sensor_i2c_cmd, sizeof(sensor_i2c_cmd));
                rc = i2c_write(st->cfg.i2c_instance_id, st->cfg.i2c_addr, st->msg_bfr,
                               sizeof(sensor_i2c_cmd));
                if (rc == 0) {
                    st->state = STATE_WRITE_MEAS_CMD;
                } else {
                    INC_SAT_U16(cnts_u16[CNT_WRITE_INIT_FAIL]);
                    i2c_release(st->cfg.i2c_instance_id);
                    st->state = STATE_IDLE;
                }
            } else {
                INC_SAT_U16(cnts_u16[CNT_RESERVE_FAIL]);
            }
            break;

        case STATE_WRITE_MEAS_CMD:
            rc = i2c_get_op_status(st->cfg.i2c_instance_id);
            if (rc != MOD_ERR_OP_IN_PROG) {
                if (rc == 0) {
                    st->i2c_op_start_ms = tmr_get_ms();
                    st->state = STATE_WAIT_MEAS;
                } else {
                    INC_SAT_U16(cnts_u16[CNT_WRITE_OP_FAIL]);
                    i2c_release(st->cfg.i2c_instance_id);
                    st->state = STATE_IDLE;
                }
            }
            break;

        case STATE_WAIT_MEAS:
            if (tmr_get_ms() - st->i2c_op_start_ms >= st->cfg.meas_time_ms) {
                rc = i2c_read(st->cfg.i2c_instance_id, st->cfg.i2c_addr,
                              st->msg_bfr, I2C_MSG_BFR_LEN);
                if (rc == 0) {
                    st->state = STATE_READ_MEAS_VALUE;
                } else {
                    INC_SAT_U16(cnts_u16[CNT_READ_INIT_FAIL]);
                    i2c_release(st->cfg.i2c_instance_id);
                    st->state = STATE_IDLE;
                }
            }
            break;

        case STATE_READ_MEAS_VALUE:
            rc = i2c_get_op_status(st->cfg.i2c_instance_id);
            if (rc != MOD_ERR_OP_IN_PROG) {
                if (rc == 0) {
                    const uint32_t divisor = 65535;
                    uint8_t* msg = st->msg_bfr;

                    if (crc8(&msg[0], 2) != msg[2] ||
                        crc8(&msg[3], 2) != msg[5]) {
                        INC_SAT_U16(cnts_u16[CNT_CRC_FAIL]);
                    } else {
                        int32_t temp = (msg[0] << 8) + msg[1];
                        uint32_t hum = (msg[3] << 8) + msg[4];
                        temp = -450 +
                            (1750 * temp + divisor/2) / divisor;
                        hum = (1000 * hum + divisor/2) / divisor;
                        st->last_meas.temp_deg_c_x10 = temp;
                        st->last_meas.rh_percent_x10 = hum;
                        st->last_meas_ms = tmr_get_ms();
                        st->got_meas = true;
                        log_info("temp=%ld degC*10 hum=%d %%*10\n",
                                 temp, hum);
                    }
                    
                } else {
                    INC_SAT_U16(cnts_u16[CNT_READ_OP_FAIL]);
                }
                i2c_release(st->cfg.i2c_instance_id);
                st->state = STATE_IDLE;
            }
            break;
    }
    return 0;
}

/*
 * @brief Get last measurement.
 *
 * @param[in]  instance_id Identifies the tmphm instance.
 * @param[out] meas Measurement structure to fill in.
 * @param[out] meas_age_ms Variable to place measurement age in ms (or NULL).
 *                 
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t tmphm_get_last_meas(enum tmphm_instance_id instance_id,
                            struct tmphm_meas* meas, uint32_t* meas_age_ms)
{
    struct tmphm_state* st;

    if (instance_id >= TMPHM_NUM_INSTANCES)
        return MOD_ERR_BAD_INSTANCE;
    if (meas == NULL)
        return MOD_ERR_ARG;

    st = &tmphm_states[instance_id];
    if (!st->got_meas)
        return MOD_ERR_UNAVAIL;

    *meas = st->last_meas;
    if (meas_age_ms != NULL)
        *meas_age_ms = tmr_get_ms() - st->last_meas_ms;

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////

static enum tmr_cb_action tmr_callback(int32_t tmr_id, uint32_t user_data)
{
    enum tmphm_instance_id instance_id = (enum tmphm_instance_id)user_data;
    struct tmphm_state* st;

    log_trace("tmr_callback(tmr_id=%d user_data=%lu)\n", tmr_id, user_data);
    if (instance_id >= TMPHM_NUM_INSTANCES)
        return TMR_CB_RESTART;

    st = &tmphm_states[instance_id];
    if (st->state == STATE_IDLE)
        st->state = STATE_RESERVE_I2C;
    else
        INC_SAT_U16(cnts_u16[CNT_TASK_OVERRUN]);
    return TMR_CB_RESTART;
}

/*
 * @brief Console command function for "tmphm status".
 *
 * @param[in] argc Number of arguments, including "tmphm"
 * @param[in] argv Argument values, including "tmphm"
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: tmphm status
 */
static int32_t cmd_tmphm_status(int32_t argc, const char** argv)
{
    uint32_t instance_id;

    for (instance_id = 0; instance_id < TMPHM_NUM_INSTANCES; instance_id++)
    {
        uint32_t idx;
        struct tmphm_state* st;
        st = &tmphm_states[instance_id];

        printc("         Got  Last Last Meas Meas\n"
               "ID State Meas Temp Hum  Age  Time\n"
               "-- ----- ---- ---- ---- ---- ----\n");
        for (idx = 0, st = tmphm_states;
             idx < TMPHM_NUM_INSTANCES;
             idx++, st++) {
            printc("%2lu %5d %4d %4u %4u %4lu %4lu\n", idx, st->state,
                   st->got_meas, st->last_meas.temp_deg_c_x10,
                   st->last_meas.rh_percent_x10,
                   tmr_get_ms() - st->last_meas_ms, st->cfg.meas_time_ms);
        }
    }
    return 0;
}

/*
 * @brief Console command function for "tmphm test".
 *
 * @param[in] argc Number of arguments, including "tmphm"
 * @param[in] argv Argument values, including "tmphm"
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: tmphm test [<op> [<arg>]]
 */
static int32_t cmd_tmphm_test(int32_t argc, const char** argv)
{
    struct cmd_arg_val arg_vals[4];
    int32_t rc;
    uint32_t idx;
    enum tmphm_instance_id instance_id = 0;

    // Handle help case.
    if (argc == 2) {
        printc("Test operations and param(s) are as follows:\n"
               "  Get last meas, usage: tmphm test lastmeas <instance-id>\n"
               "  Set meas time, usage: tmphm test meastime <instance-id> <time-ms>\n"
               "  Test crc8, usage: tmphm test crc8 byte1 ... (up to 4 bytes)\n"
            );
        return 0;
    }

    if (argc < 3) {
        return MOD_ERR_BAD_CMD;
    }

    // Get instance ID (except for msg option).
    if (strcasecmp(argv[2], "crc8") != 0) {
        rc = cmd_parse_args(argc-3, argv+3, "u+", arg_vals);
        if (rc < 1)
            return MOD_ERR_BAD_CMD;
        instance_id = (enum i2c_instance_id)arg_vals[0].val.u;
        if (instance_id >= TMPHM_NUM_INSTANCES) {
            printc("Bad instance\n");
            return MOD_ERR_BAD_INSTANCE;
        }
    }

    if (strcasecmp(argv[2], "lastmeas") == 0) {
        struct tmphm_meas meas;
        uint32_t meas_age_ms;
        rc = tmphm_get_last_meas(instance_id, &meas, &meas_age_ms);
        if (rc == 0)
            printf("Temp=%d.%d C Hum=%u.%u %% age=%lu ms\n",
                   meas.temp_deg_c_x10 / 10,
                   meas.temp_deg_c_x10 % 10,
                   meas.rh_percent_x10 / 10,
                   meas.rh_percent_x10 % 10,
                   meas_age_ms);
        else
            printf("tmphm_get_last_meas fails rc=%ld\n", rc);
    } else if (strcasecmp(argv[2], "meastime") == 0) {
        rc = cmd_parse_args(argc-4, argv+4, "u", arg_vals);
        if (rc != 1) {
            return MOD_ERR_BAD_CMD;
        }
        tmphm_states[instance_id].cfg.meas_time_ms = arg_vals[0].val.u;
    } else if (strcasecmp(argv[2], "crc8") == 0) {
        uint8_t data[4];

        rc = cmd_parse_args(argc-3, argv+3, "u[u[u[u]]]", arg_vals);
        if (rc < 1) {
            return MOD_ERR_BAD_CMD;
        }
        for (idx = 0; idx < 4; idx++)
            data[idx] = (uint8_t)arg_vals[idx].val.u;

        printc("crc8: 0x%02x\n", crc8(data, rc));
    } else {
        printc("Invalid operation '%s'\n", argv[2]);
        return MOD_ERR_BAD_CMD;
    }

    return 0;
}


/*
 * @brief CRC8 calculation.
 *
 * @param[in] data Pointer to data to be checked.
 * @param[in] len Number of data bytes.
 *
 * @return The CRC.
 *
 * Because the data to be validated is small (2 bytes) we don't use a faster
 * table-based algoritm.
 *
 * I can't find the origin of this code - there are very many copies of this
 * code, or something very similar to it, on the web.
 *
 */

static uint8_t crc8(const uint8_t *data, int len)
{
    //
    // Parameters comes from page 14 of the STH31-D datasheet (see @ref).
    // - Polynomial: x^8 + x^5 + x^4 + x^0 which is 100110001 binary, or 0x31 in
    //   "normal" representation
    // - Initialization: 0xff.
    // - Final XOR: 0x00 (i.e. none).
    // - Example: 0xbeef => 0x92.
    //
    const uint8_t polynomial = 0x31;
    uint8_t crc = 0xff;
    uint32_t idx1;
    uint32_t idx2;

    for (idx1 = len; idx1 > 0; idx1--)
    {
        crc ^= *data++;
        for (idx2 = 8; idx2 > 0; idx2--)
        {
            crc = (crc & 0x80) ? (crc << 1) ^ polynomial : (crc << 1);
        }
    }
    return crc;
}
