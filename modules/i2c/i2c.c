/*
 * @brief Implementation of i2c module.
 *
 * This module supports the I2C interface. It is a multiple-instance module, one
 * instance for each I2C interface. Main features:
 * - Master mode. It does not support slave mode.
 * - Single-message read and write transactions. It does not support multiple
 *   operations per transaction.
 * - 7-bit addressing supported. It does not support 10-bit addressing.
 *
 * Usage is like this:
 * 1. You reserve an I2C interface using i2c_reserve(), polling as necessary.
 * 2. You initiate an operation using i2c_write() or i2c_read().
 * 3. You check the status of the operation using i2c_get_op_status(), polling
 *    as necessary.
 * 4. In the case of an "in progress" error (MOD_ERR_OP_IN_PROG), try again
 *    later. If you get a "peripheral" error (MOD_ERR_PERIPH), use
 *    i2co_get_error() to get an I2C-specific error code.
 * 5. You release the reservation using i2c_release().
 *
 * The following console commands are provided:
 * > i2c status
 * > i2c test
 * > i2c pm
 * > i2c log
 * See code for details.
 *
 * Currently, this module does not perform hardware initialization of the I2C
 * and associated hardware (e.g. GPIO), except for the interrupt controller (see
 * below). It is expected that the LL driver library has been used for
 * initilazation (e.g. via generated IDE code). This avoids having to deal with
 * the issue of inconsistent driver libraries among MCUs.
 *
 * This module enables I2C interrupts on the Nested Vector Interrupt
 * Controller (NVIC). It also overrides the (weak) I2C interrupt handler
 * functions (I2Cx_xx_IRQHandler). Thus, in the IDE device configuration tool,
 * the "I2Cx event/error interrupt" should NOT be chosen or you will get a
 * duplicate symbol at link time.

 * @see RM0368 Reference Manual for STM32F401xD/E, Rev 5, particularly Section
 * 18.3.3 (I2C master mode)
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
#include <stdio.h>
#include <string.h>

#include "config.h"
#include CONFIG_STM32_LL_GPIO_HDR
#include CONFIG_STM32_LL_I2C_HDR

#include "cmd.h"
#include "console.h"
#include "log.h"
#include "module.h"
#include "tmr.h"

#include "i2c.h"

#if CONFIG_I2C_TYPE == 1

////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

#define INTERRUPT_ENABLE_MASK (LL_I2C_CR2_ITEVTEN | LL_I2C_CR2_ITBUFEN | \
                               LL_I2C_CR2_ITERREN)
#define DISABLE_ALL_INTERRUPTS(st) st->i2c_reg_base->CR2 &= \
        ~INTERRUPT_ENABLE_MASK
#define ENABLE_ALL_INTERRUPTS(st) st->i2c_reg_base->CR2 |= \
        INTERRUPT_ENABLE_MASK

// Defines based on reference manual Table 71.
#define INTERRUPT_EVT_MASK (I2C_SR1_SB | I2C_SR1_ADDR | I2C_SR1_ADD10 | \
                            I2C_SR1_STOPF | I2C_SR1_BTF | I2C_SR1_RXNE | \
                            I2C_SR1_TXE)

#define INTERRUPT_ERR_MASK (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | \
                            I2C_SR1_OVR | I2C_SR1_PECERR | I2C_SR1_TIMEOUT | \
                            I2C_SR1_SMBALERT)

////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////

enum states {
    STATE_IDLE,
    STATE_MSTR_WR_GEN_START,
    STATE_MSTR_WR_SENDING_ADDR,
    STATE_MSTR_WR_SENDING_DATA,
    STATE_MSTR_RD_GEN_START,
    STATE_MSTR_RD_SENDING_ADDR,
    STATE_MSTR_RD_READING_DATA,
};

enum interrupt_type {
    INTER_TYPE_EVT,
    INTER_TYPE_ERR,
};

// Per-instance i2c state information.
struct i2c_state {
    struct i2c_cfg cfg;
    I2C_TypeDef* i2c_reg_base;
    int32_t guard_tmr_id;

    uint8_t* msg_bfr;
    uint32_t msg_len;
    uint32_t msg_bytes_xferred;

    uint16_t dest_addr;

    bool reserved;
    enum states state;
    enum i2c_errors last_op_error;
    enum states last_op_error_state;
};

// Performance measurements for i2c. Currently these are common to all
// instances.  A future enhancement would be to make them per-instance.

enum i2c_u16_pms {
    CNT_RESERVE_FAIL,
    CNT_BUS_BUSY,
    CNT_GUARD_TMR,
    CNT_PEC_ERR,
    CNT_TIMEOUT,
    CNT_ACK_FAIL,
    CNT_BUS_ERR,
    CNT_INTR_UNEXPECT,

    NUM_U16_PMS
};

////////////////////////////////////////////////////////////////////////////////
// Private (static) function declarations
////////////////////////////////////////////////////////////////////////////////

static int32_t start_op(enum i2c_instance_id instance_id, uint32_t dest_addr,
                        uint8_t* msg_bfr, uint32_t msg_len,
                        enum states init_state);
static void i2c_interrupt(enum i2c_instance_id instance_id,
                          enum interrupt_type inter_type,
                          IRQn_Type irq_type);
static enum tmr_cb_action tmr_callback(int32_t tmr_id, uint32_t user_data);
static void handle_receive_addr(struct i2c_state* st);
static void handle_receive_rxne(struct i2c_state* st);
static void handle_receive_btf(struct i2c_state* st);
static void op_stop_success(struct i2c_state* st, bool set_stop);
static void op_stop_fail(struct i2c_state* st, enum i2c_errors error,
                         enum i2c_u16_pms pm);

static int32_t cmd_i2c_status(int32_t argc, const char** argv);
static int32_t cmd_i2c_test(int32_t argc, const char** argv);

////////////////////////////////////////////////////////////////////////////////
// Private (static) variables
////////////////////////////////////////////////////////////////////////////////

static struct i2c_state i2c_states[I2C_NUM_INSTANCES];

static int32_t log_level = LOG_DEBUG;

// Storage for performance measurements.
static uint16_t cnts_u16[NUM_U16_PMS];

// Names of performance measurements.
static const char* cnts_u16_names[NUM_U16_PMS] = {
    "i2c reserve fail",
    "i2c bus busy",
    "i2c guard tmr",
    "i2c pec",
    "i2c timeout",
    "i2c ack fail",
    "i2c bus error",
    "i2c unexpect intr",
};

// Data structure with console command info.
static struct cmd_cmd_info cmds[] = {
    {
        .name = "status",
        .func = cmd_i2c_status,
        .help = "Get module status, usage: i2c status",
    },
    {
        .name = "test",
        .func = cmd_i2c_test,
        .help = "Run test, usage: i2c test [<op> [<arg>]] (enter no op/arg for help)",
    }
};

// Data structure passed to cmd module for console interaction.
static struct cmd_client_info cmd_info = {
    .name = "i2c",
    .num_cmds = ARRAY_SIZE(cmds),
    .cmds = cmds,
    .log_level_ptr = &log_level,
    .num_u16_pms = NUM_U16_PMS,
    .u16_pms = cnts_u16,
    .u16_pm_names = cnts_u16_names,
};

////////////////////////////////////////////////////////////////////////////////
// Public (global) variables and externs
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Public (global) functions
////////////////////////////////////////////////////////////////////////////////

/*
 * @brief Get default i2c configuration.
 *
 * @param[in] instance_id Identifies the i2c instance.
 * @param[out] cfg The i2c configuration with defaults filled in.
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t i2c_get_def_cfg(enum i2c_instance_id instance_id, struct i2c_cfg* cfg)
{
    cfg->transaction_guard_time_ms = CONFIG_I2C_DFLT_TRANS_GUARD_TIME_MS;
    return 0;
}

/*
 * @brief Initialize i2c instance.
 *
 * @param[in] instance_id Identifies the i2c instance.
 * @param[in] cfg The i2c configuration. (FUTURE)
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function initializes a i2c module instance. Generally, it should not
 * access other modules as they might not have been initialized yet.  An
 * exception is the log module.
 */
int32_t i2c_init(enum i2c_instance_id instance_id, struct i2c_cfg* cfg)
{
    struct i2c_state* st;

    if (instance_id >= I2C_NUM_INSTANCES)
        return MOD_ERR_BAD_INSTANCE;

    if (cfg == NULL)
        return MOD_ERR_ARG;

    st = &i2c_states[instance_id];
    memset(st, 0, sizeof(*st));
    st->cfg = *cfg;

    switch (instance_id) {

#if CONFIG_I2C_1_PRESENT
        case I2C_INSTANCE_1:
            st->i2c_reg_base = I2C1;
            break;
#endif

#if CONFIG_I2C_2_PRESENT
        case I2C_INSTANCE_2:
            st->i2c_reg_base = I2C2;
            break;
#endif

#if CONFIG_I2C_3_PRESENT
        case I2C_INSTANCE_3:
            st->i2c_reg_base = I2C3;
            break;
#endif

        default:
            return MOD_ERR_BAD_INSTANCE;
    }
    return 0;
}

/*
 * @brief Start i2c instance.
 *
 * @param[in] instance_id Identifies the i2c instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function starts a i2c module instance, to enter normal operation.
 */
int32_t i2c_start(enum i2c_instance_id instance_id)
{
    struct i2c_state* st;
    IRQn_Type evt_irq_type;
    IRQn_Type err_irq_type;
    int32_t result;

    if (instance_id >= I2C_NUM_INSTANCES ||
        i2c_states[instance_id].i2c_reg_base == NULL)
        return MOD_ERR_BAD_INSTANCE;

    result = cmd_register(&cmd_info);
    if (result < 0) {
        log_error("i2c_start: cmd error %d\n", result);
        return MOD_ERR_RESOURCE;
    }

    st = &i2c_states[instance_id];

    st->guard_tmr_id = tmr_inst_get_cb(0, tmr_callback, (uint32_t)instance_id,
                                       false);
    if (st->guard_tmr_id < 0)
        return MOD_ERR_RESOURCE;

    LL_I2C_Disable(st->i2c_reg_base);
    DISABLE_ALL_INTERRUPTS(st);

    switch (instance_id) {
#if CONFIG_I2C_1_PRESENT
        case I2C_INSTANCE_1:
            evt_irq_type = I2C1_EV_IRQn;
            err_irq_type = I2C1_ER_IRQn;
            break;
#endif

#if CONFIG_I2C_2_PRESENT
        case I2C_INSTANCE_2:
            evt_irq_type = I2C2_EV_IRQn;
            err_irq_type = I2C2_ER_IRQn;
            break;
#endif

#if CONFIG_I2C_3_PRESENT
        case I2C_INSTANCE_3:
            evt_irq_type = I2C3_EV_IRQn;
            err_irq_type = I2C3_ER_IRQn;
            break;
#endif

        default:
            return MOD_ERR_BAD_INSTANCE;
    }
    NVIC_SetPriority(evt_irq_type,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    NVIC_EnableIRQ(evt_irq_type);
    NVIC_SetPriority(err_irq_type,
                     NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    NVIC_EnableIRQ(err_irq_type);

    return 0;
}

/*
 * @brief Run i2c instance.
 *
 * @param[in] instance_id Identifies the i2c instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * @note This function should not block.
 *
 * This function runs a i2c module instance, during normal operation.
 */
int32_t i2c_run(enum i2c_instance_id instance_id)
{
    return 0;
}

/*
 * @brief Reserve I2C bus.
 *
 * @param[in] instance_id Identifies the i2c instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t i2c_reserve(enum i2c_instance_id instance_id)
{
    if (instance_id >= I2C_NUM_INSTANCES ||
        i2c_states[instance_id].i2c_reg_base == NULL)
        return MOD_ERR_BAD_INSTANCE;
    if (i2c_states[instance_id].reserved) {
        INC_SAT_U16(cnts_u16[CNT_RESERVE_FAIL]);
        return MOD_ERR_RESOURCE;
    } else {
        i2c_states[instance_id].reserved = true;
    }
    return 0;
}

/*
 * @brief Release I2C bus.
 *
 * @param[in] instance_id Identifies the i2c instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t i2c_release(enum i2c_instance_id instance_id)
{
    if (instance_id >= I2C_NUM_INSTANCES ||
        i2c_states[instance_id].i2c_reg_base == NULL)
        return MOD_ERR_BAD_INSTANCE;
    i2c_states[instance_id].reserved = false;
    return 0;
}

/*
 * @brief Initiate I2C write.
 *
 * @param[in] instance_id Identifies the i2c instance.
 * @param[in] dest_Addr I2C destination address (not shifted).
 * @param[in] msg_bfr Buffer containing data bytes to send.
 * @param[in] msg_len Number of bytes in messages.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * @note In case of a return value of MOD_ERR_PERIPH, use i2c_get_error() to
 *       get more detailed error information.
 */
int32_t i2c_write(enum i2c_instance_id instance_id, uint32_t dest_addr,
                  uint8_t* msg_bfr, uint32_t msg_len)
{
    return start_op(instance_id, dest_addr, msg_bfr, msg_len,
                    STATE_MSTR_WR_GEN_START);
}

/*
 * @brief Initiate I2C read.
 *
 * @param[in] instance_id Identifies the i2c instance.
 * @param[in] dest_Addr I2C destination address (not shifted).
 * @param[in] msg_bfr Buffer to contain received data bytes.
 * @param[in] msg_len Number of bytes to receive.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * @note In case of a return value of MOD_ERR_PERIPH, use i2c_get_error() to
 *       get more detailed error information.
 */
int32_t i2c_read(enum i2c_instance_id instance_id, uint32_t dest_addr,
                 uint8_t* msg_bfr, uint32_t msg_len)
{
    return start_op(instance_id, dest_addr, msg_bfr, msg_len,
                    STATE_MSTR_RD_GEN_START);
}

/*
 * @brief Get detailed error information.
 *
 * @param[in] instance_id Identifies the i2c instance.
 *
 * @return I2C error code (enum i2c_errors).
 */
enum i2c_errors i2c_get_error(enum i2c_instance_id instance_id)
{
   if (instance_id >= I2C_NUM_INSTANCES ||
        i2c_states[instance_id].i2c_reg_base == NULL)
       return I2C_ERR_INVALID_INSTANCE;

   return i2c_states[instance_id].last_op_error;
}

/*
 * @brief Get status of operation.
 *
 * @param[in] instance_id Identifies the i2c instance.
 *
 * @return 0 if operation was successful, else a "MOD_ERR" value. See code for
 *         details.
 *
 * @note Instance should be held reserved until operation status is obtained.
 */
int32_t i2c_get_op_status(enum i2c_instance_id instance_id)
{
    struct i2c_state* st;
    int32_t rc;

    if (instance_id >= I2C_NUM_INSTANCES)
        return MOD_ERR_BAD_INSTANCE;

    st = &i2c_states[instance_id];
    if (!st->reserved)
        rc = MOD_ERR_NOT_RESERVED;
    else if (st->state == STATE_IDLE)
        rc = st->last_op_error == I2C_ERR_NONE ? 0 : MOD_ERR_PERIPH;
    else
        rc = MOD_ERR_OP_IN_PROG;
    return rc;
}

/*
 * @brief Check if the bus is busy.
 *
 * @param[in] instance_id Identifies the i2c instance.
 *
 * @return 0 if not busy, 1 if busy, else a "MOD_ERR" value (< 0).
 */
int32_t i2c_bus_busy(enum i2c_instance_id instance_id)
{
   if (instance_id >= I2C_NUM_INSTANCES ||
        i2c_states[instance_id].i2c_reg_base == NULL)
        return MOD_ERR_BAD_INSTANCE;

   return LL_I2C_IsActiveFlag_BUSY(i2c_states[instance_id].i2c_reg_base);
}

#if CONFIG_I2C_1_PRESENT

void I2C1_EV_IRQHandler(void)
{
    i2c_interrupt(I2C_INSTANCE_1, INTER_TYPE_EVT, I2C1_EV_IRQn);
}

void I2C1_ER_IRQHandler(void)
{
    i2c_interrupt(I2C_INSTANCE_1, INTER_TYPE_ERR, I2C1_ER_IRQn);
}

#endif

#if CONFIG_I2C_2_PRESENT

void I2C2_EV_IRQHandler(void)
{
    i2c_interrupt(I2C_INSTANCE_2, INTER_TYPE_EVT, I2C2_EV_IRQn);
}

void I2C2_ER_IRQHandler(void)
{
    i2c_interrupt(I2C_INSTANCE_2, INTER_TYPE_ERR, I2C2_ER_IRQn);
}

#endif

#if CONFIG_I2C_3_PRESENT

void I2C3_EV_IRQHandler(void)
{
    i2c_interrupt(I2C_INSTANCE_3, INTER_TYPE_EVT, I2C3_EV_IRQn);
}

void I2C3_ER_IRQHandler(void)
{
    i2c_interrupt(I2C_INSTANCE_3, INTER_TYPE_ERR, I2C3_ER_IRQn);
}

#endif

////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////

/*
 * @brief Start operation (common parts of read and write).
 *
 * @param[in] instance_id Identifies the i2c instance.
 * @param[in] dest_Addr I2C destination address (not shifted).
 * @param[in] msg_bfr Buffer containing data bytes to send.
 * @param[in] msg_len Number of bytes in messages.
 * @param[in] init_state Initial state if start operation is successful.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
static int32_t start_op(enum i2c_instance_id instance_id, uint32_t dest_addr,
                        uint8_t* msg_bfr, uint32_t msg_len,
                        enum states init_state)
{
    struct i2c_state* st;

    log_trace("op_start state=%d msg_len=%d dest_addr=0x%02x\n", init_state,
              msg_len, dest_addr);

    if (instance_id >= I2C_NUM_INSTANCES ||
        i2c_states[instance_id].i2c_reg_base == NULL)
        return MOD_ERR_BAD_INSTANCE;

    st = &i2c_states[instance_id];
    if (!st->reserved)
        return MOD_ERR_NOT_RESERVED;

    if (st->state != STATE_IDLE)
        return MOD_ERR_STATE;

    if (LL_I2C_IsActiveFlag_BUSY(st->i2c_reg_base)) {
        INC_SAT_U16(cnts_u16[CNT_BUS_BUSY]);
        st->last_op_error = I2C_ERR_BUS_BUSY;
        st->last_op_error_state = STATE_IDLE;
        return MOD_ERR_PERIPH;
    }

    tmr_inst_start(st->guard_tmr_id, 100);

    st->dest_addr = dest_addr;
    st->msg_bfr = msg_bfr;
    st->msg_len = msg_len;
    st->msg_bytes_xferred = 0;

    st->last_op_error = I2C_ERR_NONE;
    st->last_op_error_state = STATE_IDLE;

    st->state = init_state;

    LL_I2C_Enable(st->i2c_reg_base);
    LL_I2C_DisableBitPOS(st->i2c_reg_base);
    LL_I2C_GenerateStartCondition(st->i2c_reg_base);

    ENABLE_ALL_INTERRUPTS(st);

    return 0;
}

/*
 * @brief Process I2C interrupt.
 *
 * @param[in] instance_id Identifies the i2c instance.
 * @param[in] inter_type Interrupt type (event or error).
 * @param[in] irq_type Interrupt number, needed to disable it.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * @note The "unused" attribute allows this file to be compiled without warnings
 *       even if no I2C instances are configured.
 */
__attribute__((unused)) 
static void i2c_interrupt(enum i2c_instance_id instance_id,
                          enum interrupt_type inter_type,
                          IRQn_Type irq_type)
{
    struct i2c_state* st;
    uint16_t sr1;

    if (instance_id >= I2C_NUM_INSTANCES)
        return;

    st = &i2c_states[instance_id];

    // If instance is not initialized, we should not get an interrupt, but for
    // safety just disable it.

    if (st->i2c_reg_base == NULL) {
        NVIC_DisableIRQ(irq_type);
        return;
    }
    sr1 = st->i2c_reg_base->SR1;

    log_trace("i2c_interrupt state=%d xferred=%lu sr1=0x%04x\n", st->state,
              st->msg_bytes_xferred, sr1);

    if (inter_type == INTER_TYPE_EVT)
    {
        uint32_t sr1_handled_mask = 0;

        switch (st->state) {
            case STATE_MSTR_WR_GEN_START:
                if (sr1 & LL_I2C_SR1_SB) {
                    // SB is cleared by reading SR1 followed by writing address
                    // to DR.

                    st->i2c_reg_base->DR = st->dest_addr << 1;
                    st->state = STATE_MSTR_WR_SENDING_ADDR;
                }
                sr1_handled_mask = LL_I2C_SR1_SB;
                break;

            case STATE_MSTR_WR_SENDING_ADDR:
                sr1_handled_mask = LL_I2C_SR1_ADDR;
                if (sr1 & LL_I2C_SR1_ADDR) {
                    // Clear ADDR flag (SR1 already read).
                    (void)st->i2c_reg_base->SR2;

                    // We either check these bits, or want to ignore them.
                    sr1_handled_mask |= LL_I2C_SR1_TXE | LL_I2C_SR1_BTF;

                    if (st->msg_len == 0) {
                        st->state = STATE_IDLE;
                        op_stop_success(st, true);
                    } else {
                        st->state = STATE_MSTR_WR_SENDING_DATA;
                        if (sr1 & (LL_I2C_SR1_TXE | LL_I2C_SR1_BTF)) {
                            st->i2c_reg_base->DR =
                                st->msg_bfr[st->msg_bytes_xferred++];
                        }
                    }
                }
                break;

            case STATE_MSTR_WR_SENDING_DATA:
                if (sr1 & (LL_I2C_SR1_TXE | LL_I2C_SR1_BTF)) {
                    if (st->msg_bytes_xferred < st->msg_len) {
                        st->i2c_reg_base->DR =
                            st->msg_bfr[st->msg_bytes_xferred++];
                    } else {

                        // No more bytes to write. We wait until we get BTF, to
                        // ensure we detect any errors (e.g. NACK) for the last
                        // transmitted byte.

                        if (sr1 & LL_I2C_SR1_BTF) {
                            op_stop_success(st, true);
                            LL_I2C_EnableIT_ERR(st->i2c_reg_base); // TODO TMP
                        } else {
                            // Prevent further TXE interrupts.
                            LL_I2C_DisableIT_BUF(st->i2c_reg_base);
                        }
                    }
                }
                sr1_handled_mask = LL_I2C_SR1_TXE | LL_I2C_SR1_BTF;
                break;

            case STATE_MSTR_RD_GEN_START:
                if (sr1 & LL_I2C_SR1_SB) {

                    // SB is cleared by reading SR1 followed by writing address
                    // to DR.

                    st->i2c_reg_base->DR = (st->dest_addr << 1) + 1;
                    st->state = STATE_MSTR_RD_SENDING_ADDR;
                }
                sr1_handled_mask = LL_I2C_SR1_SB;
                break;

            case STATE_MSTR_RD_SENDING_ADDR:
                if (sr1 & LL_I2C_SR1_ADDR) {
                    handle_receive_addr(st);
                }
                sr1_handled_mask = LL_I2C_SR1_ADDR;
                break;

            case STATE_MSTR_RD_READING_DATA:
                if (sr1 & (I2C_SR1_RXNE)) {
                    handle_receive_rxne(st);
                }
                if (sr1 & (I2C_SR1_BTF)) {
                    handle_receive_btf(st);
                }
                sr1_handled_mask = LL_I2C_SR1_RXNE | LL_I2C_SR1_BTF;
                break;

            default:
                break;
        }

        // Check for unexpected events.
        sr1 &= ~sr1_handled_mask;
        if (sr1 & INTERRUPT_EVT_MASK)
            op_stop_fail(st, I2C_ERR_INTR_UNEXPECT, CNT_INTR_UNEXPECT);

    } else if (inter_type == INTER_TYPE_ERR) {
        enum i2c_errors i2c_error = I2C_ERR_INTR_UNEXPECT;
        enum i2c_u16_pms pm_ctr = NUM_U16_PMS;

        // Clear errors.
        st->i2c_reg_base->SR1 &= ~(sr1 & INTERRUPT_ERR_MASK);

        // Record and report error.
        if (sr1 & I2C_SR1_TIMEOUT) {
            pm_ctr = CNT_TIMEOUT;
            i2c_error = I2C_ERR_TIMEOUT;
        }
        if (sr1 & I2C_SR1_PECERR) {
            pm_ctr = CNT_PEC_ERR;
            i2c_error = I2C_ERR_PEC;
        }
        if (sr1 & I2C_SR1_AF) {
            pm_ctr = CNT_ACK_FAIL;
            i2c_error = I2C_ERR_ACK_FAIL;
        }
        if (sr1 & I2C_SR1_BERR) {
            pm_ctr = CNT_BUS_ERR;
            i2c_error = I2C_ERR_BUS_ERR;
        }
        op_stop_fail(st, i2c_error, pm_ctr);
    }
}

/*
 * @brief Timer callback
 *
 * @param[in] tmr_id The timer ID (not used).
 * @param[in] user_data User data for the timer (is i2c instance).
 *
 * @return Timer disposition (always TMR_CB_NONE since timers are one-shot).
 */
static enum tmr_cb_action tmr_callback(int32_t tmr_id, uint32_t user_data)
{
    struct i2c_state* st;
    enum i2c_instance_id instance_id = (enum i2c_instance_id)user_data;

    log_trace("i2c tmr_callback\n");
    if (instance_id >= I2C_NUM_INSTANCES ||
        i2c_states[instance_id].i2c_reg_base == NULL)
        return TMR_CB_NONE;

    st = &i2c_states[instance_id];
    op_stop_fail(st, I2C_ERR_GUARD_TMR, CNT_GUARD_TMR);

    return TMR_CB_NONE;
}

/*
 * @brief Handle receive ADDR event.
 *
 * @param[in] st Pointer to struct st_state.
 */
static void handle_receive_addr(struct i2c_state* st)
{
    switch (st->msg_len) {
        case 0:
            // A zero byte receive is really just a "ping" of the address,
            // so if we get an ADDR event, we are done.

            // Clear ADDR flag (SR1 already read).
            (void)st->i2c_reg_base->SR2;

            op_stop_success(st, true);
            break;

        case 1:
            LL_I2C_AcknowledgeNextData(st->i2c_reg_base, LL_I2C_NACK);

            // Clear ADDR flag (SR1 already read).
            (void)st->i2c_reg_base->SR2;

            LL_I2C_GenerateStopCondition(st->i2c_reg_base);
            break;

        case 2:
            LL_I2C_AcknowledgeNextData(st->i2c_reg_base, LL_I2C_NACK);
            LL_I2C_EnableBitPOS(st->i2c_reg_base);

            // Clear ADDR flag (SR1 already read).
            (void)st->i2c_reg_base->SR2;
            break;

        default:
            LL_I2C_AcknowledgeNextData(st->i2c_reg_base, LL_I2C_ACK);
            LL_I2C_DisableBitPOS(st->i2c_reg_base);

            // Clear ADDR flag (SR1 already read).
            (void)st->i2c_reg_base->SR2;
            break;
    }
    if (st->msg_len != 0)
        st->state = STATE_MSTR_RD_READING_DATA;
}

/*
 * @brief Handle receive RXNE event.
 *
 * @param[in] st Pointer to struct st_state.
 */
static void handle_receive_rxne(struct i2c_state* st)
{
    log_trace("handle rxne left=%lu\n", st->msg_len - st->msg_bytes_xferred);
    switch (st->msg_len - st->msg_bytes_xferred) {
        case 1:
            // Seems like this case should never happen.
            st->msg_bfr[st->msg_bytes_xferred++] = st->i2c_reg_base->DR;
            op_stop_success(st, true);
            break;

        case 2:
        case 3:
            // At this point, we only use BTF, to force byte queuing in the
            // device, per the reference manual. Note that stopping the
            // interrupt will not stop the RXNE bit from being set in SR1.

            LL_I2C_DisableIT_BUF(st->i2c_reg_base);
            break;

        default:
            st->msg_bfr[st->msg_bytes_xferred++] = st->i2c_reg_base->DR;
            break;
    }
}

/*
 * @brief Handle receive BTF event.
 *
 * @param[in] st Pointer to struct st_state.
 */
static void handle_receive_btf(struct i2c_state* st)
{
    log_trace("handle btf left=%lu\n", st->msg_len - st->msg_bytes_xferred);
    switch (st->msg_len - st->msg_bytes_xferred) {
        case 0:
        case 1:
            op_stop_fail(st, I2C_ERR_INTR_UNEXPECT, CNT_INTR_UNEXPECT);
            break;

        case 2:
            LL_I2C_GenerateStopCondition(st->i2c_reg_base);
            st->msg_bfr[st->msg_bytes_xferred++] = st->i2c_reg_base->DR;
            st->msg_bfr[st->msg_bytes_xferred++] = st->i2c_reg_base->DR;
            op_stop_success(st, false);
            break;

        case 3:
            LL_I2C_AcknowledgeNextData(st->i2c_reg_base, LL_I2C_NACK);
            st->msg_bfr[st->msg_bytes_xferred++] = st->i2c_reg_base->DR;
            break;

        default:
            st->msg_bfr[st->msg_bytes_xferred++] = st->i2c_reg_base->DR;
            break;
    }
}

/*
 * @brief Send stop and handle successful operation.
 *
 * @param[in] st Pointer to struct st_state.
 */
static void op_stop_success(struct i2c_state* st, bool set_stop)
{
    log_trace("op_stop_success state=%d\n", st->state);
    DISABLE_ALL_INTERRUPTS(st);
    if (set_stop)
        LL_I2C_GenerateStopCondition(st->i2c_reg_base);
    tmr_inst_start(st->guard_tmr_id, 0);
    LL_I2C_Disable(st->i2c_reg_base);
    st->state = STATE_IDLE;
}

/*
 * @brief Send stop and handle failed operation.
 *
 * @param[in] st Pointer to struct st_state.
 * @param[in] error The I2C-level error.
 */
static void op_stop_fail(struct i2c_state* st, enum i2c_errors error,
                         enum i2c_u16_pms pm)
{
    // The recovery actions are not clear, for example whether we should be
    // clearing CR1 PE. We just do it.
    log_trace("op_stop_fail state=%d error=%d pm=%d\n", st->state, error, pm);
    DISABLE_ALL_INTERRUPTS(st);
    LL_I2C_GenerateStopCondition(st->i2c_reg_base);
    tmr_inst_start(st->guard_tmr_id, 0);
    LL_I2C_Disable(st->i2c_reg_base);

    // Only record the first error in a transaction.
    if (st->last_op_error == I2C_ERR_NONE) {
        st->last_op_error = error;
        st->last_op_error_state = st->state;
    }
    if (pm < NUM_U16_PMS)
        INC_SAT_U16(cnts_u16[pm]);
    st->state = STATE_IDLE;
}

/*
 * @brief Console command function for "i2c status".
 *
 * @param[in] argc Number of arguments, including "i2c"
 * @param[in] argv Argument values, including "i2c"
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: i2c status
 */
static int32_t cmd_i2c_status(int32_t argc, const char** argv)
{
    uint32_t idx;
    struct i2c_state* st;

    printc("   Rsr Sta Dest Msg Byte I2C Err  Register\n"
           "ID vrd te  Addr Len Xfrd Err Sta  BaseAddr\n"
           "-- --- --- ---- --- ---- --- --- ----------\n");
    for (idx = 0, st = i2c_states; idx < I2C_NUM_INSTANCES; idx++, st++) {
        printc("%2lu %3d %3d 0x%02x %3lu %4lu %3d %3d %10p\n",
               idx, st->reserved, st->state, st->dest_addr, st->msg_len,
               st->msg_bytes_xferred, st->last_op_error,
               st->last_op_error_state, st->i2c_reg_base);
    }
    return 0;
}

/*
 * @brief Console command function for "i2c test".
 *
 * @param[in] argc Number of arguments, including "i2c"
 * @param[in] argv Argument values, including "i2c"
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: i2c test [<op> [<arg>]]
 */
static int32_t cmd_i2c_test(int32_t argc, const char** argv)
{
    struct cmd_arg_val arg_vals[5];
    int32_t rc = 0;
    int32_t idx;
#define MAX_MSG_LEN 7
    static uint32_t msg_len;
    enum i2c_instance_id instance_id = 0;
    static uint8_t msg_bfr[MAX_MSG_LEN];

    // Handle help case.
    if (argc == 2) {
        printc("Test operations and param(s) are as follows:\n"
               "  Reserve I2C, usage: i2c test reserve <instance-id>\n"
               "  Release I2C, usage: i2c test reserve <instance-id>\n");
        printc("  Start write, usage: i2c test write <instance-id> <addr> [<bytes> ...]\n"
               "  Start read, usage: i2c test read <instance-id> <addr> <num-bytes>\n"
               "  Get op status/error, usage: i2c test status <instance-id>\n");
        printc("  Bus busy, usage: i2c test busy <instance-id>\n"
               "  Print msg buffer, usage: i2c test msg <instance-id>\n");
        return 0;
    }

    // Get instance ID (except for msg option).
    if (strcasecmp(argv[2], "msg") != 0) {
        if (cmd_parse_args(argc-3, argv+3, "u+", arg_vals) != 1) {
            printc("Can't get instance ID\n");
            return MOD_ERR_BAD_CMD;
        }

        instance_id = (enum i2c_instance_id)arg_vals[0].val.u;
        if (instance_id >= I2C_NUM_INSTANCES ||
            i2c_states[instance_id].i2c_reg_base == NULL) {
            printc("Bad instance\n");
            return MOD_ERR_BAD_INSTANCE;
        }
    }

    if (strcasecmp(argv[2], "reserve") == 0) {
        rc = i2c_reserve(instance_id);
    } else if (strcasecmp(argv[2], "release") == 0) {
        rc = i2c_release(instance_id);
    } else if (strcasecmp(argv[2], "write") == 0) {
        rc = cmd_parse_args(argc-4, argv+4, "u[u[u[u[u]]]]", arg_vals);
        if (rc < 1) {
            return MOD_ERR_BAD_CMD;
        }
        for (idx = 1; idx < rc; idx++)
            msg_bfr[idx-1] = arg_vals[idx].val.u;
        rc = i2c_write(instance_id, arg_vals[0].val.u, msg_bfr, rc-1);
    } else if (strcasecmp(argv[2], "read") == 0) {
        rc = cmd_parse_args(argc-4, argv+4, "uu", arg_vals);
        if (rc < 2) {
            printc("Invalid command rc=%ld\n", rc);
            return MOD_ERR_BAD_CMD;
        }
        if (arg_vals[1].val.u > MAX_MSG_LEN) {
            printc("Message length limited to %d\n", MAX_MSG_LEN);
            return MOD_ERR_ARG;
        }
        msg_len = arg_vals[1].val.u;
        rc = i2c_read(instance_id, arg_vals[0].val.u, msg_bfr, msg_len);
    } else if (strcasecmp(argv[2], "status") == 0) {
        printc("op_status=%ld error=%d\n", i2c_get_op_status(instance_id),
               i2c_get_error(instance_id));
        goto done;
    } else if (strcasecmp(argv[2], "busy") == 0) {
        rc = i2c_bus_busy(instance_id);
    } else if (strcasecmp(argv[2], "msg") == 0) {
        for (idx = 0; idx < msg_len; idx++)
            printc("%02x ", msg_bfr[idx]);
        printc("\n");
    } else {
        printc("Invalid operation '%s'\n", argv[2]);
        return MOD_ERR_BAD_CMD;
    }
    printc("Return code %ld\n", rc);
done:
    return 0;
}
#endif
