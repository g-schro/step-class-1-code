/*
 * @brief Implementation of dio module.
 *
 * @todo TODO init_value
 *
 * This module provides access to discrete inputs and outputs, sometimes
 * referred to as digital inputs and outputs.
 *
 * During configuration, the user must specify the set of inputs and outputs and
 * their characteristics.
 *
 * The following console commands are provided:
 * > dio status
 * > dio get
 * > dio set
 * See code for details.
 *
 * Currently, defintions from the STMicroelectronics Low Level (LL) device
 * library are used for some configuration paramters. A future enhancment would
 * be to define all configuration parameters in this module, so that user code
 * is more portable.
 * 
 * MIT License
 * 
 * Copyright (c) 2021 Eugene R Schroed3er
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
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include CONFIG_STM32_LL_BUS_HDR

#include "cmd.h"
#include "console.h"
#include "dio.h"
#include "log.h"
#include "module.h"

////////////////////////////////////////////////////////////////////////////////
// Common macros
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Type definitions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Private (static) function declarations
////////////////////////////////////////////////////////////////////////////////

static int32_t cmd_dio_status(int32_t argc, const char** argv);
static int32_t cmd_dio_get(int32_t argc, const char** argv);
static int32_t cmd_dio_set(int32_t argc, const char** argv);

static const char* gpio_pin_mode_to_str(uint32_t mode);
static const char* gpio_output_type_to_str(uint32_t mode);
static const char* gpio_pin_speed_to_str(uint32_t mode);
static const char* gpio_pull_up_to_str(uint32_t mode);

////////////////////////////////////////////////////////////////////////////////
// Private (static) variables
////////////////////////////////////////////////////////////////////////////////

static struct dio_cfg* cfg;

static struct cmd_cmd_info cmds[] = {
    {
        .name = "status",
        .func = cmd_dio_status,
        .help = "Get module status, usage: dio status [port <port-letter>]",
    },
    {
        .name = "get",
        .func = cmd_dio_get,
        .help = "Get input value, usage: dio get <input-name>",
    },
    {
        .name = "set",
        .func = cmd_dio_set,
        .help = "Set output value, usage: dio set <output-name> {0|1}",
    },
};

static int32_t log_level = LOG_DEFAULT;

static struct cmd_client_info cmd_info = {
    .name = "dio",
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
 * @brief Initialize dio module instance.
 *
 * @param[in] cfg The dio configuration.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function initializes the dio singleton module. Generally, it should not
 * access other modules as they might not have been initialized yet. An
 * exception is the log module.
 */
int32_t dio_init(struct dio_cfg* _cfg)
{
    uint32_t idx;
    const struct dio_in_info* dii;
    const struct dio_out_info* doi;

    cfg = _cfg;

    for (idx = 0; idx < cfg->num_inputs; idx++) {
        dii = &cfg->inputs[idx];
        LL_GPIO_SetPinPull(dii->port, dii->pin, dii->pull);
        LL_GPIO_SetPinMode(dii->port, dii->pin, LL_GPIO_MODE_INPUT);
    }
    for (idx = 0; idx < cfg->num_outputs; idx++) {
        doi = &cfg->outputs[idx];
        LL_GPIO_SetPinSpeed(doi->port, doi->pin, doi->speed);
        LL_GPIO_SetPinOutputType(doi->port, doi->pin,  doi->output_type);
        LL_GPIO_SetPinPull(doi->port, doi->pin, doi->pull);
        LL_GPIO_SetPinMode(doi->port, doi->pin, LL_GPIO_MODE_OUTPUT);
    }
    return 0;
}

/*
 * @brief Start dio module instance.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * This function starts the dio singleton module, to enter normal operation.
 */
int32_t dio_start(void)
{
    int32_t result;

    result = cmd_register(&cmd_info);
    if (result < 0) {
        log_error("dio_start: cmd error %d\n", result);
        return MOD_ERR_RESOURCE;
    }
    return 0;
}

/*
 * @brief Get value of discrete input.
 *
 * @param[in] din_idx Discrete input index per module configuration.
 *
 * @return Input state (0/1), else a "MOD_ERR" value (< 0). See code for
 *         details.
 */
int32_t dio_get(uint32_t din_idx)
{
    if (din_idx >= cfg->num_inputs)
        return MOD_ERR_ARG;
    return LL_GPIO_IsInputPinSet(cfg->inputs[din_idx].port,
                                 cfg->inputs[din_idx].pin) ^
        cfg->inputs[din_idx].invert;
}

/*
 * @brief Get value of discrete output.
 *
 * @param[in] dout_idx Discrete output index per module configuration.
 *
 * @return Output state (0/1), else a "MOD_ERR" value (< 0). See code for
 *         details.
 */
int32_t dio_get_out(uint32_t dout_idx)
{
    if (dout_idx >= cfg->num_outputs)
        return MOD_ERR_ARG;

    return LL_GPIO_IsOutputPinSet(cfg->outputs[dout_idx].port,
                                  cfg->outputs[dout_idx].pin) ^
        cfg->outputs[dout_idx].invert;
}

/*
 * @brief Set value of discrete output.
 *
 * @param[in] dout_idx Discrete output index per module configuration.
 * @param[in] value Output value 0/1.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t dio_set(uint32_t dout_idx, uint32_t value)
{
    if (dout_idx >= cfg->num_outputs)
        return MOD_ERR_ARG;
    if (value ^ cfg->outputs[dout_idx].invert) {
        LL_GPIO_SetOutputPin(cfg->outputs[dout_idx].port,
                             cfg->outputs[dout_idx].pin);
    } else {
        LL_GPIO_ResetOutputPin(cfg->outputs[dout_idx].port,
                               cfg->outputs[dout_idx].pin);
    }
    return 0;
}

/*
 * @brief Get number of discrete inputs.
 *
 * @return Return number of inputs (non-negative) for success, else a "MOD_ERR"
 *         value. See code for details.
 */
int32_t dio_get_num_in(void)
{
    return cfg == NULL ? MOD_ERR_RESOURCE : cfg->num_inputs;
}

/*
 * @brief Get number of discrete output.
 *
 * @return Return number of outputs (non-negative) for success, else a "MOD_ERR"
 *         value. See code for details.
 */
int32_t dio_get_num_out(void)
{
    return cfg == NULL ? MOD_ERR_RESOURCE : cfg->num_outputs;
}

/*
 * @brief Run-time configuration of discrete output(s) on a port.
 *
 * @param[in] cfg Configuration for one more more pins.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * @note This function simply modifies GPIO registers. There is no
 *       management for this I/O.
 */
int32_t dio_cfg_out(struct dio_out_rt_cfg* cfg)
{
    uint32_t idx;

    if (cfg == NULL || cfg->port == NULL)
        return MOD_ERR_ARG;

    // The LL APIs take a bit mask to specify the pin(s), but some only allow a
    // single bit to be set. We always use a single bit.

    for (idx = 0; idx < DIO_NUM_PINS_PER_PORT; idx++) {
        uint32_t pin_mask = cfg->pin_mask & (1 << idx);
        if (pin_mask != 0) {
            LL_GPIO_SetPinSpeed(cfg->port, pin_mask, cfg->speed);
            LL_GPIO_SetPinOutputType(cfg->port, pin_mask,  cfg->output_type);
            LL_GPIO_SetPinPull(cfg->port, pin_mask, cfg->pull);
            LL_GPIO_SetPinMode(cfg->port, pin_mask, LL_GPIO_MODE_OUTPUT);
        }
    }
    return 0;
}

/*
 * @brief Set one or more output bits on a port.
 *
 * @param[in] port The GPIO port.
 * @param[in] pin_mask The pin bit mask.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t dio_set_outputs(dio_port* const port, uint32_t pin_mask)
{
    if (port == NULL)
        return MOD_ERR_ARG;

    LL_GPIO_SetOutputPin(port, pin_mask);
    return 0;
}

/*
 * @brief Reset (clear) one or more output bits on a port.
 *
 * @param[in] port The GPIO port.
 * @param[in] pin_mask The pin bit mask.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 */
int32_t dio_reset_outputs(dio_port* const port, uint32_t pin_mask)
{
    if (port == NULL)
        return MOD_ERR_ARG;

    LL_GPIO_ResetOutputPin(port, pin_mask);
    return 0;
}

/*
 * @brief Set and reset (clear) one or more output bits on a port.
 *
 * @param[in] port The GPIO port.
 * @param[in] set_mask The pin bit set mask.
 * @param[in] reset_mask The pin bit reset mask.
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * @note If there are both "set" bits and "reset" bits, interrupts are
 * disabled so the sets/resets are an atomic operation.
 */
int32_t dio_set_reset_outputs(dio_port* const port, uint32_t set_mask,
                              uint32_t reset_mask)
{
    CRIT_STATE_VAR;

    if (port == NULL)
        return MOD_ERR_ARG;

    if (set_mask && reset_mask)
        CRIT_BEGIN_NEST(); 

    LL_GPIO_SetOutputPin(port, set_mask);
    LL_GPIO_ResetOutputPin(port, reset_mask);

    if (set_mask && reset_mask)
        CRIT_END_NEST();

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Private (static) functions
////////////////////////////////////////////////////////////////////////////////

/*
 * @brief Console command function for "dio status".
 *
 * @param[in] argc Number of arguments, including "dio".
 * @param[in] argv Argument values, including "dio".
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: dio status [port <port-letter>]
 */
static int32_t cmd_dio_status(int32_t argc, const char** argv)
{
    uint32_t idx;

    if (argc == 4 && strcasecmp(argv[2], "port") == 0) {
        const char* port_name_param = argv[3];
        struct port_info
        {
            GPIO_TypeDef* port;
            const char* name;
            const uint32_t clk_enable_mask;
        } ports[] = {

#if CONFIG_DIO_TYPE == 1

#ifdef GPIOA
            {GPIOA, "A", LL_AHB1_GRP1_PERIPH_GPIOA},
#endif
#ifdef GPIOB
            {GPIOB, "B", LL_AHB1_GRP1_PERIPH_GPIOB},
#endif
#ifdef GPIOC
            {GPIOC, "C", LL_AHB1_GRP1_PERIPH_GPIOC},
#endif
#ifdef GPIOD
            {GPIOD, "D", LL_AHB1_GRP1_PERIPH_GPIOD},
#endif
#ifdef GPIOE
            {GPIOE, "E", LL_AHB1_GRP1_PERIPH_GPIOE},
#endif
#ifdef GPIOF
            {GPIOF, "F", LL_AHB1_GRP1_PERIPH_GPIOF},
#endif
#ifdef GPIOG
            {GPIOG, "G", LL_AHB1_GRP1_PERIPH_GPIOG},
#endif
#ifdef GPIOH
            {GPIOH, "H", LL_AHB1_GRP1_PERIPH_GPIOH},
#endif
#ifdef GPIOI
            {GPIOI, "I", LL_AHB1_GRP1_PERIPH_GPIOI},
#endif
#ifdef GPIOJ
            {GPIOJ, "J", LL_AHB1_GRP1_PERIPH_GPIOJ},
#endif
#ifdef GPIOK
            {GPIOK, "K", LL_AHB1_GRP1_PERIPH_GPIOK},
#endif

#elif CONFIG_DIO_TYPE == 2

#ifdef GPIOA
            {GPIOA, "A", LL_AHB2_GRP1_PERIPH_GPIOA},
#endif
#ifdef GPIOB
            {GPIOB, "B", LL_AHB2_GRP1_PERIPH_GPIOB},
#endif
#ifdef GPIOC
            {GPIOC, "C", LL_AHB2_GRP1_PERIPH_GPIOC},
#endif
#ifdef GPIOD
            {GPIOD, "D", LL_AHB2_GRP1_PERIPH_GPIOD},
#endif
#ifdef GPIOE
            {GPIOE, "E", LL_AHB2_GRP1_PERIPH_GPIOE},
#endif
#ifdef GPIOF
            {GPIOF, "F", LL_AHB2_GRP1_PERIPH_GPIOF},
#endif
#ifdef GPIOG
            {GPIOG, "G", LL_AHB2_GRP1_PERIPH_GPIOG},
#endif
#ifdef GPIOH
            {GPIOH, "H", LL_AHB2_GRP1_PERIPH_GPIOH},
#endif
#ifdef GPIOI
            {GPIOI, "I", LL_AHB2_GRP1_PERIPH_GPIOI},
#endif
#ifdef GPIOJ
            {GPIOJ, "J", LL_AHB2_GRP1_PERIPH_GPIOJ},
#endif
#ifdef GPIOK
            {GPIOK, "K", LL_AHB2_GRP1_PERIPH_GPIOK},
#endif

#elif CONFIG_DIO_TYPE == 3

#ifdef GPIOA
            {GPIOA, "A", LL_APB2_GRP1_PERIPH_GPIOA},
#endif
#ifdef GPIOB
            {GPIOB, "B", LL_APB2_GRP1_PERIPH_GPIOB},
#endif
#ifdef GPIOC
            {GPIOC, "C", LL_APB2_GRP1_PERIPH_GPIOC},
#endif
#ifdef GPIOD
            {GPIOD, "D", LL_APB2_GRP1_PERIPH_GPIOD},
#endif
#ifdef GPIOE
            {GPIOE, "E", LL_APB2_GRP1_PERIPH_GPIOE},
#endif
#ifdef GPIOF
            {GPIOF, "F", LL_APB2_GRP1_PERIPH_GPIOF},
#endif
#ifdef GPIOG
            {GPIOG, "G", LL_APB2_GRP1_PERIPH_GPIOG},
#endif
#ifdef GPIOH
            {GPIOH, "H", LL_APB2_GRP1_PERIPH_GPIOH},
#endif
#ifdef GPIOI
            {GPIOI, "I", LL_APB2_GRP1_PERIPH_GPIOI},
#endif
#ifdef GPIOJ
            {GPIOJ, "J", LL_APB2_GRP1_PERIPH_GPIOJ},
#endif
#ifdef GPIOK
            {GPIOK, "K", LL_APB2_GRP1_PERIPH_GPIOK},
#endif

#endif // CONFIG_DIO_TYPE

        };
        for (idx = 0; idx < ARRAY_SIZE(ports); idx++) {
            struct port_info* port = &ports[idx];
            uint32_t pin_idx;
            if (strcasecmp(port->name, port_name_param) != 0)
                continue;
            printc("Port %s:", port->name);
            if (!LL_AHB1_GRP1_IsEnabledClock(port->clk_enable_mask)) {
                printc(" Clock not enabled\n");
                continue;
            }
            printc("\nPin In Out Mode AF OT PS PP\n--- -- --- ---- -- -- -- --\n");
            for (pin_idx = 0; pin_idx < 16; pin_idx++) {
                printc("%3lu %2lu %3lu %4s %2lu %2s %2s %2s\n",
                       pin_idx, 
                       LL_GPIO_IsInputPinSet(port->port, 1 << pin_idx),
                       LL_GPIO_IsOutputPinSet(port->port, 1 << pin_idx),
                       gpio_pin_mode_to_str(LL_GPIO_GetPinMode(port->port, 1 << pin_idx)),
#if CONFIG_DIO_TYPE == 3
                       0LU,
#else
                       pin_idx <= 7 ?
                       LL_GPIO_GetAFPin_0_7(port->port, 1 << pin_idx) :
                       LL_GPIO_GetAFPin_8_15(port->port, 1 << pin_idx),
#endif
                       gpio_output_type_to_str(LL_GPIO_GetPinOutputType(port->port, 1 << pin_idx)),
                       gpio_pin_speed_to_str(LL_GPIO_GetPinSpeed(port->port, 1 << pin_idx)),
                       gpio_pull_up_to_str(LL_GPIO_GetPinPull(port->port, 1 << pin_idx)));
            }
            break;
        }
        return 0;
    }
    else if (argc != 2) {
        printc("Invalid arguments\n");
        return MOD_ERR_ARG;
    }
    printc("Inputs:\n");
    for (idx = 0; idx < cfg->num_inputs; idx++)
        printc("  %2lu: %s = %ld\n", idx, cfg->inputs[idx].name, dio_get(idx));
    

    printc("Outputs:\n");
    for (idx = 0; idx < cfg->num_outputs; idx++)
        printc("  %2lu: %s = %ld\n", idx, cfg->outputs[idx].name,
               dio_get_out(idx));

    return 0;
}

/*
 * @brief Console command function for "dio get".
 *
 * @param[in] argc Number of arguments, including "dio".
 * @param[in] argv Argument values, including "dio".
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: dio get <input-name>
 */
static int32_t cmd_dio_get(int32_t argc, const char** argv)
{
    uint32_t idx;
    struct cmd_arg_val arg_vals[1];

    if (cmd_parse_args(argc-2, argv+2, "s", arg_vals) != 1)
        return MOD_ERR_BAD_CMD;

    for (idx = 0; idx < cfg->num_inputs; idx++)
        if (strcasecmp(arg_vals[0].val.s, cfg->inputs[idx].name) == 0)
            break;
    if (idx < cfg->num_inputs) {
        printc("%s = %ld\n", cfg->inputs[idx].name, dio_get(idx));
        return 0;
    }

    for (idx = 0; idx < cfg->num_outputs; idx++)
        if (strcasecmp(arg_vals[0].val.s, cfg->outputs[idx].name) == 0)
            break;
    if (idx < cfg->num_outputs) {
        printc("%s %ld\n", cfg->outputs[idx].name, dio_get_out(idx));
        return 0;
    }
    printc("Invalid dio input/output name '%s'\n", arg_vals[0].val.s);
    return MOD_ERR_ARG;
}

/*
 * @brief Console command function for "dio set".
 *
 * @param[in] argc Number of arguments, including "dio".
 * @param[in] argv Argument values, including "dio".
 *
 * @return 0 for success, else a "MOD_ERR" value. See code for details.
 *
 * Command usage: dio set <output-name> {0|1}
 */
static int32_t cmd_dio_set(int32_t argc, const char** argv)
{
    uint32_t idx;
    struct cmd_arg_val arg_vals[2];
    uint32_t value;

    if (cmd_parse_args(argc-2, argv+2, "su", arg_vals) != 2)
        return MOD_ERR_BAD_CMD;

    for (idx = 0; idx < cfg->num_outputs; idx++)
        if (strcasecmp(arg_vals[0].val.s, cfg->outputs[idx].name) == 0)
            break;
    if (idx >= cfg->num_outputs) {
        printc("Invalid dio name '%s'\n", arg_vals[0].val.s);
        return MOD_ERR_ARG;
    }

    value = arg_vals[1].val.u;
    if (value != 0 && value != 1) {
        printc("Invalid value '%s'\n", argv[3]);
        return MOD_ERR_ARG;
    }
    return dio_set(idx, value);
}

/*
 * @brief Convert GPIO pin mode bit-field to a string.
 *
 * @param[in] mode Bit-field value.
 *
 * @return String form of bit-field.
 */
static const char* gpio_pin_mode_to_str(uint32_t mode)
{
    const char* str = "?  ";
    switch (mode) {
        case LL_GPIO_MODE_INPUT:
            str = "In ";
            break;
        case LL_GPIO_MODE_OUTPUT:
            str = "Out";
            break;
        case LL_GPIO_MODE_ALTERNATE:
            str = "Alt";
            break;
        case LL_GPIO_MODE_ANALOG:
            str = "Ana";
            break;
    }
    return str;
}

/*
 * @brief Convert GPIO output type bit-field to a string.
 *
 * @param[in] mode Bit-field value.
 *
 * @return String form of bit-field.
 */
static const char* gpio_output_type_to_str(uint32_t mode)
{
    const char* str = "? ";
    switch (mode) {
        case LL_GPIO_OUTPUT_PUSHPULL:
            str = "PP";
            break;
        case LL_GPIO_OUTPUT_OPENDRAIN:
            str = "OD";
            break;
    }
    return str;
}

/*
 * @brief Convert GPIO pin speed bit-field to a string.
 *
 * @param[in] mode Bit-field value.
 *
 * @return String form of bit-field.
 */
static const char* gpio_pin_speed_to_str(uint32_t mode)
{
    const char* str = "? ";
    switch (mode) {
        case LL_GPIO_SPEED_FREQ_LOW:
            str = "Lo";
            break;
        case LL_GPIO_SPEED_FREQ_MEDIUM:
            str = "Me";
            break;
        case LL_GPIO_SPEED_FREQ_HIGH:
            str = "Hi";
            break;
#if CONFIG_DIO_TYPE != 3
        case LL_GPIO_SPEED_FREQ_VERY_HIGH:
            str = "VH";
            break;
#endif
    }
    return str;
}

/*
 * @brief Convert GPIO pull up bit-field to a string.
 *
 * @param[in] mode Bit-field value.
 *
 * @return String form of bit-field.
 */
static const char* gpio_pull_up_to_str(uint32_t mode)
{
    const char* str = "? ";
    switch (mode) {
#if CONFIG_DIO_TYPE != 3
        case LL_GPIO_PULL_NO:
            str = "No";
            break;
#endif
        case LL_GPIO_PULL_UP:
            str = "Up";
            break;
        case LL_GPIO_PULL_DOWN:
            str = "Dn";
            break;
    }
    return str;
}
