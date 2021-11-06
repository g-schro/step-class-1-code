#ifndef _CONFIG_H_
#define _CONFIG_H_

/* @brief Configuration.
 *
 * This file contains configuration #defines. The inputs to this file that
 * control configuration setttings are:
 * - A #define for the CPU/MCU. For STM32 the name has the format: STM32...
 * - A #define for a feature. Its name has the format: CONFIG_FEAT_...
 *
 * Some standards:
 * - We always assume a console is present.
 * - The console is connected to UART 2 if possible.
 */

// Used when a dummy value is required.
#define CONFIG_DUMMY_0 0

////////////////////////////////////////////////////////////////////////////////
// Computed include and other defines based directly on MCU macro defined in
// the IDE.
////////////////////////////////////////////////////////////////////////////////

#if defined STM32F103xB
    #define CONFIG_STM32_LL_BUS_HDR "stm32f1xx_ll_bus.h"
    #define CONFIG_STM32_LL_GPIO_HDR "stm32f1xx_ll_gpio.h"
    #define CONFIG_STM32_LL_I2C_HDR "stm32f1xx_ll_i2c.h"
    #define CONFIG_STM32_LL_CORTEX_HDR "stm32f1xx_ll_cortex.h"
    #define CONFIG_STM32_LL_USART_HDR "stm32f1xx_ll_usart.h"

    #define CONFIG_DIO_TYPE 3
    #define CONFIG_I2C_TYPE 1
    #define CONFIG_USART_TYPE 1
#elif defined STM32F401xE
    #define CONFIG_STM32_LL_BUS_HDR "stm32f4xx_ll_bus.h"
    #define CONFIG_STM32_LL_GPIO_HDR "stm32f4xx_ll_gpio.h"
    #define CONFIG_STM32_LL_I2C_HDR "stm32f4xx_ll_i2c.h"
    #define CONFIG_STM32_LL_CORTEX_HDR "stm32f4xx_ll_cortex.h"
    #define CONFIG_STM32_LL_USART_HDR "stm32f4xx_ll_usart.h"

    #define CONFIG_DIO_TYPE 1
    #define CONFIG_I2C_TYPE 1
    #define CONFIG_USART_TYPE 1
#elif defined STM32L452xx
    #define CONFIG_STM32_LL_BUS_HDR "stm32l4xx_ll_bus.h"
    #define CONFIG_STM32_LL_GPIO_HDR "stm32l4xx_ll_gpio.h"
    #define CONFIG_STM32_LL_I2C_HDR "stm32l4xx_ll_i2c.h"
    #define CONFIG_STM32_LL_CORTEX_HDR "stm32l4xx_ll_cortex.h"
    #define CONFIG_STM32_LL_USART_HDR "stm32l4xx_ll_usart.h"

    #define CONFIG_DIO_TYPE 2
    #define CONFIG_I2C_TYPE 0
    #define CONFIG_USART_TYPE 2
#else
    #error "Unknown processor"
#endif

////////////////////////////////////////////////////////////////////////////////
// Module cmd (common settings)
////////////////////////////////////////////////////////////////////////////////
#define CONFIG_CMD_MAX_TOKENS 10
#define CONFIG_CMD_MAX_CLIENTS 12

////////////////////////////////////////////////////////////////////////////////
// Module console (common settings)
////////////////////////////////////////////////////////////////////////////////
#define CONFIG_CONSOLE_PRINT_BUF_SIZE 240
#define CONFIG_CONSOLE_DFLT_TTYS_INSTANCE TTYS_INSTANCE_2

////////////////////////////////////////////////////////////////////////////////
// Module draw (common settings)
////////////////////////////////////////////////////////////////////////////////
#define CONFIG_DRAW_DFLT_LINK_1_LEN_MM 149
#define CONFIG_DRAW_DFLT_LINK_2_LEN_MM 119

////////////////////////////////////////////////////////////////////////////////
// Module float (common settings)
////////////////////////////////////////////////////////////////////////////////
#define CONFIG_FLOAT_TYPE_FLOAT 1
#define CONFIG_FLOAT_TYPE_DOUBLE 0
#define CONFIG_FLOAT_TYPE_LONG_DOUBLE 0

////////////////////////////////////////////////////////////////////////////////
// Module i2c (common settings)
////////////////////////////////////////////////////////////////////////////////
#define CONFIG_I2C_DFLT_TRANS_GUARD_TIME_MS 100

////////////////////////////////////////////////////////////////////////////////
// Module step (common settings)
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Module tmphm parameters (common)
////////////////////////////////////////////////////////////////////////////////
#define CONFIG_TMPHM_1_DFLT_I2C_ADDR 0x44
#define CONFIG_TMPHM_DFLT_SAMPLE_TIME_MS 1000
#define CONFIG_TMPHM_DFLT_MEAS_TIME_MS 17

////////////////////////////////////////////////////////////////////////////////
// Module ttys (common)
////////////////////////////////////////////////////////////////////////////////
#define CONFIG_TTYS_2_PRESENT 1

////////////////////////////////////////////////////////////////////////////////
// Feature-dependent configuration.
////////////////////////////////////////////////////////////////////////////////

// GPS (used in base MCU video course.
// ----------------------------------------
#if defined CONFIG_FEAT_GPS
    #if defined STM32F103xB
        #define CONFIG_GPS_DFLT_TTYS_INSTANCE TTYS_INSTANCE_3
    #elif defined STM32F401xE
        #define CONFIG_GPS_DFLT_TTYS_INSTANCE TTYS_INSTANCE_6
    #elif defined STM32L452xx
        #define CONFIG_GPS_DFLT_TTYS_INSTANCE TTYS_INSTANCE_3
    #endif
#else
    #define CONFIG_GPS_DFLT_TTYS_INSTANCE CONFIG_DUMMY_0
#endif

// DRAW and STEP
// ----------------------------------------
#if defined CONFIG_FEAT_DRAW

    #define CONFIG_DRAW_PRESENT 1
    #define CONFIG_STEP_1_PRESENT 1
    #define CONFIG_STEP_2_PRESENT 1

    #define CONFIG_DRAW_DFLT_STEP_INSTANCE_1 STEP_INSTANCE_1
    #define CONFIG_DRAW_DFLT_STEP_INSTANCE_2 STEP_INSTANCE_2

    #if defined STM32F401xE

        #define CONFIG_STEP_1_DFLT_GPIO_PORT DIO_PORT_A
        #define CONFIG_STEP_1_DFLT_DIO_PIN_A DIO_PIN_10
        #define CONFIG_STEP_1_DFLT_DIO_PIN_NOT_A DIO_PIN_12
        #define CONFIG_STEP_1_DFLT_DIO_PIN_B DIO_PIN_11
        #define CONFIG_STEP_1_DFLT_DIO_PIN_NOT_B DIO_PIN_9
        #define CONFIG_STEP_1_DFLT_IDLE_TIMER_MS 2000
        #define CONFIG_STEP_1_DFLT_REV_DIRECTION false
        #define CONFIG_STEP_1_DFLT_DRIVE_MODE STEP_DRIVE_MODE_FULL

        #define CONFIG_STEP_2_DFLT_GPIO_PORT DIO_PORT_C
        #define CONFIG_STEP_2_DFLT_DIO_PIN_A DIO_PIN_1
        #define CONFIG_STEP_2_DFLT_DIO_PIN_NOT_A DIO_PIN_3
        #define CONFIG_STEP_2_DFLT_DIO_PIN_B DIO_PIN_2
        #define CONFIG_STEP_2_DFLT_DIO_PIN_NOT_B DIO_PIN_0
        #define CONFIG_STEP_2_DFLT_IDLE_TIMER_MS 2000
        #define CONFIG_STEP_2_DFLT_REV_DIRECTION false
        #define CONFIG_STEP_2_DFLT_DRIVE_MODE STEP_DRIVE_MODE_FULL
    #else
        #error DRAW not supported
    #endif
#else
    #define CONFIG_DRAW_DFLT_STEP_INSTANCE_1 CONFIG_DUMMY_0
    #define CONFIG_DRAW_DFLT_STEP_INSTANCE_2 CONFIG_DUMMY_0
#endif

// TMPHM and I2C
// ----------------------------------------
#if defined CONFIG_FEAT_TMPHM
    #if defined STM32F401xE
        #define CONFIG_TMPHM_1_DFLT_I2C_INSTANCE I2C_INSTANCE_3
        #define CONFIG_TMPHM_1_PRESENT 1
        #define CONFIG_TTYS_3_PRESENT 1
        #define CONFIG_I2C_1_PRESENT 1
    #else
        #error TMPHM not supported
    #endif
#else
    #define CONFIG_TMPHM_1_DFLT_I2C_INSTANCE CONFIG_DUMMY_0
#endif

// FLOAT
// ----------------------------------------
#if defined CONFIG_FEAT_FLOAT
    #define CONFIG_FLOAT_PRESENT 1
#endif

#endif // _CONFIG_H_
