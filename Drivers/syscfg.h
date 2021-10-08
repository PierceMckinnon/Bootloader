#ifndef _H_SYSCFG_H_
#define _H_SYSCFG_H_

#include "../General_Includes/reg_structs.h"
#include "stm32f303xe.h"
typedef enum
{
    gpio_a           = 0x0,
    gpio_b           = 0x1,
    gpio_c           = 0x2,
    gpio_d           = 0x3,
    gpio_e           = 0x4,
    gpio_f           = 0x5,
    gpio_g           = 0x6,
    gpio_h           = 0x7,
    gpio_unsupported = -1
} SYSCFG_EXTI_e;

SYSCFG_EXTI_e gpio_port_to_config(GPIO_Regs_s *GPIOx);

#endif