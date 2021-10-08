#ifndef _H_GPIO_H_
#define _H_GPIO_H_

#include "stdint.h"
#include "../Drivers/general.h"
#include "../General_Includes/reg_structs.h"
#include "stdlib.h"

typedef enum
{
    INPUT_MODE     = 0x0,
    GP_OUTPUT_MODE = 0x1,
    AF_MODE        = 0x2,
    ANALOG_MODE    = 0x3
} Port_Mode_e;

typedef enum
{
    PUSH_PULL  = 0x0,
    OPEN_DRAIN = 0x1
} OUTPUT_TYPE_e;

typedef enum
{
    LOW_SPEED    = 0x0,
    MEDIUM_SPEED = 0x1,
    HIGH_SPEED   = 0x3
} Output_Speed_e;

typedef enum
{
    XPU_XPD   = 0x0,
    PULL_UP   = 0x1,
    PULL_DOWN = 0x2,
} Pull_U_D_e;

typedef enum
{
    AF0  = 0x0,
    AF1  = 0x1,
    AF2  = 0x2,
    AF3  = 0x3,
    AF4  = 0x4,
    AF5  = 0x5,
    AF6  = 0x6,
    AF7  = 0x7,
    AF8  = 0x8,
    AF9  = 0x9,
    AF10 = 0xA,
    AF11 = 0xB,
    AF12 = 0xC,
    AF13 = 0xD,
    AF14 = 0xE,
    AF15 = 0xF
} Alternate_Function_e;

typedef enum
{
    NOP   = 0x0,
    RESET = 0x1
} BRR_e;

typedef enum
{
    GPIO_0  = 0x0,
    GPIO_1  = 0x1,
    GPIO_2  = 0x2,
    GPIO_3  = 0x3,
    GPIO_4  = 0x4,
    GPIO_5  = 0x5,
    GPIO_6  = 0x6,
    GPIO_7  = 0x7,
    GPIO_8  = 0x8,
    GPIO_9  = 0x9,
    GPIO_10 = 0xA,
    GPIO_11 = 0xB,
    GPIO_12 = 0xC,
    GPIO_13 = 0xD,
    GPIO_14 = 0xE,
    GPIO_15 = 0xF
} Pin_Number_e;

typedef struct
{
    Pin_Number_e         pin;
    Port_Mode_e          mode;
    OUTPUT_TYPE_e        output_type;
    Output_Speed_e       output_speed;
    Pull_U_D_e           up_down;
    Alternate_Function_e af;
    Driver_Bool_e        exti_t;
    Driver_Bool_e        irq;
    Driver_Bool_e        evt;
    Driver_Bool_e        ris_trig;
    Driver_Bool_e        fal_trig;

} GPIO_Init_s;

void          gpio_toggle_pin(GPIO_Regs_s *group_r, Pin_Number_e pin);
void          gpio_set_default(void *group_r, Pin_Number_e pin);
Driver_Bool_e gpio_read_pin(GPIO_Regs_s *group_r, Pin_Number_e pin);
void gpio_write_pin(GPIO_Regs_s *group_r, Pin_Number_e pin, Driver_Bool_e val);
void gpio_lock_pin(GPIO_Regs_s *group_r, Pin_Number_e pin, Driver_Bool_e val);
void gpio_enable(void *group_r, GPIO_Init_s *Settings);

#endif
