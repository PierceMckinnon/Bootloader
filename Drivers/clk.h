#ifndef _H_CLK_H_
#define _H_CLK_H_

#include "stdint.h"
#include "../General_Includes/reg_structs.h"
#include "../General_Includes/reg_op.h"
#include "../Drivers/general.h"
#include "../Drivers/uart.h"

typedef enum
{
    CLK_DISABLE,
    CLK_EN
} CLK_OP;

typedef enum
{
    HCLK_NO_DIVISION = 0x0,
    HCLK_DIVIDER_2   = 0x8,
    HCLK_DIVIDER_4   = 0x9,
    HCLK_DIVIDER_8   = 0xA,
    HCLK_DIVIDER_16  = 0xB,
    HCLK_DIVIDER_64  = 0xC,
    HCLK_DIVIDER_128 = 0xD,
    HCLK_DIVIDER_256 = 0xE,
    HCLK_DIVIDER_512 = 0xF,
} CLK_HCLK_Prescaler;

typedef enum
{
    PCLK_NO_DIVISION = 0x0,
    PCLK_DIVIDER_2   = 0x4,
    PCLK_DIVIDER_4   = 0x5,
    PCLK_DIVIDER_8   = 0x6,
    PCLK_DIVIDER_16  = 0x7
} CLK_PCLK_Prescaler;

void     clk_inc_sys_tick_count(void);
uint32_t clk_get_sys_tick_count(void);
uint32_t clk_get_sys_tick_prio(void);
uint32_t clk_get_sys_tick_freq(void);

void          clk_delay(uint32_t delay);
void          clk_stop_tick(void);
void          clk_start_tick(void);
Driver_Bool_e clk_init_sys_tick(uint32_t systic_prio, uint32_t frequency);

// clk setting
void clk_rcc_gpio_port(void *GPIOx, uint32_t is_en);
void clk_rcc_syscfg(CLK_OP is_en);
void clk_rcc_pwr(CLK_OP is_en);

// core clk
void     clk_init(void);
uint32_t clk_get_sysclk(void);
void     clk_set_apb2_prescaler(CLK_PCLK_Prescaler pclk_prescaler);
void     clk_set_apb1_prescaler(CLK_PCLK_Prescaler pclk_prescaler);
void     clk_set_hclk_prescaler(CLK_HCLK_Prescaler hclk_prescaler);
void     clk_uart_clk_config(UART_Source_CLK_e clk_src, UART_Type_e uart_type);
uint32_t clk_get_pclk1(void);
uint32_t clk_get_pclk2(void);
uint32_t clk_get_hclk(void);
uint32_t clk_get_uart_clk(UART_Source_CLK_e clk_src);
Driver_Bool_e clk_reset_uart_peripheral(UART_Type_e uart_type);
#endif