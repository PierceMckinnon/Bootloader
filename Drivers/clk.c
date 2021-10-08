#define system_core_clock 8000000

#include "../Drivers/clk.h"
#include "../Drivers/nvic.h"
#include "stm32f303xe.h"
#include <stdlib.h>

// By default we are using HSI oscillator to generate system clocks
typedef struct
{
    uint32_t count;
    uint32_t freq;
    uint32_t priority;
} systick_s;

static uint32_t sysclk;
static uint32_t pclk1;
static uint32_t pclk2;
static uint32_t hclk;

static void     clk_systick_config(uint32_t ticks);
static void     clk_set_pclk1_divide(uint32_t divider);
static void     clk_set_pclk2_divide(uint32_t divider);
static void     clk_set_hclk_divide(uint32_t divider);
static uint32_t clk_get_pclk_prescaler(CLK_PCLK_Prescaler pclk_prescaler);
static uint32_t clk_get_hclk_prescaler(CLK_HCLK_Prescaler hclk_prescaler);

static systick_s       system_tick = { 0 };
static RCC_Regs_s *    rcc_p       = (RCC_Regs_s *)RCC;
static SysTick_Regs_s *systick_p   = (SysTick_Regs_s *)SYSTICK;

void clk_rcc_syscfg(CLK_OP is_en)
{
    _write_reg_f(rcc_p->APB2ENR, is_en, CLK_SYSCFGEN_POS, CLK_SYSCFGEN_MSK);
    clk_delay(1);
}
void clk_rcc_pwr(CLK_OP is_en)
{
    _write_reg_f(rcc_p->APB1ENR, is_en, CLK_PWREN_POS, CLK_PWREN_MSK);
    clk_delay(1);
}

void clk_rcc_gpio_port(void *GPIOx, uint32_t is_en)
{
    uint32_t clk_gpio_pos = 0;

    switch ((uint32_t)GPIOx)
    {
        case ((uint32_t)GPIOA): {
            clk_gpio_pos = RCC_AHBENR_GPIOAEN_Pos;
        }
        break;
        case ((uint32_t)GPIOB): {
            clk_gpio_pos = RCC_AHBENR_GPIOBEN_Pos;
        }
        break;
        case ((uint32_t)GPIOC): {
            clk_gpio_pos = RCC_AHBENR_GPIOCEN_Pos;
        }
        break;
        case ((uint32_t)GPIOD): {
            clk_gpio_pos = RCC_AHBENR_GPIODEN_Pos;
        }
        break;
        case ((uint32_t)GPIOE): {
            clk_gpio_pos = RCC_AHBENR_GPIOEEN_Pos;
        }
        break;
        case ((uint32_t)GPIOF): {
            clk_gpio_pos = RCC_AHBENR_GPIOFEN_Pos;
        }
        break;
        case ((uint32_t)GPIOG): {
            clk_gpio_pos = RCC_AHBENR_GPIOGEN_Pos;
        }
        break;
        case ((uint32_t)GPIOH): {
            clk_gpio_pos = RCC_AHBENR_GPIOHEN_Pos;
        }
        break;
        default: {
            // assert(0x0);
        }
        break;
    }
    _write_reg_f(rcc_p->AHBENR, is_en, clk_gpio_pos, 0x1);
    // clk_delay(1);
}

void clk_inc_sys_tick_count(void)
{
    system_tick.count += 1;
}

uint32_t clk_get_sys_tick_count(void)
{
    return system_tick.count;
}

uint32_t clk_get_sys_tick_prio(void)
{
    return system_tick.priority;
}

uint32_t clk_get_sys_tick_freq(void)
{
    return system_tick.freq;
}

void clk_delay(uint32_t delay)
{
    uint32_t tickstart = clk_get_sys_tick_count();
    while ((clk_get_sys_tick_count() - tickstart) < delay)
    {
    }
}

void clk_stop_tick(void)
{
    /* stop tick by disabling int */
    _write_reg_f(systick_p->CTRL, 0x1, SYSTICK_ENABLE_POS, SYSTICK_CTRL_MSK);
}

void clk_start_tick(void)
{
    /* start tick by enabling int */
    _write_reg_f(systick_p->CTRL, 0x0, SYSTICK_ENABLE_POS, SYSTICK_CTRL_MSK);
}

Driver_Bool_e clk_init_sys_tick(uint32_t systic_prio, uint32_t frequency)
{
    if (systic_prio > NVIC_PRIO_BITS_MASK)
    {
        return DRIVER_RESET;
    }

    system_tick.freq = frequency;
    /* configure systick priority */

    nvic_set_priority_wrapper(SysTick_IRQ, systic_prio, 0x0);
    system_tick.priority = systic_prio;
    /* configure the amount of clk cycles for int/system tick*/
    clk_systick_config(clk_get_sysclk() / system_tick.freq);

    return DRIVER_SET;
}

static void clk_systick_config(uint32_t ticks)
{
    // assert(ticks != 0);
    _write_reg(systick_p->LOAD, ticks - 0x1);

    nvic_set_priority(SysTick_IRQ,
                      (0x1 << __NVIC_PRIO_BITS)
                          - 0x1);    /* set Priority for Systick Interrupt */
    _write_reg(systick_p->VAL, 0x0); /* Clear current value */
    // selects clock to just be SYSCLK rather then SYSCLK/8
    _write_reg(systick_p->CTRL,
               0x1 << SYSTICK_CLKSOURCE_POS | 0x1 << SYSTICK_TICKINT_POS
                   | 0x1 << SYSTICK_ENABLE_POS); /* Enable systick interrupt and
                                                   timer start by default*/
}

void clk_init(void)
{
    // only using HSI osciallator for now
    _write_reg_f(rcc_p->CR, 0x1, CKL_HSION_POS, CLK_HSION_MSK);
    sysclk = system_core_clock;
    pclk1  = system_core_clock;
    pclk2  = system_core_clock;
    hclk   = system_core_clock;
}

uint32_t clk_get_sysclk(void)
{
    return sysclk;
}

uint32_t clk_get_pclk1(void)
{
    return pclk1;
}

uint32_t clk_get_pclk2(void)
{
    return pclk2;
}

uint32_t clk_get_hclk(void)
{
    return hclk;
}

static void clk_set_pclk1_divide(uint32_t divider)
{
    pclk1 = hclk / divider;
}

static void clk_set_pclk2_divide(uint32_t divider)
{
    pclk1 = hclk / divider;
}

static void clk_set_hclk_divide(uint32_t divider)
{
    hclk = sysclk / divider;
}

void clk_set_apb2_prescaler(CLK_PCLK_Prescaler pclk_prescaler)
{
    // derivation from hclk
    // devision will stack with hclk divider
    _write_reg_f(
        rcc_p->CFGR, pclk_prescaler, CLK_CFGR_PPRE2_POS, CLK_CFGR_PPRE2_MSK);
    clk_set_pclk2_divide(clk_get_pclk_prescaler(pclk_prescaler));
}

void clk_set_apb1_prescaler(CLK_PCLK_Prescaler pclk_prescaler)
{
    // derivation from hclk
    _write_reg_f(
        rcc_p->CFGR, pclk_prescaler, CLK_CFGR_PPRE1_POS, CLK_CFGR_PPRE1_MSK);
    clk_set_pclk1_divide(clk_get_pclk_prescaler(pclk_prescaler));
}

void clk_set_hclk_prescaler(CLK_HCLK_Prescaler hclk_prescaler)
{
    _write_reg_f(
        rcc_p->CFGR, hclk_prescaler, CLK_CFGR_HPRE_POS, CLK_CFGR_HPRE_MSK);
    clk_set_hclk_divide(clk_get_hclk_prescaler(hclk_prescaler));
}

void clk_uart_clk_config(UART_Source_CLK_e clk_src, UART_Type_e uart_type)
{
    switch (uart_type)
    {
        case (UART_4): {
            _write_reg_f(rcc_p->APB1RSTR,
                         DRIVER_SET,
                         CLK_APB1RSTR_UART4RST_POS,
                         CLK_APB1RSTR_UART4RST_MSK);
            clk_delay(1);
            _write_reg_f(rcc_p->APB1ENR,
                         DRIVER_SET,
                         CLK_APB1ENR_UART4EN_POS,
                         CLK_APB1ENR_UART4EN_MSK);
            _write_reg_f(rcc_p->CFGR3,
                         clk_src,
                         CLK_CFGR3_UART4SW_POS,
                         CLK_CFGR3_UART4SW_MSK);
        }
        break;
        case (UART_5): {
            _write_reg_f(rcc_p->APB1RSTR,
                         DRIVER_SET,
                         CLK_APB1RSTR_UART5RST_POS,
                         CLK_APB1RSTR_UART5RST_MSK);
            clk_delay(1);
            _write_reg_f(rcc_p->APB1ENR,
                         DRIVER_SET,
                         CLK_APB1ENR_UART5EN_POS,
                         CLK_APB1ENR_UART5EN_MSK);
            _write_reg_f(rcc_p->CFGR3,
                         clk_src,
                         CLK_CFGR3_UART5SW_POS,
                         CLK_CFGR3_UART5SW_MSK);
        }
        break;
    }
}

static uint32_t clk_get_hclk_prescaler(CLK_HCLK_Prescaler hclk_prescaler)
{
    uint32_t prescaler = 0x1;
    switch (hclk_prescaler)
    {
        case (HCLK_NO_DIVISION): {
            prescaler = 0x1;
        }
        break;
        case (HCLK_DIVIDER_2): {
            prescaler = 0x2;
        }
        break;
        case (HCLK_DIVIDER_4): {
            prescaler = 0x4;
        }
        break;
        case (HCLK_DIVIDER_8): {
            prescaler = 0x8;
        }
        break;
        case (HCLK_DIVIDER_16): {
            prescaler = 0x10;
        }
        break;
        case (HCLK_DIVIDER_64): {
            prescaler = 0x40;
        }
        break;
        case (HCLK_DIVIDER_128): {
            prescaler = 0x80;
        }
        break;
        case (HCLK_DIVIDER_256): {
            prescaler = 0x100;
        }
        break;
        case (HCLK_DIVIDER_512): {
            prescaler = 0x200;
        }
        break;
    }
    return prescaler;
}

static uint32_t clk_get_pclk_prescaler(CLK_PCLK_Prescaler pclk_prescaler)
{
    uint32_t prescaler = 0x1;
    switch (pclk_prescaler)
    {
        case (PCLK_NO_DIVISION): {
            prescaler = 0x1;
        }
        break;
        case (PCLK_DIVIDER_2): {
            prescaler = 0x2;
        }
        break;
        case (PCLK_DIVIDER_4): {
            prescaler = 0x4;
        }
        break;
        case (PCLK_DIVIDER_8): {
            prescaler = 0x8;
        }
        break;
        case (PCLK_DIVIDER_16): {
            prescaler = 0x10;
        }
        break;
    }
    return prescaler;
}

uint32_t clk_get_uart_clk(UART_Source_CLK_e clk_src)
{
    switch (clk_src)
    {
        case (UART_PCLK): {
            return clk_get_pclk1();
        }
        case (UART_SYSCLK): {
            return clk_get_sysclk();
        }
        case (UART_LSECLK):
        case (UART_HSICLK): {
            return system_core_clock;
        }
    }

    return 0;
}

Driver_Bool_e clk_reset_uart_peripheral(UART_Type_e uart_type)
{
    if (rcc_p == NULL)
    {
        return DRIVER_FAIL;
    }
    if (uart_type == UART_4)
    {
        _write_reg_f(rcc_p->APB1RSTR,
                     DRIVER_SET,
                     CLK_APB1RSTR_UART4RST_POS,
                     CLK_APB1RSTR_UART4RST_MSK);
    }
    else
    {
        _write_reg_f(rcc_p->APB1RSTR,
                     DRIVER_SET,
                     CLK_APB1RSTR_UART5RST_POS,
                     CLK_APB1RSTR_UART5RST_MSK);
    }

    return DRIVER_PASS;
}