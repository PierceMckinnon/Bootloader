#include "../Drivers/gpio.h"
#include "../Drivers/clk.h"
#include "../Drivers/nvic.h"
#include "../Drivers/syscfg.h"
#include "../General_Includes/reg_op.h"
// #include <assert.h>

static SYSCFG_Regs_s *syscfg_p = (SYSCFG_Regs_s *)SYSCFG;
static EXTI_Regs_s *  exti_p   = (EXTI_Regs_s *)EXTI;

void gpio_enable(void *group_r, GPIO_Init_s *Settings)
{

    // assert(group_r != NULL);
    // assert(Settings != NULL);

    GPIO_Regs_s *group    = (GPIO_Regs_s *)group_r;
    uint32_t     position = Settings->pin;

    if (position > 16 || position < 0)
    {
        // assert(0x0);
    }
    // If output then set these regs
    if (Settings->mode == GP_OUTPUT_MODE || Settings->mode == AF_MODE)
    {
        _write_reg_f(group->OSPEEDR,
                     Settings->output_speed,
                     O_SPEED_R_WIDTH * position,
                     GPIO_OSPEED_MSK);
        _write_reg_f(
            group->OTYPER, Settings->output_type, position, DEFAULT_PIN_MASK);
    }
    // configure wether pull up or pull down
    _write_reg_f(group->PUPDR,
                 Settings->up_down,
                 PU_PD_R_WIDTH * position,
                 GPIO_PUPD_MSK);
    // configure alternate function
    if (Settings->mode == AF_MODE)
    {
        if (position > 7)
        {
            _write_reg_f(group->AFRH,
                         Settings->af,
                         AF_R_WIDTH * (position - 0x8),
                         GPIO_AF_MSK);
        }
        else
        {
            _write_reg_f(
                group->AFRL, Settings->af, AF_R_WIDTH * position, GPIO_AF_MSK);
        }
    }
    // configure I/O mode
    _write_reg_f(
        group->MODER, Settings->mode, MODE_R_WIDTH * position, GPIO_MODE_MSK);
    // External Interrupt configuration
    if (Settings->exti_t)
    {
        //    /* Enable SYSCFG Clock */
        clk_rcc_syscfg(0x1);

        _write_reg_f(syscfg_p->EXTICR0,
                     gpio_port_to_config(group_r),
                     EXTICR_WIDTH * position,
                     GPIO_EXTICR_MSK);

        _write_reg_f(exti_p->IMR, Settings->irq, position, DEFAULT_PIN_MASK);

        _write_reg_f(exti_p->EMR, Settings->evt, position, DEFAULT_PIN_MASK);

        _write_reg_f(
            exti_p->RTSR, Settings->ris_trig, position, DEFAULT_PIN_MASK);

        _write_reg_f(
            exti_p->FTSR, Settings->fal_trig, position, DEFAULT_PIN_MASK);
    }
}

void gpio_set_default(void *group_r, Pin_Number_e pin)
{
    // reset all regs to default state

    GPIO_Regs_s *group = (GPIO_Regs_s *)group_r;

    _write_reg_f(exti_p->IMR, 0x0, pin, DEFAULT_PIN_MASK);

    _write_reg_f(exti_p->EMR, 0x0, pin, DEFAULT_PIN_MASK);

    _write_reg_f(exti_p->RTSR, 0x0, pin, DEFAULT_PIN_MASK);

    _write_reg_f(exti_p->FTSR, 0x0, pin, DEFAULT_PIN_MASK);
    _write_reg_f(syscfg_p->EXTICR0, 0x0, EXTICR_WIDTH * pin, GPIO_EXTICR_MSK);

    if (pin > 7)
    {
        _write_reg_f(group->AFRH, 0x0, AF_R_WIDTH * (pin - 0x8), GPIO_AF_MSK);
    }
    else
    {
        _write_reg_f(group->AFRL, 0x0, AF_R_WIDTH * pin, GPIO_AF_MSK);
    }

    // low_speed
    _write_reg_f(
        group->OSPEEDR, LOW_SPEED, O_SPEED_R_WIDTH * pin, GPIO_OSPEED_MSK);

    _write_reg_f(group->OTYPER, PUSH_PULL, pin, DEFAULT_PIN_MASK);
    // no pu/pd
    _write_reg_f(group->PUPDR, XPU_XPD, PU_PD_R_WIDTH * pin, GPIO_PUPD_MSK);
    // input mode
    _write_reg_f(group->MODER, INPUT_MODE, MODE_R_WIDTH * pin, GPIO_MODE_MSK);
}

void gpio_toggle_pin(GPIO_Regs_s *group_r, Pin_Number_e pin)
{
    // assert(group_r != NULL);

    group_r->BSRR = DRIVER_SET
                    << ((_read_reg_f(group_r->ODR, pin, DEFAULT_PIN_MASK) * 0xF)
                        + pin);
}

// define error control
Driver_Bool_e gpio_read_pin(GPIO_Regs_s *group_r, Pin_Number_e pin)
{
    //     assert(group_r != NULL);
    return _read_reg_f(group_r->IDR, pin, DEFAULT_PIN_MASK);
}

void gpio_write_pin(GPIO_Regs_s *group_r, Pin_Number_e pin, Driver_Bool_e val)
{
    // assert(group_r != NULL);
    if (val)
    {
        group_r->BSRR = DRIVER_SET << pin;
    }
    else
    {
        group_r->BRR = DRIVER_SET << pin;
    }
}

void gpio_lock_pin(GPIO_Regs_s *group_r, Pin_Number_e pin, Driver_Bool_e val)
{
    // assert(group_r != NULL);
    uint32_t tmp = LCKK_BIT;
    tmp |= (DRIVER_SET << pin);
    // lckk sequence
    _write_reg(group_r->LCKR, tmp);
    _write_reg(group_r->LCKR, DRIVER_SET << pin);
    _write_reg(group_r->LCKR, tmp);
    _read_reg(group_r->LCKR);

    // assert(_read_reg_f(group_r->LCKR, LCKK_BIT, DEFAULT_PIN_MASK));
}