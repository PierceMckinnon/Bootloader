
#include "../Drivers/gpio.h"
#include "../Drivers/clk.h"
#include "../Drivers/general.h"
#include "../Drivers/nvic.h"
#include "../General_Includes/reg_structs.h"
#include "../General_Includes/stm32f303xe.h"

static void initialize_gpio(void);
static void initialize_clock(void);

int main(void)
{
    // make sure hall init
    // nvic_setpriority
    // mvic enableirq
    initialize_clock();
    initialize_gpio();

    while (1)
    {
        // need to config frequencyh to trigger ever 1ms
        clk_delay(1000);
        gpio_write_pin(GPIOA, GPIO_5, 0x1);
    }
    return 0;
}

static void initialize_gpio(void)
{
    GPIO_Init_s gpio_init_s = { 0 };

    clk_rcc_gpio_port(GPIOA, 0x1);
    gpio_init_s.pin          = GPIO_13;
    gpio_init_s.output_type  = PUSH_PULL;
    gpio_init_s.output_speed = MEDIUM_SPEED;
    gpio_init_s.mode         = GP_OUTPUT_MODE;
    gpio_init_s.up_down      = PULL_DOWN;

    gpio_enable(GPIOA, &gpio_init_s);
}

static void initialize_clock(void)
{
    nvic_set_priority_grouping(GROUPING_16_0);

    // pass tick freq in HZ, trigger 1000 times a second, once every ms
    clk_init_sys_tick(NVIC_PRIO_BITS_MASK, 1000);
    clk_rcc_syscfg(0x1);
}