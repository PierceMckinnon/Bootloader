#include "../Drivers/clk.h"
// need to implement handlers in startup
void system_tick_handler(void)
{
    clk_inc_sys_tick_count();
}

// void GPIO_EXTI_HANDLER() figure out how to handle gpio int, not needed for
// now