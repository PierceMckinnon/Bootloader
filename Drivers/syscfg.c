#include "../Drivers/syscfg.h"

SYSCFG_EXTI_e gpio_port_to_config(GPIO_Regs_s *GPIOx)
{
    switch ((uint32_t)GPIOx)
    {
        case ((uint32_t)GPIOA): {
            return gpio_a;
        }
        break;
        case ((uint32_t)GPIOB): {
            return gpio_b;
        }
        break;
        case ((uint32_t)GPIOC): {
            return gpio_c;
        }
        break;
        case ((uint32_t)GPIOD): {
            return gpio_d;
        }
        break;
        case ((uint32_t)GPIOE): {
            return gpio_e;
        }
        break;
        case ((uint32_t)GPIOF): {
            return gpio_f;
        }
        break;
        case ((uint32_t)GPIOG): {
            return gpio_g;
        }
        break;
        case ((uint32_t)GPIOH): {
            return gpio_h;
        }
        break;
        default: {
            return gpio_unsupported;
        }
        break;
    }
}