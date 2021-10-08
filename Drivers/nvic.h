#ifndef _H_NVIC_H_
#define _H_NVIC_H_

#include "stm32f303xe.h"
#include "../Drivers/general.h"

typedef enum
{
    GROUPING_16_0 = 0x0,
    GROUPING_8_2  = 0x4,
    GROUPING_4_4  = 0x5,
    GROUPING_2_8  = 0x6,
    GROUPING_0_16 = 0x7

} Priority_Grouping_e;

void nvic_set_priority(Driver_Irq_Val nvic_irq_type, uint8_t priority);
void nvic_set_priority_wrapper(Driver_Irq_Val nvic_irq_type,
                               uint8_t        priority,
                               uint8_t        sub_priority);
void nvic_set_priority_grouping(Priority_Grouping_e priority_group);
void nvic_system_init(void);
void nvic_reset_system(void);
void nvic_set_vtor(uint32_t offset);

#endif
