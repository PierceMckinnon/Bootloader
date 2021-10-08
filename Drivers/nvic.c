#include "../Drivers/nvic.h"
#include "../General_Includes/reg_structs.h"
#include "../General_Includes/reg_op.h"
static NVIC_Regs_s *nvic_p = (NVIC_Regs_s *)NVIC;
static SCB_Regs_s * scb_p  = (SCB_Regs_s *)SCB;
static uint32_t     nvic_get_prioritygrouping(void);
static uint8_t      nvic_encode_priority(Priority_Grouping_e priority_group,
                                         uint32_t            priority,
                                         uint32_t            sub_priority);

#define FLASH_START_ADDRESS       0x08000000
#define APPLICATION_START_ADDRESS ((FLASH_START_ADDRESS + 0x1000))

// Main Priority:Sub Priority, ex 16:0, 16 Main priority possbilities, 0 sub
// prio possibilities

// only implement bits 7:4
void nvic_set_priority(Driver_Irq_Val nvic_irq_type, uint8_t priority)
{
    if ((int32_t)(nvic_irq_type) >= 0)
    {
        _write_reg(nvic_p->IP[nvic_irq_type], priority << 0x4);
    }
    else
    {
        _write_reg(scb_p->SHP[(((uint32_t)nvic_irq_type) & 0xF) - 0x4],
                   priority << 0x4);
    }
}

void nvic_set_priority_wrapper(Driver_Irq_Val nvic_irq_type,
                               uint8_t        priority,
                               uint8_t        sub_priority)
{
    nvic_set_priority(nvic_irq_type,
                      nvic_encode_priority(
                          nvic_get_prioritygrouping(), priority, sub_priority));
}

uint8_t nvic_encode_priority(Priority_Grouping_e priority_group,
                             uint32_t            priority,
                             uint32_t            sub_priority)
{
    uint32_t prioritygroup_masked = priority_group & 0xF;
    switch (prioritygroup_masked)
    {
        case (GROUPING_8_2): {
            return ((sub_priority & 0x1) | ((priority & 0x7) << 0x1));
        }
        break;
        case (GROUPING_4_4): {
            return ((sub_priority & 0x3) | ((priority & 0x3) << 0x2));
        }
        break;
        case (GROUPING_2_8): {
            return ((sub_priority & 0x7) | ((priority & 0x1) << 0x3));
        }
        break;
        case (GROUPING_0_16): {
            return (sub_priority & 0xF);
        }
        break;
        default: {
            return (priority & 0xF);
        }
    }
}

void nvic_set_priority_grouping(Priority_Grouping_e priority_group)
{
    _write_reg_scb_aircr_f(
        scb_p->AIRCR, priority_group, NVIC_PRIGROUP_POS, NVIC_PRIGROUP_MSK);
}
// priority grouping Group Prioritites:Sub Priorities 16:0 8:2 4:4 2:8 0:16
static uint32_t nvic_get_prioritygrouping(void)
{
    return _read_reg_f(scb_p->AIRCR, NVIC_PRIGROUP_POS, NVIC_PRIGROUP_MSK);
}

void nvic_system_init(void)
{
    // Full access priviliges for coprocessors
    _write_reg_f(
        scb_p->CPACR, 0x3, NVIC_COPROC10_ACCESS_POS, NVIC_COPROC10_ACCESS_MSK);
    _write_reg_f(
        scb_p->CPACR, 0x3, NVIC_COPROC11_ACCESS_POS, NVIC_COPROC11_ACCESS_MSK);
}

void nvic_reset_system(void)
{
    _write_reg_scb_aircr_f(scb_p->AIRCR,
                           DRIVER_SET,
                           NVIC_AIRCR_SYSRESETREQ_POS,
                           NVIC_AIRCR_SYSRESETREQ_MSK);
}

void nvic_set_vtor(uint32_t offset)
{
    // set vectore table base address
    _write_reg(scb_p->VTOR, offset); // set trap call if unaligned access
}