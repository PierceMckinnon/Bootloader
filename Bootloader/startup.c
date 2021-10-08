#include <stdint.h>
#include "../Drivers/nvic.h"
#include "../Drivers/clk.h"
#include "../General_Includes/irq.h"

#define FLASH_START_ADDRESS 0x08000000

typedef void (*func_ptr)(void);

int             main(void);
extern uint32_t _boot_etext;
extern uint32_t _boot_sdata;
extern uint32_t _boot_edata;
extern uint32_t _boot_ebss;
extern uint32_t _boot_sbss;
const uint32_t  _estack = 0x20010000;

void load_bootloader(void);

func_ptr boot_vect_table[100] __attribute__((section(".isr_vector"))) = {
    (func_ptr)_estack,   // stack pointer
    load_bootloader,     // reset handler
    0,                   // Non maskable interrrupt
    0,                   // hardfault
    0,                   // memory management
    0,                   // mem access fault
    0,                   // undefined instruction or illegal state
    0,                   // reserved
    0,                   // reserved
    0,                   // reserved
    0,                   // reserved
    0,                   // Sysstem service call SVC
    0,                   // reserved
    0,                   // reserved
    0,                   // Pendable request for SVC
    system_tick_handler, // Systick
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};

//**Variables**
//_etext code segment start
//_sdata start initialized data, _edata initialized data
//_sbss start of bss segmen, _ebss end of bss segment

void load_bootloader(void)
{
    // copy initialized data from text segment to data
    uint32_t *text_seg        = &_boot_etext;
    uint32_t *start_init_data = &_boot_sdata;
    // if sections do not overlap
    if (text_seg != start_init_data)
    {
        while (start_init_data < &_boot_edata)
        {
            // copy initialize data from flash to srram
            *start_init_data = *text_seg;
            start_init_data++;
        }
    }
    // vect_table[2] = 3;
    for (uint32_t *i = &_boot_sbss; i < &_boot_ebss; i++)
    {
        // zeroe uninitalized data segment
        *i = 0;
    }

    nvic_set_vtor(FLASH_START_ADDRESS);
    // set initial clk configuration
    clk_init();
    nvic_system_init();
    main();
    while (1)
        ;
}
