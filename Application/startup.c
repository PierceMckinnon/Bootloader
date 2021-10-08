#include <stdint.h>
#include "../Drivers/nvic.h"
#include "../Drivers/clk.h"
#include "../General_Includes/irq.h"

#define FLASH_START_ADDRESS       0x08000000
#define APPLICATION_START_ADDRESS ((FLASH_START_ADDRESS + 0x1000))

typedef void (*func_ptr)(void);

int             main(void);
extern uint32_t _app_etext;
extern uint32_t _app_sdata;
extern uint32_t _app_edata;
extern uint32_t _app_ebss;
extern uint32_t _app_sbss;
const uint32_t  _estack = 0x20010000;

void load_application(void);

func_ptr app_vect_table[100] __attribute__((section(".isr_vector"))) = {
    (func_ptr)_estack,   // stack pointer
    load_application,    // reset handler
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

void load_application(void)
{
    // calling directly from bootloader, stack poiter needs reset
    __asm(" ldr   sp, =_estack");
    // copy initialized data from text segment to data
    uint32_t *text_seg        = &_app_etext;
    uint32_t *start_init_data = &_app_sdata;
    // if sections do not overlap
    if (text_seg != start_init_data)
    {
        while (start_init_data < &_app_edata)
        {
            // copy initialize data from flash to srram
            *start_init_data = *text_seg;
            start_init_data++;
        }
    }
    // vect_table[2] = 3;
    for (uint32_t *i = &_app_sbss; i < &_app_ebss; i++)
    {
        // zeroe uninitalized data segment
        *i = 0;
    }

    nvic_set_vtor(APPLICATION_START_ADDRESS);
    // set initial clk configuration
    clk_init();
    nvic_system_init();
    main();
    while (1)
        ;
}
