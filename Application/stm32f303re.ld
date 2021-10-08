_Min_Heap_Size = 0x200;
_Min_Stack_Size = 0x400;

MEMORY
{
    flash     (rx)   :   ORIGIN  =   0x08001000  ,   LENGTH  =   508k
    sram     (rwx)   :   ORIGIN  =   0x20000000  ,   LENGTH  =   64k
    ccm      (rwx)   :   ORIGIN  =   0x10000000  ,   LENGTH  =   16k
}

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector .isr_vector*)) 
        . = ALIGN(4);
    } >flash

    .text :
    {
        . = ALIGN(4);
        *(.text* .text);
        *(.rodata* .rodata);
          . = ALIGN(4);
        _app_etext = .;
      
    } >flash

    .ccmram :
    {
        . = ALIGN(4);
        _app_sccmram = .;   
        *(.ccmram .ccmram*)
        . = ALIGN(4);
        _app_eccmram = .;      
    } >ccm AT> flash

    .data :
    {
        . = ALIGN(4);
        _app_sdata = .;
        *(.data* .data)
        . = ALIGN(4);
        _app_edata = .;
    
    } >sram AT > flash  

    /*
    *sram == section where the data will be at program run, flash == load time location
    */

    .bss :
    {
        . = ALIGN(4);
        _app_sbss = .;
        *(.bss .bss.*);
        *(COMMON);
        . = ALIGN(4);
        _app_ebss = .;
    }>sram AT > flash

    /*
    *just checks enough ram left, minstack+ heap fits
    */

    .stack_heap :
    {
        /*
        *align on 8 byte boundary, define variable as end of stack/heap
        */
        . = ALIGN(8);
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >sram

} 
/*
*ccram, closely coupled ram, use for interrupts, similar speed to cache
*/
