#ifndef _H_FLASH_H_
#define _H_FLASH_H_

#include "../General_Includes/stm32f303xe.h"
#include "../Drivers/clk.h"

Driver_Pass_Fail_e flash_write_halfword(volatile uint32_t* address, uint16_t data);
Driver_Pass_Fail_e flash_page_erase(uint32_t page_address, uint32_t number_address);
Driver_Pass_Fail_e flash_lock_flash(void);
Driver_Pass_Fail_e flash_is_flash_lock(void);
Driver_Pass_Fail_e flash_unlock_flash(void);

#endif