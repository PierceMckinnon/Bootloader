#include "../General_Includes/reg_op.h"
#include "../General_Includes/reg_structs.h"
#include "../General_Includes/stm32f303xe.h"
#include "../Drivers/uart.h"
#include "../Drivers/clk.h"
#include "../Drivers/flash.h"
#include <stdlib.h>

#define PAGE_SIZE     0x800
#define FLASH_TIMEOUT 1000
static FLASH_Regs_s *flash_p = (FLASH_Regs_s *)FLASH;

static void               flash_enable_flash_programming(Driver_Bool_e enable);
static Driver_Pass_Fail_e flash_bsy_timeout(void);
static Driver_Pass_Fail_e flash_is_flash_bsy(void);
static Driver_Pass_Fail_e flash_wait_on_flash_op(void);
static Driver_Pass_Fail_e flash_is_sequence_error(void);
static Driver_Pass_Fail_e flash_is_flash_bsy(void);

Driver_Pass_Fail_e flash_write_halfword(volatile uint32_t *address,
                                        uint16_t           data)
{
    if (flash_unlock_flash())
    {
        // make sure unlocked
        return DRIVER_FAIL;
    }
    if (flash_bsy_timeout())
    {
        return DRIVER_FAIL;
    }

    flash_enable_flash_programming(DRIVER_SET);
    // hard fault int generated if we dont write 16bits
    *address = data;

    if (flash_wait_on_flash_op())
    {
        return DRIVER_FAIL;
    }
    flash_enable_flash_programming(DRIVER_RESET);

    return DRIVER_PASS;
}

Driver_Pass_Fail_e flash_page_erase(uint32_t page_address,
                                    uint32_t number_address)
{
    if (flash_unlock_flash())
    {
        // make sure unlocked
        return DRIVER_FAIL;
    }
    if (flash_bsy_timeout())
    {
        return DRIVER_FAIL;
    }
    number_address         = (number_address + PAGE_SIZE - 1) / PAGE_SIZE;
    uint32_t start_address = 0;
    for (start_address = page_address;
         page_address < (start_address + (PAGE_SIZE * number_address));
         page_address += PAGE_SIZE)
    {
        _write_reg_f(
            flash_p->CR, DRIVER_SET, FLASH_CR_PER_POS, FLASH_CR_PER_MSK);
        _write_reg(flash_p->AR, page_address);
        _write_reg_f(
            flash_p->CR, DRIVER_SET, FLASH_CR_STRT_POS, FLASH_CR_STRT_MSK);

        if (flash_wait_on_flash_op())
        {
            return DRIVER_FAIL;
        }
        flash_enable_flash_programming(DRIVER_RESET);
    }
    return DRIVER_PASS;
}

Driver_Pass_Fail_e flash_unlock_flash(void)
{

    if (flash_p == NULL)
    {
        return DRIVER_FAIL;
    }

    if (!flash_is_flash_lock())
    {
        return DRIVER_PASS;
    }

    _write_reg(flash_p->KEYR, 0x45670123);
    clk_delay(1);
    _write_reg(flash_p->KEYR, 0xCDEF89AB);
    clk_delay(1);

    if (flash_is_flash_lock())
    {
        return DRIVER_FAIL;
    }
    return DRIVER_PASS;
}

Driver_Pass_Fail_e flash_lock_flash(void)
{
    _write_reg_f(flash_p->CR, DRIVER_SET, FLASH_CR_LOCK_POS, FLASH_CR_LOCK_MSK);

    if (!flash_is_flash_lock())
    {
        return DRIVER_FAIL;
    }
    return DRIVER_FAIL;
}

Driver_Pass_Fail_e flash_is_flash_lock(void)
{
    return _read_reg_f(flash_p->CR, FLASH_CR_LOCK_POS, FLASH_CR_LOCK_MSK);
}

static Driver_Pass_Fail_e flash_is_flash_bsy(void)
{
    return _read_reg_f(flash_p->CR, FLASH_SR_BSY_POS, FLASH_SR_BSY_MSK);
}

static Driver_Pass_Fail_e flash_wait_on_flash_op(void)
{

    if (flash_bsy_timeout())
    {
        return DRIVER_FAIL;
    }
    if (!_read_reg_f(flash_p->SR, FLASH_SR_EOP_POS, FLASH_SR_EOP_MSK))
    {
        return DRIVER_FAIL;
    }

    // clear eop
    _write_reg_f(flash_p->SR, DRIVER_SET, FLASH_SR_EOP_POS, FLASH_SR_EOP_MSK);

    if (flash_is_sequence_error())
    {
        return DRIVER_FAIL;
    }

    return DRIVER_PASS;
}

static Driver_Pass_Fail_e flash_is_sequence_error(void)
{
    return _read_reg_f(
               flash_p->SR, FLASH_SR_WRPRTERR_POS, FLASH_SR_WRPRTERR_MSK)
           || _read_reg_f(flash_p->SR, FLASH_SR_PGERR_POS, FLASH_SR_PGERR_MSK);
}

static Driver_Pass_Fail_e flash_bsy_timeout(void)
{
    uint32_t start_time = clk_get_sys_tick_count();
    while (flash_is_flash_bsy())
    {
        if (clk_get_sys_tick_count() - start_time > FLASH_TIMEOUT)
        {
            return DRIVER_FAIL;
        }
    }
    return DRIVER_PASS;
}

static void flash_enable_flash_programming(Driver_Bool_e enable)
{
    _write_reg_f(flash_p->CR, enable, FLASH_CR_PG_POS, FLASH_CR_PG_MSK);
}
