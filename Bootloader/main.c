
#include "../Drivers/gpio.h"
#include "../Drivers/clk.h"
#include "../Drivers/general.h"
#include "../Drivers/nvic.h"
#include "../Drivers/uart.h"
#include "../Drivers/flash.h"
#include "../General_Includes/reg_structs.h"
#include "../General_Includes/stm32f303xe.h"

#define FLASH_START_ADDRESS       0x08000000
#define MAX_APP_SIZE              0x5000
#define APPLICATION_START_ADDRESS ((FLASH_START_ADDRESS + 0x1000))

typedef void (*func_ptr)(void);

static void               initialize_gpio(void);
static void               initialize_clock(void);
static Driver_Bool_e      initialize_uart(void);
static void               jmp_app_reset_handler(void);
static Driver_Bool_e      is_button_pressed(void);
static Driver_Pass_Fail_e setup_uart_to_receive(uint32_t *size_buffer,
                                                uint32_t  timeout);
static Driver_Pass_Fail_e application_to_flash(uint8_t *application,
                                               uint32_t application_size);

uint8_t application[MAX_APP_SIZE] = { 0 };
int     main(void)
{

    initialize_clock();
    initialize_gpio();

    // set LED indicatign we are in bootloader
    gpio_write_pin(GPIOA, GPIO_5, DRIVER_SET);

    if (is_button_pressed())
    {
        uint32_t status      = 0;
        uint32_t timeout     = 5000;
        uint32_t buffer_size = 0;

        if (setup_uart_to_receive(&buffer_size, timeout))
        {
            return 1;
        }
        if (buffer_size == 0)
        {
            return 1;
        }
        if (buffer_size % 2 != 0)
        {
            buffer_size++;
        }

        status += uart_receive(
            (uint8_t *)&application, sizeof(application), timeout);

        status += application_to_flash((uint8_t *)&application, buffer_size);
        if (status)
        {
            return 1;
        }
        nvic_reset_system();
    }

    // reset LED inidcating bootloader exit
    gpio_toggle_pin(GPIOA, GPIO_5);

    jmp_app_reset_handler();

    while (1)
    {
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

    clk_rcc_gpio_port(GPIOC, 0x1);
    gpio_init_s.pin          = GPIO_13;
    gpio_init_s.output_type  = PUSH_PULL;
    gpio_init_s.output_speed = MEDIUM_SPEED;
    gpio_init_s.mode         = INPUT_MODE;
    gpio_init_s.up_down      = PULL_UP;

    gpio_enable(GPIOA, &gpio_init_s);
}

static void initialize_clock(void)
{
    nvic_set_priority_grouping(GROUPING_16_0);

    // pass tick freq in HZ, trigger 1000 times a second, once every ms
    clk_init_sys_tick(NVIC_PRIO_BITS_MASK, 1000);
    clk_rcc_syscfg(0x1);
}

static Driver_Bool_e initialize_uart(void)
{
    UART_Configurations_s uart_init_s = { 0 };

    uart_init_s.word_length       = BIT_LENGTH_8;
    uart_init_s.clk_src           = UART_PCLK;
    uart_init_s.baud_rate         = 9600;
    uart_init_s.stop_bits         = STOP_BIT_1;
    uart_init_s.dma               = DRIVER_RESET;
    uart_init_s.oversample        = OVERSAMPLE_16;
    uart_init_s.parity_set        = EVEN_PARITY;
    uart_init_s.one_sample_method = DRIVER_SET;

    return uart_setup(&uart_init_s);
}

static void jmp_app_reset_handler(void)
{
    func_ptr app_reset_handler = (func_ptr)(APPLICATION_START_ADDRESS + 0x4);
    app_reset_handler();
}

static Driver_Bool_e is_button_pressed(void)
{
    // check that we are requesting application to be updated
    uint32_t gpio_button_timeout = 0;
    uint32_t start_time          = clk_get_sys_tick_count();
    while (gpio_button_timeout < 5000)
    {
        gpio_button_timeout = clk_get_sys_tick_count() - start_time;
        if (!gpio_read_pin(GPIOC, GPIO_13))
        {
            return DRIVER_SET;
        }
    }
    return DRIVER_RESET;
}

static Driver_Pass_Fail_e setup_uart_to_receive(uint32_t *size_buffer,
                                                uint32_t  timeout)
{
    uint32_t status = 0;
    // load new application from uart
    status += initialize_uart();
    uint32_t ack_val = 0xDEADBEEF;
    // indicate ready to receive size
    status += uart_transmit((uint8_t *)&ack_val, sizeof(ack_val), timeout);
    // get size of application
    status += uart_receive((uint8_t *)size_buffer, sizeof(uint32_t), timeout);
    // indicate ready to receive app
    ack_val = 0xBEEFDEAD;
    status += uart_transmit((uint8_t *)&ack_val, sizeof(ack_val), timeout);

    if (status)
    {
        return DRIVER_FAIL;
    }
    return DRIVER_PASS;
}

static Driver_Pass_Fail_e application_to_flash(uint8_t *application,
                                               uint32_t application_size)
{
    if (flash_unlock_flash())
    {
        return DRIVER_FAIL;
    }

    if (flash_page_erase(FLASH_START_ADDRESS, application_size))
    {
        return DRIVER_FAIL;
    }

    uint32_t  address              = FLASH_START_ADDRESS;
    uint32_t  halfword_size        = application_size / 2;
    uint16_t *halfword_application = (uint16_t *)application;
    for (uint32_t i = 0; i < halfword_size; i++)
    {
        if (flash_write_halfword((volatile uint32_t *)address,
                                 halfword_application[i]))
        {
            return DRIVER_FAIL;
        }
        address = address + 0x2;
    }
    if (flash_lock_flash())
    {
        return DRIVER_FAIL;
    }
    return DRIVER_PASS;
}