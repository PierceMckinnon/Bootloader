/*
//From programming manual

|||Character transmission procedure

1. Program the M bits in USART_CR1 to define the word length.
2. Select the desired baud rate using the USART_BRR register.
3. Program the number of stop bits in USART_CR2.
4. Enable the USART by writing the UE bit in USART_CR1 register to 1.
5. Select DMA enable (DMAT) in USART_CR3
6. Set the TE bit in USART_CR1 to send an idle frame as first transmission.
7. Write the data to send in the USART_TDR register (this clears the TXE bit).
Repeat this for each data to be transmitted in case of single buffer.
8. After writing the last data into the USART_TDR register, wait until TC=1.
This indicates that the transmission of the last frame is complete. This is
required for instance when the USART is disabled or enters the Halt mode to
avoid corrupting the last transmission


|||Character Reception

1. Program the M bits in USART_CR1 to define the word length.
2. Select the desired baud rate using the baud rate register USART_BRR
3. Program the number of stop bits in USART_CR2.
4. Enable the USART by writing the UE bit in USART_CR1 register to 1.
5. Select DMA enable (DMAR) in USART_CR3 if multibuffer communication is to take
place. Configure the DMA register as explained in multibuffer communication.
6. Set the RE bit USART_CR1. This enables the receiver which begins searching
for a start bit. When a character is received: • The RXNE bit is set to indicate
that the content of the shift register is transferred to the RDR. In other
words, data has been received and can be read (as well as its associated error
flags). • An interrupt is generated if the RXNEIE bit is set. • The error flags
can be set if a frame error, noise or an overrun error has been detected during
reception. PE flag can also be set with RXNE. • In multibuffer, RXNE is set
after every byte received and is cleared by the DMA read of the Receive data
Register. • In single buffer mode, clearing the RXNE bit is performed by a
software read to the USART_RDR register. The RXNE flag can also be cleared by
writing 1 to the RXFRQ in the USART_RQR register. The RXNE bit must be cleared
before the end of the reception of the next character to avoid an overrun error.

--Can trigger idle int with IDLEIE set
--Break character == framing error

||| Overrun error (at least one data is lost)
    --ORE bit will be set
    --RDR content is lost, shift register is overwritten, any data in overrun ==
lost
    --INT generated if RXNEIE bit is set or EIEIE
    --ORE reset by set to ORECF bit

|||Oversampling
    -- OVER8 bit
    --16 gives higher granularity, dtect deviations, but max communication is
clk/16

--ONEBIT in CR3  can majority vote in noisy env
--NF bit will indicate noisy data for majority vote, clear with NFCF in ISR

|||Baud Rate
    --baud rate = source clk/divider
    --if over8 == 0( divider == brr)
    --if (oversampling == 8) over8==1 brr[2:0] = divider[3:0](shifted one bit to
right, need to left shift) brr[15:4] == divider[15:4]

|||Parity
    --Enable with PCE bit in CR1, one less bit of data will be transmitted in
frame to account for parity bit
    --PE flag is set in ISR reg and int is genereted if PEIE, software clear PE
by writing 1 to PECF in ICR

|||DMA TODO
    -
*/
#include "../Drivers/uart.h"
#include "../Drivers/clk.h"
#include "../Drivers/gpio.h"
#include "../General_Includes/stm32f303xe.h"

typedef struct
{
    Pin_Number_e transmission_pin;
    void *       transmission_group;
    Pin_Number_e receiver_pin;
    void *       receiver_group;
} UART_GPIO_Config_s;

static Driver_Pass_Fail_e uart_setup_gpio(UART_Type_e uart);
static Driver_Pass_Fail_e uart_gpio_init(UART_GPIO_Config_s *uart_config,
                                         Driver_Bool_e       transmit);
static Driver_Pass_Fail_e uart_set_bit_config(USART_Regs_s *     uart_p,
                                              UART_Word_Length_e word_length);
static Driver_Pass_Fail_e
                          uart_set_baud_rate(USART_Regs_s *uart_p, UART_Configurations_s *uart_settings);
static Driver_Pass_Fail_e uart_send_check_idle_frame(void);
static Driver_Pass_Fail_e uart_timeout_wait(volatile uint32_t *val,
                                            Driver_Bool_e      comp,
                                            uint32_t           start_time_ms,
                                            uint32_t           timeout_ms,
                                            uint32_t           pos,
                                            uint32_t           msk);

static USART_Regs_s *uart_p = NULL;
static UART_Type_e   uart_type;

static inline void uart_start_transmitter(void)
{
    _write_reg_f(uart_p->CR1, DRIVER_SET, USART_CR1_RE_POS, USART_CR1_RE_MSK);
}

static inline void uart_start_receiver(void)
{
    _write_reg_f(uart_p->CR1, DRIVER_SET, USART_CR1_TE_POS, USART_CR1_TE_MSK);
}

Driver_Pass_Fail_e uart_setup(UART_Configurations_s *uart_settings)
{

    if (uart_settings == NULL || uart_p != NULL)
    {
        return DRIVER_FAIL;
    }

    if (uart_settings->dma)
    {
        uart_p    = (USART_Regs_s *)UART4;
        uart_type = UART_4;
    }
    else
    {
        uart_p    = (USART_Regs_s *)UART5;
        uart_type = UART_5;
    }

    // ensure uart not enabled
    _write_reg_f(uart_p->CR1,
                 DRIVER_RESET,
                 USART_CR1_USART_ENABLE_POS,
                 USART_CR1_USART_ENABLE_MSK);
    // set clk src
    clk_uart_clk_config(uart_settings->clk_src, uart_type);
    // set word length
    if (uart_set_bit_config(uart_p, uart_settings->word_length))
    {
        return DRIVER_FAIL;
    }
    // set baud rate
    if (uart_set_baud_rate(uart_p, uart_settings))
    {
        return DRIVER_FAIL;
    }
    // set stop bits
    _write_reg_f(uart_p->CR2,
                 uart_settings->stop_bits,
                 USART_CR2_STOP_POS,
                 USART_CR2_STOP_MSK);

    // setup for gpio
    if (uart_setup_gpio(uart_type))
    {
        return DRIVER_FAIL;
    }

    // configure parity
    if (uart_settings->parity_set != NO_PARITY)
    {
        _write_reg_f(
            uart_p->CR1, DRIVER_SET, USART_CR1_PCE_POS, USART_CR1_PCE_MSK);
        _write_reg_f(uart_p->CR1,
                     uart_settings->parity_set,
                     USART_CR1_PS_POS,
                     USART_CR1_PS_MSK);
    }

    // set sampling method, in noisy environment we want 3 sample
    _write_reg_f(uart_p->CR3,
                 uart_settings->one_sample_method,
                 USART_CR3_ONEBIT_POS,
                 USART_CR3_ONEBIT_MSK);
    // uart enable
    _write_reg_f(uart_p->CR1,
                 DRIVER_SET,
                 USART_CR1_USART_ENABLE_POS,
                 USART_CR1_USART_ENABLE_MSK);
    // transmit/receive
    _write_reg_f(uart_p->CR3,
                 uart_settings->dma,
                 USART_CR3_DMAT_POS,
                 USART_CR3_DMAT_MSK);
    _write_reg_f(uart_p->CR3,
                 uart_settings->dma,
                 USART_CR3_DMAR_POS,
                 USART_CR3_DMAR_MSK);

    uart_start_transmitter();
    uart_start_receiver();

    return uart_send_check_idle_frame();
}

Driver_Pass_Fail_e uart_un_setup(void)
{
    if (clk_reset_uart_peripheral(uart_type))
    {
        return DRIVER_FAIL;
    }
    _write_reg_f(uart_p->CR1,
                 DRIVER_RESET,
                 USART_CR1_USART_ENABLE_POS,
                 USART_CR1_USART_ENABLE_MSK);
    _write_reg(uart_p->CR1, DRIVER_RESET);
    _write_reg(uart_p->CR2, DRIVER_RESET);
    _write_reg(uart_p->CR3, DRIVER_RESET);
    uart_p    = NULL;
    uart_type = 0;
    return DRIVER_PASS;
}

Driver_Pass_Fail_e
uart_transmit(uint8_t *buffer, uint32_t buffer_size, uint32_t timeout)
{
    uint32_t start_time = clk_get_sys_tick_count();
    if (uart_p == NULL || buffer == NULL || buffer_size == 0)
    {
        return DRIVER_FAIL;
    }
    // TODO need 9 bit frame support

    while (buffer_size > 0)
    {
        // wait for tdr empty
        if (uart_timeout_wait(&(uart_p->ISR),
                              DRIVER_SET,
                              start_time,
                              timeout,
                              USART_ISR_TXE_POS,
                              USART_ISR_TXE_MSK))
        {
            return DRIVER_FAIL;
        }
        _write_reg(uart_p->TDR, *buffer & 0xFF);
        buffer_size--;
        buffer++;
        // wait for transmission complete
        if (uart_timeout_wait(&(uart_p->ISR),
                              DRIVER_SET,
                              start_time,
                              timeout,
                              USART_ISR_TC_POS,
                              USART_ISR_TC_MSK))
        {
            return DRIVER_FAIL;
        }
    }

    if (uart_timeout_wait(&(uart_p->ISR),
                          DRIVER_SET,
                          start_time,
                          timeout,
                          USART_ISR_TC_POS,
                          USART_ISR_TC_MSK))
    {
        return DRIVER_FAIL;
    }

    return DRIVER_PASS;
}

Driver_Pass_Fail_e
uart_receive(uint8_t *buffer, uint32_t buffer_size, uint32_t timeout)
{
    uint32_t start_time = clk_get_sys_tick_count();
    if (uart_p == NULL || buffer == NULL || buffer_size == 0)
    {
        return DRIVER_FAIL;
    }
    // TODO need 9 bit frame support

    while (buffer_size > 0)
    {
        // rxne tod
        if (uart_timeout_wait(&(uart_p->ISR),
                              DRIVER_SET,
                              start_time,
                              timeout,
                              USART_ISR_TC_POS,
                              USART_ISR_TC_MSK))
        {
            return DRIVER_FAIL;
        }
        *buffer = _read_reg(uart_p->RDR) & 0x1Ff;
        buffer_size--;
        buffer++;
    }

    return DRIVER_PASS;
}

static Driver_Pass_Fail_e uart_setup_gpio(UART_Type_e uart)
{
    Driver_Pass_Fail_e status = DRIVER_PASS;
    UART_GPIO_Config_s uart_config;

    switch (uart)
    {
        case (UART_4): {
            // PC10
            uart_config.transmission_pin   = GPIO_10;
            uart_config.transmission_group = GPIOC;
            // PC11
            uart_config.receiver_pin   = GPIO_11;
            uart_config.receiver_group = GPIOC;
        }
        break;
        case (UART_5): {
            // PC12
            uart_config.transmission_pin   = GPIO_10;
            uart_config.transmission_group = GPIOC;
            // PD2
            uart_config.receiver_pin   = GPIO_2;
            uart_config.receiver_group = GPIOD;
        }
        break;
    }

    status = uart_gpio_init(&uart_config, DRIVER_SET);
    status += uart_gpio_init(&uart_config, DRIVER_RESET);

    return status;
}

static Driver_Pass_Fail_e uart_gpio_init(UART_GPIO_Config_s *uart_config,
                                         Driver_Bool_e       transmit)
{

    if (uart_config == NULL)
    {
        return DRIVER_FAIL;
    }
    GPIO_Init_s uart_gpio_init_s = { 0 };
    void *      group            = (transmit) ? uart_config->transmission_group
                             : uart_config->receiver_group;
    clk_rcc_gpio_port(group, 0x1);
    uart_gpio_init_s.pin = (transmit) ? uart_config->transmission_pin
                                      : uart_config->receiver_pin;
    uart_gpio_init_s.output_type  = PUSH_PULL;
    uart_gpio_init_s.output_speed = HIGH_SPEED;
    uart_gpio_init_s.mode         = (transmit) ? GP_OUTPUT_MODE : INPUT_MODE;
    uart_gpio_init_s.up_down      = PULL_UP;
    uart_gpio_init_s.af           = AF5; // uart

    gpio_enable(group, &uart_gpio_init_s);
    return DRIVER_PASS;
}

static Driver_Pass_Fail_e uart_set_bit_config(USART_Regs_s *     uart_p,
                                              UART_Word_Length_e word_length)
{
    if (uart_p == NULL)
    {
        return DRIVER_FAIL;
    }
    Driver_Bool_e m0;
    Driver_Bool_e m1;
    switch (word_length)
    {
        case (BIT_LENGTH_8): {
            m0 = 0;
            m1 = 0;
        }
        break;
        case (BIT_LENGTH_9): {
            m0 = 1;
            m1 = 0;
        }
        break;
        case (BIT_LENGTH_7): {
            m0 = 0;
            m1 = 1;
        }
        break;
    }

    _write_reg_f(uart_p->CR1, m0, USART_CR1_M0_POS, USART_CR1_M0_MSK);
    _write_reg_f(uart_p->CR1, m1, USART_CR1_M1_POS, USART_CR1_M1_MSK);

    return DRIVER_PASS;
}

static Driver_Pass_Fail_e
uart_set_baud_rate(USART_Regs_s *uart_p, UART_Configurations_s *uart_settings)
{
    uint32_t brr;

    if (uart_p == NULL || uart_settings == NULL)
    {
        return DRIVER_FAIL;
    }

    uint32_t oversample  = uart_settings->oversample;
    uint32_t target_rate = uart_settings->baud_rate;
    uint32_t clk         = clk_get_uart_clk(uart_settings->clk_src);

    if (clk == 0)
    {
        return DRIVER_FAIL;
    }

    _write_reg_f(
        uart_p->CR1, oversample, USART_CR1_OVER8_POS, USART_CR1_OVER8_MSK);

    if (oversample == OVERSAMPLE_16)
    {
        brr = clk / target_rate;
    }
    else
    {
        uint32_t temp = 0;
        brr           = (2 * clk) / target_rate;
        temp          = brr & 0xF;
        temp >>= 0x1;
        brr &= ~0xF;
        brr |= temp;
    }

    if (brr < 0x10)
    {
        return DRIVER_FAIL;
    }
    _write_reg(uart_p->BRR, brr);

    return DRIVER_PASS;
}

static Driver_Pass_Fail_e uart_timeout_wait(volatile uint32_t *val,
                                            Driver_Bool_e      comp,
                                            uint32_t           start_time_ms,
                                            uint32_t           timeout_ms,
                                            uint32_t           pos,
                                            uint32_t           msk)
{
    uint32_t end_time_ms;
    if (timeout_ms == 0)
    {
        return DRIVER_FAIL;
    }
    while ((*val & (msk << pos)) != comp)
    {
        end_time_ms = clk_get_sys_tick_count();
        if ((end_time_ms - start_time_ms) > timeout_ms)
        {
            return DRIVER_FAIL;
        }
    }
    return DRIVER_PASS;
}

static Driver_Pass_Fail_e uart_send_check_idle_frame(void)
{
    uint32_t start_time = clk_get_sys_tick_count();
    if (uart_timeout_wait(&(uart_p->ISR),
                          DRIVER_SET,
                          start_time,
                          GENERAL_TIMEOUT_MS,
                          USART_ISR_TEACK_POS,
                          USART_ISR_TEACK_MSK))
    {
        return DRIVER_FAIL;
    }

    if (uart_timeout_wait(&(uart_p->ISR),
                          DRIVER_SET,
                          start_time,
                          GENERAL_TIMEOUT_MS,
                          USART_ISR_REACK_POS,
                          USART_ISR_REACK_MSK))
    {
        return DRIVER_FAIL;
    }

    return DRIVER_PASS;
}