#ifndef _H_UART_H_
#define _H_UART_H_

#include "../Drivers/general.h"
#include <stdint.h>

typedef enum
{
    STOP_BIT_1,
    STOP_BIT_0_5,
    STOP_BIT_2,
    STOP_BIT_1_5
} UART_Stop_bits_e;

typedef enum
{
    UART_4,
    UART_5
} UART_Type_e;

typedef enum
{
    BIT_LENGTH_8,
    BIT_LENGTH_9,
    BIT_LENGTH_7
} UART_Word_Length_e;

typedef enum
{
    OVERSAMPLE_16,
    OVERSAMPLE_8
} UART_OVERSAMPLE_e;

typedef enum
{
    NO_PARITY   = 0x2,
    EVEN_PARITY = 0x0,
    ODD_PARITY  = 0X1
} UART_PARITY_e;

typedef struct
{
    UART_Word_Length_e word_length;
    UART_Source_CLK_e  clk_src;
    uint32_t           baud_rate;
    UART_Stop_bits_e   stop_bits;
    Driver_Bool_e      dma;
    UART_OVERSAMPLE_e  oversample;
    UART_PARITY_e      parity_set;
    Driver_Bool_e      one_sample_method;
} UART_Configurations_s;

Driver_Pass_Fail_e uart_setup(UART_Configurations_s *uart_settings);
Driver_Pass_Fail_e
uart_transmit(uint8_t *buffer, uint32_t buffer_size, uint32_t timeout);
Driver_Pass_Fail_e
uart_receive(uint8_t *buffer, uint32_t buffer_size, uint32_t timeout);

#endif