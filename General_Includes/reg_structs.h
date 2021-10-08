#ifndef _H_REG_STRUCTS_H_
#define _H_REG_STRUCTS_H_

#include "stdint.h"
#define MODE_R_WIDTH     0x2
#define GPIO_MODE_MSK    0x3
#define O_SPEED_R_WIDTH  0x2
#define GPIO_OSPEED_MSK  0x3
#define PU_PD_R_WIDTH    0x2
#define GPIO_PUPD_MSK    0x3
#define AF_R_WIDTH       0x4
#define GPIO_AF_MSK      0xF
#define LCKK_BIT         0x10
#define DEFAULT_PIN_MASK 0x1
typedef struct
{
    // hardware changes value itself
    volatile uint32_t MODER; /*!< GPIO port mode register,               Address
                                offset: 0x00      */
    volatile uint32_t OTYPER;  /*!< GPIO port output type register,  Address
                                  offset: 0x04      */
    volatile uint32_t OSPEEDR; /*!< GPIO port output speed register, Address
                                  offset: 0x08      */
    volatile uint32_t PUPDR; /*!< GPIO port pull-up/pull-down register,  Address
                                offset: 0x0C      */
    volatile uint32_t
        IDR; /*!< GPIO port input data register,         Address offset: 0x10 */
    volatile uint32_t
                      ODR; /*!< GPIO port output data register,        Address offset: 0x14 */
    volatile uint32_t BSRR; /*!< GPIO port bit set/reset register,      Address
                               offset: 0x1A      */
    volatile uint32_t LCKR; /*!< GPIO port configuration lock register, Address
                               offset: 0x1C      */
    volatile uint32_t AFRL; /*!< GPIO alternate function low register, Address
                               offset: 0x20   */
    volatile uint32_t AFRH; /*!< GPIO alternate function high register, Address
                               offset: 0x24  */
    volatile uint32_t
        BRR; /*!< GPIO bit reset register,               Address offset: 0x28 */
} GPIO_Regs_s;

#define EXTICR_WIDTH    0x4
#define GPIO_EXTICR_MSK 0xF
typedef struct
{
    volatile uint32_t CFGR1; /*!< SYSCFG configuration register 1, Address
                                offset: 0x00 */
    volatile uint32_t RCR;   /*!< SYSCFG CCM SRAM protection register,   Address
                                offset: 0x04 */
    volatile uint32_t EXTICR0;   /*!< SYSCFG external interrupt configuration register1, Address offset: 0x08 *///selects source of interrupts for first 4 EXTI and so on
    volatile uint32_t EXTICR1; /*!< SYSCFG external interrupt configuration
                                  register2, Address offset: 0x0C */
    volatile uint32_t EXTICR2; /*!< SYSCFG external interrupt configuration
                                  register3, Address offset: 0x10 */
    volatile uint32_t EXTICR3; /*!< SYSCFG external interrupt configuration
                                  register4, Address offset: 0x14 */
    volatile uint32_t CFGR2; /*!< SYSCFG configuration register 2,      Address
                                offset: 0x18 */
    volatile uint32_t RESERVED0;  /*!< Reserved,  0x1C */
    volatile uint32_t RESERVED1;  /*!< Reserved,  0x20 */
    volatile uint32_t RESERVED2;  /*!< Reserved,  0x24 */
    volatile uint32_t RESERVED4;  /*!< Reserved,  0x28 */
    volatile uint32_t RESERVED5;  /*!< Reserved,  0x2C */
    volatile uint32_t RESERVED6;  /*!< Reserved,  0x30 */
    volatile uint32_t RESERVED7;  /*!< Reserved,  0x34 */
    volatile uint32_t RESERVED8;  /*!< Reserved,  0x38 */
    volatile uint32_t RESERVED9;  /*!< Reserved,  0x3C */
    volatile uint32_t RESERVED10; /*!< Reserved, 0x40 */
    volatile uint32_t RESERVED11; /*!< Reserved, 0x44 */
    volatile uint32_t CFGR4; /*!< SYSCFG configuration register 4,      Address
                                offset: 0x48 */
    volatile uint32_t RESERVED12; /*!< Reserved, 0x4C */
    volatile uint32_t RESERVED13; /*!< Reserved, 0x50 */
} SYSCFG_Regs_s;

typedef struct
{
    volatile uint32_t IMR; /*!<EXTI Interrupt mask register,   Address offset:
                              0x00 */
    volatile uint32_t
                      EMR; /*!<EXTI Event mask register,   Address offset: 0x04 */
    volatile uint32_t RTSR; /*!<EXTI Rising trigger selection register , Address
                               offset: 0x08 */
    volatile uint32_t FTSR; /*!<EXTI Falling trigger selection register, Address
                               offset: 0x0C */
    volatile uint32_t SWIER; /*!<EXTI Software interrupt event register, Address
                                offset: 0x10 */
    volatile uint32_t PR; /*!<EXTI Pending register,    Address offset: 0x14 */
    uint32_t          RESERVED1; /*!< Reserved, 0x18      */
    uint32_t          RESERVED2; /*!< Reserved, 0x1C      */
    volatile uint32_t
        IMR2; /*!< EXTI Interrupt mask register, Address offset: 0x20 */
    volatile uint32_t
                      EMR2; /*!< EXTI Event mask register, Address offset: 0x24 */
    volatile uint32_t RTSR2;  /*!< EXTI Rising trigger selection register,
                                 Address offset: 0x28 */
    volatile uint32_t FTSR2;  /*!< EXTI Falling trigger selection register,
                                 Address offset: 0x2C */
    volatile uint32_t SWIER2; /*!< EXTI Software interrupt event register,
                                 Address offset: 0x30 */
    volatile uint32_t
        PR2; /*!< EXTI Pending register,    Address offset: 0x34 */
} EXTI_Regs_s;

// check these are accurate
#define RCC_AHBENR_GPIOHEN_Pos 0x10
#define RCC_AHBENR_GPIOAEN_Pos 0x11
#define RCC_AHBENR_GPIOBEN_Pos 0x12
#define RCC_AHBENR_GPIOCEN_Pos 0x13
#define RCC_AHBENR_GPIODEN_Pos 0x14
#define RCC_AHBENR_GPIOEEN_Pos 0x15
#define RCC_AHBENR_GPIOFEN_Pos 0x16
#define RCC_AHBENR_GPIOGEN_Pos 0x17

#define CLK_SYSCFGEN_POS          0x0
#define CLK_SYSCFGEN_MSK          0x1
#define CLK_CFGR_PPRE2_POS        0xB
#define CLK_CFGR_PPRE2_MSK        0x7
#define CLK_CFGR_PPRE1_POS        0x8
#define CLK_CFGR_PPRE1_MSK        0x7
#define CLK_CFGR_HPRE_POS         0x4
#define CLK_CFGR_HPRE_MSK         0xF
#define CLK_CFGR3_UART5SW_POS     0x16
#define CLK_CFGR3_UART5SW_MSK     0x3
#define CLK_CFGR3_UART4SW_POS     0x14
#define CLK_CFGR3_UART4SW_MSK     0x3
#define CLK_APB1ENR_UART5EN_POS   0x14
#define CLK_APB1ENR_UART5EN_MSK   0x1
#define CLK_APB1ENR_UART4EN_POS   0x13
#define CLK_APB1ENR_UART4EN_MSK   0x1
#define CLK_APB1RSTR_UART5RST_POS 0x14
#define CLK_APB1RSTR_UART5RST_MSK 0x1
#define CLK_APB1RSTR_UART4RST_POS 0x13
#define CLK_APB1RSTR_UART4RST_MSK 0x1
#define CLK_PWREN_POS             0x1C
#define CLK_PWREN_MSK             0x1
#define CLK_HSION_MSK             0x0
#define CKL_HSION_POS             0x1
typedef struct
{
    volatile uint32_t
        CR; /*!< RCC clock control register, Address offset: 0x00 */
    volatile uint32_t
        CFGR; /*!< RCC clock configuration register, Address offset: 0x04 */
    volatile uint32_t
                      CIR; /*!< RCC clock interrupt register, Address offset: 0x08 */
    volatile uint32_t APB2RSTR; /*!< RCC APB2 peripheral reset register, Address
                               offset: 0x0C */
    volatile uint32_t APB1RSTR; /*!< RCC APB1 peripheral reset register, Address
                               offset: 0x10 */
    volatile uint32_t
                      AHBENR; /*!< RCC AHB peripheral clock register, Address offset: 0x14 */
    volatile uint32_t APB2ENR; /*!< RCC APB2 peripheral clock enable register,
                              Address offset: 0x18 */
    volatile uint32_t APB1ENR; /*!< RCC APB1 peripheral clock enable register,
                              Address offset: 0x1C */
    volatile uint32_t
        BDCR; /*!< RCC Backup domain control register, Address offset: 0x20 */
    volatile uint32_t
        CSR; /*!< RCC clock control & status register, Address offset: 0x24 */
    volatile uint32_t
        AHBRSTR; /*!< RCC AHB peripheral reset register, Address offset: 0x28 */
    volatile uint32_t
        CFGR2; /*!< RCC clock configuration register 2, Address offset: 0x2C */
    volatile uint32_t
        CFGR3; /*!< RCC clock configuration register 3, Address offset: 0x30 */
} RCC_Regs_s;

#define SYSTICK_CTRL_MSK      0x1
#define SYSTICK_CLKSOURCE_POS 0x2
#define SYSTICK_TICKINT_POS   0x1
#define SYSTICK_ENABLE_POS    0x0

typedef struct
{
    volatile uint32_t
        CTRL; /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
    volatile uint32_t
        LOAD; /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
    volatile uint32_t
        VAL; /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
    volatile const uint32_t
        CALIB; /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Regs_s;

#define NVIC_PRIO_BITS_MASK 0xF
typedef struct
{
    volatile uint32_t
             ISER[0x8]; /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
    uint32_t RESERVED0[0x18];
    volatile uint32_t
             ICER[0x8]; /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
    uint32_t RSERVED1[0x18];
    volatile uint32_t
             ISPR[0x8]; /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
    uint32_t RESERVED2[0x18];
    volatile uint32_t
             ICPR[0x8]; /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
    uint32_t RESERVED3[0x18];
    volatile uint32_t
                     IABR[0x8]; /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
    uint32_t         RESERVED4[0x38];
    volatile uint8_t IP[0xF0]; /*!< Offset: 0x300 (R/W)  Interrupt Priority
                                  Register (8Bit wide) */
    uint32_t          RESERVED5[0x284];
    volatile uint32_t STIR; /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt
                               Register write ONLY*/
} NVIC_Regs_s;

#define NVIC_PRIGROUP_MSK          0x7
#define NVIC_PRIGROUP_POS          0x8
#define NVIC_COPROC10_ACCESS_MSK   0x3
#define NVIC_COPROC10_ACCESS_POS   0x14
#define NVIC_COPROC11_ACCESS_MSK   0x3
#define NVIC_COPROC11_ACCESS_POS   0x16
#define NVIC_AIRCR_SYSRESETREQ_MSK 0x1
#define NVIC_AIRCR_SYSRESETREQ_POS 0x2

typedef struct
{
    volatile const uint32_t
        CPUID; /*!< Offset: 0x000 (R/ )  CPUID Base Register */
    volatile uint32_t
        ICSR; /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
    volatile uint32_t
        VTOR; /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
    volatile uint32_t
        AIRCR; /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control
                  Register, provides priority grouping and control for ints */
    volatile uint32_t SCR; /*!< Offset: 0x010 (R/W)  System Control Register */
    volatile uint32_t
                     CCR; /*!< Offset: 0x014 (R/W)  Configuration Control Register */
    volatile uint8_t SHP[0xC]; /*!< Offset: 0x018 (R/W)  System Handlers
                                  Priority Registers (4-7, 8-11, 12-15) */
    volatile uint32_t SHCSR;   /*!< Offset: 0x024 (R/W)  System Handler Control
                                  and   State Register */
    volatile uint32_t
        CFSR; /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
    volatile uint32_t
        HFSR; /*!< Offset: 0x02C (R/W)  HardFault Status Register */
    volatile uint32_t
        DFSR; /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
    volatile uint32_t
        MMFAR; /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
    volatile uint32_t
        BFAR; /*!< Offset: 0x038 (R/W)  BusFault Address Register */
    volatile uint32_t
        AFSR; /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
    volatile const uint32_t
        PFR[0x2]; /*!< Offset: 0x040 (R/ )  Processor Feature Register */
    volatile const uint32_t
        DFR; /*!< Offset: 0x048 (R/ )  Debug Feature Register */
    volatile const uint32_t
        ADR; /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
    volatile const uint32_t
                            MMFR[0x4]; /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
    volatile const uint32_t ISAR[0x5]; /*!< Offset: 0x060 (R/ )  Instruction Set
                                          Attributes Register */
    uint32_t RESERVED0[0x5];
    volatile uint32_t
        CPACR; /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Regs_s;

#define USART_CR1_M1_POS           0x1C
#define USART_CR1_M1_MSK           0x1
#define USART_CR1_M0_POS           0xC
#define USART_CR1_M0_MSK           0x1
#define USART_CR1_M_MSK            0x3
#define USART_CR1_USART_ENABLE_MSK 0x1
#define USART_CR1_USART_ENABLE_POS 0x0
#define USART_CR1_TE_MSK           0x1
#define USART_CR1_TE_POS           0x3
#define USART_CR1_RE_MSK           0x1
#define USART_CR1_RE_POS           0x2
#define USART_CR1_PCE_MSK          0x1
#define USART_CR1_PCE_POS          0xA
#define USART_CR1_PS_MSK           0x1
#define USART_CR1_PS_POS           0x9
#define USART_CR1_OVER8_MSK        0x1
#define USART_CR1_OVER8_POS        0xF
#define USART_CR2_STOP_MSK         0x3
#define USART_CR2_STOP_POS         0xC
#define USART_CR3_DMAT_MSK         0x1
#define USART_CR3_DMAT_POS         0x7
#define USART_CR3_DMAR_MSK         0x1
#define USART_CR3_DMAR_POS         0x6
#define USART_CR3_ONEBIT_MSK       0x1
#define USART_CR3_ONEBIT_POS       0xB
#define USART_ISR_TXE_MSK          0x1
#define USART_ISR_TXE_POS          0x7
#define USART_ISR_TC_MSK           0x1 // transmission complete
#define USART_ISR_TC_POS           0x6 // int generated if txeie is 1
#define USART_ISR_RXNE_MSK         0x1 // data received is ready to be read
#define USART_ISR_RXNE_POS         0x5
#define USART_ISR_IDLE_MSK         0x1
#define USART_ISR_IDLE_POS         0x4
#define USART_ISR_ORE_MSK          0x1
#define USART_ISR_ORE_POS          0x3
#define USART_ISR_NF_MSK           0x1
#define USART_ISR_NF_POS           0x2 // CHECK on rxne int
#define USART_ISR_PE_MSK           0x1
#define USART_ISR_PE_POS           0x0
#define USART_ISR_TEACK_MSK        0x1
#define USART_ISR_TEACK_POS        0x15
#define USART_ISR_REACK_MSK        0x1
#define USART_ISR_REACK_POS        0x16
#define USART_RQR_SBKRQ_POS        0x1 // int generated if rxneie
#define USART_RQR_SBKRQ_POS        0x1
#define USART_TDR_DATA_POS         0x0
#define SYSTICK_CLKSOURCE_POS      0x2
typedef struct
{
    volatile uint32_t CR1;  /*!< USART Control register 1,  Address offset: 0x00
                             */
    volatile uint32_t CR2;  /*!< USART Control register 2,  Address offset: 0x04
                             */
    volatile uint32_t CR3;  /*!< USART Control register 3,  Address offset: 0x08
                             */
    volatile uint32_t BRR;  /*!< USART Baud rate register,  Address offset: 0x0C
                             */
    volatile uint32_t GTPR; /*!< USART Guard time and prescaler register,
                               Address offset: 0x10 */
    volatile uint32_t RTOR; /*!< USART Receiver Time Out register, Address
                               offset: 0x14 */
    volatile uint32_t RQR;  /*!< USART Request register,  Address offset: 0x18
                             */
    volatile uint32_t ISR;  /*!< USART Interrupt and status register,  Address
                               offset: 0x1C */
    volatile uint32_t ICR;  /*!< USART Interrupt flag Clear register,  Address
                               offset: 0x20 */
    volatile uint16_t RDR;  /*!< USART Receive Data register,  Address offset:
                               0x24 */
    uint16_t RESERVED1;     /*!< Reserved, 0x26     */
    volatile uint16_t TDR;  /*!< USART Transmit Data register,  Address offset:
                               0x28 */
    uint16_t RESERVED2;     /*!< Reserved, 0x2A     */
} USART_Regs_s;

#define FLASH_SR_EOP_POS      0x5
#define FLASH_SR_EOP_MSK      0x1
#define FLASH_SR_WRPRTERR_POS 0x4
#define FLASH_SR_WRPRTERR_MSK 0x1
#define FLASH_SR_PGERR_POS    0x2
#define FLASH_SR_PGERR_MSK    0x1
#define FLASH_SR_BSY_POS      0x5
#define FLASH_SR_BSY_MSK      0x1
#define FLASH_CR_LOCK_POS     0x7 // when set, flash is locked
#define FLASH_CR_LOCK_MSK     0x1
#define FLASH_CR_STRT_POS     0x6 // triggers erase operation
#define FLASH_CR_STRT_MSK     0x1
#define FLASH_CR_PER_POS      0x1 // page erase
#define FLASH_CR_PER_MSK      0x1
#define FLASH_CR_PG_POS       0x0
#define FLASH_CR_PG_MSK       0x1
typedef struct
{
    volatile uint32_t ACR;  /*!< FLASH access control register,  Address offset:
                               0x00 */
    volatile uint32_t KEYR; /*!< FLASH key register, Address offset: 0x04 */
    volatile uint32_t OPTKEYR; /*!< FLASH option key register, Address offset:
                                  0x08 */
    volatile uint32_t SR;   /*!< FLASH status register,   Address offset: 0x0C */
    volatile uint32_t CR;   /*!< FLASH control register,   Address offset: 0x10 */
    volatile uint32_t AR;   /*!< FLASH address register,   Address offset: 0x14 */
    uint32_t RESERVED;      /*!< Reserved, 0x18      */
    volatile uint32_t OBR;  /*!< FLASH Option byte register,  Address offset:
                               0x1C */
    volatile uint32_t WRPR; /*!< FLASH Write register, Address offset: 0x20 */

} FLASH_Regs_s;
#endif