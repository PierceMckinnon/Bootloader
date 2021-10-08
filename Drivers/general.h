#ifndef _H_GENERAL_H_
#define _H_GENERAL_H_

#define GENERAL_TIMEOUT_MS 10
typedef enum
{
    DRIVER_RESET = 0,
    DRIVER_SET   = 1
} Driver_Bool_e;

typedef enum
{
    DRIVER_PASS = 0,
    DRIVER_FAIL = 1
} Driver_Pass_Fail_e;

typedef enum
{
    UART_PCLK,
    UART_SYSCLK,
    UART_LSECLK,
    UART_HSICLK
} UART_Source_CLK_e;

typedef enum
{
    /******  Cortex-M4 Processor Exceptions Numbers
     ****************************************************************/
    NonMaskableInt_IRQ   = -14, /*!< 2 Non Maskable Interrupt   */
    HardFault_IRQ        = -13, /*!< 3 Cortex-M4 Hard Fault Interrupt        */
    MemoryManagement_IRQ = -12, /*!< 4 Cortex-M4 Memory Management Interrupt */
    BusFault_IRQ         = -11, /*!< 5 Cortex-M4 Bus Fault Interrupt         */
    UsageFault_IRQ       = -10, /*!< 6 Cortex-M4 Usage Fault Interrupt       */
    SVCall_IRQ           = -5, /*!< 11 Cortex-M4 SV Call Interrupt            */
    DebugMonitor_IRQ     = -4, /*!< 12 Cortex-M4 Debug Monitor Interrupt      */
    PendSV_IRQ           = -2, /*!< 14 Cortex-M4 Pend SV Interrupt            */
    SysTick_IRQ = -1, /*!< 15 Cortex-M4 System Tick Interrupt           */
    /******  STM32 specific Interrupt Numbers
     **********************************************************************/
    WWDG_IRQ = 0, /*!< Window WatchDog Interrupt */
    PVD_IRQ  = 1, /*!< PVD through EXTI Line detection Interrupt  */
    TAMP_STAMP_IRQ
    = 2, /*!< Tamper and TimeStamp interrupts through the EXTI line 19 */
    RTC_WKUP_IRQ = 3, /*!< RTC Wakeup interrupt through the EXTI line 20 */
    FLASH_IRQ    = 4, /*!< FLASH global Interrupt    */
    RCC_IRQ      = 5, /*!< RCC global Interrupt      */
    EXTI0_IRQ    = 6, /*!< EXTI Line0 Interrupt    */
    EXTI1_IRQ    = 7, /*!< EXTI Line1 Interrupt    */
    EXTI2_TSC_IRQ
    = 8, /*!< EXTI Line2 Interrupt and Touch Sense Controller Interrupt */
    EXTI3_IRQ         = 9,  /*!< EXTI Line3 Interrupt          */
    EXTI4_IRQ         = 10, /*!< EXTI Line4 Interrupt         */
    DMA1_Channel1_IRQ = 11, /*!< DMA1 Channel 1 Interrupt */
    DMA1_Channel2_IRQ = 12, /*!< DMA1 Channel 2 Interrupt */
    DMA1_Channel3_IRQ = 13, /*!< DMA1 Channel 3 Interrupt */
    DMA1_Channel4_IRQ = 14, /*!< DMA1 Channel 4 Interrupt */
    DMA1_Channel5_IRQ = 15, /*!< DMA1 Channel 5 Interrupt */
    DMA1_Channel6_IRQ = 16, /*!< DMA1 Channel 6 Interrupt */
    DMA1_Channel7_IRQ = 17, /*!< DMA1 Channel 7 Interrupt */
    ADC1_2_IRQ        = 18, /*!< ADC1 & ADC2 Interrupts        */
    USB_HP_CAN_TX_IRQ
    = 19, /*!< USB Device High Priority or CAN TX Interrupts */
    USB_LP_CAN_RX0_IRQ
    = 20, /*!< USB Device Low Priority or CAN RX0 Interrupts                  */
    CAN_RX1_IRQ        = 21, /*!< CAN RX1 Interrupt        */
    CAN_SCE_IRQ        = 22, /*!< CAN SCE Interrupt        */
    EXTI9_5_IRQ        = 23, /*!< External Line[9:5] Interrupts        */
    TIM1_BRK_TIM15_IRQ = 24, /*!< TIM1 Break and TIM15 Interrupts */
    TIM1_UP_TIM16_IRQ  = 25, /*!< TIM1 Update and TIM16 Interrupts  */
    TIM1_TRG_COM_TIM17_IRQ
    = 26, /*!< TIM1 Trigger and Commutation and TIM17 Interrupt           */
    TIM1_CC_IRQ = 27, /*!< TIM1 Capture Compare Interrupt */
    TIM2_IRQ    = 28, /*!< TIM2 global Interrupt    */
    TIM3_IRQ    = 29, /*!< TIM3 global Interrupt    */
    TIM4_IRQ    = 30, /*!< TIM4 global Interrupt    */
    I2C1_EV_IRQ
    = 31, /*!< I2C1 Event Interrupt & EXTI Line23 Interrupt (I2C1 wakeup) */
    I2C1_ER_IRQ = 32, /*!< I2C1 Error Interrupt */
    I2C2_EV_IRQ
    = 33, /*!< I2C2 Event Interrupt & EXTI Line24 Interrupt (I2C2 wakeup) */
    I2C2_ER_IRQ = 34,   /*!< I2C2 Error Interrupt */
    SPI1_IRQ    = 35,   /*!< SPI1 global Interrupt    */
    SPI2_IRQ    = 36,   /*!< SPI2 global Interrupt    */
    USART1_IRQ  = 37,   /*!< USART1 global Interrupt & EXTI Line25 Interrupt
                           (USART1    wakeup)   */
    USART2_IRQ = 38,    /*!< USART2 global Interrupt & EXTI Line26 Interrupt
                           (USART2    wakeup)   */
    USART3_IRQ = 39,    /*!< USART3 global Interrupt & EXTI Line28 Interrupt
                           (USART3    wakeup)   */
    EXTI15_10_IRQ = 40, /*!< External Line[15:10] Interrupts */
    RTC_Alarm_IRQ
    = 41, /*!< RTC Alarm (A and B) through EXTI Line 17 Interrupt             */
    USBWakeUp_IRQ    = 42, /*!< USB Wakeup Interrupt */
    TIM8_BRK_IRQ     = 43, /*!< TIM8 Break Interrupt  */
    TIM8_UP_IRQ      = 44, /*!< TIM8 Update Interrupt   */
    TIM8_TRG_COM_IRQ = 45, /*!< TIM8 Trigger and Commutation Interrupt */
    TIM8_CC_IRQ      = 46, /*!< TIM8 Capture Compare Interrupt      */
    ADC3_IRQ         = 47, /*!< ADC3 global Interrupt         */
    FMC_IRQ          = 48, /*!< FMC global Interrupt          */
    SPI3_IRQ         = 51, /*!< SPI3 global Interrupt         */
    UART4_IRQ = 52, /*!< UART4 global Interrupt & EXTI Line34 Interrupt (UART4
                       wakeup)     */
    UART5_IRQ = 53, /*!< UART5 global Interrupt & EXTI Line35 Interrupt (UART5
                       wakeup)     */
    TIM6_DAC_IRQ      = 54, /*!< TIM6 global and DAC underrun error Interrupt */
    TIM7_IRQ          = 55, /*!< TIM7 global Interrupt     */
    DMA2_Channel1_IRQ = 56, /*!< DMA2 Channel 1 global Interrupt */
    DMA2_Channel2_IRQ = 57, /*!< DMA2 Channel 2 global Interrupt */
    DMA2_Channel3_IRQ = 58, /*!< DMA2 Channel 3 global Interrupt */
    DMA2_Channel4_IRQ = 59, /*!< DMA2 Channel 4 global Interrupt */
    DMA2_Channel5_IRQ = 60, /*!< DMA2 Channel 5 global Interrupt */
    ADC4_IRQ          = 61, /*!< ADC4  global Interrupt          */
    COMP1_2_3_IRQ = 64, /*!< COMP1, COMP2 and COMP3 global Interrupt via EXTI
                           Line21, 22 and 29*/
    COMP4_5_6_IRQ = 65, /*!< COMP4, COMP5 and COMP6 global Interrupt via EXTI
                           Line30, 31 and 32*/
    COMP7_IRQ   = 66,   /*!< COMP7 global Interrupt via EXTI Line33         */
    I2C3_EV_IRQ = 72,   /*!< I2C3 event interrupt       */
    I2C3_ER_IRQ = 73,   /*!< I2C3 Error Interrupt       */
    USB_HP_IRQ  = 74,   /*!< USB High Priority global Interrupt        */
    USB_LP_IRQ  = 75,   /*!< USB Low Priority global Interrupt        */
    USBWakeUp_RMP_IRQ = 76, /*!< USB Wakeup Interrupt remap */
    TIM20_BRK_IRQ     = 77, /*!< TIM20 Break Interrupt     */
    TIM20_UP_IRQ      = 78, /*!< TIM20 Update Interrupt      */
    TIM20_TRG_COM_IRQ = 79, /*!< TIM20 Trigger and Commutation Interrupt */
    TIM20_CC_IRQ      = 80, /*!< TIM20 Capture Compare Interrupt      */
    FPU_IRQ           = 81, /*!< Floating point Interrupt           */
    SPI4_IRQ          = 84, /*!< SPI4 global Interrupt          */
} Driver_Irq_Val;

#endif