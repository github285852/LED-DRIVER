/**
  ******************************************************************************
  * @file    stm32f10x.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer Header File. 
  *          This file contains all the peripheral register's definitions, bits 
  *          definitions and memory mapping for STM32F10x Connectivity line, 
  *          High density, High density value line, Medium density, 
  *          Medium density Value line, Low density, Low density Value line 
  *          and XL-density devices.
  *
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The device used in the target application
  *              - To use or not the peripheralís drivers in application code(i.e. 
  *                code will be based on direct access to peripheralís registers 
  *                rather than drivers API), this option is controlled by 
  *                "#define USE_STDPERIPH_DRIVER"
  *              - To change few application-specific parameters such as the HSE 
  *                crystal frequency
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripheralís registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32f10x
  * @{
  */
    
#ifndef __STM32F10x_H
#define __STM32F10x_H

#ifdef __cplusplus
 extern "C" {
#endif 
  
/** @addtogroup Library_configuration_section
  * @{
  */
  
/* Uncomment the line below according to the target STM32 device used in your
   application 
  */

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD) && !defined (STM32F10X_HD_VL) && !defined (STM32F10X_XL) && !defined (STM32F10X_CL) 
  /* #define STM32F10X_LD */     /*!< STM32F10X_LD: STM32 Low density devices */
  /* #define STM32F10X_LD_VL */  /*!< STM32F10X_LD_VL: STM32 Low density Value Line devices */  
  /* #define STM32F10X_MD */     /*!< STM32F10X_MD: STM32 Medium density devices */
  /* #define STM32F10X_MD_VL */  /*!< STM32F10X_MD_VL: STM32 Medium density Value Line devices */  
  /* #define STM32F10X_HD */     /*!< STM32F10X_HD: STM32 High density devices */
  /* #define STM32F10X_HD_VL */  /*!< STM32F10X_HD_VL: STM32 High density value line devices */  
  /* #define STM32F10X_XL */     /*!< STM32F10X_XL: STM32 XL-density devices */
  /* #define STM32F10X_CL */     /*!< STM32F10X_CL: STM32 Connectivity line devices */
#endif
/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.

 - Low-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
   where the Flash memory density ranges between 16 and 32 Kbytes.
 - Low-density value line devices are STM32F100xx microcontrollers where the Flash
   memory density ranges between 16 and 32 Kbytes.
 - Medium-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
   where the Flash memory density ranges between 64 and 128 Kbytes.
 - Medium-density value line devices are STM32F100xx microcontrollers where the 
   Flash memory density ranges between 64 and 128 Kbytes.   
 - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 256 and 512 Kbytes.
 - High-density value line devices are STM32F100xx microcontrollers where the 
   Flash memory density ranges between 256 and 512 Kbytes.   
 - XL-density devices are STM32F101xx and STM32F103xx microcontrollers where
   the Flash memory density ranges between 512 and 1024 Kbytes.
 - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.
  */

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD) && !defined (STM32F10X_HD_VL) && !defined (STM32F10X_XL) && !defined (STM32F10X_CL)
 #error "Please select first the target STM32F10x device used in your application (in stm32f10x.h file)"
#endif

#if !defined  USE_STDPERIPH_DRIVER
/**
 * @brief Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will 
   be based on direct access to peripherals registers 
   */
  /*#define USE_STDPERIPH_DRIVER*/
#endif

/**
 * @brief In the following line adjust the value of External High Speed oscillator (HSE)
   used in your application 
   
   Tip: To avoid modifying this file each time you need to use different HSE, you
        can define the HSE value in your toolchain compiler preprocessor.
  */           
#if !defined  HSE_VALUE
 #ifdef STM32F10X_CL   
  #define HSE_VALUE    ((uint32_t)25000000) /*!< Value of the External oscillator in Hz */
 #else 
  #define HSE_VALUE    ((uint32_t)12000000) /*!< Value of the External oscillator in Hz */
 #endif /* STM32F10X_CL */
#endif /* HSE_VALUE */


/**
 * @brief In the following line adjust the External High Speed oscillator (HSE) Startup 
   Timeout value 
   */
#define HSE_STARTUP_TIMEOUT   ((uint16_t)0x0500) /*!< Time out for HSE start up */

#define HSI_VALUE    ((uint32_t)8000000) /*!< Value of the Internal oscillator in Hz*/

/**
 * @brief STM32F10x Standard Peripheral Library version number
   */
#define __STM32F10X_STDPERIPH_VERSION_MAIN   (0x03) /*!< [31:24] main version */                                  
#define __STM32F10X_STDPERIPH_VERSION_SUB1   (0x05) /*!< [23:16] sub1 version */
#define __STM32F10X_STDPERIPH_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __STM32F10X_STDPERIPH_VERSION_RC     (0x00) /*!< [7:0]  release candidate */ 
#define __STM32F10X_STDPERIPH_VERSION       ( (__STM32F10X_STDPERIPH_VERSION_MAIN << 24)\
                                             |(__STM32F10X_STDPERIPH_VERSION_SUB1 << 16)\
                                             |(__STM32F10X_STDPERIPH_VERSION_SUB2 << 8)\
                                             |(__STM32F10X_STDPERIPH_VERSION_RC))

/**
  * @}
  */

/** @addtogroup Configuration_section_for_CMSIS
  * @{
  */

/**
 * @brief Configuration of the Cortex-M3 Processor and Core Peripherals 
 */
#ifdef STM32F10X_XL
 #define __MPU_PRESENT             1 /*!< STM32 XL-density devices provide an MPU */
#else
 #define __MPU_PRESENT             0 /*!< Other STM32 devices does not provide an MPU */
#endif /* STM32F10X_XL */
#define __NVIC_PRIO_BITS          4 /*!< STM32 uses 4 Bits for the Priority Levels    */
#define __Vendor_SysTickConfig    0 /*!< Set to 1 if different SysTick Config is used */

/**
 * @brief STM32F10x Interrupt Number Definition, according to the selected device 
 *        in @ref Library_configuration_section 
 */
typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                             */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M3 Memory Management Interrupt              */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M3 Bus Fault Interrupt                      */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M3 Usage Fault Interrupt                    */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M3 SV Call Interrupt                       */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M3 Debug Monitor Interrupt                 */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M3 Pend SV Interrupt                       */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M3 System Tick Interrupt                   */

/******  STM32 specific Interrupt Numbers *********************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                            */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt            */
  TAMPER_IRQn                 = 2,      /*!< Tamper Interrupt                                     */
  RTC_IRQn                    = 3,      /*!< RTC global Interrupt                                 */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                               */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                 */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                 */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                 */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                 */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                 */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                 */
  DMA1_Channel1_IRQn          = 11,     /*!< DMA1 Channel 1 global Interrupt                      */
  DMA1_Channel2_IRQn          = 12,     /*!< DMA1 Channel 2 global Interrupt                      */
  DMA1_Channel3_IRQn          = 13,     /*!< DMA1 Channel 3 global Interrupt                      */
  DMA1_Channel4_IRQn          = 14,     /*!< DMA1 Channel 4 global Interrupt                      */
  DMA1_Channel5_IRQn          = 15,     /*!< DMA1 Channel 5 global Interrupt                      */
  DMA1_Channel6_IRQn          = 16,     /*!< DMA1 Channel 6 global Interrupt                      */
  DMA1_Channel7_IRQn          = 17,     /*!< DMA1 Channel 7 global Interrupt                      */

#ifdef STM32F10X_LD
  ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
  USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
  USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn              = 42      /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */    
#endif /* STM32F10X_LD */  

#ifdef STM32F10X_LD_VL
  ADC1_IRQn                   = 18,     /*!< ADC1 global Interrupt                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break and TIM15 Interrupts                      */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update and TIM16 Interrupts                     */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation and TIM17 Interrupt     */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  CEC_IRQn                    = 42,     /*!< HDMI-CEC Interrupt                                   */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 and DAC underrun Interrupt                      */
  TIM7_IRQn                   = 55      /*!< TIM7 Interrupt                                       */       
#endif /* STM32F10X_LD_VL */

#ifdef STM32F10X_MD
  ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
  USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
  USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn              = 42      /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */  
#endif /* STM32F10X_MD */  

#ifdef STM32F10X_MD_VL
  ADC1_IRQn                   = 18,     /*!< ADC1 global Interrupt                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break and TIM15 Interrupts                      */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update and TIM16 Interrupts                     */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation and TIM17 Interrupt     */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  CEC_IRQn                    = 42,     /*!< HDMI-CEC Interrupt                                   */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 and DAC underrun Interrupt                      */
  TIM7_IRQn                   = 55      /*!< TIM7 Interrupt                                       */       
#endif /* STM32F10X_MD_VL */

#ifdef STM32F10X_HD
  ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
  USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
  USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
  TIM8_BRK_IRQn               = 43,     /*!< TIM8 Break Interrupt                                 */
  TIM8_UP_IRQn                = 44,     /*!< TIM8 Update Interrupt                                */
  TIM8_TRG_COM_IRQn           = 45,     /*!< TIM8 Trigger and Commutation Interrupt               */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                       */
  ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                */
  FSMC_IRQn                   = 48,     /*!< FSMC global Interrupt                                */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */
  TIM6_IRQn                   = 54,     /*!< TIM6 global Interrupt                                */
  TIM7_IRQn                   = 55,     /*!< TIM7 global Interrupt                                */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
  DMA2_Channel4_5_IRQn        = 59      /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
#endif /* STM32F10X_HD */  

#ifdef STM32F10X_HD_VL
  ADC1_IRQn                   = 18,     /*!< ADC1 global Interrupt                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_TIM15_IRQn         = 24,     /*!< TIM1 Break and TIM15 Interrupts                      */
  TIM1_UP_TIM16_IRQn          = 25,     /*!< TIM1 Update and TIM16 Interrupts                     */
  TIM1_TRG_COM_TIM17_IRQn     = 26,     /*!< TIM1 Trigger and Commutation and TIM17 Interrupt     */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  CEC_IRQn                    = 42,     /*!< HDMI-CEC Interrupt                                   */
  TIM12_IRQn                  = 43,     /*!< TIM12 global Interrupt                               */
  TIM13_IRQn                  = 44,     /*!< TIM13 global Interrupt                               */
  TIM14_IRQn                  = 45,     /*!< TIM14 global Interrupt                               */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */  
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 and DAC underrun Interrupt                      */
  TIM7_IRQn                   = 55,     /*!< TIM7 Interrupt                                       */  
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
  DMA2_Channel4_5_IRQn        = 59,     /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
  DMA2_Channel5_IRQn          = 60      /*!< DMA2 Channel 5 global Interrupt (DMA2 Channel 5 is 
                                             mapped at position 60 only if the MISC_REMAP bit in 
                                             the AFIO_MAPR2 register is set)                      */       
#endif /* STM32F10X_HD_VL */

#ifdef STM32F10X_XL
  ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
  USB_HP_CAN1_TX_IRQn         = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
  USB_LP_CAN1_RX0_IRQn        = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break Interrupt and TIM9 global Interrupt       */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global Interrupt     */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  USBWakeUp_IRQn              = 42,     /*!< USB Device WakeUp from suspend through EXTI Line Interrupt */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global Interrupt      */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global Interrupt     */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                       */
  ADC3_IRQn                   = 47,     /*!< ADC3 global Interrupt                                */
  FSMC_IRQn                   = 48,     /*!< FSMC global Interrupt                                */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */
  TIM6_IRQn                   = 54,     /*!< TIM6 global Interrupt                                */
  TIM7_IRQn                   = 55,     /*!< TIM7 global Interrupt                                */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
  DMA2_Channel4_5_IRQn        = 59      /*!< DMA2 Channel 4 and Channel 5 global Interrupt        */
#endif /* STM32F10X_XL */  

#ifdef STM32F10X_CL
  ADC1_2_IRQn                 = 18,     /*!< ADC1 and ADC2 global Interrupt                       */
  CAN1_TX_IRQn                = 19,     /*!< USB Device High Priority or CAN1 TX Interrupts       */
  CAN1_RX0_IRQn               = 20,     /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                   */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                   */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                        */
  TIM1_BRK_IRQn               = 24,     /*!< TIM1 Break Interrupt                                 */
  TIM1_UP_IRQn                = 25,     /*!< TIM1 Update Interrupt                                */
  TIM1_TRG_COM_IRQn           = 26,     /*!< TIM1 Trigger and Commutation Interrupt               */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                       */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                 */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                 */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                 */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                 */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                              */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                              */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                              */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                      */
  RTCAlarm_IRQn               = 41,     /*!< RTC Alarm through EXTI Line Interrupt                */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS WakeUp from suspend through EXTI Line Interrupt */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                               */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                               */
  TIM6_IRQn                   = 54,     /*!< TIM6 global Interrupt                                */
  TIM7_IRQn                   = 55,     /*!< TIM7 global Interrupt                                */
  DMA2_Channel1_IRQn          = 56,     /*!< DMA2 Channel 1 global Interrupt                      */
  DMA2_Channel2_IRQn          = 57,     /*!< DMA2 Channel 2 global Interrupt                      */
  DMA2_Channel3_IRQn          = 58,     /*!< DMA2 Channel 3 global Interrupt                      */
  DMA2_Channel4_IRQn          = 59,     /*!< DMA2 Channel 4 global Interrupt                      */
  DMA2_Channel5_IRQn          = 60,     /*!< DMA2 Channel 5 global Interrupt                      */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                            */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt          */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                    */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                   */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                   */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                   */
  OTG_FS_IRQn                 = 67      /*!< USB OTG FS global Interrupt                          */
#endif /* STM32F10X_CL */     
} IRQn_Type;

/**
  * @}
  */

#include "core_cm3.h"
#include "system_stm32f10x.h"
#include <stdint.h>

/** @addtogroup Exported_types
  * @{
  */  

/*!< STM32F10x Standard Peripheral Library old types (maintained for legacy purpose) */
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

/*!< STM32F10x Standard Peripheral Library old definitions (maintained for legacy purpose) */
#define HSEStartUp_TimeOut   HSE_STARTUP_TIMEOUT
#define HSE_Value            HSE_VALUE
#define HSI_Value            HSI_VALUE
/**
  * @}
  */

/** @addtogroup Peripheral_registers_structures
  * @{
  */   

/** 
  * @brief Analog to Digital Converter  
  */

typedef struct
{
  __IO uint32_t SR;
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t SMPR1;
  __IO uint32_t SMPR2;
  __IO uint32_t JOFR1;
  __IO uint32_t JOFR2;
  __IO uint32_t JOFR3;
  __IO uint32_t JOFR4;
  __IO uint32_t HTR;
  __IO uint32_t LTR;
  __IO uint32_t SQR1;
  __IO uint32_t SQR2;
  __IO uint32_t SQR3;
  __IO uint32_t JSQR;
  __IO uint32_t JDR1;
  __IO uint32_t JDR2;
  __IO uint32_t JDR3;
  __IO uint32_t JDR4;
  __IO uint32_t DR;
} ADC_TypeDef;

/** 
  * @brief Backup Registers  
  */

typedef struct
{
  uint32_t  RESERVED0;
  __IO uint16_t DR1;
  uint16_t  RESERVED1;
  __IO uint16_t DR2;
  uint16_t  RESERVED2;
  __IO uint16_t DR3;
  uint16_t  RESERVED3;
  __IO uint16_t DR4;
  uint16_t  RESERVED4;
  __IO uint16_t DR5;
  uint16_t  RESERVED5;
  __IO uint16_t DR6;
  uint16_t  RESERVED6;
  __IO uint16_t DR7;
  uint16_t  RESERVED7;
  __IO uint16_t DR8;
  uint16_t  RESERVED8;
  __IO uint16_t DR9;
  uint16_t  RESERVED9;
  __IO uint16_t DR10;
  uint16_t  RESERVED10; 
  __IO uint16_t RTCCR;
  uint16_t  RESERVED11;
  __IO uint16_t CR;
  uint16_t  RESERVED12;
  __IO uint16_t CSR;
  uint16_t  RESERVED13[5];
  __IO uint16_t DR11;
  uint16_t  RESERVED14;
  __IO uint16_t DR12;
  uint16_t  RESERVED15;
  __IO uint16_t DR13;
  uint16_t  RESERVED16;
  __IO uint16_t DR14;
  uint16_t  RESERVED17;
  __IO uint16_t DR15;
  uint16_t  RESERVED18;
  __IO uint16_t DR16;
  uint16_t  RESERVED19;
  __IO uint16_t DR17;
  uint16_t  RESERVED20;
  __IO uint16_t DR18;
  uint16_t  RESERVED21;
  __IO uint16_t DR19;
  uint16_t  RESERVED22;
  __IO uint16_t DR20;
  uint16_t  RESERVED23;
  __IO uint16_t DR21;
  uint16_t  RESERVED24;
  __IO uint16_t DR22;
  uint16_t  RESERVED25;
  __IO uint16_t DR23;
  uint16_t  RESERVED26;
  __IO uint16_t DR24;
  uint16_t  RESERVED27;
  __IO uint16_t DR25;
  uint16_t  RESERVED28;
  __IO uint16_t DR26;
  uint16_t  RESERVED29;
  __IO uint16_t DR27;
  uint16_t  RESERVED30;
  __IO uint16_t DR28;
  uint16_t  RESERVED31;
  __IO uint16_t DR29;
  uint16_t  RESERVED32;
  __IO uint16_t DR30;
  uint16_t  RESERVED33; 
  __IO uint16_t DR31;
  uint16_t  RESERVED34;
  __IO uint16_t DR32;
  uint16_t  RESERVED35;
  __IO uint16_t DR33;
  uint16_t  RESERVED36;
  __IO uint16_t DR34;
  uint16_t  RESERVED37;
  __IO uint16_t DR35;
  uint16_t  RESERVED38;
  __IO uint16_t DR36;
  uint16_t  RESERVED39;
  __IO uint16_t DR37;
  uint16_t  RESERVED40;
  __IO uint16_t DR38;
  uint16_t  RESERVED41;
  __IO uint16_t DR39;
  uint16_t  RESERVED42;
  __IO uint16_t DR40;
  uint16_t  RESERVED43;
  __IO uint16_t DR41;
  uint16_t  RESERVED44;
  __IO uint16_t DR42;
  uint16_t  RESERVED45;    
} BKP_TypeDef;
  
/** 
  * @brief Controller Area Network TxMailBox 
  */

typedef struct
{
  __IO uint32_t TIR;
  __IO uint32_t TDTR;
  __IO uint32_t TDLR;
  __IO uint32_t TDHR;
} CAN_TxMailBox_TypeDef;

/** 
  * @brief Controller Area Network FIFOMailBox 
  */
  
typedef struct
{
  __IO uint32_t RIR;
  __IO uint32_t RDTR;
  __IO uint32_t RDLR;
  __IO uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;

/** 
  * @brief Controller Area Network FilterRegister 
  */
  
typedef struct
{
  __IO uint32_t FR1;
  __IO uint32_t FR2;
} CAN_FilterRegister_TypeDef;

/** 
  * @brief Controller Area Network 
  */
  
typedef struct
{
  __IO uint32_t MCR;
  __IO uint32_t MSR;
  __IO uint32_t TSR;
  __IO uint32_t RF0R;
  __IO uint32_t RF1R;
  __IO uint32_t IER;
  __IO uint32_t ESR;
  __IO uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  __IO uint32_t FMR;
  __IO uint32_t FM1R;
  uint32_t  RESERVED2;
  __IO uint32_t FS1R;
  uint32_t  RESERVED3;
  __IO uint32_t FFA1R;
  uint32_t  RESERVED4;
  __IO uint32_t FA1R;
  uint32_t  RESERVED5[8];
#ifndef STM32F10X_CL
  CAN_FilterRegister_TypeDef sFilterRegister[14];
#else
  CAN_FilterRegister_TypeDef sFilterRegister[28];
#endif /* STM32F10X_CL */  
} CAN_TypeDef;

/** 
  * @brief Consumer Electronics Control (CEC)
  */
typedef struct
{
  __IO uint32_t CFGR;
  __IO uint32_t OAR;
  __IO uint32_t PRES;
  __IO uint32_t ESR;
  __IO uint32_t CSR;
  __IO uint32_t TXD;
  __IO uint32_t RXD;  
} CEC_TypeDef;

/** 
  * @brief CRC calculation unit 
  */

typedef struct
{
  __IO uint32_t DR;
  __IO uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  __IO uint32_t CR;
} CRC_TypeDef;

/** 
  * @brief Digital to Analog Converter
  */

typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t SWTRIGR;
  __IO uint32_t DHR12R1;
  __IO uint32_t DHR12L1;
  __IO uint32_t DHR8R1;
  __IO uint32_t DHR12R2;
  __IO uint32_t DHR12L2;
  __IO uint32_t DHR8R2;
  __IO uint32_t DHR12RD;
  __IO uint32_t DHR12LD;
  __IO uint32_t DHR8RD;
  __IO uint32_t DOR1;
  __IO uint32_t DOR2;
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
  __IO uint32_t SR;
#endif
} DAC_TypeDef;

/** 
  * @brief Debug MCU
  */

typedef struct
{
  __IO uint32_t IDCODE;
  __IO uint32_t CR;	
}DBGMCU_TypeDef;

/** 
  * @brief DMA Controller
  */

typedef struct
{
  __IO uint32_t CCR;
  __IO uint32_t CNDTR;
  __IO uint32_t CPAR;
  __IO uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  __IO uint32_t ISR;
  __IO uint32_t IFCR;
} DMA_TypeDef;

/** 
  * @brief Ethernet MAC
  */

typedef struct
{
  __IO uint32_t MACCR;
  __IO uint32_t MACFFR;
  __IO uint32_t MACHTHR;
  __IO uint32_t MACHTLR;
  __IO uint32_t MACMIIAR;
  __IO uint32_t MACMIIDR;
  __IO uint32_t MACFCR;
  __IO uint32_t MACVLANTR;             /*    8 */
       uint32_t RESERVED0[2];
  __IO uint32_t MACRWUFFR;             /*   11 */
  __IO uint32_t MACPMTCSR;
       uint32_t RESERVED1[2];
  __IO uint32_t MACSR;                 /*   15 */
  __IO uint32_t MACIMR;
  __IO uint32_t MACA0HR;
  __IO uint32_t MACA0LR;
  __IO uint32_t MACA1HR;
  __IO uint32_t MACA1LR;
  __IO uint32_t MACA2HR;
  __IO uint32_t MACA2LR;
  __IO uint32_t MACA3HR;
  __IO uint32_t MACA3LR;               /*   24 */
       uint32_t RESERVED2[40];
  __IO uint32_t MMCCR;                 /*   65 */
  __IO uint32_t MMCRIR;
  __IO uint32_t MMCTIR;
  __IO uint32_t MMCRIMR;
  __IO uint32_t MMCTIMR;               /*   69 */
       uint32_t RESERVED3[14];
  __IO uint32_t MMCTGFSCCR;            /*   84 */
  __IO uint32_t MMCTGFMSCCR;
       uint32_t RESERVED4[5];
  __IO uint32_t MMCTGFCR;
       uint32_t RESERVED5[10];
  __IO uint32_t MMCRFCECR;
  __IO uint32_t MMCRFAECR;
       uint32_t RESERVED6[10];
  __IO uint32_t MMCRGUFCR;
       uint32_t RESERVED7[334];
  __IO uint32_t PTPTSCR;
  __IO uint32_t PTPSSIR;
  __IO uint32_t PTPTSHR;
  __IO uint32_t PTPTSLR;
  __IO uint32_t PTPTSHUR;
  __IO uint32_t PTPTSLUR;
  __IO uint32_t PTPTSAR;
  __IO uint32_t PTPTTHR;
  __IO uint32_t PTPTTLR;
       uint32_t RESERVED8[567];
  __IO uint32_t DMABMR;
  __IO uint32_t DMATPDR;
  __IO uint32_t DMARPDR;
  __IO uint32_t DMARDLAR;
  __IO uint32_t DMATDLAR;
  __IO uint32_t DMASR;
  __IO uint32_t DMAOMR;
  __IO uint32_t DMAIER;
  __IO uint32_t DMAMFBOCR;
       uint32_t RESERVED9[9];
  __IO uint32_t DMACHTDR;
  __IO uint32_t DMACHRDR;
  __IO uint32_t DMACHTBAR;
  __IO uint32_t DMACHRBAR;
} ETH_TypeDef;

/** 
  * @brief External Interrupt/Event Controller
  */

typedef struct
{
  __IO uint32_t IMR;
  __IO uint32_t EMR;
  __IO uint32_t RTSR;
  __IO uint32_t FTSR;
  __IO uint32_t SWIER;
  __IO uint32_t PR;
} EXTI_TypeDef;

/** 
  * @brief FLASH Registers
  */

typedef struct
{
  __IO uint32_t ACR;
  __IO uint32_t KEYR;
  __IO uint32_t OPTKEYR;
  __IO uint32_t SR;
  __IO uint32_t CR;
  __IO uint32_t AR;
  __IO uint32_t RESERVED;
  __IO uint32_t OBR;
  __IO uint32_t WRPR;
#ifdef STM32F10X_XL
  uint32_t RESERVED1[8]; 
  __IO uint32_t KEYR2;
  uint32_t RESERVED2;   
  __IO uint32_t SR2;
  __IO uint32_t CR2;
  __IO uint32_t AR2; 
#endif /* STM32F10X_XL */  
} FLASH_TypeDef;

/** 
  * @brief Option Bytes Registers
  */
  
typedef struct
{
  __IO uint16_t RDP;
  __IO uint16_t USER;
  __IO uint16_t Data0;
  __IO uint16_t Data1;
  __IO uint16_t WRP0;
  __IO uint16_t WRP1;
  __IO uint16_t WRP2;
  __IO uint16_t WRP3;
} OB_TypeDef;

/** 
  * @brief Flexible Static Memory Controller
  */

typedef struct
{
  __IO uint32_t BTCR[8];   
} FSMC_Bank1_TypeDef; 

/** 
  * @brief Flexible Static Memory Controller Bank1E
  */
  
typedef struct
{
  __IO uint32_t BWTR[7];
} FSMC_Bank1E_TypeDef;

/** 
  * @brief Flexible Static Memory Controller Bank2
  */
  
typedef struct
{
  __IO uint32_t PCR2;
  __IO uint32_t SR2;
  __IO uint32_t PMEM2;
  __IO uint32_t PATT2;
  uint32_t  RESERVED0;   
  __IO uint32_t ECCR2; 
} FSMC_Bank2_TypeDef;  

/** 
  * @brief Flexible Static Memory Controller Bank3
  */
  
typedef struct
{
  __IO uint32_t PCR3;
  __IO uint32_t SR3;
  __IO uint32_t PMEM3;
  __IO uint32_t PATT3;
  uint32_t  RESERVED0;   
  __IO uint32_t ECCR3; 
} FSMC_Bank3_TypeDef; 

/** 
  * @brief Flexible Static Memory Controller Bank4
  */
  
typedef struct
{
  __IO uint32_t PCR4;
  __IO uint32_t SR4;
  __IO uint32_t PMEM4;
  __IO uint32_t PATT4;
  __IO uint32_t PIO4; 
} FSMC_Bank4_TypeDef; 

/** 
  * @brief General Purpose I/O
  */

typedef struct
{
  __IO uint32_t CRL;
  __IO uint32_t CRH;
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t BRR;
  __IO uint32_t LCKR;
} GPIO_TypeDef;

/** 
  * @brief Alternate Function I/O
  */

typedef struct
{
  __IO uint32_t EVCR;
  __IO uint32_t MAPR;
  __IO uint32_t EXTICR[4];
  uint32_t RESERVED0;
  __IO uint32_t MAPR2;  
} AFIO_TypeDef;
/** 
  * @brief Inter Integrated Circuit Interface
  */

typedef struct
{
  __IO uint16_t CR1;
  uint16_t  RESERVED0;
  __IO uint16_t CR2;
  uint16_t  RESERVED1;
  __IO uint16_t OAR1;
  uint16_t  RESERVED2;
  __IO uint16_t OAR2;
  uint16_t  RESERVED3;
  __IO uint16_t DR;
  uint16_t  RESERVED4;
  __IO uint16_t SR1;
  uint16_t  RESERVED5;
  __IO uint16_t SR2;
  uint16_t  RESERVED6;
  __IO uint16_t CCR;
  uint16_t  RESERVED7;
  __IO uint16_t TRISE;
  uint16_t  RESERVED8;
} I2C_TypeDef;

/** 
  * @brief Independent WATCHDOG
  */

typedef struct
{
  __IO uint32_t KR;
  __IO uint32_t PR;
  __IO uint32_t RLR;
  __IO uint32_t SR;
} IWDG_TypeDef;

/** 
  * @brief Power Control
  */

typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CSR;
} PWR_TypeDef;

/** 
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;

#ifdef STM32F10X_CL  
  __IO uint32_t AHBRSTR;
  __IO uint32_t CFGR2;
#endif /* STM32F10X_CL */ 

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)   
  uint32_t RESERVED0;
  __IO uint32_t CFGR2;
#endif /* STM32F10X_LD_VL || STM32F10X_MD_VL || STM32F10X_HD_VL */ 
} RCC_TypeDef;

/** 
  * @brief Real-Time Clock
  */

typedef struct
{
  __IO uint16_t CRH;
  uint16_t  RESERVED0;
  __IO uint16_t CRL;
  uint16_t  RESERVED1;
  __IO uint16_t PRLH;
  uint16_t  RESERVED2;
  __IO uint16_t PRLL;
  uint16_t  RESERVED3;
  __IO uint16_t DIVH;
  uint16_t  RESERVED4;
  __IO uint16_t DIVL;
  uint16_t  RESERVED5;
  __IO uint16_t CNTH;
  uint16_t  RESERVED6;
  __IO uint16_t CNTL;
  uint16_t  RESERVED7;
  __IO uint16_t ALRH;
  uint16_t  RESERVED8;
  __IO uint16_t ALRL;
  uint16_t  RESERVED9;
} RTC_TypeDef;

/** 
  * @brief SD host Interface
  */

typedef struct
{
  __IO uint32_t POWER;
  __IO uint32_t CLKCR;
  __IO uint32_t ARG;
  __IO uint32_t CMD;
  __I uint32_t RESPCMD;
  __I uint32_t RESP1;
  __I uint32_t RESP2;
  __I uint32_t RESP3;
  __I uint32_t RESP4;
  __IO uint32_t DTIMER;
  __IO uint32_t DLEN;
  __IO uint32_t DCTRL;
  __I uint32_t DCOUNT;
  __I uint32_t STA;
  __IO uint32_t ICR;
  __IO uint32_t MASK;
  uint32_t  RESERVED0[2];
  __I uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  __IO uint32_t FIFO;
} SDIO_TypeDef;

/** 
  * @brief Serial Peripheral Interface
  */

typedef struct
{
  __IO uint16_t CR1;
  uint16_t  RESERVED0;
  __IO uint16_t CR2;
  uint16_t  RESERVED1;
  __IO uint16_t SR;
  uint16_t  RESERVED2;
  __IO uint16_t DR;
  uint16_t  RESERVED3;
  __IO uint16_t CRCPR;
  uint16_t  RESERVED4;
  __IO uint16_t RXCRCR;
  uint16_t  RESERVED5;
  __IO uint16_t TXCRCR;
  uint16_t  RESERVED6;
  __IO uint16_t I2SCFGR;
  uint16_t  RESERVED7;
  __IO uint16_t I2SPR;
  uint16_t  RESERVED8;  
} SPI_TypeDef;

/** 
  * @brief TIM
  */

typedef struct
{
  __IO uint16_t CR1;
  uint16_t  RESERVED0;
  __IO uint16_t CR2;
  uint16_t  RESERVED1;
  __IO uint16_t SMCR;
  uint16_t  RESERVED2;
  __IO uint16_t DIER;
  uint16_t  RESERVED3;
  __IO uint16_t SR;
  uint16_t  RESERVED4;
  __IO uint16_t EGR;
  uint16_t  RESERVED5;
  __IO uint16_t CCMR1;
  uint16_t  RESERVED6;
  __IO uint16_t CCMR2;
  uint16_t  RESERVED7;
  __IO uint16_t CCER;
  uint16_t  RESERVED8;
  __IO uint16_t CNT;
  uint16_t  RESERVED9;
  __IO uint16_t PSC;
  uint16_t  RESERVED10;
  __IO uint16_t ARR;
  uint16_t  RESERVED11;
  __IO uint16_t RCR;
  uint16_t  RESERVED12;
  __IO uint16_t CCR1;
  uint16_t  RESERVED13;
  __IO uint16_t CCR2;
  uint16_t  RESERVED14;
  __IO uint16_t CCR3;
  uint16_t  RESERVED15;
  __IO uint16_t CCR4;
  uint16_t  RESERVED16;
  __IO uint16_t BDTR;
  uint16_t  RESERVED17;
  __IO uint16_t DCR;
  uint16_t  RESERVED18;
  __IO uint16_t DMAR;
  uint16_t  RESERVED19;
} TIM_TypeDef;

/** 
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
 
typedef struct
{
  __IO uint16_t SR;
  uint16_t  RESERVED0;
  __IO uint16_t DR;
  uint16_t  RESERVED1;
  __IO uint16_t BRR;
  uint16_t  RESERVED2;
  __IO uint16_t CR1;
  uint16_t  RESERVED3;
  __IO uint16_t CR2;
  uint16_t  RESERVED4;
  __IO uint16_t CR3;
  uint16_t  RESERVED5;
  __IO uint16_t GTPR;
  uint16_t  RESERVED6;
} USART_TypeDef;

/** 
  * @brief Window WATCHDOG
  */

typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFR;
  __IO uint32_t SR;
} WWDG_TypeDef;

/**
  * @}
  */
  
/** @addtogroup Peripheral_memory_map
  * @{
  */


#define FLASH_BASE            ((uint32_t)0x08000000) /*!< FLASH base address in the alias region */
#define SRAM_BASE             ((uint32_t)0x20000000) /*!< SRAM base address in the alias region */
#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region */

#define SRAM_BB_BASE          ((uint32_t)0x22000000) /*!< SRAM base address in the bit-band region */
#define PERIPH_BB_BASE        ((uint32_t)0x42000000) /*!< Peripheral base address in the bit-band region */

#define FSMC_R_BASE           ((uint32_t)0xA0000000) /*!< FSMC registers base address */

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)

#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400)
#define TIM12_BASE            (APB1PERIPH_BASE + 0x1800)
#define TIM13_BASE            (APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASE            (APB1PERIPH_BASE + 0x2000)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800)
#define CAN1_BASE             (APB1PERIPH_BASE + 0x6400)
#define CAN2_BASE             (APB1PERIPH_BASE + 0x6800)
#define BKP_BASE              (APB1PERIPH_BASE + 0x6C00)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400)
#define CEC_BASE              (APB1PERIPH_BASE + 0x7800)

#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400)
#define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASE            (APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASE            (APB2PERIPH_BASE + 0x2000)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2400)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2800)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x3400)
#define USART1_BASE           (APB2PERIPH_BASE + 0x3800)
#define ADC3_BASE             (APB2PERIPH_BASE + 0x3C00)
#define TIM15_BASE            (APB2PERIPH_BASE + 0x4000)
#define TIM16_BASE            (APB2PERIPH_BASE + 0x4400)
#define TIM17_BASE            (APB2PERIPH_BASE + 0x4800)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4C00)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x5000)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x5400)

#define SDIO_BASE             (PERIPH_BASE + 0x18000)

#define DMA1_BASE             (AHBPERIPH_BASE + 0x0000)
#define DMA1_Channel1_BASE    (AHBPERIPH_BASE + 0x0008)
#define DMA1_Channel2_BASE    (AHBPERIPH_BASE + 0x001C)
#define DMA1_Channel3_BASE    (AHBPERIPH_BASE + 0x0030)
#define DMA1_Channel4_BASE    (AHBPERIPH_BASE + 0x0044)
#define DMA1_Channel5_BASE    (AHBPERIPH_BASE + 0x0058)
#define DMA1_Channel6_BASE    (AHBPERIPH_BASE + 0x006C)
#define DMA1_Channel7_BASE    (AHBPERIPH_BASE + 0x0080)
#define DMA2_BASE             (AHBPERIPH_BASE + 0x0400)
#define DMA2_Channel1_BASE    (AHBPERIPH_BASE + 0x0408)
#define DMA2_Channel2_BASE    (AHBPERIPH_BASE + 0x041C)
#define DMA2_Channel3_BASE    (AHBPERIPH_BASE + 0x0430)
#define DMA2_Channel4_BASE    (AHBPERIPH_BASE + 0x0444)
#define DMA2_Channel5_BASE    (AHBPERIPH_BASE + 0x0458)
#define RCC_BASE              (AHBPERIPH_BASE + 0x1000)
#define CRC_BASE              (AHBPERIPH_BASE + 0x3000)

#define FLASH_R_BASE          (AHBPERIPH_BASE + 0x2000) /*!< Flash registers base address */
#define OB_BASE               ((uint32_t)0x1FFFF800)    /*!< Flash Option Bytes base address */

#define ETH_BASE              (AHBPERIPH_BASE + 0x8000)
#define ETH_MAC_BASE          (ETH_BASE)
#define ETH_MMC_BASE          (ETH_BASE + 0x0100)
#define ETH_PTP_BASE          (ETH_BASE + 0x0700)
#define ETH_DMA_BASE          (ETH_BASE + 0x1000)

#define FSMC_Bank1_R_BASE     (FSMC_R_BASE + 0x0000) /*!< FSMC Bank1 registers base address */
#define FSMC_Bank1E_R_BASE    (FSMC_R_BASE + 0x0104) /*!< FSMC Bank1E registers base address */
#define FSMC_Bank2_R_BASE     (FSMC_R_BASE + 0x0060) /*!< FSMC Bank2 registers base address */
#define FSMC_Bank3_R_BASE     (FSMC_R_BASE + 0x0080) /*!< FSMC Bank3 registers base address */
#define FSMC_Bank4_R_BASE     (FSMC_R_BASE + 0x00A0) /*!< FSMC Bank4 registers base address */

#define DBGMCU_BASE          ((uint32_t)0xE0042000) /*!< Debug MCU registers base address */

/**
  * @}
  */
  
/** @addtogroup Peripheral_declaration
  * @{
  */  

#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define TIM6                ((TIM_TypeDef *) TIM6_BASE)
#define TIM7                ((TIM_TypeDef *) TIM7_BASE)
#define TIM12               ((TIM_TypeDef *) TIM12_BASE)
#define TIM13               ((TIM_TypeDef *) TIM13_BASE)
#define TIM14               ((TIM_TypeDef *) TIM14_BASE)
#define RTC                 ((RTC_TypeDef *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)
#define USART3              ((USART_TypeDef *) USART3_BASE)
#define UART4               ((USART_TypeDef *) UART4_BASE)
#define UART5               ((USART_TypeDef *) UART5_BASE)
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define CAN1                ((CAN_TypeDef *) CAN1_BASE)
#define CAN2                ((CAN_TypeDef *) CAN2_BASE)
#define BKP                 ((BKP_TypeDef *) BKP_BASE)
#define PWR                 ((PWR_TypeDef *) PWR_BASE)
#define DAC                 ((DAC_TypeDef *) DAC_BASE)
#define CEC                 ((CEC_TypeDef *) CEC_BASE)
#define AFIO                ((AFIO_TypeDef *) AFIO_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                ((ADC_TypeDef *) ADC2_BASE)
#define TIM1                ((TIM_TypeDef *) TIM1_BASE)
#define SPI1                ((SPI_TypeDef *) SPI1_BASE)
#define TIM8                ((TIM_TypeDef *) TIM8_BASE)
#define USART1              ((USART_TypeDef *) USART1_BASE)
#define ADC3                ((ADC_TypeDef *) ADC3_BASE)
#define TIM15               ((TIM_TypeDef *) TIM15_BASE)
#define TIM16               ((TIM_TypeDef *) TIM16_BASE)
#define TIM17               ((TIM_TypeDef *) TIM17_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)
#define SDIO                ((SDIO_TypeDef *) SDIO_BASE)
#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)
#define DMA1_Channel1       ((DMA_Channel_TypeDef *) DMA1_Channel1_BASE)
#define DMA1_Channel2       ((DMA_Channel_TypeDef *) DMA1_Channel2_BASE)
#define DMA1_Channel3       ((DMA_Channel_TypeDef *) DMA1_Channel3_BASE)
#define DMA1_Channel4       ((DMA_Channel_TypeDef *) DMA1_Channel4_BASE)
#define DMA1_Channel5       ((DMA_Channel_TypeDef *) DMA1_Channel5_BASE)
#define DMA1_Channel6       ((DMA_Channel_TypeDef *) DMA1_Channel6_BASE)
#define DMA1_Channel7       ((DMA_Channel_TypeDef *) DMA1_Channel7_BASE)
#define DMA2_Channel1       ((DMA_Channel_TypeDef *) DMA2_Channel1_BASE)
#define DMA2_Channel2       ((DMA_Channel_TypeDef *) DMA2_Channel2_BASE)
#define DMA2_Channel3       ((DMA_Channel_TypeDef *) DMA2_Channel3_BASE)
#define DMA2_Channel4       ((DMA_Channel_TypeDef *) DMA2_Channel4_BASE)
#define DMA2_Channel5       ((DMA_Channel_TypeDef *) DMA2_Channel5_BASE)
#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define CRC                 ((CRC_TypeDef *) CRC_BASE)
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)
#define OB                  ((OB_TypeDef *) OB_BASE) 
#define ETH                 ((ETH_TypeDef *) ETH_BASE)
#define FSMC_Bank1          ((FSMC_Bank1_TypeDef *) FSMC_Bank1_R_BASE)
#define FSMC_Bank1E         ((FSMC_Bank1E_TypeDef *) FSMC_Bank1E_R_BASE)
#define FSMC_Bank2          ((FSMC_Bank2_TypeDef *) FSMC_Bank2_R_BASE)
#define FSMC_Bank3          ((FSMC_Bank3_TypeDef *) FSMC_Bank3_R_BASE)
#define FSMC_Bank4          ((FSMC_Bank4_TypeDef *) FSMC_Bank4_R_BASE)
#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)

/**
  * @}
  */

/** @addtogroup Exported_constants
  * @{
  */
  
  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */
    
/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                          CRC calculation unit                              */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for CRC_DR register  *********************/
#define  CRC_DR_DR                           ((uint32_t)0xFFFFFFFF) /*!< Data register bits */


/*******************  Bit definition for CRC_IDR register  ********************/
#define  CRC_IDR_IDR                         ((uint8_t)0xFF)        /*!< General-purpose 8-bit data register bits */


/********************  Bit definition for CRC_CR register  ********************/
#define  CRC_CR_RESET                        ((uint8_t)0x01)        /*!< RESET bit */

/******************************************************************************/
/*                                                                            */
/*                             Power Control                                  */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for PWR_CR register  ********************/
#define  PWR_CR_LPDS                         ((uint16_t)0x0001)     /*!< Low-Power Deepsleep */
#define  PWR_CR_PDDS                         ((uint16_t)0x0002)     /*!< Power Down Deepsleep */
#define  PWR_CR_CWUF                         ((uint16_t)0x0004)     /*!< Clear Wakeup Flag */
#define  PWR_CR_CSBF                         ((uint16_t)0x0008)     /*!< Clear Standby Flag */
#define  PWR_CR_PVDE                         ((uint16_t)0x0010)     /*!< Power Voltage Detector Enable */

#define  PWR_CR_PLS                          ((uint16_t)0x00E0)     /*!< PLS[2:0] bits (PVD Level Selection) */
#define  PWR_CR_PLS_0                        ((uint16_t)0x0020)     /*!< Bit 0 */
#define  PWR_CR_PLS_1                        ((uint16_t)0x0040)     /*!< Bit 1 */
#define  PWR_CR_PLS_2                        ((uint16_t)0x0080)     /*!< Bit 2 */

/*!< PVD level configuration */
#define  PWR_CR_PLS_2V2                      ((uint16_t)0x0000)     /*!< PVD level 2.2V */
#define  PWR_CR_PLS_2V3                      ((uint16_t)0x0020)     /*!< PVD level 2.3V */
#define  PWR_CR_PLS_2V4                      ((uint16_t)0x0040)     /*!< PVD level 2.4V */
#define  PWR_CR_PLS_2V5                      ((uint16_t)0x0060)     /*!< PVD level 2.5V */
#define  PWR_CR_PLS_2V6                      ((uint16_t)0x0080)     /*!< PVD level 2.6V */
#define  PWR_CR_PLS_2V7                      ((uint16_t)0x00A0)     /*!< PVD level 2.7V */
#define  PWR_CR_PLS_2V8                      ((uint16_t)0x00C0)     /*!< PVD level 2.8V */
#define  PWR_CR_PLS_2V9                      ((uint16_t)0x00E0)     /*!< PVD level 2.9V */

#define  PWR_CR_DBP                          ((uint16_t)0x0100)     /*!< Disable Backup Domain write protection */


/*******************  Bit definition for PWR_CSR register  ********************/
#define  PWR_CSR_WUF                         ((uint16_t)0x0001)     /*!< Wakeup Flag */
#define  PWR_CSR_SBF                         ((uint16_t)0x0002)     /*!< Standby Flag */
#define  PWR_CSR_PVDO                        ((uint16_t)0x0004)     /*!< PVD Output */
#define  PWR_CSR_EWUP                        ((uint16_t)0x0100)     /*!< Enable WKUP pin */

/******************************************************************************/
/*                                                                            */
/*                            Backup registers                                */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for BKP_DR1 register  ********************/
#define  BKP_DR1_D                           ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR2 register  ********************/
#define  BKP_DR2_D                           ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR3 register  ********************/
#define  BKP_DR3_D                           ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR4 register  ********************/
#define  BKP_DR4_D                           ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR5 register  ********************/
#define  BKP_DR5_D                           ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR6 register  ********************/
#define  BKP_DR6_D                           ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR7 register  ********************/
#define  BKP_DR7_D                           ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR8 register  ********************/
#define  BKP_DR8_D                           ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR9 register  ********************/
#define  BKP_DR9_D                           ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR10 register  *******************/
#define  BKP_DR10_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR11 register  *******************/
#define  BKP_DR11_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR12 register  *******************/
#define  BKP_DR12_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR13 register  *******************/
#define  BKP_DR13_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR14 register  *******************/
#define  BKP_DR14_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR15 register  *******************/
#define  BKP_DR15_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR16 register  *******************/
#define  BKP_DR16_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR17 register  *******************/
#define  BKP_DR17_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/******************  Bit definition for BKP_DR18 register  ********************/
#define  BKP_DR18_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR19 register  *******************/
#define  BKP_DR19_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR20 register  *******************/
#define  BKP_DR20_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR21 register  *******************/
#define  BKP_DR21_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR22 register  *******************/
#define  BKP_DR22_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR23 register  *******************/
#define  BKP_DR23_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR24 register  *******************/
#define  BKP_DR24_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR25 register  *******************/
#define  BKP_DR25_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR26 register  *******************/
#define  BKP_DR26_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR27 register  *******************/
#define  BKP_DR27_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR28 register  *******************/
#define  BKP_DR28_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR29 register  *******************/
#define  BKP_DR29_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR30 register  *******************/
#define  BKP_DR30_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR31 register  *******************/
#define  BKP_DR31_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR32 register  *******************/
#define  BKP_DR32_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR33 register  *******************/
#define  BKP_DR33_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR34 register  *******************/
#define  BKP_DR34_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR35 register  *******************/
#define  BKP_DR35_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR36 register  *******************/
#define  BKP_DR36_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR37 register  *******************/
#define  BKP_DR37_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR38 register  *******************/
#define  BKP_DR38_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR39 register  *******************/
#define  BKP_DR39_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR40 register  *******************/
#define  BKP_DR40_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR41 register  *******************/
#define  BKP_DR41_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/*******************  Bit definition for BKP_DR42 register  *******************/
#define  BKP_DR42_D                          ((uint16_t)0xFFFF)     /*!< Backup data */

/******************  Bit definition for BKP_RTCCR register  *******************/
#define  BKP_RTCCR_CAL                       ((uint16_t)0x007F)     /*!< Calibration value */
#define  BKP_RTCCR_CCO                       ((uint16_t)0x0080)     /*!< Calibration Clock Output */
#define  BKP_RTCCR_ASOE                      ((uint16_t)0x0100)     /*!< Alarm or Second Output Enable */
#define  BKP_RTCCR_ASOS                      ((uint16_t)0x0200)     /*!< Alarm or Second Output Selection */

/********************  Bit definition for BKP_CR register  ********************/
#define  BKP_CR_TPE                          ((uint8_t)0x01)        /*!< TAMPER pin enable */
#define  BKP_CR_TPAL                         ((uint8_t)0x02)        /*!< TAMPER pin active level */

/*******************  Bit definition for BKP_CSR register  ********************/
#define  BKP_CSR_CTE                         ((uint16_t)0x0001)     /*!< Clear Tamper event */
#define  BKP_CSR_CTI                         ((uint16_t)0x0002)     /*!< Clear Tamper Interrupt */
#define  BKP_CSR_TPIE                        ((uint16_t)0x0004)     /*!< TAMPER Pin interrupt enable */
#define  BKP_CSR_TEF                         ((uint16_t)0x0100)     /*!< Tamper Event Flag */
#define  BKP_CSR_TIF                         ((uint16_t)0x0200)     /*!< Tamper Interrupt Flag */

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_HSION                        ((uint32_t)0x00000001)        /*!< Internal High Speed clock enable */
#define  RCC_CR_HSIRDY                       ((uint32_t)0x00000002)        /*!< Internal High Speed clock ready flag */
#define  RCC_CR_HSITRIM                      ((uint32_t)0x000000F8)        /*!< Internal High Speed clock trimming */
#define  RCC_CR_HSICAL                       ((uint32_t)0x0000FF00)        /*!< Internal High Speed clock Calibration */
#define  RCC_CR_HSEON                        ((uint32_t)0x00010000)        /*!< External High Speed clock enable */
#define  RCC_CR_HSERDY                       ((uint32_t)0x00020000)        /*!< External High Speed clock ready flag */
#define  RCC_CR_HSEBYP                       ((uint32_t)0x00040000)        /*!< External High Speed clock Bypass */
#define  RCC_CR_CSSON                        ((uint32_t)0x00080000)        /*!< Clock Security System enable */
#define  RCC_CR_PLLON                        ((uint32_t)0x01000000)        /*!< PLL enable */
#define  RCC_CR_PLLRDY                       ((uint32_t)0x02000000)        /*!< PLL clock ready flag */

#ifdef STM32F10X_CL
 #define  RCC_CR_PLL2ON                       ((uint32_t)0x04000000)        /*!< PLL2 enable */
 #define  RCC_CR_PLL2RDY                      ((uint32_t)0x08000000)        /*!< PLL2 clock ready flag */
 #define  RCC_CR_PLL3ON                       ((uint32_t)0x10000000)        /*!< PLL3 enable */
 #define  RCC_CR_PLL3RDY                      ((uint32_t)0x20000000)        /*!< PLL3 clock ready flag */
#endif /* STM32F10X_CL */

/*******************  Bit definition for RCC_CFGR register  *******************/
/*!< SW configuration */
#define  RCC_CFGR_SW                         ((uint32_t)0x00000003)        /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  RCC_CFGR_SW_1                       ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  RCC_CFGR_SW_HSI                     ((uint32_t)0x00000000)        /*!< HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     ((uint32_t)0x00000001)        /*!< HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     ((uint32_t)0x00000002)        /*!< PLL selected as system clock */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        ((uint32_t)0x0000000C)        /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  RCC_CFGR_SWS_1                      ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  RCC_CFGR_SWS_HSI                    ((uint32_t)0x00000000)        /*!< HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((uint32_t)0x00000004)        /*!< HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((uint32_t)0x00000008)        /*!< PLL used as system clock */

/*!< HPRE configuration */
#define  RCC_CFGR_HPRE                       ((uint32_t)0x000000F0)        /*!< HPRE[3:0] bits (AHB prescaler) */
#define  RCC_CFGR_HPRE_0                     ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  RCC_CFGR_HPRE_1                     ((uint32_t)0x00000020)        /*!< Bit 1 */
#define  RCC_CFGR_HPRE_2                     ((uint32_t)0x00000040)        /*!< Bit 2 */
#define  RCC_CFGR_HPRE_3                     ((uint32_t)0x00000080)        /*!< Bit 3 */

#define  RCC_CFGR_HPRE_DIV1                  ((uint32_t)0x00000000)        /*!< SYSCLK not divided */
#define  RCC_CFGR_HPRE_DIV2                  ((uint32_t)0x00000080)        /*!< SYSCLK divided by 2 */
#define  RCC_CFGR_HPRE_DIV4                  ((uint32_t)0x00000090)        /*!< SYSCLK divided by 4 */
#define  RCC_CFGR_HPRE_DIV8                  ((uint32_t)0x000000A0)        /*!< SYSCLK divided by 8 */
#define  RCC_CFGR_HPRE_DIV16                 ((uint32_t)0x000000B0)        /*!< SYSCLK divided by 16 */
#define  RCC_CFGR_HPRE_DIV64                 ((uint32_t)0x000000C0)        /*!< SYSCLK divided by 64 */
#define  RCC_CFGR_HPRE_DIV128                ((uint32_t)0x000000D0)        /*!< SYSCLK divided by 128 */
#define  RCC_CFGR_HPRE_DIV256                ((uint32_t)0x000000E0)        /*!< SYSCLK divided by 256 */
#define  RCC_CFGR_HPRE_DIV512                ((uint32_t)0x000000F0)        /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define  RCC_CFGR_PPRE1                      ((uint32_t)0x00000700)        /*!< PRE1[2:0] bits (APB1 prescaler) */
#define  RCC_CFGR_PPRE1_0                    ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE1_1                    ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE1_2                    ((uint32_t)0x00000400)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE1_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE1_DIV2                 ((uint32_t)0x00000400)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE1_DIV4                 ((uint32_t)0x00000500)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE1_DIV8                 ((uint32_t)0x00000600)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE1_DIV16                ((uint32_t)0x00000700)        /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define  RCC_CFGR_PPRE2                      ((uint32_t)0x00003800)        /*!< PRE2[2:0] bits (APB2 prescaler) */
#define  RCC_CFGR_PPRE2_0                    ((uint32_t)0x00000800)        /*!< Bit 0 */
#define  RCC_CFGR_PPRE2_1                    ((uint32_t)0x00001000)        /*!< Bit 1 */
#define  RCC_CFGR_PPRE2_2                    ((uint32_t)0x00002000)        /*!< Bit 2 */

#define  RCC_CFGR_PPRE2_DIV1                 ((uint32_t)0x00000000)        /*!< HCLK not divided */
#define  RCC_CFGR_PPRE2_DIV2                 ((uint32_t)0x00002000)        /*!< HCLK divided by 2 */
#define  RCC_CFGR_PPRE2_DIV4                 ((uint32_t)0x00002800)        /*!< HCLK divided by 4 */
#define  RCC_CFGR_PPRE2_DIV8                 ((uint32_t)0x00003000)        /*!< HCLK divided by 8 */
#define  RCC_CFGR_PPRE2_DIV16                ((uint32_t)0x00003800)        /*!< HCLK divided by 16 */

/*!< ADCPPRE configuration */
#define  RCC_CFGR_ADCPRE                     ((uint32_t)0x0000C000)        /*!< ADCPRE[1:0] bits (ADC prescaler) */
#define  RCC_CFGR_ADCPRE_0                   ((uint32_t)0x00004000)        /*!< Bit 0 */
#define  RCC_CFGR_ADCPRE_1                   ((uint32_t)0x00008000)        /*!< Bit 1 */

#define  RCC_CFGR_ADCPRE_DIV2                ((uint32_t)0x00000000)        /*!< PCLK2 divided by 2 */
#define  RCC_CFGR_ADCPRE_DIV4                ((uint32_t)0x00004000)        /*!< PCLK2 divided by 4 */
#define  RCC_CFGR_ADCPRE_DIV6                ((uint32_t)0x00008000)        /*!< PCLK2 divided by 6 */
#define  RCC_CFGR_ADCPRE_DIV8                ((uint32_t)0x0000C000)        /*!< PCLK2 divided by 8 */

#define  RCC_CFGR_PLLSRC                     ((uint32_t)0x00010000)        /*!< PLL entry clock source */

#define  RCC_CFGR_PLLXTPRE                   ((uint32_t)0x00020000)        /*!< HSE divider for PLL entry */

/*!< PLLMUL configuration */
#define  RCC_CFGR_PLLMULL                    ((uint32_t)0x003C0000)        /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define  RCC_CFGR_PLLMULL_0                  ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  RCC_CFGR_PLLMULL_1                  ((uint32_t)0x00080000)        /*!< Bit 1 */
#define  RCC_CFGR_PLLMULL_2                  ((uint32_t)0x00100000)        /*!< Bit 2 */
#define  RCC_CFGR_PLLMULL_3                  ((uint32_t)0x00200000)        /*!< Bit 3 */

#ifdef STM32F10X_CL
 #define  RCC_CFGR_PLLSRC_HSI_Div2           ((uint32_t)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
 #define  RCC_CFGR_PLLSRC_PREDIV1            ((uint32_t)0x00010000)        /*!< PREDIV1 clock selected as PLL entry clock source */

 #define  RCC_CFGR_PLLXTPRE_PREDIV1          ((uint32_t)0x00000000)        /*!< PREDIV1 clock not divided for PLL entry */
 #define  RCC_CFGR_PLLXTPRE_PREDIV1_Div2     ((uint32_t)0x00020000)        /*!< PREDIV1 clock divided by 2 for PLL entry */

 #define  RCC_CFGR_PLLMULL4                  ((uint32_t)0x00080000)        /*!< PLL input clock * 4 */
 #define  RCC_CFGR_PLLMULL5                  ((uint32_t)0x000C0000)        /*!< PLL input clock * 5 */
 #define  RCC_CFGR_PLLMULL6                  ((uint32_t)0x00100000)        /*!< PLL input clock * 6 */
 #define  RCC_CFGR_PLLMULL7                  ((uint32_t)0x00140000)        /*!< PLL input clock * 7 */
 #define  RCC_CFGR_PLLMULL8                  ((uint32_t)0x00180000)        /*!< PLL input clock * 8 */
 #define  RCC_CFGR_PLLMULL9                  ((uint32_t)0x001C0000)        /*!< PLL input clock * 9 */
 #define  RCC_CFGR_PLLMULL6_5                ((uint32_t)0x00340000)        /*!< PLL input clock * 6.5 */
 
 #define  RCC_CFGR_OTGFSPRE                  ((uint32_t)0x00400000)        /*!< USB OTG FS prescaler */
 
/*!< MCO configuration */
 #define  RCC_CFGR_MCO                       ((uint32_t)0x0F000000)        /*!< MCO[3:0] bits (Microcontroller Clock Output) */
 #define  RCC_CFGR_MCO_0                     ((uint32_t)0x01000000)        /*!< Bit 0 */
 #define  RCC_CFGR_MCO_1                     ((uint32_t)0x02000000)        /*!< Bit 1 */
 #define  RCC_CFGR_MCO_2                     ((uint32_t)0x04000000)        /*!< Bit 2 */
 #define  RCC_CFGR_MCO_3                     ((uint32_t)0x08000000)        /*!< Bit 3 */

 #define  RCC_CFGR_MCO_NOCLOCK               ((uint32_t)0x00000000)        /*!< No clock */
 #define  RCC_CFGR_MCO_SYSCLK                ((uint32_t)0x04000000)        /*!< System clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSI                   ((uint32_t)0x05000000)        /*!< HSI clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSE                   ((uint32_t)0x06000000)        /*!< HSE clock selected as MCO source */
 #define  RCC_CFGR_MCO_PLLCLK_Div2           ((uint32_t)0x07000000)        /*!< PLL clock divided by 2 selected as MCO source */
 #define  RCC_CFGR_MCO_PLL2CLK               ((uint32_t)0x08000000)        /*!< PLL2 clock selected as MCO source*/
 #define  RCC_CFGR_MCO_PLL3CLK_Div2          ((uint32_t)0x09000000)        /*!< PLL3 clock divided by 2 selected as MCO source*/
 #define  RCC_CFGR_MCO_Ext_HSE               ((uint32_t)0x0A000000)        /*!< XT1 external 3-25 MHz oscillator clock selected as MCO source */
 #define  RCC_CFGR_MCO_PLL3CLK               ((uint32_t)0x0B000000)        /*!< PLL3 clock selected as MCO source */
#elif defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
 #define  RCC_CFGR_PLLSRC_HSI_Div2           ((uint32_t)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
 #define  RCC_CFGR_PLLSRC_PREDIV1            ((uint32_t)0x00010000)        /*!< PREDIV1 clock selected as PLL entry clock source */

 #define  RCC_CFGR_PLLXTPRE_PREDIV1          ((uint32_t)0x00000000)        /*!< PREDIV1 clock not divided for PLL entry */
 #define  RCC_CFGR_PLLXTPRE_PREDIV1_Div2     ((uint32_t)0x00020000)        /*!< PREDIV1 clock divided by 2 for PLL entry */

 #define  RCC_CFGR_PLLMULL2                  ((uint32_t)0x00000000)        /*!< PLL input clock*2 */
 #define  RCC_CFGR_PLLMULL3                  ((uint32_t)0x00040000)        /*!< PLL input clock*3 */
 #define  RCC_CFGR_PLLMULL4                  ((uint32_t)0x00080000)        /*!< PLL input clock*4 */
 #define  RCC_CFGR_PLLMULL5                  ((uint32_t)0x000C0000)        /*!< PLL input clock*5 */
 #define  RCC_CFGR_PLLMULL6                  ((uint32_t)0x00100000)        /*!< PLL input clock*6 */
 #define  RCC_CFGR_PLLMULL7                  ((uint32_t)0x00140000)        /*!< PLL input clock*7 */
 #define  RCC_CFGR_PLLMULL8                  ((uint32_t)0x00180000)        /*!< PLL input clock*8 */
 #define  RCC_CFGR_PLLMULL9                  ((uint32_t)0x001C0000)        /*!< PLL input clock*9 */
 #define  RCC_CFGR_PLLMULL10                 ((uint32_t)0x00200000)        /*!< PLL input clock10 */
 #define  RCC_CFGR_PLLMULL11                 ((uint32_t)0x00240000)        /*!< PLL input clock*11 */
 #define  RCC_CFGR_PLLMULL12                 ((uint32_t)0x00280000)        /*!< PLL input clock*12 */
 #define  RCC_CFGR_PLLMULL13                 ((uint32_t)0x002C0000)        /*!< PLL input clock*13 */
 #define  RCC_CFGR_PLLMULL14                 ((uint32_t)0x00300000)        /*!< PLL input clock*14 */
 #define  RCC_CFGR_PLLMULL15                 ((uint32_t)0x00340000)        /*!< PLL input clock*15 */
 #define  RCC_CFGR_PLLMULL16                 ((uint32_t)0x00380000)        /*!< PLL input clock*16 */

/*!< MCO configuration */
 #define  RCC_CFGR_MCO                       ((uint32_t)0x07000000)        /*!< MCO[2:0] bits (Microcontroller Clock Output) */
 #define  RCC_CFGR_MCO_0                     ((uint32_t)0x01000000)        /*!< Bit 0 */
 #define  RCC_CFGR_MCO_1                     ((uint32_t)0x02000000)        /*!< Bit 1 */
 #define  RCC_CFGR_MCO_2                     ((uint32_t)0x04000000)        /*!< Bit 2 */

 #define  RCC_CFGR_MCO_NOCLOCK               ((uint32_t)0x00000000)        /*!< No clock */
 #define  RCC_CFGR_MCO_SYSCLK                ((uint32_t)0x04000000)        /*!< System clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSI                   ((uint32_t)0x05000000)        /*!< HSI clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSE                   ((uint32_t)0x06000000)        /*!< HSE clock selected as MCO source  */
 #define  RCC_CFGR_MCO_PLL                   ((uint32_t)0x07000000)        /*!< PLL clock divided by 2 selected as MCO source */
#else
 #define  RCC_CFGR_PLLSRC_HSI_Div2           ((uint32_t)0x00000000)        /*!< HSI clock divided by 2 selected as PLL entry clock source */
 #define  RCC_CFGR_PLLSRC_HSE                ((uint32_t)0x00010000)        /*!< HSE clock selected as PLL entry clock source */

 #define  RCC_CFGR_PLLXTPRE_HSE              ((uint32_t)0x00000000)        /*!< HSE clock not divided for PLL entry */
 #define  RCC_CFGR_PLLXTPRE_HSE_Div2         ((uint32_t)0x00020000)        /*!< HSE clock divided by 2 for PLL entry */

 #define  RCC_CFGR_PLLMULL2                  ((uint32_t)0x00000000)        /*!< PLL input clock*2 */
 #define  RCC_CFGR_PLLMULL3                  ((uint32_t)0x00040000)        /*!< PLL input clock*3 */
 #define  RCC_CFGR_PLLMULL4                  ((uint32_t)0x00080000)        /*!< PLL input clock*4 */
 #define  RCC_CFGR_PLLMULL5                  ((uint32_t)0x000C0000)        /*!< PLL input clock*5 */
 #define  RCC_CFGR_PLLMULL6                  ((uint32_t)0x00100000)        /*!< PLL input clock*6 */
 #define  RCC_CFGR_PLLMULL7                  ((uint32_t)0x00140000)        /*!< PLL input clock*7 */
 #define  RCC_CFGR_PLLMULL8                  ((uint32_t)0x00180000)        /*!< PLL input clock*8 */
 #define  RCC_CFGR_PLLMULL9                  ((uint32_t)0x001C0000)        /*!< PLL input clock*9 */
 #define  RCC_CFGR_PLLMULL10                 ((uint32_t)0x00200000)        /*!< PLL input clock10 */
 #define  RCC_CFGR_PLLMULL11                 ((uint32_t)0x00240000)        /*!< PLL input clock*11 */
 #define  RCC_CFGR_PLLMULL12                 ((uint32_t)0x00280000)        /*!< PLL input clock*12 */
 #define  RCC_CFGR_PLLMULL13                 ((uint32_t)0x002C0000)        /*!< PLL input clock*13 */
 #define  RCC_CFGR_PLLMULL14                 ((uint32_t)0x00300000)        /*!< PLL input clock*14 */
 #define  RCC_CFGR_PLLMULL15                 ((uint32_t)0x00340000)        /*!< PLL input clock*15 */
 #define  RCC_CFGR_PLLMULL16                 ((uint32_t)0x00380000)        /*!< PLL input clock*16 */
 #define  RCC_CFGR_USBPRE                    ((uint32_t)0x00400000)        /*!< USB Device prescaler */

/*!< MCO configuration */
 #define  RCC_CFGR_MCO                       ((uint32_t)0x07000000)        /*!< MCO[2:0] bits (Microcontroller Clock Output) */
 #define  RCC_CFGR_MCO_0                     ((uint32_t)0x01000000)        /*!< Bit 0 */
 #define  RCC_CFGR_MCO_1                     ((uint32_t)0x02000000)        /*!< Bit 1 */
 #define  RCC_CFGR_MCO_2                     ((uint32_t)0x04000000)        /*!< Bit 2 */

 #define  RCC_CFGR_MCO_NOCLOCK               ((uint32_t)0x00000000)        /*!< No clock */
 #define  RCC_CFGR_MCO_SYSCLK                ((uint32_t)0x04000000)        /*!< System clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSI                   ((uint32_t)0x05000000)        /*!< HSI clock selected as MCO source */
 #define  RCC_CFGR_MCO_HSE                   ((uint32_t)0x06000000)        /*!< HSE clock selected as MCO source  */
 #define  RCC_CFGR_MCO_PLL                   ((uint32_t)0x07000000)        /*!< PLL clock divided by 2 selected as MCO source */
#endif /* STM32F10X_CL */

/*!<******************  Bit definition for RCC_CIR register  ********************/
#define  RCC_CIR_LSIRDYF                     ((uint32_t)0x00000001)        /*!< LSI Ready Interrupt flag */
#define  RCC_CIR_LSERDYF                     ((uint32_t)0x00000002)        /*!< LSE Ready Interrupt flag */
#define  RCC_CIR_HSIRDYF                     ((uint32_t)0x00000004)        /*!< HSI Ready Interrupt flag */
#define  RCC_CIR_HSERDYF                     ((uint32_t)0x00000008)        /*!< HSE Ready Interrupt flag */
#define  RCC_CIR_PLLRDYF                     ((uint32_t)0x00000010)        /*!< PLL Ready Interrupt flag */
#define  RCC_CIR_CSSF                        ((uint32_t)0x00000080)        /*!< Clock Security System Interrupt flag */
#define  RCC_CIR_LSIRDYIE                    ((uint32_t)0x00000100)        /*!< LSI Ready Interrupt Enable */
#define  RCC_CIR_LSERDYIE                    ((uint32_t)0x00000200)        /*!< LSE Ready Interrupt Enable */
#define  RCC_CIR_HSIRDYIE                    ((uint32_t)0x00000400)        /*!< HSI Ready Interrupt Enable */
#define  RCC_CIR_HSERDYIE                    ((uint32_t)0x00000800)        /*!< HSE Ready Interrupt Enable */
#define  RCC_CIR_PLLRDYIE                    ((uint32_t)0x00001000)        /*!< PLL Ready Interrupt Enable */
#define  RCC_CIR_LSIRDYC                     ((uint32_t)0x00010000)        /*!< LSI Ready Interrupt Clear */
#define  RCC_CIR_LSERDYC                     ((uint32_t)0x00020000)        /*!< LSE Ready Interrupt Clear */
#define  RCC_CIR_HSIRDYC                     ((uint32_t)0x00040000)        /*!< HSI Ready Interrupt Clear */
#define  RCC_CIR_HSERDYC                     ((uint32_t)0x00080000)        /*!< HSE Ready Interrupt Clear */
#define  RCC_CIR_PLLRDYC                     ((uint32_t)0x00100000)        /*!< PLL Ready Interrupt Clear */
#define  RCC_CIR_CSSC                        ((uint32_t)0x00800000)        /*!< Clock Security System Interrupt Clear */

#ifdef STM32F10X_CL
 #define  RCC_CIR_PLL2RDYF                    ((uint32_t)0x00000020)        /*!< PLL2 Ready Interrupt flag */
 #define  RCC_CIR_PLL3RDYF                    ((uint32_t)0x00000040)        /*!< PLL3 Ready Interrupt flag */
 #define  RCC_CIR_PLL2RDYIE                   ((uint32_t)0x00002000)        /*!< PLL2 Ready Interrupt Enable */
 #define  RCC_CIR_PLL3RDYIE                   ((uint32_t)0x00004000)        /*!< PLL3 Ready Interrupt Enable */
 #define  RCC_CIR_PLL2RDYC                    ((uint32_t)0x00200000)        /*!< PLL2 Ready Interrupt Clear */
 #define  RCC_CIR_PLL3RDYC                    ((uint32_t)0x00400000)        /*!< PLL3 Ready Interrupt Clear */
#endif /* STM32F10X_CL */

/*****************  Bit definition for RCC_APB2RSTR register  *****************/
#define  RCC_APB2RSTR_AFIORST                ((uint32_t)0x00000001)        /*!< Alternate Function I/O reset */
#define  RCC_APB2RSTR_IOPARST                ((uint32_t)0x00000004)        /*!< I/O port A reset */
#define  RCC_APB2RSTR_IOPBRST                ((uint32_t)0x00000008)        /*!< I/O port B reset */
#define  RCC_APB2RSTR_IOPCRST                ((uint32_t)0x00000010)        /*!< I/O port C reset */
#define  RCC_APB2RSTR_IOPDRST                ((uint32_t)0x00000020)        /*!< I/O port D reset */
#define  RCC_APB2RSTR_ADC1RST                ((uint32_t)0x00000200)        /*!< ADC 1 interface reset */

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD_VL)
#define  RCC_APB2RSTR_ADC2RST                ((uint32_t)0x00000400)        /*!< ADC 2 interface reset */
#endif

#define  RCC_APB2RSTR_TIM1RST                ((uint32_t)0x00000800)        /*!< TIM1 Timer reset */
#define  RCC_APB2RSTR_SPI1RST                ((uint32_t)0x00001000)        /*!< SPI 1 reset */
#define  RCC_APB2RSTR_USART1RST              ((uint32_t)0x00004000)        /*!< USART1 reset */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
#define  RCC_APB2RSTR_TIM15RST               ((uint32_t)0x00010000)        /*!< TIM15 Timer reset */
#define  RCC_APB2RSTR_TIM16RST               ((uint32_t)0x00020000)        /*!< TIM16 Timer reset */
#define  RCC_APB2RSTR_TIM17RST               ((uint32_t)0x00040000)        /*!< TIM17 Timer reset */
#endif

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
 #define  RCC_APB2RSTR_IOPERST               ((uint32_t)0x00000040)        /*!< I/O port E reset */
#endif /* STM32F10X_LD && STM32F10X_LD_VL */

#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
 #define  RCC_APB2RSTR_IOPFRST               ((uint32_t)0x00000080)        /*!< I/O port F reset */
 #define  RCC_APB2RSTR_IOPGRST               ((uint32_t)0x00000100)        /*!< I/O port G reset */
 #define  RCC_APB2RSTR_TIM8RST               ((uint32_t)0x00002000)        /*!< TIM8 Timer reset */
 #define  RCC_APB2RSTR_ADC3RST               ((uint32_t)0x00008000)        /*!< ADC3 interface reset */
#endif

#if defined (STM32F10X_HD_VL)
 #define  RCC_APB2RSTR_IOPFRST               ((uint32_t)0x00000080)        /*!< I/O port F reset */
 #define  RCC_APB2RSTR_IOPGRST               ((uint32_t)0x00000100)        /*!< I/O port G reset */
#endif

#ifdef STM32F10X_XL
 #define  RCC_APB2RSTR_TIM9RST               ((uint32_t)0x00080000)         /*!< TIM9 Timer reset */
 #define  RCC_APB2RSTR_TIM10RST              ((uint32_t)0x00100000)         /*!< TIM10 Timer reset */
 #define  RCC_APB2RSTR_TIM11RST              ((uint32_t)0x00200000)         /*!< TIM11 Timer reset */
#endif /* STM32F10X_XL */

/*****************  Bit definition for RCC_APB1RSTR register  *****************/
#define  RCC_APB1RSTR_TIM2RST                ((uint32_t)0x00000001)        /*!< Timer 2 reset */
#define  RCC_APB1RSTR_TIM3RST                ((uint32_t)0x00000002)        /*!< Timer 3 reset */
#define  RCC_APB1RSTR_WWDGRST                ((uint32_t)0x00000800)        /*!< Window Watchdog reset */
#define  RCC_APB1RSTR_USART2RST              ((uint32_t)0x00020000)        /*!< USART 2 reset */
#define  RCC_APB1RSTR_I2C1RST                ((uint32_t)0x00200000)        /*!< I2C 1 reset */

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD_VL)
#define  RCC_APB1RSTR_CAN1RST                ((uint32_t)0x02000000)        /*!< CAN1 reset */
#endif

#define  RCC_APB1RSTR_BKPRST                 ((uint32_t)0x08000000)        /*!< Backup interface reset */
#define  RCC_APB1RSTR_PWRRST                 ((uint32_t)0x10000000)        /*!< Power interface reset */

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
 #define  RCC_APB1RSTR_TIM4RST               ((uint32_t)0x00000004)        /*!< Timer 4 reset */
 #define  RCC_APB1RSTR_SPI2RST               ((uint32_t)0x00004000)        /*!< SPI 2 reset */
 #define  RCC_APB1RSTR_USART3RST             ((uint32_t)0x00040000)        /*!< USART 3 reset */
 #define  RCC_APB1RSTR_I2C2RST               ((uint32_t)0x00400000)        /*!< I2C 2 reset */
#endif /* STM32F10X_LD && STM32F10X_LD_VL */

#if defined (STM32F10X_HD) || defined (STM32F10X_MD) || defined (STM32F10X_LD) || defined  (STM32F10X_XL)
 #define  RCC_APB1RSTR_USBRST                ((uint32_t)0x00800000)        /*!< USB Device reset */
#endif

#if defined (STM32F10X_HD) || defined  (STM32F10X_CL) || defined  (STM32F10X_XL)
 #define  RCC_APB1RSTR_TIM5RST                ((uint32_t)0x00000008)        /*!< Timer 5 reset */
 #define  RCC_APB1RSTR_TIM6RST                ((uint32_t)0x00000010)        /*!< Timer 6 reset */
 #define  RCC_APB1RSTR_TIM7RST                ((uint32_t)0x00000020)        /*!< Timer 7 reset */
 #define  RCC_APB1RSTR_SPI3RST                ((uint32_t)0x00008000)        /*!< SPI 3 reset */
 #define  RCC_APB1RSTR_UART4RST               ((uint32_t)0x00080000)        /*!< UART 4 reset */
 #define  RCC_APB1RSTR_UART5RST               ((uint32_t)0x00100000)        /*!< UART 5 reset */
 #define  RCC_APB1RSTR_DACRST                 ((uint32_t)0x20000000)        /*!< DAC interface reset */
#endif

#if defined (STM32F10X_LD_VL) || defined  (STM32F10X_MD_VL) || defined  (STM32F10X_HD_VL)
 #define  RCC_APB1RSTR_TIM6RST                ((uint32_t)0x00000010)        /*!< Timer 6 reset */
 #define  RCC_APB1RSTR_TIM7RST                ((uint32_t)0x00000020)        /*!< Timer 7 reset */
 #define  RCC_APB1RSTR_DACRST                 ((uint32_t)0x20000000)        /*!< DAC interface reset */
 #define  RCC_APB1RSTR_CECRST                 ((uint32_t)0x40000000)        /*!< CEC interface reset */ 
#endif

#if defined  (STM32F10X_HD_VL)
 #define  RCC_APB1RSTR_TIM5RST                ((uint32_t)0x00000008)        /*!< Timer 5 reset */
 #define  RCC_APB1RSTR_TIM12RST               ((uint32_t)0x00000040)        /*!< TIM12 Timer reset */
 #define  RCC_APB1RSTR_TIM13RST               ((uint32_t)0x00000080)        /*!< TIM13 Timer reset */
 #define  RCC_APB1RSTR_TIM14RST               ((uint32_t)0x00000100)        /*!< TIM14 Timer reset */
 #define  RCC_APB1RSTR_SPI3RST                ((uint32_t)0x00008000)        /*!< SPI 3 reset */ 
 #define  RCC_APB1RSTR_UART4RST               ((uint32_t)0x00080000)        /*!< UART 4 reset */
 #define  RCC_APB1RSTR_UART5RST               ((uint32_t)0x00100000)        /*!< UART 5 reset */ 
#endif

#ifdef STM32F10X_CL
 #define  RCC_APB1RSTR_CAN2RST                ((uint32_t)0x04000000)        /*!< CAN2 reset */
#endif /* STM32F10X_CL */

#ifdef STM32F10X_XL
 #define  RCC_APB1RSTR_TIM12RST               ((uint32_t)0x00000040)         /*!< TIM12 Timer reset */
 #define  RCC_APB1RSTR_TIM13RST               ((uint32_t)0x00000080)         /*!< TIM13 Timer reset */
 #define  RCC_APB1RSTR_TIM14RST               ((uint32_t)0x00000100)         /*!< TIM14 Timer reset */
#endif /* STM32F10X_XL */

/******************  Bit definition for RCC_AHBENR register  ******************/
#define  RCC_AHBENR_DMA1EN                   ((uint16_t)0x0001)            /*!< DMA1 clock enable */
#define  RCC_AHBENR_SRAMEN                   ((uint16_t)0x0004)            /*!< SRAM interface clock enable */
#define  RCC_AHBENR_FLITFEN                  ((uint16_t)0x0010)            /*!< FLITF clock enable */
#define  RCC_AHBENR_CRCEN                    ((uint16_t)0x0040)            /*!< CRC clock enable */

#if defined (STM32F10X_HD) || defined  (STM32F10X_CL) || defined  (STM32F10X_HD_VL)
 #define  RCC_AHBENR_DMA2EN                  ((uint16_t)0x0002)            /*!< DMA2 clock enable */
#endif

#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
 #define  RCC_AHBENR_FSMCEN                  ((uint16_t)0x0100)            /*!< FSMC clock enable */
 #define  RCC_AHBENR_SDIOEN                  ((uint16_t)0x0400)            /*!< SDIO clock enable */
#endif

#if defined (STM32F10X_HD_VL)
 #define  RCC_AHBENR_FSMCEN                  ((uint16_t)0x0100)            /*!< FSMC clock enable */
#endif

#ifdef STM32F10X_CL
 #define  RCC_AHBENR_OTGFSEN                 ((uint32_t)0x00001000)         /*!< USB OTG FS clock enable */
 #define  RCC_AHBENR_ETHMACEN                ((uint32_t)0x00004000)         /*!< ETHERNET MAC clock enable */
 #define  RCC_AHBENR_ETHMACTXEN              ((uint32_t)0x00008000)         /*!< ETHERNET MAC Tx clock enable */
 #define  RCC_AHBENR_ETHMACRXEN              ((uint32_t)0x00010000)         /*!< ETHERNET MAC Rx clock enable */
#endif /* STM32F10X_CL */

/******************  Bit definition for RCC_APB2ENR register  *****************/
#define  RCC_APB2ENR_AFIOEN                  ((uint32_t)0x00000001)         /*!< Alternate Function I/O clock enable */
#define  RCC_APB2ENR_IOPAEN                  ((uint32_t)0x00000004)         /*!< I/O port A clock enable */
#define  RCC_APB2ENR_IOPBEN                  ((uint32_t)0x00000008)         /*!< I/O port B clock enable */
#define  RCC_APB2ENR_IOPCEN                  ((uint32_t)0x00000010)         /*!< I/O port C clock enable */
#define  RCC_APB2ENR_IOPDEN                  ((uint32_t)0x00000020)         /*!< I/O port D clock enable */
#define  RCC_APB2ENR_ADC1EN                  ((uint32_t)0x00000200)         /*!< ADC 1 interface clock enable */

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD_VL)
#define  RCC_APB2ENR_ADC2EN                  ((uint32_t)0x00000400)         /*!< ADC 2 interface clock enable */
#endif

#define  RCC_APB2ENR_TIM1EN                  ((uint32_t)0x00000800)         /*!< TIM1 Timer clock enable */
#define  RCC_APB2ENR_SPI1EN                  ((uint32_t)0x00001000)         /*!< SPI 1 clock enable */
#define  RCC_APB2ENR_USART1EN                ((uint32_t)0x00004000)         /*!< USART1 clock enable */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
#define  RCC_APB2ENR_TIM15EN                 ((uint32_t)0x00010000)         /*!< TIM15 Timer clock enable */
#define  RCC_APB2ENR_TIM16EN                 ((uint32_t)0x00020000)         /*!< TIM16 Timer clock enable */
#define  RCC_APB2ENR_TIM17EN                 ((uint32_t)0x00040000)         /*!< TIM17 Timer clock enable */
#endif

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
 #define  RCC_APB2ENR_IOPEEN                 ((uint32_t)0x00000040)         /*!< I/O port E clock enable */
#endif /* STM32F10X_LD && STM32F10X_LD_VL */

#if defined (STM32F10X_HD) || defined (STM32F10X_XL)
 #define  RCC_APB2ENR_IOPFEN                 ((uint32_t)0x00000080)         /*!< I/O port F clock enable */
 #define  RCC_APB2ENR_IOPGEN                 ((uint32_t)0x00000100)         /*!< I/O port G clock enable */
 #define  RCC_APB2ENR_TIM8EN                 ((uint32_t)0x00002000)         /*!< TIM8 Timer clock enable */
 #define  RCC_APB2ENR_ADC3EN                 ((uint32_t)0x00008000)         /*!< DMA1 clock enable */
#endif

#if defined (STM32F10X_HD_VL)
 #define  RCC_APB2ENR_IOPFEN                 ((uint32_t)0x00000080)         /*!< I/O port F clock enable */
 #define  RCC_APB2ENR_IOPGEN                 ((uint32_t)0x00000100)         /*!< I/O port G clock enable */
#endif

#ifdef STM32F10X_XL
 #define  RCC_APB2ENR_TIM9EN                 ((uint32_t)0x00080000)         /*!< TIM9 Timer clock enable  */
 #define  RCC_APB2ENR_TIM10EN                ((uint32_t)0x00100000)         /*!< TIM10 Timer clock enable  */
 #define  RCC_APB2ENR_TIM11EN                ((uint32_t)0x00200000)         /*!< TIM11 Timer clock enable */
#endif

/*****************  Bit definition for RCC_APB1ENR register  ******************/
#define  RCC_APB1ENR_TIM2EN                  ((uint32_t)0x00000001)        /*!< Timer 2 clock enabled*/
#define  RCC_APB1ENR_TIM3EN                  ((uint32_t)0x00000002)        /*!< Timer 3 clock enable */
#define  RCC_APB1ENR_WWDGEN                  ((uint32_t)0x00000800)        /*!< Window Watchdog clock enable */
#define  RCC_APB1ENR_USART2EN                ((uint32_t)0x00020000)        /*!< USART 2 clock enable */
#define  RCC_APB1ENR_I2C1EN                  ((uint32_t)0x00200000)        /*!< I2C 1 clock enable */

#if !defined (STM32F10X_LD_VL) && !defined (STM32F10X_MD_VL) && !defined (STM32F10X_HD_VL)
#define  RCC_APB1ENR_CAN1EN                  ((uint32_t)0x02000000)        /*!< CAN1 clock enable */
#endif

#define  RCC_APB1ENR_BKPEN                   ((uint32_t)0x08000000)        /*!< Backup interface clock enable */
#define  RCC_APB1ENR_PWREN                   ((uint32_t)0x10000000)        /*!< Power interface clock enable */

#if !defined (STM32F10X_LD) && !defined (STM32F10X_LD_VL)
 #define  RCC_APB1ENR_TIM4EN                 ((uint32_t)0x00000004)        /*!< Timer 4 clock enable */
 #define  RCC_APB1ENR_SPI2EN                 ((uint32_t)0x00004000)        /*!< SPI 2 clock enable */
 #define  RCC_APB1ENR_USART3EN               ((uint32_t)0x00040000)        /*!< USART 3 clock enable */
 #define  RCC_APB1ENR_I2C2EN                 ((uint32_t)0x00400000)        /*!< I2C 2 clock enable */
#endif /* STM32F10X_LD && STM32F10X_LD_VL */

#if defined (STM32F10X_HD) || defined (STM32F10X_MD) || defined  (STM32F10X_LD)
 #define  RCC_APB1ENR_USBEN                  ((uint32_t)0x00800000)        /*!< USB Device clock enable */
#endif

#if defined (STM32F10X_HD) || defined  (STM32F10X_CL)
 #define  RCC_APB1ENR_TIM5EN                 ((uint32_t)0x00000008)        /*!< Timer 5 clock enable */
 #define  RCC_APB1ENR_TIM6EN                 ((uint32_t)0x00000010)        /*!< Timer 6 clock enable */
 #define  RCC_APB1ENR_TIM7EN                 ((uint32_t)0x00000020)        /*!< Timer 7 clock enable */
 #define  RCC_APB1ENR_SPI3EN                 ((uint32_t)0x00008000)        /*!< SPI 3 clock enable */
 #define  RCC_APB1ENR_UART4EN                ((uint32_t)0x00080000)        /*!< UART 4 clock enable */
 #define  RCC_APB1ENR_UART5EN                ((uint32_t)0x00100000)        /*!< UART 5 clock enable */
 #define  RCC_APB1ENR_DACEN                  ((uint32_t)0x20000000)        /*!< DAC interface clock enable */
#endif

#if defined (STM32F10X_LD_VL) || defined  (STM32F10X_MD_VL) || defined  (STM32F10X_HD_VL)
 #define  RCC_APB1ENR_TIM6EN                 ((uint32_t)0x00000010)        /*!< Timer 6 clock enable */
 #define  RCC_APB1ENR_TIM7EN                 ((uint32_t)0x00000020)        /*!< Timer 7 clock enable */
 #define  RCC_APB1ENR_DACEN                  ((uint32_t)0x20000000)        /*!< DAC interface clock enable */
 #define  RCC_APB1ENR_CECEN                  ((uint32_t)0x40000000)        /*!< CEC interface clock enable */ 
#endif

#ifdef STM32F10X_HD_VL
 #define  RCC_APB1ENR_TIM5EN                 ((uint32_t)0x00000008)        /*!< Timer 5 clock enable */
 #define  RCC_APB1ENR_TIM12EN                ((uint32_t)0x00000040)         /*!< TIM12 Timer clock enable  */
 #define  RCC_APB1ENR_TIM13EN                ((uint32_t)0x00000080)         /*!< TIM13 Timer clock enable  */
 #define  RCC_APB1ENR_TIM14EN                ((uint32_t)0x00000100)         /*!< TIM14 Timer clock enable */
 #define  RCC_APB1ENR_SPI3EN                 ((uint32_t)0x00008000)        /*!< SPI 3 clock enable */
 #define  RCC_APB1ENR_UART4EN                ((uint32_t)0x00080000)        /*!< UART 4 clock enable */
 #define  RCC_APB1ENR_UART5EN                ((uint32_t)0x00100000)        /*!< UART 5 clock enable */ 
#endif /* STM32F10X_HD_VL */

#ifdef STM32F10X_CL
 #define  RCC_APB1ENR_CAN2EN                  ((uint32_t)0x04000000)        /*!< CAN2 clock enable */
#endif /* STM32F10X_CL */

#ifdef STM32F10X_XL
 #define  RCC_APB1ENR_TIM12EN                ((uint32_t)0x00000040)         /*!< TIM12 Timer clock enable  */
 #define  RCC_APB1ENR_TIM13EN                ((uint32_t)0x00000080)         /*!< TIM13 Timer clock enable  */
 #define  RCC_APB1ENR_TIM14EN                ((uint32_t)0x00000100)         /*!< TIM14 Timer clock enable */
#endif /* STM32F10X_XL */

/*******************  Bit definition for RCC_BDCR register  *******************/
#define  RCC_BDCR_LSEON                      ((uint32_t)0x00000001)        /*!< External Low Speed oscillator enable */
#define  RCC_BDCR_LSERDY                     ((uint32_t)0x00000002)        /*!< External Low Speed oscillator Ready */
#define  RCC_BDCR_LSEBYP                     ((uint32_t)0x00000004)        /*!< External Low Speed oscillator Bypass */

#define  RCC_BDCR_RTCSEL                     ((uint32_t)0x00000300)        /*!< RTCSEL[1:0] bits (RTC clock source selection) */
#define  RCC_BDCR_RTCSEL_0                   ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  RCC_BDCR_RTCSEL_1                   ((uint32_t)0x00000200)        /*!< Bit 1 */

/*!< RTC congiguration */
#define  RCC_BDCR_RTCSEL_NOCLOCK             ((uint32_t)0x00000000)        /*!< No clock */
#define  RCC_BDCR_RTCSEL_LSE                 ((uint32_t)0x00000100)        /*!< LSE oscillator clock used as RTC clock */
#define  RCC_BDCR_RTCSEL_LSI                 ((uint32_t)0x00000200)        /*!< LSI oscillator clock used as RTC clock */
#define  RCC_BDCR_RTCSEL_HSE                 ((uint32_t)0x00000300)        /*!< HSE oscillator clock divided by 128 used as RTC clock */

#define  RCC_BDCR_RTCEN                      ((uint32_t)0x00008000)        /*!< RTC clock enable */
#define  RCC_BDCR_BDRST                      ((uint32_t)0x00010000)        /*!< Backup domain software reset  */

/*******************  Bit definition for RCC_CSR register  ********************/  
#define  RCC_CSR_LSION                       ((uint32_t)0x00000001)        /*!< Internal Low Speed oscillator enable */
#define  RCC_CSR_LSIRDY                      ((uint32_t)0x00000002)        /*!< Internal Low Speed oscillator Ready */
#define  RCC_CSR_RMVF                        ((uint32_t)0x01000000)        /*!< Remove reset flag */
#define  RCC_CSR_PINRSTF                     ((uint32_t)0x04000000)        /*!< PIN reset flag */
#define  RCC_CSR_PORRSTF                     ((uint32_t)0x08000000)        /*!< POR/PDR reset flag */
#define  RCC_CSR_SFTRSTF                     ((uint32_t)0x10000000)        /*!< Software Reset flag */
#define  RCC_CSR_IWDGRSTF                    ((uint32_t)0x20000000)        /*!< Independent Watchdog reset flag */
#define  RCC_CSR_WWDGRSTF                    ((uint32_t)0x40000000)        /*!< Window watchdog reset flag */
#define  RCC_CSR_LPWRRSTF                    ((uint32_t)0x80000000)        /*!< Low-Power reset flag */

#ifdef STM32F10X_CL
/*******************  Bit definition for RCC_AHBRSTR register  ****************/
 #define  RCC_AHBRSTR_OTGFSRST               ((uint32_t)0x00001000)         /*!< USB OTG FS reset */
 #define  RCC_AHBRSTR_ETHMACRST              ((uint32_t)0x00004000)         /*!< ETHERNET MAC reset */

/*******************  Bit definition for RCC_CFGR2 register  ******************/
/*!< PREDIV1 configuration */
 #define  RCC_CFGR2_PREDIV1                  ((uint32_t)0x0000000F)        /*!< PREDIV1[3:0] bits */
 #define  RCC_CFGR2_PREDIV1_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
 #define  RCC_CFGR2_PREDIV1_1                ((uint32_t)0x00000002)        /*!< Bit 1 */
 #define  RCC_CFGR2_PREDIV1_2                ((uint32_t)0x00000004)        /*!< Bit 2 */
 #define  RCC_CFGR2_PREDIV1_3                ((uint32_t)0x00000008)        /*!< Bit 3 */

 #define  RCC_CFGR2_PREDIV1_DIV1             ((uint32_t)0x00000000)        /*!< PREDIV1 input clock not divided */
 #define  RCC_CFGR2_PREDIV1_DIV2             ((uint32_t)0x00000001)        /*!< PREDIV1 input clock divided by 2 */
 #define  RCC_CFGR2_PREDIV1_DIV3             ((uint32_t)0x00000002)        /*!< PREDIV1 input clock divided by 3 */
 #define  RCC_CFGR2_PREDIV1_DIV4             ((uint32_t)0x00000003)        /*!< PREDIV1 input clock divided by 4 */
 #define  RCC_CFGR2_PREDIV1_DIV5             ((uint32_t)0x00000004)        /*!< PREDIV1 input clock divided by 5 */
 #define  RCC_CFGR2_PREDIV1_DIV6             ((uint32_t)0x00000005)        /*!< PREDIV1 input clock divided by 6 */
 #define  RCC_CFGR2_PREDIV1_DIV7             ((uint32_t)0x00000006)        /*!< PREDIV1 input clock divided by 7 */
 #define  RCC_CFGR2_PREDIV1_DIV8             ((uint32_t)0x00000007)        /*!< PREDIV1 input clock divided by 8 */
 #define  RCC_CFGR2_PREDIV1_DIV9             ((uint32_t)0x00000008)        /*!< PREDIV1 input clock divided by 9 */
 #define  RCC_CFGR2_PREDIV1_DIV10            ((uint32_t)0x00000009)        /*!< PREDIV1 input clock divided by 10 */
 #define  RCC_CFGR2_PREDIV1_DIV11            ((uint32_t)0x0000000A)        /*!< PREDIV1 input clock divided by 11 */
 #define  RCC_CFGR2_PREDIV1_DIV12            ((uint32_t)0x0000000B)        /*!< PREDIV1 input clock divided by 12 */
 #define  RCC_CFGR2_PREDIV1_DIV13            ((uint32_t)0x0000000C)        /*!< PREDIV1 input clock divided by 13 */
 #define  RCC_CFGR2_PREDIV1_DIV14            ((uint32_t)0x0000000D)        /*!< PREDIV1 input clock divided by 14 */
 #define  RCC_CFGR2_PREDIV1_DIV15            ((uint32_t)0x0000000E)        /*!< PREDIV1 input clock divided by 15 */
 #define  RCC_CFGR2_PREDIV1_DIV16            ((uint32_t)0x0000000F)        /*!< PREDIV1 input clock divided by 16 */

/*!< PREDIV2 configuration */
 #define  RCC_CFGR2_PREDIV2                  ((uint32_t)0x000000F0)        /*!< PREDIV2[3:0] bits */
 #define  RCC_CFGR2_PREDIV2_0                ((uint32_t)0x00000010)        /*!< Bit 0 */
 #define  RCC_CFGR2_PREDIV2_1                ((uint32_t)0x00000020)        /*!< Bit 1 */
 #define  RCC_CFGR2_PREDIV2_2                ((uint32_t)0x00000040)        /*!< Bit 2 */
 #define  RCC_CFGR2_PREDIV2_3                ((uint32_t)0x00000080)        /*!< Bit 3 */

 #define  RCC_CFGR2_PREDIV2_DIV1             ((uint32_t)0x00000000)        /*!< PREDIV2 input clock not divided */
 #define  RCC_CFGR2_PREDIV2_DIV2             ((uint32_t)0x00000010)        /*!< PREDIV2 input clock divided by 2 */
 #define  RCC_CFGR2_PREDIV2_DIV3             ((uint32_t)0x00000020)        /*!< PREDIV2 input clock divided by 3 */
 #define  RCC_CFGR2_PREDIV2_DIV4             ((uint32_t)0x00000030)        /*!< PREDIV2 input clock divided by 4 */
 #define  RCC_CFGR2_PREDIV2_DIV5             ((uint32_t)0x00000040)        /*!< PREDIV2 input clock divided by 5 */
 #define  RCC_CFGR2_PREDIV2_DIV6             ((uint32_t)0x00000050)        /*!< PREDIV2 input clock divided by 6 */
 #define  RCC_CFGR2_PREDIV2_DIV7             ((uint32_t)0x00000060)        /*!< PREDIV2 input clock divided by 7 */
 #define  RCC_CFGR2_PREDIV2_DIV8             ((uint32_t)0x00000070)        /*!< PREDIV2 input clock divided by 8 */
 #define  RCC_CFGR2_PREDIV2_DIV9             ((uint32_t)0x00000080)        /*!< PREDIV2 input clock divided by 9 */
 #define  RCC_CFGR2_PREDIV2_DIV10            ((uint32_t)0x00000090)        /*!< PREDIV2 input clock divided by 10 */
 #define  RCC_CFGR2_PREDIV2_DIV11            ((uint32_t)0x000000A0)        /*!< PREDIV2 input clock divided by 11 */
 #define  RCC_CFGR2_PREDIV2_DIV12            ((uint32_t)0x000000B0)        /*!< PREDIV2 input clock divided by 12 */
 #define  RCC_CFGR2_PREDIV2_DIV13            ((uint32_t)0x000000C0)        /*!< PREDIV2 input clock divided by 13 */
 #define  RCC_CFGR2_PREDIV2_DIV14            ((uint32_t)0x000000D0)        /*!< PREDIV2 input clock divided by 14 */
 #define  RCC_CFGR2_PREDIV2_DIV15            ((uint32_t)0x000000E0)        /*!< PREDIV2 input clock divided by 15 */
 #define  RCC_CFGR2_PREDIV2_DIV16            ((uint32_t)0x000000F0)        /*!< PREDIV2 input clock divided by 16 */

/*!< PLL2MUL configuration */
 #define  RCC_CFGR2_PLL2MUL                  ((uint32_t)0x00000F00)        /*!< PLL2MUL[3:0] bits */
 #define  RCC_CFGR2_PLL2MUL_0                ((uint32_t)0x00000100)        /*!< Bit 0 */
 #define  RCC_CFGR2_PLL2MUL_1                ((uint32_t)0x00000200)        /*!< Bit 1 */
 #define  RCC_CFGR2_PLL2MUL_2                ((uint32_t)0x00000400)        /*!< Bit 2 */
 #define  RCC_CFGR2_PLL2MUL_3                ((uint32_t)0x00000800)        /*!< Bit 3 */

 #define  RCC_CFGR2_PLL2MUL8                 ((uint32_t)0x00000600)        /*!< PLL2 input clock * 8 */
 #define  RCC_CFGR2_PLL2MUL9                 ((uint32_t)0x00000700)        /*!< PLL2 input clock * 9 */
 #define  RCC_CFGR2_PLL2MUL10                ((uint32_t)0x00000800)        /*!< PLL2 input clock * 10 */
 #define  RCC_CFGR2_PLL2MUL11                ((uint32_t)0x00000900)        /*!< PLL2 input clock * 11 */
 #define  RCC_CFGR2_PLL2MUL12                ((uint32_t)0x00000A00)        /*!< PLL2 input clock * 12 */
 #define  RCC_CFGR2_PLL2MUL13                ((uint32_t)0x00000B00)        /*!< PLL2 input clock * 13 */
 #define  RCC_CFGR2_PLL2MUL14                ((uint32_t)0x00000C00)        /*!< PLL2 input clock * 14 */
 #define  RCC_CFGR2_PLL2MUL16                ((uint32_t)0x00000E00)        /*!< PLL2 input clock * 16 */
 #define  RCC_CFGR2_PLL2MUL20                ((uint32_t)0x00000F00)        /*!< PLL2 input clock * 20 */

/*!< PLL3MUL configuration */
 #define  RCC_CFGR2_PLL3MUL                  ((uint32_t)0x0000F000)        /*!< PLL3MUL[3:0] bits */
 #define  RCC_CFGR2_PLL3MUL_0                ((uint32_t)0x00001000)        /*!< Bit 0 */
 #define  RCC_CFGR2_PLL3MUL_1                ((uint32_t)0x00002000)        /*!< Bit 1 */
 #define  RCC_CFGR2_PLL3MUL_2                ((uint32_t)0x00004000)        /*!< Bit 2 */
 #define  RCC_CFGR2_PLL3MUL_3                ((uint32_t)0x00008000)        /*!< Bit 3 */

 #define  RCC_CFGR2_PLL3MUL8                 ((uint32_t)0x00006000)        /*!< PLL3 input clock * 8 */
 #define  RCC_CFGR2_PLL3MUL9                 ((uint32_t)0x00007000)        /*!< PLL3 input clock * 9 */
 #define  RCC_CFGR2_PLL3MUL10                ((uint32_t)0x00008000)        /*!< PLL3 input clock * 10 */
 #define  RCC_CFGR2_PLL3MUL11                ((uint32_t)0x00009000)        /*!< PLL3 input clock * 11 */
 #define  RCC_CFGR2_PLL3MUL12                ((uint32_t)0x0000A000)        /*!< PLL3 input clock * 12 */
 #define  RCC_CFGR2_PLL3MUL13                ((uint32_t)0x0000B000)        /*!< PLL3 input clock * 13 */
 #define  RCC_CFGR2_PLL3MUL14                ((uint32_t)0x0000C000)        /*!< PLL3 input clock * 14 */
 #define  RCC_CFGR2_PLL3MUL16                ((uint32_t)0x0000E000)        /*!< PLL3 input clock * 16 */
 #define  RCC_CFGR2_PLL3MUL20                ((uint32_t)0x0000F000)        /*!< PLL3 input clock * 20 */

 #define  RCC_CFGR2_PREDIV1SRC               ((uint32_t)0x00010000)        /*!< PREDIV1 entry clock source */
 #define  RCC_CFGR2_PREDIV1SRC_PLL2          ((uint32_t)0x00010000)        /*!< PLL2 selected as PREDIV1 entry clock source */
 #define  RCC_CFGR2_PREDIV1SRC_HSE           ((uint32_t)0x00000000)        /*!< HSE selected as PREDIV1 entry clock source */
 #define  RCC_CFGR2_I2S2SRC                  ((uint32_t)0x00020000)        /*!< I2S2 entry clock source */
 #define  RCC_CFGR2_I2S3SRC                  ((uint32_t)0x00040000)        /*!< I2S3 clock source */
#endif /* STM32F10X_CL */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
/*******************  Bit definition for RCC_CFGR2 register  ******************/
/*!< PREDIV1 configuration */
 #define  RCC_CFGR2_PREDIV1                  ((uint32_t)0x0000000F)        /*!< PREDIV1[3:0] bits */
 #define  RCC_CFGR2_PREDIV1_0                ((uint32_t)0x00000001)        /*!< Bit 0 */
 #define  RCC_CFGR2_PREDIV1_1                ((uint32_t)0x00000002)        /*!< Bit 1 */
 #define  RCC_CFGR2_PREDIV1_2                ((uint32_t)0x00000004)        /*!< Bit 2 */
 #define  RCC_CFGR2_PREDIV1_3                ((uint32_t)0x00000008)        /*!< Bit 3 */

 #define  RCC_CFGR2_PREDIV1_DIV1             ((uint32_t)0x00000000)        /*!< PREDIV1 input clock not divided */
 #define  RCC_CFGR2_PREDIV1_DIV2             ((uint32_t)0x00000001)        /*!< PREDIV1 input clock divided by 2 */
 #define  RCC_CFGR2_PREDIV1_DIV3             ((uint32_t)0x00000002)        /*!< PREDIV1 input clock divided by 3 */
 #define  RCC_CFGR2_PREDIV1_DIV4             ((uint32_t)0x00000003)        /*!< PREDIV1 input clock divided by 4 */
 #define  RCC_CFGR2_PREDIV1_DIV5             ((uint32_t)0x00000004)        /*!< PREDIV1 input clock divided by 5 */
 #define  RCC_CFGR2_PREDIV1_DIV6             ((uint32_t)0x00000005)        /*!< PREDIV1 input clock divided by 6 */
 #define  RCC_CFGR2_PREDIV1_DIV7             ((uint32_t)0x00000006)        /*!< PREDIV1 input clock divided by 7 */
 #define  RCC_CFGR2_PREDIV1_DIV8             ((uint32_t)0x00000007)        /*!< PREDIV1 input clock divided by 8 */
 #define  RCC_CFGR2_PREDIV1_DIV9             ((uint32_t)0x00000008)        /*!< PREDIV1 input clock divided by 9 */
 #define  RCC_CFGR2_PREDIV1_DIV10            ((uint32_t)0x00000009)        /*!< PREDIV1 input clock divided by 10 */
 #define  RCC_CFGR2_PREDIV1_DIV11            ((uint32_t)0x0000000A)        /*!< PREDIV1 input clock divided by 11 */
 #define  RCC_CFGR2_PREDIV1_DIV12            ((uint32_t)0x0000000B)        /*!< PREDIV1 input clock divided by 12 */
 #define  RCC_CFGR2_PREDIV1_DIV13            ((uint32_t)0x0000000C)        /*!< PREDIV1 input clock divided by 13 */
 #define  RCC_CFGR2_PREDIV1_DIV14            ((uint32_t)0x0000000D)        /*!< PREDIV1 input clock divided by 14 */
 #define  RCC_CFGR2_PREDIV1_DIV15            ((uint32_t)0x0000000E)        /*!< PREDIV1 input clock divided by 15 */
 #define  RCC_CFGR2_PREDIV1_DIV16            ((uint32_t)0x0000000F)        /*!< PREDIV1 input clock divided by 16 */
#endif
 
/******************************************************************************/
/*                                                                            */
/*                General Purpose and Alternate Function I/O                  */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for GPIO_CRL register  *******************/
#define  GPIO_CRL_MODE                       ((uint32_t)0x33333333)        /*!< Port x mode bits */

#define  GPIO_CRL_MODE0                      ((uint32_t)0x00000003)        /*!< MODE0[1:0] bits (Port x mode bits, pin 0) */
#define  GPIO_CRL_MODE0_0                    ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  GPIO_CRL_MODE0_1                    ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  GPIO_CRL_MODE1                      ((uint32_t)0x00000030)        /*!< MODE1[1:0] bits (Port x mode bits, pin 1) */
#define  GPIO_CRL_MODE1_0                    ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  GPIO_CRL_MODE1_1                    ((uint32_t)0x00000020)        /*!< Bit 1 */

#define  GPIO_CRL_MODE2                      ((uint32_t)0x00000300)        /*!< MODE2[1:0] bits (Port x mode bits, pin 2) */
#define  GPIO_CRL_MODE2_0                    ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  GPIO_CRL_MODE2_1                    ((uint32_t)0x00000200)        /*!< Bit 1 */

#define  GPIO_CRL_MODE3                      ((uint32_t)0x00003000)        /*!< MODE3[1:0] bits (Port x mode bits, pin 3) */
#define  GPIO_CRL_MODE3_0                    ((uint32_t)0x00001000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE3_1                    ((uint32_t)0x00002000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE4                      ((uint32_t)0x00030000)        /*!< MODE4[1:0] bits (Port x mode bits, pin 4) */
#define  GPIO_CRL_MODE4_0                    ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE4_1                    ((uint32_t)0x00020000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE5                      ((uint32_t)0x00300000)        /*!< MODE5[1:0] bits (Port x mode bits, pin 5) */
#define  GPIO_CRL_MODE5_0                    ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE5_1                    ((uint32_t)0x00200000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE6                      ((uint32_t)0x03000000)        /*!< MODE6[1:0] bits (Port x mode bits, pin 6) */
#define  GPIO_CRL_MODE6_0                    ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE6_1                    ((uint32_t)0x02000000)        /*!< Bit 1 */

#define  GPIO_CRL_MODE7                      ((uint32_t)0x30000000)        /*!< MODE7[1:0] bits (Port x mode bits, pin 7) */
#define  GPIO_CRL_MODE7_0                    ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  GPIO_CRL_MODE7_1                    ((uint32_t)0x20000000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF                        ((uint32_t)0xCCCCCCCC)        /*!< Port x configuration bits */

#define  GPIO_CRL_CNF0                       ((uint32_t)0x0000000C)        /*!< CNF0[1:0] bits (Port x configuration bits, pin 0) */
#define  GPIO_CRL_CNF0_0                     ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  GPIO_CRL_CNF0_1                     ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  GPIO_CRL_CNF1                       ((uint32_t)0x000000C0)        /*!< CNF1[1:0] bits (Port x configuration bits, pin 1) */
#define  GPIO_CRL_CNF1_0                     ((uint32_t)0x00000040)        /*!< Bit 0 */
#define  GPIO_CRL_CNF1_1                     ((uint32_t)0x00000080)        /*!< Bit 1 */

#define  GPIO_CRL_CNF2                       ((uint32_t)0x00000C00)        /*!< CNF2[1:0] bits (Port x configuration bits, pin 2) */
#define  GPIO_CRL_CNF2_0                     ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  GPIO_CRL_CNF2_1                     ((uint32_t)0x00000800)        /*!< Bit 1 */

#define  GPIO_CRL_CNF3                       ((uint32_t)0x0000C000)        /*!< CNF3[1:0] bits (Port x configuration bits, pin 3) */
#define  GPIO_CRL_CNF3_0                     ((uint32_t)0x00004000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF3_1                     ((uint32_t)0x00008000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF4                       ((uint32_t)0x000C0000)        /*!< CNF4[1:0] bits (Port x configuration bits, pin 4) */
#define  GPIO_CRL_CNF4_0                     ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF4_1                     ((uint32_t)0x00080000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF5                       ((uint32_t)0x00C00000)        /*!< CNF5[1:0] bits (Port x configuration bits, pin 5) */
#define  GPIO_CRL_CNF5_0                     ((uint32_t)0x00400000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF5_1                     ((uint32_t)0x00800000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF6                       ((uint32_t)0x0C000000)        /*!< CNF6[1:0] bits (Port x configuration bits, pin 6) */
#define  GPIO_CRL_CNF6_0                     ((uint32_t)0x04000000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF6_1                     ((uint32_t)0x08000000)        /*!< Bit 1 */

#define  GPIO_CRL_CNF7                       ((uint32_t)0xC0000000)        /*!< CNF7[1:0] bits (Port x configuration bits, pin 7) */
#define  GPIO_CRL_CNF7_0                     ((uint32_t)0x40000000)        /*!< Bit 0 */
#define  GPIO_CRL_CNF7_1                     ((uint32_t)0x80000000)        /*!< Bit 1 */

/*******************  Bit definition for GPIO_CRH register  *******************/
#define  GPIO_CRH_MODE                       ((uint32_t)0x33333333)        /*!< Port x mode bits */

#define  GPIO_CRH_MODE8                      ((uint32_t)0x00000003)        /*!< MODE8[1:0] bits (Port x mode bits, pin 8) */
#define  GPIO_CRH_MODE8_0                    ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  GPIO_CRH_MODE8_1                    ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  GPIO_CRH_MODE9                      ((uint32_t)0x00000030)        /*!< MODE9[1:0] bits (Port x mode bits, pin 9) */
#define  GPIO_CRH_MODE9_0                    ((uint32_t)0x00000010)        /*!< Bit 0 */
#define  GPIO_CRH_MODE9_1                    ((uint32_t)0x00000020)        /*!< Bit 1 */

#define  GPIO_CRH_MODE10                     ((uint32_t)0x00000300)        /*!< MODE10[1:0] bits (Port x mode bits, pin 10) */
#define  GPIO_CRH_MODE10_0                   ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  GPIO_CRH_MODE10_1                   ((uint32_t)0x00000200)        /*!< Bit 1 */

#define  GPIO_CRH_MODE11                     ((uint32_t)0x00003000)        /*!< MODE11[1:0] bits (Port x mode bits, pin 11) */
#define  GPIO_CRH_MODE11_0                   ((uint32_t)0x00001000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE11_1                   ((uint32_t)0x00002000)        /*!< Bit 1 */

#define  GPIO_CRH_MODE12                     ((uint32_t)0x00030000)        /*!< MODE12[1:0] bits (Port x mode bits, pin 12) */
#define  GPIO_CRH_MODE12_0                   ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE12_1                   ((uint32_t)0x00020000)        /*!< Bit 1 */

#define  GPIO_CRH_MODE13                     ((uint32_t)0x00300000)        /*!< MODE13[1:0] bits (Port x mode bits, pin 13) */
#define  GPIO_CRH_MODE13_0                   ((uint32_t)0x00100000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE13_1                   ((uint32_t)0x00200000)        /*!< Bit 1 */

#define  GPIO_CRH_MODE14                     ((uint32_t)0x03000000)        /*!< MODE14[1:0] bits (Port x mode bits, pin 14) */
#define  GPIO_CRH_MODE14_0                   ((uint32_t)0x01000000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE14_1                   ((uint32_t)0x02000000)        /*!< Bit 1 */

#define  GPIO_CRH_MODE15                     ((uint32_t)0x30000000)        /*!< MODE15[1:0] bits (Port x mode bits, pin 15) */
#define  GPIO_CRH_MODE15_0                   ((uint32_t)0x10000000)        /*!< Bit 0 */
#define  GPIO_CRH_MODE15_1                   ((uint32_t)0x20000000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF                        ((uint32_t)0xCCCCCCCC)        /*!< Port x configuration bits */

#define  GPIO_CRH_CNF8                       ((uint32_t)0x0000000C)        /*!< CNF8[1:0] bits (Port x configuration bits, pin 8) */
#define  GPIO_CRH_CNF8_0                     ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  GPIO_CRH_CNF8_1                     ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  GPIO_CRH_CNF9                       ((uint32_t)0x000000C0)        /*!< CNF9[1:0] bits (Port x configuration bits, pin 9) */
#define  GPIO_CRH_CNF9_0                     ((uint32_t)0x00000040)        /*!< Bit 0 */
#define  GPIO_CRH_CNF9_1                     ((uint32_t)0x00000080)        /*!< Bit 1 */

#define  GPIO_CRH_CNF10                      ((uint32_t)0x00000C00)        /*!< CNF10[1:0] bits (Port x configuration bits, pin 10) */
#define  GPIO_CRH_CNF10_0                    ((uint32_t)0x00000400)        /*!< Bit 0 */
#define  GPIO_CRH_CNF10_1                    ((uint32_t)0x00000800)        /*!< Bit 1 */

#define  GPIO_CRH_CNF11                      ((uint32_t)0x0000C000)        /*!< CNF11[1:0] bits (Port x configuration bits, pin 11) */
#define  GPIO_CRH_CNF11_0                    ((uint32_t)0x00004000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF11_1                    ((uint32_t)0x00008000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF12                      ((uint32_t)0x000C0000)        /*!< CNF12[1:0] bits (Port x configuration bits, pin 12) */
#define  GPIO_CRH_CNF12_0                    ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF12_1                    ((uint32_t)0x00080000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF13                      ((uint32_t)0x00C00000)        /*!< CNF13[1:0] bits (Port x configuration bits, pin 13) */
#define  GPIO_CRH_CNF13_0                    ((uint32_t)0x00400000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF13_1                    ((uint32_t)0x00800000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF14                      ((uint32_t)0x0C000000)        /*!< CNF14[1:0] bits (Port x configuration bits, pin 14) */
#define  GPIO_CRH_CNF14_0                    ((uint32_t)0x04000000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF14_1                    ((uint32_t)0x08000000)        /*!< Bit 1 */

#define  GPIO_CRH_CNF15                      ((uint32_t)0xC0000000)        /*!< CNF15[1:0] bits (Port x configuration bits, pin 15) */
#define  GPIO_CRH_CNF15_0                    ((uint32_t)0x40000000)        /*!< Bit 0 */
#define  GPIO_CRH_CNF15_1                    ((uint32_t)0x80000000)        /*!< Bit 1 */

/*!<******************  Bit definition for GPIO_IDR register  *******************/
#define GPIO_IDR_IDR0                        ((uint16_t)0x0001)            /*!< Port input data, bit 0 */
#define GPIO_IDR_IDR1                        ((uint16_t)0x0002)            /*!< Port input data, bit 1 */
#define GPIO_IDR_IDR2                        ((uint16_t)0x0004)            /*!< Port input data, bit 2 */
#define GPIO_IDR_IDR3                        ((uint16_t)0x0008)            /*!< Port input data, bit 3 */
#define GPIO_IDR_IDR4                        ((uint16_t)0x0010)            /*!< Port input data, bit 4 */
#define GPIO_IDR_IDR5                        ((uint16_t)0x0020)            /*!< Port input data, bit 5 */
#define GPIO_IDR_IDR6                        ((uint16_t)0x0040)            /*!< Port input data, bit 6 */
#define GPIO_IDR_IDR7                        ((uint16_t)0x0080)            /*!< Port input data, bit 7 */
#define GPIO_IDR_IDR8                        ((uint16_t)0x0100)            /*!< Port input data, bit 8 */
#define GPIO_IDR_IDR9                        ((uint16_t)0x0200)            /*!< Port input data, bit 9 */
#define GPIO_IDR_IDR10                       ((uint16_t)0x0400)            /*!< Port input data, bit 10 */
#define GPIO_IDR_IDR11                       ((uint16_t)0x0800)            /*!< Port input data, bit 11 */
#define GPIO_IDR_IDR12                       ((uint16_t)0x1000)            /*!< Port input data, bit 12 */
#define GPIO_IDR_IDR13                       ((uint16_t)0x2000)            /*!< Port input data, bit 13 */
#define GPIO_IDR_IDR14                       ((uint16_t)0x4000)            /*!< Port input data, bit 14 */
#define GPIO_IDR_IDR15                       ((uint16_t)0x8000)            /*!< Port input data, bit 15 */

/*******************  Bit definition for GPIO_ODR register  *******************/
#define GPIO_ODR_ODR0                        ((uint16_t)0x0001)            /*!< Port output data, bit 0 */
#define GPIO_ODR_ODR1                        ((uint16_t)0x0002)            /*!< Port output data, bit 1 */
#define GPIO_ODR_ODR2                        ((uint16_t)0x0004)            /*!< Port output data, bit 2 */
#define GPIO_ODR_ODR3                        ((uint16_t)0x0008)            /*!< Port output data, bit 3 */
#define GPIO_ODR_ODR4                        ((uint16_t)0x0010)            /*!< Port output data, bit 4 */
#define GPIO_ODR_ODR5                        ((uint16_t)0x0020)            /*!< Port output data, bit 5 */
#define GPIO_ODR_ODR6                        ((uint16_t)0x0040)            /*!< Port output data, bit 6 */
#define GPIO_ODR_ODR7                        ((uint16_t)0x0080)            /*!< Port output data, bit 7 */
#define GPIO_ODR_ODR8                        ((uint16_t)0x0100)            /*!< Port output data, bit 8 */
#define GPIO_ODR_ODR9                        ((uint16_t)0x0200)            /*!< Port output data, bit 9 */
#define GPIO_ODR_ODR10                       ((uint16_t)0x0400)            /*!< Port output data, bit 10 */
#define GPIO_ODR_ODR11                       ((uint16_t)0x0800)            /*!< Port output data, bit 11 */
#define GPIO_ODR_ODR12                       ((uint16_t)0x1000)            /*!< Port output data, bit 12 */
#define GPIO_ODR_ODR13                       ((uint16_t)0x2000)            /*!< Port output data, bit 13 */
#define GPIO_ODR_ODR14                       ((uint16_t)0x4000)            /*!< Port output data, bit 14 */
#define GPIO_ODR_ODR15                       ((uint16_t)0x8000)            /*!< Port output data, bit 15 */

/******************  Bit definition for GPIO_BSRR register  *******************/
#define GPIO_BSRR_BS0                        ((uint32_t)0x00000001)        /*!< Port x Set bit 0 */
#define GPIO_BSRR_BS1                        ((uint32_t)0x00000002)        /*!< Port x Set bit 1 */
#define GPIO_BSRR_BS2                        ((uint32_t)0x00000004)        /*!< Port x Set bit 2 */
#define GPIO_BSRR_BS3                        ((uint32_t)0x00000008)        /*!< Port x Set bit 3 */
#define GPIO_BSRR_BS4                        ((uint32_t)0x00000010)        /*!< Port x Set bit 4 */
#define GPIO_BSRR_BS5                        ((uint32_t)0x00000020)        /*!< Port x Set bit 5 */
#define GPIO_BSRR_BS6                        ((uint32_t)0x00000040)        /*!< Port x Set bit 6 */
#define GPIO_BSRR_BS7                        ((uint32_t)0x00000080)        /*!< Port x Set bit 7 */
#define GPIO_BSRR_BS8                        ((uint32_t)0x00000100)        /*!< Port x Set bit 8 */
#define GPIO_BSRR_BS9                        ((uint32_t)0x00000200)        /*!< Port x Set bit 9 */
#define GPIO_BSRR_BS10                       ((uint32_t)0x00000400)        /*!< Port x Set bit 10 */
#define GPIO_BSRR_BS11                       ((uint32_t)0x00000800)        /*!< Port x Set bit 11 */
#define GPIO_BSRR_BS12                       ((uint32_t)0x00001000)        /*!< Port x Set bit 12 */
#define GPIO_BSRR_BS13                       ((uint32_t)0x00002000)        /*!< Port x Set bit 13 */
#define GPIO_BSRR_BS14                       ((uint32_t)0x00004000)        /*!< Port x Set bit 14 */
#define GPIO_BSRR_BS15                       ((uint32_t)0x00008000)        /*!< Port x Set bit 15 */

#define GPIO_BSRR_BR0                        ((uint32_t)0x00010000)        /*!< Port x Reset bit 0 */
#define GPIO_BSRR_BR1                        ((uint32_t)0x00020000)        /*!< Port x Reset bit 1 */
#define GPIO_BSRR_BR2                        ((uint32_t)0x00040000)        /*!< Port x Reset bit 2 */
#define GPIO_BSRR_BR3                        ((uint32_t)0x00080000)        /*!< Port x Reset bit 3 */
#define GPIO_BSRR_BR4                        ((uint32_t)0x00100000)        /*!< Port x Reset bit 4 */
#define GPIO_BSRR_BR5                        ((uint32_t)0x00200000)        /*!< Port x Reset bit 5 */
#define GPIO_BSRR_BR6                        ((uint32_t)0x00400000)        /*!< Port x Reset bit 6 */
#define GPIO_BSRR_BR7                        ((uint32_t)0x00800000)        /*!< Port x Reset bit 7 */
#define GPIO_BSRR_BR8                        ((uint32_t)0x01000000)        /*!< Port x Reset bit 8 */
#define GPIO_BSRR_BR9                        ((uint32_t)0x02000000)        /*!< Port x Reset bit 9 */
#define GPIO_BSRR_BR10                       ((uint32_t)0x04000000)        /*!< Port x Reset bit 10 */
#define GPIO_BSRR_BR11                       ((uint32_t)0x08000000)        /*!< Port x Reset bit 11 */
#define GPIO_BSRR_BR12                       ((uint32_t)0x10000000)        /*!< Port x Reset bit 12 */
#define GPIO_BSRR_BR13                       ((uint32_t)0x20000000)        /*!< Port x Reset bit 13 */
#define GPIO_BSRR_BR14                       ((uint32_t)0x40000000)        /*!< Port x Reset bit 14 */
#define GPIO_BSRR_BR15                       ((uint32_t)0x80000000)        /*!< Port x Reset bit 15 */

/*******************  Bit definition for GPIO_BRR register  *******************/
#define GPIO_BRR_BR0                         ((uint16_t)0x0001)            /*!< Port x Reset bit 0 */
#define GPIO_BRR_BR1                         ((uint16_t)0x0002)            /*!< Port x Reset bit 1 */
#define GPIO_BRR_BR2                         ((uint16_t)0x0004)            /*!< Port x Reset bit 2 */
#define GPIO_BRR_BR3                         ((uint16_t)0x0008)            /*!< Port x Reset bit 3 */
#define GPIO_BRR_BR4                         ((uint16_t)0x0010)            /*!< Port x Reset bit 4 */
#define GPIO_BRR_BR5                         ((uint16_t)0x0020)            /*!< Port x Reset bit 5 */
#define GPIO_BRR_BR6                         ((uint16_t)0x0040)            /*!< Port x Reset bit 6 */
#define GPIO_BRR_BR7                         ((uint16_t)0x0080)            /*!< Port x Reset bit 7 */
#define GPIO_BRR_BR8                         ((uint16_t)0x0100)            /*!< Port x Reset bit 8 */
#define GPIO_BRR_BR9                         ((uint16_t)0x0200)            /*!< Port x Reset bit 9 */
#define GPIO_BRR_BR10                        ((uint16_t)0x0400)            /*!< Port x Reset bit 10 */
#define GPIO_BRR_BR11                        ((uint16_t)0x0800)            /*!< Port x Reset bit 11 */
#define GPIO_BRR_BR12                        ((uint16_t)0x1000)            /*!< Port x Reset bit 12 */
#define GPIO_BRR_BR13                        ((uint16_t)0x2000)            /*!< Port x Reset bit 13 */
#define GPIO_BRR_BR14                        ((uint16_t)0x4000)            /*!< Port x Reset bit 14 */
#define GPIO_BRR_BR15                        ((uint16_t)0x8000)            /*!< Port x Reset bit 15 */

/******************  Bit definition for GPIO_LCKR register  *******************/
#define GPIO_LCKR_LCK0                       ((uint32_t)0x00000001)        /*!< Port x Lock bit 0 */
#define GPIO_LCKR_LCK1                       ((uint32_t)0x00000002)        /*!< Port x Lock bit 1 */
#define GPIO_LCKR_LCK2                       ((uint32_t)0x00000004)        /*!< Port x Lock bit 2 */
#define GPIO_LCKR_LCK3                       ((uint32_t)0x00000008)        /*!< Port x Lock bit 3 */
#define GPIO_LCKR_LCK4                       ((uint32_t)0x00000010)        /*!< Port x Lock bit 4 */
#define GPIO_LCKR_LCK5                       ((uint32_t)0x00000020)        /*!< Port x Lock bit 5 */
#define GPIO_LCKR_LCK6                       ((uint32_t)0x00000040)        /*!< Port x Lock bit 6 */
#define GPIO_LCKR_LCK7                       ((uint32_t)0x00000080)        /*!< Port x Lock bit 7 */
#define GPIO_LCKR_LCK8                       ((uint32_t)0x00000100)        /*!< Port x Lock bit 8 */
#define GPIO_LCKR_LCK9                       ((uint32_t)0x00000200)        /*!< Port x Lock bit 9 */
#define GPIO_LCKR_LCK10                      ((uint32_t)0x00000400)        /*!< Port x Lock bit 10 */
#define GPIO_LCKR_LCK11                      ((uint32_t)0x00000800)        /*!< Port x Lock bit 11 */
#define GPIO_LCKR_LCK12                      ((uint32_t)0x00001000)        /*!< Port x Lock bit 12 */
#define GPIO_LCKR_LCK13                      ((uint32_t)0x00002000)        /*!< Port x Lock bit 13 */
#define GPIO_LCKR_LCK14                      ((uint32_t)0x00004000)        /*!< Port x Lock bit 14 */
#define GPIO_LCKR_LCK15                      ((uint32_t)0x00008000)        /*!< Port x Lock bit 15 */
#define GPIO_LCKR_LCKK                       ((uint32_t)0x00010000)        /*!< Lock key */

/*----------------------------------------------------------------------------*/

/******************  Bit definition for AFIO_EVCR register  *******************/
#define AFIO_EVCR_PIN                        ((uint8_t)0x0F)               /*!< PIN[3:0] bits (Pin selection) */
#define AFIO_EVCR_PIN_0                      ((uint8_t)0x01)               /*!< Bit 0 */
#define AFIO_EVCR_PIN_1                      ((uint8_t)0x02)               /*!< Bit 1 */
#define AFIO_EVCR_PIN_2                      ((uint8_t)0x04)               /*!< Bit 2 */
#define AFIO_EVCR_PIN_3                      ((uint8_t)0x08)               /*!< Bit 3 */

/*!< PIN configuration */
#define AFIO_EVCR_PIN_PX0                    ((uint8_t)0x00)               /*!< Pin 0 selected */
#define AFIO_EVCR_PIN_PX1                    ((uint8_t)0x01)               /*!< Pin 1 selected */
#define AFIO_EVCR_PIN_PX2                    ((uint8_t)0x02)               /*!< Pin 2 selected */
#define AFIO_EVCR_PIN_PX3                    ((uint8_t)0x03)               /*!< Pin 3 selected */
#define AFIO_EVCR_PIN_PX4                    ((uint8_t)0x04)               /*!< Pin 4 selected */
#define AFIO_EVCR_PIN_PX5                    ((uint8_t)0x05)               /*!< Pin 5 selected */
#define AFIO_EVCR_PIN_PX6                    ((uint8_t)0x06)               /*!< Pin 6 selected */
#define AFIO_EVCR_PIN_PX7                    ((uint8_t)0x07)               /*!< Pin 7 selected */
#define AFIO_EVCR_PIN_PX8                    ((uint8_t)0x08)               /*!< Pin 8 selected */
#define AFIO_EVCR_PIN_PX9                    ((uint8_t)0x09)               /*!< Pin 9 selected */
#define AFIO_EVCR_PIN_PX10                   ((uint8_t)0x0A)               /*!< Pin 10 selected */
#define AFIO_EVCR_PIN_PX11                   ((uint8_t)0x0B)               /*!< Pin 11 selected */
#define AFIO_EVCR_PIN_PX12                   ((uint8_t)0x0C)               /*!< Pin 12 selected */
#define AFIO_EVCR_PIN_PX13                   ((uint8_t)0x0D)               /*!< Pin 13 selected */
#define AFIO_EVCR_PIN_PX14                   ((uint8_t)0x0E)               /*!< Pin 14 selected */
#define AFIO_EVCR_PIN_PX15                   ((uint8_t)0x0F)               /*!< Pin 15 selected */

#define AFIO_EVCR_PORT                       ((uint8_t)0x70)               /*!< PORT[2:0] bits (Port selection) */
#define AFIO_EVCR_PORT_0                     ((uint8_t)0x10)               /*!< Bit 0 */
#define AFIO_EVCR_PORT_1                     ((uint8_t)0x20)               /*!< Bit 1 */
#define AFIO_EVCR_PORT_2                     ((uint8_t)0x40)               /*!< Bit 2 */

/*!< PORT configuration */
#define AFIO_EVCR_PORT_PA                    ((uint8_t)0x00)               /*!< Port A selected */
#define AFIO_EVCR_PORT_PB                    ((uint8_t)0x10)               /*!< Port B selected */
#define AFIO_EVCR_PORT_PC                    ((uint8_t)0x20)               /*!< Port C selected */
#define AFIO_EVCR_PORT_PD                    ((uint8_t)0x30)               /*!< Port D selected */
#define AFIO_EVCR_PORT_PE                    ((uint8_t)0x40)               /*!< Port E selected */

#define AFIO_EVCR_EVOE                       ((uint8_t)0x80)               /*!< Event Output Enable */

/******************  Bit definition for AFIO_MAPR register  *******************/
#define AFIO_MAPR_SPI1_REMAP                 ((uint32_t)0x00000001)        /*!< SPI1 remapping */
#define AFIO_MAPR_I2C1_REMAP                 ((uint32_t)0x00000002)        /*!< I2C1 remapping */
#define AFIO_MAPR_USART1_REMAP               ((uint32_t)0x00000004)        /*!< USART1 remapping */
#define AFIO_MAPR_USART2_REMAP               ((uint32_t)0x00000008)        /*!< USART2 remapping */

#define AFIO_MAPR_USART3_REMAP               ((uint32_t)0x00000030)        /*!< USART3_REMAP[1:0] bits (USART3 remapping) */
#define AFIO_MAPR_USART3_REMAP_0             ((uint32_t)0x00000010)        /*!< Bit 0 */
#define AFIO_MAPR_USART3_REMAP_1             ((uint32_t)0x00000020)        /*!< Bit 1 */

/* USART3_REMAP configuration */
#define AFIO_MAPR_USART3_REMAP_NOREMAP       ((uint32_t)0x00000000)        /*!< No remap (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14) */
#define AFIO_MAPR_USART3_REMAP_PARTIALREMAP  ((uint32_t)0x00000010)        /*!< Partial remap (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14) */
#define AFIO_MAPR_USART3_REMAP_FULLREMAP     ((uint32_t)0x00000030)        /*!< Full remap (TX/PD8, RX/PD9, CK/PD10, CTS/PD11, RTS/PD12) */

#define AFIO_MAPR_TIM1_REMAP                 ((uint32_t)0x000000C0)        /*!< TIM1_REMAP[1:0] bits (TIM1 remapping) */
#define AFIO_MAPR_TIM1_REMAP_0               ((uint32_t)0x00000040)        /*!< Bit 0 */
#define AFIO_MAPR_TIM1_REMAP_1               ((uint32_t)0x00000080)        /*!< Bit 1 */

/*!< TIM1_REMAP configuration */
#define AFIO_MAPR_TIM1_REMAP_NOREMAP         ((uint32_t)0x00000000)        /*!< No remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15) */
#define AFIO_MAPR_TIM1_REMAP_PARTIALREMAP    ((uint32_t)0x00000040)        /*!< Partial remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1) */
#define AFIO_MAPR_TIM1_REMAP_FULLREMAP       ((uint32_t)0x000000C0)        /*!< Full remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12) */

#define AFIO_MAPR_TIM2_REMAP                 ((uint32_t)0x00000300)        /*!< TIM2_REMAP[1:0] bits (TIM2 remapping) */
#define AFIO_MAPR_TIM2_REMAP_0               ((uint32_t)0x00000100)        /*!< Bit 0 */
#define AFIO_MAPR_TIM2_REMAP_1               ((uint32_t)0x00000200)        /*!< Bit 1 */

/*!< TIM2_REMAP configuration */
#define AFIO_MAPR_TIM2_REMAP_NOREMAP         ((uint32_t)0x00000000)        /*!< No remap (CH1/ETR/PA0, CH2/PA1, CH3/PA2, CH4/PA3) */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1   ((uint32_t)0x00000100)        /*!< Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2, CH4/PA3) */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2   ((uint32_t)0x00000200)        /*!< Partial remap (CH1/ETR/PA0, CH2/PA1, CH3/PB10, CH4/PB11) */
#define AFIO_MAPR_TIM2_REMAP_FULLREMAP       ((uint32_t)0x00000300)        /*!< Full remap (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11) */

#define AFIO_MAPR_TIM3_REMAP                 ((uint32_t)0x00000C00)        /*!< TIM3_REMAP[1:0] bits (TIM3 remapping) */
#define AFIO_MAPR_TIM3_REMAP_0               ((uint32_t)0x00000400)        /*!< Bit 0 */
#define AFIO_MAPR_TIM3_REMAP_1               ((uint32_t)0x00000800)        /*!< Bit 1 */

/*!< TIM3_REMAP configuration */
#define AFIO_MAPR_TIM3_REMAP_NOREMAP         ((uint32_t)0x00000000)        /*!< No remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1) */
#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP    ((uint32_t)0x00000800)        /*!< Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1) */
#define AFIO_MAPR_TIM3_REMAP_FULLREMAP       ((uint32_t)0x00000C00)        /*!< Full remap (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9) */

#define AFIO_MAPR_TIM4_REMAP                 ((uint32_t)0x00001000)        /*!< TIM4_REMAP bit (TIM4 remapping) */

#define AFIO_MAPR_CAN_REMAP                  ((uint32_t)0x00006000)        /*!< CAN_REMAP[1:0] bits (CAN Alternate function remapping) */
#define AFIO_MAPR_CAN_REMAP_0                ((uint32_t)0x00002000)        /*!< Bit 0 */
#define AFIO_MAPR_CAN_REMAP_1                ((uint32_t)0x00004000)        /*!< Bit 1 */

/*!< CAN_REMAP configuration */
#define AFIO_MAPR_CAN_REMAP_REMAP1           ((uint32_t)0x00000000)        /*!< CANRX mapped to PA11, CANTX mapped to PA12 */
#define AFIO_MAPR_CAN_REMAP_REMAP2           ((uint32_t)0x00004000)        /*!< CANRX mapped to PB8, CANTX mapped to PB9 */
#define AFIO_MAPR_CAN_REMAP_REMAP3           ((uint32_t)0x00006000)        /*!< CANRX mapped to PD0, CANTX mapped to PD1 */

#define AFIO_MAPR_PD01_REMAP                 ((uint32_t)0x00008000)        /*!< Port D0/Port D1 mapping on OSC_IN/OSC_OUT */
#define AFIO_MAPR_TIM5CH4_IREMAP             ((uint32_t)0x00010000)        /*!< TIM5 Channel4 Internal Remap */
#define AFIO_MAPR_ADC1_ETRGINJ_REMAP         ((uint32_t)0x00020000)        /*!< ADC 1 External Trigger Injected Conversion remapping */
#define AFIO_MAPR_ADC1_ETRGREG_REMAP         ((uint32_t)0x00040000)        /*!< ADC 1 External Trigger Regular Conversion remapping */
#define AFIO_MAPR_ADC2_ETRGINJ_REMAP         ((uint32_t)0x00080000)        /*!< ADC 2 External Trigger Injected Conversion remapping */
#define AFIO_MAPR_ADC2_ETRGREG_REMAP         ((uint32_t)0x00100000)        /*!< ADC 2 External Trigger Regular Conversion remapping */

/*!< SWJ_CFG configuration */
#define AFIO_MAPR_SWJ_CFG                    ((uint32_t)0x07000000)        /*!< SWJ_CFG[2:0] bits (Serial Wire JTAG configuration) */
#define AFIO_MAPR_SWJ_CFG_0                  ((uint32_t)0x01000000)        /*!< Bit 0 */
#define AFIO_MAPR_SWJ_CFG_1                  ((uint32_t)0x02000000)        /*!< Bit 1 */
#define AFIO_MAPR_SWJ_CFG_2                  ((uint32_t)0x04000000)        /*!< Bit 2 */

#define AFIO_MAPR_SWJ_CFG_RESET              ((uint32_t)0x00000000)        /*!< Full SWJ (JTAG-DP + SW-DP) : Reset State */
#define AFIO_MAPR_SWJ_CFG_NOJNTRST           ((uint32_t)0x01000000)        /*!< Full SWJ (JTAG-DP + SW-DP) but without JNTRST */
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE        ((uint32_t)0x02000000)        /*!< JTAG-DP Disabled and SW-DP Enabled */
#define AFIO_MAPR_SWJ_CFG_DISABLE            ((uint32_t)0x04000000)        /*!< JTAG-DP Disabled and SW-DP Disabled */

#ifdef STM32F10X_CL
/*!< ETH_REMAP configuration */
 #define AFIO_MAPR_ETH_REMAP                  ((uint32_t)0x00200000)        /*!< SPI3_REMAP bit (Ethernet MAC I/O remapping) */

/*!< CAN2_REMAP configuration */
 #define AFIO_MAPR_CAN2_REMAP                 ((uint32_t)0x00400000)        /*!< CAN2_REMAP bit (CAN2 I/O remapping) */

/*!< MII_RMII_SEL configuration */
 #define AFIO_MAPR_MII_RMII_SEL               ((uint32_t)0x00800000)        /*!< MII_RMII_SEL bit (Ethernet MII or RMII selection) */

/*!< SPI3_REMAP configuration */
 #define AFIO_MAPR_SPI3_REMAP                 ((uint32_t)0x10000000)        /*!< SPI3_REMAP bit (SPI3 remapping) */

/*!< TIM2ITR1_IREMAP configuration */
 #define AFIO_MAPR_TIM2ITR1_IREMAP            ((uint32_t)0x20000000)        /*!< TIM2ITR1_IREMAP bit (TIM2 internal trigger 1 remapping) */

/*!< PTP_PPS_REMAP configuration */
 #define AFIO_MAPR_PTP_PPS_REMAP              ((uint32_t)0x40000000)        /*!< PTP_PPS_REMAP bit (Ethernet PTP PPS remapping) */
#endif

/*****************  Bit definition for AFIO_EXTICR1 register  *****************/
#define AFIO_EXTICR1_EXTI0                   ((uint16_t)0x000F)            /*!< EXTI 0 configuration */
#define AFIO_EXTICR1_EXTI1                   ((uint16_t)0x00F0)            /*!< EXTI 1 configuration */
#define AFIO_EXTICR1_EXTI2                   ((uint16_t)0x0F00)            /*!< EXTI 2 configuration */
#define AFIO_EXTICR1_EXTI3                   ((uint16_t)0xF000)            /*!< EXTI 3 configuration */

/*!< EXTI0 configuration */
#define AFIO_EXTICR1_EXTI0_PA                ((uint16_t)0x0000)            /*!< PA[0] pin */
#define AFIO_EXTICR1_EXTI0_PB                ((uint16_t)0x0001)            /*!< PB[0] pin */
#define AFIO_EXTICR1_EXTI0_PC                ((uint16_t)0x0002)            /*!< PC[0] pin */
#define AFIO_EXTICR1_EXTI0_PD                ((uint16_t)0x0003)            /*!< PD[0] pin */
#define AFIO_EXTICR1_EXTI0_PE                ((uint16_t)0x0004)            /*!< PE[0] pin */
#define AFIO_EXTICR1_EXTI0_PF                ((uint16_t)0x0005)            /*!< PF[0] pin */
#define AFIO_EXTICR1_EXTI0_PG                ((uint16_t)0x0006)            /*!< PG[0] pin */

/*!< EXTI1 configuration */
#define AFIO_EXTICR1_EXTI1_PA                ((uint16_t)0x0000)            /*!< PA[1] pin */
#define AFIO_EXTICR1_EXTI1_PB                ((uint16_t)0x0010)            /*!< PB[1] pin */
#define AFIO_EXTICR1_EXTI1_PC                ((uint16_t)0x0020)            /*!< PC[1] pin */
#define AFIO_EXTICR1_EXTI1_PD                ((uint16_t)0x0030)            /*!< PD[1] pin */
#define AFIO_EXTICR1_EXTI1_PE                ((uint16_t)0x0040)            /*!< PE[1] pin */
#define AFIO_EXTICR1_EXTI1_PF                ((uint16_t)0x0050)            /*!< PF[1] pin */
#define AFIO_EXTICR1_EXTI1_PG                ((uint16_t)0x0060)            /*!< PG[1] pin */

/*!< EXTI2 configuration */  
#define AFIO_EXTICR1_EXTI2_PA                ((uint16_t)0x0000)            /*!< PA[2] pin */
#define AFIO_EXTICR1_EXTI2_PB                ((uint16_t)0x0100)            /*!< PB[2] pin */
#define AFIO_EXTICR1_EXTI2_PC                ((uint16_t)0x0200)            /*!< PC[2] pin */
#define AFIO_EXTICR1_EXTI2_PD                ((uint16_t)0x0300)            /*!< PD[2] pin */
#define AFIO_EXTICR1_EXTI2_PE                ((uint16_t)0x0400)            /*!< PE[2] pin */
#define AFIO_EXTICR1_EXTI2_PF                ((uint16_t)0x0500)            /*!< PF[2] pin */
#define AFIO_EXTICR1_EXTI2_PG                ((uint16_t)0x0600)            /*!< PG[2] pin */

/*!< EXTI3 configuration */
#define AFIO_EXTICR1_EXTI3_PA                ((uint16_t)0x0000)            /*!< PA[3] pin */
#define AFIO_EXTICR1_EXTI3_PB                ((uint16_t)0x1000)            /*!< PB[3] pin */
#define AFIO_EXTICR1_EXTI3_PC                ((uint16_t)0x2000)            /*!< PC[3] pin */
#define AFIO_EXTICR1_EXTI3_PD                ((uint16_t)0x3000)            /*!< PD[3] pin */
#define AFIO_EXTICR1_EXTI3_PE                ((uint16_t)0x4000)            /*!< PE[3] pin */
#define AFIO_EXTICR1_EXTI3_PF                ((uint16_t)0x5000)            /*!< PF[3] pin */
#define AFIO_EXTICR1_EXTI3_PG                ((uint16_t)0x6000)            /*!< PG[3] pin */

/*****************  Bit definition for AFIO_EXTICR2 register  *****************/
#define AFIO_EXTICR2_EXTI4                   ((uint16_t)0x000F)            /*!< EXTI 4 configuration */
#define AFIO_EXTICR2_EXTI5                   ((uint16_t)0x00F0)            /*!< EXTI 5 configuration */
#define AFIO_EXTICR2_EXTI6                   ((uint16_t)0x0F00)            /*!< EXTI 6 configuration */
#define AFIO_EXTICR2_EXTI7                   ((uint16_t)0xF000)            /*!< EXTI 7 configuration */

/*!< EXTI4 configuration */
#define AFIO_EXTICR2_EXTI4_PA                ((uint16_t)0x0000)            /*!< PA[4] pin */
#define AFIO_EXTICR2_EXTI4_PB                ((uint16_t)0x0001)            /*!< PB[4] pin */
#define AFIO_EXTICR2_EXTI4_PC                ((uint16_t)0x0002)            /*!< PC[4] pin */
#define AFIO_EXTICR2_EXTI4_PD                ((uint16_t)0x0003)            /*!< PD[4] pin */
#define AFIO_EXTICR2_EXTI4_PE                ((uint16_t)0x0004)            /*!< PE[4] pin */
#define AFIO_EXTICR2_EXTI4_PF                ((uint16_t)0x0005)            /*!< PF[4] pin */
#define AFIO_EXTICR2_EXTI4_PG                ((uint16_t)0x0006)            /*!< PG[4] pin */

/* EXTI5 configuration */
#define AFIO_EXTICR2_EXTI5_PA                ((uint16_t)0x0000)            /*!< PA[5] pin */
#define AFIO_EXTICR2_EXTI5_PB                ((uint16_t)0x0010)            /*!< PB[5] pin */
#define AFIO_EXTICR2_EXTI5_PC                ((uint16_t)0x0020)            /*!< PC[5] pin */
#define AFIO_EXTICR2_EXTI5_PD                ((uint16_t)0x0030)            /*!< PD[5] pin */
#define AFIO_EXTICR2_EXTI5_PE                ((uint16_t)0x0040)            /*!< PE[5] pin */
#define AFIO_EXTICR2_EXTI5_PF                ((uint16_t)0x0050)            /*!< PF[5] pin */
#define AFIO_EXTICR2_EXTI5_PG                ((uint16_t)0x0060)            /*!< PG[5] pin */

/*!< EXTI6 configuration */  
#define AFIO_EXTICR2_EXTI6_PA                ((uint16_t)0x0000)            /*!< PA[6] pin */
#define AFIO_EXTICR2_EXTI6_PB                ((uint16_t)0x0100)            /*!< PB[6] pin */
#define AFIO_EXTICR2_EXTI6_PC                ((uint16_t)0x0200)            /*!< PC[6] pin */
#define AFIO_EXTICR2_EXTI6_PD                ((uint16_t)0x0300)            /*!< PD[6] pin */
#define AFIO_EXTICR2_EXTI6_PE                ((uint16_t)0x0400)            /*!< PE[6] pin */
#define AFIO_EXTICR2_EXTI6_PF                ((uint16_t)0x0500)            /*!< PF[6] pin */
#define AFIO_EXTICR2_EXTI6_PG                ((uint16_t)0x0600)            /*!< PG[6] pin */

/*!< EXTI7 configuration */
#define AFIO_EXTICR2_EXTI7_PA                ((uint16_t)0x0000)            /*!< PA[7] pin */
#define AFIO_EXTICR2_EXTI7_PB                ((uint16_t)0x1000)            /*!< PB[7] pin */
#define AFIO_EXTICR2_EXTI7_PC                ((uint16_t)0x2000)            /*!< PC[7] pin */
#define AFIO_EXTICR2_EXTI7_PD                ((uint16_t)0x3000)            /*!< PD[7] pin */
#define AFIO_EXTICR2_EXTI7_PE                ((uint16_t)0x4000)            /*!< PE[7] pin */
#define AFIO_EXTICR2_EXTI7_PF                ((uint16_t)0x5000)            /*!< PF[7] pin */
#define AFIO_EXTICR2_EXTI7_PG                ((uint16_t)0x6000)            /*!< PG[7] pin */

/*****************  Bit definition for AFIO_EXTICR3 register  *****************/
#define AFIO_EXTICR3_EXTI8                   ((uint16_t)0x000F)            /*!< EXTI 8 configuration */
#define AFIO_EXTICR3_EXTI9                   ((uint16_t)0x00F0)            /*!< EXTI 9 configuration */
#define AFIO_EXTICR3_EXTI10                  ((uint16_t)0x0F00)            /*!< EXTI 10 configuration */
#define AFIO_EXTICR3_EXTI11                  ((uint16_t)0xF000)            /*!< EXTI 11 configuration */

/*!< EXTI8 configuration */
#define AFIO_EXTICR3_EXTI8_PA                ((uint16_t)0x0000)            /*!< PA[8] pin */
#define AFIO_EXTICR3_EXTI8_PB                ((uint16_t)0x0001)            /*!< PB[8] pin */
#define AFIO_EXTICR3_EXTI8_PC                ((uint16_t)0x0002)            /*!< PC[8] pin */
#define AFIO_EXTICR3_EXTI8_PD                ((uint16_t)0x0003)            /*!< PD[8] pin */
#define AFIO_EXTICR3_EXTI8_PE                ((uint16_t)0x0004)            /*!< PE[8] pin */
#define AFIO_EXTICR3_EXTI8_PF                ((uint16_t)0x0005)            /*!< PF[8] pin */
#define AFIO_EXTICR3_EXTI8_PG                ((uint16_t)0x0006)            /*!< PG[8] pin */

/*!< EXTI9 configuration */
#define AFIO_EXTICR3_EXTI9_PA                ((uint16_t)0x0000)            /*!< PA[9] pin */
#define AFIO_EXTICR3_EXTI9_PB                ((uint16_t)0x0010)            /*!< PB[9] pin */
#define AFIO_EXTICR3_EXTI9_PC                ((uint16_t)0x0020)            /*!< PC[9] pin */
#define AFIO_EXTICR3_EXTI9_PD                ((uint16_t)0x0030)            /*!< PD[9] pin */
#define AFIO_EXTICR3_EXTI9_PE                ((uint16_t)0x0040)            /*!< PE[9] pin */
#define AFIO_EXTICR3_EXTI9_PF                ((uint16_t)0x0050)            /*!< PF[9] pin */
#define AFIO_EXTICR3_EXTI9_PG                ((uint16_t)0x0060)            /*!< PG[9] pin */

/*!< EXTI10 configuration */  
#define AFIO_EXTICR3_EXTI10_PA               ((uint16_t)0x0000)            /*!< PA[10] pin */
#define AFIO_EXTICR3_EXTI10_PB               ((uint16_t)0x0100)            /*!< PB[10] pin */
#define AFIO_EXTICR3_EXTI10_PC               ((uint16_t)0x0200)            /*!< PC[10] pin */
#define AFIO_EXTICR3_EXTI10_PD               ((uint16_t)0x0300)            /*!< PD[10] pin */
#define AFIO_EXTICR3_EXTI10_PE               ((uint16_t)0x0400)            /*!< PE[10] pin */
#define AFIO_EXTICR3_EXTI10_PF               ((uint16_t)0x0500)            /*!< PF[10] pin */
#define AFIO_EXTICR3_EXTI10_PG               ((uint16_t)0x0600)            /*!< PG[10] pin */

/*!< EXTI11 configuration */
#define AFIO_EXTICR3_EXTI11_PA               ((uint16_t)0x0000)            /*!< PA[11] pin */
#define AFIO_EXTICR3_EXTI11_PB               ((uint16_t)0x1000)            /*!< PB[11] pin */
#define AFIO_EXTICR3_EXTI11_PC               ((uint16_t)0x2000)            /*!< PC[11] pin */
#define AFIO_EXTICR3_EXTI11_PD               ((uint16_t)0x3000)            /*!< PD[11] pin */
#define AFIO_EXTICR3_EXTI11_PE               ((uint16_t)0x4000)            /*!< PE[11] pin */
#define AFIO_EXTICR3_EXTI11_PF               ((uint16_t)0x5000)            /*!< PF[11] pin */
#define AFIO_EXTICR3_EXTI11_PG               ((uint16_t)0x6000)            /*!< PG[11] pin */

/*****************  Bit definition for AFIO_EXTICR4 register  *****************/
#define AFIO_EXTICR4_EXTI12                  ((uint16_t)0x000F)            /*!< EXTI 12 configuration */
#define AFIO_EXTICR4_EXTI13                  ((uint16_t)0x00F0)            /*!< EXTI 13 configuration */
#define AFIO_EXTICR4_EXTI14                  ((uint16_t)0x0F00)            /*!< EXTI 14 configuration */
#define AFIO_EXTICR4_EXTI15                  ((uint16_t)0xF000)            /*!< EXTI 15 configuration */

/* EXTI12 configuration */
#define AFIO_EXTICR4_EXTI12_PA               ((uint16_t)0x0000)            /*!< PA[12] pin */
#define AFIO_EXTICR4_EXTI12_PB               ((uint16_t)0x0001)            /*!< PB[12] pin */
#define AFIO_EXTICR4_EXTI12_PC               ((uint16_t)0x0002)            /*!< PC[12] pin */
#define AFIO_EXTICR4_EXTI12_PD               ((uint16_t)0x0003)            /*!< PD[12] pin */
#define AFIO_EXTICR4_EXTI12_PE               ((uint16_t)0x0004)            /*!< PE[12] pin */
#define AFIO_EXTICR4_EXTI12_PF               ((uint16_t)0x0005)            /*!< PF[12] pin */
#define AFIO_EXTICR4_EXTI12_PG               ((uint16_t)0x0006)            /*!< PG[12] pin */

/* EXTI13 configuration */
#define AFIO_EXTICR4_EXTI13_PA               ((uint16_t)0x0000)            /*!< PA[13] pin */
#define AFIO_EXTICR4_EXTI13_PB               ((uint16_t)0x0010)            /*!< PB[13] pin */
#define AFIO_EXTICR4_EXTI13_PC               ((uint16_t)0x0020)            /*!< PC[13] pin */
#define AFIO_EXTICR4_EXTI13_PD               ((uint16_t)0x0030)            /*!< PD[13] pin */
#define AFIO_EXTICR4_EXTI13_PE               ((uint16_t)0x0040)            /*!< PE[13] pin */
#define AFIO_EXTICR4_EXTI13_PF               ((uint16_t)0x0050)            /*!< PF[13] pin */
#define AFIO_EXTICR4_EXTI13_PG               ((uint16_t)0x0060)            /*!< PG[13] pin */

/*!< EXTI14 configuration */  
#define AFIO_EXTICR4_EXTI14_PA               ((uint16_t)0x0000)            /*!< PA[14] pin */
#define AFIO_EXTICR4_EXTI14_PB               ((uint16_t)0x0100)            /*!< PB[14] pin */
#define AFIO_EXTICR4_EXTI14_PC               ((uint16_t)0x0200)            /*!< PC[14] pin */
#define AFIO_EXTICR4_EXTI14_PD               ((uint16_t)0x0300)            /*!< PD[14] pin */
#define AFIO_EXTICR4_EXTI14_PE               ((uint16_t)0x0400)            /*!< PE[14] pin */
#define AFIO_EXTICR4_EXTI14_PF               ((uint16_t)0x0500)            /*!< PF[14] pin */
#define AFIO_EXTICR4_EXTI14_PG               ((uint16_t)0x0600)            /*!< PG[14] pin */

/*!< EXTI15 configuration */
#define AFIO_EXTICR4_EXTI15_PA               ((uint16_t)0x0000)            /*!< PA[15] pin */
#define AFIO_EXTICR4_EXTI15_PB               ((uint16_t)0x1000)            /*!< PB[15] pin */
#define AFIO_EXTICR4_EXTI15_PC               ((uint16_t)0x2000)            /*!< PC[15] pin */
#define AFIO_EXTICR4_EXTI15_PD               ((uint16_t)0x3000)            /*!< PD[15] pin */
#define AFIO_EXTICR4_EXTI15_PE               ((uint16_t)0x4000)            /*!< PE[15] pin */
#define AFIO_EXTICR4_EXTI15_PF               ((uint16_t)0x5000)            /*!< PF[15] pin */
#define AFIO_EXTICR4_EXTI15_PG               ((uint16_t)0x6000)            /*!< PG[15] pin */

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
/******************  Bit definition for AFIO_MAPR2 register  ******************/
#define AFIO_MAPR2_TIM15_REMAP               ((uint32_t)0x00000001)        /*!< TIM15 remapping */
#define AFIO_MAPR2_TIM16_REMAP               ((uint32_t)0x00000002)        /*!< TIM16 remapping */
#define AFIO_MAPR2_TIM17_REMAP               ((uint32_t)0x00000004)        /*!< TIM17 remapping */
#define AFIO_MAPR2_CEC_REMAP                 ((uint32_t)0x00000008)        /*!< CEC remapping */
#define AFIO_MAPR2_TIM1_DMA_REMAP            ((uint32_t)0x00000010)        /*!< TIM1_DMA remapping */
#endif

#ifdef STM32F10X_HD_VL
#define AFIO_MAPR2_TIM13_REMAP               ((uint32_t)0x00000100)        /*!< TIM13 remapping */
#define AFIO_MAPR2_TIM14_REMAP               ((uint32_t)0x00000200)        /*!< TIM14 remapping */
#define AFIO_MAPR2_FSMC_NADV_REMAP           ((uint32_t)0x00000400)        /*!< FSMC NADV remapping */
#define AFIO_MAPR2_TIM67_DAC_DMA_REMAP       ((uint32_t)0x00000800)        /*!< TIM6/TIM7 and DAC DMA remapping */
#define AFIO_MAPR2_TIM12_REMAP               ((uint32_t)0x00001000)        /*!< TIM12 remapping */
#define AFIO_MAPR2_MISC_REMAP                ((uint32_t)0x00002000)        /*!< Miscellaneous remapping */
#endif

#ifdef STM32F10X_XL 
/******************  Bit definition for AFIO_MAPR2 register  ******************/
#define AFIO_MAPR2_TIM9_REMAP                ((uint32_t)0x00000020)        /*!< TIM9 remapping */
#define AFIO_MAPR2_TIM10_REMAP               ((uint32_t)0x00000040)        /*!< TIM10 remapping */
#define AFIO_MAPR2_TIM11_REMAP               ((uint32_t)0x00000080)        /*!< TIM11 remapping */
#define AFIO_MAPR2_TIM13_REMAP               ((uint32_t)0x00000100)        /*!< TIM13 remapping */
#define AFIO_MAPR2_TIM14_REMAP               ((uint32_t)0x00000200)        /*!< TIM14 remapping */
#define AFIO_MAPR2_FSMC_NADV_REMAP           ((uint32_t)0x00000400)        /*!< FSMC NADV remapping */
#endif

/******************************************************************************/
/*                                                                            */
/*                               SystemTick                                   */
/*                                                                            */
/******************************************************************************/

/*****************  Bit definition for SysTick_CTRL register  *****************/
#define  SysTick_CTRL_ENABLE                 ((uint32_t)0x00000001)        /*!< Counter enable */
#define  SysTick_CTRL_TICKINT                ((uint32_t)0x00000002)        /*!< Counting down to 0 pends the SysTick handler */
#define  SysTick_CTRL_CLKSOURCE              ((uint32_t)0x00000004)        /*!< Clock source */
#define  SysTick_CTRL_COUNTFLAG              ((uint32_t)0x00010000)        /*!< Count Flag */

/*****************  Bit definition for SysTick_LOAD register  *****************/
#define  SysTick_LOAD_RELOAD                 ((uint32_t)0x00FFFFFF)        /*!< Value to load into the SysTick Current Value Register when the counter reaches 0 */

/*****************  Bit definition for SysTick_VAL register  ******************/
#define  SysTick_VAL_CURRENT                 ((uint32_t)0x00FFFFFF)        /*!< Current value at the time the register is accessed */

/*****************  Bit definition for SysTick_CALIB register  ****************/
#define  SysTick_CALIB_TENMS                 ((uint32_t)0x00FFFFFF)        /*!< Reload value to use for 10ms timing */
#define  SysTick_CALIB_SKEW                  ((uint32_t)0x40000000)        /*!< Calibration value is not exactly 10 ms */
#define  SysTick_CALIB_NOREF                 ((uint32_t)0x80000000)        /*!< The reference clock is not provided */

/******************************************************************************/
/*                                                                            */
/*                  Nested Vectored Interrupt Controller                      */
/*                                                                            */
/******************************************************************************/

/******************  Bit definition for NVIC_ISER register  *******************/
#define  NVIC_ISER_SETENA                    ((uint32_t)0xFFFFFFFF)        /*!< Interrupt set enable bits */
#define  NVIC_ISER_SETENA_0                  ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ISER_SETENA_1                  ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ISER_SETENA_2                  ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ISER_SETENA_3                  ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ISER_SETENA_4                  ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ISER_SETENA_5                  ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ISER_SETENA_6                  ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ISER_SETENA_7                  ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ISER_SETENA_8                  ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ISER_SETENA_9                  ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ISER_SETENA_10                 ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ISER_SETENA_11                 ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ISER_SETENA_12                 ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ISER_SETENA_13                 ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ISER_SETENA_14                 ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ISER_SETENA_15                 ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ISER_SETENA_16                 ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ISER_SETENA_17                 ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ISER_SETENA_18                 ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ISER_SETENA_19                 ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ISER_SETENA_20                 ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ISER_SETENA_21                 ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ISER_SETENA_22                 ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ISER_SETENA_23                 ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ISER_SETENA_24                 ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ISER_SETENA_25                 ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ISER_SETENA_26                 ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ISER_SETENA_27                 ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ISER_SETENA_28                 ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ISER_SETENA_29                 ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ISER_SETENA_30                 ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ISER_SETENA_31                 ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ICER register  *******************/
#define  NVIC_ICER_CLRENA                   ((uint32_t)0xFFFFFFFF)        /*!< Interrupt clear-enable bits */
#define  NVIC_ICER_CLRENA_0                  ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ICER_CLRENA_1                  ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ICER_CLRENA_2                  ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ICER_CLRENA_3                  ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ICER_CLRENA_4                  ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ICER_CLRENA_5                  ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ICER_CLRENA_6                  ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ICER_CLRENA_7                  ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ICER_CLRENA_8                  ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ICER_CLRENA_9                  ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ICER_CLRENA_10                 ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ICER_CLRENA_11                 ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ICER_CLRENA_12                 ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ICER_CLRENA_13                 ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ICER_CLRENA_14                 ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ICER_CLRENA_15                 ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ICER_CLRENA_16                 ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ICER_CLRENA_17                 ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ICER_CLRENA_18                 ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ICER_CLRENA_19                 ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ICER_CLRENA_20                 ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ICER_CLRENA_21                 ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ICER_CLRENA_22                 ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ICER_CLRENA_23                 ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ICER_CLRENA_24                 ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ICER_CLRENA_25                 ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ICER_CLRENA_26                 ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ICER_CLRENA_27                 ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ICER_CLRENA_28                 ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ICER_CLRENA_29                 ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ICER_CLRENA_30                 ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ICER_CLRENA_31                 ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ISPR register  *******************/
#define  NVIC_ISPR_SETPEND                   ((uint32_t)0xFFFFFFFF)        /*!< Interrupt set-pending bits */
#define  NVIC_ISPR_SETPEND_0                 ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ISPR_SETPEND_1                 ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ISPR_SETPEND_2                 ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ISPR_SETPEND_3                 ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ISPR_SETPEND_4                 ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ISPR_SETPEND_5                 ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ISPR_SETPEND_6                 ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ISPR_SETPEND_7                 ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ISPR_SETPEND_8                 ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ISPR_SETPEND_9                 ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ISPR_SETPEND_10                ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ISPR_SETPEND_11                ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ISPR_SETPEND_12                ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ISPR_SETPEND_13                ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ISPR_SETPEND_14                ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ISPR_SETPEND_15                ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ISPR_SETPEND_16                ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ISPR_SETPEND_17                ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ISPR_SETPEND_18                ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ISPR_SETPEND_19                ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ISPR_SETPEND_20                ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ISPR_SETPEND_21                ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ISPR_SETPEND_22                ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ISPR_SETPEND_23                ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ISPR_SETPEND_24                ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ISPR_SETPEND_25                ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ISPR_SETPEND_26                ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ISPR_SETPEND_27                ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ISPR_SETPEND_28                ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ISPR_SETPEND_29                ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ISPR_SETPEND_30                ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ISPR_SETPEND_31                ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_ICPR register  *******************/
#define  NVIC_ICPR_CLRPEND                   ((uint32_t)0xFFFFFFFF)        /*!< Interrupt clear-pending bits */
#define  NVIC_ICPR_CLRPEND_0                 ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_ICPR_CLRPEND_1                 ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_ICPR_CLRPEND_2                 ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_ICPR_CLRPEND_3                 ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_ICPR_CLRPEND_4                 ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_ICPR_CLRPEND_5                 ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_ICPR_CLRPEND_6                 ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_ICPR_CLRPEND_7                 ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_ICPR_CLRPEND_8                 ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_ICPR_CLRPEND_9                 ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_ICPR_CLRPEND_10                ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_ICPR_CLRPEND_11                ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_ICPR_CLRPEND_12                ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_ICPR_CLRPEND_13                ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_ICPR_CLRPEND_14                ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_ICPR_CLRPEND_15                ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_ICPR_CLRPEND_16                ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_ICPR_CLRPEND_17                ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_ICPR_CLRPEND_18                ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_ICPR_CLRPEND_19                ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_ICPR_CLRPEND_20                ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_ICPR_CLRPEND_21                ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_ICPR_CLRPEND_22                ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_ICPR_CLRPEND_23                ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_ICPR_CLRPEND_24                ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_ICPR_CLRPEND_25                ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_ICPR_CLRPEND_26                ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_ICPR_CLRPEND_27                ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_ICPR_CLRPEND_28                ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_ICPR_CLRPEND_29                ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_ICPR_CLRPEND_30                ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_ICPR_CLRPEND_31                ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_IABR register  *******************/
#define  NVIC_IABR_ACTIVE                    ((uint32_t)0xFFFFFFFF)        /*!< Interrupt active flags */
#define  NVIC_IABR_ACTIVE_0                  ((uint32_t)0x00000001)        /*!< bit 0 */
#define  NVIC_IABR_ACTIVE_1                  ((uint32_t)0x00000002)        /*!< bit 1 */
#define  NVIC_IABR_ACTIVE_2                  ((uint32_t)0x00000004)        /*!< bit 2 */
#define  NVIC_IABR_ACTIVE_3                  ((uint32_t)0x00000008)        /*!< bit 3 */
#define  NVIC_IABR_ACTIVE_4                  ((uint32_t)0x00000010)        /*!< bit 4 */
#define  NVIC_IABR_ACTIVE_5                  ((uint32_t)0x00000020)        /*!< bit 5 */
#define  NVIC_IABR_ACTIVE_6                  ((uint32_t)0x00000040)        /*!< bit 6 */
#define  NVIC_IABR_ACTIVE_7                  ((uint32_t)0x00000080)        /*!< bit 7 */
#define  NVIC_IABR_ACTIVE_8                  ((uint32_t)0x00000100)        /*!< bit 8 */
#define  NVIC_IABR_ACTIVE_9                  ((uint32_t)0x00000200)        /*!< bit 9 */
#define  NVIC_IABR_ACTIVE_10                 ((uint32_t)0x00000400)        /*!< bit 10 */
#define  NVIC_IABR_ACTIVE_11                 ((uint32_t)0x00000800)        /*!< bit 11 */
#define  NVIC_IABR_ACTIVE_12                 ((uint32_t)0x00001000)        /*!< bit 12 */
#define  NVIC_IABR_ACTIVE_13                 ((uint32_t)0x00002000)        /*!< bit 13 */
#define  NVIC_IABR_ACTIVE_14                 ((uint32_t)0x00004000)        /*!< bit 14 */
#define  NVIC_IABR_ACTIVE_15                 ((uint32_t)0x00008000)        /*!< bit 15 */
#define  NVIC_IABR_ACTIVE_16                 ((uint32_t)0x00010000)        /*!< bit 16 */
#define  NVIC_IABR_ACTIVE_17                 ((uint32_t)0x00020000)        /*!< bit 17 */
#define  NVIC_IABR_ACTIVE_18                 ((uint32_t)0x00040000)        /*!< bit 18 */
#define  NVIC_IABR_ACTIVE_19                 ((uint32_t)0x00080000)        /*!< bit 19 */
#define  NVIC_IABR_ACTIVE_20                 ((uint32_t)0x00100000)        /*!< bit 20 */
#define  NVIC_IABR_ACTIVE_21                 ((uint32_t)0x00200000)        /*!< bit 21 */
#define  NVIC_IABR_ACTIVE_22                 ((uint32_t)0x00400000)        /*!< bit 22 */
#define  NVIC_IABR_ACTIVE_23                 ((uint32_t)0x00800000)        /*!< bit 23 */
#define  NVIC_IABR_ACTIVE_24                 ((uint32_t)0x01000000)        /*!< bit 24 */
#define  NVIC_IABR_ACTIVE_25                 ((uint32_t)0x02000000)        /*!< bit 25 */
#define  NVIC_IABR_ACTIVE_26                 ((uint32_t)0x04000000)        /*!< bit 26 */
#define  NVIC_IABR_ACTIVE_27                 ((uint32_t)0x08000000)        /*!< bit 27 */
#define  NVIC_IABR_ACTIVE_28                 ((uint32_t)0x10000000)        /*!< bit 28 */
#define  NVIC_IABR_ACTIVE_29                 ((uint32_t)0x20000000)        /*!< bit 29 */
#define  NVIC_IABR_ACTIVE_30                 ((uint32_t)0x40000000)        /*!< bit 30 */
#define  NVIC_IABR_ACTIVE_31                 ((uint32_t)0x80000000)        /*!< bit 31 */

/******************  Bit definition for NVIC_PRI0 register  *******************/
#define  NVIC_IPR0_PRI_0                     ((uint32_t)0x000000FF)        /*!< Priority of interrupt 0 */
#define  NVIC_IPR0_PRI_1                     ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 1 */
#define  NVIC_IPR0_PRI_2                     ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 2 */
#define  NVIC_IPR0_PRI_3                     ((uint32_t)0xFF000000)        /*!< Priority of interrupt 3 */

/******************  Bit definition for NVIC_PRI1 register  *******************/
#define  NVIC_IPR1_PRI_4                     ((uint32_t)0x000000FF)        /*!< Priority of interrupt 4 */
#define  NVIC_IPR1_PRI_5                     ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 5 */
#define  NVIC_IPR1_PRI_6                     ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 6 */
#define  NVIC_IPR1_PRI_7                     ((uint32_t)0xFF000000)        /*!< Priority of interrupt 7 */

/******************  Bit definition for NVIC_PRI2 register  *******************/
#define  NVIC_IPR2_PRI_8                     ((uint32_t)0x000000FF)        /*!< Priority of interrupt 8 */
#define  NVIC_IPR2_PRI_9                     ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 9 */
#define  NVIC_IPR2_PRI_10                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 10 */
#define  NVIC_IPR2_PRI_11                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 11 */

/******************  Bit definition for NVIC_PRI3 register  *******************/
#define  NVIC_IPR3_PRI_12                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 12 */
#define  NVIC_IPR3_PRI_13                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 13 */
#define  NVIC_IPR3_PRI_14                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 14 */
#define  NVIC_IPR3_PRI_15                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 15 */

/******************  Bit definition for NVIC_PRI4 register  *******************/
#define  NVIC_IPR4_PRI_16                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 16 */
#define  NVIC_IPR4_PRI_17                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 17 */
#define  NVIC_IPR4_PRI_18                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 18 */
#define  NVIC_IPR4_PRI_19                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 19 */

/******************  Bit definition for NVIC_PRI5 register  *******************/
#define  NVIC_IPR5_PRI_20                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 20 */
#define  NVIC_IPR5_PRI_21                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 21 */
#define  NVIC_IPR5_PRI_22                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 22 */
#define  NVIC_IPR5_PRI_23                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 23 */

/******************  Bit definition for NVIC_PRI6 register  *******************/
#define  NVIC_IPR6_PRI_24                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 24 */
#define  NVIC_IPR6_PRI_25                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 25 */
#define  NVIC_IPR6_PRI_26                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 26 */
#define  NVIC_IPR6_PRI_27                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 27 */

/******************  Bit definition for NVIC_PRI7 register  *******************/
#define  NVIC_IPR7_PRI_28                    ((uint32_t)0x000000FF)        /*!< Priority of interrupt 28 */
#define  NVIC_IPR7_PRI_29                    ((uint32_t)0x0000FF00)        /*!< Priority of interrupt 29 */
#define  NVIC_IPR7_PRI_30                    ((uint32_t)0x00FF0000)        /*!< Priority of interrupt 30 */
#define  NVIC_IPR7_PRI_31                    ((uint32_t)0xFF000000)        /*!< Priority of interrupt 31 */

/******************  Bit definition for SCB_CPUID register  *******************/
#define  SCB_CPUID_REVISION                  ((uint32_t)0x0000000F)        /*!< Implementation defined revision number */
#define  SCB_CPUID_PARTNO                    ((uint32_t)0x0000FFF0)        /*!< Number of processor within family */
#define  SCB_CPUID_Constant                  ((uint32_t)0x000F0000)        /*!< Reads as 0x0F */
#define  SCB_CPUID_VARIANT                   ((uint32_t)0x00F00000)        /*!< Implementation defined variant number */
#define  SCB_CPUID_IMPLEMENTER               ((uint32_t)0xFF000000)        /*!< Implementer code. ARM is 0x41 */

/*******************  Bit definition for SCB_ICSR register  *******************/
#define  SCB_ICSR_VECTACTIVE                 ((uint32_t)0x000001FF)        /*!< Active ISR number field */
#define  SCB_ICSR_RETTOBASE                  ((uint32_t)0x00000800)        /*!< All active exceptions minus the IPSR_current_exception yields the empty set */
#define  SCB_ICSR_VECTPENDING                ((uint32_t)0x003FF000)        /*!< Pending ISR number field */
#define  SCB_ICSR_ISRPENDING                 ((uint32_t)0x00400000)        /*!< Interrupt pending flag */
#define  SCB_ICSR_ISRPREEMPT                 ((uint32_t)0x00800000)        /*!< It indicates that a pending interrupt becomes active in the next running cycle */
#define  SCB_ICSR_PENDSTCLR                  ((uint32_t)0x02000000)        /*!< Clear pending SysTick bit */
#define  SCB_ICSR_PENDSTSET                  ((uint32_t)0x04000000)        /*!< Set pending SysTick bit */
#define  SCB_ICSR_PENDSVCLR                  ((uint32_t)0x08000000)        /*!< Clear pending pendSV bit */
#define  SCB_ICSR_PENDSVSET                  ((uint32_t)0x10000000)        /*!< Set pending pendSV bit */
#define  SCB_ICSR_NMIPENDSET                 ((uint32_t)0x80000000)        /*!< Set pending NMI bit */

/*******************  Bit definition for SCB_VTOR register  *******************/
#define  SCB_VTOR_TBLOFF                     ((uint32_t)0x1FFFFF80)        /*!< Vector table base offset field */
#define  SCB_VTOR_TBLBASE                    ((uint32_t)0x20000000)        /*!< Table base in code(0) or RAM(1) */

/*!<*****************  Bit definition for SCB_AIRCR register  *******************/
#define  SCB_AIRCR_VECTRESET                 ((uint32_t)0x00000001)        /*!< System Reset bit */
#define  SCB_AIRCR_VECTCLRACTIVE             ((uint32_t)0x00000002)        /*!< Clear active vector bit */
#define  SCB_AIRCR_SYSRESETREQ               ((uint32_t)0x00000004)        /*!< Requests chip control logic to generate a reset */

#define  SCB_AIRCR_PRIGROUP                  ((uint32_t)0x00000700)        /*!< PRIGROUP[2:0] bits (Priority group) */
#define  SCB_AIRCR_PRIGROUP_0                ((uint32_t)0x00000100)        /*!< Bit 0 */
#define  SCB_AIRCR_PRIGROUP_1                ((uint32_t)0x00000200)        /*!< Bit 1 */
#define  SCB_AIRCR_PRIGROUP_2                ((uint32_t)0x00000400)        /*!< Bit 2  */

/* prority group configuration */
#define  SCB_AIRCR_PRIGROUP0                 ((uint32_t)0x00000000)        /*!< Priority group=0 (7 bits of pre-emption priority, 1 bit of subpriority) */
#define  SCB_AIRCR_PRIGROUP1                 ((uint32_t)0x00000100)        /*!< Priority group=1 (6 bits of pre-emption priority, 2 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP2                 ((uint32_t)0x00000200)        /*!< Priority group=2 (5 bits of pre-emption priority, 3 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP3                 ((uint32_t)0x00000300)        /*!< Priority group=3 (4 bits of pre-emption priority, 4 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP4                 ((uint32_t)0x00000400)        /*!< Priority group=4 (3 bits of pre-emption priority, 5 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP5                 ((uint32_t)0x00000500)        /*!< Priority group=5 (2 bits of pre-emption priority, 6 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP6                 ((uint32_t)0x00000600)        /*!< Priority group=6 (1 bit of pre-emption priority, 7 bits of subpriority) */
#define  SCB_AIRCR_PRIGROUP7                 ((uint32_t)0x00000700)        /*!< Priority group=7 (no pre-emption priority, 8 bits of subpriority) */

#define  SCB_AIRCR_ENDIANESS                 ((uint32_t)0x00008000)        /*!< Data endianness bit */
#define  SCB_AIRCR_VECTKEY                   ((uint32_t)0xFFFF0000)        /*!< Register key (VECTKEY) - Reads as 0xFA05 (VECTKEYSTAT) */

/*******************  Bit definition for SCB_SCR register  ********************/
#define  SCB_SCR_SLEEPONEXIT                 ((uint8_t)0x02)               /*!< Sleep on exit bit */
#define  SCB_SCR_SLEEPDEEP                   ((uint8_t)0x04)               /*!< Sleep deep bit */
#define  SCB_SCR_SEVONPEND                   ((uint8_t)0x10)               /*!< Wake up from WFE */

/********************  Bit definition for SCB_CCR register  *******************/
#define  SCB_CCR_NONBASETHRDENA              ((uint16_t)0x0001)            /*!< Thread mode can be entered from any level in Handler mode by controlled return value */
#define  SCB_CCR_USERSETMPEND                ((uint16_t)0x0002)            /*!< Enables user code to write the Software Trigger Interrupt register to trigger (pend) a Main exception */
#define  SCB_CCR_UNALIGN_TRP                 ((uint16_t)0x0008)            /*!< Trap for unaligned access */
#define  SCB_CCR_DIV_0_TRP                   ((uint16_t)0x0010)            /*!< Trap on Divide by 0 */
#define  SCB_CCR_BFHFNMIGN                   ((uint16_t)0x0100)            /*!< Handlers running at priority -1 and -2 */
#define  SCB_CCR_STKALIGN                    ((uint16_t)0x0200)            /*!< On exception entry, the SP used prior to the exception is adjusted to be 8-byte aligned */

/*******************  Bit definition for SCB_SHPR register ********************/
#define  SCB_SHPR_PRI_N                      ((uint32_t)0x000000FF)        /*!< Priority of system handler 4,8, and 12. Mem Manage, reserved and Debug Monitor */
#define  SCB_SHPR_PRI_N1                     ((uint32_t)0x0000FF00)        /*!< Priority of system handler 5,9, and 13. Bus Fault, reserved and reserved */
#define  SCB_SHPR_PRI_N2                     ((uint32_t)0x00FF0000)        /*!< Priority of system handler 6,10, and 14. Usage Fault, reserved and PendSV */
#define  SCB_SHPR_PRI_N3                     ((uint32_t)0xFF000000)        /*!< Priority of system handler 7,11, and 15. Reserved, SVCall and SysTick */

/******************  Bit definition for SCB_SHCSR register  *******************/
#define  SCB_SHCSR_MEMFAULTACT               ((uint32_t)0x00000001)        /*!< MemManage is active */
#define  SCB_SHCSR_BUSFAULTACT               ((uint32_t)0x00000002)        /*!< BusFault is active */
#define  SCB_SHCSR_USGFAULTACT               ((uint32_t)0x00000008)        /*!< UsageFault is active */
#define  SCB_SHCSR_SVCALLACT                 ((uint32_t)0x00000080)        /*!< SVCall is active */
#define  SCB_SHCSR_MONITORACT                ((uint32_t)0x00000100)        /*!< Monitor is active */
#define  SCB_SHCSR_PENDSVACT                 ((uint32_t)0x00000400)        /*!< PendSV is active */
#define  SCB_SHCSR_SYSTICKACT                ((uint32_t)0x00000800)        /*!< SysTick is active */
#define  SCB_SHCSR_USGFAULTPENDED            ((uint32_t)0x00001000)        /*!< Usage Fault is pended */
#define  SCB_SHCSR_MEMFAULTPENDED            ((uint32_t)0x00002000)        /*!< MemManage is pended */
#define  SCB_SHCSR_BUSFAULTPENDED            ((uint32_t)0x00004000)        /*!< Bus Fault is pended */
#define  SCB_SHCSR_SVCALLPENDED              ((uint32_t)0x00008000)        /*!< SVCall is pended */
#define  SCB_SHCSR_MEMFAULTENA               ((uint32_t)0x00010000)        /*!< MemManage enable */
#define  SCB_SHCSR_BUSFAULTENA               ((uint32_t)0x00020000)        /*!< Bus Fault enable */
#define  SCB_SHCSR_USGFAULTENA               ((uint32_t)0x00040000)        /*!< UsageFault enable */

/*******************  Bit definition for SCB_CFSR register  *******************/
/*!< MFSR */
#define  SCB_CFSR_IACCVIOL                   ((uint32_t)0x00000001)        /*!< Instruction access violation */
#define  SCB_CFSR_DACCVIOL                   ((uint32_t)0x00000002)        /*!< Data access violation */
#define  SCB_CFSR_MUNSTKERR                  ((uint32_t)0x00000008)        /*!< Unstacking error */
#define  SCB_CFSR_MSTKERR                    ((uint32_t)0x00000010)        /*!< Stacking error */
#define  SCB_CFSR_MMARVALID                  ((uint32_t)0x00000080)        /*!< Memory Manage Address Register address valid flag */
/*!< BFSR */
#define  SCB_CFSR_IBUSERR                    ((uint32_t)0x00000100)        /*!< Instruction bus error flag */
#define  SCB_CFSR_PRECISERR                  ((uint32_t)0x00000200)        /*!< Precise data bus error */
#define  SCB_CFSR_IMPRECISERR                ((uint32_t)0x00000400)        /*!< Imprecise data bus error */
#define  SCB_CFSR_UNSTKERR                   ((uint32_t)0x00000800)        /*!< Unstacking error */
#define  SCB_CFSR_STKERR                     ((uint32_t)0x00001000)        /*!< Stacking error */
#define  SCB_CFSR_BFARVALID                  ((uint32_t)0x00008000)        /*!< Bus Fault Address Register address valid flag */
/*!< UFSR */
#define  SCB_CFSR_UNDEFINSTR                 ((uint32_t)0x00010000)        /*!< The processor attempt to execute an undefined instruction */
#define  SCB_CFSR_INVSTATE                   ((uint32_t)0x00020000)        /*!< Invalid combination of EPSR and instruction */
#define  SCB_CFSR_INVPC                      ((uint32_t)0x00040000)        /*!< Attempt to load EXC_RETURN into pc illegally */
#define  SCB_CFSR_NOCP                       ((uint32_t)0x00080000)        /*!< Attempt to use a coprocessor instruction */
#define  SCB_CFSR_UNALIGNED                  ((uint32_t)0x01000000)        /*!< Fault occurs when there is an attempt to make an unaligned memory access */
#define  SCB_CFSR_DIVBYZERO                  ((uint32_t)0x02000000)        /*!< Fault occurs when SDIV or DIV instruction is used with a divisor of 0 */

/*******************  Bit definition for SCB_HFSR register  *******************/
#define  SCB_HFSR_VECTTBL                    ((uint32_t)0x00000002)        /*!< Fault occurs because of vector table read on exception processing */
#define  SCB_HFSR_FORCED                     ((uint32_t)0x40000000)        /*!< Hard Fault activated when a configurable Fault was received and cannot activate */
#define  SCB_HFSR_DEBUGEVT                   ((uint32_t)0x80000000)        /*!< Fault related to debug */

/*******************  Bit definition for SCB_DFSR register  *******************/
#define  SCB_DFSR_HALTED                     ((uint8_t)0x01)               /*!< Halt request flag */
#define  SCB_DFSR_BKPT                       ((uint8_t)0x02)               /*!< BKPT flag */
#define  SCB_DFSR_DWTTRAP                    ((uint8_t)0x04)               /*!< Data Watchpoint and Trace (DWT) flag */
#define  SCB_DFSR_VCATCH                     ((uint8_t)0x08)               /*!< Vector catch flag */
#define  SCB_DFSR_EXTERNAL                   ((uint8_t)0x10)               /*!< External debug request flag */

/*******************  Bit definition for SCB_MMFAR register  ******************/
#define  SCB_MMFAR_ADDRESS                   ((uint32_t)0xFFFFFFFF)        /*!< Mem Manage fault address field */

/*******************  Bit definition for SCB_BFAR register  *******************/
#define  SCB_BFAR_ADDRESS                    ((uint32_t)0xFFFFFFFF)        /*!< Bus fault address field */

/*******************  Bit definition for SCB_afsr register  *******************/
#define  SCB_AFSR_IMPDEF                     ((uint32_t)0xFFFFFFFF)        /*!< Implementation defined */

/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for EXTI_IMR register  *******************/
#define  EXTI_IMR_MR0                        ((uint32_t)0x00000001)        /*!< Interrupt Mask on line 0 */
#define  EXTI_IMR_MR1                        ((uint32_t)0x00000002)        /*!< Interrupt Mask on line 1 */
#define  EXTI_IMR_MR2                        ((uint32_t)0x00000004)        /*!< Interrupt Mask on line 2 */
#define  EXTI_IMR_MR3                        ((uint32_t)0x00000008)        /*!< Interrupt Mask on line 3 */
#define  EXTI_IMR_MR4                        ((uint32_t)0x00000010)        /*!< Interrupt Mask on line 4 */
#define  EXTI_IMR_MR5                        ((uint32_t)0x00000020)        /*!< Interrupt Mask on line 5 */
#define  EXTI_IMR_MR6                        ((uint32_t)0x00000040)        /*!< Interrupt Mask on line 6 */
#define  EXTI_IMR_MR7                        ((uint32_t)0x00000080)        /*!< Interrupt Mask on line 7 */
#define  EXTI_IMR_MR8                        ((uint32_t)0x00000100)        /*!< Interrupt Mask on line 8 */
#define  EXTI_IMR_MR9                        ((uint32_t)0x00000200)        /*!< Interrupt Mask on line 9 */
#define  EXTI_IMR_MR10                       ((uint32_t)0x00000400)        /*!< Interrupt Mask on line 10 */
#define  EXTI_IMR_MR11                       ((uint32_t)0x00000800)        /*!< Interrupt Mask on line 11 */
#define  EXTI_IMR_MR12                       ((uint32_t)0x00001000)        /*!< Interrupt Mask on line 12 */
#define  EXTI_IMR_MR13                       ((uint32_t)0x00002000)        /*!< Interrupt Mask on line 13 */
#define  EXTI_IMR_MR14                       ((uint32_t)0x00004000)        /*!< Interrupt Mask on line 14 */
#define  EXTI_IMR_MR15                       ((uint32_t)0x00008000)        /*!< Interrupt Mask on line 15 */
#define  EXTI_IMR_MR16                       ((uint32_t)0x00010000)        /*!< Interrupt Mask on line 16 */
#define  EXTI_IMR_MR17                       ((uint32_t)0x00020000)        /*!< Interrupt Mask on line 17 */
#define  EXTI_IMR_MR18                       ((uint32_t)0x00040000)        /*!< Interrupt Mask on line 18 */
#define  EXTI_IMR_MR19                       ((uint32_t)0x00080000)        /*!< Interrupt Mask on line 19 */

/*******************  Bit definition for EXTI_EMR register  *******************/
#define  EXTI_EMR_MR0                        ((uint32_t)0x00000001)        /*!< Event Mask on line 0 */
#define  EXTI_EMR_MR1                        ((uint32_t)0x00000002)        /*!< Event Mask on line 1 */
#define  EXTI_EMR_MR2                        ((uint32_t)0x00000004)        /*!< Event Mask on line 2 */
#define  EXTI_EMR_MR3                        ((uint32_t)0x00000008)        /*!< Event Mask on line 3 */
#define  EXTI_EMR_MR4                        ((uint32_t)0x00000010)        /*!< Event Mask on line 4 */
#define  EXTI_EMR_MR5                        ((uint32_t)0x00000020)        /*!< Event Mask on line 5 */
#define  EXTI_EMR_MR6                        ((uint32_t)0x00000040)        /*!< Event Mask on line 6 */
#define  EXTI_EMR_MR7                        ((uint32_t)0x00000080)        /*!< Event Mask on line 7 */
#define  EXTI_EMR_MR8                        ((uint32_t)0x00000100)        /*!< Event Mask on line 8 */
#define  EXTI_EMR_MR9                        ((uint32_t)0x00000200)        /*!< Event Mask on line 9 */
#define  EXTI_EMR_MR10                       ((uint32_t)0x00000400)        /*!< Event Mask on line 10 */
#define  EXTI_EMR_MR11                       ((uint32_t)0x00000800)        /*!< Event Mask on line 11 */
#define  EXTI_EMR_MR12                       ((uint32_t)0x00001000)        /*!< Event Mask on line 12 */
#define  EXTI_EMR_MR13                       ((uint32_t)0x00002000)        /*!< Event Mask on line 13 */
#define  EXTI_EMR_MR14                       ((uint32_t)0x00004000)        /*!< Event Mask on line 14 */
#define  EXTI_EMR_MR15                       ((uint32_t)0x00008000)        /*!< Event Mask on line 15 */
#define  EXTI_EMR_MR16                       ((uint32_t)0x00010000)        /*!< Event Mask on line 16 */
#define  EXTI_EMR_MR17                       ((uint32_t)0x00020000)        /*!< Event Mask on line 17 */
#define  EXTI_EMR_MR18                       ((uint32_t)0x00040000)        /*!< Event Mask on line 18 */
#define  EXTI_EMR_MR19                       ((uint32_t)0x00080000)        /*!< Event Mask on line 19 */

/******************  Bit definition for EXTI_RTSR register  *******************/
#define  EXTI_RTSR_TR0                       ((uint32_t)0x00000001)        /*!< Rising trigger event configuration bit of line 0 */
#define  EXTI_RTSR_TR1                       ((uint32_t)0x00000002)        /*!< Rising trigger event configuration bit of line 1 */
#define  EXTI_RTSR_TR2                       ((uint32_t)0x00000004)        /*!< Rising trigger event configuration bit of line 2 */
#define  EXTI_RTSR_TR3                       ((uint32_t)0x00000008)        /*!< Rising trigger event configuration bit of line 3 */
#define  EXTI_RTSR_TR4                       ((uint32_t)0x00000010)        /*!< Rising trigger event configuration bit of line 4 */
#define  EXTI_RTSR_TR5                       ((uint32_t)0x00000020)        /*!< Rising trigger event configuration bit of line 5 */
#define  EXTI_RTSR_TR6                       ((uint32_t)0x00000040)        /*!< Rising trigger event configuration bit of line 6 */
#define  EXTI_RTSR_TR7                       ((uint32_t)0x00000080)        /*!< Rising trigger event configuration bit of line 7 */
#define  EXTI_RTSR_TR8                       ((uint32_t)0x00000100)        /*!< Rising trigger event configuration bit of line 8 */
#define  EXTI_RTSR_TR9                       ((uint32_t)0x00000200)        /*!< Rising trigger event configuration bit of line 9 */
#define  EXTI_RTSR_TR10                      ((uint32_t)0x00000400)        /*!< Rising trigger event configuration bit of line 10 */
#define  EXTI_RTSR_TR11                      ((uint32_t)0x00000800)        /*!< Rising trigger event configuration bit of line 11 */
#define  EXTI_RTSR_TR12                      ((uint32_t)0x00001000)        /*!< Rising trigger event configuration bit of line 12 */
#define  EXTI_RTSR_TR13                      ((uint32_t)0x00002000)        /*!< Rising trigger event configuration bit of line 13 */
#define  EXTI_RTSR_TR14                      ((uint32_t)0x00004000)        /*!< Rising trigger event configuration bit of line 14 */
#define  EXTI_RTSR_TR15                      ((uint32_t)0x00008000)        /*!< Rising trigger event configuration bit of line 15 */
#define  EXTI_RTSR_TR16                      ((uint32_t)0x00010000)        /*!< Rising trigger event configuration bit of line 16 */
#define  EXTI_RTSR_TR17                      ((uint32_t)0x00020000)        /*!< Rising trigger event configuration bit of line 17 */
#define  EXTI_RTSR_TR18                      ((uint32_t)0x00040000)        /*!< Rising trigger event configuration bit of line 18 */
#define  EXTI_RTSR_TR19                      ((uint32_t)0x00080000)        /*!< Rising trigger event configuration bit of line 19 */

/******************  Bit definition for EXTI_FTSR register  *******************/
#define  EXTI_FTSR_TR0                       ((uint32_t)0x00000001)        /*!< Falling trigger event configuration bit of line 0 */
#define  EXTI_FTSR_TR1                       ((uint32_t)0x00000002)        /*!< Falling trigger event configuration bit of line 1 */
#define  EXTI_FTSR_TR2                       ((uint32_t)0x00000004)        /*!< Falling trigger event configuration bit of line 2 */
#define  EXTI_FTSR_TR3                       ((uint32_t)0x00000008)        /*!< Falling trigger event configuration bit of line 3 */
#define  EXTI_FTSR_TR4                       ((uint32_t)0x00000010)        /*!< Falling trigger event configuration bit of line 4 */
#define  EXTI_FTSR_TR5                       ((uint32_t)0x00000020)        /*!< Falling trigger event configuration bit of line 5 */
#define  EXTI_FTSR_TR6                       ((uint32_t)0x00000040)        /*!< Falling trigger event configuration bit of line 6 */
#define  EXTI_FTSR_TR7                       ((uint32_t)0x00000080)        /*!< Falling trigger event configuration bit of line 7 */
#define  EXTI_FTSR_TR8                       ((uint32_t)0x00000100)        /*!< Falling trigger event configuration bit of line 8 */
#define  EXTI_FTSR_TR9                       ((uint32_t)0x00000200)        /*!< Falling trigger event configuration bit of line 9 */
#define  EXTI_FTSR_TR10                      ((uint32_t)0x00000400)        /*!< Falling trigger event configuration bit of line 10 */
#define  EXTI_FTSR_TR11                      ((uint32_t)0x00000800)        /*!< Falling trigger event configuration bit of line 11 */
#define  EXTI_FTSR_TR12                      ((uint32_t)0x00001000)        /*!< Falling trigger event configuration bit of line 12 */
#define  EXTI_FTSR_TR13                      ((uint32_t)0x00002000)        /*!< Falling trigger event configuration bit of line 13 */
#define  EXTI_FTSR_TR14                      ((uint32_t)0x00004000)        /*!< Falling trigger event configuration bit of line 14 */
#define  EXTI_FTSR_TR15                      ((uint32_t)0x00008000)        /*!< Falling trigger event configuration bit of line 15 */
#define  EXTI_FTSR_TR16                      ((uint32_t)0x00010000)        /*!< Falling trigger event configuration bit of line 16 */
#define  EXTI_FTSR_TR17                      ((uint32_t)0x00020000)        /*!< Falling trigger event configuration bit of line 17 */
#define  EXTI_FTSR_TR18                      ((uint32_t)0x00040000)        /*!< Falling trigger event configuration bit of line 18 */
#define  EXTI_FTSR_TR19                      ((uint32_t)0x00080000)        /*!< Falling trigger event configuration bit of line 19 */

/******************  Bit definition for EXTI_SWIER register  ******************/
#define  EXTI_SWIER_SWIER0                   ((uint32_t)0x00000001)        /*!< Software Interrupt on line 0 */
#define  EXTI_SWIER_SWIER1                   ((uint32_t)0x00000002)        /*!< Software Interrupt on line 1 */
#define  EXTI_SWIER_SWIER2                   ((uint32_t)0x00000004)        /*!< Software Interrupt on line 2 */
#define  EXTI_SWIER_SWIER3                   ((uint32_t)0x00000008)        /*!< Software Interrupt on line 3 */
#define  EXTI_SWIER_SWIER4                   ((uint32_t)0x00000010)        /*!< Software Interrupt on line 4 */
#define  EXTI_SWIER_SWIER5                   ((uint32_t)0x00000020)        /*!< Software Interrupt on line 5 */
#define  EXTI_SWIER_SWIER6                   ((uint32_t)0x00000040)        /*!< Software Interrupt on line 6 */
#define  EXTI_SWIER_SWIER7                   ((uint32_t)0x00000080)        /*!< Software Interrupt on line 7 */
#define  EXTI_SWIER_SWIER8                   ((uint32_t)0x00000100)        /*!< Software Interrupt on line 8 */
#define  EXTI_SWIER_SWIER9                   ((uint32_t)0x00000200)        /*!< Software Interrupt on line 9 */
#define  EXTI_SWIER_SWIER10                  ((uint32_t)0x00000400)        /*!< Software Interrupt on line 10 */
#define  EXTI_SWIER_SWIER11                  ((uint32_t)0x00000800)        /*!< Software Interrupt on line 11 */
#define  EXTI_SWIER_SWIER12                  ((uint32_t)0x00001000)        /*!< Software Interrupt on line 12 */
#define  EXTI_SWIER_SWIER13                  ((uint32_t)0x00002000)        /*!< Software Interrupt on line 13 */
#define  EXTI_SWIER_SWIER14                  ((uint32_t)0x00004000)        /*!< Software Interrupt on line 14 */
#define  EXTI_SWIER_SWIER15                  ((uint32_t)0x00008000)        /*!< Software Interrupt on line 15 */
#define  EXTI_SWIER_SWIER16                  ((uint32_t)0x00010000)        /*!< Software Interrupt on line 16 */
#define  EXTI_SWIER_SWIER17                  ((uint32_t)0x00020000)        /*!< Software Interrupt on line 17 */
#define  EXTI_SWIER_SWIER18                  ((uint32_t)0x00040000)        /*!< Software Interrupt on line 18 */
#define  EXTI_SWIER_SWIER19                  ((uint32_t)0x00080000)        /*!< Software Interrupt on line 19 */

/*******************  Bit definition for EXTI_PR register  ********************/
#define  EXTI_PR_PR0                         ((uint32_t)0x00000001)        /*!< Pending bit for line 0 */
#define  EXTI_PR_PR1                         ((uint32_t)0x00000002)        /*!< Pending bit for line 1 */
#define  EXTI_PR_PR2                         ((uint32_t)0x00000004)        /*!< Pending bit for line 2 */
#define  EXTI_PR_PR3                         ((uint32_t)0x00000008)        /*!< Pending bit for line 3 */
#define  EXTI_PR_PR4                         ((uint32_t)0x00000010)        /*!< Pending bit for line 4 */
#define  EXTI_PR_PR5                         ((uint32_t)0x00000020)        /*!< Pending bit for line 5 */
#define  EXTI_PR_PR6                         ((uint32_t)0x00000040)        /*!< Pending bit for line 6 */
#define  EXTI_PR_PR7                         ((uint32_t)0x00000080)        /*!< Pending bit for line 7 */
#define  EXTI_PR_PR8                         ((uint32_t)0x00000100)        /*!< Pending bit for line 8 */
#define  EXTI_PR_PR9                         ((uint32_t)0x00000200)        /*!< Pending bit for line 9 */
#define  EXTI_PR_PR10                        ((uint32_t)0x00000400)        /*!< Pending bit for line 10 */
#define  EXTI_PR_PR11                        ((uint32_t)0x00000800)        /*!< Pending bit for line 11 */
#define  EXTI_PR_PR12                        ((uint32_t)0x00001000)        /*!< Pending bit for line 12 */
#define  EXTI_PR_PR13                        ((uint32_t)0x00002000)        /*!< Pending bit for line 13 */
#define  EXTI_PR_PR14                        ((uint32_t)0x00004000)        /*!< Pending bit for line 14 */
#define  EXTI_PR_PR15                        ((uint32_t)0x00008000)        /*!< Pending bit for line 15 */
#define  EXTI_PR_PR16                        ((uint32_t)0x00010000)        /*!< Pending bit for line 16 */
#define  EXTI_PR_PR17                        ((uint32_t)0x00020000)        /*!< Pending bit for line 17 */
#define  EXTI_PR_PR18                        ((uint32_t)0x00040000)        /*!< Pending bit for line 18 */
#define  EXTI_PR_PR19                        ((uint32_t)0x00080000)        /*!< Pending bit for line 19 */

/******************************************************************************/
/*                                                                            */
/*                             DMA Controller                                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for DMA_ISR register  ********************/
#define  DMA_ISR_GIF1                        ((uint32_t)0x00000001)        /*!< Channel 1 Global interrupt flag */
#define  DMA_ISR_TCIF1                       ((uint32_t)0x00000002)        /*!< Channel 1 Transfer Complete flag */
#define  DMA_ISR_HTIF1                       ((uint32_t)0x00000004)        /*!< Channel 1 Half Transfer flag */
#define  DMA_ISR_TEIF1                       ((uint32_t)0x00000008)        /*!< Channel 1 Transfer Error flag */
#define  DMA_ISR_GIF2                        ((uint32_t)0x00000010)        /*!< Channel 2 Global interrupt flag */
#define  DMA_ISR_TCIF2                       ((uint32_t)0x00000020)        /*!< Channel 2 Transfer Complete flag */
#define  DMA_ISR_HTIF2                       ((uint32_t)0x00000040)        /*!< Channel 2 Half Transfer flag */
#define  DMA_ISR_TEIF2                       ((uint32_t)0x00000080)        /*!< Channel 2 Transfer Error flag */
#define  DMA_ISR_GIF3                        ((uint32_t)0x00000100)        /*!< Channel 3 Global interrupt flag */
#define  DMA_ISR_TCIF3                       ((uint32_t)0x00000200)        /*!< Channel 3 Transfer Complete flag */
#define  DMA_ISR_HTIF3                       ((uint32_t)0x00000400)        /*!< Channel 3 Half Transfer flag */
#define  DMA_ISR_TEIF3                       ((uint32_t)0x00000800)        /*!< Channel 3 Transfer Error flag */
#define  DMA_ISR_GIF4                        ((uint32_t)0x00001000)        /*!< Channel 4 Global interrupt flag */
#define  DMA_ISR_TCIF4                       ((uint32_t)0x00002000)        /*!< Channel 4 Transfer Complete flag */
#define  DMA_ISR_HTIF4                       ((uint32_t)0x00004000)        /*!< Channel 4 Half Transfer flag */
#define  DMA_ISR_TEIF4                       ((uint32_t)0x00008000)        /*!< Channel 4 Transfer Error flag */
#define  DMA_ISR_GIF5                        ((uint32_t)0x00010000)        /*!< Channel 5 Global interrupt flag */
#define  DMA_ISR_TCIF5                       ((uint32_t)0x00020000)        /*!< Channel 5 Transfer Complete flag */
#define  DMA_ISR_HTIF5                       ((uint32_t)0x00040000)        /*!< Channel 5 Half Transfer flag */
#define  DMA_ISR_TEIF5                       ((uint32_t)0x00080000)        /*!< Channel 5 Transfer Error flag */
#define  DMA_ISR_GIF6                        ((uint32_t)0x00100000)        /*!< Channel 6 Global interrupt flag */
#define  DMA_ISR_TCIF6                       ((uint32_t)0x00200000)        /*!< Channel 6 Transfer Complete flag */
#define  DMA_ISR_HTIF6                       ((uint32_t)0x00400000)        /*!< Channel 6 Half Transfer flag */
#define  DMA_ISR_TEIF6                       ((uint32_t)0x00800000)        /*!< Channel 6 Transfer Error flag */
#define  DMA_ISR_GIF7                        ((uint32_t)0x01000000)        /*!< Channel 7 Global interrupt flag */
#define  DMA_ISR_TCIF7                       ((uint32_t)0x02000000)        /*!< Channel 7 Transfer Complete flag */
#define  DMA_ISR_HTIF7                       ((uint32_t)0x04000000)        /*!< Channel 7 Half Transfer flag */
#define  DMA_ISR_TEIF7                       ((uint32_t)0x08000000)        /*!< Channel 7 Transfer Error flag */

/*******************  Bit definition for DMA_IFCR register  *******************/
#define  DMA_IFCR_CGIF1                      ((uint32_t)0x00000001)        /*!< Channel 1 Global interrupt clear */
#define  DMA_IFCR_CTCIF1                     ((uint32_t)0x00000002)        /*!< Channel 1 Transfer Complete clear */
#define  DMA_IFCR_CHTIF1                     ((uint32_t)0x00000004)        /*!< Channel 1 Half Transfer clear */
#define  DMA_IFCR_CTEIF1                     ((uint32_t)0x00000008)        /*!< Channel 1 Transfer Error clear */
#define  DMA_IFCR_CGIF2                      ((uint32_t)0x00000010)        /*!< Channel 2 Global interrupt clear */
#define  DMA_IFCR_CTCIF2                     ((uint32_t)0x00000020)        /*!< Channel 2 Transfer Complete clear */
#define  DMA_IFCR_CHTIF2                     ((uint32_t)0x00000040)        /*!< Channel 2 Half Transfer clear */
#define  DMA_IFCR_CTEIF2                     ((uint32_t)0x00000080)        /*!< Channel 2 Transfer Error clear */
#define  DMA_IFCR_CGIF3                      ((uint32_t)0x00000100)        /*!< Channel 3 Global interrupt clear */
#define  DMA_IFCR_CTCIF3                     ((uint32_t)0x00000200)        /*!< Channel 3 Transfer Complete clear */
#define  DMA_IFCR_CHTIF3                     ((uint32_t)0x00000400)        /*!< Channel 3 Half Transfer clear */
#define  DMA_IFCR_CTEIF3                     ((uint32_t)0x00000800)        /*!< Channel 3 Transfer Error clear */
#define  DMA_IFCR_CGIF4                      ((uint32_t)0x00001000)        /*!< Channel 4 Global interrupt clear */
#define  DMA_IFCR_CTCIF4                     ((uint32_t)0x00002000)        /*!< Channel 4 Transfer Complete clear */
#define  DMA_IFCR_CHTIF4                     ((uint32_t)0x00004000)        /*!< Channel 4 Half Transfer clear */
#define  DMA_IFCR_CTEIF4                     ((uint32_t)0x00008000)        /*!< Channel 4 Transfer Error clear */
#define  DMA_IFCR_CGIF5                      ((uint32_t)0x00010000)        /*!< Channel 5 Global interrupt clear */
#define  DMA_IFCR_CTCIF5                     ((uint32_t)0x00020000)        /*!< Channel 5 Transfer Complete clear */
#define  DMA_IFCR_CHTIF5                     ((uint32_t)0x00040000)        /*!< Channel 5 Half Transfer clear */
#define  DMA_IFCR_CTEIF5                     ((uint32_t)0x00080000)        /*!< Channel 5 Transfer Error clear */
#define  DMA_IFCR_CGIF6                      ((uint32_t)0x00100000)        /*!< Channel 6 Global interrupt clear */
#define  DMA_IFCR_CTCIF6                     ((uint32_t)0x00200000)        /*!< Channel 6 Transfer Complete clear */
#define  DMA_IFCR_CHTIF6                     ((uint32_t)0x00400000)        /*!< Channel 6 Half Transfer clear */
#define  DMA_IFCR_CTEIF6                     ((uint32_t)0x00800000)        /*!< Channel 6 Transfer Error clear */
#define  DMA_IFCR_CGIF7                      ((uint32_t)0x01000000)        /*!< Channel 7 Global interrupt clear */
#define  DMA_IFCR_CTCIF7                     ((uint32_t)0x02000000)        /*!< Channel 7 Transfer Complete clear */
#define  DMA_IFCR_CHTIF7                     ((uint32_t)0x04000000)        /*!< Channel 7 Half Transfer clear */
#define  DMA_IFCR_CTEIF7                     ((uint32_t)0x08000000)        /*!< Channel 7 Transfer Error clear */

/*******************  Bit definition for DMA_CCR1 register  *******************/
#define  DMA_CCR1_EN                         ((uint16_t)0x0001)            /*!< Channel enable*/
#define  DMA_CCR1_TCIE                       ((uint16_t)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CCR1_HTIE                       ((uint16_t)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CCR1_TEIE                       ((uint16_t)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CCR1_DIR                        ((uint16_t)0x0010)            /*!< Data transfer direction */
#define  DMA_CCR1_CIRC                       ((uint16_t)0x0020)            /*!< Circular mode */
#define  DMA_CCR1_PINC                       ((uint16_t)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CCR1_MINC                       ((uint16_t)0x0080)            /*!< Memory increment mode */

#define  DMA_CCR1_PSIZE                      ((uint16_t)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR1_PSIZE_0                    ((uint16_t)0x0100)            /*!< Bit 0 */
#define  DMA_CCR1_PSIZE_1                    ((uint16_t)0x0200)            /*!< Bit 1 */

#define  DMA_CCR1_MSIZE                      ((uint16_t)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR1_MSIZE_0                    ((uint16_t)0x0400)            /*!< Bit 0 */
#define  DMA_CCR1_MSIZE_1                    ((uint16_t)0x0800)            /*!< Bit 1 */

#define  DMA_CCR1_PL                         ((uint16_t)0x3000)            /*!< PL[1:0] bits(Channel Priority level) */
#define  DMA_CCR1_PL_0                       ((uint16_t)0x1000)            /*!< Bit 0 */
#define  DMA_CCR1_PL_1                       ((uint16_t)0x2000)            /*!< Bit 1 */

#define  DMA_CCR1_MEM2MEM                    ((uint16_t)0x4000)            /*!< Memory to memory mode */

/*******************  Bit definition for DMA_CCR2 register  *******************/
#define  DMA_CCR2_EN                         ((uint16_t)0x0001)            /*!< Channel enable */
#define  DMA_CCR2_TCIE                       ((uint16_t)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CCR2_HTIE                       ((uint16_t)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CCR2_TEIE                       ((uint16_t)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CCR2_DIR                        ((uint16_t)0x0010)            /*!< Data transfer direction */
#define  DMA_CCR2_CIRC                       ((uint16_t)0x0020)            /*!< Circular mode */
#define  DMA_CCR2_PINC                       ((uint16_t)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CCR2_MINC                       ((uint16_t)0x0080)            /*!< Memory increment mode */

#define  DMA_CCR2_PSIZE                      ((uint16_t)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR2_PSIZE_0                    ((uint16_t)0x0100)            /*!< Bit 0 */
#define  DMA_CCR2_PSIZE_1                    ((uint16_t)0x0200)            /*!< Bit 1 */

#define  DMA_CCR2_MSIZE                      ((uint16_t)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR2_MSIZE_0                    ((uint16_t)0x0400)            /*!< Bit 0 */
#define  DMA_CCR2_MSIZE_1                    ((uint16_t)0x0800)            /*!< Bit 1 */

#define  DMA_CCR2_PL                         ((uint16_t)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR2_PL_0                       ((uint16_t)0x1000)            /*!< Bit 0 */
#define  DMA_CCR2_PL_1                       ((uint16_t)0x2000)            /*!< Bit 1 */

#define  DMA_CCR2_MEM2MEM                    ((uint16_t)0x4000)            /*!< Memory to memory mode */

/*******************  Bit definition for DMA_CCR3 register  *******************/
#define  DMA_CCR3_EN                         ((uint16_t)0x0001)            /*!< Channel enable */
#define  DMA_CCR3_TCIE                       ((uint16_t)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CCR3_HTIE                       ((uint16_t)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CCR3_TEIE                       ((uint16_t)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CCR3_DIR                        ((uint16_t)0x0010)            /*!< Data transfer direction */
#define  DMA_CCR3_CIRC                       ((uint16_t)0x0020)            /*!< Circular mode */
#define  DMA_CCR3_PINC                       ((uint16_t)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CCR3_MINC                       ((uint16_t)0x0080)            /*!< Memory increment mode */

#define  DMA_CCR3_PSIZE                      ((uint16_t)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR3_PSIZE_0                    ((uint16_t)0x0100)            /*!< Bit 0 */
#define  DMA_CCR3_PSIZE_1                    ((uint16_t)0x0200)            /*!< Bit 1 */

#define  DMA_CCR3_MSIZE                      ((uint16_t)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR3_MSIZE_0                    ((uint16_t)0x0400)            /*!< Bit 0 */
#define  DMA_CCR3_MSIZE_1                    ((uint16_t)0x0800)            /*!< Bit 1 */

#define  DMA_CCR3_PL                         ((uint16_t)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR3_PL_0                       ((uint16_t)0x1000)            /*!< Bit 0 */
#define  DMA_CCR3_PL_1                       ((uint16_t)0x2000)            /*!< Bit 1 */

#define  DMA_CCR3_MEM2MEM                    ((uint16_t)0x4000)            /*!< Memory to memory mode */

/*!<******************  Bit definition for DMA_CCR4 register  *******************/
#define  DMA_CCR4_EN                         ((uint16_t)0x0001)            /*!< Channel enable */
#define  DMA_CCR4_TCIE                       ((uint16_t)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CCR4_HTIE                       ((uint16_t)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CCR4_TEIE                       ((uint16_t)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CCR4_DIR                        ((uint16_t)0x0010)            /*!< Data transfer direction */
#define  DMA_CCR4_CIRC                       ((uint16_t)0x0020)            /*!< Circular mode */
#define  DMA_CCR4_PINC                       ((uint16_t)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CCR4_MINC                       ((uint16_t)0x0080)            /*!< Memory increment mode */

#define  DMA_CCR4_PSIZE                      ((uint16_t)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR4_PSIZE_0                    ((uint16_t)0x0100)            /*!< Bit 0 */
#define  DMA_CCR4_PSIZE_1                    ((uint16_t)0x0200)            /*!< Bit 1 */

#define  DMA_CCR4_MSIZE                      ((uint16_t)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR4_MSIZE_0                    ((uint16_t)0x0400)            /*!< Bit 0 */
#define  DMA_CCR4_MSIZE_1                    ((uint16_t)0x0800)            /*!< Bit 1 */

#define  DMA_CCR4_PL                         ((uint16_t)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR4_PL_0                       ((uint16_t)0x1000)            /*!< Bit 0 */
#define  DMA_CCR4_PL_1                       ((uint16_t)0x2000)            /*!< Bit 1 */

#define  DMA_CCR4_MEM2MEM                    ((uint16_t)0x4000)            /*!< Memory to memory mode */

/******************  Bit definition for DMA_CCR5 register  *******************/
#define  DMA_CCR5_EN                         ((uint16_t)0x0001)            /*!< Channel enable */
#define  DMA_CCR5_TCIE                       ((uint16_t)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CCR5_HTIE                       ((uint16_t)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CCR5_TEIE                       ((uint16_t)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CCR5_DIR                        ((uint16_t)0x0010)            /*!< Data transfer direction */
#define  DMA_CCR5_CIRC                       ((uint16_t)0x0020)            /*!< Circular mode */
#define  DMA_CCR5_PINC                       ((uint16_t)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CCR5_MINC                       ((uint16_t)0x0080)            /*!< Memory increment mode */

#define  DMA_CCR5_PSIZE                      ((uint16_t)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR5_PSIZE_0                    ((uint16_t)0x0100)            /*!< Bit 0 */
#define  DMA_CCR5_PSIZE_1                    ((uint16_t)0x0200)            /*!< Bit 1 */

#define  DMA_CCR5_MSIZE                      ((uint16_t)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR5_MSIZE_0                    ((uint16_t)0x0400)            /*!< Bit 0 */
#define  DMA_CCR5_MSIZE_1                    ((uint16_t)0x0800)            /*!< Bit 1 */

#define  DMA_CCR5_PL                         ((uint16_t)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR5_PL_0                       ((uint16_t)0x1000)            /*!< Bit 0 */
#define  DMA_CCR5_PL_1                       ((uint16_t)0x2000)            /*!< Bit 1 */

#define  DMA_CCR5_MEM2MEM                    ((uint16_t)0x4000)            /*!< Memory to memory mode enable */

/*******************  Bit definition for DMA_CCR6 register  *******************/
#define  DMA_CCR6_EN                         ((uint16_t)0x0001)            /*!< Channel enable */
#define  DMA_CCR6_TCIE                       ((uint16_t)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CCR6_HTIE                       ((uint16_t)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CCR6_TEIE                       ((uint16_t)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CCR6_DIR                        ((uint16_t)0x0010)            /*!< Data transfer direction */
#define  DMA_CCR6_CIRC                       ((uint16_t)0x0020)            /*!< Circular mode */
#define  DMA_CCR6_PINC                       ((uint16_t)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CCR6_MINC                       ((uint16_t)0x0080)            /*!< Memory increment mode */

#define  DMA_CCR6_PSIZE                      ((uint16_t)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR6_PSIZE_0                    ((uint16_t)0x0100)            /*!< Bit 0 */
#define  DMA_CCR6_PSIZE_1                    ((uint16_t)0x0200)            /*!< Bit 1 */

#define  DMA_CCR6_MSIZE                      ((uint16_t)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR6_MSIZE_0                    ((uint16_t)0x0400)            /*!< Bit 0 */
#define  DMA_CCR6_MSIZE_1                    ((uint16_t)0x0800)            /*!< Bit 1 */

#define  DMA_CCR6_PL                         ((uint16_t)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR6_PL_0                       ((uint16_t)0x1000)            /*!< Bit 0 */
#define  DMA_CCR6_PL_1                       ((uint16_t)0x2000)            /*!< Bit 1 */

#define  DMA_CCR6_MEM2MEM                    ((uint16_t)0x4000)            /*!< Memory to memory mode */

/*******************  Bit definition for DMA_CCR7 register  *******************/
#define  DMA_CCR7_EN                         ((uint16_t)0x0001)            /*!< Channel enable */
#define  DMA_CCR7_TCIE                       ((uint16_t)0x0002)            /*!< Transfer complete interrupt enable */
#define  DMA_CCR7_HTIE                       ((uint16_t)0x0004)            /*!< Half Transfer interrupt enable */
#define  DMA_CCR7_TEIE                       ((uint16_t)0x0008)            /*!< Transfer error interrupt enable */
#define  DMA_CCR7_DIR                        ((uint16_t)0x0010)            /*!< Data transfer direction */
#define  DMA_CCR7_CIRC                       ((uint16_t)0x0020)            /*!< Circular mode */
#define  DMA_CCR7_PINC                       ((uint16_t)0x0040)            /*!< Peripheral increment mode */
#define  DMA_CCR7_MINC                       ((uint16_t)0x0080)            /*!< Memory increment mode */

#define  DMA_CCR7_PSIZE            ,         ((uint16_t)0x0300)            /*!< PSIZE[1:0] bits (Peripheral size) */
#define  DMA_CCR7_PSIZE_0                    ((uint16_t)0x0100)            /*!< Bit 0 */
#define  DMA_CCR7_PSIZE_1                    ((uint16_t)0x0200)            /*!< Bit 1 */

#define  DMA_CCR7_MSIZE                      ((uint16_t)0x0C00)            /*!< MSIZE[1:0] bits (Memory size) */
#define  DMA_CCR7_MSIZE_0                    ((uint16_t)0x0400)            /*!< Bit 0 */
#define  DMA_CCR7_MSIZE_1                    ((uint16_t)0x0800)            /*!< Bit 1 */

#define  DMA_CCR7_PL                         ((uint16_t)0x3000)            /*!< PL[1:0] bits (Channel Priority level) */
#define  DMA_CCR7_PL_0                       ((uint16_t)0x1000)            /*!< Bit 0 */
#define  DMA_CCR7_PL_1                       ((uint16_t)0x2000)            /*!< Bit 1 */

#define  DMA_CCR7_MEM2MEM                    ((uint16_t)0x4000)            /*!< Memory to memory mode enable */

/******************  Bit definition for DMA_CNDTR1 register  ******************/
#define  DMA_CNDTR1_NDT                      ((uint16_t)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR2 register  ******************/
#define  DMA_CNDTR2_NDT                      ((uint16_t)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR3 register  ******************/
#define  DMA_CNDTR3_NDT                      ((uint16_t)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR4 register  ******************/
#define  DMA_CNDTR4_NDT                      ((uint16_t)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR5 register  ******************/
#define  DMA_CNDTR5_NDT                      ((uint16_t)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR6 register  ******************/
#define  DMA_CNDTR6_NDT                      ((uint16_t)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CNDTR7 register  ******************/
#define  DMA_CNDTR7_NDT                      ((uint16_t)0xFFFF)            /*!< Number of data to Transfer */

/******************  Bit definition for DMA_CPAR1 register  *******************/
#define  DMA_CPAR1_PA                        ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CPAR2 register  *******************/
#define  DMA_CPAR2_PA                        ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CPAR3 register  *******************/
#define  DMA_CPAR3_PA                        ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address */


/******************  Bit definition for DMA_CPAR4 register  *******************/
#define  DMA_CPAR4_PA                        ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CPAR5 register  *******************/
#define  DMA_CPAR5_PA                        ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CPAR6 register  *******************/
#define  DMA_CPAR6_PA                        ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address */


/******************  Bit definition for DMA_CPAR7 register  *******************/
#define  DMA_CPAR7_PA                        ((uint32_t)0xFFFFFFFF)        /*!< Peripheral Address */

/******************  Bit definition for DMA_CMAR1 register  *******************/
#define  DMA_CMAR1_MA                        ((uint32_t)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMAR2 register  *******************/
#define  DMA_CMAR2_MA                        ((uint32_t)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMAR3 register  *******************/
#define  DMA_CMAR3_MA                        ((uint32_t)0xFFFFFFFF)        /*!< Memory Address */


/******************  Bit definition for DMA_CMAR4 register  *******************/
#define  DMA_CMAR4_MA                        ((uint32_t)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMAR5 register  *******************/
#define  DMA_CMAR5_MA                        ((uint32_t)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMAR6 register  *******************/
#define  DMA_CMAR6_MA                        ((uint32_t)0xFFFFFFFF)        /*!< Memory Address */

/******************  Bit definition for DMA_CMAR7 register  *******************/
#define  DMA_CMAR7_MA                        ((uint32_t)0xFFFFFFFF)        /*!< Memory Address */

/******************************************************************************/
/*                                                                            */
/*                        Analog to Digital Converter                         */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for ADC_SR register  ********************/
#define  ADC_SR_AWD                          ((uint8_t)0x01)               /*!< Analog watchdog flag */
#define  ADC_SR_EOC                          ((uint8_t)0x02)               /*!< End of conversion */
#define  ADC_SR_JEOC                         ((uint8_t)0x04)               /*!< Injected channel end of conversion */
#define  ADC_SR_JSTRT                        ((uint8_t)0x08)               /*!< Injected channel Start flag */
#define  ADC_SR_STRT                         ((uint8_t)0x10)               /*!< Regular channel Start flag */

/*******************  Bit definition for ADC_CR1 register  ********************/
#define  ADC_CR1_AWDCH                       ((uint32_t)0x0000001F)        /*!< AWDCH[4:0] bits (Analog watchdog channel select bits) */
#define  ADC_CR1_AWDCH_0                     ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_CR1_AWDCH_1                     ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_CR1_AWDCH_2                     ((uint32_t)0x00000004)        /*!< Bit 2 */
#define  ADC_CR1_AWDCH_3                     ((uint32_t)0x00000008)        /*!< Bit 3 */
#define  ADC_CR1_AWDCH_4                     ((uint32_t)0x00000010)        /*!< Bit 4 */

#define  ADC_CR1_EOCIE                       ((uint32_t)0x00000020)        /*!< Interrupt enable for EOC */
#define  ADC_CR1_AWDIE                       ((uint32_t)0x00000040)        /*!< Analog Watchdog interrupt enable */
#define  ADC_CR1_JEOCIE                      ((uint32_t)0x00000080)        /*!< Interrupt enable for injected channels */
#define  ADC_CR1_SCAN                        ((uint32_t)0x00000100)        /*!< Scan mode */
#define  ADC_CR1_AWDSGL                      ((uint32_t)0x00000200)        /*!< Enable the watchdog on a single channel in scan mode */
#define  ADC_CR1_JAUTO                       ((uint32_t)0x00000400)        /*!< Automatic injected group conversion */
#define  ADC_CR1_DISCEN                      ((uint32_t)0x00000800)        /*!< Discontinuous mode on regular channels */
#define  ADC_CR1_JDISCEN                     ((uint32_t)0x00001000)        /*!< Discontinuous mode on injected channels */

#define  ADC_CR1_DISCNUM                     ((uint32_t)0x0000E000)        /*!< DISCNUM[2:0] bits (Discontinuous mode channel count) */
#define  ADC_CR1_DISCNUM_0                   ((uint32_t)0x00002000)        /*!< Bit 0 */
#define  ADC_CR1_DISCNUM_1                   ((uint32_t)0x00004000)        /*!< Bit 1 */
#define  ADC_CR1_DISCNUM_2                   ((uint32_t)0x00008000)        /*!< Bit 2 */

#define  ADC_CR1_DUALMOD                     ((uint32_t)0x000F0000)        /*!< DUALMOD[3:0] bits (Dual mode selection) */
#define  ADC_CR1_DUALMOD_0                   ((uint32_t)0x00010000)        /*!< Bit 0 */
#define  ADC_CR1_DUALMOD_1                   ((uint32_t)0x00020000)        /*!< Bit 1 */
#define  ADC_CR1_DUALMOD_2                   ((uint32_t)0x00040000)        /*!< Bit 2 */
#define  ADC_CR1_DUALMOD_3                   ((uint32_t)0x00080000)        /*!< Bit 3 */

#define  ADC_CR1_JAWDEN                      ((uint32_t)0x00400000)        /*!< Analog watchdog enable on injected channels */
#define  ADC_CR1_AWDEN                       ((uint32_t)0x00800000)        /*!< Analog watchdog enable on regular channels */

  
/*******************  Bit definition for ADC_CR2 register  ********************/
#define  ADC_CR2_ADON                        ((uint32_t)0x00000001)        /*!< A/D Converter ON / OFF */
#define  ADC_CR2_CONT                        ((uint32_t)0x00000002)        /*!< Continuous Conversion */
#define  ADC_CR2_CAL                         ((uint32_t)0x00000004)        /*!< A/D Calibration */
#define  ADC_CR2_RSTCAL                      ((uint32_t)0x00000008)        /*!< Reset Calibration */
#define  ADC_CR2_DMA                         ((uint32_t)0x00000100)        /*!< Direct Memory access mode */
#define  ADC_CR2_ALIGN                       ((uint32_t)0x00000800)        /*!< Data Alignment */

#define  ADC_CR2_JEXTSEL                     ((uint32_t)0x00007000)        /*!< JEXTSEL[2:0] bits (External event select for injected group) */
#define  ADC_CR2_JEXTSEL_0                   ((uint32_t)0x00001000)        /*!< Bit 0 */
#define  ADC_CR2_JEXTSEL_1                   ((uint32_t)0x00002000)        /*!< Bit 1 */
#define  ADC_CR2_JEXTSEL_2                   ((uint32_t)0x00004000)        /*!< Bit 2 */

#define  ADC_CR2_JEXTTRIG                    ((uint32_t)0x00008000)        /*!< External Trigger Conversion mode for injected channels */

#define  ADC_CR2_EXTSEL                      ((uint32_t)0x000E0000)        /*!< EXTSEL[2:0] bits (External Event Select for regular group) */
#define  ADC_CR2_EXTSEL_0                    ((uint32_t)0x00020000)        /*!< Bit 0 */
#define  ADC_CR2_EXTSEL_1                    ((uint32_t)0x00040000)        /*!< Bit 1 */
#define  ADC_CR2_EXTSEL_2                    ((uint32_t)0x00080000)        /*!< Bit 2 */

#define  ADC_CR2_EXTTRIG                     ((uint32_t)0x00100000)        /*!< External Trigger Conversion mode for regular channels */
#define  ADC_CR2_JSWSTART                    ((uint32_t)0x00200000)        /*!< Start Conversion of injected channels */
#define  ADC_CR2_SWSTART                     ((uint32_t)0x00400000)        /*!< Start Conversion of regular channels */
#define  ADC_CR2_TSVREFE                     ((uint32_t)0x00800000)        /*!< Temperature Sensor and VREFINT Enable */

/******************  Bit definition for ADC_SMPR1 register  *******************/
#define  ADC_SMPR1_SMP10                     ((uint32_t)0x00000007)        /*!< SMP10[2:0] bits (Channel 10 Sample time selection) */
#define  ADC_SMPR1_SMP10_0                   ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP10_1                   ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP10_2                   ((uint32_t)0x00000004)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP11                     ((uint32_t)0x00000038)        /*!< SMP11[2:0] bits (Channel 11 Sample time selection) */
#define  ADC_SMPR1_SMP11_0                   ((uint32_t)0x00000008)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP11_1                   ((uint32_t)0x00000010)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP11_2                   ((uint32_t)0x00000020)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP12                     ((uint32_t)0x000001C0)        /*!< SMP12[2:0] bits (Channel 12 Sample time selection) */
#define  ADC_SMPR1_SMP12_0                   ((uint32_t)0x00000040)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP12_1                   ((uint32_t)0x00000080)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP12_2                   ((uint32_t)0x00000100)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP13                     ((uint32_t)0x00000E00)        /*!< SMP13[2:0] bits (Channel 13 Sample time selection) */
#define  ADC_SMPR1_SMP13_0                   ((uint32_t)0x00000200)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP13_1                   ((uint32_t)0x00000400)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP13_2                   ((uint32_t)0x00000800)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP14                     ((uint32_t)0x00007000)        /*!< SMP14[2:0] bits (Channel 14 Sample time selection) */
#define  ADC_SMPR1_SMP14_0                   ((uint32_t)0x00001000)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP14_1                   ((uint32_t)0x00002000)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP14_2                   ((uint32_t)0x00004000)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP15                     ((uint32_t)0x00038000)        /*!< SMP15[2:0] bits (Channel 15 Sample time selection) */
#define  ADC_SMPR1_SMP15_0                   ((uint32_t)0x00008000)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP15_1                   ((uint32_t)0x00010000)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP15_2                   ((uint32_t)0x00020000)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP16                     ((uint32_t)0x001C0000)        /*!< SMP16[2:0] bits (Channel 16 Sample time selection) */
#define  ADC_SMPR1_SMP16_0                   ((uint32_t)0x00040000)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP16_1                   ((uint32_t)0x00080000)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP16_2                   ((uint32_t)0x00100000)        /*!< Bit 2 */

#define  ADC_SMPR1_SMP17                     ((uint32_t)0x00E00000)        /*!< SMP17[2:0] bits (Channel 17 Sample time selection) */
#define  ADC_SMPR1_SMP17_0                   ((uint32_t)0x00200000)        /*!< Bit 0 */
#define  ADC_SMPR1_SMP17_1                   ((uint32_t)0x00400000)        /*!< Bit 1 */
#define  ADC_SMPR1_SMP17_2                   ((uint32_t)0x00800000)        /*!< Bit 2 */

/******************  Bit definition for ADC_SMPR2 register  *******************/
#define  ADC_SMPR2_SMP0                      ((uint32_t)0x00000007)        /*!< SMP0[2:0] bits (Channel 0 Sample time selection) */
#define  ADC_SMPR2_SMP0_0                    ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  ADC_SMPR2_SMP0_1                    ((uint32_t)0x00000002)        /*!< Bit 1 */
#define  ADC_SMPR2_SMP0_2                    ((uint32_t)0x00000004)        /*!< Bit 2 */

#define  ADC_SMPR2_SMP1                      ((uint32_t)0x00000038)        /*!< SMP1[2:0] bits (Channel 1 Sample time selection) */
#define  ADC_SMPR2_SMP1_0                    ((uint32˝C ∞€ ˛C˝ ¢€ ˇC˛ †€ ÄDˇ € ÅDâ ë€ ÇDä è€ ÉDã ç€ ÑDï Ç€ ÖDñ Ä€ ÜDó ~€ áD° s€ àD¢ q€ âD£ o€ äD≠ d€ ãDÆ b€ åDØ `€ çDπ U€ éD∫ S€ èDª Q€ êD≈ F€ ëD∆ D€ íD« B€ ìD— 7€ îD“ 5€ ïD” 3€ ñD› (€ óDﬁ &€ òDﬂ $€ ôDÈ € öDÍ € õDÎ € úDÏ € ùDÌ € ûD˘ € üD˙ € †D˚  € °DÖ t€ ¢DÜ r€ £Dá p€ §Dë e€ •Dí c€ ¶Dì a€ ßDù V€ ®Dû T€ ©Dü R€ ™D© G€ ´D™ E€ ¨D´ C€ ≠D¨ A€ ÆDÆ >€ ØDπ 2€ ∞Dª /€ ±DΩ ,€ ≤Dæ *€ ≥D… € ¥D  € µDÀ € ∂DÃ € ∑DÕ € ∏DŸ 	€ πD⁄ € ∫D€ € ªD‹ € ºD› € ΩDﬁ ˇ€ æDﬂ ˝€ øD‡ ˚€ ¿D· ˘€ ¡D„ ˆ€ ¬DÛ Â€ √DÙ „€ ƒDı ·€ ≈Dˆ ﬂ€ ∆D˜ ›€ «D¯ €€ »D˘ Ÿ€ …D˙ ◊€  D˚ ’€ ÀD¸ ”€ ÃD˝ —€ ÕD˛ œ€ ŒDˇ Õ€ œDÄ J€ –DÅ H€ —DÇ F€ “DÉ D€ ”DÑ B€ ‘DÖ @€ ’DÜ >€ ÷D° "€ ◊D¢  € ÿD£ € ŸD§ € ⁄D• € €D¶ € ‹Dß € ›D® € ﬁD© € ﬂD™ € ‡D´ € ·D¨ € ‚D≠ 
€ „DÆ € ‰DØ € ÂD∞ € ÊD± € ÁD≤  € ËD≥ ˛€ ÈDÕ „€ ÍDŒ ·€ ÎDœ ﬂ€ ÏD– ›€ ÌD— €€ ÓD“ Ÿ€ ÔD” ◊€ D‘ ’€ ÒDﬁ  € ÚDﬂ »€ ÛD‡ ∆€ ÙD· ƒ€ ıD‚ ¬€ ˆDÓ µ€ ˜DÔ ≥€ ¯D ±€ ˘DÒ Ø€ ˙DÚ ≠€ ˚DÛ ´€ ¸DÙ ©€ ˝Dı ß€ ˛Dˆ •€ ˇD˛ ú€ ÄEä ç€ ÅEã ã€ ÇEå â€ ÉEç á€ ÑEó |€ ÖEò z€ ÜEô x€ áE£ m€ àE§ k€ âE• i€ äEØ ^€ ãE∞ \€ åE± Z€ çEª O€ éEº M€ èEΩ K€ êEæ I€ ëE  <€ íEÀ :€ ìEÃ 8€ îEÕ 6€ ïEŒ 4€ ñEœ 2€ óE– 0€ òE— .€ ôE“ ,€ öE‹ !€ õEﬂ € úE‡ € ùEÍ € ûEÎ € üEÏ € †Eˆ € °E˜ ˇ€ ¢E¯ ˝€ £EÉ p€ §EÑ n€ •EÖ l€ ¶Eè a€ ßEê _€ ®Eë ]€ ©Eí [€ ™Eì Y€ ´Eî W€ ¨Eï U€ ≠Eñ S€ ÆEó Q€ ØEß @€ ∞E® >€ ±E© <€ ≤E™ :€ ≥E´ 8€ ¥E∑ +€ µE∏ )€ ∂Eπ '€ ∑E√ € ∏Eƒ € πE≈ € ∫E∆ € ªE« € ºE» € ΩE… € æE  € øEÀ € ¿EÃ 
€ ¡EÕ € ¬EŒ € √Eœ € ƒE› ı€ ≈EÊ Î€ ∆EÔ ·€ «E¯ ◊€ »E˘ ’€ …E˙ ”€  E˚ —€ ÀE¸ œ€ ÃE˝ Õ€ ÕE˛ À€ ŒEˇ …€ œEÄ F€ –EÅ D€ —EÇ B€ “EÉ @€ ”EÑ >€ ‘EÖ <€ ’EÜ :€ ÷Eá 8€ ◊Eà 6€ ÿEâ 4€ ŸEûÄº€ ⁄EûÄª€ €EüÄπ€ ‹EüÄ∏€ ›EüÄ∑€ ﬁE†Äµ€ ﬂE†Ä¥€ ‡E†Ä≥€ ·E°Ä±€ ‚E°Ä∞€ „E°ÄØ€ ‰E¢Ä≠€ ÂE¢Ä¨€ ÊE¢Ä´€ ÁE£Ä©€ ËE£Ä®€ ÈE£Äß€ ÍE§Ä•€ ÎE§Ä§€ ÏE§Ä£€ ÌE•Ä°€ ÓE•Ä†€ ÔE•Äü€ E¶Äù€ ÒE¶Äú€ ÚE¶Äõ€ ÛEßÄô€ ÙEßÄò€ ıE®Äñ€ ˆE®Äï€ ˜E©Äì€ ¯E©Äí€ ˘E™Äê€ ˙E™Äè€ ˚E´Äç€ ¸E´Äå€ ˝E´Äã€ ˛E¨Äâ€ ˇE¨Äà€ ÄF¨Ä€ ÅF≠Ä€ ÇF≠Ä€ ÉF≠Ä€ ÑF≠Ä€ ÖFÆÄˇ€ ÜFÆÄ˛€ áFÆÄ˝€ àFØÄ˚€ âFØÄ˙€ äFØÄ˘€ ãFØÄ¯€ åF∞Äˆ€ çF∞Äı€ éF∞ÄÙ€ èF∞ÄÛ€ êF±ÄÒ€ ëF±Ä€ íF≤ÄÓ€ ìF≤ÄÌ€ îF≤ÄÏ€ ïF≥ÄÍ€ ñF≥ÄÈ€ óF¥ÄÁ€ òF¥ÄÊ€ ôF≥ÄÊ€ öFµÄ„€ õFµÄ‚€ úFµÄ·€ ùF∂Äﬂ€ ûFµÄﬂ€ üF∑Ä‹€ †F∑Ä€€ °F∏ÄŸ€ ¢F∏Äÿ€ £F∑Äÿ€ §FπÄ’€ •FπÄ‘€ ¶FπÄ”€ ßF∫Ä—€ ®FπÄ—€ ©FªÄŒ€ ™FªÄÕ€ ´FªÄÃ€ ¨FªÄÀ€ ≠FºÄ…€ ÆFºÄ»€ ØFºÄ«€ ∞FΩÄ≈€ ±FΩÄƒ€ ≤FΩÄ√€ ≥FæÄ¡€ ¥FæÄ¿€ µFøÄæ€ ∂FøÄΩ€ ∑FæÄΩ€ ∏F¿Ä∫€ πF¿Äπ€ ∫F¿Ä∏€ ªF¡Ä∂€ ºF¡Äµ€ ΩF¡Ä¥€ æF¬Ä≤€ øF¬Ä±€ ¿F¬Ä∞€ ¡F√ÄÆ€ ¬F√Ä≠€ √F√Ä¨€ ƒFƒÄ™€ ≈FƒÄ©€ ∆FƒÄ®€ «F≈Ä¶€ »F≈Ä•€ …F≈Ä§€  F∆Ä¢€ ÀF∆Ä°€ ÃF∆Ä†€ ÕF«Äû€ ŒF«Äù€ œF«Äú€ –F»Äö€ —F»Äô€ “F»Äò€ ”F…Äñ€ ‘F…Äï€ ’F…Äî€ ÷F Äí€ ◊F Äë€ ÿF Äê€ ŸFÀÄé€ ⁄FÀÄç€ €FÀÄå€ ‹FÃÄä€ ›FÃÄâ€ ﬁFÃÄà€ ﬂFÕÄÜ€ ‡FÕÄÖ€ ·FÕÄÑ€ ‚FŒÄÇ€ „FŒÄÅ€ ‰FŒÄÄ€ ÂFœÄ~€ ÊFœÄ}€ ÁFœÄ|€ ËF–Äz€ ÈF–Äy€ ÍF–Äx€ ÎF—Äv€ ÏF—Äu€ ÌF—Ät€ ÓF“Är€ ÔF“Äq€ F“Äp€ ÒF”Än€ ÚF”Äm€ ÛF”Äl€ ÙF‘Äj€ ıF‘Äi€ ˆF‘Äh€ ˜F’Äf€ ¯F’Äe€ ˘F’Äd€ ˙F÷Äb€ ˚F÷Äa€ ¸F÷Ä`€ ˝F◊Ä^€ ˛F◊Ä]€ ˇF◊Ä\€ ÄGÿÄŸ€ ÅGÿÄÿ€ ÇGÿÄ◊€ ÉGŸÄ’€ ÑGŸÄ‘€ ÖGŸÄ”€ ÜG⁄Ä—€ áG⁄Ä–€ àG⁄Äœ€ âG€ÄÕ€ äG€ÄÃ€ ãG€ÄÀ€ åG€Ä € çG‹Ä»€ éG‹Ä«€ èG‹Ä∆€ êG‹Ä≈€ ëG›Ä√€ íG›Ä¬€ ìG›Ä¡€ îG›Ä¿€ ïGﬁÄæ€ ñGﬁÄΩ€ óGﬁÄº€ òGﬂÄ∫€ ôGﬂÄπ€ öGﬂÄ∏€ õG‡Ä∂€ úG‡Äµ€ ùG‡Ä¥€ ûG·Ä≤€ üG·Ä±€ †G·Ä∞€ °G‚ÄÆ€ ¢G‚Ä≠€ £G‚Ä¨€ §G„Ä™€ •G„Ä©€ ¶G„Ä®€ ßG‰Ä¶€ ®G‰Ä•€ ©G‰Ä§€ ™GÂÄ¢€ ´GÂÄ°€ ¨GÂÄ†€ ≠GÊÄû€ ÆGÊÄù€ ØGÊÄú€ ∞GÁÄö€ ±GÁÄô€ ≤GÁÄò€ ≥GËÄñ€ ¥GËÄï€ µGËÄî€ ∂GÈÄí€ ∑GÈÄë€ ∏GÈÄê€ πGÍÄé€ ∫GÍÄç€ ªGÍÄå€ ºGÎÄä€ ΩGÎÄâ€ æGÎÄà€ øGÏÄÜ€ ¿GÏÄÖ€ ¡GÏÄÑ€ ¬GÌÄÇ€ √GÌÄÅ€ ƒGÌÄÄ€ ≈GÓÄ~€ ∆GÓÄ}€ «GÓÄ|€ »GÔÄz€ …GÔÄy€  GÔÄx€ ÀGÄv€ ÃGÄu€ ÕGÒÄs€ ŒGÒÄr€ œGÚÄp€ –GÚÄo€ —GÛÄm€ “GÛÄl€ ”GÙÄj€ ‘GÙÄi€ ’GıÄg€ ÷GıÄf€ ◊GˆÄd€ ÿGˆÄc€ ŸGˆÄb€ ⁄G˜Ä`€ €G˜Ä_€ ‹G˜Ä^€ ›G¯Ä\€ ﬁG¯Ä[€ ﬂG¯ÄZ€ ‡G˘ÄX€ ·G˘ÄW€ ‚G˘ÄV€
 „GÄ Â€ ‰G3 Ñ€ ÂG4 Ç€ ÊG9 |€ ÁG< x€ ËG? t€ ÈGF l€ ÍGI h€ ÎGL ÓÄÔÄ=€ ÏGS \€ ÌGU Y€ ÓGX U€ ÔG[ Q€ G^ M€ ÒGa ÄÒÄ€ ÚGk >€ ÛGq 7€ ÙGu 2€ ıG} )€ ˆG~ '€ ˜GÄ "€ ¯Gä € ˘Gã € ˙Gå € ˚Gç € ¸Gé € ˝Gö € ˛Gõ  € ˇGú ˛€ ÄHù {€ ÅH® o€ ÇH© m€ ÉH™ k€ ÑH≤ b€ ÖH≥ `€ ÜH¥ ^€ áHµ \€ àH∂ Z€ âH¬ M€ äH√ K€ ãHƒ I€ åHŒ >€ çHœ <€ éH– :€ èH⁄ /€ êH€ -€ ëH‹ +€ íHÊ  € ìHÁ € îHË € ïHÚ € ñHÛ € óHÙ € òHı € ôHˆ 	€ öH˜ € õH¯ € úH˘ € ùH˙ € ûH˚ ˇ€ üH¸ ˝€ †H˝ ˚€ °HÅ u€ ¢HÜ o€ £Hê d€ §Hë b€ •Hí `€ ¶Hú U€ ßHù S€ ®Hû Q€ ©H® F€ ™H© D€ ´H™ B€ ¨Hµ 6€ ≠H∂ 4€ ÆH∑ 2€ ØH¡ '€ ∞H¬ %€ ±H√ #€ ≤Hƒ !€ ≥H≈ € ¥H∆ € µH« € ∂H» € ∑H… € ∏H  € πHÀ € ∫H— € ªH“ 
€ ºH’ € ΩH÷ € æH◊ € øHÌÄâ€ ¿HÌÄà€ ¡HÓÄÜ€ ¬HÓÄÖ€ √HÓÄÑ€ ƒHÔÄÇ€ ≈HÔÄÅ€ ∆HÄ€ «HÄ~€ »HÄ}€ …HÒÄ{€  HÒÄz€ ÀHÚÄx€ ÃHÚÄw€ ÕHÚÄv€ ŒHÛÄt€ œHÛÄs€ –HÛÄr€ —HÛÄq€ “HÙÄo€ ”HÙÄn€ ‘HÙÄm€ ’HÙÄl€ ÷HıÄj€ ◊HıÄi€ ÿHıÄh€ ŸHˆÄf€ ⁄HˆÄe€ €HˆÄd€ ‹H˜Äb€ ›H˜Äa€ ﬁH˜Ä`€ ﬂH¯Ä^€ ‡H¯Ä]€ ·H¯Ä\€ ‚H˘ÄZ€ „H˘ÄY€ ‰H˘ÄX€ ÂH˙ÄV€ ÊH˙ÄU€ ÁH˙ÄT€ ËH˚ÄR€ ÈH˚ÄQ€ ÍH¸ÄO€ ÎH¸ÄN€ ÏH˝ÄL€ ÌH˝ÄK€ ÓH˝ÄJ€ ÔH˛ÄH€ H˛ÄG€ ÒH˛ÄF€ ÚHˇÄD€ ÛHˇÄC€ ÙHˇÄB€ ıHÄÄø€ ˆHÄÄæ€ ˜HÄÄΩ€ ¯HÅÄª€ ˘HÅÄ∫€ ˙HÅÄπ€ ˚HÇÄ∑€ ¸HÇÄ∂€ ˝HÇÄµ€ ˛HÉÄ≥€ ˇHÉÄ≤€ ÄIÉÄ0€ ÅIÑÄ.€ ÇIÑÄ-€ ÉIÑÄ,€ ÑIÖÄ*€ ÖIÖÄ)€ ÜIÖÄ(€ áIÜÄ&€ àIÜÄ%€ âIÜÄ$€ äIáÄ"€ ãIáÄ!€ åIáÄ € çIàÄ€ éIàÄ€ èIàÄ€ êIâÄ€ ëIâÄ€ íIâÄ€
 ìIÄ 1€ îI:  € ïI; »€ ñI< ∆€ óI= ƒ€ òI> ¬€ ôIB Ω€ öIC ª€ õIXÄC€ úIYÄA€ ùIYÄ@€ ûIZÄ>€ üIZÄ=€ †I[Ä;€ °I\Ä9€ ¢I\Ä8€ £I]Ä6€ §I]Ä5€ •I^Ä3€ ¶I_Ä1€
 ßIÄ € ®I3 º€ ©I4 ∫€ ™I9 ¥€ ´I= Ø€ ¨IA ™€ ≠ID ≈Ä;€ ÆIs v€ ØIt t€ ∞Iu r€ ±I g€ ≤IÄ c€ ≥IÅ a€ ¥IÇ _€ µIç S€ ∂Iè P€ ∑Ië M€ ∏Iì J€ πIï G€ ∫Iò C€ ªIû <€ ºI† 9€ ΩI¢ 6€ æI¨ +€ øI≠ )€ ¿IÆ '€ ¡IƒÄÆ€ ¬IƒÄ≠€ √I≈Ä´€ ƒI≈Ä™€ ≈I∆Ä®€ ∆I∆Äß€ «I∆Ä¶€ »I«Ä§€ …I«Ä£€  I«Ä¢€ ÀI»Ä†€ ÃI»Äü€ ÕI;ÄE€ ŒIH ô€ œI@ ≥€ –IÚ@ ∞€ —IÙ@ ≠€ “Iˆ@ ™€ ”I¯@ ß€ ‘I˙@ §€ ’I¸@ °€ ÷I:Ä; >Ä¶ÄÓ€ ◊I? ãÄôÄ#êÄ*€ ÿISÄ_Ä•Ä€ ŸIWÄ"€ ⁄I[Ä€ €IwÄ‚Ä#ûÄJ€ ‹Iå #îÄ#ïÄ#ôÄ~€ ›Iç #ñÄ#ôÄΩ€ ﬁIé #öÄ˛€ ﬂIï †ÄØÄÍÄ"UÄ#ﬁÄ„€ ‡IüÄ¨Ä˙Ä#ë	Ä÷€ ·I¥Ä#ãÄÖ€ ‚I√ € „Iƒ € ‰I… #∑Ä#–Ä*€
 ÂI Ä  —€ ÊI 2 #˝Ä◊€ ÁI 3 #ÏÄÊ€ ËI 5 #öÄ4€ ÈI 6 #úÄ0€ ÍI 7 #°Ä)€ ÎI 8 #¢Ä#•Ä÷€ ÏI 9 #§Ä"€ ÌI : #®Ä€ ÓI ; #©Ä€ ÔI < #ÆÄ€ I = #ØÄ€ ÒI > #∂Ä€ ÚI ? #≥Ä€ ÛI @ #¥Ä€F ÙI C  fÄ „Ä ‰Ä ÂÄ ÁÄÜÄáÄïÄñÄóÄòÄôÄöÄC€ ıI E \€ ˆI F #ÁÄ…€ ˜I G X€ ¯I S K€ ˘I U H€ ˙I V #f#€¨€" ˚I W #ı#ó#ö#¥#¥#≈#≈Î€ ¸I X #›#Î#¸#˛#ë#ü>€" ˝I Y #Ü#ê#ì#Ÿ#‹#ˆ#˘∑€ ˛I Z #i#¨#Œ#˝¶€1 ˇI [ #s#œ#˙#ã#ã#Æ#Æ#ñ#ñ#§#§ˇ€ ÄJ \ π€ ÅJ ] ∑€
 ÇJ ^ #îÒ€
 ÉJ _ #óÏ€ ÑJ ` #é#ÃÒ€ ÖJ a Ø€ ÜJ b #“#Ÿ#êÍ€ áJ c ´€ àJ d ©€
 âJ e #õ‹€ äJ f •€ ãJ h  ‡Ä#ôÄ◊€ åJ q ò€ çJ s ï€ éJ t ì€ èJ u ë€ êJ v è€ ëJ w ç€ íJ x ã€ ìJ y â€ îJ z á€ ïJ { Ö€ ñJ | É€ óJ } Å€ òJ ~ € ôJ  }€ öJ Ä y€ õJ Å w€ úJ Ç u€ ùJ É s€ ûJ Ñ q€ üJ Ö o€ †J Ü m€ °J à  ·Ä#õÄú€ ¢J ë `€ £J ì ]€ §J î [€ •J ï Y€ ¶J ñ W€ ßJ ó U€ ®J ò S€ ©J ô Q€ ™J ö O€ ´J ú  „Ä#†Äw€ ¨J • B€ ≠J ß ?€ ÆJ ® =€ ØJ © ;€ ∞J ™ 9€ ±J ´ 7€ ≤J ¨ 5€ ≥J ≠ 3€ ¥J Ø  ‰Ä#£ÄW€ µJ ∏ &€ ∂J ∫ #ä#å#ö#ú€ ∑J ª !€ ∏J º #Œ!€ πJ Ω € ∫J æ #ù#‘D€ ªJ ø #π#Ä	{€ ºJ ¿ #™#™#ˇ-€ ΩJ ¬  ‚Ä#ßÄ9€ æJ À 
€+ øJ Õ #º#˚#Å#≈#⁄#Ó#Ò#Ö	#â	x€ ¿J Œ € ¡J œ #ª#È˛€ ¬J – #º#•>€ √J — #Û#Ïª€ ƒJ “ #˜‘€ ≈J ” ˚€ ∆J ’  ÂÄ#≠Ä€ «J ﬁ Ó€@ »J ‡ #f#i#s#Ü#ê#ì#ô#“#Ÿ#›#Î#ı#¸#˛#ê	€C »J#ó#ö#¥#¥#≈#≈#Ÿ#‹#ˆ#˘#ë#ü#¨#Œ#œ#€?€? »J#î#ó#õ#˙#˝#ã#ã#Æ#Æ#é#ñ#ñ#§#§#ÃX€ …J · #õ#€;  J ‚ #ß#ä#å#Œ#ö#ú#π#™#™#ù#‘#ˇ#Ä	Œ€ ÀJ „ #†€ ÃJ ‰ #£€C ÕJ Â #≠#ª#º#È#•#º#Û#˜#˚#Å#≈#⁄#Ï#Ó#ÒÂ€ ÕJ#Ö	#â	õ€ ŒJ Ê #©#Â#Í#©# #Õ€ œJ Á #¥#Ã#≥-€ –J Ë #∂#˝—€ —J Í ´Ä§€ “J Í ◊€ ”J Î ’€ ‘J Ì “€ ’J Ô œ€ ÷J  Õ€ ◊J Ú  € ÿJ Ù «€ ŸJ ˆ ƒ€ ⁄J ¯ ¡€ €J ˙ æ€ ‹J ¸ ª€ ›J ˛ ∏€ ﬁJ Ä 4€ ﬂJ Ç 1€ ‡J Ñ .€ ·J Ü +€ ‚J à (€ „J ã $€ ‰J å "€ ÂJ é € ÊJ ê € ÁJ í € ËJ ì € ÈJ ï € ÍJ ó € ÎJ ô € ÏJ õ € ÌJ ù € ÓJ ü € ÔJ ° € J £ ˇ€ ÒJ • ¸€ ÚJ ® ¯€ ÛJ © ˆ€ ÙJ ™ Ù€' ıJ ¨ #Ã#ı#ó#ö#≤#¥#√#≈∑€ ˆJ Æ Ó€ ˜J Ø #≥#ƒ€ ¯J ± #Û#ò€ ˘J ≥ #Ù#ô€ ˙J µ „€ ˚J ∑ ‡€ ¸J π ›€ ˝J ª ⁄€ ˛J æ #Û#Ù#ò#ô#≥#ƒA€ ˇJ ø #»Ä#∞Ä#¬Äó€ ÄK ¿ Q€' ÅK ¬ # #‹#›#Î#¸#˛#ë#ü]€ ÇK ƒ K€ ÉK ≈ I€ ÑK « F€ ÖK … #ﬂ#˝€ ÜK À @€ áK Õ =€ àK œ :€ âK — 7€ äK ” 4€ ãK ’ 1€ åK ◊ .€ çK Ÿ #È€ éK € (€ èK › #Í€ êK ﬂ "€ ëK · € íK „ #Á
€ ìK Â € îK Á € ïK È € ñK Î € óK Ì #ì#òà€ òK Ô #î#úÄ€ ôK Ò €+ öK Ù #ﬂ#Á#È#Í#˝#ì#î#ò#úá€ õK ı #∆Ä#èÄT€ úK ˆ ˇ€/ ùK ¯ #Å#Ü#ê#ì#÷#Ÿ#‹#Ù#ˆ#˘∑€ ûK ˙ ˘€ üK ˚ #è#ô|€ †K ˝ Ù€ °K ˇ Ò€ ¢K Å m€ £K É #ı#ˇ€ §K Ö #◊#‚W€ •K á #ÿ`€ ¶K â a€ ßK ã ^€ ®K ç #å¶€' ©K ê #å#è#ô#◊#ÿ#‚#ı#ˇ»€ ™K ë #~Ä#”Ä#ÒÄ€ ´K í S€- ¨K î #c#s#¯#ã#ã#„#Æ#◊#ñ#§€ ≠K ñ M€ ÆK ó K€ ØK ô H€ ∞K õ E€ ±K ù #âä€ ≤K ü #Ñå€ ≥K ° #ïv€ ¥K £ 9€ µK • 6€ ∂K ß 3€ ∑K © 0€
 ∏K ´ #lô€ πK ≠ *€ ∫K Ø '€ ªK ± $€ ºK ≥ !€ ΩK µ € æK ∑ € øK π € ¿K ª € ¡K Ω € ¬K ø #äV€ √K ¡ € ƒK √ #ÜT€ ≈K ≈ #á#≠s€ ∆K « #£/€ «K …  € »K À ˝€ …K Õ #àC€  K œ ˜€
 ÀK — #ka€1 ÃK ‘ #k#l#Ñ#Ü#á#à#â#ä#≠#ï#£€ ÕK ’ #aÄ#˜Ä#ﬁÄ#ŒÄ5€ ŒK ÷ Ï€ œK ÿ È€ –K ⁄ Ê€ —K € ‰€ “K › ·€ ”K ﬂ ﬁ€ ‘K · €€ ’K „ ÿ€ ÷K Â ’€ ◊K Á “€ ÿK È œ€ ŸK Î Ã€ ⁄K Ì …€ €K Ô ∆€ ‹K Ò √€ ›K Û ¿€ ﬁK ı Ω€ ﬂK ˜ ∫€ ‡K ˘ ∑€ ·K ˚ ¥€ ‚K ˝ ±€ „K ˇ Æ€ ‰K Å *€ ÂK É '€ ÊK Ö $€ ÁK á !€ ËK â € ÈK ã € ÍK ç € ÎK è € ÏK ë € ÌK ì € ÓK ñ € ÔK ó 	€ K ò € ÒK ö € ÚK ú € ÛK ù ˇ€ ÙK ü ¸€ ıK ° ˘€ ˆK £ ˆ€ ˜K • Û€ ¯K ß € ˘K ™ Ï€ ˙K ´ Í€ ˚K ¨ Ë€ ¸K Æ Â€ ˝K ∞ ‚€ ˛K ± ‡€ ˇK ≥ ›€ ÄL µ Y€ ÅL ∑ V€ ÇL π S€ ÉL º O€ ÑL Ω M€ ÖL æ K€ ÜL ¿ #Ò#Ú#ó#õ€ áL ¬ E€ àL √ #ï#ôπ€ âL ≈ #ñ#ö¥€ äL » #ï#ñ#ô#ö&€ ãL … #ÁÄ#ËÄ€ åL   8€ çL Ã #é#√#ÃÖ€ éL Œ 2€ èL œ #ín€ êL — #Õ0€ ëL ‘ )€ íL ’ '€ ìL ◊ $€ îL Ÿ !€ ïL € € ñL ‹ € óL ﬂ #í#ÕY€ òL ‡ #ÃÄ#øÄ*€ ôL · € öL „ € õL Â € úL Ê € ùL Ë 	€ ûL Î € üL Ï € †L Ó  € °L  #À#“#Ÿ#Ç#êû€ ¢L Ú ˙€ £L Û ¯€ §L ı #”#ÉH€ •L ˜ Ú€ ¶L ˘ #Ö>€ ßL ˚ #Ü:€ ®L ˝ #◊#ç.€ ©L ˇ #à2€ ™L Å b€# ´L Ñ #”#◊#É#Ö#Ü#à#çﬂ€ ¨L Ö #«ÄÍ€ ≠L á Y€ ÆL â V€ ØL ã S€ ∞L å Q€ ±L é N€ ≤L ê K€ ≥L í H€ ¥L î E€ µL ñ B€ ∂L ô >€ ∑L ö <€ ∏L õ :€ πL ù 7€ ∫L ü 4€ ªL † 2€ ºL ¢ /€ ΩL § ,€ æL ¶ )€ øL ® &€ ¿L ™ #€ ¡L ¨  € ¬L Æ € √L ∞ € ƒL ≤ € ≈L ¥ € ∆L ∑ € «L ∏ € »L π € …L ª 	€  L Ω € ÀL æ € ÃL ¿ € ÕL ¬ ˛€ ŒL ƒ ˚€ œL ∆ ¯€ –L » ı€ —L À Ò€ “L Ã Ô€ ”L Õ  ÈÄY€ ‘L œ Í€ ’L — Á€ ÷L “ Â€ ◊L ‘ ‚€ ÿL ÷ ﬂ€ ŸL ÿ ‹€ ⁄L ⁄ Ÿ€ €L ‹ ÷€ ‹L ﬁ ”€ ›L ‡ –€ ﬁL ‚ Õ€ ﬂL ‰  € ‡L Á ∆€ ·L Ë ƒ€ ‚L È ¬€ „L Í  ¸Ä€ ‰L Ï Ω€ ÂL Ó ∫€ ÊL Ô ∏€ ÁL Ò µ€ ËL Û ≤€ ÈL ı Ø€ ÍL ˜ ¨€ ÎL ˙ ®€ ÏL ˚ ¶€ ÌL ¸ §€ ÓL ˛ °€ ÔL Ä € L Ç € ÒL É € ÚL Ö € ÛL à € ÙL â € ıL ã € ˆL ç 	€ ˜L è € ¯L ê € ˘L í € ˙L î ˛€ ˚L ñ ˚€ ¸L ò ¯€ ˝L ö ı€ ˛L ú Ú€ ˇL û Ô€ ÄM † k€ ÅM £ g€ ÇM § e€ ÉM • c€ ÑM ß `€ ÖM © ]€ ÜM ™ [€ áM ¨ X€ àM Æ U€ âM ∞ R€ äM ≤ O€ ãM ¥ L€ åM ∂ I€ çM ∏ F€ éM ∫ C€ èM º @€ êM æ =€ ëM ¿ :€ íM ¬ 7€ ìM ƒ 4€ îM ∆ 1€ ïM … -€ ñM   +€ óM À )€ òM Õ &€ ôM œ #€ öM – !€ õM “ € úM ‘ € ùM ÷ € ûM Ÿ € üM ⁄ € †M € € °M › € ¢M ﬂ 
€ £M ‡ € §M ‚ € •M ‰ € ¶M Ê ˇ€ ßM Ë ¸€ ®M Í ˘€ ©M Ï ˆ€ ™M Ì Ù€ ´M Ó Ú€ ¨M  #ˆ#ä#å#ö#ú»€ ≠M Ú Ï€ ÆM Û #õ € ØM ı #ã.€ ∞M ¯ #ã#õ`€ ±M ˘ #ÈÄ#òÄ€ ≤M ˙ ﬂ€ ≥M ¸ ‹€ ¥M ˛ Ÿ€ µM ˇ ◊€ ∂M Å S€ ∑M Ñ O€ ∏M Ö M€ πM á J€ ∫M â G€ ªM ã D€ ºM å B€ ΩM é ?€ æM ë ;€ øM í 9€ ¿M ì 7€ ¡M ï #ù#ƒ#‘l€ ¬M ó 1€ √M ò #†_€ ƒM ö #’'€ ≈M ù (€ ∆M û &€ «M † #€ »M ¢  € …M § €  M ¶ € ÀM ® #†#’B€ ÃM © #ÕÄ#¿Ä'€ ÕM ™ €+ ŒM ¨ #€#·#Â#Í#©#Æ#»# #Õ1€ œM Æ € –M Ø € —M ± #´.€ “M ≥ #™,€ ”M µ #¨'€ ‘M ∑ ˇ€ ’M π #≠ € ÷M ª ˘€ ◊M Ω ˆ€ ÿM ø #…#Ã € ŸM ¡ € ⁄M √ Ì€ €M ≈ #‚#‰#Á#Èû€ ‹M « Á€ ›M … ‰€ ﬁM À ·€/ ﬂM Œ #‚#‰#Á#È#™#´#¨#≠#…#Ã4€ ‡M œ #ŸÄ#ßÄ#∆Äâ€ ·M – Ÿ€ ‚M “ ÷€ „M ‘ ”€ ‰M ’ —€ ÂM ◊ Œ€ ÊM ⁄  € ÁM € »€ ËM ‹ ∆€ ÈM ﬁ √€ ÍM ‡ ¿€ ÎM · æ€ ÏM „ ª€ ÌM Ê ∑€ ÓM Á µ€ ÔM Ë ≥€7 M Í #Â#±#º#÷#˚#Å#≈#⁄#Ó#Ò#Ö	#â	=€ ÒM Ï ≠€ ÚM Ì #∑ƒ€ ÛM Ô #≥≈€ ÙM Ò #¥#à	€ ıM Û ¢€ ˆM ı #µ∫€ ˜M ˜ #∂# # ¬€ ¯M ˘ ô€ ˘M ˚ #≤¥€ ˙M ˝ #∫#¸}€ ˚M ˇ #Ä#«##á	€ ¸M Å #ˇ#∆#œ#◊#Ô#Ü	€C ˝M Ñ #≤#≥#¥#µ#∂#∑#∫#¸#ˇ#Ä#∆#«# # #œ‘€ ˝M#◊#Ô##Ü	#á	#à	d€  ˛M Ö #ﬂÄ# Ä#¡Ä#ÊÄ#¸ÄH€ ˇM Ü € ÄN à Ä€ ÅN ä }€ ÇN ã {€ ÉN ç x€ ÑN è u€ ÖN ë r€ ÜN ì o€ áN ï l€ àN ò h€ âN ô f€ äN ö d€ ãN ú #Ë#È3€ åN û ^€ çN ü \€ éN ° Y€ èN £ V€ êN • S€ ëN ß P€ íN © M€ ìN ´ J€ îN ≠ G€ ïN Ø D€ ñN ± A€ óN ≥ >€ òN µ ;€ ôN ∏ 7€ öN π #‡Ä¶€ õN ∫ 3€ úN º #’#Û#È#Ï”€ ùN æ -€ ûN ø #€ üN ¡ #Ò#ÎÎ€ †N √ #Ú€ °N ≈ #Í€ ¢N » ##Ò#Ú#Í#ÎÑ€ £N … #ÀÄ#ÁÄ	€ §N   € •N Ã #‰#Ï#•€ ¶N Œ € ßN œ #˜#ä#òj€ ®N — € ©N ” #Ì€ ™N ’ #¯#å&€ ´N ◊ #˚#ã!€ ¨N Ÿ #Ö#öÑ€ ≠N € #ÇN€ ÆN › #˘#ç#ûJ€ ØN ﬂ #˛#ì#ü;€ ∞N · #è#ôo€ ±N „ #˙#é€ ≤N Â Ò€C ≥N Ë ##˜#¯#˘#˙#˚#˛#Ç#Ö#ä#ã#å#ç#é#è€ ≥N#ì#ò#ô#ö#û#üf€ ¥N È #›Ä_€ µN Î Ë€ ∂N Ì Â€ ∑N Ô ‚€ ∏N  ‡€ πN Ú ›€ ∫N Ù ⁄€ ªN ˆ ◊€ ºN ¯ ‘€ ΩN ˚ –€ æN ¸ Œ€
 øN!Ä! € ¿N!9 ó€ ¡N!: ï€ ¬N!; ì€ √N!= ê€ ƒN!> #ÊÄ˛€ ≈N!@ #–Ä#ÄÄ‚€ ∆N!A #ÒÄÓ€	 «N!K !T Ê€	 »N!L !U „€ …N!M z€  N!N x€ ÀN!O v€ ÃN!P t€ ÕN!Q r€ ŒN!R p€ œN!S n€ –N!V j€ —N!W h€ “N!X f€ ”N!Y d€ ‘N!b Z€ ’N!c X€ ÷N!d V€ ◊N!e T€ ÿN!f R€ ŸN!g P€ ⁄N!h N€ €N!i L€ ‹N!j J€ ›N!k H€ ﬁN!l F€ ﬂN!m D€ ‡N!n #öÄ˚€ ·N!o #ñÄ#∑Ä#¬Ä*€ ‚N!p >€ „N!y 4€ ‰N!z 2€ ÂN!{ 0€ ÊN!| .€ ÁN!~ +€ ËN! )€ ÈN!Ä %€ ÍN!Å #€ ÎN!É  € ÏN!Ñ € ÌN!Ö € ÓN!Ü € ÔN!à € N!â € ÒN!ä € ÚN!å #ıÄ#ãÄ0€ ÛN!ç #óÄ#úÄz€ ÙN!é #ˆÄ#åÄ*€ ıN!è #âÄ#õÄÖ€ ˆN!ê € ˜N!í € ¯N!ì € ˘N!î € ˙N!ï ˇ€ ˚N!ñ ˝€ ¸N!ü Û€ ˝N!† Ò€ ˛N!° Ô€ ˇN!¢ Ì€ ÄO!§ i€ ÅO!• g€ ÇO!¶ #µÄ € ÉO!® #ÇÄ2€ ÑO!© #ÜÄ,€ ÖO!™ ^€ ÜO!¨ # Ä‡€ áO!≠ Y€ àO!Æ # Ä‹€ âO!Ø #∏ÄÌ€ äO!± R€ ãO!æ #˚Ä#˛Ä˜€ åO!ø #ÍÄ#ÌÄ#ëÄÿ€ çO!¿ ”Ä#ﬁÄ\€R éO!ﬂ #ÜÄ#ìÄ#–Ä#“Ä#›Ä#¸Ä#óÄ#¥Ä#≈Ä#‹Ä#˘Ä#ëÄ#¨Ä#ŒÄ#œÄæ€S éO#€Ä#äÄ#ŒÄ#ãÄ#öÄ#©Ä#πÄ#™Ä#ÆÄ#˚Ä#éÄ#ñÄ#ùÄ#§Ä#≈Ä#ÃÄG€ éO#‘Ä#ÓÄ#Ö	ÄÍ€O èO!‡ #fÄ#iÄ#sÄ#êÄ#∑Ä#ŸÄ#ÎÄ#ıÄ#˛Ä#êÄ#öÄ#¥Ä#≈Ä#ŸÄ#ˆÄl€S èO#üÄ#˝Ä#åÄ#îÄ#óÄ#õÄ#ªÄ#ºÄ#ÂÄ#ÍÄ#˙Ä#˝Ä#ãÄ#úÄ# Ä#ÕÄy€S èO#ÈÄ#•Ä#™Ä#ÆÄ#ºÄ#ÛÄ#˜Ä#ÅÄ#ñÄ#§Ä#⁄Ä#ÏÄ#ÒÄ#ˇÄ#Ä	Ä#â	Ä4€* êO!· #¥Ä#≈Ä#ãÄ#™Ä#ÆÄ#ñÄ#§ÄΩ€ ëO!Á € íO!Ë € ìO!È € îO!Í € ïO!Î € ñO4 Â€ óO5 „€ òO6 ·€ ôO7 ﬂ€ öO8 ›€ õO9 €€ úO: Ÿ€ ùO< ÷€ ûO= ‘€ üO> “€ †O? –€ °OA Õ€ ¢OB À€ £OC …€ §OD «€ •OH ìÄ¶€P ¶OP #}Ä#âÄ#≠Ä#·Ä#≈Ä#ØÄ#¡Ä#“Ä#Ä#éÄ#ÊÄ#ˆÄ#€Ä#…Ä#æÄI€ ¶O#ÂÄó€ ßOQ ∑€Q ®OR —Ä“Ä‘Ä’ÄŸÄ›ÄﬁÄÂÄÊÄÎÄÏÄÌÄÓÄÔÄÄr€Q ®O#{Ä#}Ä#®Ä#≠Ä#›Ä#·Ä#√Ä#≈Ä#≠Ä#ØÄ#øÄ#¡Ä#–Ä#“Ä#ÓÄ#Äé€I ®O#åÄ#éÄ#‰Ä#ÊÄ#ÙÄ#ˆÄ#ŸÄ#€Ä#«Ä#…Ä#ºÄ#æÄ#„Ä#ÂÄE€ ©OU ±€ ™OV Ø€ ´OW ≠€ ¨OX ´€ ≠OY ©€ ÆOZ ß€ ØO[ •€ ∞O\ £€ ±O] °€ ≤O^ óÄ€ ≥Oa õ€ ¥Ob ô€ µOc ó€ ∂Od ï€ ∑Oe ì€ ∏Of òÄp€ πOi ç€ ∫Oj ã€ ªOk â€ ºOl á€ ΩOm Ö€ æOn É€ øOo Å€ ¿Op € ¡Oq }€ ¬Or {€ √Ou w€
 ƒOw #≤ì€
 ≈Ox #≥ê€ ∆Oy #¸#ë#ú#¥#Í#Ö
€
 «Oz #µä€
 »O{ #É∫€ …O| #Û#∂#∏#à≈€  O} #∑#›#›#„#Ì<€
 ÀO~ #ÚE€ ÃO #˜#Æ`€+ ÕOÄ #⁄#‹#‚#Ï##Ü#è#û#Ø›€ ŒOÅ ^€ œOÇ \€ –OÉ Z€ —OÑ X€ “OÜ ôÄ3€ ”OÜ T€ ‘Oà Q€ ’Oä N€ ÷Oã L€ ◊Oå J€ ÿOç H€ ŸOé F€ ⁄Oè D€ €Oê B€ ‹Oë @€ ›Oì =€ ﬁOî ;€ ﬂOï 9€ ‡Oñ 7€ ·Oò 4€ ‚Oô 2€ „Oö 0€ ‰Oû ÜÄáÄ€ ÂOû *€ ÊO¢ %€ ÁO§ #Ó#ˇ#´#±#π#É	ó€ ËO• #Ô#Ä#ûî€ ÈO¶ #Ê€' ÍOß #Æ#„##î#Ç#Ó#ı#Éº€ ÎO® #Á	€ ÏO© ##ÅR€ ÌO™ #Ò#–#ÄT€ ÓO´ #’#ã_€ ÔO¨ € O≠ #Ì#˛#ë€ ÒO∞ ™ÄŸ€ ÚO∞ € ÛO¥ € ÙO∂ € ıO∑ € ˆO∏ ˇ€ ˜Oπ ˝€ ¯O∫ ˚€ ˘Oª ◊Äö€ ˙OΩ ˆ€ ˚Oø Û€ ¸O¿ Ò€ ˝O¡ Ô€ ˛O¬ Ì€ ˇO√ Î€ ÄPƒ h€ ÅP≈ f€ ÇP∆ ãÄP€ ÉP∆ c€ ÑP“ V€ ÖP‘ S€ ÜP’ Q€ áP◊ N€ àPÿ L€ âPŸ J€ äP€ G€ ãP‹ E€ åP› C€ çPﬁ A€ éPﬂ ?€ èP· <€ êPÊÄ‘€ ëPÍ âÄ€ íPÓ ,€ ìP )€ îPÒ '€ ïPÚ %€ ñPÛ #€ óPÙ !€ òPˆ € ôP˜ € öP˙ äÄ€ õP¸ € úP˛ € ùPˇ € ûPÄ ç€ üPÅ ã€ †PÇ â€ °PÉ á€ ¢PÑ Ö€ £PÖ É€ §PÜ Å€ •Pá € ¶Pà }€ ßPâ {€ ®Pä y€ ©Pã w€ ™På u€ ´Pé r€ ¨Pé q€ ≠Pë m€ ÆPì j€ ØPî h€ ∞Pï f€ ±Pñ d€ ≤Pó b€ ≥Pò `€C ¥Pô #Û#¸#É#ë#ú#≤#≥#¥#µ#∂#∑#∏#⁄#‹#›z€C ¥P#›#„#‚#Í#Ì#Ï##Ú#˜#Ö#Ü#à#è#û#Æ#Ø € µPö \€ ∂Pù ∞Ä€ ∑Pù W€ ∏P† S€ πP¢ P€ ∫P£ N€ ªP§ L€ ºP¶ I€ ΩP¶ H€ æP® E€C øP™ #Æ#„#Ê#Á#Ì#Ó#Ô##Ò#˛#ˇ#Ä#Å#û#´{€; øP#±#–#’##ã#î#Ä#ë#π#Ç#Ó#ı#É#É	€@ ¿P´ #f#i#s#Ü#ê#ì#¥#Ã#ô#õ#†#£#ß#©#≠Ú€C ¿P#≥#∂#“#Ÿ#›#Î#ı#¸#˛#ê#ó#ö#¥#¥#≈#≈Î€C ¿P#Ÿ#‹#ˆ#˘#ë#ü#¨#Œ#œ#€#˝#ä#å#î#ó#õÄ€C ¿P#ª#º#Œ#Â#Í#˙#˝#ã#ã#ö#ú#©#π# #Õ#È €C ¿P#•#™#™#Æ#Æ#º#Û#˜#˚#Å#é#ñ#ñ#ù#§#§€/ ¿P#≈#Ã#‘#⁄#Ï#Ó#Ò#ˇ#Ä	#Ö	#â	h€C ¡P∞ #Û#¸#É#ë#ú#≤#≥#¥#µ#∂#∑#∏#⁄#‹#›V€C ¡P#›#„#‚#Í#Ì#Ï##Ú#˜#Ö#Ü#à#è#û#Æ#ØΩ€) ¬P≤Ä#eÄ#nÄ#ûÄ#ÎÄ#ÛÄ#≠Ä#æÄi€R √P∂ —Ä“Ä‘Ä’Ä÷ÄŸÄ›ÄﬁÄ‡Ä·Ä‚Ä„Ä‰ÄÂÄÊÄ@€P √PÎÄÏÄÌÄÓÄÔÄÄÒÄÚÄÛÄÙÄıÄˆÄ˜Ä"QÄ"TÄ"VÄÌ€P √P"WÄ#_Ä#{Ä#®Ä#≈Ä#›Ä#√Ä#≠Ä#øÄ#–Ä#ÓÄ#åÄ#™Ä#µÄ#¿Ä#ÀÄÄ€I √P#ŸÄ#‰Ä#ÃÄ#◊Ä#ÙÄ#ñÄ#•Ä#∑Ä#ƒÄ#ŸÄ#«Ä#ºÄ#„Ä#˙Ä:€ ƒP∂ 1€ ≈P—Ä≥€ ∆P—Ä#√ ¶€ «P“Ä∞€ »P”ÄÆ€ …P“Ä#› â€  P‘Ä´€ ÀP‘Ä#≠ ≥€ ÃP’Ä®€ ÕP’Ä#ø û€ ŒP÷Ä•€ œP◊Ä£€ –PÿÄ°€ —P÷Ä#≈ ï€ “PŸÄû€ ”P⁄Äú€ ‘P€Äö€ ’P‹Äò€ ÷PŸÄ#® #≠ÄL€ ◊P›Äï€ ÿP›Äî€ ŸP›Ä#– #¥Ä#é	Ä◊€ ⁄PﬁÄë€ €PﬁÄ#Ó #µÄ#ç	Ä∂€ ‹P‡Äç€ ›P‡Ä#À v€ ﬁP·Ää€ ﬂP·Ä#Ÿ e€ ‡P‚Äá€ ·P‚Ä#¿ {€ ‚P„ÄÑ€ „P„Ä#µ É€ ‰P‰ÄÅ€ ÂP‰Ä#™ #∑Ä#¬Ä∏€ ÊPÂÄ~€ ÁPÂÄ}€ ËPÊÄ{€ ÈPÁÄy€ ÍPÊÄ#å ¢€ ÎPÎÄs€ ÏPÎÄ#‰ C€ ÌPÏÄp€ ÓPÏÄ#¬Ä#Ù ø€ ÔPÌÄm€ PÌÄl€ ÒPÌÄ#Ÿ E€ ÚPÓÄi€ ÛPÓÄh€ ÙPÓÄ#º \€ ıPÔÄe€ ˆPÔÄd€ ˜PÔÄ#« N€ ¯PÄa€ ˘PÄ`€ ˙PÄ#„ -€ ˚PÒÄ]€ ¸PÒÄ#∑ X€ ˝PÚÄZ€ ˛PÚÄ#àÄ#ƒ € ˇPÛÄW€ ÄQÛÄ#·Ä#• #»Ä›€ ÅQÙÄ”€ ÇQÙÄ“€ ÉQÙÄ#øÄ#◊ A€ ÑQıÄœ€ ÖQıÄŒ€ ÜQıÄ#ÇÄ#ÜÄ#ñ Ü€ áQˆÄÀ€ àQˆÄ#Ã #¸Ä€ âQ˜Ä»€ äQ˜Ä#˙ ~€
 ãQ"Ä" € åQ"OÄO€ çQ"QÄL€ éQ"QÄK€ èQ"RÄI€ êQ"RÄ#ÖÄ#õÄ#‰Ä#ÅÄ€ ëQ"SÄF€  íQ"SÄ#ëÄ#†Ä#ÌÄ#ÀÄ#ŒÄ)€ ìQ"TÄC€ îQ"TÄB€ ïQ"VÄ?€ ñQ"VÄ#˙Äó€ óQ"WÄ<€ òQ"WÄ;€ ôQ"WÄ#ﬁÄÆ€ öQ#_ #ÄÄc€ õQ#_ #f#i#s⁄€ úQ#a #c#k#l#sB€ ùQ#{ #ÓÄ#ÄÄÆ€ ûQ#{ #Ü#ê#ìN€
 üQ#} #ú™€" †Q#~ #Å#Ü#å#è#ê#ì#ôl€ °Q# #á#é#îB€ ¢Q#® #Æ#¥ã€ £Q#© #µ#∑}€ §Q#™ #¥^€ •Q#´ #≥]€ ¶Q#≠ #∫S€ ßQ#∞ #≥#µw€ ®Q#∞ #µ#µ#µî€ ©Q#± #¥#∑q€ ™Q#≈ #Ã%€ ´Q#∆ #Œ#–#”*€ ¨Q#« # #€ ≠Q#… #Œ#Œ#Œ+€ ÆQ#  #Œ€ ØQ#Ã #–€C ∞Q#› #„#Ê#Á#Ì#Ó#Ô##Ò#˛#ˇ#Ä#Å#ô#õ#ûe€' ∞Q#†#£#ß#©#´#≠#±#≥#∂¢€ ±Q#ﬁ #Í#˚√€C ≤Q#‡ #û#û#û#¢#†#•#£#´#´#´#Ø#≠#±#±#±§€ ≤Q#¥#≥d€3 ≥Q#‡ #Ï#˝#ô#õ#°#§#ß#©#≠#≥#∂`€ ¥Q#· #∏€ µQ#ÛÄ€ ∂Q#˜Äz€ ∑Q#ÉÄÏ€ ∏Q#ÜÄË€ πQ#èÄﬁ€ ∫Q#ìÄŸ€C ªQ#√ #–#“#’#Ÿ#›#Î#Ó##ı#¸#˛#Ä#ã#ê#î|€ ªQ#ó#öè€ ºQ#≈ #•7€3 ΩQ#∆ # #‹#›#ﬂ#Á#È#Í#Î#¸#˝#˛0€7 æQ#« #À#“#”#◊#Ÿ#Ç#É#Ö#Ü#à#ç#ê%€' øQ#» #Ã#Û#Ù#ı#ó#ò#ô#öù€ ¿Q#‡ÄÜ€ ¡Q#„ÄÇ€ ¬Q#áÄ‹€ √Q#≠ #¥#¥Y€ ƒQ#Ø #µ4€ ≈Q#∞ #≤#≥#¥v€ ∆Q#ø #≈#≈"€ «Q#¡ #∆€ »Q#¬ #√#ƒ#≈.€ …Q#– #Ÿ#‹„€  Q#– #ÿÍ€ ÀQ#“ #Â⁄€ ÃQ#” #÷#◊#ÿ#Ÿ#‹#‚∆€ ÕQ#’ #›€€ ŒQ#Ó #ˆ#˘Ü€ œQ# #Ç€ –Q#Ò #Ù#ı#ˆ#˘#ˇ€ —Q#Ú #˙ù€ “Q#å #ë#ü†€ ”Q#å #ñ#öü€ ‘Q#é #°W€ ’Q#è #ë#ì#î#ò#ú#üä€ ÷Q#™ #¨.€ ◊Q#µ #∑€ ÿQ#¿ #¬ € ŸQ#À #Œ#œÌ€ ⁄Q#Õ #Œ#œ#–Î€ €Q#Ÿ #€À€C ‹Q#‰ #˙#˝#Ä#Ç#Ü#à#ä#å#ë#î#ó#õ#¥#µ#πè€ ‹Q#ª#º#ø#¬I€ ›Q#Ê #√”€ ﬁQ#Á #Ò#ï#ñ#óX€ ﬂQ#Ë #Ú#ô#ö#õI€ ‡Q#È #ˆ#ä#ã#åp€ ·Q#Ô #π#π#π#ª#º0€ ‚Q#Ã #Œ‹€ „Q#◊ #ﬁ#·#Â#Íâ€ ‰Q#◊ #ﬁ#‚#Áú€' ÂQ#Ÿ #€#·#‚#‰#Â#Á#È#Í:€ ÊQ#Ù #˙#˝#Ä#Ç#ã#ãä€ ÁQ#ˆ #åÓ€+ ËQ#˜ #¯#Ñ#Ü#á#à#â#ä#ã#ã»€ ÈQ#ñ #ö#ús€ ÍQ#ñ #õ;€ ÎQ#ò #ö#õ#ú§€ ÏQ#• #©€ ÌQ#ß #©#™#´#¨#≠#Æ“€ ÓQ#∑ #π¯€ ÔQ#ƒ #»# #ÕË€ Q#∆ #»#…# #Ã#ÕÒ€C ÒQ#Ÿ #È#Ó#Û#¸#É#ë#ú#•#™#™#Æ#Æ#≤#≥#¥€ ÒQ#µ#∂#∑#∏#º_€C ÚQ#Ÿ #È#Û#¸#É#ë#ú#•#©#≤#≥#¥#µ#∂#∑#∏2€ ÚQ#ºÙ€ ÛQ#€ #Ω € ÙQ#‹ #©#™€C ıQ#› #‰#Ï##˜#¯#˘#˙#˚#˛#Ç#Ö#ä#ã#å#ç€' ıQ#é#è#ì#ò#ô#ö#û#ü#•–€ ˆQ#ﬁ #„#≠#Æ‰€/ ˜Q#ﬂ #Â#±#≤#≥#¥#µ#∂#∑#∫#ºâ€ ¯Q#‡ #Ë#È~€C ˘Q#« #⁄#›#‹#›#„#‚#Í#Ì#Ï##Ú#Û#ı#˜#˜R€C ˘Q#˚#¸#Å#É#Ö#Ü#à#é#è#ñ#ñ#ù#û#§#§#Æ“€ ˘Q#Ø#≠€C ˙Q#« #⁄#›#‹#›#„#‚#Í#Ì#Ï##Ú#Û#˜#˜#˚K€' ˙Q#Å#Ö#Ü#à#è#û#Æ#Ø#Ø⁄€ ˚Q#… #≤›€ ¸Q#  #÷#˚#¸#ˇ#Ä#Åœ€ ˝Q#À #’##Ò#Ú#Û1€ ˛Q#Ã #é#í8€ ˇQ#Õ #ù#†€ ÄR#Œ #◊#ï#ñ#£#§˘€ ÅR#œ #è#í#û#†€# ÇR#— #‹#ﬂ#·#‚#Ë#Ï#Ò†€ ÉR#“ #‘#ﬂ#·€ ÑR#º #≈#Ã#‘#⁄B€ ÖR#º #≈#⁄A€ ÜR#æ #€2€ áR#ø #√#Ã#ÕM€ àR#¿ #ƒ#‘#’:€' âR#¡ #≈#∆#«# # #œ#◊#⁄L€ äR#„ #Ï#Ó#Òπ€ ãR#„ #Ï#Ó#Ò∏€ åR#Â #ÚÓ€ çR#Ê #Ó#Ô##Òç€ éR#Á #È#Í#Î#Ïü€# èR#˙ #ˇ#Ä	#É	#Ö	#â	#ç	#é	w€ êR#¸ #Ö	#Ü	#á	#à	#â	Y€ ëR#˝ #É	#É	#É	#Ö	#â	dÿ   "                                                                 t8_t)0x0F) îAFIO_EVCR_PORT ((uint8_t)0x70) ïAFIO_EVCR_PORT_0 ((uint8_t)0x10) ñAFIO_EVCR_PORT_1 ((uint8_t)0x20) óAFIO_EVCR_PORT_2 ((uint8_t)0x40) öAFIO_EVCR_PORT_PA ((uint8_t)0x00) õAFIO_EVCR_PORT_PB ((uint8_t)0x10) úAFIO_EVCR_PORT_PC ((uint8_t)0x20) ùAFIO_EVCR_PORT_PD ((uint8_t)0x30) ûAFIO_EVCR_PORT_PE ((uint8_t)0x40) †AFIO_EVCR_EVOE ((uint8_t)0x80) £AFIO_MAPR_SPI1_REMAP ((uint32_t)0x00000001) §AFIO_MAPR_I2C1_REMAP ((uint32_t)0x00000002) •AFIO_MAPR_USART1_REMAP ((uint32_t)0x00000004) ¶AFIO_MAPR_USART2_REMAP ((uint32_t)0x00000008) ®AFIO_MAPR_USART3_REMAP ((uint32_t)0x00000030) ©AFIO_MAPR_USART3_REMAP_0 ((uint32_t)0x00000010) ™AFIO_MAPR_USART3_REMAP_1 ((uint32_t)0x00000020) ≠AFIO_MAPR_USART3_REMAP_NOREMAP ((uint32_t)0x00000000) ÆAFIO_MAPR_USART3_REMAP_PARTIALREMAP ((uint32_t)0x00000010) ØAFIO_MAPR_USART3_REMAP_FULLREMAP ((uint32_t)0x00000030) ±AFIO_MAPR_TIM1_REMAP ((uint32_t)0x000000C0) ≤AFIO_MAPR_TIM1_REMAP_0 ((uint32_t)0x00000040) ≥AFIO_MAPR_TIM1_REMAP_1 ((uint32_t)0x00000080) ∂AFIO_MAPR_TIM1_REMAP_NOREMAP ((uint32_t)0x00000000) ∑AFIO_MAPR_TIM1_REMAP_PARTIALREMAP ((uint32_t)0x00000040) ∏AFIO_MAPR_TIM1_REMAP_FULLREMAP ((uint32_t)0x000000C0) ∫AFIO_MAPR_TIM2_REMAP ((uint32_t)0x00000300) ªAFIO_MAPR_TIM2_REMAP_0 ((uint32_t)0x00000100) ºAFIO_MAPR_TIM2_REMAP_1 ((uint32_t)0x00000200) øAFIO_MAPR_TIM2_REMAP_NOREMAP ((uint32_t)0x00000000) ¿AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1 ((uint32_t)0x00000100) ¡AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2 ((uint32_t)0x00000200) ¬AFIO_MAPR_TIM2_REMAP_FULLREMAP ((uint32_t)0x00000300) ƒAFIO_MAPR_TIM3_REMAP ((uint32_t)0x00000C00) ≈AFIO_MAPR_TIM3_REMAP_0 ((uint32_t)0x00000400) ∆AFIO_MAPR_TIM3_REMAP_1 ((uint32_t)0x00000800) …AFIO_MAPR_TIM3_REMAP_NOREMAP ((uint32_t)0x00000000)  AFIO_MAPR_TIM3_REMAP_PARTIALREMAP ((uint32_t)0x00000800) ÀAFIO_MAPR_TIM3_REMAP_FULLREMAP ((uint32_t)0x00000C00) ÕAFIO_MAPR_TIM4_REMAP ((uint32_t)0x00001000) œAFIO_MAPR_CAN_REMAP ((uint32_t)0x00006000) –AFIO_MAPR_CAN_REMAP_0 ((uint32_t)0x00002000) —AFIO_MAPR_CAN_REMAP_1 ((uint32_t)0x00004000) ‘AFIO_MAPR_CAN_REMAP_REMAP1 ((uint32_t)0x00000000) ’AFIO_MAPR_CAN_REMAP_REMAP2 ((uint32_t)0x00004000) ÷AFIO_MAPR_CAN_REMAP_REMAP3 ((uint32_t)0x00006000) ÿAFIO_MAPR_PD01_REMAP ((uint32_t)0x00008000) ŸAFIO_MAPR_TIM5CH4_IREMAP ((uint32_t)0x00010000) ⁄AFIO_MAPR_ADC1_ETRGINJ_REMAP ((uint32_t)0x00020000) €AFIO_MAPR_ADC1_ETRGREG_REMAP ((uint32_t)0x00040000) ‹AFIO_MAPR_ADC2_ETRGINJ_REMAP ((uint32_t)0x00080000) ›AFIO_MAPR_ADC2_ETRGREG_REMAP ((uint32_t)0x00100000) ‡AFIO_MAPR_SWJ_CFG ((uint32_t)0x07000000) ·AFIO_MAPR_SWJ_CFG_0 ((uint32_t)0x01000000) ‚AFIO_MAPR_SWJ_CFG_1 ((uint32_t)0x02000000) „AFIO_MAPR_SWJ_CFG_2 ((uint32_t)0x04000000) ÂAFIO_MAPR_SWJ_CFG_RESET ((uint32_t)0x00000000) ÊAFIO_MAPR_SWJ_CFG_NOJNTRST ((uint32_t)0x01000000) ÁAFIO_MAPR_SWJ_CFG_JTAGDISABLE ((uint32_t)0x02000000) ËAFIO_MAPR_SWJ_CFG_DISABLE ((uint32_t)0x04000000) ÏAFIO_MAPR_ETH_REMAP ((uint32_t)0x00200000) ÔAFIO_MAPR_CAN2_REMAP ((uint32_t)0x00400000) ÚAFIO_MAPR_MII_RMII_SEL ((uint32_t)0x00800000) ıAFIO_MAPR_SPI3_REMAP ((uint32_t)0x10000000) ¯AFIO_MAPR_TIM2ITR1_IREMAP ((uint32_t)0x20000000) ˚AFIO_MAPR_PTP_PPS_REMAP ((uint32_t)0x40000000) ˇAFIO_EXTICR1_EXTI0 ((uint16_t)0x000F) ÄAFIO_EXTICR1_EXTI1 ((uint16_t)0x00F0) ÅAFIO_EXTICR1_EXTI2 ((uint16_t)0x0F00) ÇAFIO_EXTICR1_EXTI3 ((uint16_t)0xF000) ÖAFIO_EXTICR1_EXTI0_PA ((uint16_t)0x0000) ÜAFIO_EXTICR1_EXTI0_PB ((uint16_t)0x0001) áAFIO_EXTICR1_EXTI0_PC ((uint16_t)0x0002) àAFIO_EXTICR1_EXTI0_PD ((uint16_t)0x0003) âAFIO_EXTICR1_EXTI0_PE ((uint16_t)0x0004) äAFIO_EXTICR1_EXTI0_PF ((uint16_t)0x0005) ãAFIO_EXTICR1_EXTI0_PG ((uint16_t)0x0006) éAFIO_EXTICR1_EXTI1_PA ((uint16_t)0x0000) èAFIO_EXTICR1_EXTI1_PB ((uint16_t)0x0010) êAFIO_EXTICR1_EXTI1_PC ((uint16_t)0x0020) ëAFIO_EXTICR1_EXTI1_PD ((uint16_t)0x0030) íAFIO_EXTICR1_EXTI1_PE ((uint16_t)0x0040) ìAFIO_EXTICR1_EXTI1_PF ((uint16_t)0x0050) îAFIO_EXTICR1_EXTI1_PG ((uint16_t)0x0060) óAFIO_EXTICR1_EXTI2_PA ((uint16_t)0x0000) òAFIO_EXTICR1_EXTI2_PB ((uint16_t)0x0100) ôAFIO_EXTICR1_EXTI2_PC ((uint16_t)0x0200) öAFIO_EXTICR1_EXTI2_PD ((uint16_t)0x0300) õAFIO_EXTICR1_EXTI2_PE ((uint16_t)0x0400) úAFIO_EXTICR1_EXTI2_PF ((uint16_t)0x0500) ùAFIO_EXTICR1_EXTI2_PG ((uint16_t)0x0600) †AFIO_EXTICR1_EXTI3_PA ((uint16_t)0x0000) °AFIO_EXTICR1_EXTI3_PB ((uint16_t)0x1000) ¢AFIO_EXTICR1_EXTI3_PC ((uint16_t)0x2000) £AFIO_EXTICR1_EXTI3_PD ((uint16_t)0x3000) §AFIO_EXTICR1_EXTI3_PE ((uint16_t)0x4000) •AFIO_EXTICR1_EXTI3_PF ((uint16_t)0x5000) ¶AFIO_EXTICR1_EXTI3_PG ((uint16_t)0x6000) ©AFIO_EXTICR2_EXTI4 ((uint16_t)0x000F) ™AFIO_EXTICR2_EXTI5 ((uint16_t)0x00F0) ´AFIO_EXTICR2_EXTI6 ((uint16_t)0x0F00) ¨AFIO_EXTICR2_EXTI7 ((uint16_t)0xF000) ØAFIO_EXTICR2_EXTI4_PA ((uint16_t)0x0000) ∞AFIO_EXTICR2_EXTI4_PB ((uint16_t)0x0001) ±AFIO_EXTICR2_EXTI4_PC ((uint16_t)0x0002) ≤AFIO_EXTICR2_EXTI4_PD ((uint16_t)0x0003) ≥AFIO_EXTICR2_EXTI4_PE ((uint16_t)0x0004) ¥AFIO_EXTICR2_EXTI4_PF ((uint16_t)0x0005) µAFIO_EXTICR2_EXTI4_PG ((uint16_t)0x0006) ∏AFIO_EXTICR2_EXTI5_PA ((uint16_t)0x0000) πAFIO_EXTICR2_EXTI5_PB ((uint16_t)0x0010) ∫AFIO_EXTICR2_EXTI5_PC ((uint16_t)0x0020) ªAFIO_EXTICR2_EXTI5_PD ((uint16_t)0x0030) ºAFIO_EXTICR2_EXTI5_PE ((uint16_t)0x0040) ΩAFIO_EXTICR2_EXTI5_PF ((uint16_t)0x0050) æAFIO_EXTICR2_EXTI5_PG ((uint16_t)0x0060) ¡AFIO_EXTICR2_EXTI6_PA ((uint16_t)0x0000) ¬AFIO_EXTICR2_EXTI6_PB ((uint16_t)0x0100) √AFIO_EXTICR2_EXTI6_PC ((uint16_t)0x0200) ƒAFIO_EXTICR2_EXTI6_PD ((uint16_t)0x0300) ≈AFIO_EXTICR2_EXTI6_PE ((uint16_t)0x0400) ∆AFIO_EXTICR2_EXTI6_PF ((uint16_t)0x0500) «AFIO_EXTICR2_EXTI6_PG ((uint16_t)0x0600)  AFIO_EXTICR2_EXTI7_PA ((uint16_t)0x0000) ÀAFIO_EXTICR2_EXTI7_PB ((uint16_t)0x1000) ÃAFIO_EXTICR2_EXTI7_PC ((uint16_t)0x2000) ÕAFIO_EXTICR2_EXTI7_PD ((uint16_t)0x3000) ŒAFIO_EXTICR2_EXTI7_PE ((uint16_t)0x4000) œAFIO_EXTICR2_EXTI7_PF ((uint16_t)0x5000) –AFIO_EXTICR2_EXTI7_PG ((uint16_t)0x6000) ”AFIO_EXTICR3_EXTI8 ((uint16_t)0x000F) ‘AFIO_EXTICR3_EXTI9 ((uint16_t)0x00F0) ’AFIO_EXTICR3_EXTI10 ((uint16_t)0x0F00) ÷AFIO_EXTICR3_EXTI11 ((uint16_t)0xF000) ŸAFIO_EXTICR3_EXTI8_PA ((uint16_t)0x0000) ⁄AFIO_EXTICR3_EXTI8_PB ((uint16_t)0x0001) €AFIO_EXTICR3_EXTI8_PC ((uint16_t)0x0002) ‹AFIO_EXTICR3_EXTI8_PD ((uint16_t)0x0003) ›AFIO_EXTICR3_EXTI8_PE ((uint16_t)0x0004) ﬁAFIO_EXTICR3_EXTI8_PF ((uint16_t)0x0005) ﬂAFIO_EXTICR3_EXTI8_PG ((uint16_t)0x0006) ‚AFIO_EXTICR3_EXTI9_PA ((uint16_t)0x0000) „AFIO_EXTICR3_EXTI9_PB ((uint16_t)0x0010) ‰AFIO_EXTICR3_EXTI9_PC ((uint16_t)0x0020) ÂAFIO_EXTICR3_EXTI9_PD ((uint16_t)0x0030) ÊAFIO_EXTICR3_EXTI9_PE ((uint16_t)0x0040) ÁAFIO_EXTICR3_EXTI9_PF ((uint16_t)0x0050) ËAFIO_EXTICR3_EXTI9_PG ((uint16_t)0x0060) ÎAFIO_EXTICR3_EXTI10_PA ((uint16_t)0x0000) ÏAFIO_EXTICR3_EXTI10_PB ((uint16_t)0x0100) ÌAFIO_EXTICR3_EXTI10_PC ((uint16_t)0x0200) ÓAFIO_EXTICR3_EXTI10_PD ((uint16_t)0x0300) ÔAFIO_EXTICR3_EXTI10_PE ((uint16_t)0x0400) AFIO_EXTICR3_EXTI10_PF ((uint16_t)0x0500) ÒAFIO_EXTICR3_EXTI10_PG ((uint16_t)0x0600) ÙAFIO_EXTICR3_EXTI11_PA ((uint16_t)0x0000) ıAFIO_EXTICR3_EXTI11_PB ((uint16_t)0x1000) ˆAFIO_EXTICR3_EXTI11_PC ((uint16_t)0x2000) ˜AFIO_EXTICR3_EXTI11_PD ((uint16_t)0x3000) ¯AFIO_EXTICR3_EXTI11_PE ((uint16_t)0x4000) ˘AFIO_EXTICR3_EXTI11_PF ((uint16_t)0x5000) ˙AFIO_EXTICR3_EXTI11_PG ((uint16_t)0x6000) ˝AFIO_EXTICR4_EXTI12 ((uint16_t)0x000F) ˛AFIO_EXTICR4_EXTI13 ((uint16_t)0x00F0) ˇAFIO_EXTICR4_EXTI14 ((uint16_t)0x0F00) ÄAFIO_EXTICR4_EXTI15 ((uint16_t)0xF000) ÉAFIO_EXTICR4_EXTI12_PA ((uint16_t)0x0000) ÑAFIO_EXTICR4_EXTI12_PB ((uint16_t)0x0001) ÖAFIO_EXTICR4_EXTI12_PC ((uint16_t)0x0002) ÜAFIO_EXTICR4_EXTI12_PD ((uint16_t)0x0003) áAFIO_EXTICR4_EXTI12_PE ((uint16_t)0x0004) àAFIO_EXTICR4_EXTI12_PF ((uint16_t)0x0005) âAFIO_EXTICR4_EXTI12_PG ((uint16_t)0x0006) åAFIO_EXTICR4_EXTI13_PA ((uint16_t)0x0000) çAFIO_EXTICR4_EXTI13_PB ((uint16_t)0x0010) éAFIO_EXTICR4_EXTI13_PC ((uint16_t)0x0020) èAFIO_EXTICR4_EXTI13_PD ((uint16_t)0x0030) êAFIO_EXTICR4_EXTI13_PE ((uint16_t)0x0040) ëAFIO_EXTICR4_EXTI13_PF ((uint16_t)0x0050) íAFIO_EXTICR4_EXTI13_PG ((uint16_t)0x0060) ïAFIO_EXTICR4_EXTI14_PA ((uint16_t)0x0000) ñAFIO_EXTICR4_EXTI14_PB ((uint16_t)0x0100) óAFIO_EXTICR4_EXTI14_PC ((uint16_t)0x0200) òAFIO_EXTICR4_EXTI14_PD ((uint16_t)0x0300) ôAFIO_EXTICR4_EXTI14_PE ((uint16_t)0x0400) öAFIO_EXTICR4_EXTI14_PF ((uint16_t)0x0500) õAFIO_EXTICR4_EXTI14_PG ((uint16_t)0x0600) ûAFIO_EXTICR4_EXTI15_PA ((uint16_t)0x0000) üAFIO_EXTICR4_EXTI15_PB ((uint16_t)0x1000) †AFIO_EXTICR4_EXTI15_PC ((uint16_t)0x2000) °AFIO_EXTICR4_EXTI15_PD ((uint16_t)0x3000) ¢AFIO_EXTICR4_EXTI15_PE ((uint16_t)0x4000) £AFIO_EXTICR4_EXTI15_PF ((uint16_t)0x5000) §AFIO_EXTICR4_EXTI15_PG ((uint16_t)0x6000) …SysTick_CTRL_ENABLE ((uint32_t)0x00000001)  SysTick_CTRL_TICKINT ((uint32_t)0x00000002) ÀSysTick_CTRL_CLKSOURCE ((uint32_t)0x00000004) ÃSysTick_CTRL_COUNTFLAG ((uint32_t)0x00010000) œSysTick_LOAD_RELOAD ((uint32_t)0x00FFFFFF) “SysTick_VAL_CURRENT ((uint32_t)0x00FFFFFF) ’SysTick_CALIB_TENMS ((uint32_t)0x00FFFFFF) ÷SysTick_CALIB_SKEW ((uint32_t)0x40000000) ◊SysTick_CALIB_NOREF ((uint32_t)0x80000000) ‡NVIC_ISER_SETENA ((uint32_t)0xFFFFFFFF) ·NVIC_ISER_SETENA_0 ((uint32_t)0x00000001) ‚NVIC_ISER_SETENA_1 ((uint32_t)0x00000002) „NVIC_ISER_SETENA_2 ((uint32_t)0x00000004) ‰NVIC_ISER_SETENA_3 ((uint32_t)0x00000008) ÂNVIC_ISER_SETENA_4 ((uint32_t)0x00000010) ÊNVIC_ISER_SETENA_5 ((uint32_t)0x00000020) ÁNVIC_ISER_SETENA_6 ((uint32_t)0x00000040) ËNVIC_ISER_SETENA_7 ((uint32_t)0x00000080) ÈNVIC_ISER_SETENA_8 ((uint32_t)0x00000100) ÍNVIC_ISER_SETENA_9 ((uint32_t)0x00000200) ÎNVIC_ISER_SETENA_10 ((uint32_t)0x00000400) ÏNVIC_ISER_SETENA_11 ((uint32_t)0x00000800) ÌNVIC_ISER_SETENA_12 ((uint32_t)0x00001000) ÓNVIC_ISER_SETENA_13 ((uint32_t)0x00002000) ÔNVIC_ISER_SETENA_14 ((uint32_t)0x00004000) NVIC_ISER_SETENA_15 ((uint32_t)0x00008000) ÒNVIC_ISER_SETENA_16 ((uint32_t)0x00010000) ÚNVIC_ISER_SETENA_17 ((uint32_t)0x00020000) ÛNVIC_ISER_SETENA_18 ((uint32_t)0x00040000) ÙNVIC_ISER_SETENA_19 ((uint32_t)0x00080000) ıNVIC_ISER_SETENA_20 ((uint32_t)0x00100000) ˆNVIC_ISER_SETENA_21 ((uint32_t)0x00200000) ˜NVIC_ISER_SETENA_22 ((uint32_t)0x00400000) ¯NVIC_ISER_SETENA_23 ((uint32_t)0x00800000) ˘NVIC_ISER_SETENA_24 ((uint32_t)0x01000000) ˙NVIC_ISER_SETENA_25 ((uint32_t)0x02000000) ˚NVIC_ISER_SETENA_26 ((uint32_t)0x04000000) ¸NVIC_ISER_SETENA_27 ((uint32_t)0x08000000) ˝NVIC_ISER_SETENA_28 ((uint32_t)0x10000000) ˛NVIC_ISER_SETENA_29 ((uint32_t)0x20000000) ˇNVIC_ISER_SETENA_30 ((uint32_t)0x40000000) ÄNVIC_ISER_SETENA_31 ((uint32_t)0x80000000) ÉNVIC_ICER_CLRENA ((uint32_t)0xFFFFFFFF) ÑNVIC_ICER_CLRENA_0 ((uint32_t)0x00000001) ÖNVIC_ICER_CLRENA_1 ((uint32_t)0x00000002) ÜNVIC_ICER_CLRENA_2 ((uint32_t)0x00000004) áNVIC_ICER_CLRENA_3 ((uint32_t)0x00000008) àNVIC_ICER_CLRENA_4 ((uint32_t)0x00000010) âNVIC_ICER_CLRENA_5 ((uint32_t)0x00000020) äNVIC_ICER_CLRENA_6 ((uint32_t)0x00000040) ãNVIC_ICER_CLRENA_7 ((uint32_t)0x00000080) åNVIC_ICER_CLRENA_8 ((uint32_t)0x00000100) ç.\objects\usb_core.o: ..\Source\STM32_USB_OTG_Driver\src\usb_core.c
.\objects\usb_core.o: ..\Source\STM32_USB_OTG_Driver\inc\usb_core.h
.\objects\usb_core.o: ..\Source\USER\usb_conf.h
.\objects\usb_core.o: ..\Source\USER\stm32f10x.h
.\objects\usb_core.o: ..\Source\CORE\core_cm3.h
.\objects\usb_core.o: D:\Program\keil5\ARM\ARMCC\Bin\..\include\stdint.h
.\objects\usb_core.o: ..\Source\USER\system_stm32f10x.h
.\objects\usb_core.o: ..\Source\USER\stm32f10x_conf.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_adc.h
.\objects\usb_core.o: ..\Source\USER\stm32f10x.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_bkp.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_can.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_cec.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_crc.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_dac.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_dbgmcu.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_dma.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_exti.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_flash.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_fsmc.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_gpio.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_i2c.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_iwdg.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_pwr.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_rcc.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_rtc.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_sdio.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_spi.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_tim.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_usart.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\stm32f10x_wwdg.h
.\objects\usb_core.o: ..\Source\STM32F10x_FWLib\inc\misc.h
.\objects\usb_core.o: ..\Source\STM32_USB_OTG_Driver\inc\usb_regs.h
.\objects\usb_core.o: ..\Source\STM32_USB_OTG_Driver\inc\usb_defines.h
.\objects\usb_core.o: ..\Source\STM32_USB_OTG_Driver\inc\usb_bsp.h
                                                                                                                                                                                                                                              TPEND_31 ((uint32_t)0x80000000) …NVIC_ICPR_CLRPEND ((uint32_t)0xFFFFFFFF)  NVIC_ICPR_CLRPEND_0 ((uint32_t)0x00000001) ÀNVIC_ICPR_CLRPEND_1 ((uint32_t)0x00000002) ÃNVIC_ICPR_CLRPEND_2 ((uint32_t)0x00000004) ÕNVIC_ICPR_CLRPEND_3 ((uint32_t)0x00000008) ŒNVIC_ICPR_CLRPEND_4 ((uint32_t)0x00000010) œNVIC_ICPR_CLRPEND_5 ((uint32_t)0x00000020) –NVIC_ICPR_CLRPEND_6 ((uint32_t)0x00000040) —NVIC_ICPR_CLRPEND_7 ((uint32_t)0x00000080) “NVIC_ICPR_CLRPEND_8 ((uint32_t)0x00000100) ”NVIC_ICPR_CLRPEND_9 ((uint32_t)0x00000200) ‘NVIC_ICPR_CLRPEND_10 ((uint32_t)0x00000400) ’NVIC_ICPR_CLRPEND_11 ((uint32_t)0x00000800) ÷NVIC_ICPR_CLRPEND_12 ((uint32_t)0x00001000) ◊NVIC_ICPR_CLRPEND_13 ((uint32_t)0x00002000) ÿNVIC_ICPR_CLRPEND_14 ((uint32_t)0x00004000) ŸNVIC_ICPR_CLRPEND_15 ((uint32_t)0x00008000) ⁄NVIC_ICPR_CLRPEND_16 ((uint32_t)0x00010000) €NVIC_ICPR_CLRPEND_17 ((uint32_t)0x00020000) ‹NVIC_ICPR_CLRPEND_18 ((uint32_t)0x00040000) ›NVIC_ICPR_CLRPEND_19 ((uint32_t)0x00080000) ﬁNVIC_ICPR_CLRPEND_20 ((uint32_t)0x00100000) ﬂNVIC_ICPR_CLRPEND_21 ((uint32_t)0x00200000) ‡NVIC_ICPR_CLRPEND_22 ((uint32_t)0x00400000) ·NVIC_ICPR_CLRPEND_23 ((uint32_t)0x00800000) ‚NVIC_ICPR_CLRPEND_24 ((uint32_t)0x01000000) „NVIC_ICPR_CLRPEND_25 ((uint32_t)0x02000000) ‰NVIC_ICPR_CLRPEND_26 ((uint32_t)0x04000000) ÂNVIC_ICPR_CLRPEND_27 ((uint32_t)0x08000000) ÊNVIC_ICPR_CLRPEND_28 ((uint32_t)0x10000000) ÁNVIC_ICPR_CLRPEND_29 ((uint32_t)0x20000000) ËNVIC_ICPR_CLRPEND_30 ((uint32_t)0x40000000) ÈNVIC_ICPR_CLRPEND_31 ((uint32_t)0x80000000) ÏNVIC_IABR_ACTIVE ((uint32_t)0xFFFFFFFF) ÌNVIC_IABR_ACTIVE_0 ((uint32_t)0x00000001) ÓNVIC_IABR_ACTIVE_1 ((uint32_t)0x00000002) ÔNVIC_IABR_ACTIVE_2 ((uint32_t)0x00000004) NVIC_IABR_ACTIVE_3 ((uint32_t)0x00000008) ÒNVIC_IABR_ACTIVE_4 ((uint32_t)0x00000010) ÚNVIC_IABR_ACTIVE_5 ((uint32_t)0x00000020) ÛNVIC_IABR_ACTIVE_6 ((uint32_t)0x00000040) ÙNVIC_IABR_ACTIVE_7 ((uint32_t)0x00000080) ıNVIC_IABR_ACTIVE_8 ((uint32_t)0x00000100) ˆNVIC_IABR_ACTIVE_9 ((uint32_t)0x00000200) ˜NVIC_IABR_ACTIVE_10 ((uint32_t)0x00000400) ¯NVIC_IABR_ACTIVE_11 ((uint32_t)0x00000800) ˘NVIC_IABR_ACTIVE_12 ((uint32_t)0x00001000) ˙NVIC_IABR_ACTIVE_13 ((uint32_t)0x00002000) ˚NVIC_IABR_ACTIVE_14 ((uint32_t)0x00004000) ¸NVIC_IABR_ACTIVE_15 ((uint32_t)0x00008000) ˝NVIC_IABR_ACTIVE_16 ((uint32_t)0x00010000) ˛NVIC_IABR_ACTIVE_17 ((uint32_t)0x00020000) ˇNVIC_IABR_ACTIVE_18 ((uint32_t)0x00040000) ÄNVIC_IABR_ACTIVE_19 ((uint32_t)0x00080000) ÅNVIC_IABR_ACTIVE_20 ((uint32_t)0x00100000) ÇNVIC_IABR_ACTIVE_21 ((uint32_t)0x00200000) ÉNVIC_IABR_ACTIVE_22 ((uint32_t)0x00400000) ÑNVIC_IABR_ACTIVE_23 ((uint32_t)0x00800000) ÖNVIC_IABR_ACTIVE_24 ((uint32_t)0x01000000) ÜNVIC_IABR_ACTIVE_25 ((uint32_t)0x02000000) áNVIC_IABR_ACTIVE_26 ((uint32_t)0x04000000) àNVIC_IABR_ACTIVE_27 ((uint32_t)0x08000000) âNVIC_IABR_ACTIVE_28 ((uint32_t)0x10000000) äNVIC_IABR_ACTIVE_29 ((uint32_t)0x20000000) ãNVIC_IABR_ACTIVE_30 ((uint32_t)0x40000000) åNVIC_IABR_ACTIVE_31 ((uint32_t)0x80000000) èNVIC_IPR0_PRI_0 ((uint32_t)0x000000FF) êNVIC_IPR0_PRI_1 ((uint32_t)0x0000FF00) ëNVIC_IPR0_PRI_2 ((uint32_t)0x00FF0000) íNVIC_IPR0_PRI_3 ((uint32_t)0xFF000000) ïNVIC_IPR1_PRI_4 ((uint32_t)0x000000FF) ñNVIC_IPR1_PRI_5 ((uint32_t)0x0000FF00) óNVIC_IPR1_PRI_6 ((uint32_t)0x00FF0000) òNVIC_IPR1_PRI_7 ((uint32_t)0xFF000000) õNVIC_IPR2_PRI_8 ((uint32_t)0x000000FF) úNVIC_IPR2_PRI_9 ((uint32_t)0x0000FF00) ùNVIC_IPR2_PRI_10 ((uint32_t)0x00FF0000) ûNVIC_IPR2_PRI_11 ((uint32_t)0xFF000000) °NVIC_IPR3_PRI_12 ((uint32_t)0x000000FF) ¢NVIC_IPR3_PRI_13 ((uint32_t)0x0000FF00) £NVIC_IPR3_PRI_14 ((uint32_t)0x00FF0000) §NVIC_IPR3_PRI_15 ((uint32_t)0xFF000000) ßNVIC_IPR4_PRI_16 ((uint32_t)0x000000FF) ®NVIC_IPR4_PRI_17 ((uint32_t)0x0000FF00) ©NVIC_IPR4_PRI_18 ((uint32_t)0x00FF0000) ™NVIC_IPR4_PRI_19 ((uint32_t)0xFF000000) ≠NVIC_IPR5_PRI_20 ((uint32_t)0x000000FF) ÆNVIC_IPR5_PRI_21 ((uint32_t)0x0000FF00) ØNVIC_IPR5_PRI_22 ((uint32_t)0x00FF0000) ∞NVIC_IPR5_PRI_23 ((uint32_t)0xFF000000) ≥NVIC_IPR6_PRI_24 ((uint32_t)0x000000FF) ¥NVIC_IPR6_PRI_25 ((uint32_t)0x0000FF00) µNVIC_IPR6_PRI_26 ((uint32_t)0x00FF0000) ∂NVIC_IPR6_PRI_27 ((uint32_t)0xFF000000) πNVIC_IPR7_PRI_28 ((uint32_t)0x000000FF) ∫NVIC_IPR7_PRI_29 ((uint32_t)0x0000FF00) ªNVIC_IPR7_PRI_30 ((uint32_t)0x00FF0000) ºNVIC_IPR7_PRI_31 ((uint32_t)0xFF000000) øSCB_CPUID_REVISION ((uint32_t)0x0000000F) ¿SCB_CPUID_PARTNO ((uint32_t)0x0000FFF0) ¡SCB_CPUID_Constant ((uint32_t)0x000F0000) ¬SCB_CPUID_VARIANT ((uint32_t)0x00F00000) √SCB_CPUID_IMPLEMENTER ((uint32_t)0xFF000000) ∆SCB_ICSR_VECTACTIVE ((uint32_t)0x000001FF) «SCB_ICSR_RETTOBASE ((uint32_t)0x00000800) »SCB_ICSR_VECTPENDING ((uint32_t)0x003FF000) …SCB_ICSR_ISRPENDING ((uint32_t)0x00400000)  SCB_ICSR_ISRPREEMPT ((uint32_t)0x00800000) ÀSCB_ICSR_PENDSTCLR ((uint32_t)0x02000000) ÃSCB_ICSR_PENDSTSET ((uint32_t)0x04000000) ÕSCB_ICSR_PENDSVCLR ((uint32_t)0x08000000) ŒSCB_ICSR_PENDSVSET ((uint32_t)0x10000000) œSCB_ICSR_NMIPENDSET ((uint32_t)0x80000000) “SCB_VTOR_TBLOFF ((uint32_t)0x1FFFFF80) ”SCB_VTOR_TBLBASE ((uint32_t)0x20000000) ÷SCB_AIRCR_VECTRESET ((uint32_t)0x00000001) ◊SCB_AIRCR_VECTCLRACTIVE ((uint32_t)0x00000002) ÿSCB_AIRCR_SYSRESETREQ ((uint32_t)0x00000004) ⁄SCB_AIRCR_PRIGROUP ((uint32_t)0x00000700) €SCB_AIRCR_PRIGROUP_0 ((uint32_t)0x00000100) ‹SCB_AIRCR_PRIGROUP_1 ((uint32_t)0x00000200) ›SCB_AIRCR_PRIGROUP_2 ((uint32_t)0x00000400) ‡SCB_AIRCR_PRIGROUP0 ((uint32_t)0x00000000) ·SCB_AIRCR_PRIGROUP1 ((uint32_t)0x00000100) ‚SCB_AIRCR_PRIGROUP2 ((uint32_t)0x00000200) „SCB_AIRCR_PRIGROUP3 ((uint32_t)0x00000300) ‰SCB_AIRCR_PRIGROUP4 ((uint32_t)0x00000400) ÂSCB_AIRCR_PRIGROUP5 ((uint32_t)0x00000500) ÊSCB_AIRCR_PRIGROUP6 ((uint32_t)0x00000600) ÁSCB_AIRCR_PRIGROUP7 ((uint32_t)0x00000700) ÈSCB_AIRCR_ENDIANESS ((uint32_t)0x00008000) ÍSCB_AIRCR_VECTKEY ((uint32_t)0xFFFF0000) ÌSCB_SCR_SLEEPONEXIT ((uint8_t)0x02) ÓSCB_SCR_SLEEPDEEP ((uint8_t)0x04) ÔSCB_SCR_SEVONPEND ((uint8_t)0x10) ÚSCB_CCR_NONBASETHRDENA ((uint16_t)0x0001) ÛSCB_CCR_USERSETMPEND ((uint16_t)0x0002) ÙSCB_CCR_UNALIGN_TRP ((uint16_t)0x0008) ıSCB_CCR_DIV_0_TRP ((uint16_t)0x0010) ˆSCB_CCR_BFHFNMIGN ((uint16_t)0x0100) ˜SCB_CCR_STKALIGN ((uint16_t)0x0200) ˙SCB_SHPR_PRI_N ((uint32_t)0x000000FF) ˚SCB_SHPR_PRI_N1 ((uint32_t)0x0000FF00) ¸SCB_SHPR_PRI_N2 ((uint32_t)0x00FF0000) ˝SCB_SHPR_PRI_N3 ((uint32_t)0xFF000000) ÄSCB_SHCSR_MEMFAULTACT ((uint32_t)0x00000001) ÅSCB_SHCSR_BUSFAULTACT ((uint32_t)0x00000002) ÇSCB_SHCSR_USGFAULTACT ((uint32_t)0x00000008) ÉSCB_SHCSR_SVCALLACT ((uint32_t)0x00000080) ÑSCB_SHCSR_MONITORACT ((uint32_t)0x00000100) ÖSCB_SHCSR_PENDSVACT ((uint32_t)0x00000400) ÜSCB_SHCSR_SYSTICKACT ((uint32_t)0x00000800) áSCB_SHCSR_USGFAULTPENDED ((uint32_t)0x00001000) àSCB_SHCSR_MEMFAULTPENDED ((uint32_t)0x00002000) âSCB_SHCSR_BUSFAULTPENDED ((uint32_t)0x00004000) äSCB_SHCSR_SVCALLPENDED ((uint32_t)0x00008000) ãSCB_SHCSR_MEMFAULTENA ((uint32_t)0x00010000) åSCB_SHCSR_BUSFAULTENA ((uint32_t)0x00020000) çSCB_SHCSR_USGFAULTENA ((uint32_t)0x00040000) ëSCB_CFSR_IACCVIOL ((uint32_t)0x00000001) íSCB_CFSR_DACCVIOL ((uint32_t)0x00000002) ìSCB_CFSR_MUNSTKERR ((uint32_t)0x00000008) îSCB_CFSR_MSTKERR ((uint32_t)0x00000010) ïSCB_CFSR_MMARVALID ((uint32_t)0x00000080) óSCB_CFSR_IBUSERR ((uint32_t)0x00000100) òSCB_CFSR_PRECISERR ((uint32_t)0x00000200) ôSCB_CFSR_IMPRECISERR ((uint32_t)0x00000400) öSCB_CFSR_UNSTKERR ((uint32_t)0x00000800) õSCB_CFSR_STKERR ((uint32_t)0x00001000) úSCB_CFSR_BFARVALID ((uint32_t)0x00008000) ûSCB_CFSR_UNDEFINSTR ((uint32_t)0x00010000) üSCB_CFSR_INVSTATE ((uint32_t)0x00020000) †SCB_CFSR_INVPC ((uint32_t)0x00040000) °SCB_CFSR_NOCP ((uint32_t)0x00080000) ¢SCB_CFSR_UNALIGNED ((uint32_t)0x01000000) £SCB_CFSR_DIVBYZERO ((uint32_t)0x02000000) ¶SCB_HFSR_VECTTBL ((uint32_t)0x00000002) ßSCB_HFSR_FORCED ((uint32_t)0x40000000) ®SCB_HFSR_DEBUGEVT ((uint32_t)0x80000000) ´SCB_DFSR_HALTED ((uint8_t)0x01) ¨SCB_DFSR_BKPT ((uint8_t)0x02) ≠SCB_DFSR_DWTTRAP ((uint8_t)0x04) ÆSCB_DFSR_VCATCH ((uint8_t)0x08) ØSCB_DFSR_EXTERNAL ((uint8_t)0x10) ≤SCB_MMFAR_ADDRESS ((uint32_t)0xFFFFFFFF) µSCB_BFAR_ADDRESS ((uint32_t)0xFFFFFFFF) ∏SCB_AFSR_IMPDEF ((uint32_t)0xFFFFFFFF) ¡EXTI_IMR_MR0 ((uint32_t)0x00000001) ¬EXTI_IMR_MR1 ((uint32_t)0x00000002) √EXTI_IMR_MR2 ((uint32_t)0x00000004) ƒEXTI_IMR_MR3 ((uint32_t)0x00000008) ≈EXTI_IMR_MR4 ((uint32_t)0x00000010) ∆EXTI_IMR_MR5 ((uint32_t)0x00000020) «EXTI_IMR_MR6 ((uint32_t)0x00000040) »EXTI_IMR_MR7 ((uint32_t)0x00000080) …EXTI_IMR_MR8 ((uint32_t)0x00000100)  EXTI_IMR_MR9 ((uint32_t)0x00000200) ÀEXTI_IMR_MR10 ((uint32_t)0x00000400) ÃEXTI_IMR_MR11 ((uint32_t)0x00000800) ÕEXTI_IMR_MR12 ((uint32_t)0x00001000) ŒEXTI_IMR_MR13 ((uint32_t)0x00002000) œEXTI_IMR_MR14 ((uint32_t)0x00004000) –EXTI_IMR_MR15 ((uint32_t)0x00008000) —EXTI_IMR_MR16 ((uint32_t)0x00010000) “EXTI_IMR_MR17 ((uint32_t)0x00020000) ”EXTI_IMR_MR18 ((uint32_t)0x00040000) ‘EXTI_IMR_MR19 ((uint32_t)0x00080000) ◊EXTI_EMR_MR0 ((uint32_t)0x00000001) ÿEXTI_EMR_MR1 ((uint32_t)0x00000002) ŸEXTI_EMR_MR2 ((uint32_t)0x00000004) ⁄EXTI_EMR_MR3 ((uint32_t)0x00000008) €EXTI_EMR_MR4 ((uint32_t)0x00000010) ‹EXTI_EMR_MR5 ((uint32_t)0x00000020) ›EXTI_EMR_MR6 ((uint32_t)0x00000040) ﬁEXTI_EMR_MR7 ((uint32_t)0x00000080) ﬂEXTI_EMR_MR8 ((uint32_t)0x00000100) ‡EXTI_EMR_MR9 ((uint32_t)0x00000200) ·EXTI_EMR_MR10 ((uint32_t)0x00000400) ‚EXTI_EMR_MR11 ((uint32_t)0x00000800) „EXTI_EMR_MR12 ((uint32_t)0x00001000) ‰EXTI_EMR_MR13 ((uint32_t)0x00002000) ÂEXTI_EMR_MR14 ((uint32_t)0x00004000) ÊEXTI_EMR_MR15 ((uint32_t)0x00008000) ÁEXTI_EMR_MR16 ((uint32_t)0x00010000) ËEXTI_EMR_MR17 ((uint32_t)0x00020000) ÈEXTI_EMR_MR18 ((uint32_t)0x00040000) ÍEXTI_EMR_MR19 ((uint32_t)0x00080000) ÌEXTI_RTSR_TR0 ((uint32_t)0x00000001) ÓEXTI_RTSR_TR1 ((uint32_t)0x00000002) ÔEXTI_RTSR_TR2 ((uint32_t)0x00000004) EXTI_RTSR_TR3 ((uint32_t)0x00000008) ÒEXTI_RTSR_TR4 ((uint32_t)0x00000010) ÚEXTI_RTSR_TR5 ((uint32_t)0x00000020) ÛEXTI_RTSR_TR6 ((uint32_t)0x00000040) ÙEXTI_RTSR_TR7 ((uint32_t)0x00000080) ıEXTI_RTSR_TR8 ((uint32_t)0x00000100) ˆEXTI_RTSR_TR9 ((uint32_t)0x00000200) ˜EXTI_RTSR_TR10 ((uint32_t)0x00000400) ¯EXTI_RTSR_TR11 ((uint32_t)0x00000800) ˘EXTI_RTSR_TR12 ((uint32_t)0x00001000) ˙EXTI_RTSR_TR13 ((uint32_t)0x00002000) ˚EXTI_RTSR_TR14 ((uint32_t)0x00004000) ¸EXTI_RTSR_TR15 ((uint32_t)0x00008000) ˝EXTI_RTSR_TR16 ((uint32_t)0x00010000) ˛EXTI_RTSR_TR17 ((uint32_t)0x00020000) ˇEXTI_RTSR_TR18 ((uint32_t)0x00040000) ÄEXTI_RTSR_TR19 ((uint32_t)0x00080000) ÉEXTI_FTSR_TR0 ((uint32_t)0x00000001) ÑEXTI_FTSR_TR1 ((uint32_t)0x00000002) ÖEXTI_FTSR_TR2 ((uint32_t)0x00000004) ÜEXTI_FTSR_TR3 ((uint32_t)0x00000008) áEXTI_FTSR_TR4 ((uint32_t)0x00000010) àEXTI_FTSR_TR5 ((uint32_t)0x00000020) âEXTI_FTSR_TR6 ((uint32_t)0x00000040) äEXTI_FTSR_TR7 ((uint32_t)0x00000080) ãEXTI_FTSR_TR8 ((uint32_t)0x00000100) åEXTI_FTSR_TR9 ((uint32_t)0x00000200) çEXTI_FTSR_TR10 ((uint32_t)0x00000400) éEXTI_FTSR_TR11 ((uint32_t)0x00000800) èEXTI_FTSR_TR12 ((uint32_t)0x00001000) êEXTI_FTSR_TR13 ((uint32_t)0x00002000) ëEXTI_FTSR_TR14 ((uint32_t)0x00004000) íEXTI_FTSR_TR15 ((uint32_t)0x00008000) ìEXTI_FTSR_TR16 ((uint32_t)0x00010000) îEXTI_FTSR_TR17 ((uint32_t)0x00020000) ïEXTI_FTSR_TR18 ((uint32_t)0x00040000) ñEXTI_FTSR_TR19 ((uint32_t)0x00080000) ôEXTI_SWIER_SWIER0 ((uint32_t)0x00000001) öEXTI_SWIER_SWIER1 ((uint32_t)0x00000002) õEXTI_SWIER_SWIER2 ((uint32_t)0x00000004) úEXTI_SWIER_SWIER3 ((uint32_t)0x00000008) ùEXTI_SWIER_SWIER4 ((uint32_t)0x00000010) ûEXTI_SWIER_SWIER5 ((uint32_t)0x00000020) üEXTI_SWIER_SWIER6 ((uint32_t)0x00000040) †EXTI_SWIER_SWIER7 ((uint32_t)0x00000080) °EXTI_SWIER_SWIER8 ((uint32_t)0x00000100) ¢EXTI_SWIER_SWIER9 ((uint32_t)0x00000200) £EXTI_SWIER_SWIER10 ((uint32_t)0x00000400) §EXTI_SWIER_SWIER11 ((uint32_t)0x00000800) •EXTI_SWIER_SWIER12 ((uint32_t)0x00001000) ¶EXTI_SWIER_SWIER13 ((uint32_t)0x00002000) ßEXTI_SWIER_SWIER14 ((uint32_t)0x00004000) ®EXTI_SWIER_SWIER15 ((uint32_t)0x00008000) ©EXTI_SWIER_SWIER16 ((uint32_t)0x00010000) ™EXTI_SWIER_SWIER17 ((uint32_t)0x00020000) ´EXTI_SWIER_SWIER18 ((uint32_t)0x00040000) ¨EXTI_SWIER_SWIER19 ((uint32_t)0x00080000) ØEXTI_PR_PR0 ((uint32_t)0x00000001) ∞EXTI_PR_PR1 ((uint32_t)0x00000002) ±EXTI_PR_PR2 ((uint32_t)0x00000004) ≤EXTI_PR_PR3 ((uint32_t)0x00000008) ≥EXTI_PR_PR4 ((uint32_t)0x00000010) ¥EXTI_PR_PR5 ((uint32_t)0x00000020) µEXTI_PR_PR6 ((uint32_t)0x00000040) ∂EXTI_PR_PR7 ((uint32_t)0x00000080) ∑EXTI_PR_PR8 ((uint32_t)0x00000100) ∏EXTI_PR_PR9 ((uint32_t)0x00000200) πEXTI_PR_PR10 ((uint32_t)0x00000400) ∫EXTI_PR_PR11 ((uint32_t)0x00000800) ªEXTI_PR_PR12 ((uint32_t)0x00001000) ºEXTI_PR_PR13 ((uint32_t)0x00002000) ΩEXTI_PR_PR14 ((uint32_t)0x00004000) æEXTI_PR_PR15 ((uint32_t)0x00008000) øEXTI_PR_PR16 ((uint32_t)0x00010000) ¿EXTI_PR_PR17 ((uint32_t)0x00020000) ¡EXTI_PR_PR18 ((uint32_t)0x00040000) ¬EXTI_PR_PR19 ((uint32_t)0x00080000) ÀDMA_ISR_GIF1 ((uint32_t)0x00000001) ÃDMA_ISR_TCIF1 ((uint32_t)0x00000002) ÕDMA_ISR_HTIF1 ((uint32_t)0x00000004) ŒDMA_ISR_TEIF1 ((uint32_t)0x00000008) œDMA_ISR_GIF2 ((uint32_t)0x00000010) –DMA_ISR_TCIF2 ((uint32_t)0x00000020) —DMA_ISR_HTIF2 ((uint32_t)0x00000040) “DMA_ISR_TEIF2 ((uint32_t)0x00000080) ”DMA_ISR_GIF3 ((uint32_t)0x00000100) ‘DMA_ISR_TCIF3 ((uint32_t)0x00000200) ’DMA_ISR_HTIF3 ((uint32_t)0x00000400) ÷DMA_ISR_TEIF3 ((uint32_t)0x00000800) ◊DMA_ISR_GIF4 ((uint32_t)0x00001000) ÿDMA_ISR_TCIF4 ((uint32_t)0x00002000) ŸDMA_ISR_HTIF4 ((uint32_ELF          (            ÿı    4     ( Ê„>µF %  êê ê z(=—‡hÄkêò ÙÄ0ê`z(±ò ÙÄ ıÄêò·hàc  ê‡h¿hêò @ êò ÙÄ êò Ù 0êò Ù  êò·h»` Fˇ˜˛ˇ‡x(C— ò  
0 ê ò    0 ê ò·hà`5‡‡h¿hêò @ @0êò·h»` Fˇ˜˛ˇ  êò ÙÄ0 ıÄ0êò ÙÄ  ıÄ êò Ù   ı  êò Ù  ı ê`z(±ò ÙÄ ıÄêò·hàc ˇ˜˛ˇ‡x(—‡hÄh ê ò  
0 ê ò    0 ê ò·hà`(F>Ω˛µF & %  êê ê Fˇ˜˛ˇ !‘¯` z(— Fˇ˜˛ˇ‡! Fˇ˜˛ˇ Fˇ˜˛ˇ`i h ê ò   ê ô`i`‡z(—Ä ·hHbòoÛ Ä0êòÄ≤ ı¿ êò·hàbòoÛ ‡0êòÄ≤ ı¿ êò·h¡¯ ! Fˇ˜˛ˇ Fˇ˜˛ˇ %‡Oˇ1Òê P¯% Å` !Òê P¯% ¡`m x®BÓÿ! Fˇ˜˛ˇ Fˇ˜˛ˇ0F˛Ω¯µF & %   ê ø ˇ˜˛ˇ‡h i êhFIàBŸ  ¯Ω ò¿ (Ó– % ò  @ ê ò·ha ø‡h i êhFIàB Ÿ‡ ò   (Ò— ø ˇ˜˛ˇ0F‹Á@ µF   " í ö"R í híh õöCÀhö`Ω8µFF   ê!F(Fˇ˜˛ˇ(Fˇ˜˛ˇ ê ò¿Û 0Xπ,	— ò ÙÄP ıÄP ê ô’¯Ã ` ò¿Û 0@±<π ò ÙÄP ê ô’¯Ã `» ˇ˜˛ˇ8Ωµ ! ëI¬hQ`oÄA¬hQa ô! AÒ A ë ô!Ù aı a ë ô¬hëaΩµF   " í ö"R í híh õCÀhö`Ω8µF %   ê·hàa@·hHa Fˇ˜˛ˇ‡x π ò  0 ê ò Äp ÒÄp ê ò  p Ò p ê ò  P Ò P ê ò  0 ê ò Ù  ı  ê‡hÄi ôàCC·hàa(F8Ω¯µF % &   ê ò  0 ê ò·ha ø‡h i êpFIàB Ÿ‡ ò¿Û  (Ò— ø ˇ˜˛ˇ(F¯Ω  @ ¯µFF & '   ê ò    0 ê òeÛä ê ò·ha ø‡h i êxFIàB Ÿ‡ ò¿Û@ (Ò— ø ˇ˜˛ˇ0F¯Ω@ F»h@i  pGµF   # ì õ# CÒ C ì$ õdÛ‹C ìÒêS¯!0 úaÒêS¯!0hìõ# CÒ Cìõ#ÄCìÒêS¯!0ú`ΩµÜ∞F   #ììÒêS¯!0h ì õ# CÒ C ì õ#ÄCÒÄC ì õ√ÛÅC#± õ√ÛÅC+—”h€jìõ√ÛC{π õ# C ì
‡Siiìõ√ÛCπ õ# C ìÒêS¯!0 ú`∞Ω?µF   # $îîîD îÒêT¯!@ ù•` $î‘x,—ú$$îıktÎAdy4±,j–,–,8—8‡ øú$dîú$4îú$ÄÄ4îú$ÙÄdıÄdîú$4îıktÎA§x4±ú$ÙÄtıÄtî‡ú$@@4îıktÎA$y$±ú$  4îN‡ú$dîú$4îú$4îú$ÄÄ4îú$ÙÄdıÄdîú$Ù tı tîıktÎA§x,±ú$ÙÄtıÄtî!‡ú$dîú$Ù tı tîú$  4îıktÎA§xT±ú$ÄÄ4îú$ÙÄtıÄtî ø øÒêT¯!@ùÂ`$˙ÛTi§iCUi¨aú$ tÒ tî‘h§iù,C’h¨a $îıktÎA%xúeÛúTîıktÎAexúeÛŒ$îıktÎA•xúeÛœ4îıktÎA‰x,—% ‡ %úeÛQDîıktÎAeyúeÛìDîıktÎAÂàúeÛ
îıktÎAdy,—ú$ TÒ TîÒêT¯!@ù%`∞0Ω-ÈCà∞FFO 	»FOÙÄw  êê êıkp ÎE i≥ıkp ÎEiıkp ÎE¿àDIıkp ÎE¿à±˚Ü≤æB›>Fıkp ÎE¡àqCıkp ÎEa ‡&ıkp ÎEÄxP±ıkp ÎE¡àqCıkp ÎEaıkp ÎEiòaÛ êòfÛ‹@êıkp ÎEzòaÛ^pêÒê P¯% ôa‡x(	—ıkp ÎE¡hÒê P¯% AaÒê P¯%  hê Fˇ˜˛ˇô`Û]qëò  @ Ò @êò Ä@êÒê P¯% ô`‡x (d—ıkp ÎEÄx (]—ıkp ÎE i (V–ıkp ÎE@y0±(!–(–(9—‡ ø‡h¿jêıkp ÎE i¿¿ÛèòÄ≤@E
⁄ ò    0 ê‡hÄi ôC·hàa‡ ø`i iêıkp ÎE i¿¿ÛèòÄ≤@E⁄ ò Ä` ÒÄ` ê‡hÄi ôC·hàa ‡ ø øıkp ÎE äÉ≤ıkp ÎE*F¡h Fˇ˜˛ˇHF∞ΩËÉµBih í öaÛ í õBi`Ω µFFˇ˜˛ˇ(–  Ω  ¸ÁFHiÄh  Ä pG µFFˇ˜˛ˇ( — Ω  ¸ÁF   hPi híi@pGµF—¯Ã  h ê ò   ê ò   ê ò   ê ò    ê òΩFHi@ipGF»h@hpGµF $–« ÎówΩ”¯–` ø‡0h`d	¨B˘”FΩ8µF Fˇ˜˛ˇ ê ò ÙÄp ıÄp ê ô‘¯Ã `
 ˇ˜˛ˇ ò ÙÄp ê ô‘¯Ã ` ˇ˜˛ˇ 8ΩpµFF #   %’p%ïp@%ïÄ,—O†C%’r%p%Upù’Ä%r
‡Lπ%K %’r%p%UpOÙ†e’Ä”`ı ea !‡ıeÎAÒE¯!`ı0eÎAÒTE¯!`IUxçBÎÿıÄeUa@5¬¯ÃP !‡ı†eÎAÒêE¯!`IxçBÛÿ !‡ıÄUÎ6Ò–E¯!`IxçBÛÿı`e¬¯QpΩ  @¯µFF &‡h¿h ê ò  P ê ò Ä@ ê-— ò  P Ò P ê‡-π ò Ä@ ÒÄ@ ê ò·h»`2 ˇ˜˛ˇ0F¯Ω8µF !`iÅaI`iAa %‡Òê P¯%  h ê ò  @ ê ò Ä@ ÒÄ@ ê ò Ù @ êÒê P¯%  ô`m x®B·ÿ Fˇ˜˛ˇ! Fˇ˜˛ˇ8Ω-ÈAF &‡x∞π '  ›OÍÂxÎòxOÍ®Ò–U¯"¿ ø‡hÃ¯ P@	∏B¯” ø0FΩËÅ      Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637] ArmCC --c99 --split_sections --debug -c --gnu -o.\objects\usb_core.o --depend=.\objects\usb_core.d --cpu=Cortex-M3 --apcs=interwork -O0 --diag_suppress=9931 -I..\Source\APP -I..\Source\CORE -I..\Source\Driver -I..\Source\fat_fs\inc -I..\Source\HAL -I..\Source\fat_fs\inc -I..\Source\panle -I..\Source\STM32_USB_HOST_Library\Class\MSC\inc -I..\Source\STM32_USB_HOST_Library\Core\inc -I..\Source\STM32_USB_OTG_Driver\inc -I..\Source\STM32F10x_FWLib\inc -I..\Source\USER -I.\RTE\_PANLE -ID:\Program\keil5\ARM\PACK\Keil\STM32F1xx_DFP\2.3.0\Device\Include -ID:\Program\keil5\ARM\CMSIS\Include -D__UVISION_VERSION=526 -DSTM32F10X_CL -DSTM32F10X_CL -DUSE_STDPERIPH_DRIVER -DUSE_STM3210C_EVAL -DPANLE --omf_browse=.\objects\usb_core.crf ..\Source\STM32_USB_OTG_Driver\src\usb_core.c 0   ˇˇˇˇarmcc+ |  	
             4   A~é   0   ˇˇˇˇarmcc+ |  	
             l   AzÑÖÜáé   0   ˇˇˇˇarmcc+ |  	
             @   BzÑÖÜáàé 0   ˇˇˇˇarmcc+ |  	
             (   A{ÑÖÜáé   0   ˇˇˇˇarmcc+ |  	
             Ã   A|ÑÖÜé 0   ˇˇˇˇarmcc+ |  	
             &  AzÑÖé   0   ˇˇˇˇarmcc+ |  	
             "   A~é   0   ˇˇˇˇarmcc+ |  	
             "   A~é   0   ˇˇˇˇarmcc+ |  	
             P   AzÑÖÜáé   0   ˇˇˇˇarmcc+ |  	
             F   AzÑÖÜáé   0   ˇˇˇˇarmcc+ |  	
             N   AzÑÖÜáé   0   ˇˇˇˇarmcc+ |  	
                0   ˇˇˇˇarmcc+ |  	
                Aé   0   ˇˇˇˇarmcc+ |  	
                Aé   0   ˇˇˇˇarmcc+ |  	
                0   ˇˇˇˇarmcc+ |  	
                0   ˇˇˇˇarmcc+ |  	
             t   A|ÑÖé   0   ˇˇˇˇarmcc+ |  	
             0   A~é   0   ˇˇˇˇarmcc+ |  	
             \   A|ÑÖé   0   ˇˇˇˇarmcc+ |  	
             @   A|ÑÖé   0   ˇˇˇˇarmcc+ |  	
                A~é   0   ˇˇˇˇarmcc+ |  	
             ÷   AxÑÖÜáé   0   ˇˇˇˇarmcc+ |  	
                0   ˇˇˇˇarmcc+ |  	
                0   ˇˇˇˇarmcc+ |  	
             8  AyÑÖé}  0   ˇˇˇˇarmcc+ |  	
  $           ˆ  ByÑÖÜáàâéAqˆy0   ˇˇˇˇarmcc+ |  	
             Ç   A~ÑéAx~~   0   ˇˇˇˇarmcc+ |  	
             Z   A|Ñé 0   ˇˇˇˇarmcc+ |  	
             `   A|ÑÖé   ¿        
..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle                     ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     4       ?ê_USB_OTG_EnableCommonInt      4       ipdev :  !   Yint_mask ã  ëx     @       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     l       >¬{USB_OTG_CoreReset  Õ       l       ipdev :  Z   ___result Õ   !   Zstatus Õ   G   Ygreset @  ëhZcount _  4      å       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     @       >ç®USB_OTG_WritePacket Õ       @       ipdev :  í   isrc F     ich_ep_num @  l   ilen O  Y   ^__result Õ   P<Xstatus Õ   Vå   <   Zcount32b _  F   Zi _  3   Zfifo R           `       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     (       uvoid "√ >‡≈USB_OTG_ReadPacket …     (       ipdev :  F   idest F  3   ilen O      ^__result … P&Xi _  TXcount32b _  UXfifo R  V     T       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     Ã       >‘›USB_OTG_SelectCore Õ       Ã       ipdev :  >   icoreID Û       ^__result Õ   P Xi _  QTXbaseAddress _  SXstatus Õ   P
     P       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     &      >–√USB_OTG_CoreInit Õ       &      ipdev :  "   ^__result Õ   P§Xstatus Õ   UYusbcfg r  ëpYgccfg Ã  ëlYahbcfg Ï
  ëh     8       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     "       >∑≠USB_OTG_EnableGlobalInt Õ       "       ipdev :  "   ^__result Õ   P Xstatus Õ   PYahbcfg Ï
  ëx      8       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     "       >∏øUSB_OTG_DisableGlobalInt Õ       "       ipdev :  "   ^__result Õ   P Xstatus Õ   PYahbcfg Ï
  ëx     P       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     P       >––USB_OTG_FlushTxFifo Õ       P       ipdev :  >   inum _      ^__result Õ   PNXstatus Õ   VYgreset @  ëhXcount _  W
     @       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     F       >¬ÓUSB_OTG_FlushRxFifo Õ       F       ipdev :  "   ^__result Õ   PDXstatus Õ   UYgreset @  ëhXcount _  V   D       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     N       >≈åUSB_OTG_SetCurrentMode Õ       N       ipdev :  >   imode @      ^__result Õ   PLXstatus Õ   VYusbcfg r  ëh           ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle            >è™
USB_OTG_GetMode _             ipdev :     ^__result _  P
             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle            >ïµ	USB_OTG_IsDeviceMode @             ipdev :  6   ___result @  #              ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle            >ì¿	USB_OTG_IsHostMode @             ipdev :  6   ___result @  #                ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle            >ûÀ
USB_OTG_ReadCoreItr _             ipdev :     ^__result _  PXv _  P          ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle            >íŸ
USB_OTG_ReadOtgItr _             ipdev :     ^__result _  P   4       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     t       >µÙUSB_OTG_EnableHostInt Õ       t       ipdev :  "   ^__result Õ   PrXstatus Õ   UYintmsk ã  ëp            ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     0       >†•
USB_OTG_ReadHPRT0 _      0       ipdev :  "   ^__result _  P.Yhprt0 M&  ëx            ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     \       ?ò◊USB_OTG_DriveVbus     \       ipdev :  >   istate @      Yhprt0 M&  ëp             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     @       >†ƒ
USB_OTG_ResetPort _      @       ipdev :  "   ^__result _  P>Yhprt0 M&  ëp            ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle            ?úñUSB_OTG_InitFSLSPClkSel            ipdev :  5   ifreq @  "   Yhcfg ›"  ëx     h       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     ÷       >È‰USB_OTG_CoreInitHost Õ       ÷       ipdev :  "   ^__result Õ   P‘Xstatus Õ   VYnptxfifosize O  ëhYptxfifosize O  ëdYhcfg ›"  ë`Xi _  U           ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle            >ìÃ	USB_OTG_IsEvenFrame @             ipdev :     ^__result @  P              ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle            >†∑
USB_OTG_ReadHostAllChannels_intr _             ipdev :     ^__result _  P     à       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     8      >àŸUSB_OTG_HC_Init Õ       8      ipdev :  B   ihc_num @  /   ^__result Õ   P¥Xstatus Õ   PXintr_enable _  SYhcintmsk 8,  ëpYgintmsk ã  ëlYhcchar ^(  ëhYhcint a*  ëd     Ã       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     ˆ      >À«USB_OTG_HC_StartXfer Õ       ˆ      ipdev :  V   ihc_num @  8   ^__result Õ   PXstatus Õ   YYhcchar ^(  ë`Yhctsiz ˝*  ë\Yhnptxsts 8  ëPYhptxsts µ$  ëHYintmsk ã  ëDXlen_words O  XXnum_packets O  VPXmax_hc_pkt_count O  W      `       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     Ç       >‚ºUSB_OTG_HC_Halt Õ       Ç       ipdev :  N   ihc_num @  ;   ^__result Õ   P~Xstatus Õ   PYnptxsts 8  ëlYhptxsts µ$  ëdYhcchar ^(  ë`   P       ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     Z       >“„USB_OTG_HC_DoPing Õ       Z       ipdev :  6   ihc_num @  #   ^__result Õ   PXXstatus Õ   PYhcchar ^(  ëtYhctsiz ˝*  ëp          ..\Source\STM32_USB_OTG_Driver\src\usb_core.c Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle     `       ?ì˙USB_OTG_StopHost     `       ipdev :  "   Yhcchar ^(  ëpXi _  U      P    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c      |    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          ﬂ !,+ ∏    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          ˚ '
o!&'
}% ú    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          ´2
 ~$ ê    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          «'	~#    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          ﬁ
:,{7	,~7,~7"    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          √!-!  &&&-,-..&& t    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          ≠&- t    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          ø&- ú    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          –& '
}% ò    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          Ó&'
}% î    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          å !3. h    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          ™  p    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          µ, p    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          ¿& p    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          À h    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          Ÿ †    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          Ù',,,&,2 |    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          •      †    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          ◊" ",!" " Ñ    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          ƒ ,    p    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          ñ  Ù    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          ‰*%	"#! &&&& 2,}+" h    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          Ã, h    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          ∑ L   D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          Ÿ.(X&&&,&-5&	-(&&&&,--.&,'-&-g/(,.>>>\>>3-- 4   D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          «&&-ÇO-M> >-G,4, -	23Y	:"&	.	9",	-pf ¨    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          º,,.)'!*!"- à    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          „$,&-,, - †    D             ..\Source\STM32_USB_OTG_Driver\src\usb_core.c          ˙, , ,z+
          } ñ   4    }            4    P                } ñ   l    }        &   (    P           j    U           j    V                P   j    T                }    @    }        &   8    \           8    P           8    W            @    S            @    R            @    Q                P   @    T                }    (    }            (    R            (    Q                P   (    S                }    Ã    }                Q   Ã    T                P   Ã    R                } ññ   &   }                P   &   T                } ññ   "    }                P   "    Q                } ññ   "    }                P   "    Q                }    P    }                Q   P    U                P   P    T                } ññ   F    }                P   F    T                }    N    }                Q   N    U                P   N    T                } ññ                P       Q                } ñññ       }               P                P       R                } ñññ       }               P                P       R                } ññ                P       Q                } ññ                P       Q                } ññ   t    }                P   t    T                } ññ   0    }                P   0    Q                }    \    }                Q   \    T                P   \    U                } ññ   @    }                P   @    T                } ññ       }                Q                P                } ññ   ÷    }                 P   ÷    T                } ññ                P       Q                } ññ                P       Q                } ñññ   6   }6  8   }            8   Q                P   8   R                }        }   Ú   }<Ú  ˆ   }                Q   ˆ   U                P   ˆ   T                } ñññ       }   Ä    } Ä   Ç    }            Ç    Q                P   Ç    R                } ñññ   Z    }            Z    Q                P   Z    R                } ññ   `    }                P   `    T         __DATE__ "Mar 20 2019"  __TIME__ "14:58:49"  __STDC__ 1  __STDC_VERSION__ 199901L  __STDC_HOSTED__ 1  __STDC_ISO_10646__ 200607  __EDG__ 1  __EDG_VERSION__ 407  __EDG_SIZE_TYPE__ unsigned int  __EDG_PTRDIFF_TYPE__ int  __GNUC__ 4  __GNUC_STDC_INLINE__ 1  __GNUC_MINOR__ 7  __GNUC_PATCHLEVEL__ 0  __VERSION__ "4.7 (EDG gcc mode)"  __CHAR16_TYPE__ unsigned short  __CHAR32_TYPE__ unsigned int  __USER_LABEL_PREFIX__   __CHAR_UNSIGNED__ 1  __WCHAR_UNSIGNED__ 1  __SIZE_TYPE__ unsigned int  __PTRDIFF_TYPE__ int  __WCHAR_TYPE__ unsigned short  __WINT_TYPE__ unsigned short  __INTMAX_TYPE__ long long  __UINTMAX_TYPE__ unsigned long long  __sizeof_int 4  __sizeof_long 4  __sizeof_ptr 4  __ARMCC_VERSION 5060750  __TARGET_CPU_CORTEX_M3 1  __TARGET_FPU_SOFTVFP 1  __TARGET_FPU_SOFTVFP 1  __UVISION_VERSION 526  STM32F10X_CL 1  STM32F10X_CL 1  USE_STDPERIPH_DRIVER 1  USE_STM3210C_EVAL 1  PANLE 1  __CC_ARM 1  __arm 1  __arm__ 1  __TARGET_ARCH_7_M 1  __TARGET_ARCH_ARM 0  __TARGET_ARCH_THUMB 4  __TARGET_ARCH_A64 0  __TARGET_ARCH_AARCH32 1  __TARGET_PROFILE_M 1  __TARGET_FEATURE_HALFWORD 1  __TARGET_FEATURE_THUMB 1  __TARGET_FEATURE_MULTIPLY 1  __TARGET_FEATURE_DOUBLEWORD 1  __TARGET_FEATURE_DIVIDE 1  __TARGET_FEATURE_UNALIGNED 1  __TARGET_FEATURE_CLZ 1  __TARGET_FEATURE_DMB 1  __TARGET_FEATURE_EXTENSION_REGISTER_COUNT 0  __APCS_INTERWORK 1  __thumb 1  __thumb__ 1  __t32__ 1  __OPTIMISE_SPACE 1  __OPTIMIZE_SIZE__ 1  __OPTIMISE_LEVEL 0  __SOFTFP__ 1      &        ê  √   USB_OTG_WritePacket     %        d  Õ   USB_OTG_ReadPacket     %        X  √   USB_OTG_SelectCore     #        T  √   USB_OTG_CoreInit     *        <  √   USB_OTG_EnableGlobalInt     +        <  √   USB_OTG_DisableGlobalInt     &        T  √   USB_OTG_FlushTxFifo     &        D  √   USB_OTG_FlushRxFifo     )        H  √   USB_OTG_SetCurrentMode     "          √   USB_OTG_GetMode     '          √   USB_OTG_IsDeviceMode     %          √   USB_OTG_IsHostMode     &           √   USB_OTG_ReadCoreItr     %          √   USB_OTG_ReadOtgItr     (        8  √   USB_OTG_EnableHostInt     $        $  √   USB_OTG_ReadHPRT0     $          √   USB_OTG_DriveVbus     $        $  √   USB_OTG_ResetPort     *           √   USB_OTG_InitFSLSPClkSel     '        l  √   USB_OTG_CoreInitHost     &          √   USB_OTG_IsEvenFrame     3        $  √   USB_OTG_ReadHostAllChannels_intr     "        å  √   USB_OTG_HC_Init     '        –  √   USB_OTG_HC_StartXfer     "        d  √   USB_OTG_HC_Halt     $        T  √   USB_OTG_HC_DoPing     #          √   USB_OTG_StopHost        µ   ¥   ≥          D:\Program\keil5\ARM\ARMCC\Bin\..\include\stdint.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]          signed char short int long long unsigned char unsigned short unsigned int unsigned long long Pint8_t ê 8 Pint16_t ü 9 Pint32_t ® : Pint64_t Ø ; Puint8_t º > Puint16_t Õ ? Puint32_t ﬂ @ Puint64_t Ô A Pint_least8_t ê G Pint_least16_t ü H Pint_least32_t ® I Pint_least64_t Ø J Puint_least8_t º M Puint_least16_t Õ N Puint_least32_t ﬂ O Puint_least64_t Ô P Pint_fast8_t ® U Pint_fast16_t ® V Pint_fast32_t ® W Pint_fast64_t Ø X Puint_fast8_t ﬂ [ Puint_fast16_t ﬂ \ Puint_fast32_t ﬂ ] Puint_fast64_t Ô ^ Pintptr_t ® e Puintptr_t ﬂ f Pintmax_t Ø j!Puintmax_t Ô k!   T    J            D:\Program\keil5\ARM\ARMCC\Bin\..\include\  stdint.h      __stdint_h  __ARMCLIB_VERSION 5060037 __INT64 __int64 __INT64_C_SUFFIX__ ll __PASTE2(x,y) x ## y __PASTE(x,y) __PASTE2(x, y) __INT64_C(x) __ESCAPE__(__PASTE(x, __INT64_C_SUFFIX__)) __UINT64_C(x) __ESCAPE__(__PASTE(x ## u, __INT64_C_SUFFIX__)) __LONGLONG long long #__STDINT_DECLS  %__CLIBNS ,__CLIBNS  sINT8_MIN -128 tINT16_MIN -32768 uINT32_MIN (~0x7fffffff) vINT64_MIN __INT64_C(~0x7fffffffffffffff) yINT8_MAX 127 zINT16_MAX 32767 {INT32_MAX 2147483647 |INT64_MAX __INT64_C(9223372036854775807) UINT8_MAX 255 ÄUINT16_MAX 65535 ÅUINT32_MAX 4294967295u ÇUINT64_MAX __UINT64_C(18446744073709551615) áINT_LEAST8_MIN -128 àINT_LEAST16_MIN -32768 âINT_LEAST32_MIN (~0x7fffffff) äINT_LEAST64_MIN __INT64_C(~0x7fffffffffffffff) çINT_LEAST8_MAX 127 éINT_LEAST16_MAX 32767 èINT_LEAST32_MAX 2147483647 êINT_LEAST64_MAX __INT64_C(9223372036854775807) ìUINT_LEAST8_MAX 255 îUINT_LEAST16_MAX 65535 ïUINT_LEAST32_MAX 4294967295u ñUINT_LEAST64_MAX __UINT64_C(18446744073709551615) õINT_FAST8_MIN (~0x7fffffff) úINT_FAST16_MIN (~0x7fffffff) ùINT_FAST32_MIN (~0x7fffffff) ûINT_FAST64_MIN __INT64_C(~0x7fffffffffffffff) °INT_FAST8_MAX 2147483647 ¢INT_FAST16_MAX 2147483647 £INT_FAST32_MAX 2147483647 §INT_FAST64_MAX __INT64_C(9223372036854775807) ßUINT_FAST8_MAX 4294967295u ®UINT_FAST16_MAX 4294967295u ©UINT_FAST32_MAX 4294967295u ™UINT_FAST64_MAX __UINT64_C(18446744073709551615) ≤INTPTR_MIN INT32_MIN πINTPTR_MAX INT32_MAX ¿UINTPTR_MAX UINT32_MAX ∆INTMAX_MIN __ESCAPE__(~0x7fffffffffffffffll) …INTMAX_MAX __ESCAPE__(9223372036854775807ll) ÃUINTMAX_MAX __ESCAPE__(18446744073709551615ull) ’PTRDIFF_MIN INT32_MIN ÷PTRDIFF_MAX INT32_MAX ⁄SIG_ATOMIC_MIN (~0x7fffffff) €SIG_ATOMIC_MAX 2147483647 ·SIZE_MAX UINT32_MAX ÁWCHAR_MIN ËWCHAR_MAX ÓWCHAR_MIN 0 ÔWCHAR_MAX 65535 ÛWINT_MIN (~0x7fffffff) ÙWINT_MAX 2147483647 ˚INT8_C(x) (x) ¸INT16_C(x) (x) ˝INT32_C(x) (x) ˛INT64_C(x) __INT64_C(x) ÄUINT8_C(x) (x ## u) ÅUINT16_C(x) (x ## u) ÇUINT32_C(x) (x ## u) ÉUINT64_C(x) __UINT64_C(x) ÜINTMAX_C(x) __ESCAPE__(x ## ll) áUINTMAX_C(x) __ESCAPE__(x ## ull) ≤__INT64 ≥__LONGLONG        π   ∏   ∑   ∫   Ñ       
..\Source\CORE\core_cm3.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle         int t´ qITM_RxBuffer ≤ ;ä¥__get_BASEPRI  _  a__result _  Y__regBasePri _  P <ƒ¡__set_BASEPRI  $_  basePri Y__regBasePri _  P ;ÖŒ__get_PRIMASK  _  a__result _  Y__regPriMask _  P <ø€__set_PRIMASK  $_  priMask Y__regPriMask _  P ;ÑË__get_FAULTMASK  _  a__result _  Y__regFaultMask _  P <ƒı__set_FAULTMASK  $_  faultMask Y__regFaultMask _  P ;ÖÇ__get_CONTROL  _  a__result _  Y__regControl _  P <øè__set_CONTROL  $_  control Y__regControl _  P <úºNVIC_SetPriorityGrouping  $_  PriorityGroup \reg_value _  \PriorityGroupTmp _   ;”—NVIC_GetPriorityGrouping  _  a__result _   <ˆﬁNVIC_EnableIRQ  $Ù  IRQn  <öÎNVIC_DisableIRQ  $Ù  IRQn  ;÷˘NVIC_GetPendingIRQ  _  $Ù  IRQn a__result _   <˝ÜNVIC_SetPendingIRQ  $Ù  IRQn  <¶ìNVIC_ClearPendingIRQ  $Ù  IRQn  ;ﬁ°NVIC_GetActive  _  $Ù  IRQn a__result _   <í	≤NVIC_SetPriority  $Ù  IRQn $_  priority  ;Ã	…NVIC_GetPriority  _  $Ù  IRQn a__result _   ;Å‚NVIC_EncodePriority  _  $_  PriorityGroup $_  PreemptPriority $_  SubPriority a__result _  \PriorityGroupTmp _  \PreemptPriorityBits _  \SubPriorityBits _   <ÆÅNVIC_DecodePriority  $_  Priority $_  PriorityGroup $.pPreemptPriority $.pSubPriority \PriorityGroupTmp _  \PreemptPriorityBits _  \SubPriorityBits _   "_  ;ÌûSysTick_Config  _  $_  ticks a__result _   <á∑NVIC_SystemReset   ;ª‹ITM_SendChar  _  $_  ch a__result _   ;ÏÚITM_ReceiveChar  ´ a__result ´ \ch ´  ;îÜITM_CheckChar  ´ a__result ´  *÷Ñ¢V ISER # π_   RESERVED0 .# ”V ICER J#ÄÎ_   RSERVED1 `#†ÖV ISPR |#Äù_   RESERVED2 í#†∏V ICPR Ø#Ä–_   RESERVED3 ≈#†ÎV IABR ‚#ÄÉ_  7 RESERVED4 ¯#†ü\Ô IP #Ä∂_  É RESERVED5 *#STIR V#Ä t_  t@  PNVIC_Type ì*ˇtCPUID Ö	# ICSR V#VTOR V#AIRCR V#SCR V#CCR V#…\ SHP ¿#SHCSR V#$CFSR V#(HFSR V#,DFSR V#0MMFAR V#4BFAR V#8AFSR V#<≥Ö	 PFR *	#@DFR Ö	#HADR Ö	#L›Ö	 MMFR T	#PÚÖ	 ISAR i	#` _  t	PSCB_Type t∞*œCTRL V# LOAD V#VAL V#CALIB Ö	# PSysTick_Type ö	ÛSÄu8 \u16  
u32 V tO  *ÕÄ îÕ PORT 
# ¨_  ﬂ RESERVED0  
#ÄTER V#Ä’_   RESERVED1 J
#ÑTPR V#¿˛_   RESERVED2 s
#ƒTCR V#Äß_   RESERVED3 ú
#ÑIWR V#¯IRR V#¸IMCR V#ÄÈ_  * RESERVED4 ﬁ
#ÑLAR V#∞LSR V#¥û_   RESERVED5 #∏PID4 Ö	#–PID5 Ö	#‘PID6 Ö	#ÿPID7 Ö	#‹PID0 Ö	#‡PID1 Ö	#‰PID2 Ö	#ËPID3 Ö	#ÏCID0 Ö	#CID1 Ö	#ÙCID2 Ö	#¯CID3 Ö	#¸ t‰	PITM_Type 
º*ôRESERVED0 _  # ICTR Ö	#RESERVED1 _  # PInterruptType_Type ‚Ä*ÌDHCSR V# DCRSR V#DCRDR V#DEMCR V# PCoreDebug_Type 4Ú    t    h            ..\Source\CORE\ D:\Program\keil5\ARM\ARMCC\Bin\..\include\  core_cm3.h   stdint.h      __CM3_CORE_H__  T__CM3_CMSIS_VERSION_MAIN (0x01) U__CM3_CMSIS_VERSION_SUB (0x30) V__CM3_CMSIS_VERSION ((__CM3_CMSIS_VERSION_MAIN << 16) | __CM3_CMSIS_VERSION_SUB) X__CORTEX_M (0x03) Zq__I volatile const s__O volatile t__IO volatile ≥SCB_CPUID_IMPLEMENTER_Pos 24 ¥SCB_CPUID_IMPLEMENTER_Msk (0xFFul << SCB_CPUID_IMPLEMENTER_Pos) ∂SCB_CPUID_VARIANT_Pos 20 ∑SCB_CPUID_VARIANT_Msk (0xFul << SCB_CPUID_VARIANT_Pos) πSCB_CPUID_PARTNO_Pos 4 ∫SCB_CPUID_PARTNO_Msk (0xFFFul << SCB_CPUID_PARTNO_Pos) ºSCB_CPUID_REVISION_Pos 0 ΩSCB_CPUID_REVISION_Msk (0xFul << SCB_CPUID_REVISION_Pos) ¿SCB_ICSR_NMIPENDSET_Pos 31 ¡SCB_ICSR_NMIPENDSET_Msk (1ul << SCB_ICSR_NMIPENDSET_Pos) √SCB_ICSR_PENDSVSET_Pos 28 ƒSCB_ICSR_PENDSVSET_Msk (1ul << SCB_ICSR_PENDSVSET_Pos) ∆SCB_ICSR_PENDSVCLR_Pos 27 «SCB_ICSR_PENDSVCLR_Msk (1ul << SCB_ICSR_PENDSVCLR_Pos) …SCB_ICSR_PENDSTSET_Pos 26  SCB_ICSR_PENDSTSET_Msk (1ul << SCB_ICSR_PENDSTSET_Pos) ÃSCB_ICSR_PENDSTCLR_Pos 25 ÕSCB_ICSR_PENDSTCLR_Msk (1ul << SCB_ICSR_PENDSTCLR_Pos) œSCB_ICSR_ISRPREEMPT_Pos 23 –SCB_ICSR_ISRPREEMPT_Msk (1ul << SCB_ICSR_ISRPREEMPT_Pos) “SCB_ICSR_ISRPENDING_Pos 22 ”SCB_ICSR_ISRPENDING_Msk (1ul << SCB_ICSR_ISRPENDING_Pos) ’SCB_ICSR_VECTPENDING_Pos 12 ÷SCB_ICSR_VECTPENDING_Msk (0x1FFul << SCB_ICSR_VECTPENDING_Pos) ÿSCB_ICSR_RETTOBASE_Pos 11 ŸSCB_ICSR_RETTOBASE_Msk (1ul << SCB_ICSR_RETTOBASE_Pos) €SCB_ICSR_VECTACTIVE_Pos 0 ‹SCB_ICSR_VECTACTIVE_Msk (0x1FFul << SCB_ICSR_VECTACTIVE_Pos) ﬂSCB_VTOR_TBLBASE_Pos 29 ‡SCB_VTOR_TBLBASE_Msk (0x1FFul << SCB_VTOR_TBLBASE_Pos) ‚SCB_VTOR_TBLOFF_Pos 7 „SCB_VTOR_TBLOFF_Msk (0x3FFFFFul << SCB_VTOR_TBLOFF_Pos) ÊSCB_AIRCR_VECTKEY_Pos 16 ÁSCB_AIRCR_VECTKEY_Msk (0xFFFFul << SCB_AIRCR_VECTKEY_Pos) ÈSCB_AIRCR_VECTKEYSTAT_Pos 16 ÍSCB_AIRCR_VECTKEYSTAT_Msk (0xFFFFul << SCB_AIRCR_VECTKEYSTAT_Pos) ÏSCB_AIRCR_ENDIANESS_Pos 15 ÌSCB_AIRCR_ENDIANESS_Msk (1ul << SCB_AIRCR_ENDIANESS_Pos) ÔSCB_AIRCR_PRIGROUP_Pos 8 SCB_AIRCR_PRIGROUP_Msk (7ul << SCB_AIRCR_PRIGROUP_Pos) ÚSCB_AIRCR_SYSRESETREQ_Pos 2 ÛSCB_AIRCR_SYSRESETREQ_Msk (1ul << SCB_AIRCR_SYSRESETREQ_Pos) ıSCB_AIRCR_VECTCLRACTIVE_Pos 1 ˆSCB_AIRCR_VECTCLRACTIVE_Msk (1ul << SCB_AIRCR_VECTCLRACTIVE_Pos) ¯SCB_AIRCR_VECTRESET_Pos 0 ˘SCB_AIRCR_VECTRESET_Msk (1ul << SCB_AIRCR_VECTRESET_Pos) ¸SCB_SCR_SEVONPEND_Pos 4 ˝SCB_SCR_SEVONPEND_Msk (1ul << SCB_SCR_SEVONPEND_Pos) ˇSCB_SCR_SLEEPDEEP_Pos 2 ÄSCB_SCR_SLEEPDEEP_Msk (1ul << SCB_SCR_SLEEPDEEP_Pos) ÇSCB_SCR_SLEEPONEXIT_Pos 1 ÉSCB_SCR_SLEEPONEXIT_Msk (1ul << SCB_SCR_SLEEPONEXIT_Pos) ÜSCB_CCR_STKALIGN_Pos 9 áSCB_CCR_STKALIGN_Msk (1ul << SCB_CCR_STKALIGN_Pos) âSCB_CCR_BFHFNMIGN_Pos 8 äSCB_CCR_BFHFNMIGN_Msk (1ul << SCB_CCR_BFHFNMIGN_Pos) åSCB_CCR_DIV_0_TRP_Pos 4 çSCB_CCR_DIV_0_TRP_Msk (1ul << SCB_CCR_DIV_0_TRP_Pos) èSCB_CCR_UNALIGN_TRP_Pos 3 êSCB_CCR_UNALIGN_TRP_Msk (1ul << SCB_CCR_UNALIGN_TRP_Pos) íSCB_CCR_USERSETMPEND_Pos 1 ìSCB_CCR_USERSETMPEND_Msk (1ul << SCB_CCR_USERSETMPEND_Pos) ïSCB_CCR_NONBASETHRDENA_Pos 0 ñSCB_CCR_NONBASETHRDENA_Msk (1ul << SCB_CCR_NONBASETHRDENA_Pos) ôSCB_SHCSR_USGFAULTENA_Pos 18 öSCB_SHCSR_USGFAULTENA_Msk (1ul << SCB_SHCSR_USGFAULTENA_Pos) úSCB_SHCSR_BUSFAULTENA_Pos 17 ùSCB_SHCSR_BUSFAULTENA_Msk (1ul << SCB_SHCSR_BUSFAULTENA_Pos) üSCB_SHCSR_MEMFAULTENA_Pos 16 †SCB_SHCSR_MEMFAULTENA_Msk (1ul << SCB_SHCSR_MEMFAULTENA_Pos) ¢SCB_SHCSR_SVCALLPENDED_Pos 15 £SCB_SHCSR_SVCALLPENDED_Msk (1ul << SCB_SHCSR_SVCALLPENDED_Pos) •SCB_SHCSR_BUSFAULTPENDED_Pos 14 ¶SCB_SHCSR_BUSFAULTPENDED_Msk (1ul << SCB_SHCSR_BUSFAULTPENDED_Pos) ®SCB_SHCSR_MEMFAULTPENDED_Pos 13 ©SCB_SHCSR_MEMFAULTPENDED_Msk (1ul << SCB_SHCSR_MEMFAULTPENDED_Pos) ´SCB_SHCSR_USGFAULTPENDED_Pos 12 ¨SCB_SHCSR_USGFAULTPENDED_Msk (1ul << SCB_SHCSR_USGFAULTPENDED_Pos) ÆSCB_SHCSR_SYSTICKACT_Pos 11 ØSCB_SHCSR_SYSTICKACT_Msk (1ul << SCB_SHCSR_SYSTICKACT_Pos) ±SCB_SHCSR_PENDSVACT_Pos 10 ≤SCB_SHCSR_PENDSVACT_Msk (1ul << SCB_SHCSR_PENDSVACT_Pos) ¥SCB_SHCSR_MONITORACT_Pos 8 µSCB_SHCSR_MONITORACT_Msk (1ul << SCB_SHCSR_MONITORACT_Pos) ∑SCB_SHCSR_SVCALLACT_Pos 7 ∏SCB_SHCSR_SVCALLACT_Msk (1ul << SCB_SHCSR_SVCALLACT_Pos) ∫SCB_SHCSR_USGFAULTACT_Pos 3 ªSCB_SHCSR_USGFAULTACT_Msk (1ul << SCB_SHCSR_USGFAULTACT_Pos) ΩSCB_SHCSR_BUSFAULTACT_Pos 1 æSCB_SHCSR_BUSFAULTACT_Msk (1ul << SCB_SHCSR_BUSFAULTACT_Pos) ¿SCB_SHCSR_MEMFAULTACT_Pos 0 ¡SCB_SHCSR_MEMFAULTACT_Msk (1ul << SCB_SHCSR_MEMFAULTACT_Pos) ƒSCB_CFSR_USGFAULTSR_Pos 16 ≈SCB_CFSR_USGFAULTSR_Msk (0xFFFFul << SCB_CFSR_USGFAULTSR_Pos) «SCB_CFSR_BUSFAULTSR_Pos 8 »SCB_CFSR_BUSFAULTSR_Msk (0xFFul << SCB_CFSR_BUSFAULTSR_Pos)  SCB_CFSR_MEMFAULTSR_Pos 0 ÀSCB_CFSR_MEMFAULTSR_Msk (0xFFul << SCB_CFSR_MEMFAULTSR_Pos) ŒSCB_HFSR_DEBUGEVT_Pos 31 œSCB_HFSR_DEBUGEVT_Msk (1ul << SCB_HFSR_DEBUGEVT_Pos) —SCB_HFSR_FORCED_Pos 30 “SCB_HFSR_FORCED_Msk (1ul << SCB_HFSR_FORCED_Pos) ‘SCB_HFSR_VECTTBL_Pos 1 ’SCB_HFSR_VECTTBL_Msk (1ul << SCB_HFSR_VECTTBL_Pos) ÿSCB_DFSR_EXTERNAL_Pos 4 ŸSCB_DFSR_EXTERNAL_Msk (1ul << SCB_DFSR_EXTERNAL_Pos) €SCB_DFSR_VCATCH_Pos 3 ‹SCB_DFSR_VCATCH_Msk (1ul << SCB_DFSR_VCATCH_Pos) ﬁSCB_DFSR_DWTTRAP_Pos 2 ﬂSCB_DFSR_DWTTRAP_Msk (1ul << SCB_DFSR_DWTTRAP_Pos) ·SCB_DFSR_BKPT_Pos 1 ‚SCB_DFSR_BKPT_Msk (1ul << SCB_DFSR_BKPT_Pos) ‰SCB_DFSR_HALTED_Pos 0 ÂSCB_DFSR_HALTED_Msk (1ul << SCB_DFSR_HALTED_Pos) ˆSysTick_CTRL_COUNTFLAG_Pos 16 ˜SysTick_CTRL_COUNTFLAG_Msk (1ul << SysTick_CTRL_COUNTFLAG_Pos) ˘SysTick_CTRL_CLKSOURCE_Pos 2 ˙SysTick_CTRL_CLKSOURCE_Msk (1ul << SysTick_CTRL_CLKSOURCE_Pos) ¸SysTick_CTRL_TICKINT_Pos 1 ˝SysTick_CTRL_TICKINT_Msk (1ul << SysTick_CTRL_TICKINT_Pos) ˇSysTick_CTRL_ENABLE_Pos 0 ÄSysTick_CTRL_ENABLE_Msk (1ul << SysTick_CTRL_ENABLE_Pos) ÉSysTick_LOAD_RELOAD_Pos 0 ÑSysTick_LOAD_RELOAD_Msk (0xFFFFFFul << SysTick_LOAD_RELOAD_Pos) áSysTick_VAL_CURRENT_Pos 0 àSysTick_VAL_CURRENT_Msk (0xFFFFFFul << SysTick_VAL_CURRENT_Pos) ãSysTick_CALIB_NOREF_Pos 31 åSysTick_CALIB_NOREF_Msk (1ul << SysTick_CALIB_NOREF_Pos) éSysTick_CALIB_SKEW_Pos 30 èSysTick_CALIB_SKEW_Msk (1ul << SysTick_CALIB_SKEW_Pos) ëSysTick_CALIB_TENMS_Pos 0 íSysTick_CALIB_TENMS_Msk (0xFFFFFFul << SysTick_VAL_CURRENT_Pos) øITM_TPR_PRIVMASK_Pos 0 ¿ITM_TPR_PRIVMASK_Msk (0xFul << ITM_TPR_PRIVMASK_Pos) √ITM_TCR_BUSY_Pos 23 ƒITM_TCR_BUSY_Msk (1ul << ITM_TCR_BUSY_Pos) ∆ITM_TCR_ATBID_Pos 16 «ITM_TCR_ATBID_Msk (0x7Ful << ITM_TCR_ATBID_Pos) …ITM_TCR_TSPrescale_Pos 8  ITM_TCR_TSPrescale_Msk (3ul << ITM_TCR_TSPrescale_Pos) ÃITM_TCR_SWOENA_Pos 4 ÕITM_TCR_SWOENA_Msk (1ul << ITM_TCR_SWOENA_Pos) œITM_TCR_DWTENA_Pos 3 –ITM_TCR_DWTENA_Msk (1ul << ITM_TCR_DWTENA_Pos) “ITM_TCR_SYNCENA_Pos 2 ”ITM_TCR_SYNCENA_Msk (1ul << ITM_TCR_SYNCENA_Pos) ’ITM_TCR_TSENA_Pos 1 ÷ITM_TCR_TSENA_Msk (1ul << ITM_TCR_TSENA_Pos) ÿITM_TCR_ITMENA_Pos 0 ŸITM_TCR_ITMENA_Msk (1ul << ITM_TCR_ITMENA_Pos) ‹ITM_IWR_ATVALIDM_Pos 0 ›ITM_IWR_ATVALIDM_Msk (1ul << ITM_IWR_ATVALIDM_Pos) ‡ITM_IRR_ATREADYM_Pos 0 ·ITM_IRR_ATREADYM_Msk (1ul << ITM_IRR_ATREADYM_Pos) ‰ITM_IMCR_INTEGRATION_Pos 0 ÂITM_IMCR_INTEGRATION_Msk (1ul << ITM_IMCR_INTEGRATION_Pos) ËITM_LSR_ByteAcc_Pos 2 ÈITM_LSR_ByteAcc_Msk (1ul << ITM_LSR_ByteAcc_Pos) ÎITM_LSR_Access_Pos 1 ÏITM_LSR_Access_Msk (1ul << ITM_LSR_Access_Pos) ÓITM_LSR_Present_Pos 0 ÔITM_LSR_Present_Msk (1ul << ITM_LSR_Present_Pos) ÉInterruptType_ICTR_INTLINESNUM_Pos 0 ÑInterruptType_ICTR_INTLINESNUM_Msk (0x1Ful << InterruptType_ICTR_INTLINESNUM_Pos) áInterruptType_ACTLR_DISFOLD_Pos 2 àInterruptType_ACTLR_DISFOLD_Msk (1ul << InterruptType_ACTLR_DISFOLD_Pos) äInterruptType_ACTLR_DISDEFWBUF_Pos 1 ãInterruptType_ACTLR_DISDEFWBUF_Msk (1ul << InterruptType_ACTLR_DISDEFWBUF_Pos) çInterruptType_ACTLR_DISMCYCINT_Pos 0 éInterruptType_ACTLR_DISMCYCINT_Msk (1ul << InterruptType_ACTLR_DISMCYCINT_Pos) ıCoreDebug_DHCSR_DBGKEY_Pos 16 ˆCoreDebug_DHCSR_DBGKEY_Msk (0xFFFFul << CoreDebug_DHCSR_DBGKEY_Pos) ¯CoreDebug_DHCSR_S_RESET_ST_Pos 25 ˘CoreDebug_DHCSR_S_RESET_ST_Msk (1ul << CoreDebug_DHCSR_S_RESET_ST_Pos) ˚CoreDebug_DHCSR_S_RETIRE_ST_Pos 24 ¸CoreDebug_DHCSR_S_RETIRE_ST_Msk (1ul << CoreDebug_DHCSR_S_RETIRE_ST_Pos) ˛CoreDebug_DHCSR_S_LOCKUP_Pos 19 ˇCoreDebug_DHCSR_S_LOCKUP_Msk (1ul << CoreDebug_DHCSR_S_LOCKUP_Pos) ÅCoreDebug_DHCSR_S_SLEEP_Pos 18 ÇCoreDebug_DHCSR_S_SLEEP_Msk (1ul << CoreDebug_DHCSR_S_SLEEP_Pos) ÑCoreDebug_DHCSR_S_HALT_Pos 17 ÖCoreDebug_DHCSR_S_HALT_Msk (1ul << CoreDebug_DHCSR_S_HALT_Pos) áCoreDebug_DHCSR_S_REGRDY_Pos 16 àCoreDebug_DHCSR_S_REGRDY_Msk (1ul << CoreDebug_DHCSR_S_REGRDY_Pos) äCoreDebug_DHCSR_C_SNAPSTALL_Pos 5 ãCoreDebug_DHCSR_C_SNAPSTALL_Msk (1ul << CoreDebug_DHCSR_C_SNAPSTALL_Pos) çCoreDebug_DHCSR_C_MASKINTS_Pos 3 éCoreDebug_DHCSR_C_MASKINTS_Msk (1ul << CoreDebug_DHCSR_C_MASKINTS_Pos) êCoreDebug_DHCSR_C_STEP_Pos 2 ëCoreDebug_DHCSR_C_STEP_Msk (1ul << CoreDebug_DHCSR_C_STEP_Pos) ìCoreDebug_DHCSR_C_HALT_Pos 1 îCoreDebug_DHCSR_C_HALT_Msk (1ul << CoreDebug_DHCSR_C_HALT_Pos) ñCoreDebug_DHCSR_C_DEBUGEN_Pos 0 óCoreDebug_DHCSR_C_DEBUGEN_Msk (1ul << CoreDebug_DHCSR_C_DEBUGEN_Pos) öCoreDebug_DCRSR_REGWnR_Pos 16 õCoreDebug_DCRSR_REGWnR_Msk (1ul << CoreDebug_DCRSR_REGWnR_Pos) ùCoreDebug_DCRSR_REGSEL_Pos 0 ûCoreDebug_DCRSR_REGSEL_Msk (0x1Ful << CoreDebug_DCRSR_REGSEL_Pos) °CoreDebug_DEMCR_TRCENA_Pos 24 ¢CoreDebug_DEMCR_TRCENA_Msk (1ul << CoreDebug_DEMCR_TRCENA_Pos) §CoreDebug_DEMCR_MON_REQ_Pos 19 •CoreDebug_DEMCR_MON_REQ_Msk (1ul << CoreDebug_DEMCR_MON_REQ_Pos) ßCoreDebug_DEMCR_MON_STEP_Pos 18 ®CoreDebug_DEMCR_MON_STEP_Msk (1ul << CoreDebug_DEMCR_MON_STEP_Pos) ™CoreDebug_DEMCR_MON_PEND_Pos 17 ´CoreDebug_DEMCR_MON_PEND_Msk (1ul << CoreDebug_DEMCR_MON_PEND_Pos) ≠CoreDebug_DEMCR_MON_EN_Pos 16 ÆCoreDebug_DEMCR_MON_EN_Msk (1ul << CoreDebug_DEMCR_MON_EN_Pos) ∞CoreDebug_DEMCR_VC_HARDERR_Pos 10 ±CoreDebug_DEMCR_VC_HARDERR_Msk (1ul << CoreDebug_DEMCR_VC_HARDERR_Pos) ≥CoreDebug_DEMCR_VC_INTERR_Pos 9 ¥CoreDebug_DEMCR_VC_INTERR_Msk (1ul << CoreDebug_DEMCR_VC_INTERR_Pos) ∂CoreDebug_DEMCR_VC_BUSERR_Pos 8 ∑CoreDebug_DEMCR_VC_BUSERR_Msk (1ul << CoreDebug_DEMCR_VC_BUSERR_Pos) πCoreDebug_DEMCR_VC_STATERR_Pos 7 ∫CoreDebug_DEMCR_VC_STATERR_Msk (1ul << CoreDebug_DEMCR_VC_STATERR_Pos) ºCoreDebug_DEMCR_VC_CHKERR_Pos 6 ΩCoreDebug_DEMCR_VC_CHKERR_Msk (1ul << CoreDebug_DEMCR_VC_CHKERR_Pos) øCoreDebug_DEMCR_VC_NOCPERR_Pos 5 ¿CoreDebug_DEMCR_VC_NOCPERR_Msk (1ul << CoreDebug_DEMCR_VC_NOCPERR_Pos) ¬CoreDebug_DEMCR_VC_MMERR_Pos 4 √CoreDebug_DEMCR_VC_MMERR_Msk (1ul << CoreDebug_DEMCR_VC_MMERR_Pos) ≈CoreDebug_DEMCR_VC_CORERESET_Pos 0 ∆CoreDebug_DEMCR_VC_CORERESET_Msk (1ul << CoreDebug_DEMCR_VC_CORERESET_Pos) ÀSCS_BASE (0xE000E000) ÃITM_BASE (0xE0000000) ÕCoreDebug_BASE (0xE000EDF0) ŒSysTick_BASE (SCS_BASE + 0x0010) œNVIC_BASE (SCS_BASE + 0x0100) –SCB_BASE (SCS_BASE + 0x0D00) “InterruptType ((InterruptType_Type *) SCS_BASE) ”SCB ((SCB_Type *) SCB_BASE) ‘SysTick ((SysTick_Type *) SysTick_BASE) ’NVIC ((NVIC_Type *) NVIC_BASE) ÷ITM ((ITM_Type *) ITM_BASE) ◊CoreDebug ((CoreDebug_Type *) CoreDebug_BASE) Ê__ASM __asm Á__INLINE __inline ˝__enable_fault_irq __enable_fiq ˛__disable_fault_irq __disable_fiq Ä__NOP __nop Å__WFI __wfi Ç__WFE __wfe É__SEV __sev Ñ__ISB() __isb(0) Ö__DSB() __dsb(0) Ü__DMB() __dmb(0) á__REV __rev à__RBIT __rbit â__LDREXB(ptr) ((unsigned char ) __ldrex(ptr)) ä__LDREXH(ptr) ((unsigned short) __ldrex(ptr)) ã__LDREXW(ptr) ((unsigned int ) __ldrex(ptr)) å__STREXB(value,ptr) __strex(value, ptr) ç__STREXH(value,ptr) __strex(value, ptr) é__STREXW(value,ptr) __strex(value, ptr) ´__CLREX __clrex œITM_RXBUFFER_EMPTY 0x5AA55AA5          à  ∂   ITM_RxBuffer         æ   Ω   º   ø   Ã        
..\Source\USER\system_stm32f10x.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle         qSystemCoreClock _       D    9            ..\Source\USER\  system_stm32f10x.h      "__SYSTEM_STM32F10X_H      "        –   ≥   SystemCoreClock          √   ¬   ¡   %       
..\Source\USER\stm32f10x.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle         Ù	IRQn NonMaskableInt_IRQn rMemoryManagement_IRQn tBusFault_IRQn uUsageFault_IRQn vSVCall_IRQn {DebugMonitor_IRQn |PendSV_IRQn ~SysTick_IRQn WWDG_IRQn  PVD_IRQn TAMPER_IRQn RTC_IRQn FLASH_IRQn RCC_IRQn EXTI0_IRQn EXTI1_IRQn EXTI2_IRQn EXTI3_IRQn 	EXTI4_IRQn 
DMA1_Channel1_IRQn DMA1_Channel2_IRQn DMA1_Channel3_IRQn DMA1_Channel4_IRQn DMA1_Channel5_IRQn DMA1_Channel6_IRQn DMA1_Channel7_IRQn ADC1_2_IRQn CAN1_TX_IRQn CAN1_RX0_IRQn CAN1_RX1_IRQn CAN1_SCE_IRQn EXTI9_5_IRQn TIM1_BRK_IRQn TIM1_UP_IRQn TIM1_TRG_COM_IRQn TIM1_CC_IRQn TIM2_IRQn TIM3_IRQn TIM4_IRQn I2C1_EV_IRQn I2C1_ER_IRQn  I2C2_EV_IRQn !I2C2_ER_IRQn "SPI1_IRQn #SPI2_IRQn $USART1_IRQn %USART2_IRQn &USART3_IRQn 'EXTI15_10_IRQn (RTCAlarm_IRQn )OTG_FS_WKUP_IRQn *TIM5_IRQn 2SPI3_IRQn 3UART4_IRQn 4UART5_IRQn 5TIM6_IRQn 6TIM7_IRQn 7DMA2_Channel1_IRQn 8DMA2_Channel2_IRQn 9DMA2_Channel3_IRQn :DMA2_Channel4_IRQn ;DMA2_Channel5_IRQn <ETH_IRQn =ETH_WKUP_IRQn >CAN2_TX_IRQn ?CAN2_RX0_IRQn ¿ CAN2_RX1_IRQn ¡ CAN2_SCE_IRQn ¬ OTG_FS_IRQn √  PIRQn_Type ¨ ÿPs32 "  ÁPs16   ËPs8   È"  Psc32 /Î  Psc16 BÏ  Psc8 UÌt"  Pvs32 gÔt  Pvs16 zt  Pvs8 çÒt/Pvsc32 üÛtBPvsc16 ±ÙtUPvsc8 √ıPu32 _  ˜Pu16 O  ¯Pu8 @  ˘_  Puc32 ˝˚O  Puc16 ¸@  Puc8 #˝t_  Pvu32 5ˇtO  Pvu16 HÄt@  Pvu8 [Åt˝Pvuc32 mÉtPvuc16 Ñt#Pvuc8 ëÖ∑RESET  SET  PFlagStatus ¢á(PITStatus ¢á4ıDISABLE  ENABLE  PFunctionalState €â/¶ERROR  SUCCESS  PErrorStatus å,*≠PSR 5# CR1 5#CR2 5#SMPR1 5#SMPR2 5#JOFR1 5#JOFR2 5#JOFR3 5#JOFR4 5# HTR 5#$LTR 5#(SQR1 5#,SQR2 5#0SQR3 5#4JSQR 5#8JDR1 5#<JDR2 5#@JDR3 5#DJDR4 5#HDR 5#L PADC_TypeDef :¥*è¿RESERVED0 _  # DR1 H#RESERVED1 O  #DR2 H#RESERVED2 O  #
DR3 H#RESERVED3 O  #DR4 H#RESERVED4 O  #DR5 H#RESERVED5 O  #DR6 H#RESERVED6 O  #DR7 H#RESERVED7 O  #DR8 H# RESERVED8 O  #"DR9 H#$RESERVED9 O  #&DR10 H#(RESERVED10 O  #*RTCCR H#,RESERVED11 O  #.CR H#0RESERVED12 O  #2CSR H#4‹O   RESERVED13 —	#6DR11 H#@RESERVED14 O  #BDR12 H#DRESERVED15 O  #FDR13 H#HRESERVED16 O  #JDR14 H#LRESERVED17 O  #NDR15 H#PRESERVED18 O  #RDR16 H#TRESERVED19 O  #VDR17 H#XRESERVED20 O  #ZDR18 H#\RESERVED21 O  #^DR19 H#`RESERVED22 O  #bDR20 H#dRESERVED23 O  #fDR21 H#hRESERVED24 O  #jDR22 H#lRESERVED25 O  #nDR23 H#pRESERVED26 O  #rDR24 H#tRESERVED27 O  #vDR25 H#xRESERVED28 O  #zDR26 H#|RESERVED29 O  #~DR27 H#ÄRESERVED30 O  #ÇDR28 H#ÑRESERVED31 O  #ÜDR29 H#àRESERVED32 O  #äDR30 H#åRESERVED33 O  #éDR31 H#êRESERVED34 O  #íDR32 H#îRESERVED35 O  #ñDR33 H#òRESERVED36 O  #öDR34 H#úRESERVED37 O  #ûDR35 H#†RESERVED38 O  #¢DR36 H#§RESERVED39 O  #¶DR37 H#®RESERVED40 O  #™DR38 H#¨RESERVED41 O  #ÆDR39 H#∞RESERVED42 O  #≤DR40 H#¥RESERVED43 O  #∂DR41 H#∏RESERVED44 O  #∫DR42 H#ºRESERVED45 O  #æ PBKP_TypeDef Aó*◊TIR 5# TDTR 5#TDLR 5#TDHR 5# PCAN_TxMailBox_TypeDef #£*©RIR 5# RDTR 5#RDLR 5#RDHR 5# PCAN_FIFOMailBox_TypeDef uØ*‰FR1 5# FR2 5# PCAN_FilterRegister_TypeDef …π*ï!†MCR 5# MSR 5#TSR 5#RF0R 5#RF1R 5#IER 5#ESR 5#BTR 5#Ò_  W RESERVED0 f# ãW sTxMailBox Ç#Äß© sFIFOMailBox û#∞«_   RESERVED1 º#–FMR 5#ÄFM1R 5#ÑRESERVED2 _  #àFS1R 5#åRESERVED3 _  #êFFA1R 5#îRESERVED4 _  #òFA1R 5#ú· _   RESERVED5 V#†¸ ‰ sFilterRegister s#¿ PCAN_TypeDef €*˝!CFGR 5# OAR 5#PRES 5#ESR 5#CSR 5#TXD 5#RXD 5# PCEC_TypeDef ©È*€"DR 5# IDR [#RESERVED0 @  #RESERVED1 O  #CR 5# PCRC_TypeDef ˆ*©$4CR 5# SWTRIGR 5#DHR12R1 5#DHR12L1 5#DHR8R1 5#DHR12R2 5#DHR12L2 5#DHR8R2 5#DHR12RD 5# DHR12LD 5#$DHR8RD 5#(DOR1 5#,DOR2 5#0 PDAC_TypeDef oé*⁄$IDCODE 5# CR 5# PDBGMCU_TypeDef =ò*¶%CCR 5# CNDTR 5#CPAR 5#CMAR 5# PDMA_Channel_TypeDef q§*ﬁ%ISR 5# IFCR 5# PDMA_TypeDef ¬™*Á.ÿ MACCR 5# MACFFR 5#MACHTHR 5#MACHTLR 5#MACMIIAR 5#MACMIIDR 5#MACFCR 5#MACVLANTR 5#˙&_   RESERVED0 o# MACRWUFFR 5#(MACPMTCSR 5#,∏'_   RESERVED1 ≠#0MACSR 5#8MACIMR 5#<MACA0HR 5#@MACA0LR 5#DMACA1HR 5#HMACA1LR 5#LMACA2HR 5#PMACA2LR 5#TMACA3HR 5#XMACA3LR 5#\Á(_  ' RESERVED2 \#`MMCCR 5#ÄMMCRIR 5#ÑMMCTIR 5#àMMCRIMR 5#åMMCTIMR 5#êœ)_   RESERVED3 ƒ#îMMCTGFSCCR 5#ÃMMCTGFMSCCR 5#–ì*_   RESERVED4 #‘MMCTGFCR 5#Ë¡*_  	 RESERVED5 6#ÏMMCRFCECR 5#îMMCRFAECR 5#òÇ+_  	 RESERVED6 w#úMMCRGUFCR 5#ƒ≤+_  Õ RESERVED7 ¶#»PTPTSCR 5#ÄPTPSSIR 5#ÑPTPTSHR 5#àPTPTSLR 5#åPTPTSHUR 5#êPTPTSLUR 5#îPTPTSAR 5#òPTPTTHR 5#úPTPTTLR 5#†‚,_  ∂ RESERVED8 V#§DMABMR 5#Ä DMATPDR 5#Ñ DMARPDR 5#à DMARDLAR 5#å DMATDLAR 5#ê DMASR 5#î DMAOMR 5#ò DMAIER 5#ú DMAMFBOCR 5#† é._   RESERVED9 #§ DMACHTDR 5#» DMACHRDR 5#Ã DMACHTBAR 5#– DMACHRBAR 5#‘  PETH_TypeDef ÚÒ*≈/IMR 5# EMR 5#RTSR 5#FTSR 5#SWIER 5#PR 5# PEXTI_TypeDef {ˇ* 0$ACR 5# KEYR 5#OPTKEYR 5#SR 5#CR 5#AR 5#RESERVED 5#OBR 5#WRPR 5#  PFLASH_TypeDef ⁄ò*∆1RDP H# USER H#Data0 H#Data1 H#WRP0 H#WRP1 H#
WRP2 H#WRP3 H# POB_TypeDef `®*Û1 Ê15 BTCR ›#  PFSMC_Bank1_TypeDef Ÿ±*®2õ25 BWTR #  PFSMC_Bank1E_TypeDef ∫*ö3PCR2 5# SR2 5#PMEM2 5#PATT2 5#RESERVED0 _  #ECCR2 5# PFSMC_Bank2_TypeDef D»*ã4PCR3 5# SR3 5#PMEM3 5#PATT3 5#RESERVED0 _  #ECCR3 5# PFSMC_Bank3_TypeDef µ÷*Ë4PCR4 5# SR4 5#PMEM4 5#PATT4 5#PIO4 5# PFSMC_Bank4_TypeDef &„*◊5CRL 5# CRH 5#IDR 5#ODR 5#BSRR 5#BRR 5#LCKR 5# PGPIO_TypeDef ÉÚ*¿6 EVCR 5# MAPR 5#ë65 EXTICR #RESERVED0 _  #MAPR2 5# PAFIO_TypeDef Ïˇ*Î8$CR1 H# RESERVED0 O  #CR2 H#RESERVED1 O  #OAR1 H#RESERVED2 O  #
OAR2 H#RESERVED3 O  #DR H#RESERVED4 O  #SR1 H#RESERVED5 O  #SR2 H#RESERVED6 O  #CCR H#RESERVED7 O  #TRISE H# RESERVED8 O  #" PI2C_TypeDef Uò*≠9KR 5# PR 5#RLR 5#SR 5# PIWDG_TypeDef §*‹9CR 5# CSR 5# PPWR_TypeDef ¬Æ*ï;0CR 5# CFGR 5#CIR 5#APB2RSTR 5#APB1RSTR 5#AHBENR 5#APB2ENR 5#APB1ENR 5#BDCR 5# CSR 5#$AHBRSTR 5#(CFGR2 5#, PRCC_TypeDef  *‚=(CRH H# RESERVED0 O  #CRL H#RESERVED1 O  #PRLH H#RESERVED2 O  #
PRLL H#RESERVED3 O  #DIVH H#RESERVED4 O  #DIVL H#RESERVED5 O  #CNTH H#RESERVED6 O  #CNTL H#RESERVED7 O  #ALRH H# RESERVED8 O  #"ALRL H#$RESERVED9 O  #& PRTC_TypeDef ©Ê*ö@ÑPOWER 5# CLKCR 5#ARG 5#CMD 5#RESPCMD m#RESP1 m#RESP2 m#RESP3 m#RESP4 m# DTIMER 5#$DLEN 5#(DCTRL 5#,DCOUNT m#0STA m#4ICR 5#8MASK 5#<–?_   RESERVED0 ≈#@FIFOCNT m#H˚?_   RESERVED1 #LFIFO 5#Ä PSDIO_TypeDef ˆÇ	*ŒB$CR1 H# RESERVED0 O  #CR2 H#RESERVED1 O  #SR H#RESERVED2 O  #
DR H#RESERVED3 O  #CRCPR H#RESERVED4 O  #RXCRCR H#RESERVED5 O  #TXCRCR H#RESERVED6 O  #I2SCFGR H#RESERVED7 O  #I2SPR H# RESERVED8 O  #" PSPI_TypeDef / ú	*’GPCR1 H# RESERVED0 O  #CR2 H#RESERVED1 O  #SMCR H#RESERVED2 O  #
DIER H#RESERVED3 O  #SR H#RESERVED4 O  #EGR H#RESERVED5 O  #CCMR1 H#RESERVED6 O  #CCMR2 H#RESERVED7 O  #CCER H# RESERVED8 O  #"CNT H#$RESERVED9 O  #&PSC H#(RESERVED10 O  #*ARR H#,RESERVED11 O  #.RCR H#0RESERVED12 O  #2CCR1 H#4RESERVED13 O  #6CCR2 H#8RESERVED14 O  #:CCR3 H#<RESERVED15 O  #>CCR4 H#@RESERVED16 O  #BBDTR H#DRESERVED17 O  #FDCR H#HRESERVED18 O  #JDMAR H#LRESERVED19 O  #N PTIM_TypeDef b!Ã	*øISR H# RESERVED0 O  #DR H#RESERVED1 O  #BRR H#RESERVED2 O  #
CR1 H#RESERVED3 O  #CR2 H#RESERVED4 O  #CR3 H#RESERVED5 O  #GTPR H#RESERVED6 O  # PUSART_TypeDef È#‚	*˘ICR 5# CFR 5#SR 5# PWWDG_TypeDef ’$Ì	  º    ±            ..\Source\USER\ ..\Source\CORE\ D:\Program\keil5\ARM\ARMCC\Bin\..\include\  stm32f10x.h   core_cm3.h   system_stm32f10x.h   stdint.h   stm32f10x_conf.h      3__STM32F10x_H  uHSE_VALUE ((uint32_t)25000000) ÄHSE_STARTUP_TIMEOUT ((uint16_t)0x0500) ÇHSI_VALUE ((uint32_t)8000000) á__STM32F10X_STDPERIPH_VERSION_MAIN (0x03) à__STM32F10X_STDPERIPH_VERSION_SUB1 (0x05) â__STM32F10X_STDPERIPH_VERSION_SUB2 (0x00) ä__STM32F10X_STDPERIPH_VERSION_RC (0x00) ã__STM32F10X_STDPERIPH_VERSION ( (__STM32F10X_STDPERIPH_VERSION_MAIN << 24) |(__STM32F10X_STDPERIPH_VERSION_SUB1 << 16) |(__STM32F10X_STDPERIPH_VERSION_SUB2 << 8) |(__STM32F10X_STDPERIPH_VERSION_RC)) û__MPU_PRESENT 0 †__NVIC_PRIO_BITS 4 °__Vendor_SysTickConfig 0 ﬁﬂ‡äIS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE)) èHSEStartUp_TimeOut HSE_STARTUP_TIMEOUT êHSE_Value HSE_VALUE ëHSI_Value HSI_VALUE ¯	FLASH_BASE ((uint32_t)0x08000000) ˘	SRAM_BASE ((uint32_t)0x20000000) ˙	PERIPH_BASE ((uint32_t)0x40000000) ¸	SRAM_BB_BASE ((uint32_t)0x22000000) ˝	PERIPH_BB_BASE ((uint32_t)0x42000000) ˇ	FSMC_R_BASE ((uint32_t)0xA0000000) Ç
APB1PERIPH_BASE PERIPH_BASE É
APB2PERIPH_BASE (PERIPH_BASE + 0x10000) Ñ
AHBPERIPH_BASE (PERIPH_BASE + 0x20000) Ü
TIM2_BASE (APB1PERIPH_BASE + 0x0000) á
TIM3_BASE (APB1PERIPH_BASE + 0x0400) à
TIM4_BASE (APB1PERIPH_BASE + 0x0800) â
TIM5_BASE (APB1PERIPH_BASE + 0x0C00) ä
TIM6_BASE (APB1PERIPH_BASE + 0x1000) ã
TIM7_BASE (APB1PERIPH_BASE + 0x1400) å
TIM12_BASE (APB1PERIPH_BASE + 0x1800) ç
TIM13_BASE (APB1PERIPH_BASE + 0x1C00) é
TIM14_BASE (APB1PERIPH_BASE + 0x2000) è
RTC_BASE (APB1PERIPH_BASE + 0x2800) ê
WWDG_BASE (APB1PERIPH_BASE + 0x2C00) ë
IWDG_BASE (APB1PERIPH_BASE + 0x3000) í
SPI2_BASE (APB1PERIPH_BASE + 0x3800) ì
SPI3_BASE (APB1PERIPH_BASE + 0x3C00) î
USART2_BASE (APB1PERIPH_BASE + 0x4400) ï
USART3_BASE (APB1PERIPH_BASE + 0x4800) ñ
UART4_BASE (APB1PERIPH_BASE + 0x4C00) ó
UART5_BASE (APB1PERIPH_BASE + 0x5000) ò
I2C1_BASE (APB1PERIPH_BASE + 0x5400) ô
I2C2_BASE (APB1PERIPH_BASE + 0x5800) ö
CAN1_BASE (APB1PERIPH_BASE + 0x6400) õ
CAN2_BASE (APB1PERIPH_BASE + 0x6800) ú
BKP_BASE (APB1PERIPH_BASE + 0x6C00) ù
PWR_BASE (APB1PERIPH_BASE + 0x7000) û
DAC_BASE (APB1PERIPH_BASE + 0x7400) ü
CEC_BASE (APB1PERIPH_BASE + 0x7800) °
AFIO_BASE (APB2PERIPH_BASE + 0x0000) ¢
EXTI_BASE (APB2PERIPH_BASE + 0x0400) £
GPIOA_BASE (APB2PERIPH_BASE + 0x0800) §
GPIOB_BASE (APB2PERIPH_BASE + 0x0C00) •
GPIOC_BASE (APB2PERIPH_BASE + 0x1000) ¶
GPIOD_BASE (APB2PERIPH_BASE + 0x1400) ß
GPIOE_BASE (APB2PERIPH_BASE + 0x1800) ®
GPIOF_BASE (APB2PERIPH_BASE + 0x1C00) ©
GPIOG_BASE (APB2PERIPH_BASE + 0x2000) ™
ADC1_BASE (APB2PERIPH_BASE + 0x2400) ´
ADC2_BASE (APB2PERIPH_BASE + 0x2800) ¨
TIM1_BASE (APB2PERIPH_BASE + 0x2C00) ≠
SPI1_BASE (APB2PERIPH_BASE + 0x3000) Æ
TIM8_BASE (APB2PERIPH_BASE + 0x3400) Ø
USART1_BASE (APB2PERIPH_BASE + 0x3800) ∞
ADC3_BASE (APB2PERIPH_BASE + 0x3C00) ±
TIM15_BASE (APB2PERIPH_BASE + 0x4000) ≤
TIM16_BASE (APB2PERIPH_BASE + 0x4400) ≥
TIM17_BASE (APB2PERIPH_BASE + 0x4800) ¥
TIM9_BASE (APB2PERIPH_BASE + 0x4C00) µ
TIM10_BASE (APB2PERIPH_BASE + 0x5000) ∂
TIM11_BASE (APB2PERIPH_BASE + 0x5400) ∏
SDIO_BASE (PERIPH_BASE + 0x18000) ∫
DMA1_BASE (AHBPERIPH_BASE + 0x0000) ª
DMA1_Channel1_BASE (AHBPERIPH_BASE + 0x0008) º
DMA1_Channel2_BASE (AHBPERIPH_BASE + 0x001C) Ω
DMA1_Channel3_BASE (AHBPERIPH_BASE + 0x0030) æ
DMA1_Channel4_BASE (AHBPERIPH_BASE + 0x0044) ø
DMA1_Channel5_BASE (AHBPERIPH_BASE + 0x0058) ¿
DMA1_Channel6_BASE (AHBPERIPH_BASE + 0x006C) ¡
DMA1_Channel7_BASE (AHBPERIPH_BASE + 0x0080) ¬
DMA2_BASE (AHBPERIPH_BASE + 0x0400) √
DMA2_Channel1_BASE (AHBPERIPH_BASE + 0x0408) ƒ
DMA2_Channel2_BASE (AHBPERIPH_BASE + 0x041C) ≈
DMA2_Channel3_BASE (AHBPERIPH_BASE + 0x0430) ∆
DMA2_Channel4_BASE (AHBPERIPH_BASE + 0x0444) «
DMA2_Channel5_BASE (AHBPERIPH_BASE + 0x0458) »
RCC_BASE (AHBPERIPH_BASE + 0x1000) …
CRC_BASE (AHBPERIPH_BASE + 0x3000) À
FLASH_R_BASE (AHBPERIPH_BASE + 0x2000) Ã
OB_BASE ((uint32_t)0x1FFFF800) Œ
ETH_BASE (AHBPERIPH_BASE + 0x8000) œ
ETH_MAC_BASE (ETH_BASE) –
ETH_MMC_BASE (ETH_BASE + 0x0100) —
ETH_PTP_BASE (ETH_BASE + 0x0700) “
ETH_DMA_BASE (ETH_BASE + 0x1000) ‘
FSMC_Bank1_R_BASE (FSMC_R_BASE + 0x0000) ’
FSMC_Bank1E_R_BASE (FSMC_R_BASE + 0x0104) ÷
FSMC_Bank2_R_BASE (FSMC_R_BASE + 0x0060) ◊
FSMC_Bank3_R_BASE (FSMC_R_BASE + 0x0080) ÿ
FSMC_Bank4_R_BASE (FSMC_R_BASE + 0x00A0) ⁄
DBGMCU_BASE ((uint32_t)0xE0042000) ‰
TIM2 ((TIM_TypeDef *) TIM2_BASE) Â
TIM3 ((TIM_TypeDef *) TIM3_BASE) Ê
TIM4 ((TIM_TypeDef *) TIM4_BASE) Á
TIM5 ((TIM_TypeDef *) TIM5_BASE) Ë
TIM6 ((TIM_TypeDef *) TIM6_BASE) È
TIM7 ((TIM_TypeDef *) TIM7_BASE) Í
TIM12 ((TIM_TypeDef *) TIM12_BASE) Î
TIM13 ((TIM_TypeDef *) TIM13_BASE) Ï
TIM14 ((TIM_TypeDef *) TIM14_BASE) Ì
RTC ((RTC_TypeDef *) RTC_BASE) Ó
WWDG ((WWDG_TypeDef *) WWDG_BASE) Ô
IWDG ((IWDG_TypeDef *) IWDG_BASE) 
SPI2 ((SPI_TypeDef *) SPI2_BASE) Ò
SPI3 ((SPI_TypeDef *) SPI3_BASE) Ú
USART2 ((USART_TypeDef *) USART2_BASE) Û
USART3 ((USART_TypeDef *) USART3_BASE) Ù
UART4 ((USART_TypeDef *) UART4_BASE) ı
UART5 ((USART_TypeDef *) UART5_BASE) ˆ
I2C1 ((I2C_TypeDef *) I2C1_BASE) ˜
I2C2 ((I2C_TypeDef *) I2C2_BASE) ¯
CAN1 ((CAN_TypeDef *) CAN1_BASE) ˘
CAN2 ((CAN_TypeDef *) CAN2_BASE) ˙
BKP ((BKP_TypeDef *) BKP_BASE) ˚
PWR ((PWR_TypeDef *) PWR_BASE) ¸
DAC ((DAC_TypeDef *) DAC_BASE) ˝
CEC ((CEC_TypeDef *) CEC_BASE) ˛
AFIO ((AFIO_TypeDef *) AFIO_BASE) ˇ
EXTI ((EXTI_TypeDef *) EXTI_BASE) ÄGPIOA ((GPIO_TypeDef *) GPIOA_BASE) ÅGPIOB ((GPIO_TypeDef *) GPIOB_BASE) ÇGPIOC ((GPIO_TypeDef *) GPIOC_BASE) ÉGPIOD ((GPIO_TypeDef *) GPIOD_BASE) ÑGPIOE ((GPIO_TypeDef *) GPIOE_BASE) ÖGPIOF ((GPIO_TypeDef *) GPIOF_BASE) ÜGPIOG ((GPIO_TypeDef *) GPIOG_BASE) áADC1 ((ADC_TypeDef *) ADC1_BASE) àADC2 ((ADC_TypeDef *) ADC2_BASE) âTIM1 ((TIM_TypeDef *) TIM1_BASE) äSPI1 ((SPI_TypeDef *) SPI1_BASE) ãTIM8 ((TIM_TypeDef *) TIM8_BASE) åUSART1 ((USART_TypeDef *) USART1_BASE) çADC3 ((ADC_TypeDef *) ADC3_BASE) éTIM15 ((TIM_TypeDef *) TIM15_BASE) èTIM16 ((TIM_TypeDef *) TIM16_BASE) êTIM17 ((TIM_TypeDef *) TIM17_BASE) ëTIM9 ((TIM_TypeDef *) TIM9_BASE) íTIM10 ((TIM_TypeDef *) TIM10_BASE) ìTIM11 ((TIM_TypeDef *) TIM11_BASE) îSDIO ((SDIO_TypeDef *) SDIO_BASE) ïDMA1 ((DMA_TypeDef *) DMA1_BASE) ñDMA2 ((DMA_TypeDef *) DMA2_BASE) óDMA1_Channel1 ((DMA_Channel_TypeDef *) DMA1_Channel1_BASE) òDMA1_Channel2 ((DMA_Channel_TypeDef *) DMA1_Channel2_BASE) ôDMA1_Channel3 ((DMA_Channel_TypeDef *) DMA1_Channel3_BASE) öDMA1_Channel4 ((DMA_Channel_TypeDef *) DMA1_Channel4_BASE) õDMA1_Channel5 ((DMA_Channel_TypeDef *) DMA1_Channel5_BASE) úDMA1_Channel6 ((DMA_Channel_TypeDef *) DMA1_Channel6_BASE) ùDMA1_Channel7 ((DMA_Channel_TypeDef *) DMA1_Channel7_BASE) ûDMA2_Channel1 ((DMA_Channel_TypeDef *) DMA2_Channel1_BASE) üDMA2_Channel2 ((DMA_Channel_TypeDef *) DMA2_Channel2_BASE) †DMA2_Channel3 ((DMA_Channel_TypeDef *) DMA2_Channel3_BASE) °DMA2_Channel4 ((DMA_Channel_TypeDef *) DMA2_Channel4_BASE) ¢DMA2_Channel5 ((DMA_Channel_TypeDef *) DMA2_Channel5_BASE) £RCC ((RCC_TypeDef *) RCC_BASE) §CRC ((CRC_TypeDef *) CRC_BASE) •FLASH ((FLASH_TypeDef *) FLASH_R_BASE) ¶OB ((OB_TypeDef *) OB_BASE) ßETH ((ETH_TypeDef *) ETH_BASE) ®FSMC_Bank1 ((FSMC_Bank1_TypeDef *) FSMC_Bank1_R_BASE) ©FSMC_Bank1E ((FSMC_Bank1E_TypeDef *) FSMC_Bank1E_R_BASE) ™FSMC_Bank2 ((FSMC_Bank2_TypeDef *) FSMC_Bank2_R_BASE) ´FSMC_Bank3 ((FSMC_Bank3_TypeDef *) FSMC_Bank3_R_BASE) ¨FSMC_Bank4 ((FSMC_Bank4_TypeDef *) FSMC_Bank4_R_BASE) ≠DBGMCU ((DBGMCU_TypeDef *) DBGMCU_BASE) ∆CRC_DR_DR ((uint32_t)0xFFFFFFFF)  CRC_IDR_IDR ((uint8_t)0xFF) ŒCRC_CR_RESET ((uint8_t)0x01) ◊PWR_CR_LPDS ((uint16_t)0x0001) ÿPWR_CR_PDDS ((uint16_t)0x0002) ŸPWR_CR_CWUF ((uint16_t)0x0004) ⁄PWR_CR_CSBF ((uint16_t)0x0008) €PWR_CR_PVDE ((uint16_t)0x0010) ›PWR_CR_PLS ((uint16_t)0x00E0) ﬁPWR_CR_PLS_0 ((uint16_t)0x0020) ﬂPWR_CR_PLS_1 ((uint16_t)0x0040) ‡PWR_CR_PLS_2 ((uint16_t)0x0080) „PWR_CR_PLS_2V2 ((uint16_t)0x0000) ‰PWR_CR_PLS_2V3 ((uint16_t)0x0020) ÂPWR_CR_PLS_2V4 ((uint16_t)0x0040) ÊPWR_CR_PLS_2V5 ((uint16_t)0x0060) ÁPWR_CR_PLS_2V6 ((uint16_t)0x0080) ËPWR_CR_PLS_2V7 ((uint16_t)0x00A0) ÈPWR_CR_PLS_2V8 ((uint16_t)0x00C0) ÍPWR_CR_PLS_2V9 ((uint16_t)0x00E0) ÏPWR_CR_DBP ((uint16_t)0x0100) PWR_CSR_WUF ((uint16_t)0x0001) ÒPWR_CSR_SBF ((uint16_t)0x0002) ÚPWR_CSR_PVDO ((uint16_t)0x0004) ÛPWR_CSR_EWUP ((uint16_t)0x0100) ¸BKP_DR1_D ((uint16_t)0xFFFF) ˇBKP_DR2_D ((uint16_t)0xFFFF) ÇBKP_DR3_D ((uint16_t)0xFFFF) ÖBKP_DR4_D ((uint16_t)0xFFFF) àBKP_DR5_D ((uint16_t)0xFFFF) ãBKP_DR6_D ((uint16_t)0xFFFF) éBKP_DR7_D ((uint16_t)0xFFFF) ëBKP_DR8_D ((uint16_t)0xFFFF) îBKP_DR9_D ((uint16_t)0xFFFF) óBKP_DR10_D ((uint16_t)0xFFFF) öBKP_DR11_D ((uint16_t)0xFFFF) ùBKP_DR12_D ((uint16_t)0xFFFF) †BKP_DR13_D ((uint16_t)0xFFFF) £BKP_DR14_D ((uint16_t)0xFFFF) ¶BKP_DR15_D ((uint16_t)0xFFFF) ©BKP_DR16_D ((uint16_t)0xFFFF) ¨BKP_DR17_D ((uint16_t)0xFFFF) ØBKP_DR18_D ((uint16_t)0xFFFF) ≤BKP_DR19_D ((uint16_t)0xFFFF) µBKP_DR20_D ((uint16_t)0xFFFF) ∏BKP_DR21_D ((uint16_t)0xFFFF) ªBKP_DR22_D ((uint16_t)0xFFFF) æBKP_DR23_D ((uint16_t)0xFFFF) ¡BKP_DR24_D ((uint16_t)0xFFFF) ƒBKP_DR25_D ((uint16_t)0xFFFF) «BKP_DR26_D ((uint16_t)0xFFFF)  BKP_DR27_D ((uint16_t)0xFFFF) ÕBKP_DR28_D ((uint16_t)0xFFFF) –BKP_DR29_D ((uint16_t)0xFFFF) ”BKP_DR30_D ((uint16_t)0xFFFF) ÷BKP_DR31_D ((uint16_t)0xFFFF) ŸBKP_DR32_D ((uint16_t)0xFFFF) ‹BKP_DR33_D ((uint16_t)0xFFFF) ﬂBKP_DR34_D ((uint16_t)0xFFFF) ‚BKP_DR35_D ((uint16_t)0xFFFF) ÂBKP_DR36_D ((uint16_t)0xFFFF) ËBKP_DR37_D ((uint16_t)0xFFFF) ÎBKP_DR38_D ((uint16_t)0xFFFF) ÓBKP_DR39_D ((uint16_t)0xFFFF) ÒBKP_DR40_D ((uint16_t)0xFFFF) ÙBKP_DR41_D ((uint16_t)0xFFFF) ˜BKP_DR42_D ((uint16_t)0xFFFF) ˙BKP_RTCCR_CAL ((uint16_t)0x007F) ˚BKP_RTCCR_CCO ((uint16_t)0x0080) ¸BKP_RTCCR_ASOE ((uint16_t)0x0100) ˝BKP_RTCCR_ASOS ((uint16_t)0x0200) ÄBKP_CR_TPE ((uint8_t)0x01) ÅBKP_CR_TPAL ((uint8_t)0x02) ÑBKP_CSR_CTE ((uint16_t)0x0001) ÖBKP_CSR_CTI ((uint16_t)0x0002) ÜBKP_CSR_TPIE ((uint16_t)0x0004) áBKP_CSR_TEF ((uint16_t)0x0100) àBKP_CSR_TIF ((uint16_t)0x0200) ëRCC_CR_HSION ((uint32_t)0x00000001) íRCC_CR_HSIRDY ((uint32_t)0x00000002) ìRCC_CR_HSITRIM ((uint32_t)0x000000F8) îRCC_CR_HSICAL ((uint32_t)0x0000FF00) ïRCC_CR_HSEON ((uint32_t)0x00010000) ñRCC_CR_HSERDY ((uint32_t)0x00020000) óRCC_CR_HSEBYP ((uint32_t)0x00040000) òRCC_CR_CSSON ((uint32_t)0x00080000) ôRCC_CR_PLLON ((uint32_t)0x01000000) öRCC_CR_PLLRDY ((uint32_t)0x02000000) ùRCC_CR_PLL2ON ((uint32_t)0x04000000) ûRCC_CR_PLL2RDY ((uint32_t)0x08000000) üRCC_CR_PLL3ON ((uint32_t)0x10000000) †RCC_CR_PLL3RDY ((uint32_t)0x20000000) •RCC_CFGR_SW ((uint32_t)0x00000003) ¶RCC_CFGR_SW_0 ((uint32_t)0x00000001) ßRCC_CFGR_SW_1 ((uint32_t)0x00000002) ©RCC_CFGR_SW_HSI ((uint32_t)0x00000000) ™RCC_CFGR_SW_HSE ((uint32_t)0x00000001) ´RCC_CFGR_SW_PLL ((uint32_t)0x00000002) ÆRCC_CFGR_SWS ((uint32_t)0x0000000C) ØRCC_CFGR_SWS_0 ((uint32_t)0x00000004) ∞RCC_CFGR_SWS_1 ((uint32_t)0x00000008) ≤RCC_CFGR_SWS_HSI ((uint32_t)0x00000000) ≥RCC_CFGR_SWS_HSE ((uint32_t)0x00000004) ¥RCC_CFGR_SWS_PLL ((uint32_t)0x00000008) ∑RCC_CFGR_HPRE ((uint32_t)0x000000F0) ∏RCC_CFGR_HPRE_0 ((uint32_t)0x00000010) πRCC_CFGR_HPRE_1 ((uint32_t)0x00000020) ∫RCC_CFGR_HPRE_2 ((uint32_t)0x00000040) ªRCC_CFGR_HPRE_3 ((uint32_t)0x00000080) ΩRCC_CFGR_HPRE_DIV1 ((uint32_t)0x00000000) æRCC_CFGR_HPRE_DIV2 ((uint32_t)0x00000080) øRCC_CFGR_HPRE_DIV4 ((uint32_t)0x00000090) ¿RCC_CFGR_HPRE_DIV8 ((uint32_t)0x000000A0) ¡RCC_CFGR_HPRE_DIV16 ((uint32_t)0x000000B0) ¬RCC_CFGR_HPRE_DIV64 ((uint32_t)0x000000C0) √RCC_CFGR_HPRE_DIV128 ((uint32_t)0x000000D0) ƒRCC_CFGR_HPRE_DIV256 ((uint32_t)0x000000E0) ≈RCC_CFGR_HPRE_DIV512 ((uint32_t)0x000000F0) »RCC_CFGR_PPRE1 ((uint32_t)0x00000700) …RCC_CFGR_PPRE1_0 ((uint32_t)0x00000100)  RCC_CFGR_PPRE1_1 ((uint32_t)0x00000200) ÀRCC_CFGR_PPRE1_2 ((uint32_t)0x00000400) ÕRCC_CFGR_PPRE1_DIV1 ((uint32_t)0x00000000) ŒRCC_CFGR_PPRE1_DIV2 ((uint32_t)0x00000400) œRCC_CFGR_PPRE1_DIV4 ((uint32_t)0x00000500) –RCC_CFGR_PPRE1_DIV8 ((uint32_t)0x00000600) —RCC_CFGR_PPRE1_DIV16 ((uint32_t)0x00000700) ‘RCC_CFGR_PPRE2 ((uint32_t)0x00003800) ’RCC_CFGR_PPRE2_0 ((uint32_t)0x00000800) ÷RCC_CFGR_PPRE2_1 ((uint32_t)0x00001000) ◊RCC_CFGR_PPRE2_2 ((uint32_t)0x00002000) ŸRCC_CFGR_PPRE2_DIV1 ((uint32_t)0x00000000) ⁄RCC_CFGR_PPRE2_DIV2 ((uint32_t)0x00002000) €RCC_CFGR_PPRE2_DIV4 ((uint32_t)0x00002800) ‹RCC_CFGR_PPRE2_DIV8 ((uint32_t)0x00003000) ›RCC_CFGR_PPRE2_DIV16 ((uint32_t)0x00003800) ‡RCC_CFGR_ADCPRE ((uint32_t)0x0000C000) ·RCC_CFGR_ADCPRE_0 ((uint32_t)0x00004000) ‚RCC_CFGR_ADCPRE_1 ((uint32_t)0x00008000) ‰RCC_CFGR_ADCPRE_DIV2 ((uint32_t)0x00000000) ÂRCC_CFGR_ADCPRE_DIV4 ((uint32_t)0x00004000) ÊRCC_CFGR_ADCPRE_DIV6 ((uint32_t)0x00008000) ÁRCC_CFGR_ADCPRE_DIV8 ((uint32_t)0x0000C000) ÈRCC_CFGR_PLLSRC ((uint32_t)0x00010000) ÎRCC_CFGR_PLLXTPRE ((uint32_t)0x00020000) ÓRCC_CFGR_PLLMULL ((uint32_t)0x003C0000) ÔRCC_CFGR_PLLMULL_0 ((uint32_t)0x00040000) RCC_CFGR_PLLMULL_1 ((uint32_t)0x00080000) ÒRCC_CFGR_PLLMULL_2 ((uint32_t)0x00100000) ÚRCC_CFGR_PLLMULL_3 ((uint32_t)0x00200000) ıRCC_CFGR_PLLSRC_HSI_Div2 ((uint32_t)0x00000000) ˆRCC_CFGR_PLLSRC_PREDIV1 ((uint32_t)0x00010000) ¯RCC_CFGR_PLLXTPRE_PREDIV1 ((uint32_t)0x00000000) ˘RCC_CFGR_PLLXTPRE_PREDIV1_Div2 ((uint32_t)0x00020000) ˚RCC_CFGR_PLLMULL4 ((uint32_t)0x00080000) ¸RCC_CFGR_PLLMULL5 ((uint32_t)0x000C0000) ˝RCC_CFGR_PLLMULL6 ((uint32_t)0x00100000) ˛RCC_CFGR_PLLMULL7 ((uint32_t)0x00140000) ˇRCC_CFGR_PLLMULL8 ((uint32_t)0x00180000) ÄRCC_CFGR_PLLMULL9 ((uint32_t)0x001C0000) ÅRCC_CFGR_PLLMULL6_5 ((uint32_t)0x00340000) ÉRCC_CFGR_OTGFSPRE ((uint32_t)0x00400000) ÜRCC_CFGR_MCO ((uint32_t)0x0F000000) áRCC_CFGR_MCO_0 ((uint32_t)0x01000000) àRCC_CFGR_MCO_1 ((uint32_t)0x02000000) âRCC_CFGR_MCO_2 ((uint32_t)0x04000000) äRCC_CFGR_MCO_3 ((uint32_t)0x08000000) åRCC_CFGR_MCO_NOCLOCK ((uint32_t)0x00000000) çRCC_CFGR_MCO_SYSCLK ((uint32_t)0x04000000) éRCC_CFGR_MCO_HSI ((uint32_t)0x05000000) èRCC_CFGR_MCO_HSE ((uint32_t)0x06000000) êRCC_CFGR_MCO_PLLCLK_Div2 ((uint32_t)0x07000000) ëRCC_CFGR_MCO_PLL2CLK ((uint32_t)0x08000000) íRCC_CFGR_MCO_PLL3CLK_Div2 ((uint32_t)0x09000000) ìRCC_CFGR_MCO_Ext_HSE ((uint32_t)0x0A000000) îRCC_CFGR_MCO_PLL3CLK ((uint32_t)0x0B000000) ›RCC_CIR_LSIRDYF ((uint32_t)0x00000001) ﬁRCC_CIR_LSERDYF ((uint32_t)0x00000002) ﬂRCC_CIR_HSIRDYF ((uint32_t)0x00000004) ‡RCC_CIR_HSERDYF ((uint32_t)0x00000008) ·RCC_CIR_PLLRDYF ((uint32_t)0x00000010) ‚RCC_CIR_CSSF ((uint32_t)0x00000080) „RCC_CIR_LSIRDYIE ((uint32_t)0x00000100) ‰RCC_CIR_LSERDYIE ((uint32_t)0x00000200) ÂRCC_CIR_HSIRDYIE ((uint32_t)0x00000400) ÊRCC_CIR_HSERDYIE ((uint32_t)0x00000800) ÁRCC_CIR_PLLRDYIE ((uint32_t)0x00001000) ËRCC_CIR_LSIRDYC ((uint32_t)0x00010000) ÈRCC_CIR_LSERDYC ((uint32_t)0x00020000) ÍRCC_CIR_HSIRDYC ((uint32_t)0x00040000) ÎRCC_CIR_HSERDYC ((uint32_t)0x00080000) ÏRCC_CIR_PLLRDYC ((uint32_t)0x00100000) ÌRCC_CIR_CSSC ((uint32_t)0x00800000) RCC_CIR_PLL2RDYF ((uint32_t)0x00000020) ÒRCC_CIR_PLL3RDYF ((uint32_t)0x00000040) ÚRCC_CIR_PLL2RDYIE ((uint32_t)0x00002000) ÛRCC_CIR_PLL3RDYIE ((uint32_t)0x00004000) ÙRCC_CIR_PLL2RDYC ((uint32_t)0x00200000) ıRCC_CIR_PLL3RDYC ((uint32_t)0x00400000) ˘RCC_APB2RSTR_AFIORST ((uint32_t)0x00000001) ˙RCC_APB2RSTR_IOPARST ((uint32_t)0x00000004) ˚RCC_APB2RSTR_IOPBRST ((uint32_t)0x00000008) ¸RCC_APB2RSTR_IOPCRST ((uint32_t)0x00000010) ˝RCC_APB2RSTR_IOPDRST ((uint32_t)0x00000020) ˛RCC_APB2RSTR_ADC1RST ((uint32_t)0x00000200) ÅRCC_APB2RSTR_ADC2RST ((uint32_t)0x00000400) ÑRCC_APB2RSTR_TIM1RST ((uint32_t)0x00000800) ÖRCC_APB2RSTR_SPI1RST ((uint32_t)0x00001000) ÜRCC_APB2RSTR_USART1RST ((uint32_t)0x00004000) èRCC_APB2RSTR_IOPERST ((uint32_t)0x00000040) •RCC_APB1RSTR_TIM2RST ((uint32_t)0x00000001) ¶RCC_APB1RSTR_TIM3RST ((uint32_t)0x00000002) ßRCC_APB1RSTR_WWDGRST ((uint32_t)0x00000800) ®RCC_APB1RSTR_USART2RST ((uint32_t)0x00020000) ©RCC_APB1RSTR_I2C1RST ((uint32_t)0x00200000) ¨RCC_APB1RSTR_CAN1RST ((uint32_t)0x02000000) ØRCC_APB1RSTR_BKPRST ((uint32_t)0x08000000) ∞RCC_APB1RSTR_PWRRST ((uint32_t)0x10000000) ≥RCC_APB1RSTR_TIM4RST ((uint32_t)0x00000004) ¥RCC_APB1RSTR_SPI2RST ((uint32_t)0x00004000) µRCC_APB1RSTR_USART3RST ((uint32_t)0x00040000) ∂RCC_APB1RSTR_I2C2RST ((uint32_t)0x00400000) æRCC_APB1RSTR_TIM5RST ((uint32_t)0x00000008) øRCC_APB1RSTR_TIM6RST ((uint32_t)0x00000010) ¿RCC_APB1RSTR_TIM7RST ((uint32_t)0x00000020) ¡RCC_APB1RSTR_SPI3RST ((uint32_t)0x00008000) ¬RCC_APB1RSTR_UART4RST ((uint32_t)0x00080000) √RCC_APB1RSTR_UART5RST ((uint32_t)0x00100000) ƒRCC_APB1RSTR_DACRST ((uint32_t)0x20000000) ŸRCC_APB1RSTR_CAN2RST ((uint32_t)0x04000000) „RCC_AHBENR_DMA1EN ((uint16_t)0x0001) ‰RCC_AHBENR_SRAMEN ((uint16_t)0x0004) ÂRCC_AHBENR_FLITFEN ((uint16_t)0x0010) ÊRCC_AHBENR_CRCEN ((uint16_t)0x0040) ÈRCC_AHBENR_DMA2EN ((uint16_t)0x0002) ˆRCC_AHBENR_OTGFSEN ((uint32_t)0x00001000) ˜RCC_AHBENR_ETHMACEN ((uint32_t)0x00004000) ¯RCC_AHBENR_ETHMACTXEN ((uint32_t)0x00008000) ˘RCC_AHBENR_ETHMACRXEN ((uint32_t)0x00010000) ˝RCC_APB2ENR_AFIOEN ((uint32_t)0x00000001) ˛RCC_APB2ENR_IOPAEN ((uint32_t)0x00000004) ˇRCC_APB2ENR_IOPBEN ((uint32_t)0x00000008) ÄRCC_APB2ENR_IOPCEN ((uint32_t)0x00000010) ÅRCC_APB2ENR_IOPDEN ((uint32_t)0x00000020) ÇRCC_APB2ENR_ADC1EN ((uint32_t)0x00000200) ÖRCC_APB2ENR_ADC2EN ((uint32_t)0x00000400) àRCC_APB2ENR_TIM1EN ((uint32_t)0x00000800) âRCC_APB2ENR_SPI1EN ((uint32_t)0x00001000) äRCC_APB2ENR_USART1EN ((uint32_t)0x00004000) ìRCC_APB2ENR_IOPEEN ((uint32_t)0x00000040) ©RCC_APB1ENR_TIM2EN ((uint32_t)0x00000001) ™RCC_APB1ENR_TIM3EN ((uint32_t)0x00000002) ´RCC_APB1ENR_WWDGEN ((uint32_t)0x00000800) ¨RCC_APB1ENR_USART2EN ((uint32_t)0x00020000) ≠RCC_APB1ENR_I2C1EN ((uint32_t)0x00200000) ∞RCC_APB1ENR_CAN1EN ((uint32_t)0x02000000) ≥RCC_APB1ENR_BKPEN ((uint32_t)0x08000000) ¥RCC_APB1ENR_PWREN ((uint32_t)0x10000000) ∑RCC_APB1ENR_TIM4EN ((uint32_t)0x00000004) ∏RCC_APB1ENR_SPI2EN ((uint32_t)0x00004000) πRCC_APB1ENR_USART3EN ((uint32_t)0x00040000) ∫RCC_APB1ENR_I2C2EN ((uint32_t)0x00400000) ¬RCC_APB1ENR_TIM5EN ((uint32_t)0x00000008) √RCC_APB1ENR_TIM6EN ((uint32_t)0x00000010) ƒRCC_APB1ENR_TIM7EN ((uint32_t)0x00000020) ≈RCC_APB1ENR_SPI3EN ((uint32_t)0x00008000) ∆RCC_APB1ENR_UART4EN ((uint32_t)0x00080000) «RCC_APB1ENR_UART5EN ((uint32_t)0x00100000) »RCC_APB1ENR_DACEN ((uint32_t)0x20000000) ›RCC_APB1ENR_CAN2EN ((uint32_t)0x04000000) ÁRCC_BDCR_LSEON ((uint32_t)0x00000001) ËRCC_BDCR_LSERDY ((uint32_t)0x00000002) ÈRCC_BDCR_LSEBYP ((uint32_t)0x00000004) ÎRCC_BDCR_RTCSEL ((uint32_t)0x00000300) ÏRCC_BDCR_RTCSEL_0 ((uint32_t)0x00000100) ÌRCC_BDCR_RTCSEL_1 ((uint32_t)0x00000200) RCC_BDCR_RTCSEL_NOCLOCK ((uint32_t)0x00000000) ÒRCC_BDCR_RTCSEL_LSE ((uint32_t)0x00000100) ÚRCC_BDCR_RTCSEL_LSI ((uint32_t)0x00000200) ÛRCC_BDCR_RTCSEL_HSE ((uint32_t)0x00000300) ıRCC_BDCR_RTCEN ((uint32_t)0x00008000) ˆRCC_BDCR_BDRST ((uint32_t)0x00010000) ˘RCC_CSR_LSION ((uint32_t)0x00000001) ˙RCC_CSR_LSIRDY ((uint32_t)0x00000002) ˚RCC_CSR_RMVF ((uint32_t)0x01000000) ¸RCC_CSR_PINRSTF ((uint32_t)0x04000000) ˝RCC_CSR_PORRSTF ((uint32_t)0x08000000) ˛RCC_CSR_SFTRSTF ((uint32_t)0x10000000) ˇRCC_CSR_IWDGRSTF ((uint32_t)0x20000000) ÄRCC_CSR_WWDGRSTF ((uint32_t)0x40000000) ÅRCC_CSR_LPWRRSTF ((uint32_t)0x80000000) ÖRCC_AHBRSTR_OTGFSRST ((uint32_t)0x00001000) ÜRCC_AHBRSTR_ETHMACRST ((uint32_t)0x00004000) äRCC_CFGR2_PREDIV1 ((uint32_t)0x0000000F) ãRCC_CFGR2_PREDIV1_0 ((uint32_t)0x00000001) åRCC_CFGR2_PREDIV1_1 ((uint32_t)0x00000002) çRCC_CFGR2_PREDIV1_2 ((uint32_t)0x00000004) éRCC_CFGR2_PREDIV1_3 ((uint32_t)0x00000008) êRCC_CFGR2_PREDIV1_DIV1 ((uint32_t)0x00000000) ëRCC_CFGR2_PREDIV1_DIV2 ((uint32_t)0x00000001) íRCC_CFGR2_PREDIV1_DIV3 ((uint32_t)0x00000002) ìRCC_CFGR2_PREDIV1_DIV4 ((uint32_t)0x00000003) îRCC_CFGR2_PREDIV1_DIV5 ((uint32_t)0x00000004) ïRCC_CFGR2_PREDIV1_DIV6 ((uint32_t)0x00000005) ñRCC_CFGR2_PREDIV1_DIV7 ((uint32_t)0x00000006) óRCC_CFGR2_PREDIV1_DIV8 ((uint32_t)0x00000007) òRCC_CFGR2_PREDIV1_DIV9 ((uint32_t)0x00000008) ôRCC_CFGR2_PREDIV1_DIV10 ((uint32_t)0x00000009) öRCC_CFGR2_PREDIV1_DIV11 ((uint32_t)0x0000000A) õRCC_CFGR2_PREDIV1_DIV12 ((uint32_t)0x0000000B) úRCC_CFGR2_PREDIV1_DIV13 ((uint32_t)0x0000000C) ùRCC_CFGR2_PREDIV1_DIV14 ((uint32_t)0x0000000D) ûRCC_CFGR2_PREDIV1_DIV15 ((uint32_t)0x0000000E) üRCC_CFGR2_PREDIV1_DIV16 ((uint32_t)0x0000000F) ¢RCC_CFGR2_PREDIV2 ((uint32_t)0x000000F0) £RCC_CFGR2_PREDIV2_0 ((uint32_t)0x00000010) §RCC_CFGR2_PREDIV2_1 ((uint32_t)0x00000020) •RCC_CFGR2_PREDIV2_2 ((uint32_t)0x00000040) ¶RCC_CFGR2_PREDIV2_3 ((uint32_t)0x00000080) ®RCC_CFGR2_PREDIV2_DIV1 ((uint32_t)0x00000000) ©RCC_CFGR2_PREDIV2_DIV2 ((uint32_t)0x00000010) ™RCC_CFGR2_PREDIV2_DIV3 ((uint32_t)0x00000020) ´RCC_CFGR2_PREDIV2_DIV4 ((uint32_t)0x00000030) ¨RCC_CFGR2_PREDIV2_DIV5 ((uint32_t)0x00000040) ≠RCC_CFGR2_PREDIV2_DIV6 ((uint32_t)0x00000050) ÆRCC_CFGR2_PREDIV2_DIV7 ((uint32_t)0x00000060) ØRCC_CFGR2_PREDIV2_DIV8 ((uint32_t)0x00000070) ∞RCC_CFGR2_PREDIV2_DIV9 ((uint32_t)0x00000080) ±RCC_CFGR2_PREDIV2_DIV10 ((uint32_t)0x00000090) ≤RCC_CFGR2_PREDIV2_DIV11 ((uint32_t)0x000000A0) ≥RCC_CFGR2_PREDIV2_DIV12 ((uint32_t)0x000000B0) ¥RCC_CFGR2_PREDIV2_DIV13 ((uint32_t)0x000000C0) µRCC_CFGR2_PREDIV2_DIV14 ((uint32_t)0x000000D0) ∂RCC_CFGR2_PREDIV2_DIV15 ((uint32_t)0x000000E0) ∑RCC_CFGR2_PREDIV2_DIV16 ((uint32_t)0x000000F0) ∫RCC_CFGR2_PLL2MUL ((uint32_t)0x00000F00) ªRCC_CFGR2_PLL2MUL_0 ((uint32_t)0x00000100) ºRCC_CFGR2_PLL2MUL_1 ((uint32_t)0x00000200) ΩRCC_CFGR2_PLL2MUL_2 ((uint32_t)0x00000400) æRCC_CFGR2_PLL2MUL_3 ((uint32_t)0x00000800) ¿RCC_CFGR2_PLL2MUL8 ((uint32_t)0x00000600) ¡RCC_CFGR2_PLL2MUL9 ((uint32_t)0x00000700) ¬RCC_CFGR2_PLL2MUL10 ((uint32_t)0x00000800) √RCC_CFGR2_PLL2MUL11 ((uint32_t)0x00000900) ƒRCC_CFGR2_PLL2MUL12 ((uint32_t)0x00000A00) ≈RCC_CFGR2_PLL2MUL13 ((uint32_t)0x00000B00) ∆RCC_CFGR2_PLL2MUL14 ((uint32_t)0x00000C00) «RCC_CFGR2_PLL2MUL16 ((uint32_t)0x00000E00) »RCC_CFGR2_PLL2MUL20 ((uint32_t)0x00000F00) ÀRCC_CFGR2_PLL3MUL ((uint32_t)0x0000F000) ÃRCC_CFGR2_PLL3MUL_0 ((uint32_t)0x00001000) ÕRCC_CFGR2_PLL3MUL_1 ((uint32_t)0x00002000) ŒRCC_CFGR2_PLL3MUL_2 ((uint32_t)0x00004000) œRCC_CFGR2_PLL3MUL_3 ((uint32_t)0x00008000) —RCC_CFGR2_PLL3MUL8 ((uint32_t)0x00006000) “RCC_CFGR2_PLL3MUL9 ((uint32_t)0x00007000) ”RCC_CFGR2_PLL3MUL10 ((uint32_t)0x00008000) ‘RCC_CFGR2_PLL3MUL11 ((uint32_t)0x00009000) ’RCC_CFGR2_PLL3MUL12 ((uint32_t)0x0000A000) ÷RCC_CFGR2_PLL3MUL13 ((uint32_t)0x0000B000) ◊RCC_CFGR2_PLL3MUL14 ((uint32_t)0x0000C000) ÿRCC_CFGR2_PLL3MUL16 ((uint32_t)0x0000E000) ŸRCC_CFGR2_PLL3MUL20 ((uint32_t)0x0000F000) €RCC_CFGR2_PREDIV1SRC ((uint32_t)0x00010000) ‹RCC_CFGR2_PREDIV1SRC_PLL2 ((uint32_t)0x00010000) ›RCC_CFGR2_PREDIV1SRC_HSE ((uint32_t)0x00000000) ﬁRCC_CFGR2_I2S2SRC ((uint32_t)0x00020000) ﬂRCC_CFGR2_I2S3SRC ((uint32_t)0x00040000) ÑGPIO_CRL_MODE ((uint32_t)0x33333333) ÜGPIO_CRL_MODE0 ((uint32_t)0x00000003) áGPIO_CRL_MODE0_0 ((uint32_t)0x00000001) àGPIO_CRL_MODE0_1 ((uint32_t)0x00000002) äGPIO_CRL_MODE1 ((uint32_t)0x00000030) ãGPIO_CRL_MODE1_0 ((uint32_t)0x00000010) åGPIO_CRL_MODE1_1 ((uint32_t)0x00000020) éGPIO_CRL_MODE2 ((uint32_t)0x00000300) èGPIO_CRL_MODE2_0 ((uint32_t)0x00000100) êGPIO_CRL_MODE2_1 ((uint32_t)0x00000200) íGPIO_CRL_MODE3 ((uint32_t)0x00003000) ìGPIO_CRL_MODE3_0 ((uint32_t)0x00001000) îGPIO_CRL_MODE3_1 ((uint32_t)0x00002000) ñGPIO_CRL_MODE4 ((uint32_t)0x00030000) óGPIO_CRL_MODE4_0 ((uint32_t)0x00010000) òGPIO_CRL_MODE4_1 ((uint32_t)0x00020000) öGPIO_CRL_MODE5 ((uint32_t)0x00300000) õGPIO_CRL_MODE5_0 ((uint32_t)0x00100000) úGPIO_CRL_MODE5_1 ((uint32_t)0x00200000) ûGPIO_CRL_MODE6 ((uint32_t)0x03000000) üGPIO_CRL_MODE6_0 ((uint32_t)0x01000000) †GPIO_CRL_MODE6_1 ((uint32_t)0x02000000) ¢GPIO_CRL_MODE7 ((uint32_t)0x30000000) £GPIO_CRL_MODE7_0 ((uint32_t)0x10000000) §GPIO_CRL_MODE7_1 ((uint32_t)0x20000000) ¶GPIO_CRL_CNF ((uint32_t)0xCCCCCCCC) ®GPIO_CRL_CNF0 ((uint32_t)0x0000000C) ©GPIO_CRL_CNF0_0 ((uint32_t)0x00000004) ™GPIO_CRL_CNF0_1 ((uint32_t)0x00000008) ¨GPIO_CRL_CNF1 ((uint32_t)0x000000C0) ≠GPIO_CRL_CNF1_0 ((uint32_t)0x00000040) ÆGPIO_CRL_CNF1_1 ((uint32_t)0x00000080) ∞GPIO_CRL_CNF2 ((uint32_t)0x00000C00) ±GPIO_CRL_CNF2_0 ((uint32_t)0x00000400) ≤GPIO_CRL_CNF2_1 ((uint32_t)0x00000800) ¥GPIO_CRL_CNF3 ((uint32_t)0x0000C000) µGPIO_CRL_CNF3_0 ((uint32_t)0x00004000) ∂GPIO_CRL_CNF3_1 ((uint32_t)0x00008000) ∏GPIO_CRL_CNF4 ((uint32_t)0x000C0000) πGPIO_CRL_CNF4_0 ((uint32_t)0x00040000) ∫GPIO_CRL_CNF4_1 ((uint32_t)0x00080000) ºGPIO_CRL_CNF5 ((uint32_t)0x00C00000) ΩGPIO_CRL_CNF5_0 ((uint32_t)0x00400000) æGPIO_CRL_CNF5_1 ((uint32_t)0x00800000) ¿GPIO_CRL_CNF6 ((uint32_t)0x0C000000) ¡GPIO_CRL_CNF6_0 ((uint32_t)0x04000000) ¬GPIO_CRL_CNF6_1 ((uint32_t)0x08000000) ƒGPIO_CRL_CNF7 ((uint32_t)0xC0000000) ≈GPIO_CRL_CNF7_0 ((uint32_t)0x40000000) ∆GPIO_CRL_CNF7_1 ((uint32_t)0x80000000) …GPIO_CRH_MODE ((uint32_t)0x33333333) ÀGPIO_CRH_MODE8 ((uint32_t)0x00000003) ÃGPIO_CRH_MODE8_0 ((uint32_t)0x00000001) ÕGPIO_CRH_MODE8_1 ((uint32_t)0x00000002) œGPIO_CRH_MODE9 ((uint32_t)0x00000030) –GPIO_CRH_MODE9_0 ((uint32_t)0x00000010) —GPIO_CRH_MODE9_1 ((uint32_t)0x00000020) ”GPIO_CRH_MODE10 ((uint32_t)0x00000300) ‘GPIO_CRH_MODE10_0 ((uint32_t)0x00000100) ’GPIO_CRH_MODE10_1 ((uint32_t)0x00000200) ◊GPIO_CRH_MODE11 ((uint32_t)0x00003000) ÿGPIO_CRH_MODE11_0 ((uint32_t)0x00001000) ŸGPIO_CRH_MODE11_1 ((uint32_t)0x00002000) €GPIO_CRH_MODE12 ((uint32_t)0x00030000) ‹GPIO_CRH_MODE12_0 ((uint32_t)0x00010000) ›GPIO_CRH_MODE12_1 ((uint32_t)0x00020000) ﬂGPIO_CRH_MODE13 ((uint32_t)0x00300000) ‡GPIO_CRH_MODE13_0 ((uint32_t)0x00100000) ·GPIO_CRH_MODE13_1 ((uint32_t)0x00200000) „GPIO_CRH_MODE14 ((uint32_t)0x03000000) ‰GPIO_CRH_MODE14_0 ((uint32_t)0x01000000) ÂGPIO_CRH_MODE14_1 ((uint32_t)0x02000000) ÁGPIO_CRH_MODE15 ((uint32_t)0x30000000) ËGPIO_CRH_MODE15_0 ((uint32_t)0x10000000) ÈGPIO_CRH_MODE15_1 ((uint32_t)0x20000000) ÎGPIO_CRH_CNF ((uint32_t)0xCCCCCCCC) ÌGPIO_CRH_CNF8 ((uint32_t)0x0000000C) ÓGPIO_CRH_CNF8_0 ((uint32_t)0x00000004) ÔGPIO_CRH_CNF8_1 ((uint32_t)0x00000008) ÒGPIO_CRH_CNF9 ((uint32_t)0x000000C0) ÚGPIO_CRH_CNF9_0 ((uint32_t)0x00000040) ÛGPIO_CRH_CNF9_1 ((uint32_t)0x00000080) ıGPIO_CRH_CNF10 ((uint32_t)0x00000C00) ˆGPIO_CRH_CNF10_0 ((uint32_t)0x00000400) ˜GPIO_CRH_CNF10_1 ((uint32_t)0x00000800) ˘GPIO_CRH_CNF11 ((uint32_t)0x0000C000) ˙GPIO_CRH_CNF11_0 ((uint32_t)0x00004000) ˚GPIO_CRH_CNF11_1 ((uint32_t)0x00008000) ˝GPIO_CRH_CNF12 ((uint32_t)0x000C0000) ˛GPIO_CRH_CNF12_0 ((uint32_t)0x00040000) ˇGPIO_CRH_CNF12_1 ((uint32_t)0x00080000) ÅGPIO_CRH_CNF13 ((uint32_t)0x00C00000) ÇGPIO_CRH_CNF13_0 ((uint32_t)0x00400000) ÉGPIO_CRH_CNF13_1 ((uint32_t)0x00800000) ÖGPIO_CRH_CNF14 ((uint32_t)0x0C000000) ÜGPIO_CRH_CNF14_0 ((uint32_t)0x04000000) áGPIO_CRH_CNF14_1 ((uint32_t)0x08000000) âGPIO_CRH_CNF15 ((uint32_t)0xC0000000) äGPIO_CRH_CNF15_0 ((uint32_t)0x40000000) ãGPIO_CRH_CNF15_1 ((uint32_t)0x80000000) éGPIO_IDR_IDR0 ((uint16_t)0x0001) èGPIO_IDR_IDR1 ((uint16_t)0x0002) êGPIO_IDR_IDR2 ((uint16_t)0x0004) ëGPIO_IDR_IDR3 ((uint16_t)0x0008) íGPIO_IDR_IDR4 ((uint16_t)0x0010) ìGPIO_IDR_IDR5 ((uint16_t)0x0020) îGPIO_IDR_IDR6 ((uint16_t)0x0040) ïGPIO_IDR_IDR7 ((uint16_t)0x0080) ñGPIO_IDR_IDR8 ((uint16_t)0x0100) óGPIO_IDR_IDR9 ((uint16_t)0x0200) òGPIO_IDR_IDR10 ((uint16_t)0x0400) ôGPIO_IDR_IDR11 ((uint16_t)0x0800) öGPIO_IDR_IDR12 ((uint16_t)0x1000) õGPIO_IDR_IDR13 ((uint16_t)0x2000) úGPIO_IDR_IDR14 ((uint16_t)0x4000) ùGPIO_IDR_IDR15 ((uint16_t)0x8000) †GPIO_ODR_ODR0 ((uint16_t)0x0001) °GPIO_ODR_ODR1 ((uint16_t)0x0002) ¢GPIO_ODR_ODR2 ((uint16_t)0x0004) £GPIO_ODR_ODR3 ((uint16_t)0x0008) §GPIO_ODR_ODR4 ((uint16_t)0x0010) •GPIO_ODR_ODR5 ((uint16_t)0x0020) ¶GPIO_ODR_ODR6 ((uint16_t)0x0040) ßGPIO_ODR_ODR7 ((uint16_t)0x0080) ®GPIO_ODR_ODR8 ((uint16_t)0x0100) ©GPIO_ODR_ODR9 ((uint16_t)0x0200) ™GPIO_ODR_ODR10 ((uint16_t)0x0400) ´GPIO_ODR_ODR11 ((uint16_t)0x0800) ¨GPIO_ODR_ODR12 ((uint16_t)0x1000) ≠GPIO_ODR_ODR13 ((uint16_t)0x2000) ÆGPIO_ODR_ODR14 ((uint16_t)0x4000) ØGPIO_ODR_ODR15 ((uint16_t)0x8000) ≤GPIO_BSRR_BS0 ((uint32_t)0x00000001) ≥GPIO_BSRR_BS1 ((uint32_t)0x00000002) ¥GPIO_BSRR_BS2 ((uint32_t)0x00000004) µGPIO_BSRR_BS3 ((uint32_t)0x00000008) ∂GPIO_BSRR_BS4 ((uint32_t)0x00000010) ∑GPIO_BSRR_BS5 ((uint32_t)0x00000020) ∏GPIO_BSRR_BS6 ((uint32_t)0x00000040) πGPIO_BSRR_BS7 ((uint32_t)0x00000080) ∫GPIO_BSRR_BS8 ((uint32_t)0x00000100) ªGPIO_BSRR_BS9 ((uint32_t)0x00000200) ºGPIO_BSRR_BS10 ((uint32_t)0x00000400) ΩGPIO_BSRR_BS11 ((uint32_t)0x00000800) æGPIO_BSRR_BS12 ((uint32_t)0x00001000) øGPIO_BSRR_BS13 ((uint32_t)0x00002000) ¿GPIO_BSRR_BS14 ((uint32_t)0x00004000) ¡GPIO_BSRR_BS15 ((uint32_t)0x00008000) √GPIO_BSRR_BR0 ((uint32_t)0x00010000) ƒGPIO_BSRR_BR1 ((uint32_t)0x00020000) ≈GPIO_BSRR_BR2 ((uint32_t)0x00040000) ∆GPIO_BSRR_BR3 ((uint32_t)0x00080000) «GPIO_BSRR_BR4 ((uint32_t)0x00100000) »GPIO_BSRR_BR5 ((uint32_t)0x00200000) …GPIO_BSRR_BR6 ((uint32_t)0x00400000)  GPIO_BSRR_BR7 ((uint32_t)0x00800000) ÀGPIO_BSRR_BR8 ((uint32_t)0x01000000) ÃGPIO_BSRR_BR9 ((uint32_t)0x02000000) ÕGPIO_BSRR_BR10 ((uint32_t)0x04000000) ŒGPIO_BSRR_BR11 ((uint32_t)0x08000000) œGPIO_BSRR_BR12 ((uint32_t)0x10000000) –GPIO_BSRR_BR13 ((uint32_t)0x20000000) —GPIO_BSRR_BR14 ((uint32_t)0x40000000) “GPIO_BSRR_BR15 ((uint32_t)0x80000000) ’GPIO_BRR_BR0 ((uint16_t)0x0001) ÷GPIO_BRR_BR1 ((uint16_t)0x0002) ◊GPIO_BRR_BR2 ((uint16_t)0x0004) ÿGPIO_BRR_BR3 ((uint16_t)0x0008) ŸGPIO_BRR_BR4 ((uint16_t)0x0010) ⁄GPIO_BRR_BR5 ((uint16_t)0x0020) €GPIO_BRR_BR6 ((uint16_t)0x0040) ‹GPIO_BRR_BR7 ((uint16_t)0x0080) ›GPIO_BRR_BR8 ((uint16_t)0x0100) ﬁGPIO_BRR_BR9 ((uint16_t)0x0200) ﬂGPIO_BRR_BR10 ((uint16_t)0x0400) ‡GPIO_BRR_BR11 ((uint16_t)0x0800) ·GPIO_BRR_BR12 ((uint16_t)0x1000) ‚GPIO_BRR_BR13 ((uint16_t)0x2000) „GPIO_BRR_BR14 ((uint16_t)0x4000) ‰GPIO_BRR_BR15 ((uint16_t)0x8000) ÁGPIO_LCKR_LCK0 ((uint32_t)0x00000001) ËGPIO_LCKR_LCK1 ((uint32_t)0x00000002) ÈGPIO_LCKR_LCK2 ((uint32_t)0x00000004) ÍGPIO_LCKR_LCK3 ((uint32_t)0x00000008) ÎGPIO_LCKR_LCK4 ((uint32_t)0x00000010) ÏGPIO_LCKR_LCK5 ((uint32_t)0x00000020) ÌGPIO_LCKR_LCK6 ((uint32_t)0x00000040) ÓGPIO_LCKR_LCK7 ((uint32_t)0x00000080) ÔGPIO_LCKR_LCK8 ((uint32_t)0x00000100) GPIO_LCKR_LCK9 ((uint32_t)0x00000200) ÒGPIO_LCKR_LCK10 ((uint32_t)0x00000400) ÚGPIO_LCKR_LCK11 ((uint32_t)0x00000800) ÛGPIO_LCKR_LCK12 ((uint32_t)0x00001000) ÙGPIO_LCKR_LCK13 ((uint32_t)0x00002000) ıGPIO_LCKR_LCK14 ((uint32_t)0x00004000) ˆGPIO_LCKR_LCK15 ((uint32_t)0x00008000) ˜GPIO_LCKR_LCKK ((uint32_t)0x00010000) ¸AFIO_EVCR_PIN ((uint8_t)0x0F) ˝AFIO_EVCR_PIN_0 ((uint8_t)0x01) ˛AFIO_EVCR_PIN_1 ((uint8_t)0x02) ˇAFIO_EVCR_PIN_2 ((uint8_t)0x04) ÄAFIO_EVCR_PIN_3 ((uint8_t)0x08) ÉAFIO_EVCR_PIN_PX0 ((uint8_t)0x00) ÑAFIO_EVCR_PIN_PX1 ((uint8_t)0x01) ÖAFIO_EVCR_PIN_PX2 ((uint8_t)0x02) ÜAFIO_EVCR_PIN_PX3 ((uint8_t)0x03) áAFIO_EVCR_PIN_PX4 ((uint8_t)0x04) àAFIO_EVCR_PIN_PX5 ((uint8_t)0x05) âAFIO_EVCR_PIN_PX6 ((uint8_t)0x06) äAFIO_EVCR_PIN_PX7 ((uint8_t)0x07) ãAFIO_EVCR_PIN_PX8 ((uint8_t)0x08) åAFIO_EVCR_PIN_PX9 ((uint8_t)0x09) çAFIO_EVCR_PIN_PX10 ((uint8_t)0x0A) éAFIO_EVCR_PIN_PX11 ((uint8_t)0x0B) èAFIO_EVCR_PIN_PX12 ((uint8_t)0x0C) êAFIO_EVCR_PIN_PX13 ((uint8_t)0x0D) ëAFIO_EVCR_PIN_PX14 ((uint8_t)0x0E) íAFIO_EVCR_PIN_PX15 ((uint8_t)0x0F) îAFIO_EVCR_PORT ((uint8_t)0x70) ïAFIO_EVCR_PORT_0 ((uint8_t)0x10) ñAFIO_EVCR_PORT_1 ((uint8_t)0x20) óAFIO_EVCR_PORT_2 ((uint8_t)0x40) öAFIO_EVCR_PORT_PA ((uint8_t)0x00) õAFIO_EVCR_PORT_PB ((uint8_t)0x10) úAFIO_EVCR_PORT_PC ((uint8_t)0x20) ùAFIO_EVCR_PORT_PD ((uint8_t)0x30) ûAFIO_EVCR_PORT_PE ((uint8_t)0x40) †AFIO_EVCR_EVOE ((uint8_t)0x80) £AFIO_MAPR_SPI1_REMAP ((uint32_t)0x00000001) §AFIO_MAPR_I2C1_REMAP ((uint32_t)0x00000002) •AFIO_MAPR_USART1_REMAP ((uint32_t)0x00000004) ¶AFIO_MAPR_USART2_REMAP ((uint32_t)0x00000008) ®AFIO_MAPR_USART3_REMAP ((uint32_t)0x00000030) ©AFIO_MAPR_USART3_REMAP_0 ((uint32_t)0x00000010) ™AFIO_MAPR_USART3_REMAP_1 ((uint32_t)0x00000020) ≠AFIO_MAPR_USART3_REMAP_NOREMAP ((uint32_t)0x00000000) ÆAFIO_MAPR_USART3_REMAP_PARTIALREMAP ((uint32_t)0x00000010) ØAFIO_MAPR_USART3_REMAP_FULLREMAP ((uint32_t)0x00000030) ±AFIO_MAPR_TIM1_REMAP ((uint32_t)0x000000C0) ≤AFIO_MAPR_TIM1_REMAP_0 ((uint32_t)0x00000040) ≥AFIO_MAPR_TIM1_REMAP_1 ((uint32_t)0x00000080) ∂AFIO_MAPR_TIM1_REMAP_NOREMAP ((uint32_t)0x00000000) ∑AFIO_MAPR_TIM1_REMAP_PARTIALREMAP ((uint32_t)0x00000040) ∏AFIO_MAPR_TIM1_REMAP_FULLREMAP ((uint32_t)0x000000C0) ∫AFIO_MAPR_TIM2_REMAP ((uint32_t)0x00000300) ªAFIO_MAPR_TIM2_REMAP_0 ((uint32_t)0x00000100) ºAFIO_MAPR_TIM2_REMAP_1 ((uint32_t)0x00000200) øAFIO_MAPR_TIM2_REMAP_NOREMAP ((uint32_t)0x00000000) ¿AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1 ((uint32_t)0x00000100) ¡AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2 ((uint32_t)0x00000200) ¬AFIO_MAPR_TIM2_REMAP_FULLREMAP ((uint32_t)0x00000300) ƒAFIO_MAPR_TIM3_REMAP ((uint32_t)0x00000C00) ≈AFIO_MAPR_TIM3_REMAP_0 ((uint32_t)0x00000400) ∆AFIO_MAPR_TIM3_REMAP_1 ((uint32_t)0x00000800) …AFIO_MAPR_TIM3_REMAP_NOREMAP ((uint32_t)0x00000000)  AFIO_MAPR_TIM3_REMAP_PARTIALREMAP ((uint32_t)0x00000800) ÀAFIO_MAPR_TIM3_REMAP_FULLREMAP ((uint32_t)0x00000C00) ÕAFIO_MAPR_TIM4_REMAP ((uint32_t)0x00001000) œAFIO_MAPR_CAN_REMAP ((uint32_t)0x00006000) –AFIO_MAPR_CAN_REMAP_0 ((uint32_t)0x00002000) —AFIO_MAPR_CAN_REMAP_1 ((uint32_t)0x00004000) ‘AFIO_MAPR_CAN_REMAP_REMAP1 ((uint32_t)0x00000000) ’AFIO_MAPR_CAN_REMAP_REMAP2 ((uint32_t)0x00004000) ÷AFIO_MAPR_CAN_REMAP_REMAP3 ((uint32_t)0x00006000) ÿAFIO_MAPR_PD01_REMAP ((uint32_t)0x00008000) ŸAFIO_MAPR_TIM5CH4_IREMAP ((uint32_t)0x00010000) ⁄AFIO_MAPR_ADC1_ETRGINJ_REMAP ((uint32_t)0x00020000) €AFIO_MAPR_ADC1_ETRGREG_REMAP ((uint32_t)0x00040000) ‹AFIO_MAPR_ADC2_ETRGINJ_REMAP ((uint32_t)0x00080000) ›AFIO_MAPR_ADC2_ETRGREG_REMAP ((uint32_t)0x00100000) ‡AFIO_MAPR_SWJ_CFG ((uint32_t)0x07000000) ·AFIO_MAPR_SWJ_CFG_0 ((uint32_t)0x01000000) ‚AFIO_MAPR_SWJ_CFG_1 ((uint32_t)0x02000000) „AFIO_MAPR_SWJ_CFG_2 ((uint32_t)0x04000000) ÂAFIO_MAPR_SWJ_CFG_RESET ((uint32_t)0x00000000) ÊAFIO_MAPR_SWJ_CFG_NOJNTRST ((uint32_t)0x01000000) ÁAFIO_MAPR_SWJ_CFG_JTAGDISABLE ((uint32_t)0x02000000) ËAFIO_MAPR_SWJ_CFG_DISABLE ((uint32_t)0x04000000) ÏAFIO_MAPR_ETH_REMAP ((uint32_t)0x00200000) ÔAFIO_MAPR_CAN2_REMAP ((uint32_t)0x00400000) ÚAFIO_MAPR_MII_RMII_SEL ((uint32_t)0x00800000) ıAFIO_MAPR_SPI3_REMAP ((uint32_t)0x10000000) ¯AFIO_MAPR_TIM2ITR1_IREMAP ((uint32_t)0x20000000) ˚AFIO_MAPR_PTP_PPS_REMAP ((uint32_t)0x40000000) ˇAFIO_EXTICR1_EXTI0 ((uint16_t)0x000F) ÄAFIO_EXTICR1_EXTI1 ((uint16_t)0x00F0) ÅAFIO_EXTICR1_EXTI2 ((uint16_t)0x0F00) ÇAFIO_EXTICR1_EXTI3 ((uint16_t)0xF000) ÖAFIO_EXTICR1_EXTI0_PA ((uint16_t)0x0000) ÜAFIO_EXTICR1_EXTI0_PB ((uint16_t)0x0001) áAFIO_EXTICR1_EXTI0_PC ((uint16_t)0x0002) àAFIO_EXTICR1_EXTI0_PD ((uint16_t)0x0003) âAFIO_EXTICR1_EXTI0_PE ((uint16_t)0x0004) äAFIO_EXTICR1_EXTI0_PF ((uint16_t)0x0005) ãAFIO_EXTICR1_EXTI0_PG ((uint16_t)0x0006) éAFIO_EXTICR1_EXTI1_PA ((uint16_t)0x0000) èAFIO_EXTICR1_EXTI1_PB ((uint16_t)0x0010) êAFIO_EXTICR1_EXTI1_PC ((uint16_t)0x0020) ëAFIO_EXTICR1_EXTI1_PD ((uint16_t)0x0030) íAFIO_EXTICR1_EXTI1_PE ((uint16_t)0x0040) ìAFIO_EXTICR1_EXTI1_PF ((uint16_t)0x0050) îAFIO_EXTICR1_EXTI1_PG ((uint16_t)0x0060) óAFIO_EXTICR1_EXTI2_PA ((uint16_t)0x0000) òAFIO_EXTICR1_EXTI2_PB ((uint16_t)0x0100) ôAFIO_EXTICR1_EXTI2_PC ((uint16_t)0x0200) öAFIO_EXTICR1_EXTI2_PD ((uint16_t)0x0300) õAFIO_EXTICR1_EXTI2_PE ((uint16_t)0x0400) úAFIO_EXTICR1_EXTI2_PF ((uint16_t)0x0500) ùAFIO_EXTICR1_EXTI2_PG ((uint16_t)0x0600) †AFIO_EXTICR1_EXTI3_PA ((uint16_t)0x0000) °AFIO_EXTICR1_EXTI3_PB ((uint16_t)0x1000) ¢AFIO_EXTICR1_EXTI3_PC ((uint16_t)0x2000) £AFIO_EXTICR1_EXTI3_PD ((uint16_t)0x3000) §AFIO_EXTICR1_EXTI3_PE ((uint16_t)0x4000) •AFIO_EXTICR1_EXTI3_PF ((uint16_t)0x5000) ¶AFIO_EXTICR1_EXTI3_PG ((uint16_t)0x6000) ©AFIO_EXTICR2_EXTI4 ((uint16_t)0x000F) ™AFIO_EXTICR2_EXTI5 ((uint16_t)0x00F0) ´AFIO_EXTICR2_EXTI6 ((uint16_t)0x0F00) ¨AFIO_EXTICR2_EXTI7 ((uint16_t)0xF000) ØAFIO_EXTICR2_EXTI4_PA ((uint16_t)0x0000) ∞AFIO_EXTICR2_EXTI4_PB ((uint16_t)0x0001) ±AFIO_EXTICR2_EXTI4_PC ((uint16_t)0x0002) ≤AFIO_EXTICR2_EXTI4_PD ((uint16_t)0x0003) ≥AFIO_EXTICR2_EXTI4_PE ((uint16_t)0x0004) ¥AFIO_EXTICR2_EXTI4_PF ((uint16_t)0x0005) µAFIO_EXTICR2_EXTI4_PG ((uint16_t)0x0006) ∏AFIO_EXTICR2_EXTI5_PA ((uint16_t)0x0000) πAFIO_EXTICR2_EXTI5_PB ((uint16_t)0x0010) ∫AFIO_EXTICR2_EXTI5_PC ((uint16_t)0x0020) ªAFIO_EXTICR2_EXTI5_PD ((uint16_t)0x0030) ºAFIO_EXTICR2_EXTI5_PE ((uint16_t)0x0040) ΩAFIO_EXTICR2_EXTI5_PF ((uint16_t)0x0050) æAFIO_EXTICR2_EXTI5_PG ((uint16_t)0x0060) ¡AFIO_EXTICR2_EXTI6_PA ((uint16_t)0x0000) ¬AFIO_EXTICR2_EXTI6_PB ((uint16_t)0x0100) √AFIO_EXTICR2_EXTI6_PC ((uint16_t)0x0200) ƒAFIO_EXTICR2_EXTI6_PD ((uint16_t)0x0300) ≈AFIO_EXTICR2_EXTI6_PE ((uint16_t)0x0400) ∆AFIO_EXTICR2_EXTI6_PF ((uint16_t)0x0500) «AFIO_EXTICR2_EXTI6_PG ((uint16_t)0x0600)  AFIO_EXTICR2_EXTI7_PA ((uint16_t)0x0000) ÀAFIO_EXTICR2_EXTI7_PB ((uint16_t)0x1000) ÃAFIO_EXTICR2_EXTI7_PC ((uint16_t)0x2000) ÕAFIO_EXTICR2_EXTI7_PD ((uint16_t)0x3000) ŒAFIO_EXTICR2_EXTI7_PE ((uint16_t)0x4000) œAFIO_EXTICR2_EXTI7_PF ((uint16_t)0x5000) –AFIO_EXTICR2_EXTI7_PG ((uint16_t)0x6000) ”AFIO_EXTICR3_EXTI8 ((uint16_t)0x000F) ‘AFIO_EXTICR3_EXTI9 ((uint16_t)0x00F0) ’AFIO_EXTICR3_EXTI10 ((uint16_t)0x0F00) ÷AFIO_EXTICR3_EXTI11 ((uint16_t)0xF000) ŸAFIO_EXTICR3_EXTI8_PA ((uint16_t)0x0000) ⁄AFIO_EXTICR3_EXTI8_PB ((uint16_t)0x0001) €AFIO_EXTICR3_EXTI8_PC ((uint16_t)0x0002) ‹AFIO_EXTICR3_EXTI8_PD ((uint16_t)0x0003) ›AFIO_EXTICR3_EXTI8_PE ((uint16_t)0x0004) ﬁAFIO_EXTICR3_EXTI8_PF ((uint16_t)0x0005) ﬂAFIO_EXTICR3_EXTI8_PG ((uint16_t)0x0006) ‚AFIO_EXTICR3_EXTI9_PA ((uint16_t)0x0000) „AFIO_EXTICR3_EXTI9_PB ((uint16_t)0x0010) ‰AFIO_EXTICR3_EXTI9_PC ((uint16_t)0x0020) ÂAFIO_EXTICR3_EXTI9_PD ((uint16_t)0x0030) ÊAFIO_EXTICR3_EXTI9_PE ((uint16_t)0x0040) ÁAFIO_EXTICR3_EXTI9_PF ((uint16_t)0x0050) ËAFIO_EXTICR3_EXTI9_PG ((uint16_t)0x0060) ÎAFIO_EXTICR3_EXTI10_PA ((uint16_t)0x0000) ÏAFIO_EXTICR3_EXTI10_PB ((uint16_t)0x0100) ÌAFIO_EXTICR3_EXTI10_PC ((uint16_t)0x0200) ÓAFIO_EXTICR3_EXTI10_PD ((uint16_t)0x0300) ÔAFIO_EXTICR3_EXTI10_PE ((uint16_t)0x0400) AFIO_EXTICR3_EXTI10_PF ((uint16_t)0x0500) ÒAFIO_EXTICR3_EXTI10_PG ((uint16_t)0x0600) ÙAFIO_EXTICR3_EXTI11_PA ((uint16_t)0x0000) ıAFIO_EXTICR3_EXTI11_PB ((uint16_t)0x1000) ˆAFIO_EXTICR3_EXTI11_PC ((uint16_t)0x2000) ˜AFIO_EXTICR3_EXTI11_PD ((uint16_t)0x3000) ¯AFIO_EXTICR3_EXTI11_PE ((uint16_t)0x4000) ˘AFIO_EXTICR3_EXTI11_PF ((uint16_t)0x5000) ˙AFIO_EXTICR3_EXTI11_PG ((uint16_t)0x6000) ˝AFIO_EXTICR4_EXTI12 ((uint16_t)0x000F) ˛AFIO_EXTICR4_EXTI13 ((uint16_t)0x00F0) ˇAFIO_EXTICR4_EXTI14 ((uint16_t)0x0F00) ÄAFIO_EXTICR4_EXTI15 ((uint16_t)0xF000) ÉAFIO_EXTICR4_EXTI12_PA ((uint16_t)0x0000) ÑAFIO_EXTICR4_EXTI12_PB ((uint16_t)0x0001) ÖAFIO_EXTICR4_EXTI12_PC ((uint16_t)0x0002) ÜAFIO_EXTICR4_EXTI12_PD ((uint16_t)0x0003) áAFIO_EXTICR4_EXTI12_PE ((uint16_t)0x0004) àAFIO_EXTICR4_EXTI12_PF ((uint16_t)0x0005) âAFIO_EXTICR4_EXTI12_PG ((uint16_t)0x0006) åAFIO_EXTICR4_EXTI13_PA ((uint16_t)0x0000) çAFIO_EXTICR4_EXTI13_PB ((uint16_t)0x0010) éAFIO_EXTICR4_EXTI13_PC ((uint16_t)0x0020) èAFIO_EXTICR4_EXTI13_PD ((uint16_t)0x0030) êAFIO_EXTICR4_EXTI13_PE ((uint16_t)0x0040) ëAFIO_EXTICR4_EXTI13_PF ((uint16_t)0x0050) íAFIO_EXTICR4_EXTI13_PG ((uint16_t)0x0060) ïAFIO_EXTICR4_EXTI14_PA ((uint16_t)0x0000) ñAFIO_EXTICR4_EXTI14_PB ((uint16_t)0x0100) óAFIO_EXTICR4_EXTI14_PC ((uint16_t)0x0200) òAFIO_EXTICR4_EXTI14_PD ((uint16_t)0x0300) ôAFIO_EXTICR4_EXTI14_PE ((uint16_t)0x0400) öAFIO_EXTICR4_EXTI14_PF ((uint16_t)0x0500) õAFIO_EXTICR4_EXTI14_PG ((uint16_t)0x0600) ûAFIO_EXTICR4_EXTI15_PA ((uint16_t)0x0000) üAFIO_EXTICR4_EXTI15_PB ((uint16_t)0x1000) †AFIO_EXTICR4_EXTI15_PC ((uint16_t)0x2000) °AFIO_EXTICR4_EXTI15_PD ((uint16_t)0x3000) ¢AFIO_EXTICR4_EXTI15_PE ((uint16_t)0x4000) £AFIO_EXTICR4_EXTI15_PF ((uint16_t)0x5000) §AFIO_EXTICR4_EXTI15_PG ((uint16_t)0x6000) …SysTick_CTRL_ENABLE ((uint32_t)0x00000001)  SysTick_CTRL_TICKINT ((uint32_t)0x00000002) ÀSysTick_CTRL_CLKSOURCE ((uint32_t)0x00000004) ÃSysTick_CTRL_COUNTFLAG ((uint32_t)0x00010000) œSysTick_LOAD_RELOAD ((uint32_t)0x00FFFFFF) “SysTick_VAL_CURRENT ((uint32_t)0x00FFFFFF) ’SysTick_CALIB_TENMS ((uint32_t)0x00FFFFFF) ÷SysTick_CALIB_SKEW ((uint32_t)0x40000000) ◊SysTick_CALIB_NOREF ((uint32_t)0x80000000) ‡NVIC_ISER_SETENA ((uint32_t)0xFFFFFFFF) ·NVIC_ISER_SETENA_0 ((uint32_t)0x00000001) ‚NVIC_ISER_SETENA_1 ((uint32_t)0x00000002) „NVIC_ISER_SETENA_2 ((uint32_t)0x00000004) ‰NVIC_ISER_SETENA_3 ((uint32_t)0x00000008) ÂNVIC_ISER_SETENA_4 ((uint32_t)0x00000010) ÊNVIC_ISER_SETENA_5 ((uint32_t)0x00000020) ÁNVIC_ISER_SETENA_6 ((uint32_t)0x00000040) ËNVIC_ISER_SETENA_7 ((uint32_t)0x00000080) ÈNVIC_ISER_SETENA_8 ((uint32_t)0x00000100) ÍNVIC_ISER_SETENA_9 ((uint32_t)0x00000200) ÎNVIC_ISER_SETENA_10 ((uint32_t)0x00000400) ÏNVIC_ISER_SETENA_11 ((uint32_t)0x00000800) ÌNVIC_ISER_SETENA_12 ((uint32_t)0x00001000) ÓNVIC_ISER_SETENA_13 ((uint32_t)0x00002000) ÔNVIC_ISER_SETENA_14 ((uint32_t)0x00004000) NVIC_ISER_SETENA_15 ((uint32_t)0x00008000) ÒNVIC_ISER_SETENA_16 ((uint32_t)0x00010000) ÚNVIC_ISER_SETENA_17 ((uint32_t)0x00020000) ÛNVIC_ISER_SETENA_18 ((uint32_t)0x00040000) ÙNVIC_ISER_SETENA_19 ((uint32_t)0x00080000) ıNVIC_ISER_SETENA_20 ((uint32_t)0x00100000) ˆNVIC_ISER_SETENA_21 ((uint32_t)0x00200000) ˜NVIC_ISER_SETENA_22 ((uint32_t)0x00400000) ¯NVIC_ISER_SETENA_23 ((uint32_t)0x00800000) ˘NVIC_ISER_SETENA_24 ((uint32_t)0x01000000) ˙NVIC_ISER_SETENA_25 ((uint32_t)0x02000000) ˚NVIC_ISER_SETENA_26 ((uint32_t)0x04000000) ¸NVIC_ISER_SETENA_27 ((uint32_t)0x08000000) ˝NVIC_ISER_SETENA_28 ((uint32_t)0x10000000) ˛NVIC_ISER_SETENA_29 ((uint32_t)0x20000000) ˇNVIC_ISER_SETENA_30 ((uint32_t)0x40000000) ÄNVIC_ISER_SETENA_31 ((uint32_t)0x80000000) ÉNVIC_ICER_CLRENA ((uint32_t)0xFFFFFFFF) ÑNVIC_ICER_CLRENA_0 ((uint32_t)0x00000001) ÖNVIC_ICER_CLRENA_1 ((uint32_t)0x00000002) ÜNVIC_ICER_CLRENA_2 ((uint32_t)0x00000004) áNVIC_ICER_CLRENA_3 ((uint32_t)0x00000008) àNVIC_ICER_CLRENA_4 ((uint32_t)0x00000010) âNVIC_ICER_CLRENA_5 ((uint32_t)0x00000020) äNVIC_ICER_CLRENA_6 ((uint32_t)0x00000040) ãNVIC_ICER_CLRENA_7 ((uint32_t)0x00000080) åNVIC_ICER_CLRENA_8 ((uint32_t)0x00000100) çNVIC_ICER_CLRENA_9 ((uint32_t)0x00000200) éNVIC_ICER_CLRENA_10 ((uint32_t)0x00000400) èNVIC_ICER_CLRENA_11 ((uint32_t)0x00000800) êNVIC_ICER_CLRENA_12 ((uint32_t)0x00001000) ëNVIC_ICER_CLRENA_13 ((uint32_t)0x00002000) íNVIC_ICER_CLRENA_14 ((uint32_t)0x00004000) ìNVIC_ICER_CLRENA_15 ((uint32_t)0x00008000) îNVIC_ICER_CLRENA_16 ((uint32_t)0x00010000) ïNVIC_ICER_CLRENA_17 ((uint32_t)0x00020000) ñNVIC_ICER_CLRENA_18 ((uint32_t)0x00040000) óNVIC_ICER_CLRENA_19 ((uint32_t)0x00080000) òNVIC_ICER_CLRENA_20 ((uint32_t)0x00100000) ôNVIC_ICER_CLRENA_21 ((uint32_t)0x00200000) öNVIC_ICER_CLRENA_22 ((uint32_t)0x00400000) õNVIC_ICER_CLRENA_23 ((uint32_t)0x00800000) úNVIC_ICER_CLRENA_24 ((uint32_t)0x01000000) ùNVIC_ICER_CLRENA_25 ((uint32_t)0x02000000) ûNVIC_ICER_CLRENA_26 ((uint32_t)0x04000000) üNVIC_ICER_CLRENA_27 ((uint32_t)0x08000000) †NVIC_ICER_CLRENA_28 ((uint32_t)0x10000000) °NVIC_ICER_CLRENA_29 ((uint32_t)0x20000000) ¢NVIC_ICER_CLRENA_30 ((uint32_t)0x40000000) £NVIC_ICER_CLRENA_31 ((uint32_t)0x80000000) ¶NVIC_ISPR_SETPEND ((uint32_t)0xFFFFFFFF) ßNVIC_ISPR_SETPEND_0 ((uint32_t)0x00000001) ®NVIC_ISPR_SETPEND_1 ((uint32_t)0x00000002) ©NVIC_ISPR_SETPEND_2 ((uint32_t)0x00000004) ™NVIC_ISPR_SETPEND_3 ((uint32_t)0x00000008) ´NVIC_ISPR_SETPEND_4 ((uint32_t)0x00000010) ¨NVIC_ISPR_SETPEND_5 ((uint32_t)0x00000020) ≠NVIC_ISPR_SETPEND_6 ((uint32_t)0x00000040) ÆNVIC_ISPR_SETPEND_7 ((uint32_t)0x00000080) ØNVIC_ISPR_SETPEND_8 ((uint32_t)0x00000100) ∞NVIC_ISPR_SETPEND_9 ((uint32_t)0x00000200) ±NVIC_ISPR_SETPEND_10 ((uint32_t)0x00000400) ≤NVIC_ISPR_SETPEND_11 ((uint32_t)0x00000800) ≥NVIC_ISPR_SETPEND_12 ((uint32_t)0x00001000) ¥NVIC_ISPR_SETPEND_13 ((uint32_t)0x00002000) µNVIC_ISPR_SETPEND_14 ((uint32_t)0x00004000) ∂NVIC_ISPR_SETPEND_15 ((uint32_t)0x00008000) ∑NVIC_ISPR_SETPEND_16 ((uint32_t)0x00010000) ∏NVIC_ISPR_SETPEND_17 ((uint32_t)0x00020000) πNVIC_ISPR_SETPEND_18 ((uint32_t)0x00040000) ∫NVIC_ISPR_SETPEND_19 ((uint32_t)0x00080000) ªNVIC_ISPR_SETPEND_20 ((uint32_t)0x00100000) ºNVIC_ISPR_SETPEND_21 ((uint32_t)0x00200000) ΩNVIC_ISPR_SETPEND_22 ((uint32_t)0x00400000) æNVIC_ISPR_SETPEND_23 ((uint32_t)0x00800000) øNVIC_ISPR_SETPEND_24 ((uint32_t)0x01000000) ¿NVIC_ISPR_SETPEND_25 ((uint32_t)0x02000000) ¡NVIC_ISPR_SETPEND_26 ((uint32_t)0x04000000) ¬NVIC_ISPR_SETPEND_27 ((uint32_t)0x08000000) √NVIC_ISPR_SETPEND_28 ((uint32_t)0x10000000) ƒNVIC_ISPR_SETPEND_29 ((uint32_t)0x20000000) ≈NVIC_ISPR_SETPEND_30 ((uint32_t)0x40000000) ∆NVIC_ISPR_SETPEND_31 ((uint32_t)0x80000000) …NVIC_ICPR_CLRPEND ((uint32_t)0xFFFFFFFF)  NVIC_ICPR_CLRPEND_0 ((uint32_t)0x00000001) ÀNVIC_ICPR_CLRPEND_1 ((uint32_t)0x00000002) ÃNVIC_ICPR_CLRPEND_2 ((uint32_t)0x00000004) ÕNVIC_ICPR_CLRPEND_3 ((uint32_t)0x00000008) ŒNVIC_ICPR_CLRPEND_4 ((uint32_t)0x00000010) œNVIC_ICPR_CLRPEND_5 ((uint32_t)0x00000020) –NVIC_ICPR_CLRPEND_6 ((uint32_t)0x00000040) —NVIC_ICPR_CLRPEND_7 ((uint32_t)0x00000080) “NVIC_ICPR_CLRPEND_8 ((uint32_t)0x00000100) ”NVIC_ICPR_CLRPEND_9 ((uint32_t)0x00000200) ‘NVIC_ICPR_CLRPEND_10 ((uint32_t)0x00000400) ’NVIC_ICPR_CLRPEND_11 ((uint32_t)0x00000800) ÷NVIC_ICPR_CLRPEND_12 ((uint32_t)0x00001000) ◊NVIC_ICPR_CLRPEND_13 ((uint32_t)0x00002000) ÿNVIC_ICPR_CLRPEND_14 ((uint32_t)0x00004000) ŸNVIC_ICPR_CLRPEND_15 ((uint32_t)0x00008000) ⁄NVIC_ICPR_CLRPEND_16 ((uint32_t)0x00010000) €NVIC_ICPR_CLRPEND_17 ((uint32_t)0x00020000) ‹NVIC_ICPR_CLRPEND_18 ((uint32_t)0x00040000) ›NVIC_ICPR_CLRPEND_19 ((uint32_t)0x00080000) ﬁNVIC_ICPR_CLRPEND_20 ((uint32_t)0x00100000) ﬂNVIC_ICPR_CLRPEND_21 ((uint32_t)0x00200000) ‡NVIC_ICPR_CLRPEND_22 ((uint32_t)0x00400000) ·NVIC_ICPR_CLRPEND_23 ((uint32_t)0x00800000) ‚NVIC_ICPR_CLRPEND_24 ((uint32_t)0x01000000) „NVIC_ICPR_CLRPEND_25 ((uint32_t)0x02000000) ‰NVIC_ICPR_CLRPEND_26 ((uint32_t)0x04000000) ÂNVIC_ICPR_CLRPEND_27 ((uint32_t)0x08000000) ÊNVIC_ICPR_CLRPEND_28 ((uint32_t)0x10000000) ÁNVIC_ICPR_CLRPEND_29 ((uint32_t)0x20000000) ËNVIC_ICPR_CLRPEND_30 ((uint32_t)0x40000000) ÈNVIC_ICPR_CLRPEND_31 ((uint32_t)0x80000000) ÏNVIC_IABR_ACTIVE ((uint32_t)0xFFFFFFFF) ÌNVIC_IABR_ACTIVE_0 ((uint32_t)0x00000001) ÓNVIC_IABR_ACTIVE_1 ((uint32_t)0x00000002) ÔNVIC_IABR_ACTIVE_2 ((uint32_t)0x00000004) NVIC_IABR_ACTIVE_3 ((uint32_t)0x00000008) ÒNVIC_IABR_ACTIVE_4 ((uint32_t)0x00000010) ÚNVIC_IABR_ACTIVE_5 ((uint32_t)0x00000020) ÛNVIC_IABR_ACTIVE_6 ((uint32_t)0x00000040) ÙNVIC_IABR_ACTIVE_7 ((uint32_t)0x00000080) ıNVIC_IABR_ACTIVE_8 ((uint32_t)0x00000100) ˆNVIC_IABR_ACTIVE_9 ((uint32_t)0x00000200) ˜NVIC_IABR_ACTIVE_10 ((uint32_t)0x00000400) ¯NVIC_IABR_ACTIVE_11 ((uint32_t)0x00000800) ˘NVIC_IABR_ACTIVE_12 ((uint32_t)0x00001000) ˙NVIC_IABR_ACTIVE_13 ((uint32_t)0x00002000) ˚NVIC_IABR_ACTIVE_14 ((uint32_t)0x00004000) ¸NVIC_IABR_ACTIVE_15 ((uint32_t)0x00008000) ˝NVIC_IABR_ACTIVE_16 ((uint32_t)0x00010000) ˛NVIC_IABR_ACTIVE_17 ((uint32_t)0x00020000) ˇNVIC_IABR_ACTIVE_18 ((uint32_t)0x00040000) ÄNVIC_IABR_ACTIVE_19 ((uint32_t)0x00080000) ÅNVIC_IABR_ACTIVE_20 ((uint32_t)0x00100000) ÇNVIC_IABR_ACTIVE_21 ((uint32_t)0x00200000) ÉNVIC_IABR_ACTIVE_22 ((uint32_t)0x00400000) ÑNVIC_IABR_ACTIVE_23 ((uint32_t)0x00800000) ÖNVIC_IABR_ACTIVE_24 ((uint32_t)0x01000000) ÜNVIC_IABR_ACTIVE_25 ((uint32_t)0x02000000) áNVIC_IABR_ACTIVE_26 ((uint32_t)0x04000000) àNVIC_IABR_ACTIVE_27 ((uint32_t)0x08000000) âNVIC_IABR_ACTIVE_28 ((uint32_t)0x10000000) äNVIC_IABR_ACTIVE_29 ((uint32_t)0x20000000) ãNVIC_IABR_ACTIVE_30 ((uint32_t)0x40000000) åNVIC_IABR_ACTIVE_31 ((uint32_t)0x80000000) èNVIC_IPR0_PRI_0 ((uint32_t)0x000000FF) êNVIC_IPR0_PRI_1 ((uint32_t)0x0000FF00) ëNVIC_IPR0_PRI_2 ((uint32_t)0x00FF0000) íNVIC_IPR0_PRI_3 ((uint32_t)0xFF000000) ïNVIC_IPR1_PRI_4 ((uint32_t)0x000000FF) ñNVIC_IPR1_PRI_5 ((uint32_t)0x0000FF00) óNVIC_IPR1_PRI_6 ((uint32_t)0x00FF0000) òNVIC_IPR1_PRI_7 ((uint32_t)0xFF000000) õNVIC_IPR2_PRI_8 ((uint32_t)0x000000FF) úNVIC_IPR2_PRI_9 ((uint32_t)0x0000FF00) ùNVIC_IPR2_PRI_10 ((uint32_t)0x00FF0000) ûNVIC_IPR2_PRI_11 ((uint32_t)0xFF000000) °NVIC_IPR3_PRI_12 ((uint32_t)0x000000FF) ¢NVIC_IPR3_PRI_13 ((uint32_t)0x0000FF00) £NVIC_IPR3_PRI_14 ((uint32_t)0x00FF0000) §NVIC_IPR3_PRI_15 ((uint32_t)0xFF000000) ßNVIC_IPR4_PRI_16 ((uint32_t)0x000000FF) ®NVIC_IPR4_PRI_17 ((uint32_t)0x0000FF00) ©NVIC_IPR4_PRI_18 ((uint32_t)0x00FF0000) ™NVIC_IPR4_PRI_19 ((uint32_t)0xFF000000) ≠NVIC_IPR5_PRI_20 ((uint32_t)0x000000FF) ÆNVIC_IPR5_PRI_21 ((uint32_t)0x0000FF00) ØNVIC_IPR5_PRI_22 ((uint32_t)0x00FF0000) ∞NVIC_IPR5_PRI_23 ((uint32_t)0xFF000000) ≥NVIC_IPR6_PRI_24 ((uint32_t)0x000000FF) ¥NVIC_IPR6_PRI_25 ((uint32_t)0x0000FF00) µNVIC_IPR6_PRI_26 ((uint32_t)0x00FF0000) ∂NVIC_IPR6_PRI_27 ((uint32_t)0xFF000000) πNVIC_IPR7_PRI_28 ((uint32_t)0x000000FF) ∫NVIC_IPR7_PRI_29 ((uint32_t)0x0000FF00) ªNVIC_IPR7_PRI_30 ((uint32_t)0x00FF0000) ºNVIC_IPR7_PRI_31 ((uint32_t)0xFF000000) øSCB_CPUID_REVISION ((uint32_t)0x0000000F) ¿SCB_CPUID_PARTNO ((uint32_t)0x0000FFF0) ¡SCB_CPUID_Constant ((uint32_t)0x000F0000) ¬SCB_CPUID_VARIANT ((uint32_t)0x00F00000) √SCB_CPUID_IMPLEMENTER ((uint32_t)0xFF000000) ∆SCB_ICSR_VECTACTIVE ((uint32_t)0x000001FF) «SCB_ICSR_RETTOBASE ((uint32_t)0x00000800) »SCB_ICSR_VECTPENDING ((uint32_t)0x003FF000) …SCB_ICSR_ISRPENDING ((uint32_t)0x00400000)  SCB_ICSR_ISRPREEMPT ((uint32_t)0x00800000) ÀSCB_ICSR_PENDSTCLR ((uint32_t)0x02000000) ÃSCB_ICSR_PENDSTSET ((uint32_t)0x04000000) ÕSCB_ICSR_PENDSVCLR ((uint32_t)0x08000000) ŒSCB_ICSR_PENDSVSET ((uint32_t)0x10000000) œSCB_ICSR_NMIPENDSET ((uint32_t)0x80000000) “SCB_VTOR_TBLOFF ((uint32_t)0x1FFFFF80) ”SCB_VTOR_TBLBASE ((uint32_t)0x20000000) ÷SCB_AIRCR_VECTRESET ((uint32_t)0x00000001) ◊SCB_AIRCR_VECTCLRACTIVE ((uint32_t)0x00000002) ÿSCB_AIRCR_SYSRESETREQ ((uint32_t)0x00000004) ⁄SCB_AIRCR_PRIGROUP ((uint32_t)0x00000700) €SCB_AIRCR_PRIGROUP_0 ((uint32_t)0x00000100) ‹SCB_AIRCR_PRIGROUP_1 ((uint32_t)0x00000200) ›SCB_AIRCR_PRIGROUP_2 ((uint32_t)0x00000400) ‡SCB_AIRCR_PRIGROUP0 ((uint32_t)0x00000000) ·SCB_AIRCR_PRIGROUP1 ((uint32_t)0x00000100) ‚SCB_AIRCR_PRIGROUP2 ((uint32_t)0x00000200) „SCB_AIRCR_PRIGROUP3 ((uint32_t)0x00000300) ‰SCB_AIRCR_PRIGROUP4 ((uint32_t)0x00000400) ÂSCB_AIRCR_PRIGROUP5 ((uint32_t)0x00000500) ÊSCB_AIRCR_PRIGROUP6 ((uint32_t)0x00000600) ÁSCB_AIRCR_PRIGROUP7 ((uint32_t)0x00000700) ÈSCB_AIRCR_ENDIANESS ((uint32_t)0x00008000) ÍSCB_AIRCR_VECTKEY ((uint32_t)0xFFFF0000) ÌSCB_SCR_SLEEPONEXIT ((uint8_t)0x02) ÓSCB_SCR_SLEEPDEEP ((uint8_t)0x04) ÔSCB_SCR_SEVONPEND ((uint8_t)0x10) ÚSCB_CCR_NONBASETHRDENA ((uint16_t)0x0001) ÛSCB_CCR_USERSETMPEND ((uint16_t)0x0002) ÙSCB_CCR_UNALIGN_TRP ((uint16_t)0x0008) ıSCB_CCR_DIV_0_TRP ((uint16_t)0x0010) ˆSCB_CCR_BFHFNMIGN ((uint16_t)0x0100) ˜SCB_CCR_STKALIGN ((uint16_t)0x0200) ˙SCB_SHPR_PRI_N ((uint32_t)0x000000FF) ˚SCB_SHPR_PRI_N1 ((uint32_t)0x0000FF00) ¸SCB_SHPR_PRI_N2 ((uint32_t)0x00FF0000) ˝SCB_SHPR_PRI_N3 ((uint32_t)0xFF000000) ÄSCB_SHCSR_MEMFAULTACT ((uint32_t)0x00000001) ÅSCB_SHCSR_BUSFAULTACT ((uint32_t)0x00000002) ÇSCB_SHCSR_USGFAULTACT ((uint32_t)0x00000008) ÉSCB_SHCSR_SVCALLACT ((uint32_t)0x00000080) ÑSCB_SHCSR_MONITORACT ((uint32_t)0x00000100) ÖSCB_SHCSR_PENDSVACT ((uint32_t)0x00000400) ÜSCB_SHCSR_SYSTICKACT ((uint32_t)0x00000800) áSCB_SHCSR_USGFAULTPENDED ((uint32_t)0x00001000) àSCB_SHCSR_MEMFAULTPENDED ((uint32_t)0x00002000) âSCB_SHCSR_BUSFAULTPENDED ((uint32_t)0x00004000) äSCB_SHCSR_SVCALLPENDED ((uint32_t)0x00008000) ãSCB_SHCSR_MEMFAULTENA ((uint32_t)0x00010000) åSCB_SHCSR_BUSFAULTENA ((uint32_t)0x00020000) çSCB_SHCSR_USGFAULTENA ((uint32_t)0x00040000) ëSCB_CFSR_IACCVIOL ((uint32_t)0x00000001) íSCB_CFSR_DACCVIOL ((uint32_t)0x00000002) ìSCB_CFSR_MUNSTKERR ((uint32_t)0x00000008) îSCB_CFSR_MSTKERR ((uint32_t)0x00000010) ïSCB_CFSR_MMARVALID ((uint32_t)0x00000080) óSCB_CFSR_IBUSERR ((uint32_t)0x00000100) òSCB_CFSR_PRECISERR ((uint32_t)0x00000200) ôSCB_CFSR_IMPRECISERR ((uint32_t)0x00000400) öSCB_CFSR_UNSTKERR ((uint32_t)0x00000800) õSCB_CFSR_STKERR ((uint32_t)0x00001000) úSCB_CFSR_BFARVALID ((uint32_t)0x00008000) ûSCB_CFSR_UNDEFINSTR ((uint32_t)0x00010000) üSCB_CFSR_INVSTATE ((uint32_t)0x00020000) †SCB_CFSR_INVPC ((uint32_t)0x00040000) °SCB_CFSR_NOCP ((uint32_t)0x00080000) ¢SCB_CFSR_UNALIGNED ((uint32_t)0x01000000) £SCB_CFSR_DIVBYZERO ((uint32_t)0x02000000) ¶SCB_HFSR_VECTTBL ((uint32_t)0x00000002) ßSCB_HFSR_FORCED ((uint32_t)0x40000000) ®SCB_HFSR_DEBUGEVT ((uint32_t)0x80000000) ´SCB_DFSR_HALTED ((uint8_t)0x01) ¨SCB_DFSR_BKPT ((uint8_t)0x02) ≠SCB_DFSR_DWTTRAP ((uint8_t)0x04) ÆSCB_DFSR_VCATCH ((uint8_t)0x08) ØSCB_DFSR_EXTERNAL ((uint8_t)0x10) ≤SCB_MMFAR_ADDRESS ((uint32_t)0xFFFFFFFF) µSCB_BFAR_ADDRESS ((uint32_t)0xFFFFFFFF) ∏SCB_AFSR_IMPDEF ((uint32_t)0xFFFFFFFF) ¡EXTI_IMR_MR0 ((uint32_t)0x00000001) ¬EXTI_IMR_MR1 ((uint32_t)0x00000002) √EXTI_IMR_MR2 ((uint32_t)0x00000004) ƒEXTI_IMR_MR3 ((uint32_t)0x00000008) ≈EXTI_IMR_MR4 ((uint32_t)0x00000010) ∆EXTI_IMR_MR5 ((uint32_t)0x00000020) «EXTI_IMR_MR6 ((uint32_t)0x00000040) »EXTI_IMR_MR7 ((uint32_t)0x00000080) …EXTI_IMR_MR8 ((uint32_t)0x00000100)  EXTI_IMR_MR9 ((uint32_t)0x00000200) ÀEXTI_IMR_MR10 ((uint32_t)0x00000400) ÃEXTI_IMR_MR11 ((uint32_t)0x00000800) ÕEXTI_IMR_MR12 ((uint32_t)0x00001000) ŒEXTI_IMR_MR13 ((uint32_t)0x00002000) œEXTI_IMR_MR14 ((uint32_t)0x00004000) –EXTI_IMR_MR15 ((uint32_t)0x00008000) —EXTI_IMR_MR16 ((uint32_t)0x00010000) “EXTI_IMR_MR17 ((uint32_t)0x00020000) ”EXTI_IMR_MR18 ((uint32_t)0x00040000) ‘EXTI_IMR_MR19 ((uint32_t)0x00080000) ◊EXTI_EMR_MR0 ((uint32_t)0x00000001) ÿEXTI_EMR_MR1 ((uint32_t)0x00000002) ŸEXTI_EMR_MR2 ((uint32_t)0x00000004) ⁄EXTI_EMR_MR3 ((uint32_t)0x00000008) €EXTI_EMR_MR4 ((uint32_t)0x00000010) ‹EXTI_EMR_MR5 ((uint32_t)0x00000020) ›EXTI_EMR_MR6 ((uint32_t)0x00000040) ﬁEXTI_EMR_MR7 ((uint32_t)0x00000080) ﬂEXTI_EMR_MR8 ((uint32_t)0x00000100) ‡EXTI_EMR_MR9 ((uint32_t)0x00000200) ·EXTI_EMR_MR10 ((uint32_t)0x00000400) ‚EXTI_EMR_MR11 ((uint32_t)0x00000800) „EXTI_EMR_MR12 ((uint32_t)0x00001000) ‰EXTI_EMR_MR13 ((uint32_t)0x00002000) ÂEXTI_EMR_MR14 ((uint32_t)0x00004000) ÊEXTI_EMR_MR15 ((uint32_t)0x00008000) ÁEXTI_EMR_MR16 ((uint32_t)0x00010000) ËEXTI_EMR_MR17 ((uint32_t)0x00020000) ÈEXTI_EMR_MR18 ((uint32_t)0x00040000) ÍEXTI_EMR_MR19 ((uint32_t)0x00080000) ÌEXTI_RTSR_TR0 ((uint32_t)0x00000001) ÓEXTI_RTSR_TR1 ((uint32_t)0x00000002) ÔEXTI_RTSR_TR2 ((uint32_t)0x00000004) EXTI_RTSR_TR3 ((uint32_t)0x00000008) ÒEXTI_RTSR_TR4 ((uint32_t)0x00000010) ÚEXTI_RTSR_TR5 ((uint32_t)0x00000020) ÛEXTI_RTSR_TR6 ((uint32_t)0x00000040) ÙEXTI_RTSR_TR7 ((uint32_t)0x00000080) ıEXTI_RTSR_TR8 ((uint32_t)0x00000100) ˆEXTI_RTSR_TR9 ((uint32_t)0x00000200) ˜EXTI_RTSR_TR10 ((uint32_t)0x00000400) ¯EXTI_RTSR_TR11 ((uint32_t)0x00000800) ˘EXTI_RTSR_TR12 ((uint32_t)0x00001000) ˙EXTI_RTSR_TR13 ((uint32_t)0x00002000) ˚EXTI_RTSR_TR14 ((uint32_t)0x00004000) ¸EXTI_RTSR_TR15 ((uint32_t)0x00008000) ˝EXTI_RTSR_TR16 ((uint32_t)0x00010000) ˛EXTI_RTSR_TR17 ((uint32_t)0x00020000) ˇEXTI_RTSR_TR18 ((uint32_t)0x00040000) ÄEXTI_RTSR_TR19 ((uint32_t)0x00080000) ÉEXTI_FTSR_TR0 ((uint32_t)0x00000001) ÑEXTI_FTSR_TR1 ((uint32_t)0x00000002) ÖEXTI_FTSR_TR2 ((uint32_t)0x00000004) ÜEXTI_FTSR_TR3 ((uint32_t)0x00000008) áEXTI_FTSR_TR4 ((uint32_t)0x00000010) àEXTI_FTSR_TR5 ((uint32_t)0x00000020) âEXTI_FTSR_TR6 ((uint32_t)0x00000040) äEXTI_FTSR_TR7 ((uint32_t)0x00000080) ãEXTI_FTSR_TR8 ((uint32_t)0x00000100) åEXTI_FTSR_TR9 ((uint32_t)0x00000200) çEXTI_FTSR_TR10 ((uint32_t)0x00000400) éEXTI_FTSR_TR11 ((uint32_t)0x00000800) èEXTI_FTSR_TR12 ((uint32_t)0x00001000) êEXTI_FTSR_TR13 ((uint32_t)0x00002000) ëEXTI_FTSR_TR14 ((uint32_t)0x00004000) íEXTI_FTSR_TR15 ((uint32_t)0x00008000) ìEXTI_FTSR_TR16 ((uint32_t)0x00010000) îEXTI_FTSR_TR17 ((uint32_t)0x00020000) ïEXTI_FTSR_TR18 ((uint32_t)0x00040000) ñEXTI_FTSR_TR19 ((uint32_t)0x00080000) ôEXTI_SWIER_SWIER0 ((uint32_t)0x00000001) öEXTI_SWIER_SWIER1 ((uint32_t)0x00000002) õEXTI_SWIER_SWIER2 ((uint32_t)0x00000004) úEXTI_SWIER_SWIER3 ((uint32_t)0x00000008) ùEXTI_SWIER_SWIER4 ((uint32_t)0x00000010) ûEXTI_SWIER_SWIER5 ((uint32_t)0x00000020) üEXTI_SWIER_SWIER6 ((uint32_t)0x00000040) †EXTI_SWIER_SWIER7 ((uint32_t)0x00000080) °EXTI_SWIER_SWIER8 ((uint32_t)0x00000100) ¢EXTI_SWIER_SWIER9 ((uint32_t)0x00000200) £EXTI_SWIER_SWIER10 ((uint32_t)0x00000400) §EXTI_SWIER_SWIER11 ((uint32_t)0x00000800) •EXTI_SWIER_SWIER12 ((uint32_t)0x00001000) ¶EXTI_SWIER_SWIER13 ((uint32_t)0x00002000) ßEXTI_SWIER_SWIER14 ((uint32_t)0x00004000) ®EXTI_SWIER_SWIER15 ((uint32_t)0x00008000) ©EXTI_SWIER_SWIER16 ((uint32_t)0x00010000) ™EXTI_SWIER_SWIER17 ((uint32_t)0x00020000) ´EXTI_SWIER_SWIER18 ((uint32_t)0x00040000) ¨EXTI_SWIER_SWIER19 ((uint32_t)0x00080000) ØEXTI_PR_PR0 ((uint32_t)0x00000001) ∞EXTI_PR_PR1 ((uint32_t)0x00000002) ±EXTI_PR_PR2 ((uint32_t)0x00000004) ≤EXTI_PR_PR3 ((uint32_t)0x00000008) ≥EXTI_PR_PR4 ((uint32_t)0x00000010) ¥EXTI_PR_PR5 ((uint32_t)0x00000020) µEXTI_PR_PR6 ((uint32_t)0x00000040) ∂EXTI_PR_PR7 ((uint32_t)0x00000080) ∑EXTI_PR_PR8 ((uint32_t)0x00000100) ∏EXTI_PR_PR9 ((uint32_t)0x00000200) πEXTI_PR_PR10 ((uint32_t)0x00000400) ∫EXTI_PR_PR11 ((uint32_t)0x00000800) ªEXTI_PR_PR12 ((uint32_t)0x00001000) ºEXTI_PR_PR13 ((uint32_t)0x00002000) ΩEXTI_PR_PR14 ((uint32_t)0x00004000) æEXTI_PR_PR15 ((uint32_t)0x00008000) øEXTI_PR_PR16 ((uint32_t)0x00010000) ¿EXTI_PR_PR17 ((uint32_t)0x00020000) ¡EXTI_PR_PR18 ((uint32_t)0x00040000) ¬EXTI_PR_PR19 ((uint32_t)0x00080000) ÀDMA_ISR_GIF1 ((uint32_t)0x00000001) ÃDMA_ISR_TCIF1 ((uint32_t)0x00000002) ÕDMA_ISR_HTIF1 ((uint32_t)0x00000004) ŒDMA_ISR_TEIF1 ((uint32_t)0x00000008) œDMA_ISR_GIF2 ((uint32_t)0x00000010) –DMA_ISR_TCIF2 ((uint32_t)0x00000020) —DMA_ISR_HTIF2 ((uint32_t)0x00000040) “DMA_ISR_TEIF2 ((uint32_t)0x00000080) ”DMA_ISR_GIF3 ((uint32_t)0x00000100) ‘DMA_ISR_TCIF3 ((uint32_t)0x00000200) ’DMA_ISR_HTIF3 ((uint32_t)0x00000400) ÷DMA_ISR_TEIF3 ((uint32_t)0x00000800) ◊DMA_ISR_GIF4 ((uint32_t)0x00001000) ÿDMA_ISR_TCIF4 ((uint32_t)0x00002000) ŸDMA_ISR_HTIF4 ((uint32_t)0x00004000) ⁄DMA_ISR_TEIF4 ((uint32_t)0x00008000) €DMA_ISR_GIF5 ((uint32_t)0x00010000) ‹DMA_ISR_TCIF5 ((uint32_t)0x00020000) ›DMA_ISR_HTIF5 ((uint32_t)0x00040000) ﬁDMA_ISR_TEIF5 ((uint32_t)0x00080000) ﬂDMA_ISR_GIF6 ((uint32_t)0x00100000) ‡DMA_ISR_TCIF6 ((uint32_t)0x00200000) ·DMA_ISR_HTIF6 ((uint32_t)0x00400000) ‚DMA_ISR_TEIF6 ((uint32_t)0x00800000) „DMA_ISR_GIF7 ((uint32_t)0x01000000) ‰DMA_ISR_TCIF7 ((uint32_t)0x02000000) ÂDMA_ISR_HTIF7 ((uint32_t)0x04000000) ÊDMA_ISR_TEIF7 ((uint32_t)0x08000000) ÈDMA_IFCR_CGIF1 ((uint32_t)0x00000001) ÍDMA_IFCR_CTCIF1 ((uint32_t)0x00000002) ÎDMA_IFCR_CHTIF1 ((uint32_t)0x00000004) ÏDMA_IFCR_CTEIF1 ((uint32_t)0x00000008) ÌDMA_IFCR_CGIF2 ((uint32_t)0x00000010) ÓDMA_IFCR_CTCIF2 ((uint32_t)0x00000020) ÔDMA_IFCR_CHTIF2 ((uint32_t)0x00000040) DMA_IFCR_CTEIF2 ((uint32_t)0x00000080) ÒDMA_IFCR_CGIF3 ((uint32_t)0x00000100) ÚDMA_IFCR_CTCIF3 ((uint32_t)0x00000200) ÛDMA_IFCR_CHTIF3 ((uint32_t)0x00000400) ÙDMA_IFCR_CTEIF3 ((uint32_t)0x00000800) ıDMA_IFCR_CGIF4 ((uint32_t)0x00001000) ˆDMA_IFCR_CTCIF4 ((uint32_t)0x00002000) ˜DMA_IFCR_CHTIF4 ((uint32_t)0x00004000) ¯DMA_IFCR_CTEIF4 ((uint32_t)0x00008000) ˘DMA_IFCR_CGIF5 ((uint32_t)0x00010000) ˙DMA_IFCR_CTCIF5 ((uint32_t)0x00020000) ˚DMA_IFCR_CHTIF5 ((uint32_t)0x00040000) ¸DMA_IFCR_CTEIF5 ((uint32_t)0x00080000) ˝DMA_IFCR_CGIF6 ((uint32_t)0x00100000) ˛DMA_IFCR_CTCIF6 ((uint32_t)0x00200000) ˇDMA_IFCR_CHTIF6 ((uint32_t)0x00400000) ÄDMA_IFCR_CTEIF6 ((uint32_t)0x00800000) ÅDMA_IFCR_CGIF7 ((uint32_t)0x01000000) ÇDMA_IFCR_CTCIF7 ((uint32_t)0x02000000) ÉDMA_IFCR_CHTIF7 ((uint32_t)0x04000000) ÑDMA_IFCR_CTEIF7 ((uint32_t)0x08000000) áDMA_CCR1_EN ((uint16_t)0x0001) àDMA_CCR1_TCIE ((uint16_t)0x0002) âDMA_CCR1_HTIE ((uint16_t)0x0004) äDMA_CCR1_TEIE ((uint16_t)0x0008) ãDMA_CCR1_DIR ((uint16_t)0x0010) åDMA_CCR1_CIRC ((uint16_t)0x0020) çDMA_CCR1_PINC ((uint16_t)0x0040) éDMA_CCR1_MINC ((uint16_t)0x0080) êDMA_CCR1_PSIZE ((uint16_t)0x0300) ëDMA_CCR1_PSIZE_0 ((uint16_t)0x0100) íDMA_CCR1_PSIZE_1 ((uint16_t)0x0200) îDMA_CCR1_MSIZE ((uint16_t)0x0C00) ïDMA_CCR1_MSIZE_0 ((uint16_t)0x0400) ñDMA_CCR1_MSIZE_1 ((uint16_t)0x0800) òDMA_CCR1_PL ((uint16_t)0x3000) ôDMA_CCR1_PL_0 ((uint16_t)0x1000) öDMA_CCR1_PL_1 ((uint16_t)0x2000) úDMA_CCR1_MEM2MEM ((uint16_t)0x4000) üDMA_CCR2_EN ((uint16_t)0x0001) †DMA_CCR2_TCIE ((uint16_t)0x0002) °DMA_CCR2_HTIE ((uint16_t)0x0004) ¢DMA_CCR2_TEIE ((uint16_t)0x0008) £DMA_CCR2_DIR ((uint16_t)0x0010) §DMA_CCR2_CIRC ((uint16_t)0x0020) •DMA_CCR2_PINC ((uint16_t)0x0040) ¶DMA_CCR2_MINC ((uint16_t)0x0080) ®DMA_CCR2_PSIZE ((uint16_t)0x0300) ©DMA_CCR2_PSIZE_0 ((uint16_t)0x0100) ™DMA_CCR2_PSIZE_1 ((uint16_t)0x0200) ¨DMA_CCR2_MSIZE ((uint16_t)0x0C00) ≠DMA_CCR2_MSIZE_0 ((uint16_t)0x0400) ÆDMA_CCR2_MSIZE_1 ((uint16_t)0x0800) ∞DMA_CCR2_PL ((uint16_t)0x3000) ±DMA_CCR2_PL_0 ((uint16_t)0x1000) ≤DMA_CCR2_PL_1 ((uint16_t)0x2000) ¥DMA_CCR2_MEM2MEM ((uint16_t)0x4000) ∑DMA_CCR3_EN ((uint16_t)0x0001) ∏DMA_CCR3_TCIE ((uint16_t)0x0002) πDMA_CCR3_HTIE ((uint16_t)0x0004) ∫DMA_CCR3_TEIE ((uint16_t)0x0008) ªDMA_CCR3_DIR ((uint16_t)0x0010) ºDMA_CCR3_CIRC ((uint16_t)0x0020) ΩDMA_CCR3_PINC ((uint16_t)0x0040) æDMA_CCR3_MINC ((uint16_t)0x0080) ¿DMA_CCR3_PSIZE ((uint16_t)0x0300) ¡DMA_CCR3_PSIZE_0 ((uint16_t)0x0100) ¬DMA_CCR3_PSIZE_1 ((uint16_t)0x0200) ƒDMA_CCR3_MSIZE ((uint16_t)0x0C00) ≈DMA_CCR3_MSIZE_0 ((uint16_t)0x0400) ∆DMA_CCR3_MSIZE_1 ((uint16_t)0x0800) »DMA_CCR3_PL ((uint16_t)0x3000) …DMA_CCR3_PL_0 ((uint16_t)0x1000)  DMA_CCR3_PL_1 ((uint16_t)0x2000) ÃDMA_CCR3_MEM2MEM ((uint16_t)0x4000) œDMA_CCR4_EN ((uint16_t)0x0001) –DMA_CCR4_TCIE ((uint16_t)0x0002) —DMA_CCR4_HTIE ((uint16_t)0x0004) “DMA_CCR4_TEIE ((uint16_t)0x0008) ”DMA_CCR4_DIR ((uint16_t)0x0010) ‘DMA_CCR4_CIRC ((uint16_t)0x0020) ’DMA_CCR4_PINC ((uint16_t)0x0040) ÷DMA_CCR4_MINC ((uint16_t)0x0080) ÿDMA_CCR4_PSIZE ((uint16_t)0x0300) ŸDMA_CCR4_PSIZE_0 ((uint16_t)0x0100) ⁄DMA_CCR4_PSIZE_1 ((uint16_t)0x0200) ‹DMA_CCR4_MSIZE ((uint16_t)0x0C00) ›DMA_CCR4_MSIZE_0 ((uint16_t)0x0400) ﬁDMA_CCR4_MSIZE_1 ((uint16_t)0x0800) ‡DMA_CCR4_PL ((uint16_t)0x3000) ·DMA_CCR4_PL_0 ((uint16_t)0x1000) ‚DMA_CCR4_PL_1 ((uint16_t)0x2000) ‰DMA_CCR4_MEM2MEM ((uint16_t)0x4000) ÁDMA_CCR5_EN ((uint16_t)0x0001) ËDMA_CCR5_TCIE ((uint16_t)0x0002) ÈDMA_CCR5_HTIE ((uint16_t)0x0004) ÍDMA_CCR5_TEIE ((uint16_t)0x0008) ÎDMA_CCR5_DIR ((uint16_t)0x0010) ÏDMA_CCR5_CIRC ((uint16_t)0x0020) ÌDMA_CCR5_PINC ((uint16_t)0x0040) ÓDMA_CCR5_MINC ((uint16_t)0x0080) DMA_CCR5_PSIZE ((uint16_t)0x0300) ÒDMA_CCR5_PSIZE_0 ((uint16_t)0x0100) ÚDMA_CCR5_PSIZE_1 ((uint16_t)0x0200) ÙDMA_CCR5_MSIZE ((uint16_t)0x0C00) ıDMA_CCR5_MSIZE_0 ((uint16_t)0x0400) ˆDMA_CCR5_MSIZE_1 ((uint16_t)0x0800) ¯DMA_CCR5_PL ((uint16_t)0x3000) ˘DMA_CCR5_PL_0 ((uint16_t)0x1000) ˙DMA_CCR5_PL_1 ((uint16_t)0x2000) ¸DMA_CCR5_MEM2MEM ((uint16_t)0x4000) ˇDMA_CCR6_EN ((uint16_t)0x0001) ÄDMA_CCR6_TCIE ((uint16_t)0x0002) ÅDMA_CCR6_HTIE ((uint16_t)0x0004) ÇDMA_CCR6_TEIE ((uint16_t)0x0008) ÉDMA_CCR6_DIR ((uint16_t)0x0010) ÑDMA_CCR6_CIRC ((uint16_t)0x0020) ÖDMA_CCR6_PINC ((uint16_t)0x0040) ÜDMA_CCR6_MINC ((uint16_t)0x0080) àDMA_CCR6_PSIZE ((uint16_t)0x0300) âDMA_CCR6_PSIZE_0 ((uint16_t)0x0100) äDMA_CCR6_PSIZE_1 ((uint16_t)0x0200) åDMA_CCR6_MSIZE ((uint16_t)0x0C00) çDMA_CCR6_MSIZE_0 ((uint16_t)0x0400) éDMA_CCR6_MSIZE_1 ((uint16_t)0x0800) êDMA_CCR6_PL ((uint16_t)0x3000) ëDMA_CCR6_PL_0 ((uint16_t)0x1000) íDMA_CCR6_PL_1 ((uint16_t)0x2000) îDMA_CCR6_MEM2MEM ((uint16_t)0x4000) óDMA_CCR7_EN ((uint16_t)0x0001) òDMA_CCR7_TCIE ((uint16_t)0x0002) ôDMA_CCR7_HTIE ((uint16_t)0x0004) öDMA_CCR7_TEIE ((uint16_t)0x0008) õDMA_CCR7_DIR ((uint16_t)0x0010) úDMA_CCR7_CIRC ((uint16_t)0x0020) ùDMA_CCR7_PINC ((uint16_t)0x0040) ûDMA_CCR7_MINC ((uint16_t)0x0080) †DMA_CCR7_PSIZE , ((uint16_t)0x0300) °DMA_CCR7_PSIZE_0 ((uint16_t)0x0100) ¢DMA_CCR7_PSIZE_1 ((uint16_t)0x0200) §DMA_CCR7_MSIZE ((uint16_t)0x0C00) •DMA_CCR7_MSIZE_0 ((uint16_t)0x0400) ¶DMA_CCR7_MSIZE_1 ((uint16_t)0x0800) ®DMA_CCR7_PL ((uint16_t)0x3000) ©DMA_CCR7_PL_0 ((uint16_t)0x1000) ™DMA_CCR7_PL_1 ((uint16_t)0x2000) ¨DMA_CCR7_MEM2MEM ((uint16_t)0x4000) ØDMA_CNDTR1_NDT ((uint16_t)0xFFFF) ≤DMA_CNDTR2_NDT ((uint16_t)0xFFFF) µDMA_CNDTR3_NDT ((uint16_t)0xFFFF) ∏DMA_CNDTR4_NDT ((uint16_t)0xFFFF) ªDMA_CNDTR5_NDT ((uint16_t)0xFFFF) æDMA_CNDTR6_NDT ((uint16_t)0xFFFF) ¡DMA_CNDTR7_NDT ((uint16_t)0xFFFF) ƒDMA_CPAR1_PA ((uint32_t)0xFFFFFFFF) «DMA_CPAR2_PA ((uint32_t)0xFFFFFFFF)  DMA_CPAR3_PA ((uint32_t)0xFFFFFFFF) ŒDMA_CPAR4_PA ((uint32_t)0xFFFFFFFF) —DMA_CPAR5_PA ((uint32_t)0xFFFFFFFF) ‘DMA_CPAR6_PA ((uint32_t)0xFFFFFFFF) ÿDMA_CPAR7_PA ((uint32_t)0xFFFFFFFF) €DMA_CMAR1_MA ((uint32_t)0xFFFFFFFF) ﬁDMA_CMAR2_MA ((uint32_t)0xFFFFFFFF) ·DMA_CMAR3_MA ((uint32_t)0xFFFFFFFF) ÂDMA_CMAR4_MA ((uint32_t)0xFFFFFFFF) ËDMA_CMAR5_MA ((uint32_t)0xFFFFFFFF) ÎDMA_CMAR6_MA ((uint32_t)0xFFFFFFFF) ÓDMA_CMAR7_MA ((uint32_t)0xFFFFFFFF) ˜ADC_SR_AWD ((uint8_t)0x01) ¯ADC_SR_EOC ((uint8_t)0x02) ˘ADC_SR_JEOC ((uint8_t)0x04) ˙ADC_SR_JSTRT ((uint8_t)0x08) ˚ADC_SR_STRT ((uint8_t)0x10) ˛ADC_CR1_AWDCH ((uint32_t)0x0000001F) ˇADC_CR1_AWDCH_0 ((uint32_t)0x00000001) ÄADC_CR1_AWDCH_1 ((uint32_t)0x00000002) ÅADC_CR1_AWDCH_2 ((uint32_t)0x00000004) ÇADC_CR1_AWDCH_3 ((uint32_t)0x00000008) ÉADC_CR1_AWDCH_4 ((uint32_t)0x00000010) ÖADC_CR1_EOCIE ((uint32_t)0x00000020) ÜADC_CR1_AWDIE ((uint32_t)0x00000040) áADC_CR1_JEOCIE ((uint32_t)0x00000080) àADC_CR1_SCAN ((uint32_t)0x00000100) âADC_CR1_AWDSGL ((uint32_t)0x00000200) äADC_CR1_JAUTO ((uint32_t)0x00000400) ãADC_CR1_DISCEN ((uint32_t)0x00000800) åADC_CR1_JDISCEN ((uint32_t)0x00001000) éADC_CR1_DISCNUM ((uint32_t)0x0000E000) èADC_CR1_DISCNUM_0 ((uint32_t)0x00002000) êADC_CR1_DISCNUM_1 ((uint32_t)0x00004000) ëADC_CR1_DISCNUM_2 ((uint32_t)0x00008000) ìADC_CR1_DUALMOD ((uint32_t)0x000F0000) îADC_CR1_DUALMOD_0 ((uint32_t)0x00010000) ïADC_CR1_DUALMOD_1 ((uint32_t)0x00020000) ñADC_CR1_DUALMOD_2 ((uint32_t)0x00040000) óADC_CR1_DUALMOD_3 ((uint32_t)0x00080000) ôADC_CR1_JAWDEN ((uint32_t)0x00400000) öADC_CR1_AWDEN ((uint32_t)0x00800000) ûADC_CR2_ADON ((uint32_t)0x00000001) üADC_CR2_CONT ((uint32_t)0x00000002) †ADC_CR2_CAL ((uint32_t)0x00000004) °ADC_CR2_RSTCAL ((uint32_t)0x00000008) ¢ADC_CR2_DMA ((uint32_t)0x00000100) £ADC_CR2_ALIGN ((uint32_t)0x00000800) •ADC_CR2_JEXTSEL ((uint32_t)0x00007000) ¶ADC_CR2_JEXTSEL_0 ((uint32_t)0x00001000) ßADC_CR2_JEXTSEL_1 ((uint32_t)0x00002000) ®ADC_CR2_JEXTSEL_2 ((uint32_t)0x00004000) ™ADC_CR2_JEXTTRIG ((uint32_t)0x00008000) ¨ADC_CR2_EXTSEL ((uint32_t)0x000E0000) ≠ADC_CR2_EXTSEL_0 ((uint32_t)0x00020000) ÆADC_CR2_EXTSEL_1 ((uint32_t)0x00040000) ØADC_CR2_EXTSEL_2 ((uint32_t)0x00080000) ±ADC_CR2_EXTTRIG ((uint32_t)0x00100000) ≤ADC_CR2_JSWSTART ((uint32_t)0x00200000) ≥ADC_CR2_SWSTART ((uint32_t)0x00400000) ¥ADC_CR2_TSVREFE ((uint32_t)0x00800000) ∑ADC_SMPR1_SMP10 ((uint32_t)0x00000007) ∏ADC_SMPR1_SMP10_0 ((uint32_t)0x00000001) πADC_SMPR1_SMP10_1 ((uint32_t)0x00000002) ∫ADC_SMPR1_SMP10_2 ((uint32_t)0x00000004) ºADC_SMPR1_SMP11 ((uint32_t)0x00000038) ΩADC_SMPR1_SMP11_0 ((uint32_t)0x00000008) æADC_SMPR1_SMP11_1 ((uint32_t)0x00000010) øADC_SMPR1_SMP11_2 ((uint32_t)0x00000020) ¡ADC_SMPR1_SMP12 ((uint32_t)0x000001C0) ¬ADC_SMPR1_SMP12_0 ((uint32_t)0x00000040) √ADC_SMPR1_SMP12_1 ((uint32_t)0x00000080) ƒADC_SMPR1_SMP12_2 ((uint32_t)0x00000100) ∆ADC_SMPR1_SMP13 ((uint32_t)0x00000E00) «ADC_SMPR1_SMP13_0 ((uint32_t)0x00000200) »ADC_SMPR1_SMP13_1 ((uint32_t)0x00000400) …ADC_SMPR1_SMP13_2 ((uint32_t)0x00000800) ÀADC_SMPR1_SMP14 ((uint32_t)0x00007000) ÃADC_SMPR1_SMP14_0 ((uint32_t)0x00001000) ÕADC_SMPR1_SMP14_1 ((uint32_t)0x00002000) ŒADC_SMPR1_SMP14_2 ((uint32_t)0x00004000) –ADC_SMPR1_SMP15 ((uint32_t)0x00038000) —ADC_SMPR1_SMP15_0 ((uint32_t)0x00008000) “ADC_SMPR1_SMP15_1 ((uint32_t)0x00010000) ”ADC_SMPR1_SMP15_2 ((uint32_t)0x00020000) ’ADC_SMPR1_SMP16 ((uint32_t)0x001C0000) ÷ADC_SMPR1_SMP16_0 ((uint32_t)0x00040000) ◊ADC_SMPR1_SMP16_1 ((uint32_t)0x00080000) ÿADC_SMPR1_SMP16_2 ((uint32_t)0x00100000) ⁄ADC_SMPR1_SMP17 ((uint32_t)0x00E00000) €ADC_SMPR1_SMP17_0 ((uint32_t)0x00200000) ‹ADC_SMPR1_SMP17_1 ((uint32_t)0x00400000) ›ADC_SMPR1_SMP17_2 ((uint32_t)0x00800000) ‡ADC_SMPR2_SMP0 ((uint32_t)0x00000007) ·ADC_SMPR2_SMP0_0 ((uint32_t)0x00000001) ‚ADC_SMPR2_SMP0_1 ((uint32_t)0x00000002) „ADC_SMPR2_SMP0_2 ((uint32_t)0x00000004) ÂADC_SMPR2_SMP1 ((uint32_t)0x00000038) ÊADC_SMPR2_SMP1_0 ((uint32_t)0x00000008) ÁADC_SMPR2_SMP1_1 ((uint32_t)0x00000010) ËADC_SMPR2_SMP1_2 ((uint32_t)0x00000020) ÍADC_SMPR2_SMP2 ((uint32_t)0x000001C0) ÎADC_SMPR2_SMP2_0 ((uint32_t)0x00000040) ÏADC_SMPR2_SMP2_1 ((uint32_t)0x00000080) ÌADC_SMPR2_SMP2_2 ((uint32_t)0x00000100) ÔADC_SMPR2_SMP3 ((uint32_t)0x00000E00) ADC_SMPR2_SMP3_0 ((uint32_t)0x00000200) ÒADC_SMPR2_SMP3_1 ((uint32_t)0x00000400) ÚADC_SMPR2_SMP3_2 ((uint32_t)0x00000800) ÙADC_SMPR2_SMP4 ((uint32_t)0x00007000) ıADC_SMPR2_SMP4_0 ((uint32_t)0x00001000) ˆADC_SMPR2_SMP4_1 ((uint32_t)0x00002000) ˜ADC_SMPR2_SMP4_2 ((uint32_t)0x00004000) ˘ADC_SMPR2_SMP5 ((uint32_t)0x00038000) ˙ADC_SMPR2_SMP5_0 ((uint32_t)0x00008000) ˚ADC_SMPR2_SMP5_1 ((uint32_t)0x00010000) ¸ADC_SMPR2_SMP5_2 ((uint32_t)0x00020000) ˛ADC_SMPR2_SMP6 ((uint32_t)0x001C0000) ˇADC_SMPR2_SMP6_0 ((uint32_t)0x00040000) ÄADC_SMPR2_SMP6_1 ((uint32_t)0x00080000) ÅADC_SMPR2_SMP6_2 ((uint32_t)0x00100000) ÉADC_SMPR2_SMP7 ((uint32_t)0x00E00000) ÑADC_SMPR2_SMP7_0 ((uint32_t)0x00200000) ÖADC_SMPR2_SMP7_1 ((uint32_t)0x00400000) ÜADC_SMPR2_SMP7_2 ((uint32_t)0x00800000) àADC_SMPR2_SMP8 ((uint32_t)0x07000000) âADC_SMPR2_SMP8_0 ((uint32_t)0x01000000) äADC_SMPR2_SMP8_1 ((uint32_t)0x02000000) ãADC_SMPR2_SMP8_2 ((uint32_t)0x04000000) çADC_SMPR2_SMP9 ((uint32_t)0x38000000) éADC_SMPR2_SMP9_0 ((uint32_t)0x08000000) èADC_SMPR2_SMP9_1 ((uint32_t)0x10000000) êADC_SMPR2_SMP9_2 ((uint32_t)0x20000000) ìADC_JOFR1_JOFFSET1 ((uint16_t)0x0FFF) ñADC_JOFR2_JOFFSET2 ((uint16_t)0x0FFF) ôADC_JOFR3_JOFFSET3 ((uint16_t)0x0FFF) úADC_JOFR4_JOFFSET4 ((uint16_t)0x0FFF) üADC_HTR_HT ((uint16_t)0x0FFF) ¢ADC_LTR_LT ((uint16_t)0x0FFF) •ADC_SQR1_SQ13 ((uint32_t)0x0000001F) ¶ADC_SQR1_SQ13_0 ((uint32_t)0x00000001) ßADC_SQR1_SQ13_1 ((uint32_t)0x00000002) ®ADC_SQR1_SQ13_2 ((uint32_t)0x00000004) ©ADC_SQR1_SQ13_3 ((uint32_t)0x00000008) ™ADC_SQR1_SQ13_4 ((uint32_t)0x00000010) ¨ADC_SQR1_SQ14 ((uint32_t)0x000003E0) ≠ADC_SQR1_SQ14_0 ((uint32_t)0x00000020) ÆADC_SQR1_SQ14_1 ((uint32_t)0x00000040) ØADC_SQR1_SQ14_2 ((uint32_t)0x00000080) ∞ADC_SQR1_SQ14_3 ((uint32_t)0x00000100) ±ADC_SQR1_SQ14_4 ((uint32_t)0x00000200) ≥ADC_SQR1_SQ15 ((uint32_t)0x00007C00) ¥ADC_SQR1_SQ15_0 ((uint32_t)0x00000400) µADC_SQR1_SQ15_1 ((uint32_t)0x00000800) ∂ADC_SQR1_SQ15_2 ((uint32_t)0x00001000) ∑ADC_SQR1_SQ15_3 ((uint32_t)0x00002000) ∏ADC_SQR1_SQ15_4 ((uint32_t)0x00004000) ∫ADC_SQR1_SQ16 ((uint32_t)0x000F8000) ªADC_SQR1_SQ16_0 ((uint32_t)0x00008000) ºADC_SQR1_SQ16_1 ((uint32_t)0x00010000) ΩADC_SQR1_SQ16_2 ((uint32_t)0x00020000) æADC_SQR1_SQ16_3 ((uint32_t)0x00040000) øADC_SQR1_SQ16_4 ((uint32_t)0x00080000) ¡ADC_SQR1_L ((uint32_t)0x00F00000) ¬ADC_SQR1_L_0 ((uint32_t)0x00100000) √ADC_SQR1_L_1 ((uint32_t)0x00200000) ƒADC_SQR1_L_2 ((uint32_t)0x00400000) ≈ADC_SQR1_L_3 ((uint32_t)0x00800000) »ADC_SQR2_SQ7 ((uint32_t)0x0000001F) …ADC_SQR2_SQ7_0 ((uint32_t)0x00000001)  ADC_SQR2_SQ7_1 ((uint32_t)0x00000002) ÀADC_SQR2_SQ7_2 ((uint32_t)0x00000004) ÃADC_SQR2_SQ7_3 ((uint32_t)0x00000008) ÕADC_SQR2_SQ7_4 ((uint32_t)0x00000010) œADC_SQR2_SQ8 ((uint32_t)0x000003E0) –ADC_SQR2_SQ8_0 ((uint32_t)0x00000020) —ADC_SQR2_SQ8_1 ((uint32_t)0x00000040) “ADC_SQR2_SQ8_2 ((uint32_t)0x00000080) ”ADC_SQR2_SQ8_3 ((uint32_t)0x00000100) ‘ADC_SQR2_SQ8_4 ((uint32_t)0x00000200) ÷ADC_SQR2_SQ9 ((uint32_t)0x00007C00) ◊ADC_SQR2_SQ9_0 ((uint32_t)0x00000400) ÿADC_SQR2_SQ9_1 ((uint32_t)0x00000800) ŸADC_SQR2_SQ9_2 ((uint32_t)0x00001000) ⁄ADC_SQR2_SQ9_3 ((uint32_t)0x00002000) €ADC_SQR2_SQ9_4 ((uint32_t)0x00004000) ›ADC_SQR2_SQ10 ((uint32_t)0x000F8000) ﬁADC_SQR2_SQ10_0 ((uint32_t)0x00008000) ﬂADC_SQR2_SQ10_1 ((uint32_t)0x00010000) ‡ADC_SQR2_SQ10_2 ((uint32_t)0x00020000) ·ADC_SQR2_SQ10_3 ((uint32_t)0x00040000) ‚ADC_SQR2_SQ10_4 ((uint32_t)0x00080000) ‰ADC_SQR2_SQ11 ((uint32_t)0x01F00000) ÂADC_SQR2_SQ11_0 ((uint32_t)0x00100000) ÊADC_SQR2_SQ11_1 ((uint32_t)0x00200000) ÁADC_SQR2_SQ11_2 ((uint32_t)0x00400000) ËADC_SQR2_SQ11_3 ((uint32_t)0x00800000) ÈADC_SQR2_SQ11_4 ((uint32_t)0x01000000) ÎADC_SQR2_SQ12 ((uint32_t)0x3E000000) ÏADC_SQR2_SQ12_0 ((uint32_t)0x02000000) ÌADC_SQR2_SQ12_1 ((uint32_t)0x04000000) ÓADC_SQR2_SQ12_2 ((uint32_t)0x08000000) ÔADC_SQR2_SQ12_3 ((uint32_t)0x10000000) ADC_SQR2_SQ12_4 ((uint32_t)0x20000000) ÛADC_SQR3_SQ1 ((uint32_t)0x0000001F) ÙADC_SQR3_SQ1_0 ((uint32_t)0x00000001) ıADC_SQR3_SQ1_1 ((uint32_t)0x00000002) ˆADC_SQR3_SQ1_2 ((uint32_t)0x00000004) ˜ADC_SQR3_SQ1_3 ((uint32_t)0x00000008) ¯ADC_SQR3_SQ1_4 ((uint32_t)0x00000010) ˙ADC_SQR3_SQ2 ((uint32_t)0x000003E0) ˚ADC_SQR3_SQ2_0 ((uint32_t)0x00000020) ¸ADC_SQR3_SQ2_1 ((uint32_t)0x00000040) ˝ADC_SQR3_SQ2_2 ((uint32_t)0x00000080) ˛ADC_SQR3_SQ2_3 ((uint32_t)0x00000100) ˇADC_SQR3_SQ2_4 ((uint32_t)0x00000200) ÅADC_SQR3_SQ3 ((uint32_t)0x00007C00) ÇADC_SQR3_SQ3_0 ((uint32_t)0x00000400) ÉADC_SQR3_SQ3_1 ((uint32_t)0x00000800) ÑADC_SQR3_SQ3_2 ((uint32_t)0x00001000) ÖADC_SQR3_SQ3_3 ((uint32_t)0x00002000) ÜADC_SQR3_SQ3_4 ((uint32_t)0x00004000) àADC_SQR3_SQ4 ((uint32_t)0x000F8000) âADC_SQR3_SQ4_0 ((uint32_t)0x00008000) äADC_SQR3_SQ4_1 ((uint32_t)0x00010000) ãADC_SQR3_SQ4_2 ((uint32_t)0x00020000) åADC_SQR3_SQ4_3 ((uint32_t)0x00040000) çADC_SQR3_SQ4_4 ((uint32_t)0x00080000) èADC_SQR3_SQ5 ((uint32_t)0x01F00000) êADC_SQR3_SQ5_0 ((uint32_t)0x00100000) ëADC_SQR3_SQ5_1 ((uint32_t)0x00200000) íADC_SQR3_SQ5_2 ((uint32_t)0x00400000) ìADC_SQR3_SQ5_3 ((uint32_t)0x00800000) îADC_SQR3_SQ5_4 ((uint32_t)0x01000000) ñADC_SQR3_SQ6 ((uint32_t)0x3E000000) óADC_SQR3_SQ6_0 ((uint32_t)0x02000000) òADC_SQR3_SQ6_1 ((uint32_t)0x04000000) ôADC_SQR3_SQ6_2 ((uint32_t)0x08000000) öADC_SQR3_SQ6_3 ((uint32_t)0x10000000) õADC_SQR3_SQ6_4 ((uint32_t)0x20000000) ûADC_JSQR_JSQ1 ((uint32_t)0x0000001F) üADC_JSQR_JSQ1_0 ((uint32_t)0x00000001) †ADC_JSQR_JSQ1_1 ((uint32_t)0x00000002) °ADC_JSQR_JSQ1_2 ((uint32_t)0x00000004) ¢ADC_JSQR_JSQ1_3 ((uint32_t)0x00000008) £ADC_JSQR_JSQ1_4 ((uint32_t)0x00000010) •ADC_JSQR_JSQ2 ((uint32_t)0x000003E0) ¶ADC_JSQR_JSQ2_0 ((uint32_t)0x00000020) ßADC_JSQR_JSQ2_1 ((uint32_t)0x00000040) ®ADC_JSQR_JSQ2_2 ((uint32_t)0x00000080) ©ADC_JSQR_JSQ2_3 ((uint32_t)0x00000100) ™ADC_JSQR_JSQ2_4 ((uint32_t)0x00000200) ¨ADC_JSQR_JSQ3 ((uint32_t)0x00007C00) ≠ADC_JSQR_JSQ3_0 ((uint32_t)0x00000400) ÆADC_JSQR_JSQ3_1 ((uint32_t)0x00000800) ØADC_JSQR_JSQ3_2 ((uint32_t)0x00001000) ∞ADC_JSQR_JSQ3_3 ((uint32_t)0x00002000) ±ADC_JSQR_JSQ3_4 ((uint32_t)0x00004000) ≥ADC_JSQR_JSQ4 ((uint32_t)0x000F8000) ¥ADC_JSQR_JSQ4_0 ((uint32_t)0x00008000) µADC_JSQR_JSQ4_1 ((uint32_t)0x00010000) ∂ADC_JSQR_JSQ4_2 ((uint32_t)0x00020000) ∑ADC_JSQR_JSQ4_3 ((uint32_t)0x00040000) ∏ADC_JSQR_JSQ4_4 ((uint32_t)0x00080000) ∫ADC_JSQR_JL ((uint32_t)0x00300000) ªADC_JSQR_JL_0 ((uint32_t)0x00100000) ºADC_JSQR_JL_1 ((uint32_t)0x00200000) øADC_JDR1_JDATA ((uint16_t)0xFFFF) ¬ADC_JDR2_JDATA ((uint16_t)0xFFFF) ≈ADC_JDR3_JDATA ((uint16_t)0xFFFF) »ADC_JDR4_JDATA ((uint16_t)0xFFFF) ÀADC_DR_DATA ((uint32_t)0x0000FFFF) ÃADC_DR_ADC2DATA ((uint32_t)0xFFFF0000) ’DAC_CR_EN1 ((uint32_t)0x00000001) ÷DAC_CR_BOFF1 ((uint32_t)0x00000002) ◊DAC_CR_TEN1 ((uint32_t)0x00000004) ŸDAC_CR_TSEL1 ((uint32_t)0x00000038) ⁄DAC_CR_TSEL1_0 ((uint32_t)0x00000008) €DAC_CR_TSEL1_1 ((uint32_t)0x00000010) ‹DAC_CR_TSEL1_2 ((uint32_t)0x00000020) ﬁDAC_CR_WAVE1 ((uint32_t)0x000000C0) ﬂDAC_CR_WAVE1_0 ((uint32_t)0x00000040) ‡DAC_CR_WAVE1_1 ((uint32_t)0x00000080) ‚DAC_CR_MAMP1 ((uint32_t)0x00000F00) „DAC_CR_MAMP1_0 ((uint32_t)0x00000100) ‰DAC_CR_MAMP1_1 ((uint32_t)0x00000200) ÂDAC_CR_MAMP1_2 ((uint32_t)0x00000400) ÊDAC_CR_MAMP1_3 ((uint32_t)0x00000800) ËDAC_CR_DMAEN1 ((uint32_t)0x00001000) ÈDAC_CR_EN2 ((uint32_t)0x00010000) ÍDAC_CR_BOFF2 ((uint32_t)0x00020000) ÎDAC_CR_TEN2 ((uint32_t)0x00040000) ÌDAC_CR_TSEL2 ((uint32_t)0x00380000) ÓDAC_CR_TSEL2_0 ((uint32_t)0x00080000) ÔDAC_CR_TSEL2_1 ((uint32_t)0x00100000) DAC_CR_TSEL2_2 ((uint32_t)0x00200000) ÚDAC_CR_WAVE2 ((uint32_t)0x00C00000) ÛDAC_CR_WAVE2_0 ((uint32_t)0x00400000) ÙDAC_CR_WAVE2_1 ((uint32_t)0x00800000) ˆDAC_CR_MAMP2 ((uint32_t)0x0F000000) ˜DAC_CR_MAMP2_0 ((uint32_t)0x01000000) ¯DAC_CR_MAMP2_1 ((uint32_t)0x02000000) ˘DAC_CR_MAMP2_2 ((uint32_t)0x04000000) ˙DAC_CR_MAMP2_3 ((uint32_t)0x08000000) ¸DAC_CR_DMAEN2 ((uint32_t)0x10000000) ˇDAC_SWTRIGR_SWTRIG1 ((uint8_t)0x01) Ä DAC_SWTRIGR_SWTRIG2 ((uint8_t)0x02) É DAC_DHR12R1_DACC1DHR ((uint16_t)0x0FFF) Ü DAC_DHR12L1_DACC1DHR ((uint16_t)0xFFF0) â DAC_DHR8R1_DACC1DHR ((uint8_t)0xFF) å DAC_DHR12R2_DACC2DHR ((uint16_t)0x0FFF) è DAC_DHR12L2_DACC2DHR ((uint16_t)0xFFF0) í DAC_DHR8R2_DACC2DHR ((uint8_t)0xFF) ï DAC_DHR12RD_DACC1DHR ((uint32_t)0x00000FFF) ñ DAC_DHR12RD_DACC2DHR ((uint32_t)0x0FFF0000) ô DAC_DHR12LD_DACC1DHR ((uint32_t)0x0000FFF0) ö DAC_DHR12LD_DACC2DHR ((uint32_t)0xFFF00000) ù DAC_DHR8RD_DACC1DHR ((uint16_t)0x00FF) û DAC_DHR8RD_DACC2DHR ((uint16_t)0xFF00) ° DAC_DOR1_DACC1DOR ((uint16_t)0x0FFF) § DAC_DOR2_DACC2DOR ((uint16_t)0x0FFF) ß DAC_SR_DMAUDR1 ((uint32_t)0x00002000) ® DAC_SR_DMAUDR2 ((uint32_t)0x20000000) ∞ CEC_CFGR_PE ((uint16_t)0x0001) ± CEC_CFGR_IE ((uint16_t)0x0002) ≤ CEC_CFGR_BTEM ((uint16_t)0x0004) ≥ CEC_CFGR_BPEM ((uint16_t)0x0008) ∂ CEC_OAR_OA ((uint16_t)0x000F) ∑ CEC_OAR_OA_0 ((uint16_t)0x0001) ∏ CEC_OAR_OA_1 ((uint16_t)0x0002) π CEC_OAR_OA_2 ((uint16_t)0x0004) ∫ CEC_OAR_OA_3 ((uint16_t)0x0008) Ω CEC_PRES_PRES ((uint16_t)0x3FFF) ¿ CEC_ESR_BTE ((uint16_t)0x0001) ¡ CEC_ESR_BPE ((uint16_t)0x0002) ¬ CEC_ESR_RBTFE ((uint16_t)0x0004) √ CEC_ESR_SBE ((uint16_t)0x0008) ƒ CEC_ESR_ACKE ((uint16_t)0x0010) ≈ CEC_ESR_LINE ((uint16_t)0x0020) ∆ CEC_ESR_TBTFE ((uint16_t)0x0040) … CEC_CSR_TSOM ((uint16_t)0x0001)   CEC_CSR_TEOM ((uint16_t)0x0002) À CEC_CSR_TERR ((uint16_t)0x0004) Ã CEC_CSR_TBTRF ((uint16_t)0x0008) Õ CEC_CSR_RSOM ((uint16_t)0x0010) Œ CEC_CSR_REOM ((uint16_t)0x0020) œ CEC_CSR_RERR ((uint16_t)0x0040) – CEC_CSR_RBTF ((uint16_t)0x0080) ” CEC_TXD_TXD ((uint16_t)0x00FF) ÷ CEC_RXD_RXD ((uint16_t)0x00FF) ﬂ TIM_CR1_CEN ((uint16_t)0x0001) ‡ TIM_CR1_UDIS ((uint16_t)0x0002) · TIM_CR1_URS ((uint16_t)0x0004) ‚ TIM_CR1_OPM ((uint16_t)0x0008) „ TIM_CR1_DIR ((uint16_t)0x0010) Â TIM_CR1_CMS ((uint16_t)0x0060) Ê TIM_CR1_CMS_0 ((uint16_t)0x0020) Á TIM_CR1_CMS_1 ((uint16_t)0x0040) È TIM_CR1_ARPE ((uint16_t)0x0080) Î TIM_CR1_CKD ((uint16_t)0x0300) Ï TIM_CR1_CKD_0 ((uint16_t)0x0100) Ì TIM_CR1_CKD_1 ((uint16_t)0x0200)  TIM_CR2_CCPC ((uint16_t)0x0001) Ò TIM_CR2_CCUS ((uint16_t)0x0004) Ú TIM_CR2_CCDS ((uint16_t)0x0008) Ù TIM_CR2_MMS ((uint16_t)0x0070) ı TIM_CR2_MMS_0 ((uint16_t)0x0010) ˆ TIM_CR2_MMS_1 ((uint16_t)0x0020) ˜ TIM_CR2_MMS_2 ((uint16_t)0x0040) ˘ TIM_CR2_TI1S ((uint16_t)0x0080) ˙ TIM_CR2_OIS1 ((uint16_t)0x0100) ˚ TIM_CR2_OIS1N ((uint16_t)0x0200) ¸ TIM_CR2_OIS2 ((uint16_t)0x0400) ˝ TIM_CR2_OIS2N ((uint16_t)0x0800) ˛ TIM_CR2_OIS3 ((uint16_t)0x1000) ˇ TIM_CR2_OIS3N ((uint16_t)0x2000) Ä!TIM_CR2_OIS4 ((uint16_t)0x4000) É!TIM_SMCR_SMS ((uint16_t)0x0007) Ñ!TIM_SMCR_SMS_0 ((uint16_t)0x0001) Ö!TIM_SMCR_SMS_1 ((uint16_t)0x0002) Ü!TIM_SMCR_SMS_2 ((uint16_t)0x0004) à!TIM_SMCR_TS ((uint16_t)0x0070) â!TIM_SMCR_TS_0 ((uint16_t)0x0010) ä!TIM_SMCR_TS_1 ((uint16_t)0x0020) ã!TIM_SMCR_TS_2 ((uint16_t)0x0040) ç!TIM_SMCR_MSM ((uint16_t)0x0080) è!TIM_SMCR_ETF ((uint16_t)0x0F00) ê!TIM_SMCR_ETF_0 ((uint16_t)0x0100) ë!TIM_SMCR_ETF_1 ((uint16_t)0x0200) í!TIM_SMCR_ETF_2 ((uint16_t)0x0400) ì!TIM_SMCR_ETF_3 ((uint16_t)0x0800) ï!TIM_SMCR_ETPS ((uint16_t)0x3000) ñ!TIM_SMCR_ETPS_0 ((uint16_t)0x1000) ó!TIM_SMCR_ETPS_1 ((uint16_t)0x2000) ô!TIM_SMCR_ECE ((uint16_t)0x4000) ö!TIM_SMCR_ETP ((uint16_t)0x8000) ù!TIM_DIER_UIE ((uint16_t)0x0001) û!TIM_DIER_CC1IE ((uint16_t)0x0002) ü!TIM_DIER_CC2IE ((uint16_t)0x0004) †!TIM_DIER_CC3IE ((uint16_t)0x0008) °!TIM_DIER_CC4IE ((uint16_t)0x0010) ¢!TIM_DIER_COMIE ((uint16_t)0x0020) £!TIM_DIER_TIE ((uint16_t)0x0040) §!TIM_DIER_BIE ((uint16_t)0x0080) •!TIM_DIER_UDE ((uint16_t)0x0100) ¶!TIM_DIER_CC1DE ((uint16_t)0x0200) ß!TIM_DIER_CC2DE ((uint16_t)0x0400) ®!TIM_DIER_CC3DE ((uint16_t)0x0800) ©!TIM_DIER_CC4DE ((uint16_t)0x1000) ™!TIM_DIER_COMDE ((uint16_t)0x2000) ´!TIM_DIER_TDE ((uint16_t)0x4000) Æ!TIM_SR_UIF ((uint16_t)0x0001) Ø!TIM_SR_CC1IF ((uint16_t)0x0002) ∞!TIM_SR_CC2IF ((uint16_t)0x0004) ±!TIM_SR_CC3IF ((uint16_t)0x0008) ≤!TIM_SR_CC4IF ((uint16_t)0x0010) ≥!TIM_SR_COMIF ((uint16_t)0x0020) ¥!TIM_SR_TIF ((uint16_t)0x0040) µ!TIM_SR_BIF ((uint16_t)0x0080) ∂!TIM_SR_CC1OF ((uint16_t)0x0200) ∑!TIM_SR_CC2OF ((uint16_t)0x0400) ∏!TIM_SR_CC3OF ((uint16_t)0x0800) π!TIM_SR_CC4OF ((uint16_t)0x1000) º!TIM_EGR_UG ((uint8_t)0x01) Ω!TIM_EGR_CC1G ((uint8_t)0x02) æ!TIM_EGR_CC2G ((uint8_t)0x04) ø!TIM_EGR_CC3G ((uint8_t)0x08) ¿!TIM_EGR_CC4G ((uint8_t)0x10) ¡!TIM_EGR_COMG ((uint8_t)0x20) ¬!TIM_EGR_TG ((uint8_t)0x40) √!TIM_EGR_BG ((uint8_t)0x80) ∆!TIM_CCMR1_CC1S ((uint16_t)0x0003) «!TIM_CCMR1_CC1S_0 ((uint16_t)0x0001) »!TIM_CCMR1_CC1S_1 ((uint16_t)0x0002)  !TIM_CCMR1_OC1FE ((uint16_t)0x0004) À!TIM_CCMR1_OC1PE ((uint16_t)0x0008) Õ!TIM_CCMR1_OC1M ((uint16_t)0x0070) Œ!TIM_CCMR1_OC1M_0 ((uint16_t)0x0010) œ!TIM_CCMR1_OC1M_1 ((uint16_t)0x0020) –!TIM_CCMR1_OC1M_2 ((uint16_t)0x0040) “!TIM_CCMR1_OC1CE ((uint16_t)0x0080) ‘!TIM_CCMR1_CC2S ((uint16_t)0x0300) ’!TIM_CCMR1_CC2S_0 ((uint16_t)0x0100) ÷!TIM_CCMR1_CC2S_1 ((uint16_t)0x0200) ÿ!TIM_CCMR1_OC2FE ((uint16_t)0x0400) Ÿ!TIM_CCMR1_OC2PE ((uint16_t)0x0800) €!TIM_CCMR1_OC2M ((uint16_t)0x7000) ‹!TIM_CCMR1_OC2M_0 ((uint16_t)0x1000) ›!TIM_CCMR1_OC2M_1 ((uint16_t)0x2000) ﬁ!TIM_CCMR1_OC2M_2 ((uint16_t)0x4000) ‡!TIM_CCMR1_OC2CE ((uint16_t)0x8000) ‰!TIM_CCMR1_IC1PSC ((uint16_t)0x000C) Â!TIM_CCMR1_IC1PSC_0 ((uint16_t)0x0004) Ê!TIM_CCMR1_IC1PSC_1 ((uint16_t)0x0008) Ë!TIM_CCMR1_IC1F ((uint16_t)0x00F0) È!TIM_CCMR1_IC1F_0 ((uint16_t)0x0010) Í!TIM_CCMR1_IC1F_1 ((uint16_t)0x0020) Î!TIM_CCMR1_IC1F_2 ((uint16_t)0x0040) Ï!TIM_CCMR1_IC1F_3 ((uint16_t)0x0080) Ó!TIM_CCMR1_IC2PSC ((uint16_t)0x0C00) Ô!TIM_CCMR1_IC2PSC_0 ((uint16_t)0x0400) !TIM_CCMR1_IC2PSC_1 ((uint16_t)0x0800) Ú!TIM_CCMR1_IC2F ((uint16_t)0xF000) Û!TIM_CCMR1_IC2F_0 ((uint16_t)0x1000) Ù!TIM_CCMR1_IC2F_1 ((uint16_t)0x2000) ı!TIM_CCMR1_IC2F_2 ((uint16_t)0x4000) ˆ!TIM_CCMR1_IC2F_3 ((uint16_t)0x8000) ˘!TIM_CCMR2_CC3S ((uint16_t)0x0003) ˙!TIM_CCMR2_CC3S_0 ((uint16_t)0x0001) ˚!TIM_CCMR2_CC3S_1 ((uint16_t)0x0002) ˝!TIM_CCMR2_OC3FE ((uint16_t)0x0004) ˛!TIM_CCMR2_OC3PE ((uint16_t)0x0008) Ä"TIM_CCMR2_OC3M ((uint16_t)0x0070) Å"TIM_CCMR2_OC3M_0 ((uint16_t)0x0010) Ç"TIM_CCMR2_OC3M_1 ((uint16_t)0x0020) É"TIM_CCMR2_OC3M_2 ((uint16_t)0x0040) Ö"TIM_CCMR2_OC3CE ((uint16_t)0x0080) á"TIM_CCMR2_CC4S ((uint16_t)0x0300) à"TIM_CCMR2_CC4S_0 ((uint16_t)0x0100) â"TIM_CCMR2_CC4S_1 ((uint16_t)0x0200) ã"TIM_CCMR2_OC4FE ((uint16_t)0x0400) å"TIM_CCMR2_OC4PE ((uint16_t)0x0800) é"TIM_CCMR2_OC4M ((uint16_t)0x7000) è"TIM_CCMR2_OC4M_0 ((uint16_t)0x1000) ê"TIM_CCMR2_OC4M_1 ((uint16_t)0x2000) ë"TIM_CCMR2_OC4M_2 ((uint16_t)0x4000) ì"TIM_CCMR2_OC4CE ((uint16_t)0x8000) ó"TIM_CCMR2_IC3PSC ((uint16_t)0x000C) ò"TIM_CCMR2_IC3PSC_0 ((uint16_t)0x0004) ô"TIM_CCMR2_IC3PSC_1 ((uint16_t)0x0008) õ"TIM_CCMR2_IC3F ((uint16_t)0x00F0) ú"TIM_CCMR2_IC3F_0 ((uint16_t)0x0010) ù"TIM_CCMR2_IC3F_1 ((uint16_t)0x0020) û"TIM_CCMR2_IC3F_2 ((uint16_t)0x0040) ü"TIM_CCMR2_IC3F_3 ((uint16_t)0x0080) °"TIM_CCMR2_IC4PSC ((uint16_t)0x0C00) ¢"TIM_CCMR2_IC4PSC_0 ((uint16_t)0x0400) £"TIM_CCMR2_IC4PSC_1 ((uint16_t)0x0800) •"TIM_CCMR2_IC4F ((uint16_t)0xF000) ¶"TIM_CCMR2_IC4F_0 ((uint16_t)0x1000) ß"TIM_CCMR2_IC4F_1 ((uint16_t)0x2000) ®"TIM_CCMR2_IC4F_2 ((uint16_t)0x4000) ©"TIM_CCMR2_IC4F_3 ((uint16_t)0x8000) ¨"TIM_CCER_CC1E ((uint16_t)0x0001) ≠"TIM_CCER_CC1P ((uint16_t)0x0002) Æ"TIM_CCER_CC1NE ((uint16_t)0x0004) Ø"TIM_CCER_CC1NP ((uint16_t)0x0008) ∞"TIM_CCER_CC2E ((uint16_t)0x0010) ±"TIM_CCER_CC2P ((uint16_t)0x0020) ≤"TIM_CCER_CC2NE ((uint16_t)0x0040) ≥"TIM_CCER_CC2NP ((uint16_t)0x0080) ¥"TIM_CCER_CC3E ((uint16_t)0x0100) µ"TIM_CCER_CC3P ((uint16_t)0x0200) ∂"TIM_CCER_CC3NE ((uint16_t)0x0400) ∑"TIM_CCER_CC3NP ((uint16_t)0x0800) ∏"TIM_CCER_CC4E ((uint16_t)0x1000) π"TIM_CCER_CC4P ((uint16_t)0x2000) ∫"TIM_CCER_CC4NP ((uint16_t)0x8000) Ω"TIM_CNT_CNT ((uint16_t)0xFFFF) ¿"TIM_PSC_PSC ((uint16_t)0xFFFF) √"TIM_ARR_ARR ((uint16_t)0xFFFF) ∆"TIM_RCR_REP ((uint8_t)0xFF) …"TIM_CCR1_CCR1 ((uint16_t)0xFFFF) Ã"TIM_CCR2_CCR2 ((uint16_t)0xFFFF) œ"TIM_CCR3_CCR3 ((uint16_t)0xFFFF) “"TIM_CCR4_CCR4 ((uint16_t)0xFFFF) ’"TIM_BDTR_DTG ((uint16_t)0x00FF) ÷"TIM_BDTR_DTG_0 ((uint16_t)0x0001) ◊"TIM_BDTR_DTG_1 ((uint16_t)0x0002) ÿ"TIM_BDTR_DTG_2 ((uint16_t)0x0004) Ÿ"TIM_BDTR_DTG_3 ((uint16_t)0x0008) ⁄"TIM_BDTR_DTG_4 ((uint16_t)0x0010) €"TIM_BDTR_DTG_5 ((uint16_t)0x0020) ‹"TIM_BDTR_DTG_6 ((uint16_t)0x0040) ›"TIM_BDTR_DTG_7 ((uint16_t)0x0080) ﬂ"TIM_BDTR_LOCK ((uint16_t)0x0300) ‡"TIM_BDTR_LOCK_0 ((uint16_t)0x0100) ·"TIM_BDTR_LOCK_1 ((uint16_t)0x0200) „"TIM_BDTR_OSSI ((uint16_t)0x0400) ‰"TIM_BDTR_OSSR ((uint16_t)0x0800) Â"TIM_BDTR_BKE ((uint16_t)0x1000) Ê"TIM_BDTR_BKP ((uint16_t)0x2000) Á"TIM_BDTR_AOE ((uint16_t)0x4000) Ë"TIM_BDTR_MOE ((uint16_t)0x8000) Î"TIM_DCR_DBA ((uint16_t)0x001F) Ï"TIM_DCR_DBA_0 ((uint16_t)0x0001) Ì"TIM_DCR_DBA_1 ((uint16_t)0x0002) Ó"TIM_DCR_DBA_2 ((uint16_t)0x0004) Ô"TIM_DCR_DBA_3 ((uint16_t)0x0008) "TIM_DCR_DBA_4 ((uint16_t)0x0010) Ú"TIM_DCR_DBL ((uint16_t)0x1F00) Û"TIM_DCR_DBL_0 ((uint16_t)0x0100) Ù"TIM_DCR_DBL_1 ((uint16_t)0x0200) ı"TIM_DCR_DBL_2 ((uint16_t)0x0400) ˆ"TIM_DCR_DBL_3 ((uint16_t)0x0800) ˜"TIM_DCR_DBL_4 ((uint16_t)0x1000) ˙"TIM_DMAR_DMAB ((uint16_t)0xFFFF) É#RTC_CRH_SECIE ((uint8_t)0x01) Ñ#RTC_CRH_ALRIE ((uint8_t)0x02) Ö#RTC_CRH_OWIE ((uint8_t)0x04) à#RTC_CRL_SECF ((uint8_t)0x01) â#RTC_CRL_ALRF ((uint8_t)0x02) ä#RTC_CRL_OWF ((uint8_t)0x04) ã#RTC_CRL_RSF ((uint8_t)0x08) å#RTC_CRL_CNF ((uint8_t)0x10) ç#RTC_CRL_RTOFF ((uint8_t)0x20) ê#RTC_PRLH_PRL ((uint16_t)0x000F) ì#RTC_PRLL_PRL ((uint16_t)0xFFFF) ñ#RTC_DIVH_RTC_DIV ((uint16_t)0x000F) ô#RTC_DIVL_RTC_DIV ((uint16_t)0xFFFF) ú#RTC_CNTH_RTC_CNT ((uint16_t)0xFFFF) ü#RTC_CNTL_RTC_CNT ((uint16_t)0xFFFF) ¢#RTC_ALRH_RTC_ALR ((uint16_t)0xFFFF) •#RTC_ALRL_RTC_ALR ((uint16_t)0xFFFF) Æ#IWDG_KR_KEY ((uint16_t)0xFFFF) ±#IWDG_PR_PR ((uint8_t)0x07) ≤#IWDG_PR_PR_0 ((uint8_t)0x01) ≥#IWDG_PR_PR_1 ((uint8_t)0x02) ¥#IWDG_PR_PR_2 ((uint8_t)0x04) ∑#IWDG_RLR_RL ((uint16_t)0x0FFF) ∫#IWDG_SR_PVU ((uint8_t)0x01) ª#IWDG_SR_RVU ((uint8_t)0x02) ƒ#WWDG_CR_T ((uint8_t)0x7F) ≈#WWDG_CR_T0 ((uint8_t)0x01) ∆#WWDG_CR_T1 ((uint8_t)0x02) «#WWDG_CR_T2 ((uint8_t)0x04) »#WWDG_CR_T3 ((uint8_t)0x08) …#WWDG_CR_T4 ((uint8_t)0x10)  #WWDG_CR_T5 ((uint8_t)0x20) À#WWDG_CR_T6 ((uint8_t)0x40) Õ#WWDG_CR_WDGA ((uint8_t)0x80) –#WWDG_CFR_W ((uint16_t)0x007F) —#WWDG_CFR_W0 ((uint16_t)0x0001) “#WWDG_CFR_W1 ((uint16_t)0x0002) ”#WWDG_CFR_W2 ((uint16_t)0x0004) ‘#WWDG_CFR_W3 ((uint16_t)0x0008) ’#WWDG_CFR_W4 ((uint16_t)0x0010) ÷#WWDG_CFR_W5 ((uint16_t)0x0020) ◊#WWDG_CFR_W6 ((uint16_t)0x0040) Ÿ#WWDG_CFR_WDGTB ((uint16_t)0x0180) ⁄#WWDG_CFR_WDGTB0 ((uint16_t)0x0080) €#WWDG_CFR_WDGTB1 ((uint16_t)0x0100) ›#WWDG_CFR_EWI ((uint16_t)0x0200) ‡#WWDG_SR_EWIF ((uint8_t)0x01) È#FSMC_BCR1_MBKEN ((uint32_t)0x00000001) Í#FSMC_BCR1_MUXEN ((uint32_t)0x00000002) Ï#FSMC_BCR1_MTYP ((uint32_t)0x0000000C) Ì#FSMC_BCR1_MTYP_0 ((uint32_t)0x00000004) Ó#FSMC_BCR1_MTYP_1 ((uint32_t)0x00000008) #FSMC_BCR1_MWID ((uint32_t)0x00000030) Ò#FSMC_BCR1_MWID_0 ((uint32_t)0x00000010) Ú#FSMC_BCR1_MWID_1 ((uint32_t)0x00000020) Ù#FSMC_BCR1_FACCEN ((uint32_t)0x00000040) ı#FSMC_BCR1_BURSTEN ((uint32_t)0x00000100) ˆ#FSMC_BCR1_WAITPOL ((uint32_t)0x00000200) ˜#FSMC_BCR1_WRAPMOD ((uint32_t)0x00000400) ¯#FSMC_BCR1_WAITCFG ((uint32_t)0x00000800) ˘#FSMC_BCR1_WREN ((uint32_t)0x00001000) ˙#FSMC_BCR1_WAITEN ((uint32_t)0x00002000) ˚#FSMC_BCR1_EXTMOD ((uint32_t)0x00004000) ¸#FSMC_BCR1_ASYNCWAIT ((uint32_t)0x00008000) ˝#FSMC_BCR1_CBURSTRW ((uint32_t)0x00080000) Ä$FSMC_BCR2_MBKEN ((uint32_t)0x00000001) Å$FSMC_BCR2_MUXEN ((uint32_t)0x00000002) É$FSMC_BCR2_MTYP ((uint32_t)0x0000000C) Ñ$FSMC_BCR2_MTYP_0 ((uint32_t)0x00000004) Ö$FSMC_BCR2_MTYP_1 ((uint32_t)0x00000008) á$FSMC_BCR2_MWID ((uint32_t)0x00000030) à$FSMC_BCR2_MWID_0 ((uint32_t)0x00000010) â$FSMC_BCR2_MWID_1 ((uint32_t)0x00000020) ã$FSMC_BCR2_FACCEN ((uint32_t)0x00000040) å$FSMC_BCR2_BURSTEN ((uint32_t)0x00000100) ç$FSMC_BCR2_WAITPOL ((uint32_t)0x00000200) é$FSMC_BCR2_WRAPMOD ((uint32_t)0x00000400) è$FSMC_BCR2_WAITCFG ((uint32_t)0x00000800) ê$FSMC_BCR2_WREN ((uint32_t)0x00001000) ë$FSMC_BCR2_WAITEN ((uint32_t)0x00002000) í$FSMC_BCR2_EXTMOD ((uint32_t)0x00004000) ì$FSMC_BCR2_ASYNCWAIT ((uint32_t)0x00008000) î$FSMC_BCR2_CBURSTRW ((uint32_t)0x00080000) ó$FSMC_BCR3_MBKEN ((uint32_t)0x00000001) ò$FSMC_BCR3_MUXEN ((uint32_t)0x00000002) ö$FSMC_BCR3_MTYP ((uint32_t)0x0000000C) õ$FSMC_BCR3_MTYP_0 ((uint32_t)0x00000004) ú$FSMC_BCR3_MTYP_1 ((uint32_t)0x00000008) û$FSMC_BCR3_MWID ((uint32_t)0x00000030) ü$FSMC_BCR3_MWID_0 ((uint32_t)0x00000010) †$FSMC_BCR3_MWID_1 ((uint32_t)0x00000020) ¢$FSMC_BCR3_FACCEN ((uint32_t)0x00000040) £$FSMC_BCR3_BURSTEN ((uint32_t)0x00000100) §$FSMC_BCR3_WAITPOL ((uint32_t)0x00000200) •$FSMC_BCR3_WRAPMOD ((uint32_t)0x00000400) ¶$FSMC_BCR3_WAITCFG ((uint32_t)0x00000800) ß$FSMC_BCR3_WREN ((uint32_t)0x00001000) ®$FSMC_BCR3_WAITEN ((uint32_t)0x00002000) ©$FSMC_BCR3_EXTMOD ((uint32_t)0x00004000) ™$FSMC_BCR3_ASYNCWAIT ((uint32_t)0x00008000) ´$FSMC_BCR3_CBURSTRW ((uint32_t)0x00080000) Æ$FSMC_BCR4_MBKEN ((uint32_t)0x00000001) Ø$FSMC_BCR4_MUXEN ((uint32_t)0x00000002) ±$FSMC_BCR4_MTYP ((uint32_t)0x0000000C) ≤$FSMC_BCR4_MTYP_0 ((uint32_t)0x00000004) ≥$FSMC_BCR4_MTYP_1 ((uint32_t)0x00000008) µ$FSMC_BCR4_MWID ((uint32_t)0x00000030) ∂$FSMC_BCR4_MWID_0 ((uint32_t)0x00000010) ∑$FSMC_BCR4_MWID_1 ((uint32_t)0x00000020) π$FSMC_BCR4_FACCEN ((uint32_t)0x00000040) ∫$FSMC_BCR4_BURSTEN ((uint32_t)0x00000100) ª$FSMC_BCR4_WAITPOL ((uint32_t)0x00000200) º$FSMC_BCR4_WRAPMOD ((uint32_t)0x00000400) Ω$FSMC_BCR4_WAITCFG ((uint32_t)0x00000800) æ$FSMC_BCR4_WREN ((uint32_t)0x00001000) ø$FSMC_BCR4_WAITEN ((uint32_t)0x00002000) ¿$FSMC_BCR4_EXTMOD ((uint32_t)0x00004000) ¡$FSMC_BCR4_ASYNCWAIT ((uint32_t)0x00008000) ¬$FSMC_BCR4_CBURSTRW ((uint32_t)0x00080000) ≈$FSMC_BTR1_ADDSET ((uint32_t)0x0000000F) ∆$FSMC_BTR1_ADDSET_0 ((uint32_t)0x00000001) «$FSMC_BTR1_ADDSET_1 ((uint32_t)0x00000002) »$FSMC_BTR1_ADDSET_2 ((uint32_t)0x00000004) …$FSMC_BTR1_ADDSET_3 ((uint32_t)0x00000008) À$FSMC_BTR1_ADDHLD ((uint32_t)0x000000F0) Ã$FSMC_BTR1_ADDHLD_0 ((uint32_t)0x00000010) Õ$FSMC_BTR1_ADDHLD_1 ((uint32_t)0x00000020) Œ$FSMC_BTR1_ADDHLD_2 ((uint32_t)0x00000040) œ$FSMC_BTR1_ADDHLD_3 ((uint32_t)0x00000080) —$FSMC_BTR1_DATAST ((uint32_t)0x0000FF00) “$FSMC_BTR1_DATAST_0 ((uint32_t)0x00000100) ”$FSMC_BTR1_DATAST_1 ((uint32_t)0x00000200) ‘$FSMC_BTR1_DATAST_2 ((uint32_t)0x00000400) ’$FSMC_BTR1_DATAST_3 ((uint32_t)0x00000800) ◊$FSMC_BTR1_BUSTURN ((uint32_t)0x000F0000) ÿ$FSMC_BTR1_BUSTURN_0 ((uint32_t)0x00010000) Ÿ$FSMC_BTR1_BUSTURN_1 ((uint32_t)0x00020000) ⁄$FSMC_BTR1_BUSTURN_2 ((uint32_t)0x00040000) €$FSMC_BTR1_BUSTURN_3 ((uint32_t)0x00080000) ›$FSMC_BTR1_CLKDIV ((uint32_t)0x00F00000) ﬁ$FSMC_BTR1_CLKDIV_0 ((uint32_t)0x00100000) ﬂ$FSMC_BTR1_CLKDIV_1 ((uint32_t)0x00200000) ‡$FSMC_BTR1_CLKDIV_2 ((uint32_t)0x00400000) ·$FSMC_BTR1_CLKDIV_3 ((uint32_t)0x00800000) „$FSMC_BTR1_DATLAT ((uint32_t)0x0F000000) ‰$FSMC_BTR1_DATLAT_0 ((uint32_t)0x01000000) Â$FSMC_BTR1_DATLAT_1 ((uint32_t)0x02000000) Ê$FSMC_BTR1_DATLAT_2 ((uint32_t)0x04000000) Á$FSMC_BTR1_DATLAT_3 ((uint32_t)0x08000000) È$FSMC_BTR1_ACCMOD ((uint32_t)0x30000000) Í$FSMC_BTR1_ACCMOD_0 ((uint32_t)0x10000000) Î$FSMC_BTR1_ACCMOD_1 ((uint32_t)0x20000000) Ó$FSMC_BTR2_ADDSET ((uint32_t)0x0000000F) Ô$FSMC_BTR2_ADDSET_0 ((uint32_t)0x00000001) $FSMC_BTR2_ADDSET_1 ((uint32_t)0x00000002) Ò$FSMC_BTR2_ADDSET_2 ((uint32_t)0x00000004) Ú$FSMC_BTR2_ADDSET_3 ((uint32_t)0x00000008) Ù$FSMC_BTR2_ADDHLD ((uint32_t)0x000000F0) ı$FSMC_BTR2_ADDHLD_0 ((uint32_t)0x00000010) ˆ$FSMC_BTR2_ADDHLD_1 ((uint32_t)0x00000020) ˜$FSMC_BTR2_ADDHLD_2 ((uint32_t)0x00000040) ¯$FSMC_BTR2_ADDHLD_3 ((uint32_t)0x00000080) ˙$FSMC_BTR2_DATAST ((uint32_t)0x0000FF00) ˚$FSMC_BTR2_DATAST_0 ((uint32_t)0x00000100) ¸$FSMC_BTR2_DATAST_1 ((uint32_t)0x00000200) ˝$FSMC_BTR2_DATAST_2 ((uint32_t)0x00000400) ˛$FSMC_BTR2_DATAST_3 ((uint32_t)0x00000800) Ä%FSMC_BTR2_BUSTURN ((uint32_t)0x000F0000) Å%FSMC_BTR2_BUSTURN_0 ((uint32_t)0x00010000) Ç%FSMC_BTR2_BUSTURN_1 ((uint32_t)0x00020000) É%FSMC_BTR2_BUSTURN_2 ((uint32_t)0x00040000) Ñ%FSMC_BTR2_BUSTURN_3 ((uint32_t)0x00080000) Ü%FSMC_BTR2_CLKDIV ((uint32_t)0x00F00000) á%FSMC_BTR2_CLKDIV_0 ((uint32_t)0x00100000) à%FSMC_BTR2_CLKDIV_1 ((uint32_t)0x00200000) â%FSMC_BTR2_CLKDIV_2 ((uint32_t)0x00400000) ä%FSMC_BTR2_CLKDIV_3 ((uint32_t)0x00800000) å%FSMC_BTR2_DATLAT ((uint32_t)0x0F000000) ç%FSMC_BTR2_DATLAT_0 ((uint32_t)0x01000000) é%FSMC_BTR2_DATLAT_1 ((uint32_t)0x02000000) è%FSMC_BTR2_DATLAT_2 ((uint32_t)0x04000000) ê%FSMC_BTR2_DATLAT_3 ((uint32_t)0x08000000) í%FSMC_BTR2_ACCMOD ((uint32_t)0x30000000) ì%FSMC_BTR2_ACCMOD_0 ((uint32_t)0x10000000) î%FSMC_BTR2_ACCMOD_1 ((uint32_t)0x20000000) ó%FSMC_BTR3_ADDSET ((uint32_t)0x0000000F) ò%FSMC_BTR3_ADDSET_0 ((uint32_t)0x00000001) ô%FSMC_BTR3_ADDSET_1 ((uint32_t)0x00000002) ö%FSMC_BTR3_ADDSET_2 ((uint32_t)0x00000004) õ%FSMC_BTR3_ADDSET_3 ((uint32_t)0x00000008) ù%FSMC_BTR3_ADDHLD ((uint32_t)0x000000F0) û%FSMC_BTR3_ADDHLD_0 ((uint32_t)0x00000010) ü%FSMC_BTR3_ADDHLD_1 ((uint32_t)0x00000020) †%FSMC_BTR3_ADDHLD_2 ((uint32_t)0x00000040) °%FSMC_BTR3_ADDHLD_3 ((uint32_t)0x00000080) £%FSMC_BTR3_DATAST ((uint32_t)0x0000FF00) §%FSMC_BTR3_DATAST_0 ((uint32_t)0x00000100) •%FSMC_BTR3_DATAST_1 ((uint32_t)0x00000200) ¶%FSMC_BTR3_DATAST_2 ((uint32_t)0x00000400) ß%FSMC_BTR3_DATAST_3 ((uint32_t)0x00000800) ©%FSMC_BTR3_BUSTURN ((uint32_t)0x000F0000) ™%FSMC_BTR3_BUSTURN_0 ((uint32_t)0x00010000) ´%FSMC_BTR3_BUSTURN_1 ((uint32_t)0x00020000) ¨%FSMC_BTR3_BUSTURN_2 ((uint32_t)0x00040000) ≠%FSMC_BTR3_BUSTURN_3 ((uint32_t)0x00080000) Ø%FSMC_BTR3_CLKDIV ((uint32_t)0x00F00000) ∞%FSMC_BTR3_CLKDIV_0 ((uint32_t)0x00100000) ±%FSMC_BTR3_CLKDIV_1 ((uint32_t)0x00200000) ≤%FSMC_BTR3_CLKDIV_2 ((uint32_t)0x00400000) ≥%FSMC_BTR3_CLKDIV_3 ((uint32_t)0x00800000) µ%FSMC_BTR3_DATLAT ((uint32_t)0x0F000000) ∂%FSMC_BTR3_DATLAT_0 ((uint32_t)0x01000000) ∑%FSMC_BTR3_DATLAT_1 ((uint32_t)0x02000000) ∏%FSMC_BTR3_DATLAT_2 ((uint32_t)0x04000000) π%FSMC_BTR3_DATLAT_3 ((uint32_t)0x08000000) ª%FSMC_BTR3_ACCMOD ((uint32_t)0x30000000) º%FSMC_BTR3_ACCMOD_0 ((uint32_t)0x10000000) Ω%FSMC_BTR3_ACCMOD_1 ((uint32_t)0x20000000) ¿%FSMC_BTR4_ADDSET ((uint32_t)0x0000000F) ¡%FSMC_BTR4_ADDSET_0 ((uint32_t)0x00000001) ¬%FSMC_BTR4_ADDSET_1 ((uint32_t)0x00000002) √%FSMC_BTR4_ADDSET_2 ((uint32_t)0x00000004) ƒ%FSMC_BTR4_ADDSET_3 ((uint32_t)0x00000008) ∆%FSMC_BTR4_ADDHLD ((uint32_t)0x000000F0) «%FSMC_BTR4_ADDHLD_0 ((uint32_t)0x00000010) »%FSMC_BTR4_ADDHLD_1 ((uint32_t)0x00000020) …%FSMC_BTR4_ADDHLD_2 ((uint32_t)0x00000040)  %FSMC_BTR4_ADDHLD_3 ((uint32_t)0x00000080) Ã%FSMC_BTR4_DATAST ((uint32_t)0x0000FF00) Õ%FSMC_BTR4_DATAST_0 ((uint32_t)0x00000100) Œ%FSMC_BTR4_DATAST_1 ((uint32_t)0x00000200) œ%FSMC_BTR4_DATAST_2 ((uint32_t)0x00000400) –%FSMC_BTR4_DATAST_3 ((uint32_t)0x00000800) “%FSMC_BTR4_BUSTURN ((uint32_t)0x000F0000) ”%FSMC_BTR4_BUSTURN_0 ((uint32_t)0x00010000) ‘%FSMC_BTR4_BUSTURN_1 ((uint32_t)0x00020000) ’%FSMC_BTR4_BUSTURN_2 ((uint32_t)0x00040000) ÷%FSMC_BTR4_BUSTURN_3 ((uint32_t)0x00080000) ÿ%FSMC_BTR4_CLKDIV ((uint32_t)0x00F00000) Ÿ%FSMC_BTR4_CLKDIV_0 ((uint32_t)0x00100000) ⁄%FSMC_BTR4_CLKDIV_1 ((uint32_t)0x00200000) €%FSMC_BTR4_CLKDIV_2 ((uint32_t)0x00400000) ‹%FSMC_BTR4_CLKDIV_3 ((uint32_t)0x00800000) ﬁ%FSMC_BTR4_DATLAT ((uint32_t)0x0F000000) ﬂ%FSMC_BTR4_DATLAT_0 ((uint32_t)0x01000000) ‡%FSMC_BTR4_DATLAT_1 ((uint32_t)0x02000000) ·%FSMC_BTR4_DATLAT_2 ((uint32_t)0x04000000) ‚%FSMC_BTR4_DATLAT_3 ((uint32_t)0x08000000) ‰%FSMC_BTR4_ACCMOD ((uint32_t)0x30000000) Â%FSMC_BTR4_ACCMOD_0 ((uint32_t)0x10000000) Ê%FSMC_BTR4_ACCMOD_1 ((uint32_t)0x20000000) È%FSMC_BWTR1_ADDSET ((uint32_t)0x0000000F) Í%FSMC_BWTR1_ADDSET_0 ((uint32_t)0x00000001) Î%FSMC_BWTR1_ADDSET_1 ((uint32_t)0x00000002) Ï%FSMC_BWTR1_ADDSET_2 ((uint32_t)0x00000004) Ì%FSMC_BWTR1_ADDSET_3 ((uint32_t)0x00000008) Ô%FSMC_BWTR1_ADDHLD ((uint32_t)0x000000F0) %FSMC_BWTR1_ADDHLD_0 ((uint32_t)0x00000010) Ò%FSMC_BWTR1_ADDHLD_1 ((uint32_t)0x00000020) Ú%FSMC_BWTR1_ADDHLD_2 ((uint32_t)0x00000040) Û%FSMC_BWTR1_ADDHLD_3 ((uint32_t)0x00000080) ı%FSMC_BWTR1_DATAST ((uint32_t)0x0000FF00) ˆ%FSMC_BWTR1_DATAST_0 ((uint32_t)0x00000100) ˜%FSMC_BWTR1_DATAST_1 ((uint32_t)0x00000200) ¯%FSMC_BWTR1_DATAST_2 ((uint32_t)0x00000400) ˘%FSMC_BWTR1_DATAST_3 ((uint32_t)0x00000800) ˚%FSMC_BWTR1_CLKDIV ((uint32_t)0x00F00000) ¸%FSMC_BWTR1_CLKDIV_0 ((uint32_t)0x00100000) ˝%FSMC_BWTR1_CLKDIV_1 ((uint32_t)0x00200000) ˛%FSMC_BWTR1_CLKDIV_2 ((uint32_t)0x00400000) ˇ%FSMC_BWTR1_CLKDIV_3 ((uint32_t)0x00800000) Å&FSMC_BWTR1_DATLAT ((uint32_t)0x0F000000) Ç&FSMC_BWTR1_DATLAT_0 ((uint32_t)0x01000000) É&FSMC_BWTR1_DATLAT_1 ((uint32_t)0x02000000) Ñ&FSMC_BWTR1_DATLAT_2 ((uint32_t)0x04000000) Ö&FSMC_BWTR1_DATLAT_3 ((uint32_t)0x08000000) á&FSMC_BWTR1_ACCMOD ((uint32_t)0x30000000) à&FSMC_BWTR1_ACCMOD_0 ((uint32_t)0x10000000) â&FSMC_BWTR1_ACCMOD_1 ((uint32_t)0x20000000) å&FSMC_BWTR2_ADDSET ((uint32_t)0x0000000F) ç&FSMC_BWTR2_ADDSET_0 ((uint32_t)0x00000001) é&FSMC_BWTR2_ADDSET_1 ((uint32_t)0x00000002) è&FSMC_BWTR2_ADDSET_2 ((uint32_t)0x00000004) ê&FSMC_BWTR2_ADDSET_3 ((uint32_t)0x00000008) í&FSMC_BWTR2_ADDHLD ((uint32_t)0x000000F0) ì&FSMC_BWTR2_ADDHLD_0 ((uint32_t)0x00000010) î&FSMC_BWTR2_ADDHLD_1 ((uint32_t)0x00000020) ï&FSMC_BWTR2_ADDHLD_2 ((uint32_t)0x00000040) ñ&FSMC_BWTR2_ADDHLD_3 ((uint32_t)0x00000080) ò&FSMC_BWTR2_DATAST ((uint32_t)0x0000FF00) ô&FSMC_BWTR2_DATAST_0 ((uint32_t)0x00000100) ö&FSMC_BWTR2_DATAST_1 ((uint32_t)0x00000200) õ&FSMC_BWTR2_DATAST_2 ((uint32_t)0x00000400) ú&FSMC_BWTR2_DATAST_3 ((uint32_t)0x00000800) û&FSMC_BWTR2_CLKDIV ((uint32_t)0x00F00000) ü&FSMC_BWTR2_CLKDIV_0 ((uint32_t)0x00100000) †&FSMC_BWTR2_CLKDIV_1 ((uint32_t)0x00200000) °&FSMC_BWTR2_CLKDIV_2 ((uint32_t)0x00400000) ¢&FSMC_BWTR2_CLKDIV_3 ((uint32_t)0x00800000) §&FSMC_BWTR2_DATLAT ((uint32_t)0x0F000000) •&FSMC_BWTR2_DATLAT_0 ((uint32_t)0x01000000) ¶&FSMC_BWTR2_DATLAT_1 ((uint32_t)0x02000000) ß&FSMC_BWTR2_DATLAT_2 ((uint32_t)0x04000000) ®&FSMC_BWTR2_DATLAT_3 ((uint32_t)0x08000000) ™&FSMC_BWTR2_ACCMOD ((uint32_t)0x30000000) ´&FSMC_BWTR2_ACCMOD_0 ((uint32_t)0x10000000) ¨&FSMC_BWTR2_ACCMOD_1 ((uint32_t)0x20000000) Ø&FSMC_BWTR3_ADDSET ((uint32_t)0x0000000F) ∞&FSMC_BWTR3_ADDSET_0 ((uint32_t)0x00000001) ±&FSMC_BWTR3_ADDSET_1 ((uint32_t)0x00000002) ≤&FSMC_BWTR3_ADDSET_2 ((uint32_t)0x00000004) ≥&FSMC_BWTR3_ADDSET_3 ((uint32_t)0x00000008) µ&FSMC_BWTR3_ADDHLD ((uint32_t)0x000000F0) ∂&FSMC_BWTR3_ADDHLD_0 ((uint32_t)0x00000010) ∑&FSMC_BWTR3_ADDHLD_1 ((uint32_t)0x00000020) ∏&FSMC_BWTR3_ADDHLD_2 ((uint32_t)0x00000040) π&FSMC_BWTR3_ADDHLD_3 ((uint32_t)0x00000080) ª&FSMC_BWTR3_DATAST ((uint32_t)0x0000FF00) º&FSMC_BWTR3_DATAST_0 ((uint32_t)0x00000100) Ω&FSMC_BWTR3_DATAST_1 ((uint32_t)0x00000200) æ&FSMC_BWTR3_DATAST_2 ((uint32_t)0x00000400) ø&FSMC_BWTR3_DATAST_3 ((uint32_t)0x00000800) ¡&FSMC_BWTR3_CLKDIV ((uint32_t)0x00F00000) ¬&FSMC_BWTR3_CLKDIV_0 ((uint32_t)0x00100000) √&FSMC_BWTR3_CLKDIV_1 ((uint32_t)0x00200000) ƒ&FSMC_BWTR3_CLKDIV_2 ((uint32_t)0x00400000) ≈&FSMC_BWTR3_CLKDIV_3 ((uint32_t)0x00800000) «&FSMC_BWTR3_DATLAT ((uint32_t)0x0F000000) »&FSMC_BWTR3_DATLAT_0 ((uint32_t)0x01000000) …&FSMC_BWTR3_DATLAT_1 ((uint32_t)0x02000000)  &FSMC_BWTR3_DATLAT_2 ((uint32_t)0x04000000) À&FSMC_BWTR3_DATLAT_3 ((uint32_t)0x08000000) Õ&FSMC_BWTR3_ACCMOD ((uint32_t)0x30000000) Œ&FSMC_BWTR3_ACCMOD_0 ((uint32_t)0x10000000) œ&FSMC_BWTR3_ACCMOD_1 ((uint32_t)0x20000000) “&FSMC_BWTR4_ADDSET ((uint32_t)0x0000000F) ”&FSMC_BWTR4_ADDSET_0 ((uint32_t)0x00000001) ‘&FSMC_BWTR4_ADDSET_1 ((uint32_t)0x00000002) ’&FSMC_BWTR4_ADDSET_2 ((uint32_t)0x00000004) ÷&FSMC_BWTR4_ADDSET_3 ((uint32_t)0x00000008) ÿ&FSMC_BWTR4_ADDHLD ((uint32_t)0x000000F0) Ÿ&FSMC_BWTR4_ADDHLD_0 ((uint32_t)0x00000010) ⁄&FSMC_BWTR4_ADDHLD_1 ((uint32_t)0x00000020) €&FSMC_BWTR4_ADDHLD_2 ((uint32_t)0x00000040) ‹&FSMC_BWTR4_ADDHLD_3 ((uint32_t)0x00000080) ﬁ&FSMC_BWTR4_DATAST ((uint32_t)0x0000FF00) ﬂ&FSMC_BWTR4_DATAST_0 ((uint32_t)0x00000100) ‡&FSMC_BWTR4_DATAST_1 ((uint32_t)0x00000200) ·&FSMC_BWTR4_DATAST_2 ((uint32_t)0x00000400) ‚&FSMC_BWTR4_DATAST_3 ((uint32_t)0x00000800) ‰&FSMC_BWTR4_CLKDIV ((uint32_t)0x00F00000) Â&FSMC_BWTR4_CLKDIV_0 ((uint32_t)0x00100000) Ê&FSMC_BWTR4_CLKDIV_1 ((uint32_t)0x00200000) Á&FSMC_BWTR4_CLKDIV_2 ((uint32_t)0x00400000) Ë&FSMC_BWTR4_CLKDIV_3 ((uint32_t)0x00800000) Í&FSMC_BWTR4_DATLAT ((uint32_t)0x0F000000) Î&FSMC_BWTR4_DATLAT_0 ((uint32_t)0x01000000) Ï&FSMC_BWTR4_DATLAT_1 ((uint32_t)0x02000000) Ì&FSMC_BWTR4_DATLAT_2 ((uint32_t)0x04000000) Ó&FSMC_BWTR4_DATLAT_3 ((uint32_t)0x08000000) &FSMC_BWTR4_ACCMOD ((uint32_t)0x30000000) Ò&FSMC_BWTR4_ACCMOD_0 ((uint32_t)0x10000000) Ú&FSMC_BWTR4_ACCMOD_1 ((uint32_t)0x20000000) ı&FSMC_PCR2_PWAITEN ((uint32_t)0x00000002) ˆ&FSMC_PCR2_PBKEN ((uint32_t)0x00000004) ˜&FSMC_PCR2_PTYP ((uint32_t)0x00000008) ˘&FSMC_PCR2_PWID ((uint32_t)0x00000030) ˙&FSMC_PCR2_PWID_0 ((uint32_t)0x00000010) ˚&FSMC_PCR2_PWID_1 ((uint32_t)0x00000020) ˝&FSMC_PCR2_ECCEN ((uint32_t)0x00000040) ˇ&FSMC_PCR2_TCLR ((uint32_t)0x00001E00) Ä'FSMC_PCR2_TCLR_0 ((uint32_t)0x00000200) Å'FSMC_PCR2_TCLR_1 ((uint32_t)0x00000400) Ç'FSMC_PCR2_TCLR_2 ((uint32_t)0x00000800) É'FSMC_PCR2_TCLR_3 ((uint32_t)0x00001000) Ö'FSMC_PCR2_TAR ((uint32_t)0x0001E000) Ü'FSMC_PCR2_TAR_0 ((uint32_t)0x00002000) á'FSMC_PCR2_TAR_1 ((uint32_t)0x00004000) à'FSMC_PCR2_TAR_2 ((uint32_t)0x00008000) â'FSMC_PCR2_TAR_3 ((uint32_t)0x00010000) ã'FSMC_PCR2_ECCPS ((uint32_t)0x000E0000) å'FSMC_PCR2_ECCPS_0 ((uint32_t)0x00020000) ç'FSMC_PCR2_ECCPS_1 ((uint32_t)0x00040000) é'FSMC_PCR2_ECCPS_2 ((uint32_t)0x00080000) ë'FSMC_PCR3_PWAITEN ((uint32_t)0x00000002) í'FSMC_PCR3_PBKEN ((uint32_t)0x00000004) ì'FSMC_PCR3_PTYP ((uint32_t)0x00000008) ï'FSMC_PCR3_PWID ((uint32_t)0x00000030) ñ'FSMC_PCR3_PWID_0 ((uint32_t)0x00000010) ó'FSMC_PCR3_PWID_1 ((uint32_t)0x00000020) ô'FSMC_PCR3_ECCEN ((uint32_t)0x00000040) õ'FSMC_PCR3_TCLR ((uint32_t)0x00001E00) ú'FSMC_PCR3_TCLR_0 ((uint32_t)0x00000200) ù'FSMC_PCR3_TCLR_1 ((uint32_t)0x00000400) û'FSMC_PCR3_TCLR_2 ((uint32_t)0x00000800) ü'FSMC_PCR3_TCLR_3 ((uint32_t)0x00001000) °'FSMC_PCR3_TAR ((uint32_t)0x0001E000) ¢'FSMC_PCR3_TAR_0 ((uint32_t)0x00002000) £'FSMC_PCR3_TAR_1 ((uint32_t)0x00004000) §'FSMC_PCR3_TAR_2 ((uint32_t)0x00008000) •'FSMC_PCR3_TAR_3 ((uint32_t)0x00010000) ß'FSMC_PCR3_ECCPS ((uint32_t)0x000E0000) ®'FSMC_PCR3_ECCPS_0 ((uint32_t)0x00020000) ©'FSMC_PCR3_ECCPS_1 ((uint32_t)0x00040000) ™'FSMC_PCR3_ECCPS_2 ((uint32_t)0x00080000) ≠'FSMC_PCR4_PWAITEN ((uint32_t)0x00000002) Æ'FSMC_PCR4_PBKEN ((uint32_t)0x00000004) Ø'FSMC_PCR4_PTYP ((uint32_t)0x00000008) ±'FSMC_PCR4_PWID ((uint32_t)0x00000030) ≤'FSMC_PCR4_PWID_0 ((uint32_t)0x00000010) ≥'FSMC_PCR4_PWID_1 ((uint32_t)0x00000020) µ'FSMC_PCR4_ECCEN ((uint32_t)0x00000040) ∑'FSMC_PCR4_TCLR ((uint32_t)0x00001E00) ∏'FSMC_PCR4_TCLR_0 ((uint32_t)0x00000200) π'FSMC_PCR4_TCLR_1 ((uint32_t)0x00000400) ∫'FSMC_PCR4_TCLR_2 ((uint32_t)0x00000800) ª'FSMC_PCR4_TCLR_3 ((uint32_t)0x00001000) Ω'FSMC_PCR4_TAR ((uint32_t)0x0001E000) æ'FSMC_PCR4_TAR_0 ((uint32_t)0x00002000) ø'FSMC_PCR4_TAR_1 ((uint32_t)0x00004000) ¿'FSMC_PCR4_TAR_2 ((uint32_t)0x00008000) ¡'FSMC_PCR4_TAR_3 ((uint32_t)0x00010000) √'FSMC_PCR4_ECCPS ((uint32_t)0x000E0000) ƒ'FSMC_PCR4_ECCPS_0 ((uint32_t)0x00020000) ≈'FSMC_PCR4_ECCPS_1 ((uint32_t)0x00040000) ∆'FSMC_PCR4_ECCPS_2 ((uint32_t)0x00080000) …'FSMC_SR2_IRS ((uint8_t)0x01)  'FSMC_SR2_ILS ((uint8_t)0x02) À'FSMC_SR2_IFS ((uint8_t)0x04) Ã'FSMC_SR2_IREN ((uint8_t)0x08) Õ'FSMC_SR2_ILEN ((uint8_t)0x10) Œ'FSMC_SR2_IFEN ((uint8_t)0x20) œ'FSMC_SR2_FEMPT ((uint8_t)0x40) “'FSMC_SR3_IRS ((uint8_t)0x01) ”'FSMC_SR3_ILS ((uint8_t)0x02) ‘'FSMC_SR3_IFS ((uint8_t)0x04) ’'FSMC_SR3_IREN ((uint8_t)0x08) ÷'FSMC_SR3_ILEN ((uint8_t)0x10) ◊'FSMC_SR3_IFEN ((uint8_t)0x20) ÿ'FSMC_SR3_FEMPT ((uint8_t)0x40) €'FSMC_SR4_IRS ((uint8_t)0x01) ‹'FSMC_SR4_ILS ((uint8_t)0x02) ›'FSMC_SR4_IFS ((uint8_t)0x04) ﬁ'FSMC_SR4_IREN ((uint8_t)0x08) ﬂ'FSMC_SR4_ILEN ((uint8_t)0x10) ‡'FSMC_SR4_IFEN ((uint8_t)0x20) ·'FSMC_SR4_FEMPT ((uint8_t)0x40) ‰'FSMC_PMEM2_MEMSET2 ((uint32_t)0x000000FF) Â'FSMC_PMEM2_MEMSET2_0 ((uint32_t)0x00000001) Ê'FSMC_PMEM2_MEMSET2_1 ((uint32_t)0x00000002) Á'FSMC_PMEM2_MEMSET2_2 ((uint32_t)0x00000004) Ë'FSMC_PMEM2_MEMSET2_3 ((uint32_t)0x00000008) È'FSMC_PMEM2_MEMSET2_4 ((uint32_t)0x00000010) Í'FSMC_PMEM2_MEMSET2_5 ((uint32_t)0x00000020) Î'FSMC_PMEM2_MEMSET2_6 ((uint32_t)0x00000040) Ï'FSMC_PMEM2_MEMSET2_7 ((uint32_t)0x00000080) Ó'FSMC_PMEM2_MEMWAIT2 ((uint32_t)0x0000FF00) Ô'FSMC_PMEM2_MEMWAIT2_0 ((uint32_t)0x00000100) 'FSMC_PMEM2_MEMWAIT2_1 ((uint32_t)0x00000200) Ò'FSMC_PMEM2_MEMWAIT2_2 ((uint32_t)0x00000400) Ú'FSMC_PMEM2_MEMWAIT2_3 ((uint32_t)0x00000800) Û'FSMC_PMEM2_MEMWAIT2_4 ((uint32_t)0x00001000) Ù'FSMC_PMEM2_MEMWAIT2_5 ((uint32_t)0x00002000) ı'FSMC_PMEM2_MEMWAIT2_6 ((uint32_t)0x00004000) ˆ'FSMC_PMEM2_MEMWAIT2_7 ((uint32_t)0x00008000) ¯'FSMC_PMEM2_MEMHOLD2 ((uint32_t)0x00FF0000) ˘'FSMC_PMEM2_MEMHOLD2_0 ((uint32_t)0x00010000) ˙'FSMC_PMEM2_MEMHOLD2_1 ((uint32_t)0x00020000) ˚'FSMC_PMEM2_MEMHOLD2_2 ((uint32_t)0x00040000) ¸'FSMC_PMEM2_MEMHOLD2_3 ((uint32_t)0x00080000) ˝'FSMC_PMEM2_MEMHOLD2_4 ((uint32_t)0x00100000) ˛'FSMC_PMEM2_MEMHOLD2_5 ((uint32_t)0x00200000) ˇ'FSMC_PMEM2_MEMHOLD2_6 ((uint32_t)0x00400000) Ä(FSMC_PMEM2_MEMHOLD2_7 ((uint32_t)0x00800000) Ç(FSMC_PMEM2_MEMHIZ2 ((uint32_t)0xFF000000) É(FSMC_PMEM2_MEMHIZ2_0 ((uint32_t)0x01000000) Ñ(FSMC_PMEM2_MEMHIZ2_1 ((uint32_t)0x02000000) Ö(FSMC_PMEM2_MEMHIZ2_2 ((uint32_t)0x04000000) Ü(FSMC_PMEM2_MEMHIZ2_3 ((uint32_t)0x08000000) á(FSMC_PMEM2_MEMHIZ2_4 ((uint32_t)0x10000000) à(FSMC_PMEM2_MEMHIZ2_5 ((uint32_t)0x20000000) â(FSMC_PMEM2_MEMHIZ2_6 ((uint32_t)0x40000000) ä(FSMC_PMEM2_MEMHIZ2_7 ((uint32_t)0x80000000) ç(FSMC_PMEM3_MEMSET3 ((uint32_t)0x000000FF) é(FSMC_PMEM3_MEMSET3_0 ((uint32_t)0x00000001) è(FSMC_PMEM3_MEMSET3_1 ((uint32_t)0x00000002) ê(FSMC_PMEM3_MEMSET3_2 ((uint32_t)0x00000004) ë(FSMC_PMEM3_MEMSET3_3 ((uint32_t)0x00000008) í(FSMC_PMEM3_MEMSET3_4 ((uint32_t)0x00000010) ì(FSMC_PMEM3_MEMSET3_5 ((uint32_t)0x00000020) î(FSMC_PMEM3_MEMSET3_6 ((uint32_t)0x00000040) ï(FSMC_PMEM3_MEMSET3_7 ((uint32_t)0x00000080) ó(FSMC_PMEM3_MEMWAIT3 ((uint32_t)0x0000FF00) ò(FSMC_PMEM3_MEMWAIT3_0 ((uint32_t)0x00000100) ô(FSMC_PMEM3_MEMWAIT3_1 ((uint32_t)0x00000200) ö(FSMC_PMEM3_MEMWAIT3_2 ((uint32_t)0x00000400) õ(FSMC_PMEM3_MEMWAIT3_3 ((uint32_t)0x00000800) ú(FSMC_PMEM3_MEMWAIT3_4 ((uint32_t)0x00001000) ù(FSMC_PMEM3_MEMWAIT3_5 ((uint32_t)0x00002000) û(FSMC_PMEM3_MEMWAIT3_6 ((uint32_t)0x00004000) ü(FSMC_PMEM3_MEMWAIT3_7 ((uint32_t)0x00008000) °(FSMC_PMEM3_MEMHOLD3 ((uint32_t)0x00FF0000) ¢(FSMC_PMEM3_MEMHOLD3_0 ((uint32_t)0x00010000) £(FSMC_PMEM3_MEMHOLD3_1 ((uint32_t)0x00020000) §(FSMC_PMEM3_MEMHOLD3_2 ((uint32_t)0x00040000) •(FSMC_PMEM3_MEMHOLD3_3 ((uint32_t)0x00080000) ¶(FSMC_PMEM3_MEMHOLD3_4 ((uint32_t)0x00100000) ß(FSMC_PMEM3_MEMHOLD3_5 ((uint32_t)0x00200000) ®(FSMC_PMEM3_MEMHOLD3_6 ((uint32_t)0x00400000) ©(FSMC_PMEM3_MEMHOLD3_7 ((uint32_t)0x00800000) ´(FSMC_PMEM3_MEMHIZ3 ((uint32_t)0xFF000000) ¨(FSMC_PMEM3_MEMHIZ3_0 ((uint32_t)0x01000000) ≠(FSMC_PMEM3_MEMHIZ3_1 ((uint32_t)0x02000000) Æ(FSMC_PMEM3_MEMHIZ3_2 ((uint32_t)0x04000000) Ø(FSMC_PMEM3_MEMHIZ3_3 ((uint32_t)0x08000000) ∞(FSMC_PMEM3_MEMHIZ3_4 ((uint32_t)0x10000000) ±(FSMC_PMEM3_MEMHIZ3_5 ((uint32_t)0x20000000) ≤(FSMC_PMEM3_MEMHIZ3_6 ((uint32_t)0x40000000) ≥(FSMC_PMEM3_MEMHIZ3_7 ((uint32_t)0x80000000) ∂(FSMC_PMEM4_MEMSET4 ((uint32_t)0x000000FF) ∑(FSMC_PMEM4_MEMSET4_0 ((uint32_t)0x00000001) ∏(FSMC_PMEM4_MEMSET4_1 ((uint32_t)0x00000002) π(FSMC_PMEM4_MEMSET4_2 ((uint32_t)0x00000004) ∫(FSMC_PMEM4_MEMSET4_3 ((uint32_t)0x00000008) ª(FSMC_PMEM4_MEMSET4_4 ((uint32_t)0x00000010) º(FSMC_PMEM4_MEMSET4_5 ((uint32_t)0x00000020) Ω(FSMC_PMEM4_MEMSET4_6 ((uint32_t)0x00000040) æ(FSMC_PMEM4_MEMSET4_7 ((uint32_t)0x00000080) ¿(FSMC_PMEM4_MEMWAIT4 ((uint32_t)0x0000FF00) ¡(FSMC_PMEM4_MEMWAIT4_0 ((uint32_t)0x00000100) ¬(FSMC_PMEM4_MEMWAIT4_1 ((uint32_t)0x00000200) √(FSMC_PMEM4_MEMWAIT4_2 ((uint32_t)0x00000400) ƒ(FSMC_PMEM4_MEMWAIT4_3 ((uint32_t)0x00000800) ≈(FSMC_PMEM4_MEMWAIT4_4 ((uint32_t)0x00001000) ∆(FSMC_PMEM4_MEMWAIT4_5 ((uint32_t)0x00002000) «(FSMC_PMEM4_MEMWAIT4_6 ((uint32_t)0x00004000) »(FSMC_PMEM4_MEMWAIT4_7 ((uint32_t)0x00008000)  (FSMC_PMEM4_MEMHOLD4 ((uint32_t)0x00FF0000) À(FSMC_PMEM4_MEMHOLD4_0 ((uint32_t)0x00010000) Ã(FSMC_PMEM4_MEMHOLD4_1 ((uint32_t)0x00020000) Õ(FSMC_PMEM4_MEMHOLD4_2 ((uint32_t)0x00040000) Œ(FSMC_PMEM4_MEMHOLD4_3 ((uint32_t)0x00080000) œ(FSMC_PMEM4_MEMHOLD4_4 ((uint32_t)0x00100000) –(FSMC_PMEM4_MEMHOLD4_5 ((uint32_t)0x00200000) —(FSMC_PMEM4_MEMHOLD4_6 ((uint32_t)0x00400000) “(FSMC_PMEM4_MEMHOLD4_7 ((uint32_t)0x00800000) ‘(FSMC_PMEM4_MEMHIZ4 ((uint32_t)0xFF000000) ’(FSMC_PMEM4_MEMHIZ4_0 ((uint32_t)0x01000000) ÷(FSMC_PMEM4_MEMHIZ4_1 ((uint32_t)0x02000000) ◊(FSMC_PMEM4_MEMHIZ4_2 ((uint32_t)0x04000000) ÿ(FSMC_PMEM4_MEMHIZ4_3 ((uint32_t)0x08000000) Ÿ(FSMC_PMEM4_MEMHIZ4_4 ((uint32_t)0x10000000) ⁄(FSMC_PMEM4_MEMHIZ4_5 ((uint32_t)0x20000000) €(FSMC_PMEM4_MEMHIZ4_6 ((uint32_t)0x40000000) ‹(FSMC_PMEM4_MEMHIZ4_7 ((uint32_t)0x80000000) ﬂ(FSMC_PATT2_ATTSET2 ((uint32_t)0x000000FF) ‡(FSMC_PATT2_ATTSET2_0 ((uint32_t)0x00000001) ·(FSMC_PATT2_ATTSET2_1 ((uint32_t)0x00000002) ‚(FSMC_PATT2_ATTSET2_2 ((uint32_t)0x00000004) „(FSMC_PATT2_ATTSET2_3 ((uint32_t)0x00000008) ‰(FSMC_PATT2_ATTSET2_4 ((uint32_t)0x00000010) Â(FSMC_PATT2_ATTSET2_5 ((uint32_t)0x00000020) Ê(FSMC_PATT2_ATTSET2_6 ((uint32_t)0x00000040) Á(FSMC_PATT2_ATTSET2_7 ((uint32_t)0x00000080) È(FSMC_PATT2_ATTWAIT2 ((uint32_t)0x0000FF00) Í(FSMC_PATT2_ATTWAIT2_0 ((uint32_t)0x00000100) Î(FSMC_PATT2_ATTWAIT2_1 ((uint32_t)0x00000200) Ï(FSMC_PATT2_ATTWAIT2_2 ((uint32_t)0x00000400) Ì(FSMC_PATT2_ATTWAIT2_3 ((uint32_t)0x00000800) Ó(FSMC_PATT2_ATTWAIT2_4 ((uint32_t)0x00001000) Ô(FSMC_PATT2_ATTWAIT2_5 ((uint32_t)0x00002000) (FSMC_PATT2_ATTWAIT2_6 ((uint32_t)0x00004000) Ò(FSMC_PATT2_ATTWAIT2_7 ((uint32_t)0x00008000) Û(FSMC_PATT2_ATTHOLD2 ((uint32_t)0x00FF0000) Ù(FSMC_PATT2_ATTHOLD2_0 ((uint32_t)0x00010000) ı(FSMC_PATT2_ATTHOLD2_1 ((uint32_t)0x00020000) ˆ(FSMC_PATT2_ATTHOLD2_2 ((uint32_t)0x00040000) ˜(FSMC_PATT2_ATTHOLD2_3 ((uint32_t)0x00080000) ¯(FSMC_PATT2_ATTHOLD2_4 ((uint32_t)0x00100000) ˘(FSMC_PATT2_ATTHOLD2_5 ((uint32_t)0x00200000) ˙(FSMC_PATT2_ATTHOLD2_6 ((uint32_t)0x00400000) ˚(FSMC_PATT2_ATTHOLD2_7 ((uint32_t)0x00800000) ˝(FSMC_PATT2_ATTHIZ2 ((uint32_t)0xFF000000) ˛(FSMC_PATT2_ATTHIZ2_0 ((uint32_t)0x01000000) ˇ(FSMC_PATT2_ATTHIZ2_1 ((uint32_t)0x02000000) Ä)FSMC_PATT2_ATTHIZ2_2 ((uint32_t)0x04000000) Å)FSMC_PATT2_ATTHIZ2_3 ((uint32_t)0x08000000) Ç)FSMC_PATT2_ATTHIZ2_4 ((uint32_t)0x10000000) É)FSMC_PATT2_ATTHIZ2_5 ((uint32_t)0x20000000) Ñ)FSMC_PATT2_ATTHIZ2_6 ((uint32_t)0x40000000) Ö)FSMC_PATT2_ATTHIZ2_7 ((uint32_t)0x80000000) à)FSMC_PATT3_ATTSET3 ((uint32_t)0x000000FF) â)FSMC_PATT3_ATTSET3_0 ((uint32_t)0x00000001) ä)FSMC_PATT3_ATTSET3_1 ((uint32_t)0x00000002) ã)FSMC_PATT3_ATTSET3_2 ((uint32_t)0x00000004) å)FSMC_PATT3_ATTSET3_3 ((uint32_t)0x00000008) ç)FSMC_PATT3_ATTSET3_4 ((uint32_t)0x00000010) é)FSMC_PATT3_ATTSET3_5 ((uint32_t)0x00000020) è)FSMC_PATT3_ATTSET3_6 ((uint32_t)0x00000040) ê)FSMC_PATT3_ATTSET3_7 ((uint32_t)0x00000080) í)FSMC_PATT3_ATTWAIT3 ((uint32_t)0x0000FF00) ì)FSMC_PATT3_ATTWAIT3_0 ((uint32_t)0x00000100) î)FSMC_PATT3_ATTWAIT3_1 ((uint32_t)0x00000200) ï)FSMC_PATT3_ATTWAIT3_2 ((uint32_t)0x00000400) ñ)FSMC_PATT3_ATTWAIT3_3 ((uint32_t)0x00000800) ó)FSMC_PATT3_ATTWAIT3_4 ((uint32_t)0x00001000) ò)FSMC_PATT3_ATTWAIT3_5 ((uint32_t)0x00002000) ô)FSMC_PATT3_ATTWAIT3_6 ((uint32_t)0x00004000) ö)FSMC_PATT3_ATTWAIT3_7 ((uint32_t)0x00008000) ú)FSMC_PATT3_ATTHOLD3 ((uint32_t)0x00FF0000) ù)FSMC_PATT3_ATTHOLD3_0 ((uint32_t)0x00010000) û)FSMC_PATT3_ATTHOLD3_1 ((uint32_t)0x00020000) ü)FSMC_PATT3_ATTHOLD3_2 ((uint32_t)0x00040000) †)FSMC_PATT3_ATTHOLD3_3 ((uint32_t)0x00080000) °)FSMC_PATT3_ATTHOLD3_4 ((uint32_t)0x00100000) ¢)FSMC_PATT3_ATTHOLD3_5 ((uint32_t)0x00200000) £)FSMC_PATT3_ATTHOLD3_6 ((uint32_t)0x00400000) §)FSMC_PATT3_ATTHOLD3_7 ((uint32_t)0x00800000) ¶)FSMC_PATT3_ATTHIZ3 ((uint32_t)0xFF000000) ß)FSMC_PATT3_ATTHIZ3_0 ((uint32_t)0x01000000) ®)FSMC_PATT3_ATTHIZ3_1 ((uint32_t)0x02000000) ©)FSMC_PATT3_ATTHIZ3_2 ((uint32_t)0x04000000) ™)FSMC_PATT3_ATTHIZ3_3 ((uint32_t)0x08000000) ´)FSMC_PATT3_ATTHIZ3_4 ((uint32_t)0x10000000) ¨)FSMC_PATT3_ATTHIZ3_5 ((uint32_t)0x20000000) ≠)FSMC_PATT3_ATTHIZ3_6 ((uint32_t)0x40000000) Æ)FSMC_PATT3_ATTHIZ3_7 ((uint32_t)0x80000000) ±)FSMC_PATT4_ATTSET4 ((uint32_t)0x000000FF) ≤)FSMC_PATT4_ATTSET4_0 ((uint32_t)0x00000001) ≥)FSMC_PATT4_ATTSET4_1 ((uint32_t)0x00000002) ¥)FSMC_PATT4_ATTSET4_2 ((uint32_t)0x00000004) µ)FSMC_PATT4_ATTSET4_3 ((uint32_t)0x00000008) ∂)FSMC_PATT4_ATTSET4_4 ((uint32_t)0x00000010) ∑)FSMC_PATT4_ATTSET4_5 ((uint32_t)0x00000020) ∏)FSMC_PATT4_ATTSET4_6 ((uint32_t)0x00000040) π)FSMC_PATT4_ATTSET4_7 ((uint32_t)0x00000080) ª)FSMC_PATT4_ATTWAIT4 ((uint32_t)0x0000FF00) º)FSMC_PATT4_ATTWAIT4_0 ((uint32_t)0x00000100) Ω)FSMC_PATT4_ATTWAIT4_1 ((uint32_t)0x00000200) æ)FSMC_PATT4_ATTWAIT4_2 ((uint32_t)0x00000400) ø)FSMC_PATT4_ATTWAIT4_3 ((uint32_t)0x00000800) ¿)FSMC_PATT4_ATTWAIT4_4 ((uint32_t)0x00001000) ¡)FSMC_PATT4_ATTWAIT4_5 ((uint32_t)0x00002000) ¬)FSMC_PATT4_ATTWAIT4_6 ((uint32_t)0x00004000) √)FSMC_PATT4_ATTWAIT4_7 ((uint32_t)0x00008000) ≈)FSMC_PATT4_ATTHOLD4 ((uint32_t)0x00FF0000) ∆)FSMC_PATT4_ATTHOLD4_0 ((uint32_t)0x00010000) «)FSMC_PATT4_ATTHOLD4_1 ((uint32_t)0x00020000) »)FSMC_PATT4_ATTHOLD4_2 ((uint32_t)0x00040000) …)FSMC_PATT4_ATTHOLD4_3 ((uint32_t)0x00080000)  )FSMC_PATT4_ATTHOLD4_4 ((uint32_t)0x00100000) À)FSMC_PATT4_ATTHOLD4_5 ((uint32_t)0x00200000) Ã)FSMC_PATT4_ATTHOLD4_6 ((uint32_t)0x00400000) Õ)FSMC_PATT4_ATTHOLD4_7 ((uint32_t)0x00800000) œ)FSMC_PATT4_ATTHIZ4 ((uint32_t)0xFF000000) –)FSMC_PATT4_ATTHIZ4_0 ((uint32_t)0x01000000) —)FSMC_PATT4_ATTHIZ4_1 ((uint32_t)0x02000000) “)FSMC_PATT4_ATTHIZ4_2 ((uint32_t)0x04000000) ”)FSMC_PATT4_ATTHIZ4_3 ((uint32_t)0x08000000) ‘)FSMC_PATT4_ATTHIZ4_4 ((uint32_t)0x10000000) ’)FSMC_PATT4_ATTHIZ4_5 ((uint32_t)0x20000000) ÷)FSMC_PATT4_ATTHIZ4_6 ((uint32_t)0x40000000) ◊)FSMC_PATT4_ATTHIZ4_7 ((uint32_t)0x80000000) ⁄)FSMC_PIO4_IOSET4 ((uint32_t)0x000000FF) €)FSMC_PIO4_IOSET4_0 ((uint32_t)0x00000001) ‹)FSMC_PIO4_IOSET4_1 ((uint32_t)0x00000002) ›)FSMC_PIO4_IOSET4_2 ((uint32_t)0x00000004) ﬁ)FSMC_PIO4_IOSET4_3 ((uint32_t)0x00000008) ﬂ)FSMC_PIO4_IOSET4_4 ((uint32_t)0x00000010) ‡)FSMC_PIO4_IOSET4_5 ((uint32_t)0x00000020) ·)FSMC_PIO4_IOSET4_6 ((uint32_t)0x00000040) ‚)FSMC_PIO4_IOSET4_7 ((uint32_t)0x00000080) ‰)FSMC_PIO4_IOWAIT4 ((uint32_t)0x0000FF00) Â)FSMC_PIO4_IOWAIT4_0 ((uint32_t)0x00000100) Ê)FSMC_PIO4_IOWAIT4_1 ((uint32_t)0x00000200) Á)FSMC_PIO4_IOWAIT4_2 ((uint32_t)0x00000400) Ë)FSMC_PIO4_IOWAIT4_3 ((uint32_t)0x00000800) È)FSMC_PIO4_IOWAIT4_4 ((uint32_t)0x00001000) Í)FSMC_PIO4_IOWAIT4_5 ((uint32_t)0x00002000) Î)FSMC_PIO4_IOWAIT4_6 ((uint32_t)0x00004000) Ï)FSMC_PIO4_IOWAIT4_7 ((uint32_t)0x00008000) Ó)FSMC_PIO4_IOHOLD4 ((uint32_t)0x00FF0000) Ô)FSMC_PIO4_IOHOLD4_0 ((uint32_t)0x00010000) )FSMC_PIO4_IOHOLD4_1 ((uint32_t)0x00020000) Ò)FSMC_PIO4_IOHOLD4_2 ((uint32_t)0x00040000) Ú)FSMC_PIO4_IOHOLD4_3 ((uint32_t)0x00080000) Û)FSMC_PIO4_IOHOLD4_4 ((uint32_t)0x00100000) Ù)FSMC_PIO4_IOHOLD4_5 ((uint32_t)0x00200000) ı)FSMC_PIO4_IOHOLD4_6 ((uint32_t)0x00400000) ˆ)FSMC_PIO4_IOHOLD4_7 ((uint32_t)0x00800000) ¯)FSMC_PIO4_IOHIZ4 ((uint32_t)0xFF000000) ˘)FSMC_PIO4_IOHIZ4_0 ((uint32_t)0x01000000) ˙)FSMC_PIO4_IOHIZ4_1 ((uint32_t)0x02000000) ˚)FSMC_PIO4_IOHIZ4_2 ((uint32_t)0x04000000) ¸)FSMC_PIO4_IOHIZ4_3 ((uint32_t)0x08000000) ˝)FSMC_PIO4_IOHIZ4_4 ((uint32_t)0x10000000) ˛)FSMC_PIO4_IOHIZ4_5 ((uint32_t)0x20000000) ˇ)FSMC_PIO4_IOHIZ4_6 ((uint32_t)0x40000000) Ä*FSMC_PIO4_IOHIZ4_7 ((uint32_t)0x80000000) É*FSMC_ECCR2_ECC2 ((uint32_t)0xFFFFFFFF) Ü*FSMC_ECCR3_ECC3 ((uint32_t)0xFFFFFFFF) è*SDIO_POWER_PWRCTRL ((uint8_t)0x03) ê*SDIO_POWER_PWRCTRL_0 ((uint8_t)0x01) ë*SDIO_POWER_PWRCTRL_1 ((uint8_t)0x02) î*SDIO_CLKCR_CLKDIV ((uint16_t)0x00FF) ï*SDIO_CLKCR_CLKEN ((uint16_t)0x0100) ñ*SDIO_CLKCR_PWRSAV ((uint16_t)0x0200) ó*SDIO_CLKCR_BYPASS ((uint16_t)0x0400) ô*SDIO_CLKCR_WIDBUS ((uint16_t)0x1800) ö*SDIO_CLKCR_WIDBUS_0 ((uint16_t)0x0800) õ*SDIO_CLKCR_WIDBUS_1 ((uint16_t)0x1000) ù*SDIO_CLKCR_NEGEDGE ((uint16_t)0x2000) û*SDIO_CLKCR_HWFC_EN ((uint16_t)0x4000) °*SDIO_ARG_CMDARG ((uint32_t)0xFFFFFFFF) §*SDIO_CMD_CMDINDEX ((uint16_t)0x003F) ¶*SDIO_CMD_WAITRESP ((uint16_t)0x00C0) ß*SDIO_CMD_WAITRESP_0 ((uint16_t)0x0040) ®*SDIO_CMD_WAITRESP_1 ((uint16_t)0x0080) ™*SDIO_CMD_WAITINT ((uint16_t)0x0100) ´*SDIO_CMD_WAITPEND ((uint16_t)0x0200) ¨*SDIO_CMD_CPSMEN ((uint16_t)0x0400) ≠*SDIO_CMD_SDIOSUSPEND ((uint16_t)0x0800) Æ*SDIO_CMD_ENCMDCOMPL ((uint16_t)0x1000) Ø*SDIO_CMD_NIEN ((uint16_t)0x2000) ∞*SDIO_CMD_CEATACMD ((uint16_t)0x4000) ≥*SDIO_RESPCMD_RESPCMD ((uint8_t)0x3F) ∂*SDIO_RESP0_CARDSTATUS0 ((uint32_t)0xFFFFFFFF) π*SDIO_RESP1_CARDSTATUS1 ((uint32_t)0xFFFFFFFF) º*SDIO_RESP2_CARDSTATUS2 ((uint32_t)0xFFFFFFFF) ø*SDIO_RESP3_CARDSTATUS3 ((uint32_t)0xFFFFFFFF) ¬*SDIO_RESP4_CARDSTATUS4 ((uint32_t)0xFFFFFFFF) ≈*SDIO_DTIMER_DATATIME ((uint32_t)0xFFFFFFFF) »*SDIO_DLEN_DATALENGTH ((uint32_t)0x01FFFFFF) À*SDIO_DCTRL_DTEN ((uint16_t)0x0001) Ã*SDIO_DCTRL_DTDIR ((uint16_t)0x0002) Õ*SDIO_DCTRL_DTMODE ((uint16_t)0x0004) Œ*SDIO_DCTRL_DMAEN ((uint16_t)0x0008) –*SDIO_DCTRL_DBLOCKSIZE ((uint16_t)0x00F0) —*SDIO_DCTRL_DBLOCKSIZE_0 ((uint16_t)0x0010) “*SDIO_DCTRL_DBLOCKSIZE_1 ((uint16_t)0x0020) ”*SDIO_DCTRL_DBLOCKSIZE_2 ((uint16_t)0x0040) ‘*SDIO_DCTRL_DBLOCKSIZE_3 ((uint16_t)0x0080) ÷*SDIO_DCTRL_RWSTART ((uint16_t)0x0100) ◊*SDIO_DCTRL_RWSTOP ((uint16_t)0x0200) ÿ*SDIO_DCTRL_RWMOD ((uint16_t)0x0400) Ÿ*SDIO_DCTRL_SDIOEN ((uint16_t)0x0800) ‹*SDIO_DCOUNT_DATACOUNT ((uint32_t)0x01FFFFFF) ﬂ*SDIO_STA_CCRCFAIL ((uint32_t)0x00000001) ‡*SDIO_STA_DCRCFAIL ((uint32_t)0x00000002) ·*SDIO_STA_CTIMEOUT ((uint32_t)0x00000004) ‚*SDIO_STA_DTIMEOUT ((uint32_t)0x00000008) „*SDIO_STA_TXUNDERR ((uint32_t)0x00000010) ‰*SDIO_STA_RXOVERR ((uint32_t)0x00000020) Â*SDIO_STA_CMDREND ((uint32_t)0x00000040) Ê*SDIO_STA_CMDSENT ((uint32_t)0x00000080) Á*SDIO_STA_DATAEND ((uint32_t)0x00000100) Ë*SDIO_STA_STBITERR ((uint32_t)0x00000200) È*SDIO_STA_DBCKEND ((uint32_t)0x00000400) Í*SDIO_STA_CMDACT ((uint32_t)0x00000800) Î*SDIO_STA_TXACT ((uint32_t)0x00001000) Ï*SDIO_STA_RXACT ((uint32_t)0x00002000) Ì*SDIO_STA_TXFIFOHE ((uint32_t)0x00004000) Ó*SDIO_STA_RXFIFOHF ((uint32_t)0x00008000) Ô*SDIO_STA_TXFIFOF ((uint32_t)0x00010000) *SDIO_STA_RXFIFOF ((uint32_t)0x00020000) Ò*SDIO_STA_TXFIFOE ((uint32_t)0x00040000) Ú*SDIO_STA_RXFIFOE ((uint32_t)0x00080000) Û*SDIO_STA_TXDAVL ((uint32_t)0x00100000) Ù*SDIO_STA_RXDAVL ((uint32_t)0x00200000) ı*SDIO_STA_SDIOIT ((uint32_t)0x00400000) ˆ*SDIO_STA_CEATAEND ((uint32_t)0x00800000) ˘*SDIO_ICR_CCRCFAILC ((uint32_t)0x00000001) ˙*SDIO_ICR_DCRCFAILC ((uint32_t)0x00000002) ˚*SDIO_ICR_CTIMEOUTC ((uint32_t)0x00000004) ¸*SDIO_ICR_DTIMEOUTC ((uint32_t)0x00000008) ˝*SDIO_ICR_TXUNDERRC ((uint32_t)0x00000010) ˛*SDIO_ICR_RXOVERRC ((uint32_t)0x00000020) ˇ*SDIO_ICR_CMDRENDC ((uint32_t)0x00000040) Ä+SDIO_ICR_CMDSENTC ((uint32_t)0x00000080) Å+SDIO_ICR_DATAENDC ((uint32_t)0x00000100) Ç+SDIO_ICR_STBITERRC ((uint32_t)0x00000200) É+SDIO_ICR_DBCKENDC ((uint32_t)0x00000400) Ñ+SDIO_ICR_SDIOITC ((uint32_t)0x00400000) Ö+SDIO_ICR_CEATAENDC ((uint32_t)0x00800000) à+SDIO_MASK_CCRCFAILIE ((uint32_t)0x00000001) â+SDIO_MASK_DCRCFAILIE ((uint32_t)0x00000002) ä+SDIO_MASK_CTIMEOUTIE ((uint32_t)0x00000004) ã+SDIO_MASK_DTIMEOUTIE ((uint32_t)0x00000008) å+SDIO_MASK_TXUNDERRIE ((uint32_t)0x00000010) ç+SDIO_MASK_RXOVERRIE ((uint32_t)0x00000020) é+SDIO_MASK_CMDRENDIE ((uint32_t)0x00000040) è+SDIO_MASK_CMDSENTIE ((uint32_t)0x00000080) ê+SDIO_MASK_DATAENDIE ((uint32_t)0x00000100) ë+SDIO_MASK_STBITERRIE ((uint32_t)0x00000200) í+SDIO_MASK_DBCKENDIE ((uint32_t)0x00000400) ì+SDIO_MASK_CMDACTIE ((uint32_t)0x00000800) î+SDIO_MASK_TXACTIE ((uint32_t)0x00001000) ï+SDIO_MASK_RXACTIE ((uint32_t)0x00002000) ñ+SDIO_MASK_TXFIFOHEIE ((uint32_t)0x00004000) ó+SDIO_MASK_RXFIFOHFIE ((uint32_t)0x00008000) ò+SDIO_MASK_TXFIFOFIE ((uint32_t)0x00010000) ô+SDIO_MASK_RXFIFOFIE ((uint32_t)0x00020000) ö+SDIO_MASK_TXFIFOEIE ((uint32_t)0x00040000) õ+SDIO_MASK_RXFIFOEIE ((uint32_t)0x00080000) ú+SDIO_MASK_TXDAVLIE ((uint32_t)0x00100000) ù+SDIO_MASK_RXDAVLIE ((uint32_t)0x00200000) û+SDIO_MASK_SDIOITIE ((uint32_t)0x00400000) ü+SDIO_MASK_CEATAENDIE ((uint32_t)0x00800000) ¢+SDIO_FIFOCNT_FIFOCOUNT ((uint32_t)0x00FFFFFF) •+SDIO_FIFO_FIFODATA ((uint32_t)0xFFFFFFFF) Ø+USB_EP0R_EA ((uint16_t)0x000F) ±+USB_EP0R_STAT_TX ((uint16_t)0x0030) ≤+USB_EP0R_STAT_TX_0 ((uint16_t)0x0010) ≥+USB_EP0R_STAT_TX_1 ((uint16_t)0x0020) µ+USB_EP0R_DTOG_TX ((uint16_t)0x0040) ∂+USB_EP0R_CTR_TX ((uint16_t)0x0080) ∑+USB_EP0R_EP_KIND ((uint16_t)0x0100) π+USB_EP0R_EP_TYPE ((uint16_t)0x0600) ∫+USB_EP0R_EP_TYPE_0 ((uint16_t)0x0200) ª+USB_EP0R_EP_TYPE_1 ((uint16_t)0x0400) Ω+USB_EP0R_SETUP ((uint16_t)0x0800) ø+USB_EP0R_STAT_RX ((uint16_t)0x3000) ¿+USB_EP0R_STAT_RX_0 ((uint16_t)0x1000) ¡+USB_EP0R_STAT_RX_1 ((uint16_t)0x2000) √+USB_EP0R_DTOG_RX ((uint16_t)0x4000) ƒ+USB_EP0R_CTR_RX ((uint16_t)0x8000) «+USB_EP1R_EA ((uint16_t)0x000F) …+USB_EP1R_STAT_TX ((uint16_t)0x0030)  +USB_EP1R_STAT_TX_0 ((uint16_t)0x0010) À+USB_EP1R_STAT_TX_1 ((uint16_t)0x0020) Õ+USB_EP1R_DTOG_TX ((uint16_t)0x0040) Œ+USB_EP1R_CTR_TX ((uint16_t)0x0080) œ+USB_EP1R_EP_KIND ((uint16_t)0x0100) —+USB_EP1R_EP_TYPE ((uint16_t)0x0600) “+USB_EP1R_EP_TYPE_0 ((uint16_t)0x0200) ”+USB_EP1R_EP_TYPE_1 ((uint16_t)0x0400) ’+USB_EP1R_SETUP ((uint16_t)0x0800) ◊+USB_EP1R_STAT_RX ((uint16_t)0x3000) ÿ+USB_EP1R_STAT_RX_0 ((uint16_t)0x1000) Ÿ+USB_EP1R_STAT_RX_1 ((uint16_t)0x2000) €+USB_EP1R_DTOG_RX ((uint16_t)0x4000) ‹+USB_EP1R_CTR_RX ((uint16_t)0x8000) ﬂ+USB_EP2R_EA ((uint16_t)0x000F) ·+USB_EP2R_STAT_TX ((uint16_t)0x0030) ‚+USB_EP2R_STAT_TX_0 ((uint16_t)0x0010) „+USB_EP2R_STAT_TX_1 ((uint16_t)0x0020) Â+USB_EP2R_DTOG_TX ((uint16_t)0x0040) Ê+USB_EP2R_CTR_TX ((uint16_t)0x0080) Á+USB_EP2R_EP_KIND ((uint16_t)0x0100) È+USB_EP2R_EP_TYPE ((uint16_t)0x0600) Í+USB_EP2R_EP_TYPE_0 ((uint16_t)0x0200) Î+USB_EP2R_EP_TYPE_1 ((uint16_t)0x0400) Ì+USB_EP2R_SETUP ((uint16_t)0x0800) Ô+USB_EP2R_STAT_RX ((uint16_t)0x3000) +USB_EP2R_STAT_RX_0 ((uint16_t)0x1000) Ò+USB_EP2R_STAT_RX_1 ((uint16_t)0x2000) Û+USB_EP2R_DTOG_RX ((uint16_t)0x4000) Ù+USB_EP2R_CTR_RX ((uint16_t)0x8000) ˜+USB_EP3R_EA ((uint16_t)0x000F) ˘+USB_EP3R_STAT_TX ((uint16_t)0x0030) ˙+USB_EP3R_STAT_TX_0 ((uint16_t)0x0010) ˚+USB_EP3R_STAT_TX_1 ((uint16_t)0x0020) ˝+USB_EP3R_DTOG_TX ((uint16_t)0x0040) ˛+USB_EP3R_CTR_TX ((uint16_t)0x0080) ˇ+USB_EP3R_EP_KIND ((uint16_t)0x0100) Å,USB_EP3R_EP_TYPE ((uint16_t)0x0600) Ç,USB_EP3R_EP_TYPE_0 ((uint16_t)0x0200) É,USB_EP3R_EP_TYPE_1 ((uint16_t)0x0400) Ö,USB_EP3R_SETUP ((uint16_t)0x0800) á,USB_EP3R_STAT_RX ((uint16_t)0x3000) à,USB_EP3R_STAT_RX_0 ((uint16_t)0x1000) â,USB_EP3R_STAT_RX_1 ((uint16_t)0x2000) ã,USB_EP3R_DTOG_RX ((uint16_t)0x4000) å,USB_EP3R_CTR_RX ((uint16_t)0x8000) è,USB_EP4R_EA ((uint16_t)0x000F) ë,USB_EP4R_STAT_TX ((uint16_t)0x0030) í,USB_EP4R_STAT_TX_0 ((uint16_t)0x0010) ì,USB_EP4R_STAT_TX_1 ((uint16_t)0x0020) ï,USB_EP4R_DTOG_TX ((uint16_t)0x0040) ñ,USB_EP4R_CTR_TX ((uint16_t)0x0080) ó,USB_EP4R_EP_KIND ((uint16_t)0x0100) ô,USB_EP4R_EP_TYPE ((uint16_t)0x0600) ö,USB_EP4R_EP_TYPE_0 ((uint16_t)0x0200) õ,USB_EP4R_EP_TYPE_1 ((uint16_t)0x0400) ù,USB_EP4R_SETUP ((uint16_t)0x0800) ü,USB_EP4R_STAT_RX ((uint16_t)0x3000) †,USB_EP4R_STAT_RX_0 ((uint16_t)0x1000) °,USB_EP4R_STAT_RX_1 ((uint16_t)0x2000) £,USB_EP4R_DTOG_RX ((uint16_t)0x4000) §,USB_EP4R_CTR_RX ((uint16_t)0x8000) ß,USB_EP5R_EA ((uint16_t)0x000F) ©,USB_EP5R_STAT_TX ((uint16_t)0x0030) ™,USB_EP5R_STAT_TX_0 ((uint16_t)0x0010) ´,USB_EP5R_STAT_TX_1 ((uint16_t)0x0020) ≠,USB_EP5R_DTOG_TX ((uint16_t)0x0040) Æ,USB_EP5R_CTR_TX ((uint16_t)0x0080) Ø,USB_EP5R_EP_KIND ((uint16_t)0x0100) ±,USB_EP5R_EP_TYPE ((uint16_t)0x0600) ≤,USB_EP5R_EP_TYPE_0 ((uint16_t)0x0200) ≥,USB_EP5R_EP_TYPE_1 ((uint16_t)0x0400) µ,USB_EP5R_SETUP ((uint16_t)0x0800) ∑,USB_EP5R_STAT_RX ((uint16_t)0x3000) ∏,USB_EP5R_STAT_RX_0 ((uint16_t)0x1000) π,USB_EP5R_STAT_RX_1 ((uint16_t)0x2000) ª,USB_EP5R_DTOG_RX ((uint16_t)0x4000) º,USB_EP5R_CTR_RX ((uint16_t)0x8000) ø,USB_EP6R_EA ((uint16_t)0x000F) ¡,USB_EP6R_STAT_TX ((uint16_t)0x0030) ¬,USB_EP6R_STAT_TX_0 ((uint16_t)0x0010) √,USB_EP6R_STAT_TX_1 ((uint16_t)0x0020) ≈,USB_EP6R_DTOG_TX ((uint16_t)0x0040) ∆,USB_EP6R_CTR_TX ((uint16_t)0x0080) «,USB_EP6R_EP_KIND ((uint16_t)0x0100) …,USB_EP6R_EP_TYPE ((uint16_t)0x0600)  ,USB_EP6R_EP_TYPE_0 ((uint16_t)0x0200) À,USB_EP6R_EP_TYPE_1 ((uint16_t)0x0400) Õ,USB_EP6R_SETUP ((uint16_t)0x0800) œ,USB_EP6R_STAT_RX ((uint16_t)0x3000) –,USB_EP6R_STAT_RX_0 ((uint16_t)0x1000) —,USB_EP6R_STAT_RX_1 ((uint16_t)0x2000) ”,USB_EP6R_DTOG_RX ((uint16_t)0x4000) ‘,USB_EP6R_CTR_RX ((uint16_t)0x8000) ◊,USB_EP7R_EA ((uint16_t)0x000F) Ÿ,USB_EP7R_STAT_TX ((uint16_t)0x0030) ⁄,USB_EP7R_STAT_TX_0 ((uint16_t)0x0010) €,USB_EP7R_STAT_TX_1 ((uint16_t)0x0020) ›,USB_EP7R_DTOG_TX ((uint16_t)0x0040) ﬁ,USB_EP7R_CTR_TX ((uint16_t)0x0080) ﬂ,USB_EP7R_EP_KIND ((uint16_t)0x0100) ·,USB_EP7R_EP_TYPE ((uint16_t)0x0600) ‚,USB_EP7R_EP_TYPE_0 ((uint16_t)0x0200) „,USB_EP7R_EP_TYPE_1 ((uint16_t)0x0400) Â,USB_EP7R_SETUP ((uint16_t)0x0800) Á,USB_EP7R_STAT_RX ((uint16_t)0x3000) Ë,USB_EP7R_STAT_RX_0 ((uint16_t)0x1000) È,USB_EP7R_STAT_RX_1 ((uint16_t)0x2000) Î,USB_EP7R_DTOG_RX ((uint16_t)0x4000) Ï,USB_EP7R_CTR_RX ((uint16_t)0x8000) ,USB_CNTR_FRES ((uint16_t)0x0001) Ò,USB_CNTR_PDWN ((uint16_t)0x0002) Ú,USB_CNTR_LP_MODE ((uint16_t)0x0004) Û,USB_CNTR_FSUSP ((uint16_t)0x0008) Ù,USB_CNTR_RESUME ((uint16_t)0x0010) ı,USB_CNTR_ESOFM ((uint16_t)0x0100) ˆ,USB_CNTR_SOFM ((uint16_t)0x0200) ˜,USB_CNTR_RESETM ((uint16_t)0x0400) ¯,USB_CNTR_SUSPM ((uint16_t)0x0800) ˘,USB_CNTR_WKUPM ((uint16_t)0x1000) ˙,USB_CNTR_ERRM ((uint16_t)0x2000) ˚,USB_CNTR_PMAOVRM ((uint16_t)0x4000) ¸,USB_CNTR_CTRM ((uint16_t)0x8000) ˇ,USB_ISTR_EP_ID ((uint16_t)0x000F) Ä-USB_ISTR_DIR ((uint16_t)0x0010) Å-USB_ISTR_ESOF ((uint16_t)0x0100) Ç-USB_ISTR_SOF ((uint16_t)0x0200) É-USB_ISTR_RESET ((uint16_t)0x0400) Ñ-USB_ISTR_SUSP ((uint16_t)0x0800) Ö-USB_ISTR_WKUP ((uint16_t)0x1000) Ü-USB_ISTR_ERR ((uint16_t)0x2000) á-USB_ISTR_PMAOVR ((uint16_t)0x4000) à-USB_ISTR_CTR ((uint16_t)0x8000) ã-USB_FNR_FN ((uint16_t)0x07FF) å-USB_FNR_LSOF ((uint16_t)0x1800) ç-USB_FNR_LCK ((uint16_t)0x2000) é-USB_FNR_RXDM ((uint16_t)0x4000) è-USB_FNR_RXDP ((uint16_t)0x8000) í-USB_DADDR_ADD ((uint8_t)0x7F) ì-USB_DADDR_ADD0 ((uint8_t)0x01) î-USB_DADDR_ADD1 ((uint8_t)0x02) ï-USB_DADDR_ADD2 ((uint8_t)0x04) ñ-USB_DADDR_ADD3 ((uint8_t)0x08) ó-USB_DADDR_ADD4 ((uint8_t)0x10) ò-USB_DADDR_ADD5 ((uint8_t)0x20) ô-USB_DADDR_ADD6 ((uint8_t)0x40) õ-USB_DADDR_EF ((uint8_t)0x80) û-USB_BTABLE_BTABLE ((uint16_t)0xFFF8) ¢-USB_ADDR0_TX_ADDR0_TX ((uint16_t)0xFFFE) •-USB_ADDR1_TX_ADDR1_TX ((uint16_t)0xFFFE) ®-USB_ADDR2_TX_ADDR2_TX ((uint16_t)0xFFFE) ´-USB_ADDR3_TX_ADDR3_TX ((uint16_t)0xFFFE) Æ-USB_ADDR4_TX_ADDR4_TX ((uint16_t)0xFFFE) ±-USB_ADDR5_TX_ADDR5_TX ((uint16_t)0xFFFE) ¥-USB_ADDR6_TX_ADDR6_TX ((uint16_t)0xFFFE) ∑-USB_ADDR7_TX_ADDR7_TX ((uint16_t)0xFFFE) º-USB_COUNT0_TX_COUNT0_TX ((uint16_t)0x03FF) ø-USB_COUNT1_TX_COUNT1_TX ((uint16_t)0x03FF) ¬-USB_COUNT2_TX_COUNT2_TX ((uint16_t)0x03FF) ≈-USB_COUNT3_TX_COUNT3_TX ((uint16_t)0x03FF) »-USB_COUNT4_TX_COUNT4_TX ((uint16_t)0x03FF) À-USB_COUNT5_TX_COUNT5_TX ((uint16_t)0x03FF) Œ-USB_COUNT6_TX_COUNT6_TX ((uint16_t)0x03FF) —-USB_COUNT7_TX_COUNT7_TX ((uint16_t)0x03FF) ÷-USB_COUNT0_TX_0_COUNT0_TX_0 ((uint32_t)0x000003FF) Ÿ-USB_COUNT0_TX_1_COUNT0_TX_1 ((uint32_t)0x03FF0000) ‹-USB_COUNT1_TX_0_COUNT1_TX_0 ((uint32_t)0x000003FF) ﬂ-USB_COUNT1_TX_1_COUNT1_TX_1 ((uint32_t)0x03FF0000) ‚-USB_COUNT2_TX_0_COUNT2_TX_0 ((uint32_t)0x000003FF) Â-USB_COUNT2_TX_1_COUNT2_TX_1 ((uint32_t)0x03FF0000) Ë-USB_COUNT3_TX_0_COUNT3_TX_0 ((uint16_t)0x000003FF) Î-USB_COUNT3_TX_1_COUNT3_TX_1 ((uint16_t)0x03FF0000) Ó-USB_COUNT4_TX_0_COUNT4_TX_0 ((uint32_t)0x000003FF) Ò-USB_COUNT4_TX_1_COUNT4_TX_1 ((uint32_t)0x03FF0000) Ù-USB_COUNT5_TX_0_COUNT5_TX_0 ((uint32_t)0x000003FF) ˜-USB_COUNT5_TX_1_COUNT5_TX_1 ((uint32_t)0x03FF0000) ˙-USB_COUNT6_TX_0_COUNT6_TX_0 ((uint32_t)0x000003FF) ˝-USB_COUNT6_TX_1_COUNT6_TX_1 ((uint32_t)0x03FF0000) Ä.USB_COUNT7_TX_0_COUNT7_TX_0 ((uint32_t)0x000003FF) É.USB_COUNT7_TX_1_COUNT7_TX_1 ((uint32_t)0x03FF0000) à.USB_ADDR0_RX_ADDR0_RX ((uint16_t)0xFFFE) ã.USB_ADDR1_RX_ADDR1_RX ((uint16_t)0xFFFE) é.USB_ADDR2_RX_ADDR2_RX ((uint16_t)0xFFFE) ë.USB_ADDR3_RX_ADDR3_RX ((uint16_t)0xFFFE) î.USB_ADDR4_RX_ADDR4_RX ((uint16_t)0xFFFE) ó.USB_ADDR5_RX_ADDR5_RX ((uint16_t)0xFFFE) ö.USB_ADDR6_RX_ADDR6_RX ((uint16_t)0xFFFE) ù.USB_ADDR7_RX_ADDR7_RX ((uint16_t)0xFFFE) ¢.USB_COUNT0_RX_COUNT0_RX ((uint16_t)0x03FF) §.USB_COUNT0_RX_NUM_BLOCK ((uint16_t)0x7C00) •.USB_COUNT0_RX_NUM_BLOCK_0 ((uint16_t)0x0400) ¶.USB_COUNT0_RX_NUM_BLOCK_1 ((uint16_t)0x0800) ß.USB_COUNT0_RX_NUM_BLOCK_2 ((uint16_t)0x1000) ®.USB_COUNT0_RX_NUM_BLOCK_3 ((uint16_t)0x2000) ©.USB_COUNT0_RX_NUM_BLOCK_4 ((uint16_t)0x4000) ´.USB_COUNT0_RX_BLSIZE ((uint16_t)0x8000) Æ.USB_COUNT1_RX_COUNT1_RX ((uint16_t)0x03FF) ∞.USB_COUNT1_RX_NUM_BLOCK ((uint16_t)0x7C00) ±.USB_COUNT1_RX_NUM_BLOCK_0 ((uint16_t)0x0400) ≤.USB_COUNT1_RX_NUM_BLOCK_1 ((uint16_t)0x0800) ≥.USB_COUNT1_RX_NUM_BLOCK_2 ((uint16_t)0x1000) ¥.USB_COUNT1_RX_NUM_BLOCK_3 ((uint16_t)0x2000) µ.USB_COUNT1_RX_NUM_BLOCK_4 ((uint16_t)0x4000) ∑.USB_COUNT1_RX_BLSIZE ((uint16_t)0x8000) ∫.USB_COUNT2_RX_COUNT2_RX ((uint16_t)0x03FF) º.USB_COUNT2_RX_NUM_BLOCK ((uint16_t)0x7C00) Ω.USB_COUNT2_RX_NUM_BLOCK_0 ((uint16_t)0x0400) æ.USB_COUNT2_RX_NUM_BLOCK_1 ((uint16_t)0x0800) ø.USB_COUNT2_RX_NUM_BLOCK_2 ((uint16_t)0x1000) ¿.USB_COUNT2_RX_NUM_BLOCK_3 ((uint16_t)0x2000) ¡.USB_COUNT2_RX_NUM_BLOCK_4 ((uint16_t)0x4000) √.USB_COUNT2_RX_BLSIZE ((uint16_t)0x8000) ∆.USB_COUNT3_RX_COUNT3_RX ((uint16_t)0x03FF) ».USB_COUNT3_RX_NUM_BLOCK ((uint16_t)0x7C00) ….USB_COUNT3_RX_NUM_BLOCK_0 ((uint16_t)0x0400)  .USB_COUNT3_RX_NUM_BLOCK_1 ((uint16_t)0x0800) À.USB_COUNT3_RX_NUM_BLOCK_2 ((uint16_t)0x1000) Ã.USB_COUNT3_RX_NUM_BLOCK_3 ((uint16_t)0x2000) Õ.USB_COUNT3_RX_NUM_BLOCK_4 ((uint16_t)0x4000) œ.USB_COUNT3_RX_BLSIZE ((uint16_t)0x8000) “.USB_COUNT4_RX_COUNT4_RX ((uint16_t)0x03FF) ‘.USB_COUNT4_RX_NUM_BLOCK ((uint16_t)0x7C00) ’.USB_COUNT4_RX_NUM_BLOCK_0 ((uint16_t)0x0400) ÷.USB_COUNT4_RX_NUM_BLOCK_1 ((uint16_t)0x0800) ◊.USB_COUNT4_RX_NUM_BLOCK_2 ((uint16_t)0x1000) ÿ.USB_COUNT4_RX_NUM_BLOCK_3 ((uint16_t)0x2000) Ÿ.USB_COUNT4_RX_NUM_BLOCK_4 ((uint16_t)0x4000) €.USB_COUNT4_RX_BLSIZE ((uint16_t)0x8000) ﬁ.USB_COUNT5_RX_COUNT5_RX ((uint16_t)0x03FF) ‡.USB_COUNT5_RX_NUM_BLOCK ((uint16_t)0x7C00) ·.USB_COUNT5_RX_NUM_BLOCK_0 ((uint16_t)0x0400) ‚.USB_COUNT5_RX_NUM_BLOCK_1 ((uint16_t)0x0800) „.USB_COUNT5_RX_NUM_BLOCK_2 ((uint16_t)0x1000) ‰.USB_COUNT5_RX_NUM_BLOCK_3 ((uint16_t)0x2000) Â.USB_COUNT5_RX_NUM_BLOCK_4 ((uint16_t)0x4000) Á.USB_COUNT5_RX_BLSIZE ((uint16_t)0x8000) Í.USB_COUNT6_RX_COUNT6_RX ((uint16_t)0x03FF) Ï.USB_COUNT6_RX_NUM_BLOCK ((uint16_t)0x7C00) Ì.USB_COUNT6_RX_NUM_BLOCK_0 ((uint16_t)0x0400) Ó.USB_COUNT6_RX_NUM_BLOCK_1 ((uint16_t)0x0800) Ô.USB_COUNT6_RX_NUM_BLOCK_2 ((uint16_t)0x1000) .USB_COUNT6_RX_NUM_BLOCK_3 ((uint16_t)0x2000) Ò.USB_COUNT6_RX_NUM_BLOCK_4 ((uint16_t)0x4000) Û.USB_COUNT6_RX_BLSIZE ((uint16_t)0x8000) ˆ.USB_COUNT7_RX_COUNT7_RX ((uint16_t)0x03FF) ¯.USB_COUNT7_RX_NUM_BLOCK ((uint16_t)0x7C00) ˘.USB_COUNT7_RX_NUM_BLOCK_0 ((uint16_t)0x0400) ˙.USB_COUNT7_RX_NUM_BLOCK_1 ((uint16_t)0x0800) ˚.USB_COUNT7_RX_NUM_BLOCK_2 ((uint16_t)0x1000) ¸.USB_COUNT7_RX_NUM_BLOCK_3 ((uint16_t)0x2000) ˝.USB_COUNT7_RX_NUM_BLOCK_4 ((uint16_t)0x4000) ˇ.USB_COUNT7_RX_BLSIZE ((uint16_t)0x8000) Ñ/USB_COUNT0_RX_0_COUNT0_RX_0 ((uint32_t)0x000003FF) Ü/USB_COUNT0_RX_0_NUM_BLOCK_0 ((uint32_t)0x00007C00) á/USB_COUNT0_RX_0_NUM_BLOCK_0_0 ((uint32_t)0x00000400) à/USB_COUNT0_RX_0_NUM_BLOCK_0_1 ((uint32_t)0x00000800) â/USB_COUNT0_RX_0_NUM_BLOCK_0_2 ((uint32_t)0x00001000) ä/USB_COUNT0_RX_0_NUM_BLOCK_0_3 ((uint32_t)0x00002000) ã/USB_COUNT0_RX_0_NUM_BLOCK_0_4 ((uint32_t)0x00004000) ç/USB_COUNT0_RX_0_BLSIZE_0 ((uint32_t)0x00008000) ê/USB_COUNT0_RX_1_COUNT0_RX_1 ((uint32_t)0x03FF0000) í/USB_COUNT0_RX_1_NUM_BLOCK_1 ((uint32_t)0x7C000000) ì/USB_COUNT0_RX_1_NUM_BLOCK_1_0 ((uint32_t)0x04000000) î/USB_COUNT0_RX_1_NUM_BLOCK_1_1 ((uint32_t)0x08000000) ï/USB_COUNT0_RX_1_NUM_BLOCK_1_2 ((uint32_t)0x10000000) ñ/USB_COUNT0_RX_1_NUM_BLOCK_1_3 ((uint32_t)0x20000000) ó/USB_COUNT0_RX_1_NUM_BLOCK_1_4 ((uint32_t)0x40000000) ô/USB_COUNT0_RX_1_BLSIZE_1 ((uint32_t)0x80000000) ú/USB_COUNT1_RX_0_COUNT1_RX_0 ((uint32_t)0x000003FF) û/USB_COUNT1_RX_0_NUM_BLOCK_0 ((uint32_t)0x00007C00) ü/USB_COUNT1_RX_0_NUM_BLOCK_0_0 ((uint32_t)0x00000400) †/USB_COUNT1_RX_0_NUM_BLOCK_0_1 ((uint32_t)0x00000800) °/USB_COUNT1_RX_0_NUM_BLOCK_0_2 ((uint32_t)0x00001000) ¢/USB_COUNT1_RX_0_NUM_BLOCK_0_3 ((uint32_t)0x00002000) £/USB_COUNT1_RX_0_NUM_BLOCK_0_4 ((uint32_t)0x00004000) •/USB_COUNT1_RX_0_BLSIZE_0 ((uint32_t)0x00008000) ®/USB_COUNT1_RX_1_COUNT1_RX_1 ((uint32_t)0x03FF0000) ™/USB_COUNT1_RX_1_NUM_BLOCK_1 ((uint32_t)0x7C000000) ´/USB_COUNT1_RX_1_NUM_BLOCK_1_0 ((uint32_t)0x04000000) ¨/USB_COUNT1_RX_1_NUM_BLOCK_1_1 ((uint32_t)0x08000000) ≠/USB_COUNT1_RX_1_NUM_BLOCK_1_2 ((uint32_t)0x10000000) Æ/USB_COUNT1_RX_1_NUM_BLOCK_1_3 ((uint32_t)0x20000000) Ø/USB_COUNT1_RX_1_NUM_BLOCK_1_4 ((uint32_t)0x40000000) ±/USB_COUNT1_RX_1_BLSIZE_1 ((uint32_t)0x80000000) ¥/USB_COUNT2_RX_0_COUNT2_RX_0 ((uint32_t)0x000003FF) ∂/USB_COUNT2_RX_0_NUM_BLOCK_0 ((uint32_t)0x00007C00) ∑/USB_COUNT2_RX_0_NUM_BLOCK_0_0 ((uint32_t)0x00000400) ∏/USB_COUNT2_RX_0_NUM_BLOCK_0_1 ((uint32_t)0x00000800) π/USB_COUNT2_RX_0_NUM_BLOCK_0_2 ((uint32_t)0x00001000) ∫/USB_COUNT2_RX_0_NUM_BLOCK_0_3 ((uint32_t)0x00002000) ª/USB_COUNT2_RX_0_NUM_BLOCK_0_4 ((uint32_t)0x00004000) Ω/USB_COUNT2_RX_0_BLSIZE_0 ((uint32_t)0x00008000) ¿/USB_COUNT2_RX_1_COUNT2_RX_1 ((uint32_t)0x03FF0000) ¬/USB_COUNT2_RX_1_NUM_BLOCK_1 ((uint32_t)0x7C000000) √/USB_COUNT2_RX_1_NUM_BLOCK_1_0 ((uint32_t)0x04000000) ƒ/USB_COUNT2_RX_1_NUM_BLOCK_1_1 ((uint32_t)0x08000000) ≈/USB_COUNT2_RX_1_NUM_BLOCK_1_2 ((uint32_t)0x10000000) ∆/USB_COUNT2_RX_1_NUM_BLOCK_1_3 ((uint32_t)0x20000000) «/USB_COUNT2_RX_1_NUM_BLOCK_1_4 ((uint32_t)0x40000000) …/USB_COUNT2_RX_1_BLSIZE_1 ((uint32_t)0x80000000) Ã/USB_COUNT3_RX_0_COUNT3_RX_0 ((uint32_t)0x000003FF) Œ/USB_COUNT3_RX_0_NUM_BLOCK_0 ((uint32_t)0x00007C00) œ/USB_COUNT3_RX_0_NUM_BLOCK_0_0 ((uint32_t)0x00000400) –/USB_COUNT3_RX_0_NUM_BLOCK_0_1 ((uint32_t)0x00000800) —/USB_COUNT3_RX_0_NUM_BLOCK_0_2 ((uint32_t)0x00001000) “/USB_COUNT3_RX_0_NUM_BLOCK_0_3 ((uint32_t)0x00002000) ”/USB_COUNT3_RX_0_NUM_BLOCK_0_4 ((uint32_t)0x00004000) ’/USB_COUNT3_RX_0_BLSIZE_0 ((uint32_t)0x00008000) ÿ/USB_COUNT3_RX_1_COUNT3_RX_1 ((uint32_t)0x03FF0000) ⁄/USB_COUNT3_RX_1_NUM_BLOCK_1 ((uint32_t)0x7C000000) €/USB_COUNT3_RX_1_NUM_BLOCK_1_0 ((uint32_t)0x04000000) ‹/USB_COUNT3_RX_1_NUM_BLOCK_1_1 ((uint32_t)0x08000000) ›/USB_COUNT3_RX_1_NUM_BLOCK_1_2 ((uint32_t)0x10000000) ﬁ/USB_COUNT3_RX_1_NUM_BLOCK_1_3 ((uint32_t)0x20000000) ﬂ/USB_COUNT3_RX_1_NUM_BLOCK_1_4 ((uint32_t)0x40000000) ·/USB_COUNT3_RX_1_BLSIZE_1 ((uint32_t)0x80000000) ‰/USB_COUNT4_RX_0_COUNT4_RX_0 ((uint32_t)0x000003FF) Ê/USB_COUNT4_RX_0_NUM_BLOCK_0 ((uint32_t)0x00007C00) Á/USB_COUNT4_RX_0_NUM_BLOCK_0_0 ((uint32_t)0x00000400) Ë/USB_COUNT4_RX_0_NUM_BLOCK_0_1 ((uint32_t)0x00000800) È/USB_COUNT4_RX_0_NUM_BLOCK_0_2 ((uint32_t)0x00001000) Í/USB_COUNT4_RX_0_NUM_BLOCK_0_3 ((uint32_t)0x00002000) Î/USB_COUNT4_RX_0_NUM_BLOCK_0_4 ((uint32_t)0x00004000) Ì/USB_COUNT4_RX_0_BLSIZE_0 ((uint32_t)0x00008000) /USB_COUNT4_RX_1_COUNT4_RX_1 ((uint32_t)0x03FF0000) Ú/USB_COUNT4_RX_1_NUM_BLOCK_1 ((uint32_t)0x7C000000) Û/USB_COUNT4_RX_1_NUM_BLOCK_1_0 ((uint32_t)0x04000000) Ù/USB_COUNT4_RX_1_NUM_BLOCK_1_1 ((uint32_t)0x08000000) ı/USB_COUNT4_RX_1_NUM_BLOCK_1_2 ((uint32_t)0x10000000) ˆ/USB_COUNT4_RX_1_NUM_BLOCK_1_3 ((uint32_t)0x20000000) ˜/USB_COUNT4_RX_1_NUM_BLOCK_1_4 ((uint32_t)0x40000000) ˘/USB_COUNT4_RX_1_BLSIZE_1 ((uint32_t)0x80000000) ¸/USB_COUNT5_RX_0_COUNT5_RX_0 ((uint32_t)0x000003FF) ˛/USB_COUNT5_RX_0_NUM_BLOCK_0 ((uint32_t)0x00007C00) ˇ/USB_COUNT5_RX_0_NUM_BLOCK_0_0 ((uint32_t)0x00000400) Ä0USB_COUNT5_RX_0_NUM_BLOCK_0_1 ((uint32_t)0x00000800) Å0USB_COUNT5_RX_0_NUM_BLOCK_0_2 ((uint32_t)0x00001000) Ç0USB_COUNT5_RX_0_NUM_BLOCK_0_3 ((uint32_t)0x00002000) É0USB_COUNT5_RX_0_NUM_BLOCK_0_4 ((uint32_t)0x00004000) Ö0USB_COUNT5_RX_0_BLSIZE_0 ((uint32_t)0x00008000) à0USB_COUNT5_RX_1_COUNT5_RX_1 ((uint32_t)0x03FF0000) ä0USB_COUNT5_RX_1_NUM_BLOCK_1 ((uint32_t)0x7C000000) ã0USB_COUNT5_RX_1_NUM_BLOCK_1_0 ((uint32_t)0x04000000) å0USB_COUNT5_RX_1_NUM_BLOCK_1_1 ((uint32_t)0x08000000) ç0USB_COUNT5_RX_1_NUM_BLOCK_1_2 ((uint32_t)0x10000000) é0USB_COUNT5_RX_1_NUM_BLOCK_1_3 ((uint32_t)0x20000000) è0USB_COUNT5_RX_1_NUM_BLOCK_1_4 ((uint32_t)0x40000000) ë0USB_COUNT5_RX_1_BLSIZE_1 ((uint32_t)0x80000000) î0USB_COUNT6_RX_0_COUNT6_RX_0 ((uint32_t)0x000003FF) ñ0USB_COUNT6_RX_0_NUM_BLOCK_0 ((uint32_t)0x00007C00) ó0USB_COUNT6_RX_0_NUM_BLOCK_0_0 ((uint32_t)0x00000400) ò0USB_COUNT6_RX_0_NUM_BLOCK_0_1 ((uint32_t)0x00000800) ô0USB_COUNT6_RX_0_NUM_BLOCK_0_2 ((uint32_t)0x00001000) ö0USB_COUNT6_RX_0_NUM_BLOCK_0_3 ((uint32_t)0x00002000) õ0USB_COUNT6_RX_0_NUM_BLOCK_0_4 ((uint32_t)0x00004000) ù0USB_COUNT6_RX_0_BLSIZE_0 ((uint32_t)0x00008000) †0USB_COUNT6_RX_1_COUNT6_RX_1 ((uint32_t)0x03FF0000) ¢0USB_COUNT6_RX_1_NUM_BLOCK_1 ((uint32_t)0x7C000000) £0USB_COUNT6_RX_1_NUM_BLOCK_1_0 ((uint32_t)0x04000000) §0USB_COUNT6_RX_1_NUM_BLOCK_1_1 ((uint32_t)0x08000000) •0USB_COUNT6_RX_1_NUM_BLOCK_1_2 ((uint32_t)0x10000000) ¶0USB_COUNT6_RX_1_NUM_BLOCK_1_3 ((uint32_t)0x20000000) ß0USB_COUNT6_RX_1_NUM_BLOCK_1_4 ((uint32_t)0x40000000) ©0USB_COUNT6_RX_1_BLSIZE_1 ((uint32_t)0x80000000) ¨0USB_COUNT7_RX_0_COUNT7_RX_0 ((uint32_t)0x000003FF) Æ0USB_COUNT7_RX_0_NUM_BLOCK_0 ((uint32_t)0x00007C00) Ø0USB_COUNT7_RX_0_NUM_BLOCK_0_0 ((uint32_t)0x00000400) ∞0USB_COUNT7_RX_0_NUM_BLOCK_0_1 ((uint32_t)0x00000800) ±0USB_COUNT7_RX_0_NUM_BLOCK_0_2 ((uint32_t)0x00001000) ≤0USB_COUNT7_RX_0_NUM_BLOCK_0_3 ((uint32_t)0x00002000) ≥0USB_COUNT7_RX_0_NUM_BLOCK_0_4 ((uint32_t)0x00004000) µ0USB_COUNT7_RX_0_BLSIZE_0 ((uint32_t)0x00008000) ∏0USB_COUNT7_RX_1_COUNT7_RX_1 ((uint32_t)0x03FF0000) ∫0USB_COUNT7_RX_1_NUM_BLOCK_1 ((uint32_t)0x7C000000) ª0USB_COUNT7_RX_1_NUM_BLOCK_1_0 ((uint32_t)0x04000000) º0USB_COUNT7_RX_1_NUM_BLOCK_1_1 ((uint32_t)0x08000000) Ω0USB_COUNT7_RX_1_NUM_BLOCK_1_2 ((uint32_t)0x10000000) æ0USB_COUNT7_RX_1_NUM_BLOCK_1_3 ((uint32_t)0x20000000) ø0USB_COUNT7_RX_1_NUM_BLOCK_1_4 ((uint32_t)0x40000000) ¡0USB_COUNT7_RX_1_BLSIZE_1 ((uint32_t)0x80000000) À0CAN_MCR_INRQ ((uint16_t)0x0001) Ã0CAN_MCR_SLEEP ((uint16_t)0x0002) Õ0CAN_MCR_TXFP ((uint16_t)0x0004) Œ0CAN_MCR_RFLM ((uint16_t)0x0008) œ0CAN_MCR_NART ((uint16_t)0x0010) –0CAN_MCR_AWUM ((uint16_t)0x0020) —0CAN_MCR_ABOM ((uint16_t)0x0040) “0CAN_MCR_TTCM ((uint16_t)0x0080) ”0CAN_MCR_RESET ((uint16_t)0x8000) ÷0CAN_MSR_INAK ((uint16_t)0x0001) ◊0CAN_MSR_SLAK ((uint16_t)0x0002) ÿ0CAN_MSR_ERRI ((uint16_t)0x0004) Ÿ0CAN_MSR_WKUI ((uint16_t)0x0008) ⁄0CAN_MSR_SLAKI ((uint16_t)0x0010) €0CAN_MSR_TXM ((uint16_t)0x0100) ‹0CAN_MSR_RXM ((uint16_t)0x0200) ›0CAN_MSR_SAMP ((uint16_t)0x0400) ﬁ0CAN_MSR_RX ((uint16_t)0x0800) ·0CAN_TSR_RQCP0 ((uint32_t)0x00000001) ‚0CAN_TSR_TXOK0 ((uint32_t)0x00000002) „0CAN_TSR_ALST0 ((uint32_t)0x00000004) ‰0CAN_TSR_TERR0 ((uint32_t)0x00000008) Â0CAN_TSR_ABRQ0 ((uint32_t)0x00000080) Ê0CAN_TSR_RQCP1 ((uint32_t)0x00000100) Á0CAN_TSR_TXOK1 ((uint32_t)0x00000200) Ë0CAN_TSR_ALST1 ((uint32_t)0x00000400) È0CAN_TSR_TERR1 ((uint32_t)0x00000800) Í0CAN_TSR_ABRQ1 ((uint32_t)0x00008000) Î0CAN_TSR_RQCP2 ((uint32_t)0x00010000) Ï0CAN_TSR_TXOK2 ((uint32_t)0x00020000) Ì0CAN_TSR_ALST2 ((uint32_t)0x00040000) Ó0CAN_TSR_TERR2 ((uint32_t)0x00080000) Ô0CAN_TSR_ABRQ2 ((uint32_t)0x00800000) 0CAN_TSR_CODE ((uint32_t)0x03000000) Ú0CAN_TSR_TME ((uint32_t)0x1C000000) Û0CAN_TSR_TME0 ((uint32_t)0x04000000) Ù0CAN_TSR_TME1 ((uint32_t)0x08000000) ı0CAN_TSR_TME2 ((uint32_t)0x10000000) ˜0CAN_TSR_LOW ((uint32_t)0xE0000000) ¯0CAN_TSR_LOW0 ((uint32_t)0x20000000) ˘0CAN_TSR_LOW1 ((uint32_t)0x40000000) ˙0CAN_TSR_LOW2 ((uint32_t)0x80000000) ˝0CAN_RF0R_FMP0 ((uint8_t)0x03) ˛0CAN_RF0R_FULL0 ((uint8_t)0x08) ˇ0CAN_RF0R_FOVR0 ((uint8_t)0x10) Ä1CAN_RF0R_RFOM0 ((uint8_t)0x20) É1CAN_RF1R_FMP1 ((uint8_t)0x03) Ñ1CAN_RF1R_FULL1 ((uint8_t)0x08) Ö1CAN_RF1R_FOVR1 ((uint8_t)0x10) Ü1CAN_RF1R_RFOM1 ((uint8_t)0x20) â1CAN_IER_TMEIE ((uint32_t)0x00000001) ä1CAN_IER_FMPIE0 ((uint32_t)0x00000002) ã1CAN_IER_FFIE0 ((uint32_t)0x00000004) å1CAN_IER_FOVIE0 ((uint32_t)0x00000008) ç1CAN_IER_FMPIE1 ((uint32_t)0x00000010) é1CAN_IER_FFIE1 ((uint32_t)0x00000020) è1CAN_IER_FOVIE1 ((uint32_t)0x00000040) ê1CAN_IER_EWGIE ((uint32_t)0x00000100) ë1CAN_IER_EPVIE ((uint32_t)0x00000200) í1CAN_IER_BOFIE ((uint32_t)0x00000400) ì1CAN_IER_LECIE ((uint32_t)0x00000800) î1CAN_IER_ERRIE ((uint32_t)0x00008000) ï1CAN_IER_WKUIE ((uint32_t)0x00010000) ñ1CAN_IER_SLKIE ((uint32_t)0x00020000) ô1CAN_ESR_EWGF ((uint32_t)0x00000001) ö1CAN_ESR_EPVF ((uint32_t)0x00000002) õ1CAN_ESR_BOFF ((uint32_t)0x00000004) ù1CAN_ESR_LEC ((uint32_t)0x00000070) û1CAN_ESR_LEC_0 ((uint32_t)0x00000010) ü1CAN_ESR_LEC_1 ((uint32_t)0x00000020) †1CAN_ESR_LEC_2 ((uint32_t)0x00000040) ¢1CAN_ESR_TEC ((uint32_t)0x00FF0000) £1CAN_ESR_REC ((uint32_t)0xFF000000) ¶1CAN_BTR_BRP ((uint32_t)0x000003FF) ß1CAN_BTR_TS1 ((uint32_t)0x000F0000) ®1CAN_BTR_TS2 ((uint32_t)0x00700000) ©1CAN_BTR_SJW ((uint32_t)0x03000000) ™1CAN_BTR_LBKM ((uint32_t)0x40000000) ´1CAN_BTR_SILM ((uint32_t)0x80000000) Ø1CAN_TI0R_TXRQ ((uint32_t)0x00000001) ∞1CAN_TI0R_RTR ((uint32_t)0x00000002) ±1CAN_TI0R_IDE ((uint32_t)0x00000004) ≤1CAN_TI0R_EXID ((uint32_t)0x001FFFF8) ≥1CAN_TI0R_STID ((uint32_t)0xFFE00000) ∂1CAN_TDT0R_DLC ((uint32_t)0x0000000F) ∑1CAN_TDT0R_TGT ((uint32_t)0x00000100) ∏1CAN_TDT0R_TIME ((uint32_t)0xFFFF0000) ª1CAN_TDL0R_DATA0 ((uint32_t)0x000000FF) º1CAN_TDL0R_DATA1 ((uint32_t)0x0000FF00) Ω1CAN_TDL0R_DATA2 ((uint32_t)0x00FF0000) æ1CAN_TDL0R_DATA3 ((uint32_t)0xFF000000) ¡1CAN_TDH0R_DATA4 ((uint32_t)0x000000FF) ¬1CAN_TDH0R_DATA5 ((uint32_t)0x0000FF00) √1CAN_TDH0R_DATA6 ((uint32_t)0x00FF0000) ƒ1CAN_TDH0R_DATA7 ((uint32_t)0xFF000000) «1CAN_TI1R_TXRQ ((uint32_t)0x00000001) »1CAN_TI1R_RTR ((uint32_t)0x00000002) …1CAN_TI1R_IDE ((uint32_t)0x00000004)  1CAN_TI1R_EXID ((uint32_t)0x001FFFF8) À1CAN_TI1R_STID ((uint32_t)0xFFE00000) Œ1CAN_TDT1R_DLC ((uint32_t)0x0000000F) œ1CAN_TDT1R_TGT ((uint32_t)0x00000100) –1CAN_TDT1R_TIME ((uint32_t)0xFFFF0000) ”1CAN_TDL1R_DATA0 ((uint32_t)0x000000FF) ‘1CAN_TDL1R_DATA1 ((uint32_t)0x0000FF00) ’1CAN_TDL1R_DATA2 ((uint32_t)0x00FF0000) ÷1CAN_TDL1R_DATA3 ((uint32_t)0xFF000000) Ÿ1CAN_TDH1R_DATA4 ((uint32_t)0x000000FF) ⁄1CAN_TDH1R_DATA5 ((uint32_t)0x0000FF00) €1CAN_TDH1R_DATA6 ((uint32_t)0x00FF0000) ‹1CAN_TDH1R_DATA7 ((uint32_t)0xFF000000) ﬂ1CAN_TI2R_TXRQ ((uint32_t)0x00000001) ‡1CAN_TI2R_RTR ((uint32_t)0x00000002) ·1CAN_TI2R_IDE ((uint32_t)0x00000004) ‚1CAN_TI2R_EXID ((uint32_t)0x001FFFF8) „1CAN_TI2R_STID ((uint32_t)0xFFE00000) Ê1CAN_TDT2R_DLC ((uint32_t)0x0000000F) Á1CAN_TDT2R_TGT ((uint32_t)0x00000100) Ë1CAN_TDT2R_TIME ((uint32_t)0xFFFF0000) Î1CAN_TDL2R_DATA0 ((uint32_t)0x000000FF) Ï1CAN_TDL2R_DATA1 ((uint32_t)0x0000FF00) Ì1CAN_TDL2R_DATA2 ((uint32_t)0x00FF0000) Ó1CAN_TDL2R_DATA3 ((uint32_t)0xFF000000) Ò1CAN_TDH2R_DATA4 ((uint32_t)0x000000FF) Ú1CAN_TDH2R_DATA5 ((uint32_t)0x0000FF00) Û1CAN_TDH2R_DATA6 ((uint32_t)0x00FF0000) Ù1CAN_TDH2R_DATA7 ((uint32_t)0xFF000000) ˜1CAN_RI0R_RTR ((uint32_t)0x00000002) ¯1CAN_RI0R_IDE ((uint32_t)0x00000004) ˘1CAN_RI0R_EXID ((uint32_t)0x001FFFF8) ˙1CAN_RI0R_STID ((uint32_t)0xFFE00000) ˝1CAN_RDT0R_DLC ((uint32_t)0x0000000F) ˛1CAN_RDT0R_FMI ((uint32_t)0x0000FF00) ˇ1CAN_RDT0R_TIME ((uint32_t)0xFFFF0000) Ç2CAN_RDL0R_DATA0 ((uint32_t)0x000000FF) É2CAN_RDL0R_DATA1 ((uint32_t)0x0000FF00) Ñ2CAN_RDL0R_DATA2 ((uint32_t)0x00FF0000) Ö2CAN_RDL0R_DATA3 ((uint32_t)0xFF000000) à2CAN_RDH0R_DATA4 ((uint32_t)0x000000FF) â2CAN_RDH0R_DATA5 ((uint32_t)0x0000FF00) ä2CAN_RDH0R_DATA6 ((uint32_t)0x00FF0000) ã2CAN_RDH0R_DATA7 ((uint32_t)0xFF000000) é2CAN_RI1R_RTR ((uint32_t)0x00000002) è2CAN_RI1R_IDE ((uint32_t)0x00000004) ê2CAN_RI1R_EXID ((uint32_t)0x001FFFF8) ë2CAN_RI1R_STID ((uint32_t)0xFFE00000) î2CAN_RDT1R_DLC ((uint32_t)0x0000000F) ï2CAN_RDT1R_FMI ((uint32_t)0x0000FF00) ñ2CAN_RDT1R_TIME ((uint32_t)0xFFFF0000) ô2CAN_RDL1R_DATA0 ((uint32_t)0x000000FF) ö2CAN_RDL1R_DATA1 ((uint32_t)0x0000FF00) õ2CAN_RDL1R_DATA2 ((uint32_t)0x00FF0000) ú2CAN_RDL1R_DATA3 ((uint32_t)0xFF000000) ü2CAN_RDH1R_DATA4 ((uint32_t)0x000000FF) †2CAN_RDH1R_DATA5 ((uint32_t)0x0000FF00) °2CAN_RDH1R_DATA6 ((uint32_t)0x00FF0000) ¢2CAN_RDH1R_DATA7 ((uint32_t)0xFF000000) ¶2CAN_FMR_FINIT ((uint8_t)0x01) ©2CAN_FM1R_FBM ((uint16_t)0x3FFF) ™2CAN_FM1R_FBM0 ((uint16_t)0x0001) ´2CAN_FM1R_FBM1 ((uint16_t)0x0002) ¨2CAN_FM1R_FBM2 ((uint16_t)0x0004) ≠2CAN_FM1R_FBM3 ((uint16_t)0x0008) Æ2CAN_FM1R_FBM4 ((uint16_t)0x0010) Ø2CAN_FM1R_FBM5 ((uint16_t)0x0020) ∞2CAN_FM1R_FBM6 ((uint16_t)0x0040) ±2CAN_FM1R_FBM7 ((uint16_t)0x0080) ≤2CAN_FM1R_FBM8 ((uint16_t)0x0100) ≥2CAN_FM1R_FBM9 ((uint16_t)0x0200) ¥2CAN_FM1R_FBM10 ((uint16_t)0x0400) µ2CAN_FM1R_FBM11 ((uint16_t)0x0800) ∂2CAN_FM1R_FBM12 ((uint16_t)0x1000) ∑2CAN_FM1R_FBM13 ((uint16_t)0x2000) ∫2CAN_FS1R_FSC ((uint16_t)0x3FFF) ª2CAN_FS1R_FSC0 ((uint16_t)0x0001) º2CAN_FS1R_FSC1 ((uint16_t)0x0002) Ω2CAN_FS1R_FSC2 ((uint16_t)0x0004) æ2CAN_FS1R_FSC3 ((uint16_t)0x0008) ø2CAN_FS1R_FSC4 ((uint16_t)0x0010) ¿2CAN_FS1R_FSC5 ((uint16_t)0x0020) ¡2CAN_FS1R_FSC6 ((uint16_t)0x0040) ¬2CAN_FS1R_FSC7 ((uint16_t)0x0080) √2CAN_FS1R_FSC8 ((uint16_t)0x0100) ƒ2CAN_FS1R_FSC9 ((uint16_t)0x0200) ≈2CAN_FS1R_FSC10 ((uint16_t)0x0400) ∆2CAN_FS1R_FSC11 ((uint16_t)0x0800) «2CAN_FS1R_FSC12 ((uint16_t)0x1000) »2CAN_FS1R_FSC13 ((uint16_t)0x2000) À2CAN_FFA1R_FFA ((uint16_t)0x3FFF) Ã2CAN_FFA1R_FFA0 ((uint16_t)0x0001) Õ2CAN_FFA1R_FFA1 ((uint16_t)0x0002) Œ2CAN_FFA1R_FFA2 ((uint16_t)0x0004) œ2CAN_FFA1R_FFA3 ((uint16_t)0x0008) –2CAN_FFA1R_FFA4 ((uint16_t)0x0010) —2CAN_FFA1R_FFA5 ((uint16_t)0x0020) “2CAN_FFA1R_FFA6 ((uint16_t)0x0040) ”2CAN_FFA1R_FFA7 ((uint16_t)0x0080) ‘2CAN_FFA1R_FFA8 ((uint16_t)0x0100) ’2CAN_FFA1R_FFA9 ((uint16_t)0x0200) ÷2CAN_FFA1R_FFA10 ((uint16_t)0x0400) ◊2CAN_FFA1R_FFA11 ((uint16_t)0x0800) ÿ2CAN_FFA1R_FFA12 ((uint16_t)0x1000) Ÿ2CAN_FFA1R_FFA13 ((uint16_t)0x2000) ‹2CAN_FA1R_FACT ((uint16_t)0x3FFF) ›2CAN_FA1R_FACT0 ((uint16_t)0x0001) ﬁ2CAN_FA1R_FACT1 ((uint16_t)0x0002) ﬂ2CAN_FA1R_FACT2 ((uint16_t)0x0004) ‡2CAN_FA1R_FACT3 ((uint16_t)0x0008) ·2CAN_FA1R_FACT4 ((uint16_t)0x0010) ‚2CAN_FA1R_FACT5 ((uint16_t)0x0020) „2CAN_FA1R_FACT6 ((uint16_t)0x0040) ‰2CAN_FA1R_FACT7 ((uint16_t)0x0080) Â2CAN_FA1R_FACT8 ((uint16_t)0x0100) Ê2CAN_FA1R_FACT9 ((uint16_t)0x0200) Á2CAN_FA1R_FACT10 ((uint16_t)0x0400) Ë2CAN_FA1R_FACT11 ((uint16_t)0x0800) È2CAN_FA1R_FACT12 ((uint16_t)0x1000) Í2CAN_FA1R_FACT13 ((uint16_t)0x2000) Ì2CAN_F0R1_FB0 ((uint32_t)0x00000001) Ó2CAN_F0R1_FB1 ((uint32_t)0x00000002) Ô2CAN_F0R1_FB2 ((uint32_t)0x00000004) 2CAN_F0R1_FB3 ((uint32_t)0x00000008) Ò2CAN_F0R1_FB4 ((uint32_t)0x00000010) Ú2CAN_F0R1_FB5 ((uint32_t)0x00000020) Û2CAN_F0R1_FB6 ((uint32_t)0x00000040) Ù2CAN_F0R1_FB7 ((uint32_t)0x00000080) ı2CAN_F0R1_FB8 ((uint32_t)0x00000100) ˆ2CAN_F0R1_FB9 ((uint32_t)0x00000200) ˜2CAN_F0R1_FB10 ((uint32_t)0x00000400) ¯2CAN_F0R1_FB11 ((uint32_t)0x00000800) ˘2CAN_F0R1_FB12 ((uint32_t)0x00001000) ˙2CAN_F0R1_FB13 ((uint32_t)0x00002000) ˚2CAN_F0R1_FB14 ((uint32_t)0x00004000) ¸2CAN_F0R1_FB15 ((uint32_t)0x00008000) ˝2CAN_F0R1_FB16 ((uint32_t)0x00010000) ˛2CAN_F0R1_FB17 ((uint32_t)0x00020000) ˇ2CAN_F0R1_FB18 ((uint32_t)0x00040000) Ä3CAN_F0R1_FB19 ((uint32_t)0x00080000) Å3CAN_F0R1_FB20 ((uint32_t)0x00100000) Ç3CAN_F0R1_FB21 ((uint32_t)0x00200000) É3CAN_F0R1_FB22 ((uint32_t)0x00400000) Ñ3CAN_F0R1_FB23 ((uint32_t)0x00800000) Ö3CAN_F0R1_FB24 ((uint32_t)0x01000000) Ü3CAN_F0R1_FB25 ((uint32_t)0x02000000) á3CAN_F0R1_FB26 ((uint32_t)0x04000000) à3CAN_F0R1_FB27 ((uint32_t)0x08000000) â3CAN_F0R1_FB28 ((uint32_t)0x10000000) ä3CAN_F0R1_FB29 ((uint32_t)0x20000000) ã3CAN_F0R1_FB30 ((uint32_t)0x40000000) å3CAN_F0R1_FB31 ((uint32_t)0x80000000) è3CAN_F1R1_FB0 ((uint32_t)0x00000001) ê3CAN_F1R1_FB1 ((uint32_t)0x00000002) ë3CAN_F1R1_FB2 ((uint32_t)0x00000004) í3CAN_F1R1_FB3 ((uint32_t)0x00000008) ì3CAN_F1R1_FB4 ((uint32_t)0x00000010) î3CAN_F1R1_FB5 ((uint32_t)0x00000020) ï3CAN_F1R1_FB6 ((uint32_t)0x00000040) ñ3CAN_F1R1_FB7 ((uint32_t)0x00000080) ó3CAN_F1R1_FB8 ((uint32_t)0x00000100) ò3CAN_F1R1_FB9 ((uint32_t)0x00000200) ô3CAN_F1R1_FB10 ((uint32_t)0x00000400) ö3CAN_F1R1_FB11 ((uint32_t)0x00000800) õ3CAN_F1R1_FB12 ((uint32_t)0x00001000) ú3CAN_F1R1_FB13 ((uint32_t)0x00002000) ù3CAN_F1R1_FB14 ((uint32_t)0x00004000) û3CAN_F1R1_FB15 ((uint32_t)0x00008000) ü3CAN_F1R1_FB16 ((uint32_t)0x00010000) †3CAN_F1R1_FB17 ((uint32_t)0x00020000) °3CAN_F1R1_FB18 ((uint32_t)0x00040000) ¢3CAN_F1R1_FB19 ((uint32_t)0x00080000) £3CAN_F1R1_FB20 ((uint32_t)0x00100000) §3CAN_F1R1_FB21 ((uint32_t)0x00200000) •3CAN_F1R1_FB22 ((uint32_t)0x00400000) ¶3CAN_F1R1_FB23 ((uint32_t)0x00800000) ß3CAN_F1R1_FB24 ((uint32_t)0x01000000) ®3CAN_F1R1_FB25 ((uint32_t)0x02000000) ©3CAN_F1R1_FB26 ((uint32_t)0x04000000) ™3CAN_F1R1_FB27 ((uint32_t)0x08000000) ´3CAN_F1R1_FB28 ((uint32_t)0x10000000) ¨3CAN_F1R1_FB29 ((uint32_t)0x20000000) ≠3CAN_F1R1_FB30 ((uint32_t)0x40000000) Æ3CAN_F1R1_FB31 ((uint32_t)0x80000000) ±3CAN_F2R1_FB0 ((uint32_t)0x00000001) ≤3CAN_F2R1_FB1 ((uint32_t)0x00000002) ≥3CAN_F2R1_FB2 ((uint32_t)0x00000004) ¥3CAN_F2R1_FB3 ((uint32_t)0x00000008) µ3CAN_F2R1_FB4 ((uint32_t)0x00000010) ∂3CAN_F2R1_FB5 ((uint32_t)0x00000020) ∑3CAN_F2R1_FB6 ((uint32_t)0x00000040) ∏3CAN_F2R1_FB7 ((uint32_t)0x00000080) π3CAN_F2R1_FB8 ((uint32_t)0x00000100) ∫3CAN_F2R1_FB9 ((uint32_t)0x00000200) ª3CAN_F2R1_FB10 ((uint32_t)0x00000400) º3CAN_F2R1_FB11 ((uint32_t)0x00000800) Ω3CAN_F2R1_FB12 ((uint32_t)0x00001000) æ3CAN_F2R1_FB13 ((uint32_t)0x00002000) ø3CAN_F2R1_FB14 ((uint32_t)0x00004000) ¿3CAN_F2R1_FB15 ((uint32_t)0x00008000) ¡3CAN_F2R1_FB16 ((uint32_t)0x00010000) ¬3CAN_F2R1_FB17 ((uint32_t)0x00020000) √3CAN_F2R1_FB18 ((uint32_t)0x00040000) ƒ3CAN_F2R1_FB19 ((uint32_t)0x00080000) ≈3CAN_F2R1_FB20 ((uint32_t)0x00100000) ∆3CAN_F2R1_FB21 ((uint32_t)0x00200000) «3CAN_F2R1_FB22 ((uint32_t)0x00400000) »3CAN_F2R1_FB23 ((uint32_t)0x00800000) …3CAN_F2R1_FB24 ((uint32_t)0x01000000)  3CAN_F2R1_FB25 ((uint32_t)0x02000000) À3CAN_F2R1_FB26 ((uint32_t)0x04000000) Ã3CAN_F2R1_FB27 ((uint32_t)0x08000000) Õ3CAN_F2R1_FB28 ((uint32_t)0x10000000) Œ3CAN_F2R1_FB29 ((uint32_t)0x20000000) œ3CAN_F2R1_FB30 ((uint32_t)0x40000000) –3CAN_F2R1_FB31 ((uint32_t)0x80000000) ”3CAN_F3R1_FB0 ((uint32_t)0x00000001) ‘3CAN_F3R1_FB1 ((uint32_t)0x00000002) ’3CAN_F3R1_FB2 ((uint32_t)0x00000004) ÷3CAN_F3R1_FB3 ((uint32_t)0x00000008) ◊3CAN_F3R1_FB4 ((uint32_t)0x00000010) ÿ3CAN_F3R1_FB5 ((uint32_t)0x00000020) Ÿ3CAN_F3R1_FB6 ((uint32_t)0x00000040) ⁄3CAN_F3R1_FB7 ((uint32_t)0x00000080) €3CAN_F3R1_FB8 ((uint32_t)0x00000100) ‹3CAN_F3R1_FB9 ((uint32_t)0x00000200) ›3CAN_F3R1_FB10 ((uint32_t)0x00000400) ﬁ3CAN_F3R1_FB11 ((uint32_t)0x00000800) ﬂ3CAN_F3R1_FB12 ((uint32_t)0x00001000) ‡3CAN_F3R1_FB13 ((uint32_t)0x00002000) ·3CAN_F3R1_FB14 ((uint32_t)0x00004000) ‚3CAN_F3R1_FB15 ((uint32_t)0x00008000) „3CAN_F3R1_FB16 ((uint32_t)0x00010000) ‰3CAN_F3R1_FB17 ((uint32_t)0x00020000) Â3CAN_F3R1_FB18 ((uint32_t)0x00040000) Ê3CAN_F3R1_FB19 ((uint32_t)0x00080000) Á3CAN_F3R1_FB20 ((uint32_t)0x00100000) Ë3CAN_F3R1_FB21 ((uint32_t)0x00200000) È3CAN_F3R1_FB22 ((uint32_t)0x00400000) Í3CAN_F3R1_FB23 ((uint32_t)0x00800000) Î3CAN_F3R1_FB24 ((uint32_t)0x01000000) Ï3CAN_F3R1_FB25 ((uint32_t)0x02000000) Ì3CAN_F3R1_FB26 ((uint32_t)0x04000000) Ó3CAN_F3R1_FB27 ((uint32_t)0x08000000) Ô3CAN_F3R1_FB28 ((uint32_t)0x10000000) 3CAN_F3R1_FB29 ((uint32_t)0x20000000) Ò3CAN_F3R1_FB30 ((uint32_t)0x40000000) Ú3CAN_F3R1_FB31 ((uint32_t)0x80000000) ı3CAN_F4R1_FB0 ((uint32_t)0x00000001) ˆ3CAN_F4R1_FB1 ((uint32_t)0x00000002) ˜3CAN_F4R1_FB2 ((uint32_t)0x00000004) ¯3CAN_F4R1_FB3 ((uint32_t)0x00000008) ˘3CAN_F4R1_FB4 ((uint32_t)0x00000010) ˙3CAN_F4R1_FB5 ((uint32_t)0x00000020) ˚3CAN_F4R1_FB6 ((uint32_t)0x00000040) ¸3CAN_F4R1_FB7 ((uint32_t)0x00000080) ˝3CAN_F4R1_FB8 ((uint32_t)0x00000100) ˛3CAN_F4R1_FB9 ((uint32_t)0x00000200) ˇ3CAN_F4R1_FB10 ((uint32_t)0x00000400) Ä4CAN_F4R1_FB11 ((uint32_t)0x00000800) Å4CAN_F4R1_FB12 ((uint32_t)0x00001000) Ç4CAN_F4R1_FB13 ((uint32_t)0x00002000) É4CAN_F4R1_FB14 ((uint32_t)0x00004000) Ñ4CAN_F4R1_FB15 ((uint32_t)0x00008000) Ö4CAN_F4R1_FB16 ((uint32_t)0x00010000) Ü4CAN_F4R1_FB17 ((uint32_t)0x00020000) á4CAN_F4R1_FB18 ((uint32_t)0x00040000) à4CAN_F4R1_FB19 ((uint32_t)0x00080000) â4CAN_F4R1_FB20 ((uint32_t)0x00100000) ä4CAN_F4R1_FB21 ((uint32_t)0x00200000) ã4CAN_F4R1_FB22 ((uint32_t)0x00400000) å4CAN_F4R1_FB23 ((uint32_t)0x00800000) ç4CAN_F4R1_FB24 ((uint32_t)0x01000000) é4CAN_F4R1_FB25 ((uint32_t)0x02000000) è4CAN_F4R1_FB26 ((uint32_t)0x04000000) ê4CAN_F4R1_FB27 ((uint32_t)0x08000000) ë4CAN_F4R1_FB28 ((uint32_t)0x10000000) í4CAN_F4R1_FB29 ((uint32_t)0x20000000) ì4CAN_F4R1_FB30 ((uint32_t)0x40000000) î4CAN_F4R1_FB31 ((uint32_t)0x80000000) ó4CAN_F5R1_FB0 ((uint32_t)0x00000001) ò4CAN_F5R1_FB1 ((uint32_t)0x00000002) ô4CAN_F5R1_FB2 ((uint32_t)0x00000004) ö4CAN_F5R1_FB3 ((uint32_t)0x00000008) õ4CAN_F5R1_FB4 ((uint32_t)0x00000010) ú4CAN_F5R1_FB5 ((uint32_t)0x00000020) ù4CAN_F5R1_FB6 ((uint32_t)0x00000040) û4CAN_F5R1_FB7 ((uint32_t)0x00000080) ü4CAN_F5R1_FB8 ((uint32_t)0x00000100) †4CAN_F5R1_FB9 ((uint32_t)0x00000200) °4CAN_F5R1_FB10 ((uint32_t)0x00000400) ¢4CAN_F5R1_FB11 ((uint32_t)0x00000800) £4CAN_F5R1_FB12 ((uint32_t)0x00001000) §4CAN_F5R1_FB13 ((uint32_t)0x00002000) •4CAN_F5R1_FB14 ((uint32_t)0x00004000) ¶4CAN_F5R1_FB15 ((uint32_t)0x00008000) ß4CAN_F5R1_FB16 ((uint32_t)0x00010000) ®4CAN_F5R1_FB17 ((uint32_t)0x00020000) ©4CAN_F5R1_FB18 ((uint32_t)0x00040000) ™4CAN_F5R1_FB19 ((uint32_t)0x00080000) ´4CAN_F5R1_FB20 ((uint32_t)0x00100000) ¨4CAN_F5R1_FB21 ((uint32_t)0x00200000) ≠4CAN_F5R1_FB22 ((uint32_t)0x00400000) Æ4CAN_F5R1_FB23 ((uint32_t)0x00800000) Ø4CAN_F5R1_FB24 ((uint32_t)0x01000000) ∞4CAN_F5R1_FB25 ((uint32_t)0x02000000) ±4CAN_F5R1_FB26 ((uint32_t)0x04000000) ≤4CAN_F5R1_FB27 ((uint32_t)0x08000000) ≥4CAN_F5R1_FB28 ((uint32_t)0x10000000) ¥4CAN_F5R1_FB29 ((uint32_t)0x20000000) µ4CAN_F5R1_FB30 ((uint32_t)0x40000000) ∂4CAN_F5R1_FB31 ((uint32_t)0x80000000) π4CAN_F6R1_FB0 ((uint32_t)0x00000001) ∫4CAN_F6R1_FB1 ((uint32_t)0x00000002) ª4CAN_F6R1_FB2 ((uint32_t)0x00000004) º4CAN_F6R1_FB3 ((uint32_t)0x00000008) Ω4CAN_F6R1_FB4 ((uint32_t)0x00000010) æ4CAN_F6R1_FB5 ((uint32_t)0x00000020) ø4CAN_F6R1_FB6 ((uint32_t)0x00000040) ¿4CAN_F6R1_FB7 ((uint32_t)0x00000080) ¡4CAN_F6R1_FB8 ((uint32_t)0x00000100) ¬4CAN_F6R1_FB9 ((uint32_t)0x00000200) √4CAN_F6R1_FB10 ((uint32_t)0x00000400) ƒ4CAN_F6R1_FB11 ((uint32_t)0x00000800) ≈4CAN_F6R1_FB12 ((uint32_t)0x00001000) ∆4CAN_F6R1_FB13 ((uint32_t)0x00002000) «4CAN_F6R1_FB14 ((uint32_t)0x00004000) »4CAN_F6R1_FB15 ((uint32_t)0x00008000) …4CAN_F6R1_FB16 ((uint32_t)0x00010000)  4CAN_F6R1_FB17 ((uint32_t)0x00020000) À4CAN_F6R1_FB18 ((uint32_t)0x00040000) Ã4CAN_F6R1_FB19 ((uint32_t)0x00080000) Õ4CAN_F6R1_FB20 ((uint32_t)0x00100000) Œ4CAN_F6R1_FB21 ((uint32_t)0x00200000) œ4CAN_F6R1_FB22 ((uint32_t)0x00400000) –4CAN_F6R1_FB23 ((uint32_t)0x00800000) —4CAN_F6R1_FB24 ((uint32_t)0x01000000) “4CAN_F6R1_FB25 ((uint32_t)0x02000000) ”4CAN_F6R1_FB26 ((uint32_t)0x04000000) ‘4CAN_F6R1_FB27 ((uint32_t)0x08000000) ’4CAN_F6R1_FB28 ((uint32_t)0x10000000) ÷4CAN_F6R1_FB29 ((uint32_t)0x20000000) ◊4CAN_F6R1_FB30 ((uint32_t)0x40000000) ÿ4CAN_F6R1_FB31 ((uint32_t)0x80000000) €4CAN_F7R1_FB0 ((uint32_t)0x00000001) ‹4CAN_F7R1_FB1 ((uint32_t)0x00000002) ›4CAN_F7R1_FB2 ((uint32_t)0x00000004) ﬁ4CAN_F7R1_FB3 ((uint32_t)0x00000008) ﬂ4CAN_F7R1_FB4 ((uint32_t)0x00000010) ‡4CAN_F7R1_FB5 ((uint32_t)0x00000020) ·4CAN_F7R1_FB6 ((uint32_t)0x00000040) ‚4CAN_F7R1_FB7 ((uint32_t)0x00000080) „4CAN_F7R1_FB8 ((uint32_t)0x00000100) ‰4CAN_F7R1_FB9 ((uint32_t)0x00000200) Â4CAN_F7R1_FB10 ((uint32_t)0x00000400) Ê4CAN_F7R1_FB11 ((uint32_t)0x00000800) Á4CAN_F7R1_FB12 ((uint32_t)0x00001000) Ë4CAN_F7R1_FB13 ((uint32_t)0x00002000) È4CAN_F7R1_FB14 ((uint32_t)0x00004000) Í4CAN_F7R1_FB15 ((uint32_t)0x00008000) Î4CAN_F7R1_FB16 ((uint32_t)0x00010000) Ï4CAN_F7R1_FB17 ((uint32_t)0x00020000) Ì4CAN_F7R1_FB18 ((uint32_t)0x00040000) Ó4CAN_F7R1_FB19 ((uint32_t)0x00080000) Ô4CAN_F7R1_FB20 ((uint32_t)0x00100000) 4CAN_F7R1_FB21 ((uint32_t)0x00200000) Ò4CAN_F7R1_FB22 ((uint32_t)0x00400000) Ú4CAN_F7R1_FB23 ((uint32_t)0x00800000) Û4CAN_F7R1_FB24 ((uint32_t)0x01000000) Ù4CAN_F7R1_FB25 ((uint32_t)0x02000000) ı4CAN_F7R1_FB26 ((uint32_t)0x04000000) ˆ4CAN_F7R1_FB27 ((uint32_t)0x08000000) ˜4CAN_F7R1_FB28 ((uint32_t)0x10000000) ¯4CAN_F7R1_FB29 ((uint32_t)0x20000000) ˘4CAN_F7R1_FB30 ((uint32_t)0x40000000) ˙4CAN_F7R1_FB31 ((uint32_t)0x80000000) ˝4CAN_F8R1_FB0 ((uint32_t)0x00000001) ˛4CAN_F8R1_FB1 ((uint32_t)0x00000002) ˇ4CAN_F8R1_FB2 ((uint32_t)0x00000004) Ä5CAN_F8R1_FB3 ((uint32_t)0x00000008) Å5CAN_F8R1_FB4 ((uint32_t)0x00000010) Ç5CAN_F8R1_FB5 ((uint32_t)0x00000020) É5CAN_F8R1_FB6 ((uint32_t)0x00000040) Ñ5CAN_F8R1_FB7 ((uint32_t)0x00000080) Ö5CAN_F8R1_FB8 ((uint32_t)0x00000100) Ü5CAN_F8R1_FB9 ((uint32_t)0x00000200) á5CAN_F8R1_FB10 ((uint32_t)0x00000400) à5CAN_F8R1_FB11 ((uint32_t)0x00000800) â5CAN_F8R1_FB12 ((uint32_t)0x00001000) ä5CAN_F8R1_FB13 ((uint32_t)0x00002000) ã5CAN_F8R1_FB14 ((uint32_t)0x00004000) å5CAN_F8R1_FB15 ((uint32_t)0x00008000) ç5CAN_F8R1_FB16 ((uint32_t)0x00010000) é5CAN_F8R1_FB17 ((uint32_t)0x00020000) è5CAN_F8R1_FB18 ((uint32_t)0x00040000) ê5CAN_F8R1_FB19 ((uint32_t)0x00080000) ë5CAN_F8R1_FB20 ((uint32_t)0x00100000) í5CAN_F8R1_FB21 ((uint32_t)0x00200000) ì5CAN_F8R1_FB22 ((uint32_t)0x00400000) î5CAN_F8R1_FB23 ((uint32_t)0x00800000) ï5CAN_F8R1_FB24 ((uint32_t)0x01000000) ñ5CAN_F8R1_FB25 ((uint32_t)0x02000000) ó5CAN_F8R1_FB26 ((uint32_t)0x04000000) ò5CAN_F8R1_FB27 ((uint32_t)0x08000000) ô5CAN_F8R1_FB28 ((uint32_t)0x10000000) ö5CAN_F8R1_FB29 ((uint32_t)0x20000000) õ5CAN_F8R1_FB30 ((uint32_t)0x40000000) ú5CAN_F8R1_FB31 ((uint32_t)0x80000000) ü5CAN_F9R1_FB0 ((uint32_t)0x00000001) †5CAN_F9R1_FB1 ((uint32_t)0x00000002) °5CAN_F9R1_FB2 ((uint32_t)0x00000004) ¢5CAN_F9R1_FB3 ((uint32_t)0x00000008) £5CAN_F9R1_FB4 ((uint32_t)0x00000010) §5CAN_F9R1_FB5 ((uint32_t)0x00000020) •5CAN_F9R1_FB6 ((uint32_t)0x00000040) ¶5CAN_F9R1_FB7 ((uint32_t)0x00000080) ß5CAN_F9R1_FB8 ((uint32_t)0x00000100) ®5CAN_F9R1_FB9 ((uint32_t)0x00000200) ©5CAN_F9R1_FB10 ((uint32_t)0x00000400) ™5CAN_F9R1_FB11 ((uint32_t)0x00000800) ´5CAN_F9R1_FB12 ((uint32_t)0x00001000) ¨5CAN_F9R1_FB13 ((uint32_t)0x00002000) ≠5CAN_F9R1_FB14 ((uint32_t)0x00004000) Æ5CAN_F9R1_FB15 ((uint32_t)0x00008000) Ø5CAN_F9R1_FB16 ((uint32_t)0x00010000) ∞5CAN_F9R1_FB17 ((uint32_t)0x00020000) ±5CAN_F9R1_FB18 ((uint32_t)0x00040000) ≤5CAN_F9R1_FB19 ((uint32_t)0x00080000) ≥5CAN_F9R1_FB20 ((uint32_t)0x00100000) ¥5CAN_F9R1_FB21 ((uint32_t)0x00200000) µ5CAN_F9R1_FB22 ((uint32_t)0x00400000) ∂5CAN_F9R1_FB23 ((uint32_t)0x00800000) ∑5CAN_F9R1_FB24 ((uint32_t)0x01000000) ∏5CAN_F9R1_FB25 ((uint32_t)0x02000000) π5CAN_F9R1_FB26 ((uint32_t)0x04000000) ∫5CAN_F9R1_FB27 ((uint32_t)0x08000000) ª5CAN_F9R1_FB28 ((uint32_t)0x10000000) º5CAN_F9R1_FB29 ((uint32_t)0x20000000) Ω5CAN_F9R1_FB30 ((uint32_t)0x40000000) æ5CAN_F9R1_FB31 ((uint32_t)0x80000000) ¡5CAN_F10R1_FB0 ((uint32_t)0x00000001) ¬5CAN_F10R1_FB1 ((uint32_t)0x00000002) √5CAN_F10R1_FB2 ((uint32_t)0x00000004) ƒ5CAN_F10R1_FB3 ((uint32_t)0x00000008) ≈5CAN_F10R1_FB4 ((uint32_t)0x00000010) ∆5CAN_F10R1_FB5 ((uint32_t)0x00000020) «5CAN_F10R1_FB6 ((uint32_t)0x00000040) »5CAN_F10R1_FB7 ((uint32_t)0x00000080) …5CAN_F10R1_FB8 ((uint32_t)0x00000100)  5CAN_F10R1_FB9 ((uint32_t)0x00000200) À5CAN_F10R1_FB10 ((uint32_t)0x00000400) Ã5CAN_F10R1_FB11 ((uint32_t)0x00000800) Õ5CAN_F10R1_FB12 ((uint32_t)0x00001000) Œ5CAN_F10R1_FB13 ((uint32_t)0x00002000) œ5CAN_F10R1_FB14 ((uint32_t)0x00004000) –5CAN_F10R1_FB15 ((uint32_t)0x00008000) —5CAN_F10R1_FB16 ((uint32_t)0x00010000) “5CAN_F10R1_FB17 ((uint32_t)0x00020000) ”5CAN_F10R1_FB18 ((uint32_t)0x00040000) ‘5CAN_F10R1_FB19 ((uint32_t)0x00080000) ’5CAN_F10R1_FB20 ((uint32_t)0x00100000) ÷5CAN_F10R1_FB21 ((uint32_t)0x00200000) ◊5CAN_F10R1_FB22 ((uint32_t)0x00400000) ÿ5CAN_F10R1_FB23 ((uint32_t)0x00800000) Ÿ5CAN_F10R1_FB24 ((uint32_t)0x01000000) ⁄5CAN_F10R1_FB25 ((uint32_t)0x02000000) €5CAN_F10R1_FB26 ((uint32_t)0x04000000) ‹5CAN_F10R1_FB27 ((uint32_t)0x08000000) ›5CAN_F10R1_FB28 ((uint32_t)0x10000000) ﬁ5CAN_F10R1_FB29 ((uint32_t)0x20000000) ﬂ5CAN_F10R1_FB30 ((uint32_t)0x40000000) ‡5CAN_F10R1_FB31 ((uint32_t)0x80000000) „5CAN_F11R1_FB0 ((uint32_t)0x00000001) ‰5CAN_F11R1_FB1 ((uint32_t)0x00000002) Â5CAN_F11R1_FB2 ((uint32_t)0x00000004) Ê5CAN_F11R1_FB3 ((uint32_t)0x00000008) Á5CAN_F11R1_FB4 ((uint32_t)0x00000010) Ë5CAN_F11R1_FB5 ((uint32_t)0x00000020) È5CAN_F11R1_FB6 ((uint32_t)0x00000040) Í5CAN_F11R1_FB7 ((uint32_t)0x00000080) Î5CAN_F11R1_FB8 ((uint32_t)0x00000100) Ï5CAN_F11R1_FB9 ((uint32_t)0x00000200) Ì5CAN_F11R1_FB10 ((uint32_t)0x00000400) Ó5CAN_F11R1_FB11 ((uint32_t)0x00000800) Ô5CAN_F11R1_FB12 ((uint32_t)0x00001000) 5CAN_F11R1_FB13 ((uint32_t)0x00002000) Ò5CAN_F11R1_FB14 ((uint32_t)0x00004000) Ú5CAN_F11R1_FB15 ((uint32_t)0x00008000) Û5CAN_F11R1_FB16 ((uint32_t)0x00010000) Ù5CAN_F11R1_FB17 ((uint32_t)0x00020000) ı5CAN_F11R1_FB18 ((uint32_t)0x00040000) ˆ5CAN_F11R1_FB19 ((uint32_t)0x00080000) ˜5CAN_F11R1_FB20 ((uint32_t)0x00100000) ¯5CAN_F11R1_FB21 ((uint32_t)0x00200000) ˘5CAN_F11R1_FB22 ((uint32_t)0x00400000) ˙5CAN_F11R1_FB23 ((uint32_t)0x00800000) ˚5CAN_F11R1_FB24 ((uint32_t)0x01000000) ¸5CAN_F11R1_FB25 ((uint32_t)0x02000000) ˝5CAN_F11R1_FB26 ((uint32_t)0x04000000) ˛5CAN_F11R1_FB27 ((uint32_t)0x08000000) ˇ5CAN_F11R1_FB28 ((uint32_t)0x10000000) Ä6CAN_F11R1_FB29 ((uint32_t)0x20000000) Å6CAN_F11R1_FB30 ((uint32_t)0x40000000) Ç6CAN_F11R1_FB31 ((uint32_t)0x80000000) Ö6CAN_F12R1_FB0 ((uint32_t)0x00000001) Ü6CAN_F12R1_FB1 ((uint32_t)0x00000002) á6CAN_F12R1_FB2 ((uint32_t)0x00000004) à6CAN_F12R1_FB3 ((uint32_t)0x00000008) â6CAN_F12R1_FB4 ((uint32_t)0x00000010) ä6CAN_F12R1_FB5 ((uint32_t)0x00000020) ã6CAN_F12R1_FB6 ((uint32_t)0x00000040) å6CAN_F12R1_FB7 ((uint32_t)0x00000080) ç6CAN_F12R1_FB8 ((uint32_t)0x00000100) é6CAN_F12R1_FB9 ((uint32_t)0x00000200) è6CAN_F12R1_FB10 ((uint32_t)0x00000400) ê6CAN_F12R1_FB11 ((uint32_t)0x00000800) ë6CAN_F12R1_FB12 ((uint32_t)0x00001000) í6CAN_F12R1_FB13 ((uint32_t)0x00002000) ì6CAN_F12R1_FB14 ((uint32_t)0x00004000) î6CAN_F12R1_FB15 ((uint32_t)0x00008000) ï6CAN_F12R1_FB16 ((uint32_t)0x00010000) ñ6CAN_F12R1_FB17 ((uint32_t)0x00020000) ó6CAN_F12R1_FB18 ((uint32_t)0x00040000) ò6CAN_F12R1_FB19 ((uint32_t)0x00080000) ô6CAN_F12R1_FB20 ((uint32_t)0x00100000) ö6CAN_F12R1_FB21 ((uint32_t)0x00200000) õ6CAN_F12R1_FB22 ((uint32_t)0x00400000) ú6CAN_F12R1_FB23 ((uint32_t)0x00800000) ù6CAN_F12R1_FB24 ((uint32_t)0x01000000) û6CAN_F12R1_FB25 ((uint32_t)0x02000000) ü6CAN_F12R1_FB26 ((uint32_t)0x04000000) †6CAN_F12R1_FB27 ((uint32_t)0x08000000) °6CAN_F12R1_FB28 ((uint32_t)0x10000000) ¢6CAN_F12R1_FB29 ((uint32_t)0x20000000) £6CAN_F12R1_FB30 ((uint32_t)0x40000000) §6CAN_F12R1_FB31 ((uint32_t)0x80000000) ß6CAN_F13R1_FB0 ((uint32_t)0x00000001) ®6CAN_F13R1_FB1 ((uint32_t)0x00000002) ©6CAN_F13R1_FB2 ((uint32_t)0x00000004) ™6CAN_F13R1_FB3 ((uint32_t)0x00000008) ´6CAN_F13R1_FB4 ((uint32_t)0x00000010) ¨6CAN_F13R1_FB5 ((uint32_t)0x00000020) ≠6CAN_F13R1_FB6 ((uint32_t)0x00000040) Æ6CAN_F13R1_FB7 ((uint32_t)0x00000080) Ø6CAN_F13R1_FB8 ((uint32_t)0x00000100) ∞6CAN_F13R1_FB9 ((uint32_t)0x00000200) ±6CAN_F13R1_FB10 ((uint32_t)0x00000400) ≤6CAN_F13R1_FB11 ((uint32_t)0x00000800) ≥6CAN_F13R1_FB12 ((uint32_t)0x00001000) ¥6CAN_F13R1_FB13 ((uint32_t)0x00002000) µ6CAN_F13R1_FB14 ((uint32_t)0x00004000) ∂6CAN_F13R1_FB15 ((uint32_t)0x00008000) ∑6CAN_F13R1_FB16 ((uint32_t)0x00010000) ∏6CAN_F13R1_FB17 ((uint32_t)0x00020000) π6CAN_F13R1_FB18 ((uint32_t)0x00040000) ∫6CAN_F13R1_FB19 ((uint32_t)0x00080000) ª6CAN_F13R1_FB20 ((uint32_t)0x00100000) º6CAN_F13R1_FB21 ((uint32_t)0x00200000) Ω6CAN_F13R1_FB22 ((uint32_t)0x00400000) æ6CAN_F13R1_FB23 ((uint32_t)0x00800000) ø6CAN_F13R1_FB24 ((uint32_t)0x01000000) ¿6CAN_F13R1_FB25 ((uint32_t)0x02000000) ¡6CAN_F13R1_FB26 ((uint32_t)0x04000000) ¬6CAN_F13R1_FB27 ((uint32_t)0x08000000) √6CAN_F13R1_FB28 ((uint32_t)0x10000000) ƒ6CAN_F13R1_FB29 ((uint32_t)0x20000000) ≈6CAN_F13R1_FB30 ((uint32_t)0x40000000) ∆6CAN_F13R1_FB31 ((uint32_t)0x80000000) …6CAN_F0R2_FB0 ((uint32_t)0x00000001)  6CAN_F0R2_FB1 ((uint32_t)0x00000002) À6CAN_F0R2_FB2 ((uint32_t)0x00000004) Ã6CAN_F0R2_FB3 ((uint32_t)0x00000008) Õ6CAN_F0R2_FB4 ((uint32_t)0x00000010) Œ6CAN_F0R2_FB5 ((uint32_t)0x00000020) œ6CAN_F0R2_FB6 ((uint32_t)0x00000040) –6CAN_F0R2_FB7 ((uint32_t)0x00000080) —6CAN_F0R2_FB8 ((uint32_t)0x00000100) “6CAN_F0R2_FB9 ((uint32_t)0x00000200) ”6CAN_F0R2_FB10 ((uint32_t)0x00000400) ‘6CAN_F0R2_FB11 ((uint32_t)0x00000800) ’6CAN_F0R2_FB12 ((uint32_t)0x00001000) ÷6CAN_F0R2_FB13 ((uint32_t)0x00002000) ◊6CAN_F0R2_FB14 ((uint32_t)0x00004000) ÿ6CAN_F0R2_FB15 ((uint32_t)0x00008000) Ÿ6CAN_F0R2_FB16 ((uint32_t)0x00010000) ⁄6CAN_F0R2_FB17 ((uint32_t)0x00020000) €6CAN_F0R2_FB18 ((uint32_t)0x00040000) ‹6CAN_F0R2_FB19 ((uint32_t)0x00080000) ›6CAN_F0R2_FB20 ((uint32_t)0x00100000) ﬁ6CAN_F0R2_FB21 ((uint32_t)0x00200000) ﬂ6CAN_F0R2_FB22 ((uint32_t)0x00400000) ‡6CAN_F0R2_FB23 ((uint32_t)0x00800000) ·6CAN_F0R2_FB24 ((uint32_t)0x01000000) ‚6CAN_F0R2_FB25 ((uint32_t)0x02000000) „6CAN_F0R2_FB26 ((uint32_t)0x04000000) ‰6CAN_F0R2_FB27 ((uint32_t)0x08000000) Â6CAN_F0R2_FB28 ((uint32_t)0x10000000) Ê6CAN_F0R2_FB29 ((uint32_t)0x20000000) Á6CAN_F0R2_FB30 ((uint32_t)0x40000000) Ë6CAN_F0R2_FB31 ((uint32_t)0x80000000) Î6CAN_F1R2_FB0 ((uint32_t)0x00000001) Ï6CAN_F1R2_FB1 ((uint32_t)0x00000002) Ì6CAN_F1R2_FB2 ((uint32_t)0x00000004) Ó6CAN_F1R2_FB3 ((uint32_t)0x00000008) Ô6CAN_F1R2_FB4 ((uint32_t)0x00000010) 6CAN_F1R2_FB5 ((uint32_t)0x00000020) Ò6CAN_F1R2_FB6 ((uint32_t)0x00000040) Ú6CAN_F1R2_FB7 ((uint32_t)0x00000080) Û6CAN_F1R2_FB8 ((uint32_t)0x00000100) Ù6CAN_F1R2_FB9 ((uint32_t)0x00000200) ı6CAN_F1R2_FB10 ((uint32_t)0x00000400) ˆ6CAN_F1R2_FB11 ((uint32_t)0x00000800) ˜6CAN_F1R2_FB12 ((uint32_t)0x00001000) ¯6CAN_F1R2_FB13 ((uint32_t)0x00002000) ˘6CAN_F1R2_FB14 ((uint32_t)0x00004000) ˙6CAN_F1R2_FB15 ((uint32_t)0x00008000) ˚6CAN_F1R2_FB16 ((uint32_t)0x00010000) ¸6CAN_F1R2_FB17 ((uint32_t)0x00020000) ˝6CAN_F1R2_FB18 ((uint32_t)0x00040000) ˛6CAN_F1R2_FB19 ((uint32_t)0x00080000) ˇ6CAN_F1R2_FB20 ((uint32_t)0x00100000) Ä7CAN_F1R2_FB21 ((uint32_t)0x00200000) Å7CAN_F1R2_FB22 ((uint32_t)0x00400000) Ç7CAN_F1R2_FB23 ((uint32_t)0x00800000) É7CAN_F1R2_FB24 ((uint32_t)0x01000000) Ñ7CAN_F1R2_FB25 ((uint32_t)0x02000000) Ö7CAN_F1R2_FB26 ((uint32_t)0x04000000) Ü7CAN_F1R2_FB27 ((uint32_t)0x08000000) á7CAN_F1R2_FB28 ((uint32_t)0x10000000) à7CAN_F1R2_FB29 ((uint32_t)0x20000000) â7CAN_F1R2_FB30 ((uint32_t)0x40000000) ä7CAN_F1R2_FB31 ((uint32_t)0x80000000) ç7CAN_F2R2_FB0 ((uint32_t)0x00000001) é7CAN_F2R2_FB1 ((uint32_t)0x00000002) è7CAN_F2R2_FB2 ((uint32_t)0x00000004) ê7CAN_F2R2_FB3 ((uint32_t)0x00000008) ë7CAN_F2R2_FB4 ((uint32_t)0x00000010) í7CAN_F2R2_FB5 ((uint32_t)0x00000020) ì7CAN_F2R2_FB6 ((uint32_t)0x00000040) î7CAN_F2R2_FB7 ((uint32_t)0x00000080) ï7CAN_F2R2_FB8 ((uint32_t)0x00000100) ñ7CAN_F2R2_FB9 ((uint32_t)0x00000200) ó7CAN_F2R2_FB10 ((uint32_t)0x00000400) ò7CAN_F2R2_FB11 ((uint32_t)0x00000800) ô7CAN_F2R2_FB12 ((uint32_t)0x00001000) ö7CAN_F2R2_FB13 ((uint32_t)0x00002000) õ7CAN_F2R2_FB14 ((uint32_t)0x00004000) ú7CAN_F2R2_FB15 ((uint32_t)0x00008000) ù7CAN_F2R2_FB16 ((uint32_t)0x00010000) û7CAN_F2R2_FB17 ((uint32_t)0x00020000) ü7CAN_F2R2_FB18 ((uint32_t)0x00040000) †7CAN_F2R2_FB19 ((uint32_t)0x00080000) °7CAN_F2R2_FB20 ((uint32_t)0x00100000) ¢7CAN_F2R2_FB21 ((uint32_t)0x00200000) £7CAN_F2R2_FB22 ((uint32_t)0x00400000) §7CAN_F2R2_FB23 ((uint32_t)0x00800000) •7CAN_F2R2_FB24 ((uint32_t)0x01000000) ¶7CAN_F2R2_FB25 ((uint32_t)0x02000000) ß7CAN_F2R2_FB26 ((uint32_t)0x04000000) ®7CAN_F2R2_FB27 ((uint32_t)0x08000000) ©7CAN_F2R2_FB28 ((uint32_t)0x10000000) ™7CAN_F2R2_FB29 ((uint32_t)0x20000000) ´7CAN_F2R2_FB30 ((uint32_t)0x40000000) ¨7CAN_F2R2_FB31 ((uint32_t)0x80000000) Ø7CAN_F3R2_FB0 ((uint32_t)0x00000001) ∞7CAN_F3R2_FB1 ((uint32_t)0x00000002) ±7CAN_F3R2_FB2 ((uint32_t)0x00000004) ≤7CAN_F3R2_FB3 ((uint32_t)0x00000008) ≥7CAN_F3R2_FB4 ((uint32_t)0x00000010) ¥7CAN_F3R2_FB5 ((uint32_t)0x00000020) µ7CAN_F3R2_FB6 ((uint32_t)0x00000040) ∂7CAN_F3R2_FB7 ((uint32_t)0x00000080) ∑7CAN_F3R2_FB8 ((uint32_t)0x00000100) ∏7CAN_F3R2_FB9 ((uint32_t)0x00000200) π7CAN_F3R2_FB10 ((uint32_t)0x00000400) ∫7CAN_F3R2_FB11 ((uint32_t)0x00000800) ª7CAN_F3R2_FB12 ((uint32_t)0x00001000) º7CAN_F3R2_FB13 ((uint32_t)0x00002000) Ω7CAN_F3R2_FB14 ((uint32_t)0x00004000) æ7CAN_F3R2_FB15 ((uint32_t)0x00008000) ø7CAN_F3R2_FB16 ((uint32_t)0x00010000) ¿7CAN_F3R2_FB17 ((uint32_t)0x00020000) ¡7CAN_F3R2_FB18 ((uint32_t)0x00040000) ¬7CAN_F3R2_FB19 ((uint32_t)0x00080000) √7CAN_F3R2_FB20 ((uint32_t)0x00100000) ƒ7CAN_F3R2_FB21 ((uint32_t)0x00200000) ≈7CAN_F3R2_FB22 ((uint32_t)0x00400000) ∆7CAN_F3R2_FB23 ((uint32_t)0x00800000) «7CAN_F3R2_FB24 ((uint32_t)0x01000000) »7CAN_F3R2_FB25 ((uint32_t)0x02000000) …7CAN_F3R2_FB26 ((uint32_t)0x04000000)  7CAN_F3R2_FB27 ((uint32_t)0x08000000) À7CAN_F3R2_FB28 ((uint32_t)0x10000000) Ã7CAN_F3R2_FB29 ((uint32_t)0x20000000) Õ7CAN_F3R2_FB30 ((uint32_t)0x40000000) Œ7CAN_F3R2_FB31 ((uint32_t)0x80000000) —7CAN_F4R2_FB0 ((uint32_t)0x00000001) “7CAN_F4R2_FB1 ((uint32_t)0x00000002) ”7CAN_F4R2_FB2 ((uint32_t)0x00000004) ‘7CAN_F4R2_FB3 ((uint32_t)0x00000008) ’7CAN_F4R2_FB4 ((uint32_t)0x00000010) ÷7CAN_F4R2_FB5 ((uint32_t)0x00000020) ◊7CAN_F4R2_FB6 ((uint32_t)0x00000040) ÿ7CAN_F4R2_FB7 ((uint32_t)0x00000080) Ÿ7CAN_F4R2_FB8 ((uint32_t)0x00000100) ⁄7CAN_F4R2_FB9 ((uint32_t)0x00000200) €7CAN_F4R2_FB10 ((uint32_t)0x00000400) ‹7CAN_F4R2_FB11 ((uint32_t)0x00000800) ›7CAN_F4R2_FB12 ((uint32_t)0x00001000) ﬁ7CAN_F4R2_FB13 ((uint32_t)0x00002000) ﬂ7CAN_F4R2_FB14 ((uint32_t)0x00004000) ‡7CAN_F4R2_FB15 ((uint32_t)0x00008000) ·7CAN_F4R2_FB16 ((uint32_t)0x00010000) ‚7CAN_F4R2_FB17 ((uint32_t)0x00020000) „7CAN_F4R2_FB18 ((uint32_t)0x00040000) ‰7CAN_F4R2_FB19 ((uint32_t)0x00080000) Â7CAN_F4R2_FB20 ((uint32_t)0x00100000) Ê7CAN_F4R2_FB21 ((uint32_t)0x00200000) Á7CAN_F4R2_FB22 ((uint32_t)0x00400000) Ë7CAN_F4R2_FB23 ((uint32_t)0x00800000) È7CAN_F4R2_FB24 ((uint32_t)0x01000000) Í7CAN_F4R2_FB25 ((uint32_t)0x02000000) Î7CAN_F4R2_FB26 ((uint32_t)0x04000000) Ï7CAN_F4R2_FB27 ((uint32_t)0x08000000) Ì7CAN_F4R2_FB28 ((uint32_t)0x10000000) Ó7CAN_F4R2_FB29 ((uint32_t)0x20000000) Ô7CAN_F4R2_FB30 ((uint32_t)0x40000000) 7CAN_F4R2_FB31 ((uint32_t)0x80000000) Û7CAN_F5R2_FB0 ((uint32_t)0x00000001) Ù7CAN_F5R2_FB1 ((uint32_t)0x00000002) ı7CAN_F5R2_FB2 ((uint32_t)0x00000004) ˆ7CAN_F5R2_FB3 ((uint32_t)0x00000008) ˜7CAN_F5R2_FB4 ((uint32_t)0x00000010) ¯7CAN_F5R2_FB5 ((uint32_t)0x00000020) ˘7CAN_F5R2_FB6 ((uint32_t)0x00000040) ˙7CAN_F5R2_FB7 ((uint32_t)0x00000080) ˚7CAN_F5R2_FB8 ((uint32_t)0x00000100) ¸7CAN_F5R2_FB9 ((uint32_t)0x00000200) ˝7CAN_F5R2_FB10 ((uint32_t)0x00000400) ˛7CAN_F5R2_FB11 ((uint32_t)0x00000800) ˇ7CAN_F5R2_FB12 ((uint32_t)0x00001000) Ä8CAN_F5R2_FB13 ((uint32_t)0x00002000) Å8CAN_F5R2_FB14 ((uint32_t)0x00004000) Ç8CAN_F5R2_FB15 ((uint32_t)0x00008000) É8CAN_F5R2_FB16 ((uint32_t)0x00010000) Ñ8CAN_F5R2_FB17 ((uint32_t)0x00020000) Ö8CAN_F5R2_FB18 ((uint32_t)0x00040000) Ü8CAN_F5R2_FB19 ((uint32_t)0x00080000) á8CAN_F5R2_FB20 ((uint32_t)0x00100000) à8CAN_F5R2_FB21 ((uint32_t)0x00200000) â8CAN_F5R2_FB22 ((uint32_t)0x00400000) ä8CAN_F5R2_FB23 ((uint32_t)0x00800000) ã8CAN_F5R2_FB24 ((uint32_t)0x01000000) å8CAN_F5R2_FB25 ((uint32_t)0x02000000) ç8CAN_F5R2_FB26 ((uint32_t)0x04000000) é8CAN_F5R2_FB27 ((uint32_t)0x08000000) è8CAN_F5R2_FB28 ((uint32_t)0x10000000) ê8CAN_F5R2_FB29 ((uint32_t)0x20000000) ë8CAN_F5R2_FB30 ((uint32_t)0x40000000) í8CAN_F5R2_FB31 ((uint32_t)0x80000000) ï8CAN_F6R2_FB0 ((uint32_t)0x00000001) ñ8CAN_F6R2_FB1 ((uint32_t)0x00000002) ó8CAN_F6R2_FB2 ((uint32_t)0x00000004) ò8CAN_F6R2_FB3 ((uint32_t)0x00000008) ô8CAN_F6R2_FB4 ((uint32_t)0x00000010) ö8CAN_F6R2_FB5 ((uint32_t)0x00000020) õ8CAN_F6R2_FB6 ((uint32_t)0x00000040) ú8CAN_F6R2_FB7 ((uint32_t)0x00000080) ù8CAN_F6R2_FB8 ((uint32_t)0x00000100) û8CAN_F6R2_FB9 ((uint32_t)0x00000200) ü8CAN_F6R2_FB10 ((uint32_t)0x00000400) †8CAN_F6R2_FB11 ((uint32_t)0x00000800) °8CAN_F6R2_FB12 ((uint32_t)0x00001000) ¢8CAN_F6R2_FB13 ((uint32_t)0x00002000) £8CAN_F6R2_FB14 ((uint32_t)0x00004000) §8CAN_F6R2_FB15 ((uint32_t)0x00008000) •8CAN_F6R2_FB16 ((uint32_t)0x00010000) ¶8CAN_F6R2_FB17 ((uint32_t)0x00020000) ß8CAN_F6R2_FB18 ((uint32_t)0x00040000) ®8CAN_F6R2_FB19 ((uint32_t)0x00080000) ©8CAN_F6R2_FB20 ((uint32_t)0x00100000) ™8CAN_F6R2_FB21 ((uint32_t)0x00200000) ´8CAN_F6R2_FB22 ((uint32_t)0x00400000) ¨8CAN_F6R2_FB23 ((uint32_t)0x00800000) ≠8CAN_F6R2_FB24 ((uint32_t)0x01000000) Æ8CAN_F6R2_FB25 ((uint32_t)0x02000000) Ø8CAN_F6R2_FB26 ((uint32_t)0x04000000) ∞8CAN_F6R2_FB27 ((uint32_t)0x08000000) ±8CAN_F6R2_FB28 ((uint32_t)0x10000000) ≤8CAN_F6R2_FB29 ((uint32_t)0x20000000) ≥8CAN_F6R2_FB30 ((uint32_t)0x40000000) ¥8CAN_F6R2_FB31 ((uint32_t)0x80000000) ∑8CAN_F7R2_FB0 ((uint32_t)0x00000001) ∏8CAN_F7R2_FB1 ((uint32_t)0x00000002) π8CAN_F7R2_FB2 ((uint32_t)0x00000004) ∫8CAN_F7R2_FB3 ((uint32_t)0x00000008) ª8CAN_F7R2_FB4 ((uint32_t)0x00000010) º8CAN_F7R2_FB5 ((uint32_t)0x00000020) Ω8CAN_F7R2_FB6 ((uint32_t)0x00000040) æ8CAN_F7R2_FB7 ((uint32_t)0x00000080) ø8CAN_F7R2_FB8 ((uint32_t)0x00000100) ¿8CAN_F7R2_FB9 ((uint32_t)0x00000200) ¡8CAN_F7R2_FB10 ((uint32_t)0x00000400) ¬8CAN_F7R2_FB11 ((uint32_t)0x00000800) √8CAN_F7R2_FB12 ((uint32_t)0x00001000) ƒ8CAN_F7R2_FB13 ((uint32_t)0x00002000) ≈8CAN_F7R2_FB14 ((uint32_t)0x00004000) ∆8CAN_F7R2_FB15 ((uint32_t)0x00008000) «8CAN_F7R2_FB16 ((uint32_t)0x00010000) »8CAN_F7R2_FB17 ((uint32_t)0x00020000) …8CAN_F7R2_FB18 ((uint32_t)0x00040000)  8CAN_F7R2_FB19 ((uint32_t)0x00080000) À8CAN_F7R2_FB20 ((uint32_t)0x00100000) Ã8CAN_F7R2_FB21 ((uint32_t)0x00200000) Õ8CAN_F7R2_FB22 ((uint32_t)0x00400000) Œ8CAN_F7R2_FB23 ((uint32_t)0x00800000) œ8CAN_F7R2_FB24 ((uint32_t)0x01000000) –8CAN_F7R2_FB25 ((uint32_t)0x02000000) —8CAN_F7R2_FB26 ((uint32_t)0x04000000) “8CAN_F7R2_FB27 ((uint32_t)0x08000000) ”8CAN_F7R2_FB28 ((uint32_t)0x10000000) ‘8CAN_F7R2_FB29 ((uint32_t)0x20000000) ’8CAN_F7R2_FB30 ((uint32_t)0x40000000) ÷8CAN_F7R2_FB31 ((uint32_t)0x80000000) Ÿ8CAN_F8R2_FB0 ((uint32_t)0x00000001) ⁄8CAN_F8R2_FB1 ((uint32_t)0x00000002) €8CAN_F8R2_FB2 ((uint32_t)0x00000004) ‹8CAN_F8R2_FB3 ((uint32_t)0x00000008) ›8CAN_F8R2_FB4 ((uint32_t)0x00000010) ﬁ8CAN_F8R2_FB5 ((uint32_t)0x00000020) ﬂ8CAN_F8R2_FB6 ((uint32_t)0x00000040) ‡8CAN_F8R2_FB7 ((uint32_t)0x00000080) ·8CAN_F8R2_FB8 ((uint32_t)0x00000100) ‚8CAN_F8R2_FB9 ((uint32_t)0x00000200) „8CAN_F8R2_FB10 ((uint32_t)0x00000400) ‰8CAN_F8R2_FB11 ((uint32_t)0x00000800) Â8CAN_F8R2_FB12 ((uint32_t)0x00001000) Ê8CAN_F8R2_FB13 ((uint32_t)0x00002000) Á8CAN_F8R2_FB14 ((uint32_t)0x00004000) Ë8CAN_F8R2_FB15 ((uint32_t)0x00008000) È8CAN_F8R2_FB16 ((uint32_t)0x00010000) Í8CAN_F8R2_FB17 ((uint32_t)0x00020000) Î8CAN_F8R2_FB18 ((uint32_t)0x00040000) Ï8CAN_F8R2_FB19 ((uint32_t)0x00080000) Ì8CAN_F8R2_FB20 ((uint32_t)0x00100000) Ó8CAN_F8R2_FB21 ((uint32_t)0x00200000) Ô8CAN_F8R2_FB22 ((uint32_t)0x00400000) 8CAN_F8R2_FB23 ((uint32_t)0x00800000) Ò8CAN_F8R2_FB24 ((uint32_t)0x01000000) Ú8CAN_F8R2_FB25 ((uint32_t)0x02000000) Û8CAN_F8R2_FB26 ((uint32_t)0x04000000) Ù8CAN_F8R2_FB27 ((uint32_t)0x08000000) ı8CAN_F8R2_FB28 ((uint32_t)0x10000000) ˆ8CAN_F8R2_FB29 ((uint32_t)0x20000000) ˜8CAN_F8R2_FB30 ((uint32_t)0x40000000) ¯8CAN_F8R2_FB31 ((uint32_t)0x80000000) ˚8CAN_F9R2_FB0 ((uint32_t)0x00000001) ¸8CAN_F9R2_FB1 ((uint32_t)0x00000002) ˝8CAN_F9R2_FB2 ((uint32_t)0x00000004) ˛8CAN_F9R2_FB3 ((uint32_t)0x00000008) ˇ8CAN_F9R2_FB4 ((uint32_t)0x00000010) Ä9CAN_F9R2_FB5 ((uint32_t)0x00000020) Å9CAN_F9R2_FB6 ((uint32_t)0x00000040) Ç9CAN_F9R2_FB7 ((uint32_t)0x00000080) É9CAN_F9R2_FB8 ((uint32_t)0x00000100) Ñ9CAN_F9R2_FB9 ((uint32_t)0x00000200) Ö9CAN_F9R2_FB10 ((uint32_t)0x00000400) Ü9CAN_F9R2_FB11 ((uint32_t)0x00000800) á9CAN_F9R2_FB12 ((uint32_t)0x00001000) à9CAN_F9R2_FB13 ((uint32_t)0x00002000) â9CAN_F9R2_FB14 ((uint32_t)0x00004000) ä9CAN_F9R2_FB15 ((uint32_t)0x00008000) ã9CAN_F9R2_FB16 ((uint32_t)0x00010000) å9CAN_F9R2_FB17 ((uint32_t)0x00020000) ç9CAN_F9R2_FB18 ((uint32_t)0x00040000) é9CAN_F9R2_FB19 ((uint32_t)0x00080000) è9CAN_F9R2_FB20 ((uint32_t)0x00100000) ê9CAN_F9R2_FB21 ((uint32_t)0x00200000) ë9CAN_F9R2_FB22 ((uint32_t)0x00400000) í9CAN_F9R2_FB23 ((uint32_t)0x00800000) ì9CAN_F9R2_FB24 ((uint32_t)0x01000000) î9CAN_F9R2_FB25 ((uint32_t)0x02000000) ï9CAN_F9R2_FB26 ((uint32_t)0x04000000) ñ9CAN_F9R2_FB27 ((uint32_t)0x08000000) ó9CAN_F9R2_FB28 ((uint32_t)0x10000000) ò9CAN_F9R2_FB29 ((uint32_t)0x20000000) ô9CAN_F9R2_FB30 ((uint32_t)0x40000000) ö9CAN_F9R2_FB31 ((uint32_t)0x80000000) ù9CAN_F10R2_FB0 ((uint32_t)0x00000001) û9CAN_F10R2_FB1 ((uint32_t)0x00000002) ü9CAN_F10R2_FB2 ((uint32_t)0x00000004) †9CAN_F10R2_FB3 ((uint32_t)0x00000008) °9CAN_F10R2_FB4 ((uint32_t)0x00000010) ¢9CAN_F10R2_FB5 ((uint32_t)0x00000020) £9CAN_F10R2_FB6 ((uint32_t)0x00000040) §9CAN_F10R2_FB7 ((uint32_t)0x00000080) •9CAN_F10R2_FB8 ((uint32_t)0x00000100) ¶9CAN_F10R2_FB9 ((uint32_t)0x00000200) ß9CAN_F10R2_FB10 ((uint32_t)0x00000400) ®9CAN_F10R2_FB11 ((uint32_t)0x00000800) ©9CAN_F10R2_FB12 ((uint32_t)0x00001000) ™9CAN_F10R2_FB13 ((uint32_t)0x00002000) ´9CAN_F10R2_FB14 ((uint32_t)0x00004000) ¨9CAN_F10R2_FB15 ((uint32_t)0x00008000) ≠9CAN_F10R2_FB16 ((uint32_t)0x00010000) Æ9CAN_F10R2_FB17 ((uint32_t)0x00020000) Ø9CAN_F10R2_FB18 ((uint32_t)0x00040000) ∞9CAN_F10R2_FB19 ((uint32_t)0x00080000) ±9CAN_F10R2_FB20 ((uint32_t)0x00100000) ≤9CAN_F10R2_FB21 ((uint32_t)0x00200000) ≥9CAN_F10R2_FB22 ((uint32_t)0x00400000) ¥9CAN_F10R2_FB23 ((uint32_t)0x00800000) µ9CAN_F10R2_FB24 ((uint32_t)0x01000000) ∂9CAN_F10R2_FB25 ((uint32_t)0x02000000) ∑9CAN_F10R2_FB26 ((uint32_t)0x04000000) ∏9CAN_F10R2_FB27 ((uint32_t)0x08000000) π9CAN_F10R2_FB28 ((uint32_t)0x10000000) ∫9CAN_F10R2_FB29 ((uint32_t)0x20000000) ª9CAN_F10R2_FB30 ((uint32_t)0x40000000) º9CAN_F10R2_FB31 ((uint32_t)0x80000000) ø9CAN_F11R2_FB0 ((uint32_t)0x00000001) ¿9CAN_F11R2_FB1 ((uint32_t)0x00000002) ¡9CAN_F11R2_FB2 ((uint32_t)0x00000004) ¬9CAN_F11R2_FB3 ((uint32_t)0x00000008) √9CAN_F11R2_FB4 ((uint32_t)0x00000010) ƒ9CAN_F11R2_FB5 ((uint32_t)0x00000020) ≈9CAN_F11R2_FB6 ((uint32_t)0x00000040) ∆9CAN_F11R2_FB7 ((uint32_t)0x00000080) «9CAN_F11R2_FB8 ((uint32_t)0x00000100) »9CAN_F11R2_FB9 ((uint32_t)0x00000200) …9CAN_F11R2_FB10 ((uint32_t)0x00000400)  9CAN_F11R2_FB11 ((uint32_t)0x00000800) À9CAN_F11R2_FB12 ((uint32_t)0x00001000) Ã9CAN_F11R2_FB13 ((uint32_t)0x00002000) Õ9CAN_F11R2_FB14 ((uint32_t)0x00004000) Œ9CAN_F11R2_FB15 ((uint32_t)0x00008000) œ9CAN_F11R2_FB16 ((uint32_t)0x00010000) –9CAN_F11R2_FB17 ((uint32_t)0x00020000) —9CAN_F11R2_FB18 ((uint32_t)0x00040000) “9CAN_F11R2_FB19 ((uint32_t)0x00080000) ”9CAN_F11R2_FB20 ((uint32_t)0x00100000) ‘9CAN_F11R2_FB21 ((uint32_t)0x00200000) ’9CAN_F11R2_FB22 ((uint32_t)0x00400000) ÷9CAN_F11R2_FB23 ((uint32_t)0x00800000) ◊9CAN_F11R2_FB24 ((uint32_t)0x01000000) ÿ9CAN_F11R2_FB25 ((uint32_t)0x02000000) Ÿ9CAN_F11R2_FB26 ((uint32_t)0x04000000) ⁄9CAN_F11R2_FB27 ((uint32_t)0x08000000) €9CAN_F11R2_FB28 ((uint32_t)0x10000000) ‹9CAN_F11R2_FB29 ((uint32_t)0x20000000) ›9CAN_F11R2_FB30 ((uint32_t)0x40000000) ﬁ9CAN_F11R2_FB31 ((uint32_t)0x80000000) ·9CAN_F12R2_FB0 ((uint32_t)0x00000001) ‚9CAN_F12R2_FB1 ((uint32_t)0x00000002) „9CAN_F12R2_FB2 ((uint32_t)0x00000004) ‰9CAN_F12R2_FB3 ((uint32_t)0x00000008) Â9CAN_F12R2_FB4 ((uint32_t)0x00000010) Ê9CAN_F12R2_FB5 ((uint32_t)0x00000020) Á9CAN_F12R2_FB6 ((uint32_t)0x00000040) Ë9CAN_F12R2_FB7 ((uint32_t)0x00000080) È9CAN_F12R2_FB8 ((uint32_t)0x00000100) Í9CAN_F12R2_FB9 ((uint32_t)0x00000200) Î9CAN_F12R2_FB10 ((uint32_t)0x00000400) Ï9CAN_F12R2_FB11 ((uint32_t)0x00000800) Ì9CAN_F12R2_FB12 ((uint32_t)0x00001000) Ó9CAN_F12R2_FB13 ((uint32_t)0x00002000) Ô9CAN_F12R2_FB14 ((uint32_t)0x00004000) 9CAN_F12R2_FB15 ((uint32_t)0x00008000) Ò9CAN_F12R2_FB16 ((uint32_t)0x00010000) Ú9CAN_F12R2_FB17 ((uint32_t)0x00020000) Û9CAN_F12R2_FB18 ((uint32_t)0x00040000) Ù9CAN_F12R2_FB19 ((uint32_t)0x00080000) ı9CAN_F12R2_FB20 ((uint32_t)0x00100000) ˆ9CAN_F12R2_FB21 ((uint32_t)0x00200000) ˜9CAN_F12R2_FB22 ((uint32_t)0x00400000) ¯9CAN_F12R2_FB23 ((uint32_t)0x00800000) ˘9CAN_F12R2_FB24 ((uint32_t)0x01000000) ˙9CAN_F12R2_FB25 ((uint32_t)0x02000000) ˚9CAN_F12R2_FB26 ((uint32_t)0x04000000) ¸9CAN_F12R2_FB27 ((uint32_t)0x08000000) ˝9CAN_F12R2_FB28 ((uint32_t)0x10000000) ˛9CAN_F12R2_FB29 ((uint32_t)0x20000000) ˇ9CAN_F12R2_FB30 ((uint32_t)0x40000000) Ä:CAN_F12R2_FB31 ((uint32_t)0x80000000) É:CAN_F13R2_FB0 ((uint32_t)0x00000001) Ñ:CAN_F13R2_FB1 ((uint32_t)0x00000002) Ö:CAN_F13R2_FB2 ((uint32_t)0x00000004) Ü:CAN_F13R2_FB3 ((uint32_t)0x00000008) á:CAN_F13R2_FB4 ((uint32_t)0x00000010) à:CAN_F13R2_FB5 ((uint32_t)0x00000020) â:CAN_F13R2_FB6 ((uint32_t)0x00000040) ä:CAN_F13R2_FB7 ((uint32_t)0x00000080) ã:CAN_F13R2_FB8 ((uint32_t)0x00000100) å:CAN_F13R2_FB9 ((uint32_t)0x00000200) ç:CAN_F13R2_FB10 ((uint32_t)0x00000400) é:CAN_F13R2_FB11 ((uint32_t)0x00000800) è:CAN_F13R2_FB12 ((uint32_t)0x00001000) ê:CAN_F13R2_FB13 ((uint32_t)0x00002000) ë:CAN_F13R2_FB14 ((uint32_t)0x00004000) í:CAN_F13R2_FB15 ((uint32_t)0x00008000) ì:CAN_F13R2_FB16 ((uint32_t)0x00010000) î:CAN_F13R2_FB17 ((uint32_t)0x00020000) ï:CAN_F13R2_FB18 ((uint32_t)0x00040000) ñ:CAN_F13R2_FB19 ((uint32_t)0x00080000) ó:CAN_F13R2_FB20 ((uint32_t)0x00100000) ò:CAN_F13R2_FB21 ((uint32_t)0x00200000) ô:CAN_F13R2_FB22 ((uint32_t)0x00400000) ö:CAN_F13R2_FB23 ((uint32_t)0x00800000) õ:CAN_F13R2_FB24 ((uint32_t)0x01000000) ú:CAN_F13R2_FB25 ((uint32_t)0x02000000) ù:CAN_F13R2_FB26 ((uint32_t)0x04000000) û:CAN_F13R2_FB27 ((uint32_t)0x08000000) ü:CAN_F13R2_FB28 ((uint32_t)0x10000000) †:CAN_F13R2_FB29 ((uint32_t)0x20000000) °:CAN_F13R2_FB30 ((uint32_t)0x40000000) ¢:CAN_F13R2_FB31 ((uint32_t)0x80000000) ´:SPI_CR1_CPHA ((uint16_t)0x0001) ¨:SPI_CR1_CPOL ((uint16_t)0x0002) ≠:SPI_CR1_MSTR ((uint16_t)0x0004) Ø:SPI_CR1_BR ((uint16_t)0x0038) ∞:SPI_CR1_BR_0 ((uint16_t)0x0008) ±:SPI_CR1_BR_1 ((uint16_t)0x0010) ≤:SPI_CR1_BR_2 ((uint16_t)0x0020) ¥:SPI_CR1_SPE ((uint16_t)0x0040) µ:SPI_CR1_LSBFIRST ((uint16_t)0x0080) ∂:SPI_CR1_SSI ((uint16_t)0x0100) ∑:SPI_CR1_SSM ((uint16_t)0x0200) ∏:SPI_CR1_RXONLY ((uint16_t)0x0400) π:SPI_CR1_DFF ((uint16_t)0x0800) ∫:SPI_CR1_CRCNEXT ((uint16_t)0x1000) ª:SPI_CR1_CRCEN ((uint16_t)0x2000) º:SPI_CR1_BIDIOE ((uint16_t)0x4000) Ω:SPI_CR1_BIDIMODE ((uint16_t)0x8000) ¿:SPI_CR2_RXDMAEN ((uint8_t)0x01) ¡:SPI_CR2_TXDMAEN ((uint8_t)0x02) ¬:SPI_CR2_SSOE ((uint8_t)0x04) √:SPI_CR2_ERRIE ((uint8_t)0x20) ƒ:SPI_CR2_RXNEIE ((uint8_t)0x40) ≈:SPI_CR2_TXEIE ((uint8_t)0x80) »:SPI_SR_RXNE ((uint8_t)0x01) …:SPI_SR_TXE ((uint8_t)0x02)  :SPI_SR_CHSIDE ((uint8_t)0x04) À:SPI_SR_UDR ((uint8_t)0x08) Ã:SPI_SR_CRCERR ((uint8_t)0x10) Õ:SPI_SR_MODF ((uint8_t)0x20) Œ:SPI_SR_OVR ((uint8_t)0x40) œ:SPI_SR_BSY ((uint8_t)0x80) “:SPI_DR_DR ((uint16_t)0xFFFF) ’:SPI_CRCPR_CRCPOLY ((uint16_t)0xFFFF) ÿ:SPI_RXCRCR_RXCRC ((uint16_t)0xFFFF) €:SPI_TXCRCR_TXCRC ((uint16_t)0xFFFF) ﬁ:SPI_I2SCFGR_CHLEN ((uint16_t)0x0001) ‡:SPI_I2SCFGR_DATLEN ((uint16_t)0x0006) ·:SPI_I2SCFGR_DATLEN_0 ((uint16_t)0x0002) ‚:SPI_I2SCFGR_DATLEN_1 ((uint16_t)0x0004) ‰:SPI_I2SCFGR_CKPOL ((uint16_t)0x0008) Ê:SPI_I2SCFGR_I2SSTD ((uint16_t)0x0030) Á:SPI_I2SCFGR_I2SSTD_0 ((uint16_t)0x0010) Ë:SPI_I2SCFGR_I2SSTD_1 ((uint16_t)0x0020) Í:SPI_I2SCFGR_PCMSYNC ((uint16_t)0x0080) Ï:SPI_I2SCFGR_I2SCFG ((uint16_t)0x0300) Ì:SPI_I2SCFGR_I2SCFG_0 ((uint16_t)0x0100) Ó:SPI_I2SCFGR_I2SCFG_1 ((uint16_t)0x0200) :SPI_I2SCFGR_I2SE ((uint16_t)0x0400) Ò:SPI_I2SCFGR_I2SMOD ((uint16_t)0x0800) Ù:SPI_I2SPR_I2SDIV ((uint16_t)0x00FF) ı:SPI_I2SPR_ODD ((uint16_t)0x0100) ˆ:SPI_I2SPR_MCKOE ((uint16_t)0x0200) ˇ:I2C_CR1_PE ((uint16_t)0x0001) Ä;I2C_CR1_SMBUS ((uint16_t)0x0002) Å;I2C_CR1_SMBTYPE ((uint16_t)0x0008) Ç;I2C_CR1_ENARP ((uint16_t)0x0010) É;I2C_CR1_ENPEC ((uint16_t)0x0020) Ñ;I2C_CR1_ENGC ((uint16_t)0x0040) Ö;I2C_CR1_NOSTRETCH ((uint16_t)0x0080) Ü;I2C_CR1_START ((uint16_t)0x0100) á;I2C_CR1_STOP ((uint16_t)0x0200) à;I2C_CR1_ACK ((uint16_t)0x0400) â;I2C_CR1_POS ((uint16_t)0x0800) ä;I2C_CR1_PEC ((uint16_t)0x1000) ã;I2C_CR1_ALERT ((uint16_t)0x2000) å;I2C_CR1_SWRST ((uint16_t)0x8000) è;I2C_CR2_FREQ ((uint16_t)0x003F) ê;I2C_CR2_FREQ_0 ((uint16_t)0x0001) ë;I2C_CR2_FREQ_1 ((uint16_t)0x0002) í;I2C_CR2_FREQ_2 ((uint16_t)0x0004) ì;I2C_CR2_FREQ_3 ((uint16_t)0x0008) î;I2C_CR2_FREQ_4 ((uint16_t)0x0010) ï;I2C_CR2_FREQ_5 ((uint16_t)0x0020) ó;I2C_CR2_ITERREN ((uint16_t)0x0100) ò;I2C_CR2_ITEVTEN ((uint16_t)0x0200) ô;I2C_CR2_ITBUFEN ((uint16_t)0x0400) ö;I2C_CR2_DMAEN ((uint16_t)0x0800) õ;I2C_CR2_LAST ((uint16_t)0x1000) û;I2C_OAR1_ADD1_7 ((uint16_t)0x00FE) ü;I2C_OAR1_ADD8_9 ((uint16_t)0x0300) °;I2C_OAR1_ADD0 ((uint16_t)0x0001) ¢;I2C_OAR1_ADD1 ((uint16_t)0x0002) £;I2C_OAR1_ADD2 ((uint16_t)0x0004) §;I2C_OAR1_ADD3 ((uint16_t)0x0008) •;I2C_OAR1_ADD4 ((uint16_t)0x0010) ¶;I2C_OAR1_ADD5 ((uint16_t)0x0020) ß;I2C_OAR1_ADD6 ((uint16_t)0x0040) ®;I2C_OAR1_ADD7 ((uint16_t)0x0080) ©;I2C_OAR1_ADD8 ((uint16_t)0x0100) ™;I2C_OAR1_ADD9 ((uint16_t)0x0200) ¨;I2C_OAR1_ADDMODE ((uint16_t)0x8000) Ø;I2C_OAR2_ENDUAL ((uint8_t)0x01) ∞;I2C_OAR2_ADD2 ((uint8_t)0xFE) ≥;I2C_DR_DR ((uint8_t)0xFF) ∂;I2C_SR1_SB ((uint16_t)0x0001) ∑;I2C_SR1_ADDR ((uint16_t)0x0002) ∏;I2C_SR1_BTF ((uint16_t)0x0004) π;I2C_SR1_ADD10 ((uint16_t)0x0008) ∫;I2C_SR1_STOPF ((uint16_t)0x0010) ª;I2C_SR1_RXNE ((uint16_t)0x0040) º;I2C_SR1_TXE ((uint16_t)0x0080) Ω;I2C_SR1_BERR ((uint16_t)0x0100) æ;I2C_SR1_ARLO ((uint16_t)0x0200) ø;I2C_SR1_AF ((uint16_t)0x0400) ¿;I2C_SR1_OVR ((uint16_t)0x0800) ¡;I2C_SR1_PECERR ((uint16_t)0x1000) ¬;I2C_SR1_TIMEOUT ((uint16_t)0x4000) √;I2C_SR1_SMBALERT ((uint16_t)0x8000) ∆;I2C_SR2_MSL ((uint16_t)0x0001) «;I2C_SR2_BUSY ((uint16_t)0x0002) »;I2C_SR2_TRA ((uint16_t)0x0004) …;I2C_SR2_GENCALL ((uint16_t)0x0010)  ;I2C_SR2_SMBDEFAULT ((uint16_t)0x0020) À;I2C_SR2_SMBHOST ((uint16_t)0x0040) Ã;I2C_SR2_DUALF ((uint16_t)0x0080) Õ;I2C_SR2_PEC ((uint16_t)0xFF00) –;I2C_CCR_CCR ((uint16_t)0x0FFF) —;I2C_CCR_DUTY ((uint16_t)0x4000) “;I2C_CCR_FS ((uint16_t)0x8000) ’;I2C_TRISE_TRISE ((uint8_t)0x3F) ﬁ;USART_SR_PE ((uint16_t)0x0001) ﬂ;USART_SR_FE ((uint16_t)0x0002) ‡;USART_SR_NE ((uint16_t)0x0004) ·;USART_SR_ORE ((uint16_t)0x0008) ‚;USART_SR_IDLE ((uint16_t)0x0010) „;USART_SR_RXNE ((uint16_t)0x0020) ‰;USART_SR_TC ((uint16_t)0x0040) Â;USART_SR_TXE ((uint16_t)0x0080) Ê;USART_SR_LBD ((uint16_t)0x0100) Á;USART_SR_CTS ((uint16_t)0x0200) Í;USART_DR_DR ((uint16_t)0x01FF) Ì;USART_BRR_DIV_Fraction ((uint16_t)0x000F) Ó;USART_BRR_DIV_Mantissa ((uint16_t)0xFFF0) Ò;USART_CR1_SBK ((uint16_t)0x0001) Ú;USART_CR1_RWU ((uint16_t)0x0002) Û;USART_CR1_RE ((uint16_t)0x0004) Ù;USART_CR1_TE ((uint16_t)0x0008) ı;USART_CR1_IDLEIE ((uint16_t)0x0010) ˆ;USART_CR1_RXNEIE ((uint16_t)0x0020) ˜;USART_CR1_TCIE ((uint16_t)0x0040) ¯;USART_CR1_TXEIE ((uint16_t)0x0080) ˘;USART_CR1_PEIE ((uint16_t)0x0100) ˙;USART_CR1_PS ((uint16_t)0x0200) ˚;USART_CR1_PCE ((uint16_t)0x0400) ¸;USART_CR1_WAKE ((uint16_t)0x0800) ˝;USART_CR1_M ((uint16_t)0x1000) ˛;USART_CR1_UE ((uint16_t)0x2000) ˇ;USART_CR1_OVER8 ((uint16_t)0x8000) Ç<USART_CR2_ADD ((uint16_t)0x000F) É<USART_CR2_LBDL ((uint16_t)0x0020) Ñ<USART_CR2_LBDIE ((uint16_t)0x0040) Ö<USART_CR2_LBCL ((uint16_t)0x0100) Ü<USART_CR2_CPHA ((uint16_t)0x0200) á<USART_CR2_CPOL ((uint16_t)0x0400) à<USART_CR2_CLKEN ((uint16_t)0x0800) ä<USART_CR2_STOP ((uint16_t)0x3000) ã<USART_CR2_STOP_0 ((uint16_t)0x1000) å<USART_CR2_STOP_1 ((uint16_t)0x2000) é<USART_CR2_LINEN ((uint16_t)0x4000) ë<USART_CR3_EIE ((uint16_t)0x0001) í<USART_CR3_IREN ((uint16_t)0x0002) ì<USART_CR3_IRLP ((uint16_t)0x0004) î<USART_CR3_HDSEL ((uint16_t)0x0008) ï<USART_CR3_NACK ((uint16_t)0x0010) ñ<USART_CR3_SCEN ((uint16_t)0x0020) ó<USART_CR3_DMAR ((uint16_t)0x0040) ò<USART_CR3_DMAT ((uint16_t)0x0080) ô<USART_CR3_RTSE ((uint16_t)0x0100) ö<USART_CR3_CTSE ((uint16_t)0x0200) õ<USART_CR3_CTSIE ((uint16_t)0x0400) ú<USART_CR3_ONEBIT ((uint16_t)0x0800) ü<USART_GTPR_PSC ((uint16_t)0x00FF) †<USART_GTPR_PSC_0 ((uint16_t)0x0001) °<USART_GTPR_PSC_1 ((uint16_t)0x0002) ¢<USART_GTPR_PSC_2 ((uint16_t)0x0004) £<USART_GTPR_PSC_3 ((uint16_t)0x0008) §<USART_GTPR_PSC_4 ((uint16_t)0x0010) •<USART_GTPR_PSC_5 ((uint16_t)0x0020) ¶<USART_GTPR_PSC_6 ((uint16_t)0x0040) ß<USART_GTPR_PSC_7 ((uint16_t)0x0080) ©<USART_GTPR_GT ((uint16_t)0xFF00) ≤<DBGMCU_IDCODE_DEV_ID ((uint32_t)0x00000FFF) ¥<DBGMCU_IDCODE_REV_ID ((uint32_t)0xFFFF0000) µ<DBGMCU_IDCODE_REV_ID_0 ((uint32_t)0x00010000) ∂<DBGMCU_IDCODE_REV_ID_1 ((uint32_t)0x00020000) ∑<DBGMCU_IDCODE_REV_ID_2 ((uint32_t)0x00040000) ∏<DBGMCU_IDCODE_REV_ID_3 ((uint32_t)0x00080000) π<DBGMCU_IDCODE_REV_ID_4 ((uint32_t)0x00100000) ∫<DBGMCU_IDCODE_REV_ID_5 ((uint32_t)0x00200000) ª<DBGMCU_IDCODE_REV_ID_6 ((uint32_t)0x00400000) º<DBGMCU_IDCODE_REV_ID_7 ((uint32_t)0x00800000) Ω<DBGMCU_IDCODE_REV_ID_8 ((uint32_t)0x01000000) æ<DBGMCU_IDCODE_REV_ID_9 ((uint32_t)0x02000000) ø<DBGMCU_IDCODE_REV_ID_10 ((uint32_t)0x04000000) ¿<DBGMCU_IDCODE_REV_ID_11 ((uint32_t)0x08000000) ¡<DBGMCU_IDCODE_REV_ID_12 ((uint32_t)0x10000000) ¬<DBGMCU_IDCODE_REV_ID_13 ((uint32_t)0x20000000) √<DBGMCU_IDCODE_REV_ID_14 ((uint32_t)0x40000000) ƒ<DBGMCU_IDCODE_REV_ID_15 ((uint32_t)0x80000000) «<DBGMCU_CR_DBG_SLEEP ((uint32_t)0x00000001) »<DBGMCU_CR_DBG_STOP ((uint32_t)0x00000002) …<DBGMCU_CR_DBG_STANDBY ((uint32_t)0x00000004)  <DBGMCU_CR_TRACE_IOEN ((uint32_t)0x00000020) Ã<DBGMCU_CR_TRACE_MODE ((uint32_t)0x000000C0) Õ<DBGMCU_CR_TRACE_MODE_0 ((uint32_t)0x00000040) Œ<DBGMCU_CR_TRACE_MODE_1 ((uint32_t)0x00000080) –<DBGMCU_CR_DBG_IWDG_STOP ((uint32_t)0x00000100) —<DBGMCU_CR_DBG_WWDG_STOP ((uint32_t)0x00000200) “<DBGMCU_CR_DBG_TIM1_STOP ((uint32_t)0x00000400) ”<DBGMCU_CR_DBG_TIM2_STOP ((uint32_t)0x00000800) ‘<DBGMCU_CR_DBG_TIM3_STOP ((uint32_t)0x00001000) ’<DBGMCU_CR_DBG_TIM4_STOP ((uint32_t)0x00002000) ÷<DBGMCU_CR_DBG_CAN1_STOP ((uint32_t)0x00004000) ◊<DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT ((uint32_t)0x00008000) ÿ<DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT ((uint32_t)0x00010000) Ÿ<DBGMCU_CR_DBG_TIM8_STOP ((uint32_t)0x00020000) ⁄<DBGMCU_CR_DBG_TIM5_STOP ((uint32_t)0x00040000) €<DBGMCU_CR_DBG_TIM6_STOP ((uint32_t)0x00080000) ‹<DBGMCU_CR_DBG_TIM7_STOP ((uint32_t)0x00100000) ›<DBGMCU_CR_DBG_CAN2_STOP ((uint32_t)0x00200000) ﬁ<DBGMCU_CR_DBG_TIM15_STOP ((uint32_t)0x00400000) ﬂ<DBGMCU_CR_DBG_TIM16_STOP ((uint32_t)0x00800000) ‡<DBGMCU_CR_DBG_TIM17_STOP ((uint32_t)0x01000000) ·<DBGMCU_CR_DBG_TIM12_STOP ((uint32_t)0x02000000) ‚<DBGMCU_CR_DBG_TIM13_STOP ((uint32_t)0x04000000) „<DBGMCU_CR_DBG_TIM14_STOP ((uint32_t)0x08000000) ‰<DBGMCU_CR_DBG_TIM9_STOP ((uint32_t)0x10000000) Â<DBGMCU_CR_DBG_TIM10_STOP ((uint32_t)0x20000000) Ê<DBGMCU_CR_DBG_TIM11_STOP ((uint32_t)0x40000000) Ô<FLASH_ACR_LATENCY ((uint8_t)0x03) <FLASH_ACR_LATENCY_0 ((uint8_t)0x00) Ò<FLASH_ACR_LATENCY_1 ((uint8_t)0x01) Ú<FLASH_ACR_LATENCY_2 ((uint8_t)0x02) Ù<FLASH_ACR_HLFCYA ((uint8_t)0x08) ı<FLASH_ACR_PRFTBE ((uint8_t)0x10) ˆ<FLASH_ACR_PRFTBS ((uint8_t)0x20) ˘<FLASH_KEYR_FKEYR ((uint32_t)0xFFFFFFFF) ¸<FLASH_OPTKEYR_OPTKEYR ((uint32_t)0xFFFFFFFF) ˇ<FLASH_SR_BSY ((uint8_t)0x01) Ä=FLASH_SR_PGERR ((uint8_t)0x04) Å=FLASH_SR_WRPRTERR ((uint8_t)0x10) Ç=FLASH_SR_EOP ((uint8_t)0x20) Ö=FLASH_CR_PG ((uint16_t)0x0001) Ü=FLASH_CR_PER ((uint16_t)0x0002) á=FLASH_CR_MER ((uint16_t)0x0004) à=FLASH_CR_OPTPG ((uint16_t)0x0010) â=FLASH_CR_OPTER ((uint16_t)0x0020) ä=FLASH_CR_STRT ((uint16_t)0x0040) ã=FLASH_CR_LOCK ((uint16_t)0x0080) å=FLASH_CR_OPTWRE ((uint16_t)0x0200) ç=FLASH_CR_ERRIE ((uint16_t)0x0400) é=FLASH_CR_EOPIE ((uint16_t)0x1000) ë=FLASH_AR_FAR ((uint32_t)0xFFFFFFFF) î=FLASH_OBR_OPTERR ((uint16_t)0x0001) ï=FLASH_OBR_RDPRT ((uint16_t)0x0002) ó=FLASH_OBR_USER ((uint16_t)0x03FC) ò=FLASH_OBR_WDG_SW ((uint16_t)0x0004) ô=FLASH_OBR_nRST_STOP ((uint16_t)0x0008) ö=FLASH_OBR_nRST_STDBY ((uint16_t)0x0010) õ=FLASH_OBR_BFB2 ((uint16_t)0x0020) û=FLASH_WRPR_WRP ((uint32_t)0xFFFFFFFF) £=FLASH_RDP_RDP ((uint32_t)0x000000FF) §=FLASH_RDP_nRDP ((uint32_t)0x0000FF00) ß=FLASH_USER_USER ((uint32_t)0x00FF0000) ®=FLASH_USER_nUSER ((uint32_t)0xFF000000) ´=FLASH_Data0_Data0 ((uint32_t)0x000000FF) ¨=FLASH_Data0_nData0 ((uint32_t)0x0000FF00) Ø=FLASH_Data1_Data1 ((uint32_t)0x00FF0000) ∞=FLASH_Data1_nData1 ((uint32_t)0xFF000000) ≥=FLASH_WRP0_WRP0 ((uint32_t)0x000000FF) ¥=FLASH_WRP0_nWRP0 ((uint32_t)0x0000FF00) ∑=FLASH_WRP1_WRP1 ((uint32_t)0x00FF0000) ∏=FLASH_WRP1_nWRP1 ((uint32_t)0xFF000000) ª=FLASH_WRP2_WRP2 ((uint32_t)0x000000FF) º=FLASH_WRP2_nWRP2 ((uint32_t)0x0000FF00) ø=FLASH_WRP3_WRP3 ((uint32_t)0x00FF0000) ¿=FLASH_WRP3_nWRP3 ((uint32_t)0xFF000000) «=ETH_MACCR_WD ((uint32_t)0x00800000) »=ETH_MACCR_JD ((uint32_t)0x00400000) …=ETH_MACCR_IFG ((uint32_t)0x000E0000)  =ETH_MACCR_IFG_96Bit ((uint32_t)0x00000000) À=ETH_MACCR_IFG_88Bit ((uint32_t)0x00020000) Ã=ETH_MACCR_IFG_80Bit ((uint32_t)0x00040000) Õ=ETH_MACCR_IFG_72Bit ((uint32_t)0x00060000) Œ=ETH_MACCR_IFG_64Bit ((uint32_t)0x00080000) œ=ETH_MACCR_IFG_56Bit ((uint32_t)0x000A0000) –=ETH_MACCR_IFG_48Bit ((uint32_t)0x000C0000) —=ETH_MACCR_IFG_40Bit ((uint32_t)0x000E0000) “=ETH_MACCR_CSD ((uint32_t)0x00010000) ”=ETH_MACCR_FES ((uint32_t)0x00004000) ‘=ETH_MACCR_ROD ((uint32_t)0x00002000) ’=ETH_MACCR_LM ((uint32_t)0x00001000) ÷=ETH_MACCR_DM ((uint32_t)0x00000800) ◊=ETH_MACCR_IPCO ((uint32_t)0x00000400) ÿ=ETH_MACCR_RD ((uint32_t)0x00000200) Ÿ=ETH_MACCR_APCS ((uint32_t)0x00000080) ⁄=ETH_MACCR_BL ((uint32_t)0x00000060) ‹=ETH_MACCR_BL_10 ((uint32_t)0x00000000) ›=ETH_MACCR_BL_8 ((uint32_t)0x00000020) ﬁ=ETH_MACCR_BL_4 ((uint32_t)0x00000040) ﬂ=ETH_MACCR_BL_1 ((uint32_t)0x00000060) ‡=ETH_MACCR_DC ((uint32_t)0x00000010) ·=ETH_MACCR_TE ((uint32_t)0x00000008) ‚=ETH_MACCR_RE ((uint32_t)0x00000004) Â=ETH_MACFFR_RA ((uint32_t)0x80000000) Ê=ETH_MACFFR_HPF ((uint32_t)0x00000400) Á=ETH_MACFFR_SAF ((uint32_t)0x00000200) Ë=ETH_MACFFR_SAIF ((uint32_t)0x00000100) È=ETH_MACFFR_PCF ((uint32_t)0x000000C0) Í=ETH_MACFFR_PCF_BlockAll ((uint32_t)0x00000040) Î=ETH_MACFFR_PCF_ForwardAll ((uint32_t)0x00000080) Ï=ETH_MACFFR_PCF_ForwardPassedAddrFilter ((uint32_t)0x000000C0) Ì=ETH_MACFFR_BFD ((uint32_t)0x00000020) Ó=ETH_MACFFR_PAM ((uint32_t)0x00000010) Ô=ETH_MACFFR_DAIF ((uint32_t)0x00000008) =ETH_MACFFR_HM ((uint32_t)0x00000004) Ò=ETH_MACFFR_HU ((uint32_t)0x00000002) Ú=ETH_MACFFR_PM ((uint32_t)0x00000001) ı=ETH_MACHTHR_HTH ((uint32_t)0xFFFFFFFF) ¯=ETH_MACHTLR_HTL ((uint32_t)0xFFFFFFFF) ˚=ETH_MACMIIAR_PA ((uint32_t)0x0000F800) ¸=ETH_MACMIIAR_MR ((uint32_t)0x000007C0) ˝=ETH_MACMIIAR_CR ((uint32_t)0x0000001C) ˛=ETH_MACMIIAR_CR_Div42 ((uint32_t)0x00000000) ˇ=ETH_MACMIIAR_CR_Div16 ((uint32_t)0x00000008) Ä>ETH_MACMIIAR_CR_Div26 ((uint32_t)0x0000000C) Å>ETH_MACMIIAR_MW ((uint32_t)0x00000002) Ç>ETH_MACMIIAR_MB ((uint32_t)0x00000001) Ö>ETH_MACMIIDR_MD ((uint32_t)0x0000FFFF) à>ETH_MACFCR_PT ((uint32_t)0xFFFF0000) â>ETH_MACFCR_ZQPD ((uint32_t)0x00000080) ä>ETH_MACFCR_PLT ((uint32_t)0x00000030) ã>ETH_MACFCR_PLT_Minus4 ((uint32_t)0x00000000) å>ETH_MACFCR_PLT_Minus28 ((uint32_t)0x00000010) ç>ETH_MACFCR_PLT_Minus144 ((uint32_t)0x00000020) é>ETH_MACFCR_PLT_Minus256 ((uint32_t)0x00000030) è>ETH_MACFCR_UPFD ((uint32_t)0x00000008) ê>ETH_MACFCR_RFCE ((uint32_t)0x00000004) ë>ETH_MACFCR_TFCE ((uint32_t)0x00000002) í>ETH_MACFCR_FCBBPA ((uint32_t)0x00000001) ï>ETH_MACVLANTR_VLANTC ((uint32_t)0x00010000) ñ>ETH_MACVLANTR_VLANTI ((uint32_t)0x0000FFFF) ô>ETH_MACRWUFFR_D ((uint32_t)0xFFFFFFFF) ß>ETH_MACPMTCSR_WFFRPR ((uint32_t)0x80000000) ®>ETH_MACPMTCSR_GU ((uint32_t)0x00000200) ©>ETH_MACPMTCSR_WFR ((uint32_t)0x00000040) ™>ETH_MACPMTCSR_MPR ((uint32_t)0x00000020) ´>ETH_MACPMTCSR_WFE ((uint32_t)0x00000004) ¨>ETH_MACPMTCSR_MPE ((uint32_t)0x00000002) ≠>ETH_MACPMTCSR_PD ((uint32_t)0x00000001) ∞>ETH_MACSR_TSTS ((uint32_t)0x00000200) ±>ETH_MACSR_MMCTS ((uint32_t)0x00000040) ≤>ETH_MACSR_MMMCRS ((uint32_t)0x00000020) ≥>ETH_MACSR_MMCS ((uint32_t)0x00000010) ¥>ETH_MACSR_PMTS ((uint32_t)0x00000008) ∑>ETH_MACIMR_TSTIM ((uint32_t)0x00000200) ∏>ETH_MACIMR_PMTIM ((uint32_t)0x00000008) ª>ETH_MACA0HR_MACA0H ((uint32_t)0x0000FFFF) æ>ETH_MACA0LR_MACA0L ((uint32_t)0xFFFFFFFF) ¡>ETH_MACA1HR_AE ((uint32_t)0x80000000) ¬>ETH_MACA1HR_SA ((uint32_t)0x40000000) √>ETH_MACA1HR_MBC ((uint32_t)0x3F000000) ƒ>ETH_MACA1HR_MBC_HBits15_8 ((uint32_t)0x20000000) ≈>ETH_MACA1HR_MBC_HBits7_0 ((uint32_t)0x10000000) ∆>ETH_MACA1HR_MBC_LBits31_24 ((uint32_t)0x08000000) «>ETH_MACA1HR_MBC_LBits23_16 ((uint32_t)0x04000000) »>ETH_MACA1HR_MBC_LBits15_8 ((uint32_t)0x02000000) …>ETH_MACA1HR_MBC_LBits7_0 ((uint32_t)0x01000000)  >ETH_MACA1HR_MACA1H ((uint32_t)0x0000FFFF) Õ>ETH_MACA1LR_MACA1L ((uint32_t)0xFFFFFFFF) –>ETH_MACA2HR_AE ((uint32_t)0x80000000) —>ETH_MACA2HR_SA ((uint32_t)0x40000000) “>ETH_MACA2HR_MBC ((uint32_t)0x3F000000) ”>ETH_MACA2HR_MBC_HBits15_8 ((uint32_t)0x20000000) ‘>ETH_MACA2HR_MBC_HBits7_0 ((uint32_t)0x10000000) ’>ETH_MACA2HR_MBC_LBits31_24 ((uint32_t)0x08000000) ÷>ETH_MACA2HR_MBC_LBits23_16 ((uint32_t)0x04000000) ◊>ETH_MACA2HR_MBC_LBits15_8 ((uint32_t)0x02000000) ÿ>ETH_MACA2HR_MBC_LBits7_0 ((uint32_t)0x01000000) Ÿ>ETH_MACA2HR_MACA2H ((uint32_t)0x0000FFFF) ‹>ETH_MACA2LR_MACA2L ((uint32_t)0xFFFFFFFF) ﬂ>ETH_MACA3HR_AE ((uint32_t)0x80000000) ‡>ETH_MACA3HR_SA ((uint32_t)0x40000000) ·>ETH_MACA3HR_MBC ((uint32_t)0x3F000000) ‚>ETH_MACA3HR_MBC_HBits15_8 ((uint32_t)0x20000000) „>ETH_MACA3HR_MBC_HBits7_0 ((uint32_t)0x10000000) ‰>ETH_MACA3HR_MBC_LBits31_24 ((uint32_t)0x08000000) Â>ETH_MACA3HR_MBC_LBits23_16 ((uint32_t)0x04000000) Ê>ETH_MACA3HR_MBC_LBits15_8 ((uint32_t)0x02000000) Á>ETH_MACA3HR_MBC_LBits7_0 ((uint32_t)0x01000000) Ë>ETH_MACA3HR_MACA3H ((uint32_t)0x0000FFFF) Î>ETH_MACA3LR_MACA3L ((uint32_t)0xFFFFFFFF) Ú>ETH_MMCCR_MCF ((uint32_t)0x00000008) Û>ETH_MMCCR_ROR ((uint32_t)0x00000004) Ù>ETH_MMCCR_CSR ((uint32_t)0x00000002) ı>ETH_MMCCR_CR ((uint32_t)0x00000001) ¯>ETH_MMCRIR_RGUFS ((uint32_t)0x00020000) ˘>ETH_MMCRIR_RFAES ((uint32_t)0x00000040) ˙>ETH_MMCRIR_RFCES ((uint32_t)0x00000020) ˝>ETH_MMCTIR_TGFS ((uint32_t)0x00200000) ˛>ETH_MMCTIR_TGFMSCS ((uint32_t)0x00008000) ˇ>ETH_MMCTIR_TGFSCS ((uint32_t)0x00004000) Ç?ETH_MMCRIMR_RGUFM ((uint32_t)0x00020000) É?ETH_MMCRIMR_RFAEM ((uint32_t)0x00000040) Ñ?ETH_MMCRIMR_RFCEM ((uint32_t)0x00000020) á?ETH_MMCTIMR_TGFM ((uint32_t)0x00200000) à?ETH_MMCTIMR_TGFMSCM ((uint32_t)0x00008000) â?ETH_MMCTIMR_TGFSCM ((uint32_t)0x00004000) å?ETH_MMCTGFSCCR_TGFSCC ((uint32_t)0xFFFFFFFF) è?ETH_MMCTGFMSCCR_TGFMSCC ((uint32_t)0xFFFFFFFF) í?ETH_MMCTGFCR_TGFC ((uint32_t)0xFFFFFFFF) ï?ETH_MMCRFCECR_RFCEC ((uint32_t)0xFFFFFFFF) ò?ETH_MMCRFAECR_RFAEC ((uint32_t)0xFFFFFFFF) õ?ETH_MMCRGUFCR_RGUFC ((uint32_t)0xFFFFFFFF) ¢?ETH_PTPTSCR_TSARU ((uint32_t)0x00000020) £?ETH_PTPTSCR_TSITE ((uint32_t)0x00000010) §?ETH_PTPTSCR_TSSTU ((uint32_t)0x00000008) •?ETH_PTPTSCR_TSSTI ((uint32_t)0x00000004) ¶?ETH_PTPTSCR_TSFCU ((uint32_t)0x00000002) ß?ETH_PTPTSCR_TSE ((uint32_t)0x00000001) ™?ETH_PTPSSIR_STSSI ((uint32_t)0x000000FF) ≠?ETH_PTPTSHR_STS ((uint32_t)0xFFFFFFFF) ∞?ETH_PTPTSLR_STPNS ((uint32_t)0x80000000) ±?ETH_PTPTSLR_STSS ((uint32_t)0x7FFFFFFF) ¥?ETH_PTPTSHUR_TSUS ((uint32_t)0xFFFFFFFF) ∑?ETH_PTPTSLUR_TSUPNS ((uint32_t)0x80000000) ∏?ETH_PTPTSLUR_TSUSS ((uint32_t)0x7FFFFFFF) ª?ETH_PTPTSAR_TSA ((uint32_t)0xFFFFFFFF) æ?ETH_PTPTTHR_TTSH ((uint32_t)0xFFFFFFFF) ¡?ETH_PTPTTLR_TTSL ((uint32_t)0xFFFFFFFF) »?ETH_DMABMR_AAB ((uint32_t)0x02000000) …?ETH_DMABMR_FPM ((uint32_t)0x01000000)  ?ETH_DMABMR_USP ((uint32_t)0x00800000) À?ETH_DMABMR_RDP ((uint32_t)0x007E0000) Ã?ETH_DMABMR_RDP_1Beat ((uint32_t)0x00020000) Õ?ETH_DMABMR_RDP_2Beat ((uint32_t)0x00040000) Œ?ETH_DMABMR_RDP_4Beat ((uint32_t)0x00080000) œ?ETH_DMABMR_RDP_8Beat ((uint32_t)0x00100000) –?ETH_DMABMR_RDP_16Beat ((uint32_t)0x00200000) —?ETH_DMABMR_RDP_32Beat ((uint32_t)0x00400000) “?ETH_DMABMR_RDP_4xPBL_4Beat ((uint32_t)0x01020000) ”?ETH_DMABMR_RDP_4xPBL_8Beat ((uint32_t)0x01040000) ‘?ETH_DMABMR_RDP_4xPBL_16Beat ((uint32_t)0x01080000) ’?ETH_DMABMR_RDP_4xPBL_32Beat ((uint32_t)0x01100000) ÷?ETH_DMABMR_RDP_4xPBL_64Beat ((uint32_t)0x01200000) ◊?ETH_DMABMR_RDP_4xPBL_128Beat ((uint32_t)0x01400000) ÿ?ETH_DMABMR_FB ((uint32_t)0x00010000) Ÿ?ETH_DMABMR_RTPR ((uint32_t)0x0000C000) ⁄?ETH_DMABMR_RTPR_1_1 ((uint32_t)0x00000000) €?ETH_DMABMR_RTPR_2_1 ((uint32_t)0x00004000) ‹?ETH_DMABMR_RTPR_3_1 ((uint32_t)0x00008000) ›?ETH_DMABMR_RTPR_4_1 ((uint32_t)0x0000C000) ﬁ?ETH_DMABMR_PBL ((uint32_t)0x00003F00) ﬂ?ETH_DMABMR_PBL_1Beat ((uint32_t)0x00000100) ‡?ETH_DMABMR_PBL_2Beat ((uint32_t)0x00000200) ·?ETH_DMABMR_PBL_4Beat ((uint32_t)0x00000400) ‚?ETH_DMABMR_PBL_8Beat ((uint32_t)0x00000800) „?ETH_DMABMR_PBL_16Beat ((uint32_t)0x00001000) ‰?ETH_DMABMR_PBL_32Beat ((uint32_t)0x00002000) Â?ETH_DMABMR_PBL_4xPBL_4Beat ((uint32_t)0x01000100) Ê?ETH_DMABMR_PBL_4xPBL_8Beat ((uint32_t)0x01000200) Á?ETH_DMABMR_PBL_4xPBL_16Beat ((uint32_t)0x01000400) Ë?ETH_DMABMR_PBL_4xPBL_32Beat ((uint32_t)0x01000800) È?ETH_DMABMR_PBL_4xPBL_64Beat ((uint32_t)0x01001000) Í?ETH_DMABMR_PBL_4xPBL_128Beat ((uint32_t)0x01002000) Î?ETH_DMABMR_DSL ((uint32_t)0x0000007C) Ï?ETH_DMABMR_DA ((uint32_t)0x00000002) Ì?ETH_DMABMR_SR ((uint32_t)0x00000001) ?ETH_DMATPDR_TPD ((uint32_t)0xFFFFFFFF) Û?ETH_DMARPDR_RPD ((uint32_t)0xFFFFFFFF) ˆ?ETH_DMARDLAR_SRL ((uint32_t)0xFFFFFFFF) ˘?ETH_DMATDLAR_STL ((uint32_t)0xFFFFFFFF) ¸?ETH_DMASR_TSTS ((uint32_t)0x20000000) ˝?ETH_DMASR_PMTS ((uint32_t)0x10000000) ˛?ETH_DMASR_MMCS ((uint32_t)0x08000000) ˇ?ETH_DMASR_EBS ((uint32_t)0x03800000) Å@ETH_DMASR_EBS_DescAccess ((uint32_t)0x02000000) Ç@ETH_DMASR_EBS_ReadTransf ((uint32_t)0x01000000) É@ETH_DMASR_EBS_DataTransfTx ((uint32_t)0x00800000) Ñ@ETH_DMASR_TPS ((uint32_t)0x00700000) Ö@ETH_DMASR_TPS_Stopped ((uint32_t)0x00000000) Ü@ETH_DMASR_TPS_Fetching ((uint32_t)0x00100000) á@ETH_DMASR_TPS_Waiting ((uint32_t)0x00200000) à@ETH_DMASR_TPS_Reading ((uint32_t)0x00300000) â@ETH_DMASR_TPS_Suspended ((uint32_t)0x00600000) ä@ETH_DMASR_TPS_Closing ((uint32_t)0x00700000) ã@ETH_DMASR_RPS ((uint32_t)0x000E0000) å@ETH_DMASR_RPS_Stopped ((uint32_t)0x00000000) ç@ETH_DMASR_RPS_Fetching ((uint32_t)0x00020000) é@ETH_DMASR_RPS_Waiting ((uint32_t)0x00060000) è@ETH_DMASR_RPS_Suspended ((uint32_t)0x00080000) ê@ETH_DMASR_RPS_Closing ((uint32_t)0x000A0000) ë@ETH_DMASR_RPS_Queuing ((uint32_t)0x000E0000) í@ETH_DMASR_NIS ((uint32_t)0x00010000) ì@ETH_DMASR_AIS ((uint32_t)0x00008000) î@ETH_DMASR_ERS ((uint32_t)0x00004000) ï@ETH_DMASR_FBES ((uint32_t)0x00002000) ñ@ETH_DMASR_ETS ((uint32_t)0x00000400) ó@ETH_DMASR_RWTS ((uint32_t)0x00000200) ò@ETH_DMASR_RPSS ((uint32_t)0x00000100) ô@ETH_DMASR_RBUS ((uint32_t)0x00000080) ö@ETH_DMASR_RS ((uint32_t)0x00000040) õ@ETH_DMASR_TUS ((uint32_t)0x00000020) ú@ETH_DMASR_ROS ((uint32_t)0x00000010) ù@ETH_DMASR_TJTS ((uint32_t)0x00000008) û@ETH_DMASR_TBUS ((uint32_t)0x00000004) ü@ETH_DMASR_TPSS ((uint32_t)0x00000002) †@ETH_DMASR_TS ((uint32_t)0x00000001) £@ETH_DMAOMR_DTCEFD ((uint32_t)0x04000000) §@ETH_DMAOMR_RSF ((uint32_t)0x02000000) •@ETH_DMAOMR_DFRF ((uint32_t)0x01000000) ¶@ETH_DMAOMR_TSF ((uint32_t)0x00200000) ß@ETH_DMAOMR_FTF ((uint32_t)0x00100000) ®@ETH_DMAOMR_TTC ((uint32_t)0x0001C000) ©@ETH_DMAOMR_TTC_64Bytes ((uint32_t)0x00000000) ™@ETH_DMAOMR_TTC_128Bytes ((uint32_t)0x00004000) ´@ETH_DMAOMR_TTC_192Bytes ((uint32_t)0x00008000) ¨@ETH_DMAOMR_TTC_256Bytes ((uint32_t)0x0000C000) ≠@ETH_DMAOMR_TTC_40Bytes ((uint32_t)0x00010000) Æ@ETH_DMAOMR_TTC_32Bytes ((uint32_t)0x00014000) Ø@ETH_DMAOMR_TTC_24Bytes ((uint32_t)0x00018000) ∞@ETH_DMAOMR_TTC_16Bytes ((uint32_t)0x0001C000) ±@ETH_DMAOMR_ST ((uint32_t)0x00002000) ≤@ETH_DMAOMR_FEF ((uint32_t)0x00000080) ≥@ETH_DMAOMR_FUGF ((uint32_t)0x00000040) ¥@ETH_DMAOMR_RTC ((uint32_t)0x00000018) µ@ETH_DMAOMR_RTC_64Bytes ((uint32_t)0x00000000) ∂@ETH_DMAOMR_RTC_32Bytes ((uint32_t)0x00000008) ∑@ETH_DMAOMR_RTC_96Bytes ((uint32_t)0x00000010) ∏@ETH_DMAOMR_RTC_128Bytes ((uint32_t)0x00000018) π@ETH_DMAOMR_OSF ((uint32_t)0x00000004) ∫@ETH_DMAOMR_SR ((uint32_t)0x00000002) Ω@ETH_DMAIER_NISE ((uint32_t)0x00010000) æ@ETH_DMAIER_AISE ((uint32_t)0x00008000) ø@ETH_DMAIER_ERIE ((uint32_t)0x00004000) ¿@ETH_DMAIER_FBEIE ((uint32_t)0x00002000) ¡@ETH_DMAIER_ETIE ((uint32_t)0x00000400) ¬@ETH_DMAIER_RWTIE ((uint32_t)0x00000200) √@ETH_DMAIER_RPSIE ((uint32_t)0x00000100) ƒ@ETH_DMAIER_RBUIE ((uint32_t)0x00000080) ≈@ETH_DMAIER_RIE ((uint32_t)0x00000040) ∆@ETH_DMAIER_TUIE ((uint32_t)0x00000020) «@ETH_DMAIER_ROIE ((uint32_t)0x00000010) »@ETH_DMAIER_TJTIE ((uint32_t)0x00000008) …@ETH_DMAIER_TBUIE ((uint32_t)0x00000004)  @ETH_DMAIER_TPSIE ((uint32_t)0x00000002) À@ETH_DMAIER_TIE ((uint32_t)0x00000001) Œ@ETH_DMAMFBOCR_OFOC ((uint32_t)0x10000000) œ@ETH_DMAMFBOCR_MFA ((uint32_t)0x0FFE0000) –@ETH_DMAMFBOCR_OMFC ((uint32_t)0x00010000) —@ETH_DMAMFBOCR_MFC ((uint32_t)0x0000FFFF) ‘@ETH_DMACHTDR_HTDAP ((uint32_t)0xFFFFFFFF) ◊@ETH_DMACHRDR_HRDAP ((uint32_t)0xFFFFFFFF) ⁄@ETH_DMACHTBAR_HTBAP ((uint32_t)0xFFFFFFFF) ›@ETH_DMACHRBAR_HRBAP ((uint32_t)0xFFFFFFFF) È@@SET_BIT(REG,BIT) ((REG) |= (BIT)) Ú@CLEAR_BIT(REG,BIT) ((REG) &= ~(BIT)) Ù@READ_BIT(REG,BIT) ((REG) & (BIT)) ˆ@CLEAR_REG(REG) ((REG) = (0x0)) ¯@WRITE_REG(REG,VAL) ((REG) = (VAL)) ˙@READ_REG(REG) ((REG)) ¸@MODIFY_REG(REG,CLEARMASK,SETMASK) WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))     «   ∆   ≈   t       
..\Source\STM32F10x_FWLib\inc\stm32f10x_adc.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle         *ﬂADC_Mode _  # ADC_ScanConvMode ı  #ADC_ContinuousConvMode ı  #ADC_ExternalTrigConv _  #ADC_DataAlign _  #ADC_NbrOfChannel @  # PADC_InitTypeDef ø J  p    d            ..\Source\STM32F10x_FWLib\inc\ ..\Source\USER\  stm32f10x_adc.h   stm32f10x.h      __STM32F10x_ADC_H   SIS_ADC_ALL_PERIPH(PERIPH) (((PERIPH) == ADC1) || ((PERIPH) == ADC2) || ((PERIPH) == ADC3)) WIS_ADC_DMA_PERIPH(PERIPH) (((PERIPH) == ADC1) || ((PERIPH) == ADC3)) ^ADC_Mode_Independent ((uint32_t)0x00000000) _ADC_Mode_RegInjecSimult ((uint32_t)0x00010000) `ADC_Mode_RegSimult_AlterTrig ((uint32_t)0x00020000) aADC_Mode_InjecSimult_FastInterl ((uint32_t)0x00030000) bADC_Mode_InjecSimult_SlowInterl ((uint32_t)0x00040000) cADC_Mode_InjecSimult ((uint32_t)0x00050000) dADC_Mode_RegSimult ((uint32_t)0x00060000) eADC_Mode_FastInterl ((uint32_t)0x00070000) fADC_Mode_SlowInterl ((uint32_t)0x00080000) gADC_Mode_AlterTrig ((uint32_t)0x00090000) iIS_ADC_MODE(MODE) (((MODE) == ADC_Mode_Independent) || ((MODE) == ADC_Mode_RegInjecSimult) || ((MODE) == ADC_Mode_RegSimult_AlterTrig) || ((MODE) == ADC_Mode_InjecSimult_FastInterl) || ((MODE) == ADC_Mode_InjecSimult_SlowInterl) || ((MODE) == ADC_Mode_InjecSimult) || ((MODE) == ADC_Mode_RegSimult) || ((MODE) == ADC_Mode_FastInterl) || ((MODE) == ADC_Mode_SlowInterl) || ((MODE) == ADC_Mode_AlterTrig)) {ADC_ExternalTrigConv_T1_CC1 ((uint32_t)0x00000000) |ADC_ExternalTrigConv_T1_CC2 ((uint32_t)0x00020000) }ADC_ExternalTrigConv_T2_CC2 ((uint32_t)0x00060000) ~ADC_ExternalTrigConv_T3_TRGO ((uint32_t)0x00080000) ADC_ExternalTrigConv_T4_CC4 ((uint32_t)0x000A0000) ÄADC_ExternalTrigConv_Ext_IT11_TIM8_TRGO ((uint32_t)0x000C0000) ÇADC_ExternalTrigConv_T1_CC3 ((uint32_t)0x00040000) ÉADC_ExternalTrigConv_None ((uint32_t)0x000E0000) ÖADC_ExternalTrigConv_T3_CC1 ((uint32_t)0x00000000) ÜADC_ExternalTrigConv_T2_CC3 ((uint32_t)0x00020000) áADC_ExternalTrigConv_T8_CC1 ((uint32_t)0x00060000) àADC_ExternalTrigConv_T8_TRGO ((uint32_t)0x00080000) âADC_ExternalTrigConv_T5_CC1 ((uint32_t)0x000A0000) äADC_ExternalTrigConv_T5_CC3 ((uint32_t)0x000C0000) åIS_ADC_EXT_TRIG(REGTRIG) (((REGTRIG) == ADC_ExternalTrigConv_T1_CC1) || ((REGTRIG) == ADC_ExternalTrigConv_T1_CC2) || ((REGTRIG) == ADC_ExternalTrigConv_T1_CC3) || ((REGTRIG) == ADC_ExternalTrigConv_T2_CC2) || ((REGTRIG) == ADC_ExternalTrigConv_T3_TRGO) || ((REGTRIG) == ADC_ExternalTrigConv_T4_CC4) || ((REGTRIG) == ADC_ExternalTrigConv_Ext_IT11_TIM8_TRGO) || ((REGTRIG) == ADC_ExternalTrigConv_None) || ((REGTRIG) == ADC_ExternalTrigConv_T3_CC1) || ((REGTRIG) == ADC_ExternalTrigConv_T2_CC3) || ((REGTRIG) == ADC_ExternalTrigConv_T8_CC1) || ((REGTRIG) == ADC_ExternalTrigConv_T8_TRGO) || ((REGTRIG) == ADC_ExternalTrigConv_T5_CC1) || ((REGTRIG) == ADC_ExternalTrigConv_T5_CC3)) ¢ADC_DataAlign_Right ((uint32_t)0x00000000) £ADC_DataAlign_Left ((uint32_t)0x00000800) §IS_ADC_DATA_ALIGN(ALIGN) (((ALIGN) == ADC_DataAlign_Right) || ((ALIGN) == ADC_DataAlign_Left)) ÆADC_Channel_0 ((uint8_t)0x00) ØADC_Channel_1 ((uint8_t)0x01) ∞ADC_Channel_2 ((uint8_t)0x02) ±ADC_Channel_3 ((uint8_t)0x03) ≤ADC_Channel_4 ((uint8_t)0x04) ≥ADC_Channel_5 ((uint8_t)0x05) ¥ADC_Channel_6 ((uint8_t)0x06) µADC_Channel_7 ((uint8_t)0x07) ∂ADC_Channel_8 ((uint8_t)0x08) ∑ADC_Channel_9 ((uint8_t)0x09) ∏ADC_Channel_10 ((uint8_t)0x0A) πADC_Channel_11 ((uint8_t)0x0B) ∫ADC_Channel_12 ((uint8_t)0x0C) ªADC_Channel_13 ((uint8_t)0x0D) ºADC_Channel_14 ((uint8_t)0x0E) ΩADC_Channel_15 ((uint8_t)0x0F) æADC_Channel_16 ((uint8_t)0x10) øADC_Channel_17 ((uint8_t)0x11) ¡ADC_Channel_TempSensor ((uint8_t)ADC_Channel_16) ¬ADC_Channel_Vrefint ((uint8_t)ADC_Channel_17) ƒIS_ADC_CHANNEL(CHANNEL) (((CHANNEL) == ADC_Channel_0) || ((CHANNEL) == ADC_Channel_1) || ((CHANNEL) == ADC_Channel_2) || ((CHANNEL) == ADC_Channel_3) || ((CHANNEL) == ADC_Channel_4) || ((CHANNEL) == ADC_Channel_5) || ((CHANNEL) == ADC_Channel_6) || ((CHANNEL) == ADC_Channel_7) || ((CHANNEL) == ADC_Channel_8) || ((CHANNEL) == ADC_Channel_9) || ((CHANNEL) == ADC_Channel_10) || ((CHANNEL) == ADC_Channel_11) || ((CHANNEL) == ADC_Channel_12) || ((CHANNEL) == ADC_Channel_13) || ((CHANNEL) == ADC_Channel_14) || ((CHANNEL) == ADC_Channel_15) || ((CHANNEL) == ADC_Channel_16) || ((CHANNEL) == ADC_Channel_17)) ’ADC_SampleTime_1Cycles5 ((uint8_t)0x00) ÷ADC_SampleTime_7Cycles5 ((uint8_t)0x01) ◊ADC_SampleTime_13Cycles5 ((uint8_t)0x02) ÿADC_SampleTime_28Cycles5 ((uint8_t)0x03) ŸADC_SampleTime_41Cycles5 ((uint8_t)0x04) ⁄ADC_SampleTime_55Cycles5 ((uint8_t)0x05) €ADC_SampleTime_71Cycles5 ((uint8_t)0x06) ‹ADC_SampleTime_239Cycles5 ((uint8_t)0x07) ›IS_ADC_SAMPLE_TIME(TIME) (((TIME) == ADC_SampleTime_1Cycles5) || ((TIME) == ADC_SampleTime_7Cycles5) || ((TIME) == ADC_SampleTime_13Cycles5) || ((TIME) == ADC_SampleTime_28Cycles5) || ((TIME) == ADC_SampleTime_41Cycles5) || ((TIME) == ADC_SampleTime_55Cycles5) || ((TIME) == ADC_SampleTime_71Cycles5) || ((TIME) == ADC_SampleTime_239Cycles5)) ÌADC_ExternalTrigInjecConv_T2_TRGO ((uint32_t)0x00002000) ÓADC_ExternalTrigInjecConv_T2_CC1 ((uint32_t)0x00003000) ÔADC_ExternalTrigInjecConv_T3_CC4 ((uint32_t)0x00004000) ADC_ExternalTrigInjecConv_T4_TRGO ((uint32_t)0x00005000) ÒADC_ExternalTrigInjecConv_Ext_IT15_TIM8_CC4 ((uint32_t)0x00006000) ÛADC_ExternalTrigInjecConv_T1_TRGO ((uint32_t)0x00000000) ÙADC_ExternalTrigInjecConv_T1_CC4 ((uint32_t)0x00001000) ıADC_ExternalTrigInjecConv_None ((uint32_t)0x00007000) ˜ADC_ExternalTrigInjecConv_T4_CC3 ((uint32_t)0x00002000) ¯ADC_ExternalTrigInjecConv_T8_CC2 ((uint32_t)0x00003000) ˘ADC_ExternalTrigInjecConv_T8_CC4 ((uint32_t)0x00004000) ˙ADC_ExternalTrigInjecConv_T5_TRGO ((uint32_t)0x00005000) ˚ADC_ExternalTrigInjecConv_T5_CC4 ((uint32_t)0x00006000) ˝IS_ADC_EXT_INJEC_TRIG(INJTRIG) (((INJTRIG) == ADC_ExternalTrigInjecConv_T1_TRGO) || ((INJTRIG) == ADC_ExternalTrigInjecConv_T1_CC4) || ((INJTRIG) == ADC_ExternalTrigInjecConv_T2_TRGO) || ((INJTRIG) == ADC_ExternalTrigInjecConv_T2_CC1) || ((INJTRIG) == ADC_ExternalTrigInjecConv_T3_CC4) || ((INJTRIG) == ADC_ExternalTrigInjecConv_T4_TRGO) || ((INJTRIG) == ADC_ExternalTrigInjecConv_Ext_IT15_TIM8_CC4) || ((INJTRIG) == ADC_ExternalTrigInjecConv_None) || ((INJTRIG) == ADC_ExternalTrigInjecConv_T4_CC3) || ((INJTRIG) == ADC_ExternalTrigInjecConv_T8_CC2) || ((INJTRIG) == ADC_ExternalTrigInjecConv_T8_CC4) || ((INJTRIG) == ADC_ExternalTrigInjecConv_T5_TRGO) || ((INJTRIG) == ADC_ExternalTrigInjecConv_T5_CC4)) íADC_InjectedChannel_1 ((uint8_t)0x14) ìADC_InjectedChannel_2 ((uint8_t)0x18) îADC_InjectedChannel_3 ((uint8_t)0x1C) ïADC_InjectedChannel_4 ((uint8_t)0x20) ñIS_ADC_INJECTED_CHANNEL(CHANNEL) (((CHANNEL) == ADC_InjectedChannel_1) || ((CHANNEL) == ADC_InjectedChannel_2) || ((CHANNEL) == ADC_InjectedChannel_3) || ((CHANNEL) == ADC_InjectedChannel_4)) ¢ADC_AnalogWatchdog_SingleRegEnable ((uint32_t)0x00800200) £ADC_AnalogWatchdog_SingleInjecEnable ((uint32_t)0x00400200) §ADC_AnalogWatchdog_SingleRegOrInjecEnable ((uint32_t)0x00C00200) •ADC_AnalogWatchdog_AllRegEnable ((uint32_t)0x00800000) ¶ADC_AnalogWatchdog_AllInjecEnable ((uint32_t)0x00400000) ßADC_AnalogWatchdog_AllRegAllInjecEnable ((uint32_t)0x00C00000) ®ADC_AnalogWatchdog_None ((uint32_t)0x00000000) ™IS_ADC_ANALOG_WATCHDOG(WATCHDOG) (((WATCHDOG) == ADC_AnalogWatchdog_SingleRegEnable) || ((WATCHDOG) == ADC_AnalogWatchdog_SingleInjecEnable) || ((WATCHDOG) == ADC_AnalogWatchdog_SingleRegOrInjecEnable) || ((WATCHDOG) == ADC_AnalogWatchdog_AllRegEnable) || ((WATCHDOG) == ADC_AnalogWatchdog_AllInjecEnable) || ((WATCHDOG) == ADC_AnalogWatchdog_AllRegAllInjecEnable) || ((WATCHDOG) == ADC_AnalogWatchdog_None)) πADC_IT_EOC ((uint16_t)0x0220) ∫ADC_IT_AWD ((uint16_t)0x0140) ªADC_IT_JEOC ((uint16_t)0x0480) ΩIS_ADC_IT(IT) ((((IT) & (uint16_t)0xF81F) == 0x00) && ((IT) != 0x00)) øIS_ADC_GET_IT(IT) (((IT) == ADC_IT_EOC) || ((IT) == ADC_IT_AWD) || ((IT) == ADC_IT_JEOC)) …ADC_FLAG_AWD ((uint8_t)0x01)  ADC_FLAG_EOC ((uint8_t)0x02) ÀADC_FLAG_JEOC ((uint8_t)0x04) ÃADC_FLAG_JSTRT ((uint8_t)0x08) ÕADC_FLAG_STRT ((uint8_t)0x10) ŒIS_ADC_CLEAR_FLAG(FLAG) ((((FLAG) & (uint8_t)0xE0) == 0x00) && ((FLAG) != 0x00)) œIS_ADC_GET_FLAG(FLAG) (((FLAG) == ADC_FLAG_AWD) || ((FLAG) == ADC_FLAG_EOC) || ((FLAG) == ADC_FLAG_JEOC) || ((FLAG)== ADC_FLAG_JSTRT) || ((FLAG) == ADC_FLAG_STRT)) ⁄IS_ADC_THRESHOLD(THRESHOLD) ((THRESHOLD) <= 0xFFF) ‰IS_ADC_OFFSET(OFFSET) ((OFFSET) <= 0xFFF) ÓIS_ADC_INJECTED_LENGTH(LENGTH) (((LENGTH) >= 0x1) && ((LENGTH) <= 0x4)) ¯IS_ADC_INJECTED_RANK(RANK) (((RANK) >= 0x1) && ((RANK) <= 0x4)) ÉIS_ADC_REGULAR_LENGTH(LENGTH) (((LENGTH) >= 0x1) && ((LENGTH) <= 0x10)) åIS_ADC_REGULAR_RANK(RANK) (((RANK) >= 0x1) && ((RANK) <= 0x10)) ñIS_ADC_REGULAR_DISC_NUMBER(NUMBER) (((NUMBER) >= 0x1) && ((NUMBER) <= 0x8))        À       …   ¿        
..\Source\STM32F10x_FWLib\inc\stm32f10x_bkp.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle              p    d            ..\Source\STM32F10x_FWLib\inc\ ..\Source\USER\  stm32f10x_bkp.h   stm32f10x.h      __STM32F10x_BKP_H   :BKP_TamperPinLevel_High ((uint16_t)0x0000) ;BKP_TamperPinLevel_Low ((uint16_t)0x0001) <IS_BKP_TAMPER_PIN_LEVEL(LEVEL) (((LEVEL) == BKP_TamperPinLevel_High) || ((LEVEL) == BKP_TamperPinLevel_Low)) FBKP_RTCOutputSource_None ((uint16_t)0x0000) GBKP_RTCOutputSource_CalibClock ((uint16_t)0x0080) HBKP_RTCOutputSource_Alarm ((uint16_t)0x0100) IBKP_RTCOutputSource_Second ((uint16_t)0x0300) JIS_BKP_RTC_OUTPUT_SOURCE(SOURCE) (((SOURCE) == BKP_RTCOutputSource_None) || ((SOURCE) == BKP_RTCOutputSource_CalibClock) || ((SOURCE) == BKP_RTCOutputSource_Alarm) || ((SOURCE) == BKP_RTCOutputSource_Second)) VBKP_DR1 ((uint16_t)0x0004) WBKP_DR2 ((uint16_t)0x0008) XBKP_DR3 ((uint16_t)0x000C) YBKP_DR4 ((uint16_t)0x0010) ZBKP_DR5 ((uint16_t)0x0014) [BKP_DR6 ((uint16_t)0x0018) \BKP_DR7 ((uint16_t)0x001C) ]BKP_DR8 ((uint16_t)0x0020) ^BKP_DR9 ((uint16_t)0x0024) _BKP_DR10 ((uint16_t)0x0028) `BKP_DR11 ((uint16_t)0x0040) aBKP_DR12 ((uint16_t)0x0044) bBKP_DR13 ((uint16_t)0x0048) cBKP_DR14 ((uint16_t)0x004C) dBKP_DR15 ((uint16_t)0x0050) eBKP_DR16 ((uint16_t)0x0054) fBKP_DR17 ((uint16_t)0x0058) gBKP_DR18 ((uint16_t)0x005C) hBKP_DR19 ((uint16_t)0x0060) iBKP_DR20 ((uint16_t)0x0064) jBKP_DR21 ((uint16_t)0x0068) kBKP_DR22 ((uint16_t)0x006C) lBKP_DR23 ((uint16_t)0x0070) mBKP_DR24 ((uint16_t)0x0074) nBKP_DR25 ((uint16_t)0x0078) oBKP_DR26 ((uint16_t)0x007C) pBKP_DR27 ((uint16_t)0x0080) qBKP_DR28 ((uint16_t)0x0084) rBKP_DR29 ((uint16_t)0x0088) sBKP_DR30 ((uint16_t)0x008C) tBKP_DR31 ((uint16_t)0x0090) uBKP_DR32 ((uint16_t)0x0094) vBKP_DR33 ((uint16_t)0x0098) wBKP_DR34 ((uint16_t)0x009C) xBKP_DR35 ((uint16_t)0x00A0) yBKP_DR36 ((uint16_t)0x00A4) zBKP_DR37 ((uint16_t)0x00A8) {BKP_DR38 ((uint16_t)0x00AC) |BKP_DR39 ((uint16_t)0x00B0) }BKP_DR40 ((uint16_t)0x00B4) ~BKP_DR41 ((uint16_t)0x00B8) BKP_DR42 ((uint16_t)0x00BC) ÅIS_BKP_DR(DR) (((DR) == BKP_DR1) || ((DR) == BKP_DR2) || ((DR) == BKP_DR3) || ((DR) == BKP_DR4) || ((DR) == BKP_DR5) || ((DR) == BKP_DR6) || ((DR) == BKP_DR7) || ((DR) == BKP_DR8) || ((DR) == BKP_DR9) || ((DR) == BKP_DR10) || ((DR) == BKP_DR11) || ((DR) == BKP_DR12) || ((DR) == BKP_DR13) || ((DR) == BKP_DR14) || ((DR) == BKP_DR15) || ((DR) == BKP_DR16) || ((DR) == BKP_DR17) || ((DR) == BKP_DR18) || ((DR) == BKP_DR19) || ((DR) == BKP_DR20) || ((DR) == BKP_DR21) || ((DR) == BKP_DR22) || ((DR) == BKP_DR23) || ((DR) == BKP_DR24) || ((DR) == BKP_DR25) || ((DR) == BKP_DR26) || ((DR) == BKP_DR27) || ((DR) == BKP_DR28) || ((DR) == BKP_DR29) || ((DR) == BKP_DR30) || ((DR) == BKP_DR31) || ((DR) == BKP_DR32) || ((DR) == BKP_DR33) || ((DR) == BKP_DR34) || ((DR) == BKP_DR35) || ((DR) == BKP_DR36) || ((DR) == BKP_DR37) || ((DR) == BKP_DR38) || ((DR) == BKP_DR39) || ((DR) == BKP_DR40) || ((DR) == BKP_DR41) || ((DR) == BKP_DR42)) êIS_BKP_CALIBRATION_VALUE(VALUE) ((VALUE) <= 0x7F)        œ   Œ   Õ   ∞       
..\Source\STM32F10x_FWLib\inc\stm32f10x_can.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle         *åCAN_Prescaler O  # CAN_Mode @  #CAN_SJW @  #CAN_BS1 @  #CAN_BS2 @  #CAN_TTCM ı  #CAN_ABOM ı  #CAN_AWUM ı  #CAN_NART ı  #	CAN_RFLM ı  #
CAN_TXFP ı  # PCAN_InitTypeDef ø d*°CAN_FilterIdHigh O  # CAN_FilterIdLow O  #CAN_FilterMaskIdHigh O  #CAN_FilterMaskIdLow O  #CAN_FilterFIFOAssignment O  #CAN_FilterNumber @  #
CAN_FilterMode @  #CAN_FilterScale @  #CAN_FilterActivation ı  # PCAN_FilterInitTypeDef £ã*†StdId _  # ExtId _  #IDE @  #RTR @  #	DLC @  #
ì@   Data # PCanTxMsg øß*üStdId _  # ExtId _  #IDE @  #RTR @  #	DLC @  #
Ö@   Data z#FMI @  # PCanRxMsg 1∆    p    d            ..\Source\STM32F10x_FWLib\inc\ ..\Source\USER\  stm32f10x_can.h   stm32f10x.h      __STM32F10x_CAN_H   .IS_CAN_ALL_PERIPH(PERIPH) (((PERIPH) == CAN1) || ((PERIPH) == CAN2)) ‘CAN_InitStatus_Failed ((uint8_t)0x00) ’CAN_InitStatus_Success ((uint8_t)0x01) ﬂCAN_Mode_Normal ((uint8_t)0x00) ‡CAN_Mode_LoopBack ((uint8_t)0x01) ·CAN_Mode_Silent ((uint8_t)0x02) ‚CAN_Mode_Silent_LoopBack ((uint8_t)0x03) ‰IS_CAN_MODE(MODE) (((MODE) == CAN_Mode_Normal) || ((MODE) == CAN_Mode_LoopBack)|| ((MODE) == CAN_Mode_Silent) || ((MODE) == CAN_Mode_Silent_LoopBack)) ÒCAN_OperatingMode_Initialization ((uint8_t)0x00) ÚCAN_OperatingMode_Normal ((uint8_t)0x01) ÛCAN_OperatingMode_Sleep ((uint8_t)0x02) ˆIS_CAN_OPERATING_MODE(MODE) (((MODE) == CAN_OperatingMode_Initialization) || ((MODE) == CAN_OperatingMode_Normal)|| ((MODE) == CAN_OperatingMode_Sleep)) ÇCAN_ModeStatus_Failed ((uint8_t)0x00) ÉCAN_ModeStatus_Success ((uint8_t)!CAN_ModeStatus_Failed) éCAN_SJW_1tq ((uint8_t)0x00) èCAN_SJW_2tq ((uint8_t)0x01) êCAN_SJW_3tq ((uint8_t)0x02) ëCAN_SJW_4tq ((uint8_t)0x03) ìIS_CAN_SJW(SJW) (((SJW) == CAN_SJW_1tq) || ((SJW) == CAN_SJW_2tq)|| ((SJW) == CAN_SJW_3tq) || ((SJW) == CAN_SJW_4tq)) ùCAN_BS1_1tq ((uint8_t)0x00) ûCAN_BS1_2tq ((uint8_t)0x01) üCAN_BS1_3tq ((uint8_t)0x02) †CAN_BS1_4tq ((uint8_t)0x03) °CAN_BS1_5tq ((uint8_t)0x04) ¢CAN_BS1_6tq ((uint8_t)0x05) £CAN_BS1_7tq ((uint8_t)0x06) §CAN_BS1_8tq ((uint8_t)0x07) •CAN_BS1_9tq ((uint8_t)0x08) ¶CAN_BS1_10tq ((uint8_t)0x09) ßCAN_BS1_11tq ((uint8_t)0x0A) ®CAN_BS1_12tq ((uint8_t)0x0B) ©CAN_BS1_13tq ((uint8_t)0x0C) ™CAN_BS1_14tq ((uint8_t)0x0D) ´CAN_BS1_15tq ((uint8_t)0x0E) ¨CAN_BS1_16tq ((uint8_t)0x0F) ÆIS_CAN_BS1(BS1) ((BS1) <= CAN_BS1_16tq) ∑CAN_BS2_1tq ((uint8_t)0x00) ∏CAN_BS2_2tq ((uint8_t)0x01) πCAN_BS2_3tq ((uint8_t)0x02) ∫CAN_BS2_4tq ((uint8_t)0x03) ªCAN_BS2_5tq ((uint8_t)0x04) ºCAN_BS2_6tq ((uint8_t)0x05) ΩCAN_BS2_7tq ((uint8_t)0x06) æCAN_BS2_8tq ((uint8_t)0x07) ¿IS_CAN_BS2(BS2) ((BS2) <= CAN_BS2_8tq)  IS_CAN_PRESCALER(PRESCALER) (((PRESCALER) >= 1) && ((PRESCALER) <= 1024)) ÷IS_CAN_FILTER_NUMBER(NUMBER) ((NUMBER) <= 27) ‡CAN_FilterMode_IdMask ((uint8_t)0x00) ·CAN_FilterMode_IdList ((uint8_t)0x01) „IS_CAN_FILTER_MODE(MODE) (((MODE) == CAN_FilterMode_IdMask) || ((MODE) == CAN_FilterMode_IdList)) ÌCAN_FilterScale_16bit ((uint8_t)0x00) ÓCAN_FilterScale_32bit ((uint8_t)0x01) IS_CAN_FILTER_SCALE(SCALE) (((SCALE) == CAN_FilterScale_16bit) || ((SCALE) == CAN_FilterScale_32bit)) ˚CAN_Filter_FIFO0 ((uint8_t)0x00) ¸CAN_Filter_FIFO1 ((uint8_t)0x01) ˝IS_CAN_FILTER_FIFO(FIFO) (((FIFO) == CAN_FilterFIFO0) || ((FIFO) == CAN_FilterFIFO1)) ÜIS_CAN_BANKNUMBER(BANKNUMBER) (((BANKNUMBER) >= 1) && ((BANKNUMBER) <= 27)) èIS_CAN_TRANSMITMAILBOX(TRANSMITMAILBOX) ((TRANSMITMAILBOX) <= ((uint8_t)0x02)) êIS_CAN_STDID(STDID) ((STDID) <= ((uint32_t)0x7FF)) ëIS_CAN_EXTID(EXTID) ((EXTID) <= ((uint32_t)0x1FFFFFFF)) íIS_CAN_DLC(DLC) ((DLC) <= ((uint8_t)0x08)) úCAN_Id_Standard ((uint32_t)0x00000000) ùCAN_Id_Extended ((uint32_t)0x00000004) ûIS_CAN_IDTYPE(IDTYPE) (((IDTYPE) == CAN_Id_Standard) || ((IDTYPE) == CAN_Id_Extended)) ®CAN_RTR_Data ((uint32_t)0x00000000) ©CAN_RTR_Remote ((uint32_t)0x00000002) ™IS_CAN_RTR(RTR) (((RTR) == CAN_RTR_Data) || ((RTR) == CAN_RTR_Remote)) ¥CAN_TxStatus_Failed ((uint8_t)0x00) µCAN_TxStatus_Ok ((uint8_t)0x01) ∂CAN_TxStatus_Pending ((uint8_t)0x02) ∑CAN_TxStatus_NoMailBox ((uint8_t)0x04) ¡CAN_FIFO0 ((uint8_t)0x00) ¬CAN_FIFO1 ((uint8_t)0x01) ƒIS_CAN_FIFO(FIFO) (((FIFO) == CAN_FIFO0) || ((FIFO) == CAN_FIFO1)) ŒCAN_Sleep_Failed ((uint8_t)0x00) œCAN_Sleep_Ok ((uint8_t)0x01) ŸCAN_WakeUp_Failed ((uint8_t)0x00) ⁄CAN_WakeUp_Ok ((uint8_t)0x01) ÂCAN_ErrorCode_NoErr ((uint8_t)0x00) ÊCAN_ErrorCode_StuffErr ((uint8_t)0x10) ÁCAN_ErrorCode_FormErr ((uint8_t)0x20) ËCAN_ErrorCode_ACKErr ((uint8_t)0x30) ÈCAN_ErrorCode_BitRecessiveErr ((uint8_t)0x40) ÍCAN_ErrorCode_BitDominantErr ((uint8_t)0x50) ÎCAN_ErrorCode_CRCErr ((uint8_t)0x60) ÏCAN_ErrorCode_SoftwareSetErr ((uint8_t)0x70) ˚CAN_FLAG_RQCP0 ((uint32_t)0x38000001) ¸CAN_FLAG_RQCP1 ((uint32_t)0x38000100) ˝CAN_FLAG_RQCP2 ((uint32_t)0x38010000) ÄCAN_FLAG_FMP0 ((uint32_t)0x12000003) ÅCAN_FLAG_FF0 ((uint32_t)0x32000008) ÇCAN_FLAG_FOV0 ((uint32_t)0x32000010) ÉCAN_FLAG_FMP1 ((uint32_t)0x14000003) ÑCAN_FLAG_FF1 ((uint32_t)0x34000008) ÖCAN_FLAG_FOV1 ((uint32_t)0x34000010) àCAN_FLAG_WKU ((uint32_t)0x31000008) âCAN_FLAG_SLAK ((uint32_t)0x31000012) éCAN_FLAG_EWG ((uint32_t)0x10F00001) èCAN_FLAG_EPV ((uint32_t)0x10F00002) êCAN_FLAG_BOF ((uint32_t)0x10F00004) ëCAN_FLAG_LEC ((uint32_t)0x30F00070) ìIS_CAN_GET_FLAG(FLAG) (((FLAG) == CAN_FLAG_LEC) || ((FLAG) == CAN_FLAG_BOF) || ((FLAG) == CAN_FLAG_EPV) || ((FLAG) == CAN_FLAG_EWG) || ((FLAG) == CAN_FLAG_WKU) || ((FLAG) == CAN_FLAG_FOV0) || ((FLAG) == CAN_FLAG_FF0) || ((FLAG) == CAN_FLAG_FMP0) || ((FLAG) == CAN_FLAG_FOV1) || ((FLAG) == CAN_FLAG_FF1) || ((FLAG) == CAN_FLAG_FMP1) || ((FLAG) == CAN_FLAG_RQCP2) || ((FLAG) == CAN_FLAG_RQCP1)|| ((FLAG) == CAN_FLAG_RQCP0) || ((FLAG) == CAN_FLAG_SLAK )) úIS_CAN_CLEAR_FLAG(FLAG) (((FLAG) == CAN_FLAG_LEC) || ((FLAG) == CAN_FLAG_RQCP2) || ((FLAG) == CAN_FLAG_RQCP1) || ((FLAG) == CAN_FLAG_RQCP0) || ((FLAG) == CAN_FLAG_FF0) || ((FLAG) == CAN_FLAG_FOV0) || ((FLAG) == CAN_FLAG_FF1) || ((FLAG) == CAN_FLAG_FOV1) || ((FLAG) == CAN_FLAG_WKU) || ((FLAG) == CAN_FLAG_SLAK)) ¨CAN_IT_TME ((uint32_t)0x00000001) ØCAN_IT_FMP0 ((uint32_t)0x00000002) ∞CAN_IT_FF0 ((uint32_t)0x00000004) ±CAN_IT_FOV0 ((uint32_t)0x00000008) ≤CAN_IT_FMP1 ((uint32_t)0x00000010) ≥CAN_IT_FF1 ((uint32_t)0x00000020) ¥CAN_IT_FOV1 ((uint32_t)0x00000040) ∑CAN_IT_WKU ((uint32_t)0x00010000) ∏CAN_IT_SLK ((uint32_t)0x00020000) ªCAN_IT_EWG ((uint32_t)0x00000100) ºCAN_IT_EPV ((uint32_t)0x00000200) ΩCAN_IT_BOF ((uint32_t)0x00000400) æCAN_IT_LEC ((uint32_t)0x00000800) øCAN_IT_ERR ((uint32_t)0x00008000) ¬CAN_IT_RQCP0 CAN_IT_TME √CAN_IT_RQCP1 CAN_IT_TME ƒCAN_IT_RQCP2 CAN_IT_TME «IS_CAN_IT(IT) (((IT) == CAN_IT_TME) || ((IT) == CAN_IT_FMP0) || ((IT) == CAN_IT_FF0) || ((IT) == CAN_IT_FOV0) || ((IT) == CAN_IT_FMP1) || ((IT) == CAN_IT_FF1) || ((IT) == CAN_IT_FOV1) || ((IT) == CAN_IT_EWG) || ((IT) == CAN_IT_EPV) || ((IT) == CAN_IT_BOF) || ((IT) == CAN_IT_LEC) || ((IT) == CAN_IT_ERR) || ((IT) == CAN_IT_WKU) || ((IT) == CAN_IT_SLK)) œIS_CAN_CLEAR_IT(IT) (((IT) == CAN_IT_TME) || ((IT) == CAN_IT_FF0) || ((IT) == CAN_IT_FOV0)|| ((IT) == CAN_IT_FF1) || ((IT) == CAN_IT_FOV1)|| ((IT) == CAN_IT_EWG) || ((IT) == CAN_IT_EPV) || ((IT) == CAN_IT_BOF) || ((IT) == CAN_IT_LEC) || ((IT) == CAN_IT_ERR) || ((IT) == CAN_IT_WKU) || ((IT) == CAN_IT_SLK)) ›CANINITFAILED CAN_InitStatus_Failed ﬁCANINITOK CAN_InitStatus_Success ﬂCAN_FilterFIFO0 CAN_Filter_FIFO0 ‡CAN_FilterFIFO1 CAN_Filter_FIFO1 ·CAN_ID_STD CAN_Id_Standard ‚CAN_ID_EXT CAN_Id_Extended „CAN_RTR_DATA CAN_RTR_Data ‰CAN_RTR_REMOTE CAN_RTR_Remote ÂCANTXFAILE CAN_TxStatus_Failed ÊCANTXOK CAN_TxStatus_Ok ÁCANTXPENDING CAN_TxStatus_Pending ËCAN_NO_MB CAN_TxStatus_NoMailBox ÈCANSLEEPFAILED CAN_Sleep_Failed ÍCANSLEEPOK CAN_Sleep_Ok ÎCANWAKEUPFAILED CAN_WakeUp_Failed ÏCANWAKEUPOK CAN_WakeUp_Ok       ”   “   —          
..\Source\STM32F10x_FWLib\inc\stm32f10x_cec.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle         *˙CEC_BitTimingMode O  # CEC_BitPeriodMode O  # PCEC_InitTypeDef ø 8   p    d            ..\Source\STM32F10x_FWLib\inc\ ..\Source\USER\  stm32f10x_cec.h   stm32f10x.h      __STM32F10x_CEC_H   ECEC_BitTimingStdMode ((uint16_t)0x00) FCEC_BitTimingErrFreeMode CEC_CFGR_BTEM HIS_CEC_BIT_TIMING_ERROR_MODE(MODE) (((MODE) == CEC_BitTimingStdMode) || ((MODE) == CEC_BitTimingErrFreeMode)) QCEC_BitPeriodStdMode ((uint16_t)0x00) RCEC_BitPeriodFlexibleMode CEC_CFGR_BPEM TIS_CEC_BIT_PERIOD_ERROR_MODE(MODE) (((MODE) == CEC_BitPeriodStdMode) || ((MODE) == CEC_BitPeriodFlexibleMode)) ^CEC_IT_TERR CEC_CSR_TERR _CEC_IT_TBTRF CEC_CSR_TBTRF `CEC_IT_RERR CEC_CSR_RERR aCEC_IT_RBTF CEC_CSR_RBTF bIS_CEC_GET_IT(IT) (((IT) == CEC_IT_TERR) || ((IT) == CEC_IT_TBTRF) || ((IT) == CEC_IT_RERR) || ((IT) == CEC_IT_RBTF)) lIS_CEC_ADDRESS(ADDRESS) ((ADDRESS) < 0x10) tIS_CEC_PRESCALER(PRESCALER) ((PRESCALER) <= 0x3FFF) ÅCEC_FLAG_BTE ((uint32_t)0x10010000) ÇCEC_FLAG_BPE ((uint32_t)0x10020000) ÉCEC_FLAG_RBTFE ((uint32_t)0x10040000) ÑCEC_FLAG_SBE ((uint32_t)0x10080000) ÖCEC_FLAG_ACKE ((uint32_t)0x10100000) ÜCEC_FLAG_LINE ((uint32_t)0x10200000) áCEC_FLAG_TBTFE ((uint32_t)0x10400000) åCEC_FLAG_TEOM ((uint32_t)0x00000002) çCEC_FLAG_TERR ((uint32_t)0x00000004) éCEC_FLAG_TBTRF ((uint32_t)0x00000008) èCEC_FLAG_RSOM ((uint32_t)0x00000010) êCEC_FLAG_REOM ((uint32_t)0x00000020) ëCEC_FLAG_RERR ((uint32_t)0x00000040) íCEC_FLAG_RBTF ((uint32_t)0x00000080) îIS_CEC_CLEAR_FLAG(FLAG) ((((FLAG) & (uint32_t)0xFFFFFF03) == 0x00) && ((FLAG) != 0x00)) ñIS_CEC_GET_FLAG(FLAG) (((FLAG) == CEC_FLAG_BTE) || ((FLAG) == CEC_FLAG_BPE) || ((FLAG) == CEC_FLAG_RBTFE) || ((FLAG)== CEC_FLAG_SBE) || ((FLAG) == CEC_FLAG_ACKE) || ((FLAG) == CEC_FLAG_LINE) || ((FLAG) == CEC_FLAG_TBTFE) || ((FLAG) == CEC_FLAG_TEOM) || ((FLAG) == CEC_FLAG_TERR) || ((FLAG) == CEC_FLAG_TBTRF) || ((FLAG) == CEC_FLAG_RSOM) || ((FLAG) == CEC_FLAG_REOM) || ((FLAG) == CEC_FLAG_RERR) || ((FLAG) == CEC_FLAG_RBTF))      ◊   ÷   ’   ¿        
..\Source\STM32F10x_FWLib\inc\stm32f10x_crc.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle              p    d            ..\Source\STM32F10x_FWLib\inc\ ..\Source\USER\  stm32f10x_crc.h   stm32f10x.h      __STM32F10x_CRC_H         €   ⁄   Ÿ   P       
..\Source\STM32F10x_FWLib\inc\stm32f10x_dac.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle         *πDAC_Trigger _  # DAC_WaveGeneration _  #DAC_LFSRUnmask_TriangleAmplitude _  #DAC_OutputBuffer _  # PDAC_InitTypeDef ø A    p    d            ..\Source\STM32F10x_FWLib\inc\ ..\Source\USER\  stm32f10x_dac.h   stm32f10x.h      __STM32F10x_DAC_H   ODAC_Trigger_None ((uint32_t)0x00000000) QDAC_Trigger_T6_TRGO ((uint32_t)0x00000004) RDAC_Trigger_T8_TRGO ((uint32_t)0x0000000C) TDAC_Trigger_T3_TRGO ((uint32_t)0x0000000C) VDAC_Trigger_T7_TRGO ((uint32_t)0x00000014) WDAC_Trigger_T5_TRGO ((uint32_t)0x0000001C) XDAC_Trigger_T15_TRGO ((uint32_t)0x0000001C) ZDAC_Trigger_T2_TRGO ((uint32_t)0x00000024) [DAC_Trigger_T4_TRGO ((uint32_t)0x0000002C) \DAC_Trigger_Ext_IT9 ((uint32_t)0x00000034) ]DAC_Trigger_Software ((uint32_t)0x0000003C) _IS_DAC_TRIGGER(TRIGGER) (((TRIGGER) == DAC_Trigger_None) || ((TRIGGER) == DAC_Trigger_T6_TRGO) || ((TRIGGER) == DAC_Trigger_T8_TRGO) || ((TRIGGER) == DAC_Trigger_T7_TRGO) || ((TRIGGER) == DAC_Trigger_T5_TRGO) || ((TRIGGER) == DAC_Trigger_T2_TRGO) || ((TRIGGER) == DAC_Trigger_T4_TRGO) || ((TRIGGER) == DAC_Trigger_Ext_IT9) || ((TRIGGER) == DAC_Trigger_Software)) qDAC_WaveGeneration_None ((uint32_t)0x00000000) rDAC_WaveGeneration_Noise ((uint32_t)0x00000040) sDAC_WaveGeneration_Triangle ((uint32_t)0x00000080) tIS_DAC_GENERATE_WAVE(WAVE) (((WAVE) == DAC_WaveGeneration_None) || ((WAVE) == DAC_WaveGeneration_Noise) || ((WAVE) == DAC_WaveGeneration_Triangle)) DAC_LFSRUnmask_Bit0 ((uint32_t)0x00000000) ÄDAC_LFSRUnmask_Bits1_0 ((uint32_t)0x00000100) ÅDAC_LFSRUnmask_Bits2_0 ((uint32_t)0x00000200) ÇDAC_LFSRUnmask_Bits3_0 ((uint32_t)0x00000300) ÉDAC_LFSRUnmask_Bits4_0 ((uint32_t)0x00000400) ÑDAC_LFSRUnmask_Bits5_0 ((uint32_t)0x00000500) ÖDAC_LFSRUnmask_Bits6_0 ((uint32_t)0x00000600) ÜDAC_LFSRUnmask_Bits7_0 ((uint32_t)0x00000700) áDAC_LFSRUnmask_Bits8_0 ((uint32_t)0x00000800) àDAC_LFSRUnmask_Bits9_0 ((uint32_t)0x00000900) âDAC_LFSRUnmask_Bits10_0 ((uint32_t)0x00000A00) äDAC_LFSRUnmask_Bits11_0 ((uint32_t)0x00000B00) ãDAC_TriangleAmplitude_1 ((uint32_t)0x00000000) åDAC_TriangleAmplitude_3 ((uint32_t)0x00000100) çDAC_TriangleAmplitude_7 ((uint32_t)0x00000200) éDAC_TriangleAmplitude_15 ((uint32_t)0x00000300) èDAC_TriangleAmplitude_31 ((uint32_t)0x00000400) êDAC_TriangleAmplitude_63 ((uint32_t)0x00000500) ëDAC_TriangleAmplitude_127 ((uint32_t)0x00000600) íDAC_TriangleAmplitude_255 ((uint32_t)0x00000700) ìDAC_TriangleAmplitude_511 ((uint32_t)0x00000800) îDAC_TriangleAmplitude_1023 ((uint32_t)0x00000900) ïDAC_TriangleAmplitude_2047 ((uint32_t)0x00000A00) ñDAC_TriangleAmplitude_4095 ((uint32_t)0x00000B00) òIS_DAC_LFSR_UNMASK_TRIANGLE_AMPLITUDE(VALUE) (((VALUE) == DAC_LFSRUnmask_Bit0) || ((VALUE) == DAC_LFSRUnmask_Bits1_0) || ((VALUE) == DAC_LFSRUnmask_Bits2_0) || ((VALUE) == DAC_LFSRUnmask_Bits3_0) || ((VALUE) == DAC_LFSRUnmask_Bits4_0) || ((VALUE) == DAC_LFSRUnmask_Bits5_0) || ((VALUE) == DAC_LFSRUnmask_Bits6_0) || ((VALUE) == DAC_LFSRUnmask_Bits7_0) || ((VALUE) == DAC_LFSRUnmask_Bits8_0) || ((VALUE) == DAC_LFSRUnmask_Bits9_0) || ((VALUE) == DAC_LFSRUnmask_Bits10_0) || ((VALUE) == DAC_LFSRUnmask_Bits11_0) || ((VALUE) == DAC_TriangleAmplitude_1) || ((VALUE) == DAC_TriangleAmplitude_3) || ((VALUE) == DAC_TriangleAmplitude_7) || ((VALUE) == DAC_TriangleAmplitude_15) || ((VALUE) == DAC_TriangleAmplitude_31) || ((VALUE) == DAC_TriangleAmplitude_63) || ((VALUE) == DAC_TriangleAmplitude_127) || ((VALUE) == DAC_TriangleAmplitude_255) || ((VALUE) == DAC_TriangleAmplitude_511) || ((VALUE) == DAC_TriangleAmplitude_1023) || ((VALUE) == DAC_TriangleAmplitude_2047) || ((VALUE) == DAC_TriangleAmplitude_4095)) ∏DAC_OutputBuffer_Enable ((uint32_t)0x00000000) πDAC_OutputBuffer_Disable ((uint32_t)0x00000002) ∫IS_DAC_OUTPUT_BUFFER_STATE(STATE) (((STATE) == DAC_OutputBuffer_Enable) || ((STATE) == DAC_OutputBuffer_Disable)) ƒDAC_Channel_1 ((uint32_t)0x00000000) ≈DAC_Channel_2 ((uint32_t)0x00000010) ∆IS_DAC_CHANNEL(CHANNEL) (((CHANNEL) == DAC_Channel_1) || ((CHANNEL) == DAC_Channel_2)) –DAC_Align_12b_R ((uint32_t)0x00000000) —DAC_Align_12b_L ((uint32_t)0x00000004) “DAC_Align_8b_R ((uint32_t)0x00000008) ”IS_DAC_ALIGN(ALIGN) (((ALIGN) == DAC_Align_12b_R) || ((ALIGN) == DAC_Align_12b_L) || ((ALIGN) == DAC_Align_8b_R)) ﬁDAC_Wave_Noise ((uint32_t)0x00000040) ﬂDAC_Wave_Triangle ((uint32_t)0x00000080) ‡IS_DAC_WAVE(WAVE) (((WAVE) == DAC_Wave_Noise) || ((WAVE) == DAC_Wave_Triangle)) ÍIS_DAC_DATA(DATA) ((DATA) <= 0xFFF0)        ﬂ   ﬁ   ›   ¿        
..\Source\STM32F10x_FWLib\inc\stm32f10x_dbgmcu.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle           p    g            ..\Source\STM32F10x_FWLib\inc\ ..\Source\USER\  stm32f10x_dbgmcu.h   stm32f10x.h      __STM32F10x_DBGMCU_H   6DBGMCU_SLEEP ((uint32_t)0x00000001) 7DBGMCU_STOP ((uint32_t)0x00000002) 8DBGMCU_STANDBY ((uint32_t)0x00000004) 9DBGMCU_IWDG_STOP ((uint32_t)0x00000100) :DBGMCU_WWDG_STOP ((uint32_t)0x00000200) ;DBGMCU_TIM1_STOP ((uint32_t)0x00000400) <DBGMCU_TIM2_STOP ((uint32_t)0x00000800) =DBGMCU_TIM3_STOP ((uint32_t)0x00001000) >DBGMCU_TIM4_STOP ((uint32_t)0x00002000) ?DBGMCU_CAN1_STOP ((uint32_t)0x00004000) @DBGMCU_I2C1_SMBUS_TIMEOUT ((uint32_t)0x00008000) ADBGMCU_I2C2_SMBUS_TIMEOUT ((uint32_t)0x00010000) BDBGMCU_TIM8_STOP ((uint32_t)0x00020000) CDBGMCU_TIM5_STOP ((uint32_t)0x00040000) DDBGMCU_TIM6_STOP ((uint32_t)0x00080000) EDBGMCU_TIM7_STOP ((uint32_t)0x00100000) FDBGMCU_CAN2_STOP ((uint32_t)0x00200000) GDBGMCU_TIM15_STOP ((uint32_t)0x00400000) HDBGMCU_TIM16_STOP ((uint32_t)0x00800000) IDBGMCU_TIM17_STOP ((uint32_t)0x01000000) JDBGMCU_TIM12_STOP ((uint32_t)0x02000000) KDBGMCU_TIM13_STOP ((uint32_t)0x04000000) LDBGMCU_TIM14_STOP ((uint32_t)0x08000000) MDBGMCU_TIM9_STOP ((uint32_t)0x10000000) NDBGMCU_TIM10_STOP ((uint32_t)0x20000000) ODBGMCU_TIM11_STOP ((uint32_t)0x40000000) QIS_DBGMCU_PERIPH(PERIPH) ((((PERIPH) & 0x800000F8) == 0x00) && ((PERIPH) != 0x00))        „   ‚   ·   Ë       
..\Source\STM32F10x_FWLib\inc\stm32f10x_dma.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle         *–,DMA_PeripheralBaseAddr _  # DMA_MemoryBaseAddr _  #DMA_DIR _  #DMA_BufferSize _  #DMA_PeripheralInc _  #DMA_MemoryInc _  #DMA_PeripheralDataSize _  #DMA_MemoryDataSize _  #DMA_Mode _  # DMA_Priority _  #$DMA_M2M _  #( PDMA_InitTypeDef ø U     p    d            ..\Source\STM32F10x_FWLib\inc\ ..\Source\USER\  stm32f10x_dma.h   stm32f10x.h      __STM32F10x_DMA_H   _IS_DMA_ALL_PERIPH(PERIPH) (((PERIPH) == DMA1_Channel1) || ((PERIPH) == DMA1_Channel2) || ((PERIPH) == DMA1_Channel3) || ((PERIPH) == DMA1_Channel4) || ((PERIPH) == DMA1_Channel5) || ((PERIPH) == DMA1_Channel6) || ((PERIPH) == DMA1_Channel7) || ((PERIPH) == DMA2_Channel1) || ((PERIPH) == DMA2_Channel2) || ((PERIPH) == DMA2_Channel3) || ((PERIPH) == DMA2_Channel4) || ((PERIPH) == DMA2_Channel5)) pDMA_DIR_PeripheralDST ((uint32_t)0x00000010) qDMA_DIR_PeripheralSRC ((uint32_t)0x00000000) rIS_DMA_DIR(DIR) (((DIR) == DMA_DIR_PeripheralDST) || ((DIR) == DMA_DIR_PeripheralSRC)) |DMA_PeripheralInc_Enable ((uint32_t)0x00000040) }DMA_PeripheralInc_Disable ((uint32_t)0x00000000) ~IS_DMA_PERIPHERAL_INC_STATE(STATE) (((STATE) == DMA_PeripheralInc_Enable) || ((STATE) == DMA_PeripheralInc_Disable)) àDMA_MemoryInc_Enable ((uint32_t)0x00000080) âDMA_MemoryInc_Disable ((uint32_t)0x00000000) äIS_DMA_MEMORY_INC_STATE(STATE) (((STATE) == DMA_MemoryInc_Enable) || ((STATE) == DMA_MemoryInc_Disable)) îDMA_PeripheralDataSize_Byte ((uint32_t)0x00000000) ïDMA_PeripheralDataSize_HalfWord ((uint32_t)0x00000100) ñDMA_PeripheralDataSize_Word ((uint32_t)0x00000200) óIS_DMA_PERIPHERAL_DATA_SIZE(SIZE) (((SIZE) == DMA_PeripheralDataSize_Byte) || ((SIZE) == DMA_PeripheralDataSize_HalfWord) || ((SIZE) == DMA_PeripheralDataSize_Word)) ¢DMA_MemoryDataSize_Byte ((uint32_t)0x00000000) £DMA_MemoryDataSize_HalfWord ((uint32_t)0x00000400) §DMA_MemoryDataSize_Word ((uint32_t)0x00000800) •IS_DMA_MEMORY_DATA_SIZE(SIZE) (((SIZE) == DMA_MemoryDataSize_Byte) || ((SIZE) == DMA_MemoryDataSize_HalfWord) || ((SIZE) == DMA_MemoryDataSize_Word)) ∞DMA_Mode_Circular ((uint32_t)0x00000020) ±DMA_Mode_Normal ((uint32_t)0x00000000) ≤IS_DMA_MODE(MODE) (((MODE) == DMA_Mode_Circular) || ((MODE) == DMA_Mode_Normal)) ªDMA_Priority_VeryHigh ((uint32_t)0x00003000) ºDMA_Priority_High ((uint32_t)0x00002000) ΩDMA_Priority_Medium ((uint32_t)0x00001000) æDMA_Priority_Low ((uint32_t)0x00000000) øIS_DMA_PRIORITY(PRIORITY) (((PRIORITY) == DMA_Priority_VeryHigh) || ((PRIORITY) == DMA_Priority_High) || ((PRIORITY) == DMA_Priority_Medium) || ((PRIORITY) == DMA_Priority_Low)) ÀDMA_M2M_Enable ((uint32_t)0x00004000) ÃDMA_M2M_Disable ((uint32_t)0x00000000) ÕIS_DMA_M2M_STATE(STATE) (((STATE) == DMA_M2M_Enable) || ((STATE) == DMA_M2M_Disable)) ◊DMA_IT_TC ((uint32_t)0x00000002) ÿDMA_IT_HT ((uint32_t)0x00000004) ŸDMA_IT_TE ((uint32_t)0x00000008) ⁄IS_DMA_CONFIG_IT(IT) ((((IT) & 0xFFFFFFF1) == 0x00) && ((IT) != 0x00)) ‹DMA1_IT_GL1 ((uint32_t)0x00000001) ›DMA1_IT_TC1 ((uint32_t)0x00000002) ﬁDMA1_IT_HT1 ((uint32_t)0x00000004) ﬂDMA1_IT_TE1 ((uint32_t)0x00000008) ‡DMA1_IT_GL2 ((uint32_t)0x00000010) ·DMA1_IT_TC2 ((uint32_t)0x00000020) ‚DMA1_IT_HT2 ((uint32_t)0x00000040) „DMA1_IT_TE2 ((uint32_t)0x00000080) ‰DMA1_IT_GL3 ((uint32_t)0x00000100) ÂDMA1_IT_TC3 ((uint32_t)0x00000200) ÊDMA1_IT_HT3 ((uint32_t)0x00000400) ÁDMA1_IT_TE3 ((uint32_t)0x00000800) ËDMA1_IT_GL4 ((uint32_t)0x00001000) ÈDMA1_IT_TC4 ((uint32_t)0x00002000) ÍDMA1_IT_HT4 ((uint32_t)0x00004000) ÎDMA1_IT_TE4 ((uint32_t)0x00008000) ÏDMA1_IT_GL5 ((uint32_t)0x00010000) ÌDMA1_IT_TC5 ((uint32_t)0x00020000) ÓDMA1_IT_HT5 ((uint32_t)0x00040000) ÔDMA1_IT_TE5 ((uint32_t)0x00080000) DMA1_IT_GL6 ((uint32_t)0x00100000) ÒDMA1_IT_TC6 ((uint32_t)0x00200000) ÚDMA1_IT_HT6 ((uint32_t)0x00400000) ÛDMA1_IT_TE6 ((uint32_t)0x00800000) ÙDMA1_IT_GL7 ((uint32_t)0x01000000) ıDMA1_IT_TC7 ((uint32_t)0x02000000) ˆDMA1_IT_HT7 ((uint32_t)0x04000000) ˜DMA1_IT_TE7 ((uint32_t)0x08000000) ˘DMA2_IT_GL1 ((uint32_t)0x10000001) ˙DMA2_IT_TC1 ((uint32_t)0x10000002) ˚DMA2_IT_HT1 ((uint32_t)0x10000004) ¸DMA2_IT_TE1 ((uint32_t)0x10000008) ˝DMA2_IT_GL2 ((uint32_t)0x10000010) ˛DMA2_IT_TC2 ((uint32_t)0x10000020) ˇDMA2_IT_HT2 ((uint32_t)0x10000040) ÄDMA2_IT_TE2 ((uint32_t)0x10000080) ÅDMA2_IT_GL3 ((uint32_t)0x10000100) ÇDMA2_IT_TC3 ((uint32_t)0x10000200) ÉDMA2_IT_HT3 ((uint32_t)0x10000400) ÑDMA2_IT_TE3 ((uint32_t)0x10000800) ÖDMA2_IT_GL4 ((uint32_t)0x10001000) ÜDMA2_IT_TC4 ((uint32_t)0x10002000) áDMA2_IT_HT4 ((uint32_t)0x10004000) àDMA2_IT_TE4 ((uint32_t)0x10008000) âDMA2_IT_GL5 ((uint32_t)0x10010000) äDMA2_IT_TC5 ((uint32_t)0x10020000) ãDMA2_IT_HT5 ((uint32_t)0x10040000) åDMA2_IT_TE5 ((uint32_t)0x10080000) éIS_DMA_CLEAR_IT(IT) (((((IT) & 0xF0000000) == 0x00) || (((IT) & 0xEFF00000) == 0x00)) && ((IT) != 0x00)) êIS_DMA_GET_IT(IT) (((IT) == DMA1_IT_GL1) || ((IT) == DMA1_IT_TC1) || ((IT) == DMA1_IT_HT1) || ((IT) == DMA1_IT_TE1) || ((IT) == DMA1_IT_GL2) || ((IT) == DMA1_IT_TC2) || ((IT) == DMA1_IT_HT2) || ((IT) == DMA1_IT_TE2) || ((IT) == DMA1_IT_GL3) || ((IT) == DMA1_IT_TC3) || ((IT) == DMA1_IT_HT3) || ((IT) == DMA1_IT_TE3) || ((IT) == DMA1_IT_GL4) || ((IT) == DMA1_IT_TC4) || ((IT) == DMA1_IT_HT4) || ((IT) == DMA1_IT_TE4) || ((IT) == DMA1_IT_GL5) || ((IT) == DMA1_IT_TC5) || ((IT) == DMA1_IT_HT5) || ((IT) == DMA1_IT_TE5) || ((IT) == DMA1_IT_GL6) || ((IT) == DMA1_IT_TC6) || ((IT) == DMA1_IT_HT6) || ((IT) == DMA1_IT_TE6) || ((IT) == DMA1_IT_GL7) || ((IT) == DMA1_IT_TC7) || ((IT) == DMA1_IT_HT7) || ((IT) == DMA1_IT_TE7) || ((IT) == DMA2_IT_GL1) || ((IT) == DMA2_IT_TC1) || ((IT) == DMA2_IT_HT1) || ((IT) == DMA2_IT_TE1) || ((IT) == DMA2_IT_GL2) || ((IT) == DMA2_IT_TC2) || ((IT) == DMA2_IT_HT2) || ((IT) == DMA2_IT_TE2) || ((IT) == DMA2_IT_GL3) || ((IT) == DMA2_IT_TC3) || ((IT) == DMA2_IT_HT3) || ((IT) == DMA2_IT_TE3) || ((IT) == DMA2_IT_GL4) || ((IT) == DMA2_IT_TC4) || ((IT) == DMA2_IT_HT4) || ((IT) == DMA2_IT_TE4) || ((IT) == DMA2_IT_GL5) || ((IT) == DMA2_IT_TC5) || ((IT) == DMA2_IT_HT5) || ((IT) == DMA2_IT_TE5)) ∞DMA1_FLAG_GL1 ((uint32_t)0x00000001) ±DMA1_FLAG_TC1 ((uint32_t)0x00000002) ≤DMA1_FLAG_HT1 ((uint32_t)0x00000004) ≥DMA1_FLAG_TE1 ((uint32_t)0x00000008) ¥DMA1_FLAG_GL2 ((uint32_t)0x00000010) µDMA1_FLAG_TC2 ((uint32_t)0x00000020) ∂DMA1_FLAG_HT2 ((uint32_t)0x00000040) ∑DMA1_FLAG_TE2 ((uint32_t)0x00000080) ∏DMA1_FLAG_GL3 ((uint32_t)0x00000100) πDMA1_FLAG_TC3 ((uint32_t)0x00000200) ∫DMA1_FLAG_HT3 ((uint32_t)0x00000400) ªDMA1_FLAG_TE3 ((uint32_t)0x00000800) ºDMA1_FLAG_GL4 ((uint32_t)0x00001000) ΩDMA1_FLAG_TC4 ((uint32_t)0x00002000) æDMA1_FLAG_HT4 ((uint32_t)0x00004000) øDMA1_FLAG_TE4 ((uint32_t)0x00008000) ¿DMA1_FLAG_GL5 ((uint32_t)0x00010000) ¡DMA1_FLAG_TC5 ((uint32_t)0x00020000) ¬DMA1_FLAG_HT5 ((uint32_t)0x00040000) √DMA1_FLAG_TE5 ((uint32_t)0x00080000) ƒDMA1_FLAG_GL6 ((uint32_t)0x00100000) ≈DMA1_FLAG_TC6 ((uint32_t)0x00200000) ∆DMA1_FLAG_HT6 ((uint32_t)0x00400000) «DMA1_FLAG_TE6 ((uint32_t)0x00800000) »DMA1_FLAG_GL7 ((uint32_t)0x01000000) …DMA1_FLAG_TC7 ((uint32_t)0x02000000)  DMA1_FLAG_HT7 ((uint32_t)0x04000000) ÀDMA1_FLAG_TE7 ((uint32_t)0x08000000) ÕDMA2_FLAG_GL1 ((uint32_t)0x10000001) ŒDMA2_FLAG_TC1 ((uint32_t)0x10000002) œDMA2_FLAG_HT1 ((uint32_t)0x10000004) –DMA2_FLAG_TE1 ((uint32_t)0x10000008) —DMA2_FLAG_GL2 ((uint32_t)0x10000010) “DMA2_FLAG_TC2 ((uint32_t)0x10000020) ”DMA2_FLAG_HT2 ((uint32_t)0x10000040) ‘DMA2_FLAG_TE2 ((uint32_t)0x10000080) ’DMA2_FLAG_GL3 ((uint32_t)0x10000100) ÷DMA2_FLAG_TC3 ((uint32_t)0x10000200) ◊DMA2_FLAG_HT3 ((uint32_t)0x10000400) ÿDMA2_FLAG_TE3 ((uint32_t)0x10000800) ŸDMA2_FLAG_GL4 ((uint32_t)0x10001000) ⁄DMA2_FLAG_TC4 ((uint32_t)0x10002000) €DMA2_FLAG_HT4 ((uint32_t)0x10004000) ‹DMA2_FLAG_TE4 ((uint32_t)0x10008000) ›DMA2_FLAG_GL5 ((uint32_t)0x10010000) ﬁDMA2_FLAG_TC5 ((uint32_t)0x10020000) ﬂDMA2_FLAG_HT5 ((uint32_t)0x10040000) ‡DMA2_FLAG_TE5 ((uint32_t)0x10080000) ‚IS_DMA_CLEAR_FLAG(FLAG) (((((FLAG) & 0xF0000000) == 0x00) || (((FLAG) & 0xEFF00000) == 0x00)) && ((FLAG) != 0x00)) ‰IS_DMA_GET_FLAG(FLAG) (((FLAG) == DMA1_FLAG_GL1) || ((FLAG) == DMA1_FLAG_TC1) || ((FLAG) == DMA1_FLAG_HT1) || ((FLAG) == DMA1_FLAG_TE1) || ((FLAG) == DMA1_FLAG_GL2) || ((FLAG) == DMA1_FLAG_TC2) || ((FLAG) == DMA1_FLAG_HT2) || ((FLAG) == DMA1_FLAG_TE2) || ((FLAG) == DMA1_FLAG_GL3) || ((FLAG) == DMA1_FLAG_TC3) || ((FLAG) == DMA1_FLAG_HT3) || ((FLAG) == DMA1_FLAG_TE3) || ((FLAG) == DMA1_FLAG_GL4) || ((FLAG) == DMA1_FLAG_TC4) || ((FLAG) == DMA1_FLAG_HT4) || ((FLAG) == DMA1_FLAG_TE4) || ((FLAG) == DMA1_FLAG_GL5) || ((FLAG) == DMA1_FLAG_TC5) || ((FLAG) == DMA1_FLAG_HT5) || ((FLAG) == DMA1_FLAG_TE5) || ((FLAG) == DMA1_FLAG_GL6) || ((FLAG) == DMA1_FLAG_TC6) || ((FLAG) == DMA1_FLAG_HT6) || ((FLAG) == DMA1_FLAG_TE6) || ((FLAG) == DMA1_FLAG_GL7) || ((FLAG) == DMA1_FLAG_TC7) || ((FLAG) == DMA1_FLAG_HT7) || ((FLAG) == DMA1_FLAG_TE7) || ((FLAG) == DMA2_FLAG_GL1) || ((FLAG) == DMA2_FLAG_TC1) || ((FLAG) == DMA2_FLAG_HT1) || ((FLAG) == DMA2_FLAG_TE1) || ((FLAG) == DMA2_FLAG_GL2) || ((FLAG) == DMA2_FLAG_TC2) || ((FLAG) == DMA2_FLAG_HT2) || ((FLAG) == DMA2_FLAG_TE2) || ((FLAG) == DMA2_FLAG_GL3) || ((FLAG) == DMA2_FLAG_TC3) || ((FLAG) == DMA2_FLAG_HT3) || ((FLAG) == DMA2_FLAG_TE3) || ((FLAG) == DMA2_FLAG_GL4) || ((FLAG) == DMA2_FLAG_TC4) || ((FLAG) == DMA2_FLAG_HT4) || ((FLAG) == DMA2_FLAG_TE4) || ((FLAG) == DMA2_FLAG_GL5) || ((FLAG) == DMA2_FLAG_TC5) || ((FLAG) == DMA2_FLAG_HT5) || ((FLAG) == DMA2_FLAG_TE5)) ÑIS_DMA_BUFFER_SIZE(SIZE) (((SIZE) >= 0x1) && ((SIZE) < 0x10000))        Á   Ê   Â   ‡       
..\Source\STM32F10x_FWLib\inc\stm32f10x_exti.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle         ÔEXTI_Mode_Interrupt  EXTI_Mode_Event  PEXTIMode_TypeDef ¿ 6⁄EXTI_Trigger_Rising EXTI_Trigger_Falling EXTI_Trigger_Rising_Falling  PEXTITrigger_TypeDef C*»EXTI_Line _  # EXTI_Mode Ô #EXTI_Trigger Z#EXTI_LineCmd ı  # PEXTI_InitTypeDef uY    p    e            ..\Source\STM32F10x_FWLib\inc\ ..\Source\USER\  stm32f10x_exti.h   stm32f10x.h      __STM32F10x_EXTI_H   8IS_EXTI_MODE(MODE) (((MODE) == EXTI_Mode_Interrupt) || ((MODE) == EXTI_Mode_Event)) EIS_EXTI_TRIGGER(TRIGGER) (((TRIGGER) == EXTI_Trigger_Rising) || ((TRIGGER) == EXTI_Trigger_Falling) || ((TRIGGER) == EXTI_Trigger_Rising_Falling)) gEXTI_Line0 ((uint32_t)0x00001) hEXTI_Line1 ((uint32_t)0x00002) iEXTI_Line2 ((uint32_t)0x00004) jEXTI_Line3 ((uint32_t)0x00008) kEXTI_Line4 ((uint32_t)0x00010) lEXTI_Line5 ((uint32_t)0x00020) mEXTI_Line6 ((uint32_t)0x00040) nEXTI_Line7 ((uint32_t)0x00080) oEXTI_Line8 ((uint32_t)0x00100) pEXTI_Line9 ((uint32_t)0x00200) qEXTI_Line10 ((uint32_t)0x00400) rEXTI_Line11 ((uint32_t)0x00800) sEXTI_Line12 ((uint32_t)0x01000) tEXTI_Line13 ((uint32_t)0x02000) uEXTI_Line14 ((uint32_t)0x04000) vEXTI_Line15 ((uint32_t)0x08000) wEXTI_Line16 ((uint32_t)0x10000) xEXTI_Line17 ((uint32_t)0x20000) yEXTI_Line18 ((uint32_t)0x40000) {EXTI_Line19 ((uint32_t)0x80000) }IS_EXTI_LINE(LINE) ((((LINE) & (uint32_t)0xFFF00000) == 0x00) && ((LINE) != (uint16_t)0x00)) ~IS_GET_EXTI_LINE(LINE) (((LINE) == EXTI_Line0) || ((LINE) == EXTI_Line1) || ((LINE) == EXTI_Line2) || ((LINE) == EXTI_Line3) || ((LINE) == EXTI_Line4) || ((LINE) == EXTI_Line5) || ((LINE) == EXTI_Line6) || ((LINE) == EXTI_Line7) || ((LINE) == EXTI_Line8) || ((LINE) == EXTI_Line9) || ((LINE) == EXTI_Line10) || ((LINE) == EXTI_Line11) || ((LINE) == EXTI_Line12) || ((LINE) == EXTI_Line13) || ((LINE) == EXTI_Line14) || ((LINE) == EXTI_Line15) || ((LINE) == EXTI_Line16) || ((LINE) == EXTI_Line17) || ((LINE) == EXTI_Line18) || ((LINE) == EXTI_Line19))        Î   Í   È   0       
..\Source\STM32F10x_FWLib\inc\stm32f10x_flash.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle         úFLASH_BUSY FLASH_ERROR_PG FLASH_ERROR_WRP FLASH_COMPLETE FLASH_TIMEOUT  PFLASH_Status ¡ 9    p    f            ..\Source\STM32F10x_FWLib\inc\ ..\Source\USER\  stm32f10x_flash.h   stm32f10x.h      __STM32F10x_FLASH_H   GFLASH_Latency_0 ((uint32_t)0x00000000) HFLASH_Latency_1 ((uint32_t)0x00000001) IFLASH_Latency_2 ((uint32_t)0x00000002) JIS_FLASH_LATENCY(LATENCY) (((LATENCY) == FLASH_Latency_0) || ((LATENCY) == FLASH_Latency_1) || ((LATENCY) == FLASH_Latency_2)) UFLASH_HalfCycleAccess_Enable ((uint32_t)0x00000008) VFLASH_HalfCycleAccess_Disable ((uint32_t)0x00000000) WIS_FLASH_HALFCYCLEACCESS_STATE(STATE) (((STATE) == FLASH_HalfCycleAccess_Enable) || ((STATE) == FLASH_HalfCycleAccess_Disable)) aFLASH_PrefetchBuffer_Enable ((uint32_t)0x00000010) bFLASH_PrefetchBuffer_Disable ((uint32_t)0x00000000) cIS_FLASH_PREFETCHBUFFER_STATE(STATE) (((STATE) == FLASH_PrefetchBuffer_Enable) || ((STATE) == FLASH_PrefetchBuffer_Disable)) nFLASH_WRProt_Pages0to3 ((uint32_t)0x00000001) oFLASH_WRProt_Pages4to7 ((uint32_t)0x00000002) pFLASH_WRProt_Pages8to11 ((uint32_t)0x00000004) qFLASH_WRProt_Pages12to15 ((uint32_t)0x00000008) rFLASH_WRProt_Pages16to19 ((uint32_t)0x00000010) sFLASH_WRProt_Pages20to23 ((uint32_t)0x00000020) tFLASH_WRProt_Pages24to27 ((uint32_t)0x00000040) uFLASH_WRProt_Pages28to31 ((uint32_t)0x00000080) xFLASH_WRProt_Pages32to35 ((uint32_t)0x00000100) yFLASH_WRProt_Pages36to39 ((uint32_t)0x00000200) zFLASH_WRProt_Pages40to43 ((uint32_t)0x00000400) {FLASH_WRProt_Pages44to47 ((uint32_t)0x00000800) |FLASH_WRProt_Pages48to51 ((uint32_t)0x00001000) }FLASH_WRProt_Pages52to55 ((uint32_t)0x00002000) ~FLASH_WRProt_Pages56to59 ((uint32_t)0x00004000) FLASH_WRProt_Pages60to63 ((uint32_t)0x00008000) ÄFLASH_WRProt_Pages64to67 ((uint32_t)0x00010000) ÅFLASH_WRProt_Pages68to71 ((uint32_t)0x00020000) ÇFLASH_WRProt_Pages72to75 ((uint32_t)0x00040000) ÉFLASH_WRProt_Pages76to79 ((uint32_t)0x00080000) ÑFLASH_WRProt_Pages80to83 ((uint32_t)0x00100000) ÖFLASH_WRProt_Pages84to87 ((uint32_t)0x00200000) ÜFLASH_WRProt_Pages88to91 ((uint32_t)0x00400000) áFLASH_WRProt_Pages92to95 ((uint32_t)0x00800000) àFLASH_WRProt_Pages96to99 ((uint32_t)0x01000000) âFLASH_WRProt_Pages100to103 ((uint32_t)0x02000000) äFLASH_WRProt_Pages104to107 ((uint32_t)0x04000000) ãFLASH_WRProt_Pages108to111 ((uint32_t)0x08000000) åFLASH_WRProt_Pages112to115 ((uint32_t)0x10000000) çFLASH_WRProt_Pages116to119 ((uint32_t)0x20000000) éFLASH_WRProt_Pages120to123 ((uint32_t)0x40000000) èFLASH_WRProt_Pages124to127 ((uint32_t)0x80000000) íFLASH_WRProt_Pages0to1 ((uint32_t)0x00000001) îFLASH_WRProt_Pages2to3 ((uint32_t)0x00000002) ñFLASH_WRProt_Pages4to5 ((uint32_t)0x00000004) òFLASH_WRProt_Pages6to7 ((uint32_t)0x00000008) öFLASH_WRProt_Pages8to9 ((uint32_t)0x00000010) úFLASH_WRProt_Pages10to11 ((uint32_t)0x00000020) ûFLASH_WRProt_Pages12to13 ((uint32_t)0x00000040) †FLASH_WRProt_Pages14to15 ((uint32_t)0x00000080) ¢FLASH_WRProt_Pages16to17 ((uint32_t)0x00000100) §FLASH_WRProt_Pages18to19 ((uint32_t)0x00000200) ¶FLASH_WRProt_Pages20to21 ((uint32_t)0x00000400) ®FLASH_WRProt_Pages22to23 ((uint32_t)0x00000800) ™FLASH_WRProt_Pages24to25 ((uint32_t)0x00001000) ¨FLASH_WRProt_Pages26to27 ((uint32_t)0x00002000) ÆFLASH_WRProt_Pages28to29 ((uint32_t)0x00004000) ∞FLASH_WRProt_Pages30to31 ((uint32_t)0x00008000) ≤FLASH_WRProt_Pages32to33 ((uint32_t)0x00010000) ¥FLASH_WRProt_Pages34to35 ((uint32_t)0x00020000) ∂FLASH_WRProt_Pages36to37 ((uint32_t)0x00040000) ∏FLASH_WRProt_Pages38to39 ((uint32_t)0x00080000) ∫FLASH_WRProt_Pages40to41 ((uint32_t)0x00100000) ºFLASH_WRProt_Pages42to43 ((uint32_t)0x00200000) æFLASH_WRProt_Pages44to45 ((uint32_t)0x00400000) ¿FLASH_WRProt_Pages46to47 ((uint32_t)0x00800000) ¬FLASH_WRProt_Pages48to49 ((uint32_t)0x01000000) ƒFLASH_WRProt_Pages50to51 ((uint32_t)0x02000000) ∆FLASH_WRProt_Pages52to53 ((uint32_t)0x04000000) »FLASH_WRProt_Pages54to55 ((uint32_t)0x08000000)  FLASH_WRProt_Pages56to57 ((uint32_t)0x10000000) ÃFLASH_WRProt_Pages58to59 ((uint32_t)0x20000000) ŒFLASH_WRProt_Pages60to61 ((uint32_t)0x40000000) –FLASH_WRProt_Pages62to127 ((uint32_t)0x80000000) —FLASH_WRProt_Pages62to255 ((uint32_t)0x80000000) “FLASH_WRProt_Pages62to511 ((uint32_t)0x80000000) ‘FLASH_WRProt_AllPages ((uint32_t)0xFFFFFFFF) ÷IS_FLASH_WRPROT_PAGE(PAGE) (((PAGE) != 0x00000000)) ÿIS_FLASH_ADDRESS(ADDRESS) (((ADDRESS) >= 0x08000000) && ((ADDRESS) < 0x080FFFFF)) ⁄IS_OB_DATA_ADDRESS(ADDRESS) (((ADDRESS) == 0x1FFFF804) || ((ADDRESS) == 0x1FFFF806)) ‰OB_IWDG_SW ((uint16_t)0x0001) ÂOB_IWDG_HW ((uint16_t)0x0000) ÊIS_OB_IWDG_SOURCE(SOURCE) (((SOURCE) == OB_IWDG_SW) || ((SOURCE) == OB_IWDG_HW)) OB_STOP_NoRST ((uint16_t)0x0002) ÒOB_STOP_RST ((uint16_t)0x0000) ÚIS_OB_STOP_SOURCE(SOURCE) (((SOURCE) == OB_STOP_NoRST) || ((SOURCE) == OB_STOP_RST)) ¸OB_STDBY_NoRST ((uint16_t)0x0004) ˝OB_STDBY_RST ((uint16_t)0x0000) ˛IS_OB_STDBY_SOURCE(SOURCE) (((SOURCE) == OB_STDBY_NoRST) || ((SOURCE) == OB_STDBY_RST)) üFLASH_IT_ERROR ((uint32_t)0x00000400) †FLASH_IT_EOP ((uint32_t)0x00001000) °FLASH_IT_BANK1_ERROR FLASH_IT_ERROR ¢FLASH_IT_BANK1_EOP FLASH_IT_EOP §IS_FLASH_IT(IT) ((((IT) & (uint32_t)0xFFFFEBFF) == 0x00000000) && (((IT) != 0x00000000))) »FLASH_FLAG_BSY ((uint32_t)0x00000001) …FLASH_FLAG_EOP ((uint32_t)0x00000020)  FLASH_FLAG_PGERR ((uint32_t)0x00000004) ÀFLASH_FLAG_WRPRTERR ((uint32_t)0x00000010) ÃFLASH_FLAG_OPTERR ((uint32_t)0x00000001) ŒFLASH_FLAG_BANK1_BSY FLASH_FLAG_BSY œFLASH_FLAG_BANK1_EOP FLASH_FLAG_EOP –FLASH_FLAG_BANK1_PGERR FLASH_FLAG_PGERR —FLASH_FLAG_BANK1_WRPRTERR FLASH_FLAG_WRPRTERR ”IS_FLASH_CLEAR_FLAG(FLAG) ((((FLAG) & (uint32_t)0xFFFFFFCA) == 0x00000000) && ((FLAG) != 0x00000000)) ‘IS_FLASH_GET_FLAG(FLAG) (((FLAG) == FLASH_FLAG_BSY) || ((FLAG) == FLASH_FLAG_EOP) || ((FLAG) == FLASH_FLAG_PGERR) || ((FLAG) == FLASH_FLAG_WRPRTERR) || ((FLAG) == FLASH_FLAG_BANK1_BSY) || ((FLAG) == FLASH_FLAG_BANK1_EOP) || ((FLAG) == FLASH_FLAG_BANK1_PGERR) || ((FLAG) == FLASH_FLAG_BANK1_WRPRTERR) || ((FLAG) == FLASH_FLAG_OPTERR))       Ô   Ó   Ì          
..\Source\STM32F10x_FWLib\inc\stm32f10x_fsmc.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle         *èFSMC_AddressSetupTime _  # FSMC_AddressHoldTime _  #FSMC_DataSetupTime _  #FSMC_BusTurnAroundDuration _  #FSMC_CLKDivision _  #FSMC_DataLatency _  #FSMC_AccessMode _  # PFSMC_NORSRAMTimingInitTypeDef ¿ V*ﬁ<FSMC_Bank _  # FSMC_DataAddressMux _  #FSMC_MemoryType _  #FSMC_MemoryDataWidth _  #FSMC_BurstAccessMode _  #FSMC_AsynchronousWait _  #FSMC_WaitSignalPolarity _  #FSMC_WrapMode _  #FSMC_WaitSignalActive _  # FSMC_WriteOperation _  #$FSMC_WaitSignal _  #(FSMC_ExtendedMode _  #,FSMC_WriteBurst _  #0FSMC_ReadWriteTimingStruct ^#4FSMC_WriteTimingStruct ^#8 "èPFSMC_NORSRAMInitTypeDef ¥ë*ÚFSMC_SetupTime _  # FSMC_WaitSetupTime _  #FSMC_HoldSetupTime _  #FSMC_HiZSetupTime _  # PFSMC_NAND_PCCARDTimingInitTypeDef Ç±*ö
$FSMC_Bank _  # FSMC_Waitfeature _  #FSMC_MemoryDataWidth _  #FSMC_ECC _  #FSMC_ECCPageSize _  #FSMC_TCLRSetupTime _  #FSMC_TARSetupTime _  #FSMC_CommonSpaceTimingStruct #FSMC_AttributeSpaceTimingStruct #  "ÚPFSMC_NANDInitTypeDef ”*¸FSMC_Waitfeature _  # FSMC_TCLRSetupTime _  #FSMC_TARSetupTime _  #FSMC_CommonSpaceTimingStruct #FSMC_AttributeSpaceTimingStruct #FSMC_IOSpaceTimingStruct # PFSMC_PCCARDInitTypeDef ;Ï     p    e            ..\Source\STM32F10x_FWLib\inc\ ..\Source\USER\  stm32f10x_fsmc.h   stm32f10x.h      __STM32F10x_FSMC_H   ˘FSMC_Bank1_NORSRAM1 ((uint32_t)0x00000000) ˙FSMC_Bank1_NORSRAM2 ((uint32_t)0x00000002) ˚FSMC_Bank1_NORSRAM3 ((uint32_t)0x00000004) ¸FSMC_Bank1_NORSRAM4 ((uint32_t)0x00000006) ÑFSMC_Bank2_NAND ((uint32_t)0x00000010) ÖFSMC_Bank3_NAND ((uint32_t)0x00000100) çFSMC_Bank4_PCCARD ((uint32_t)0x00001000) íIS_FSMC_NORSRAM_BANK(BANK) (((BANK) == FSMC_Bank1_NORSRAM1) || ((BANK) == FSMC_Bank1_NORSRAM2) || ((BANK) == FSMC_Bank1_NORSRAM3) || ((BANK) == FSMC_Bank1_NORSRAM4)) óIS_FSMC_NAND_BANK(BANK) (((BANK) == FSMC_Bank2_NAND) || ((BANK) == FSMC_Bank3_NAND)) öIS_FSMC_GETFLAG_BANK(BANK) (((BANK) == FSMC_Bank2_NAND) || ((BANK) == FSMC_Bank3_NAND) || ((BANK) == FSMC_Bank4_PCCARD)) ûIS_FSMC_IT_BANK(BANK) (((BANK) == FSMC_Bank2_NAND) || ((BANK) == FSMC_Bank3_NAND) || ((BANK) == FSMC_Bank4_PCCARD)) ™FSMC_DataAddressMux_Disable ((uint32_t)0x00000000) ´FSMC_DataAddressMux_Enable ((uint32_t)0x00000002) ¨IS_FSMC_MUX(MUX) (((MUX) == FSMC_DataAddressMux_Disable) || ((MUX) == FSMC_DataAddressMux_Enable)) ∑FSMC_MemoryType_SRAM ((uint32_t)0x00000000) ∏FSMC_MemoryType_PSRAM ((uint32_t)0x00000004) πFSMC_MemoryType_NOR ((uint32_t)0x00000008) ∫IS_FSMC_MEMORY(MEMORY) (((MEMORY) == FSMC_MemoryType_SRAM) || ((MEMORY) == FSMC_MemoryType_PSRAM)|| ((MEMORY) == FSMC_MemoryType_NOR)) ∆FSMC_MemoryDataWidth_8b ((uint32_t)0x00000000) «FSMC_MemoryDataWidth_16b ((uint32_t)0x00000010) »IS_FSMC_MEMORY_WIDTH(WIDTH) (((WIDTH) == FSMC_MemoryDataWidth_8b) || ((WIDTH) == FSMC_MemoryDataWidth_16b)) ”FSMC_BurstAccessMode_Disable ((uint32_t)0x00000000) ‘FSMC_BurstAccessMode_Enable ((uint32_t)0x00000100) ’IS_FSMC_BURSTMODE(STATE) (((STATE) == FSMC_BurstAccessMode_Disable) || ((STATE) == FSMC_BurstAccessMode_Enable)) ﬁFSMC_AsynchronousWait_Disable ((uint32_t)0x00000000) ﬂFSMC_AsynchronousWait_Enable ((uint32_t)0x00008000) ‡IS_FSMC_ASYNWAIT(STATE) (((STATE) == FSMC_AsynchronousWait_Disable) || ((STATE) == FSMC_AsynchronousWait_Enable)) ÎFSMC_WaitSignalPolarity_Low ((uint32_t)0x00000000) ÏFSMC_WaitSignalPolarity_High ((uint32_t)0x00000200) ÌIS_FSMC_WAIT_POLARITY(POLARITY) (((POLARITY) == FSMC_WaitSignalPolarity_Low) || ((POLARITY) == FSMC_WaitSignalPolarity_High)) ¯FSMC_WrapMode_Disable ((uint32_t)0x00000000) ˘FSMC_WrapMode_Enable ((uint32_t)0x00000400) ˙IS_FSMC_WRAP_MODE(MODE) (((MODE) == FSMC_WrapMode_Disable) || ((MODE) == FSMC_WrapMode_Enable)) ÖFSMC_WaitSignalActive_BeforeWaitState ((uint32_t)0x00000000) ÜFSMC_WaitSignalActive_DuringWaitState ((uint32_t)0x00000800) áIS_FSMC_WAIT_SIGNAL_ACTIVE(ACTIVE) (((ACTIVE) == FSMC_WaitSignalActive_BeforeWaitState) || ((ACTIVE) == FSMC_WaitSignalActive_DuringWaitState)) íFSMC_WriteOperation_Disable ((uint32_t)0x00000000) ìFSMC_WriteOperation_Enable ((uint32_t)0x00001000) îIS_FSMC_WRITE_OPERATION(OPERATION) (((OPERATION) == FSMC_WriteOperation_Disable) || ((OPERATION) == FSMC_WriteOperation_Enable)) üFSMC_WaitSignal_Disable ((uint32_t)0x00000000) †FSMC_WaitSignal_Enable ((uint32_t)0x00002000) °IS_FSMC_WAITE_SIGNAL(SIGNAL) (((SIGNAL) == FSMC_WaitSignal_Disable) || ((SIGNAL) == FSMC_WaitSignal_Enable)) ´FSMC_ExtendedMode_Disable ((uint32_t)0x00000000) ¨FSMC_ExtendedMode_Enable ((uint32_t)0x00004000) ÆIS_FSMC_EXTENDED_MODE(MODE) (((MODE) == FSMC_ExtendedMode_Disable) || ((MODE) == FSMC_ExtendedMode_Enable)) πFSMC_WriteBurst_Disable ((uint32_t)0x00000000) ∫FSMC_WriteBurst_Enable ((uint32_t)0x00080000) ªIS_FSMC_WRITE_BURST(BURST) (((BURST) == FSMC_WriteBurst_Disable) || ((BURST) == FSMC_WriteBurst_Enable)) ≈IS_FSMC_ADDRESS_SETUP_TIME(TIME) ((TIME) <= 0xF) œIS_FSMC_ADDRESS_HOLD_TIME(TIME) ((TIME) <= 0xF) ŸIS_FSMC_DATASETUP_TIME(TIME) (((TIME) > 0) && ((TIME) <= 0xFF)) „IS_FSMC_TURNAROUND_TIME(TIME) ((TIME) <= 0xF) ÌIS_FSMC_CLK_DIV(DIV) ((DIV) <= 0xF) ˜IS_FSMC_DATA_LATENCY(LATENCY) ((LATENCY) <= 0xF) ÅFSMC_AccessMode_A ((uint32_t)0x00000000) ÇFSMC_AccessMode_B ((uint32_t)0x10000000) ÉFSMC_AccessMode_C ((uint32_t)0x20000000) ÑFSMC_AccessMode_D ((uint32_t)0x30000000) ÖIS_FSMC_ACCESS_MODE(MODE) (((MODE) == FSMC_AccessMode_A) || ((MODE) == FSMC_AccessMode_B) || ((MODE) == FSMC_AccessMode_C) || ((MODE) == FSMC_AccessMode_D)) öFSMC_Waitfeature_Disable ((uint32_t)0x00000000) õFSMC_Waitfeature_Enable ((uint32_t)0x00000002) úIS_FSMC_WAIT_FEATURE(FEATURE) (((FEATURE) == FSMC_Waitfeature_Disable) || ((FEATURE) == FSMC_Waitfeature_Enable)) ®FSMC_ECC_Disable ((uint32_t)0x00000000) ©FSMC_ECC_Enable ((uint32_t)0x00000040) ™IS_FSMC_ECC_STATE(STATE) (((STATE) == FSMC_ECC_Disable) || ((STATE) == FSMC_ECC_Enable)) µFSMC_ECCPageSize_256Bytes ((uint32_t)0x00000000) ∂FSMC_ECCPageSize_512Bytes ((uint32_t)0x00020000) ∑FSMC_ECCPageSize_1024Bytes ((uint32_t)0x00040000) ∏FSMC_ECCPageSize_2048Bytes ((uint32_t)0x00060000) πFSMC_ECCPageSize_4096Bytes ((uint32_t)0x00080000) ∫FSMC_ECCPageSize_8192Bytes ((uint32_t)0x000A0000) ªIS_FSMC_ECCPAGE_SIZE(SIZE) (((SIZE) == FSMC_ECCPageSize_256Bytes) || ((SIZE) == FSMC_ECCPageSize_512Bytes) || ((SIZE) == FSMC_ECCPageSize_1024Bytes) || ((SIZE) == FSMC_ECCPageSize_2048Bytes) || ((SIZE) == FSMC_ECCPageSize_4096Bytes) || ((SIZE) == FSMC_ECCPageSize_8192Bytes))  IS_FSMC_TCLR_TIME(TIME) ((TIME) <= 0xFF) ‘IS_FSMC_TAR_TIME(TIME) ((TIME) <= 0xFF) ﬁIS_FSMC_SETUP_TIME(TIME) ((TIME) <= 0xFF) ËIS_FSMC_WAIT_TIME(TIME) ((TIME) <= 0xFF) ÚIS_FSMC_HOLD_TIME(TIME) ((TIME) <= 0xFF) ¸IS_FSMC_HIZ_TIME(TIME) ((TIME) <= 0xFF) ÜFSMC_IT_RisingEdge ((uint32_t)0x00000008) áFSMC_IT_Level ((uint32_t)0x00000010) àFSMC_IT_FallingEdge ((uint32_t)0x00000020) âIS_FSMC_IT(IT) ((((IT) & (uint32_t)0xFFFFFFC7) == 0x00000000) && ((IT) != 0x00000000)) äIS_FSMC_GET_IT(IT) (((IT) == FSMC_IT_RisingEdge) || ((IT) == FSMC_IT_Level) || ((IT) == FSMC_IT_FallingEdge)) ïFSMC_FLAG_RisingEdge ((uint32_t)0x00000001) ñFSMC_FLAG_Level ((uint32_t)0x00000002) óFSMC_FLAG_FallingEdge ((uint32_t)0x00000004) òFSMC_FLAG_FEMPT ((uint32_t)0x00000040) ôIS_FSMC_GET_FLAG(FLAG) (((FLAG) == FSMC_FLAG_RisingEdge) || ((FLAG) == FSMC_FLAG_Level) || ((FLAG) == FSMC_FLAG_FallingEdge) || ((FLAG) == FSMC_FLAG_FEMPT)) ûIS_FSMC_CLEAR_FLAG(FLAG) ((((FLAG) & (uint32_t)0xFFFFFFF8) == 0x00000000) && ((FLAG) != 0x00000000))       Û   Ú   Ò   P       
..\Source\STM32F10x_FWLib\inc\stm32f10x_gpio.h Component: ARM Compiler 5.06 update 6 (build 750) Tool: ArmCC [4d3637]  D:\FILMGEAR\F_J_Y-C_O_L_O_R-L_I_G_H_T-001\Prj_panle         ÄGPIO_Speed_10MHz GPIO_Speed_2MHz GPIO_Speed_50MHz  PGPIOSpeed_TypeDef ¿ ?∏GPIO_Mode_AIN  GPIO_Mode_IN_FLOATING GPIO_Mode_IPD (GPIO_Mode_IPU HGPIO_Mode_Out_OD GPIO_Mode_Out_PP GPIO_Mode_AF_OD GPIO_Mode_AF_PP  PGPIOMode_TypeDef P*äGPIO_Pin O  # GPIO_Speed  #GPIO_Mode ∏# PGPIO_InitTypeDef –eøBit_RESET  Bit_SET  PBitAction "o    p    e            ..\Source\STM32F10x_FWLib\inc\ ..\Source\USER\  stm32f10x_gpio.h   stm32f10x.h      __STM32F10x_GPIO_H   .IS_GPIO_ALL_PERIPH(PERIPH) (((PERIPH) == GPIOA) || ((PERIPH) == GPIOB) || ((PERIPH) == GPIOC) || ((PERIPH) == GPIOD) || ((PERIPH) == GPIOE) || ((PERIPH) == GPIOF) || ((PERIPH) == GPIOG)) @IS_GPIO_SPEED(SPEED) (((SPEED) == GPIO_Speed_10MHz) || ((SPEED) == GPIO_Speed_2MHz) || ((SPEED) == GPIO_Speed_50MHz)) RIS_GPIO_MODE(MODE) (((MODE) == GPIO_Mode_AIN) || ((MODE) == GPIO_Mode_IN_FLOATING) || ((MODE) == GPIO_Mode_IPD) || ((MODE) == GPIO_Mode_IPU) || ((MODE) == GPIO_Mode_Out_OD) || ((MODE) == GPIO_Mode_Out_PP) || ((MODE) == GPIO_Mode_AF_OD) || ((MODE) == GPIO_Mode_AF_PP)) qIS_GPIO_BIT_ACTION(ACTION) (((ACTION) == Bit_RESET) || ((ACTION) == Bit_SET)) GPIO_Pin_0 ((uint16_t)0x0001) ÄGPIO_Pin_1 ((uint16_t)0x0002) ÅGPIO_Pin_2 ((uint16_t)0x0004) ÇGPIO_Pin_3 ((uint16_t)0x0008) ÉGPIO_Pin_4 ((uint16_t)0x0010) ÑGPIO_Pin_5 ((uint16_t)0x0020) ÖGPIO_Pin_6 ((uint16_t)0x0040) ÜGPIO_Pin_7 ((uint16_t)0x0080) áGPIO_Pin_8 ((uint16_t)0x0100) àGPIO_Pin_9 ((uint16_t)0x0200) âGPIO_Pin_10 ((uint16_t)0x0400) äGPIO_Pin_11 ((uint16_t)0x0800) ãGPIO_Pin_12 ((uint16_t)0x1000) åGPIO_Pin_13 ((uint16_t)0x2000) çGPIO_Pin_14 ((uint16_t)0x4000) éGPIO_Pin_15 ((uint16_t)0x8000) èGPIO_Pin_All ((uint16_t)0xFFFF) ëIS_GPIO_PIN(PIN) ((((PIN) & (uint16_t)0x00) == 0x00) && ((PIN) != (uint16_t)0x00)) ìIS_GET_GPIO_PIN(PIN) (((PIN) == GPIO_Pin_0) || ((PIN) == GPIO_Pin_1) || ((PIN) == GPIO_Pin_2) || ((PIN) == GPIO_Pin_3) || ((PIN) == GPIO_Pin_4) || ((PIN) == GPIO_Pin_5) || ((PIN) == GPIO_Pin_6) || ((PIN) == GPIO_Pin_7) || ((PIN) == GPIO_Pin_8) || ((PIN) == GPIO_Pin_9) || ((PIN) == GPIO_Pin_10) || ((PIN) == GPIO_Pin_11) || ((PIN) == GPIO_Pin_12) || ((PIN) == GPIO_Pin_13) || ((PIN) == GPIO_Pin_14) || ((PIN) == GPIO_Pin_15)) ¨GPIO_Remap_SPI1 ((uint32_t)0x00000001) ≠GPIO_Remap_I2C1 ((uint32_t)0x00000002) ÆGPIO_Remap_USART1 ((uint32_t)0x00000004) ØGPIO_Remap_USART2 ((uint32_t)0x00000008) ∞GPIO_PartialRemap_USART3 ((uint32_t)0x00140010) ±GPIO_FullRemap_USART3 ((uint32_t)0x00140030) ≤GPIO_PartialRemap_TIM1 ((uint32_t)0x00160040) ≥GPIO_FullRemap_TIM1 ((uint32_t)0x001600C0) ¥GPIO_PartialRemap1_TIM2 ((uint32_t)0x00180100) µGPIO_PartialRemap2_TIM2 ((uint32_t)0x00180200) ∂GPIO_FullRemap_TIM2 ((uint32_t)0x00180300) ∑GPIO_PartialRemap_TIM3 ((uint32_t)0x001A0800) ∏GPIO_FullRemap_