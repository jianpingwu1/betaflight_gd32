/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#if defined(GD32F425) || defined(GD32F450) || defined(GD32F460) || defined(GD32F470)

#include "gd32f4xx.h"

// Chip Unique ID on F4
#define U_ID_0 (*(uint32_t*)0x1fff7a10)
#define U_ID_1 (*(uint32_t*)0x1fff7a14)
#define U_ID_2 (*(uint32_t*)0x1fff7a18)

#ifndef GD32F4
#define GD32F4
#endif

#elif defined(GD32H737) || defined(GD32H757) || defined(GD32H759)

#include "gd32h7xx.h"

// Chip Unique ID on H7xx
#define U_ID_0 (*(uint32_t*)0x1ff0f7e8)
#define U_ID_1 (*(uint32_t*)0x1ff0f7ec)
#define U_ID_2 (*(uint32_t*)0x1ff0f7f0)

#ifndef GD32H7
#define GD32H7
#endif

#define MAX_MPU_REGIONS    16

#define USE_PIN_AF

#endif

#if defined(GD32F4) || defined(GD32H7)

/* DMA data mode type */
typedef enum {
    DMA_DATA_MODE_SINGLE = 0,
    DMA_DATA_MODE_MULTI  = 1
} dma_data_mode_enum;

/* DMA general configuration struct */
typedef struct
{
    dma_data_mode_enum data_mode;
#if defined(GD32F4)
    dma_subperipheral_enum sub_periph;
#endif
    union {
        dma_single_data_parameter_struct init_struct_s;
        dma_multi_data_parameter_struct  init_struct_m;
    } config;
} dma_general_config_struct;

#endif // GD32F4 || GD32H7

#ifdef GD32F4

#define USE_FAST_DATA

#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID
#define USE_USB_MSC
#define USE_PERSISTENT_MSC_RTC
#define USE_MCO
#define USE_DMA_SPEC
#define USE_PERSISTENT_OBJECTS
#define USE_LATE_TASK_STATISTICS

#endif // GD32F4

#ifdef GD32H7

#define SPI_TRAIT_AF_PIN          1
#define I2C_TRAIT_STATE           1
#define I2C_TRAIT_AF_PIN          1
#define UART_TRAIT_AF_PIN         1
#define I2CDEV_COUNT              4

#ifdef USE_DSHOT
#define USE_DSHOT_CACHE_MGMT
#endif

#define USE_ITCM_RAM
#define USE_FAST_DATA
#define USE_RPM_FILTER
#define USE_DYN_IDLE
#define USE_DYN_NOTCH_FILTER
#define USE_ADC_INTERNAL
#define USE_USB_CDC_HID
#define USE_DMA_SPEC
#define USE_PERSISTENT_OBJECTS
#define USE_DMA_RAM
#define USE_USB_MSC
#define USE_RTC_TIME
#define USE_PERSISTENT_MSC_RTC
#define USE_LATE_TASK_STATISTICS

#endif // GD32H7

#define PLATFORM_TRAIT_ADC_DEVICE    1
#define PLATFORM_TRAIT_RCC           1
#define DMA_TRAIT_CHANNEL            1

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))
#define CLEAR_REG(REG)        ((REG) = (0x0))
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))


#define I2C_TypeDef          void
#define I2C_HandleTypeDef    void
#define GPIO_TypeDef         void
#define GPIO_InitTypeDef     void
#define TIM_TypeDef          void
#define DMA_TypeDef          void
#define DMA_Stream_TypeDef   void
#define DMA_InitTypeDef      dma_general_config_struct
#define DMA_Channel_TypeDef  void
#define SPI_TypeDef          void
#define ADC_TypeDef          void
#define USART_TypeDef        void
#define TIM_OCInitTypeDef    timer_oc_parameter_struct
#define TIM_ICInitTypeDef    timer_ic_parameter_struct
#define TIM_OCStructInit     timer_channel_output_struct_para_init
#define TIM_Cmd              void
#define TIM_CtrlPWMOutputs   void
#define TIM_TimeBaseInit     void
#define TIM_TimeBaseInitTypeDef timer_parameter_struct
#define TIM_ARRPreloadConfig  void
#define EXTI_TypeDef         void
#define EXTI_InitTypeDef     void
#if defined(GD32H7)
#define OCTOSPI_TypeDef      void
#endif

#define TIM_ICPolarity_Falling      TIMER_IC_POLARITY_FALLING
#define TIM_ICPolarity_Rising       TIMER_IC_POLARITY_RISING

// Convert peripheral pointer back to uint32_t base address for GD32 SPL API calls
#define PERIPH_INT(periph)    ((uint32_t)(periph))

// Redefine ADC peripherals as typed pointers for BF compatibility
// (GD32 library defines them as uint32_t; BF uses them as ADC_TypeDef*)
#if defined(GD32H7)
// H7: ADC registers are 0x400 apart
#define GD_ADC0    ((ADC_TypeDef*)(ADC_BASE + 0x000))
#define GD_ADC1    ((ADC_TypeDef*)(ADC_BASE + 0x400))
#define GD_ADC2    ((ADC_TypeDef*)(ADC_BASE + 0x800))
#elif defined(GD32F4)
// F4: ADC registers are 0x100 apart
#define GD_ADC0    ((ADC_TypeDef*)(ADC_BASE + 0x000))
#define GD_ADC1    ((ADC_TypeDef*)(ADC_BASE + 0x100))
#define GD_ADC2    ((ADC_TypeDef*)(ADC_BASE + 0x200))
#endif
#if defined(GD32F4) || defined(GD32H7)
#undef ADC0
#define ADC0       PERIPH_INT(GD_ADC0)
#undef ADC1
#define ADC1       PERIPH_INT(GD_ADC1)
#undef ADC2
#define ADC2       PERIPH_INT(GD_ADC2)
#endif

// Redefine SPI peripherals as typed pointers for BF compatibility
#if defined(GD32F4) || defined(GD32H7)
#define GD_SPI0    ((SPI_TypeDef*)(SPI_BASE + 0x0000F800U))
#define GD_SPI1    ((SPI_TypeDef*)(SPI_BASE + 0x00000000U))
#define GD_SPI2    ((SPI_TypeDef*)(SPI_BASE + 0x00000400U))
#define GD_SPI3    ((SPI_TypeDef*)(SPI_BASE + 0x0000FC00U))
#define GD_SPI4    ((SPI_TypeDef*)(SPI_BASE + 0x00011800U))
#if defined(GD32H7)
#define GD_SPI5    ((SPI_TypeDef*)(SPI_BASE + 0x00010000U))
#else
#define GD_SPI5    ((SPI_TypeDef*)(SPI_BASE + 0x00011C00U))
#endif
#undef SPI0
#define SPI0       GD_SPI0
#undef SPI1
#define SPI1       GD_SPI1
#undef SPI2
#define SPI2       GD_SPI2
#undef SPI3
#define SPI3       GD_SPI3
#undef SPI4
#define SPI4       GD_SPI4
#undef SPI5
#define SPI5       GD_SPI5
#endif

#if defined(GD32H7)
#define GD_OCTOSPI1    ((OCTOSPI_TypeDef*)OSPI_BASE)
#undef OSPI0
#define OSPI0          GD_OCTOSPI1
#endif

#define DMA0_CH0_BASE        (DMA0 + 0x10)
#define DMA0_CH1_BASE        (DMA0 + 0x28)
#define DMA0_CH2_BASE        (DMA0 + 0x40)
#define DMA0_CH3_BASE        (DMA0 + 0x58)
#define DMA0_CH4_BASE        (DMA0 + 0x70)
#define DMA0_CH5_BASE        (DMA0 + 0x88)
#define DMA0_CH6_BASE        (DMA0 + 0xA0)
#define DMA0_CH7_BASE        (DMA0 + 0xB8)

#define DMA1_CH0_BASE        (DMA1 + 0x10)
#define DMA1_CH1_BASE        (DMA1 + 0x28)
#define DMA1_CH2_BASE        (DMA1 + 0x40)
#define DMA1_CH3_BASE        (DMA1 + 0x58)
#define DMA1_CH4_BASE        (DMA1 + 0x70)
#define DMA1_CH5_BASE        (DMA1 + 0x88)
#define DMA1_CH6_BASE        (DMA1 + 0xA0)
#define DMA1_CH7_BASE        (DMA1 + 0xB8)


extern void gd32_timer_deinit(void *timer);
void gd32_timer_set_counter(void* timer, uint32_t counter);
#define TIM_DeInit           gd32_timer_deinit
#define TIM_SetCounter       gd32_timer_set_counter

extern uint32_t timerPrescaler(const TIM_TypeDef *tim);

#if defined(GD32H7)
#define UART_TX_BUFFER_ATTRIBUTE DMA_RAM /* SRAM0_1 SRAM */
#define UART_RX_BUFFER_ATTRIBUTE DMA_RAM /* SRAM0_1 SRAM */
#else
#define UART_TX_BUFFER_ATTRIBUTE /* EMPTY */
#define UART_RX_BUFFER_ATTRIBUTE /* EMPTY */
#endif


#define GPIOA_BASE    GPIOA



#if defined(GD32F4) || defined(GD32H7)
#define TASK_GYROPID_DESIRED_PERIOD     125 // 125us = 8kHz
#define SCHEDULER_DELAY_LIMIT           10
#else
#define TASK_GYROPID_DESIRED_PERIOD     1000 // 1000us = 1kHz
#define SCHEDULER_DELAY_LIMIT           100
#endif

#define DEFAULT_CPU_OVERCLOCK 0

#if defined(GD32F4)
#define FAST_IRQ_HANDLER
#elif defined(GD32H7)
#define FAST_IRQ_HANDLER FAST_CODE
#endif

#if defined(GD32F4)
// F4 can't DMA to/from TCM SRAM (where the stack lives)
#define DMA_DATA_ZERO_INIT
#define DMA_DATA
#define STATIC_DMA_DATA_AUTO        static
#elif defined(GD32H7)
#define DMA_DATA_ZERO_INIT          __attribute__ ((section(".dmaram_bss"), aligned(32)))
#define DMA_DATA                    __attribute__ ((section(".dmaram_data"), aligned(32)))
#define STATIC_DMA_DATA_AUTO        static DMA_DATA
#endif

#if defined(GD32F4) || defined(GD32H7)
// Data in RAM which is guaranteed to not be reset on hot reboot
#define PERSISTENT                  __attribute__ ((section(".persistent_data"), aligned(4)))
#endif

#ifdef USE_DMA_RAM
#if defined(GD32H7)
#define DMA_RAM             __attribute__((section(".DMA_RAM"), aligned(32)))
#define DMA_RW_AXI          __attribute__((section(".DMA_RW_AXI"), aligned(32)))
extern uint8_t _dmaram_start__;
extern uint8_t _dmaram_end__;
#endif
#else
#define DMA_RAM
#define DMA_RW_AXI
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW
#endif

#define USE_TIMER_MGMT
#define USE_TIMER_AF

#if defined(GD32F4) || defined(GD32H7)

#define IO_CONFIG(mode, speed, otype, pupd) ((mode) | ((speed) << 2) | ((otype) << 4) | ((pupd) << 5))

#define IOCFG_OUT_PP         IO_CONFIG(GPIO_MODE_OUTPUT, 0, GPIO_OTYPE_PP, GPIO_PUPD_NONE)
#define IOCFG_OUT_PP_UP      IO_CONFIG(GPIO_MODE_OUTPUT, 0, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP)
#if defined(GD32H7)
#define IOCFG_OUT_PP_60      IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_OSPEED_60MHZ, GPIO_OTYPE_PP, GPIO_PUPD_NONE)
#else
#define IOCFG_OUT_PP_25      IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_OSPEED_25MHZ, GPIO_OTYPE_PP, GPIO_PUPD_NONE)
#endif
#define IOCFG_OUT_OD         IO_CONFIG(GPIO_MODE_OUTPUT, 0, GPIO_OTYPE_OD, GPIO_PUPD_NONE)
#define IOCFG_AF_PP          IO_CONFIG(GPIO_MODE_AF,  0, GPIO_OTYPE_PP, GPIO_PUPD_NONE)
#define IOCFG_AF_PP_PD       IO_CONFIG(GPIO_MODE_AF,  0, GPIO_OTYPE_PP, GPIO_PUPD_PULLDOWN)
#define IOCFG_AF_PP_UP       IO_CONFIG(GPIO_MODE_AF,  0, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP)
#define IOCFG_AF_OD          IO_CONFIG(GPIO_MODE_AF,  0, GPIO_OTYPE_OD, GPIO_PUPD_NONE)
#define IOCFG_IPD            IO_CONFIG(GPIO_MODE_INPUT,  0, 0,             GPIO_PUPD_PULLDOWN)
#define IOCFG_IPU            IO_CONFIG(GPIO_MODE_INPUT,  0, 0,             GPIO_PUPD_PULLUP)
#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_MODE_INPUT,  0, 0,             GPIO_PUPD_NONE)
#if defined(GD32H7)
#define IOCFG_IPU_60         IO_CONFIG(GPIO_MODE_INPUT,  GPIO_OSPEED_60MHZ, 0, GPIO_PUPD_PULLUP)
#else
#define IOCFG_IPU_25         IO_CONFIG(GPIO_MODE_INPUT,  GPIO_OSPEED_25MHZ, 0, GPIO_PUPD_PULLUP)
#endif

#endif  // GD32F4 || GD32H7


#define FLASH_CONFIG_BUFFER_TYPE uint32_t


#if defined(GD32F4)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_AF,  GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP, GPIO_PUPD_NONE)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(GPIO_MODE_AF,  GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLDOWN)
#define SPI_IO_AF_SDI_CFG       IO_CONFIG(GPIO_MODE_AF,  GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP, GPIO_PUPD_NONE)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(GPIO_MODE_INPUT,  GPIO_OSPEED_50MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP)
#elif defined(GD32H7)
#define SPI_IO_AF_CFG           IO_CONFIG(GPIO_MODE_AF,  GPIO_OSPEED_60MHZ, GPIO_OTYPE_PP, GPIO_PUPD_NONE)
#define SPI_IO_AF_SCK_CFG       IO_CONFIG(GPIO_MODE_AF,  GPIO_OSPEED_60MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLDOWN)
#define SPI_IO_AF_SDI_CFG       IO_CONFIG(GPIO_MODE_AF,  GPIO_OSPEED_60MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP)
#define SPI_IO_CS_CFG           IO_CONFIG(GPIO_MODE_OUTPUT, GPIO_OSPEED_60MHZ, GPIO_OTYPE_PP, GPIO_PUPD_NONE)
#define SPI_IO_CS_HIGH_CFG      IO_CONFIG(GPIO_MODE_INPUT,  GPIO_OSPEED_60MHZ, GPIO_OTYPE_PP, GPIO_PUPD_PULLUP)
#else
#error "Invalid GD32 MCU defined - requires SPI implementation"
#endif

#if defined(GD32F4)
#if defined(GD32F460)
#define SPIDEV_COUNT 6
#else
#define SPIDEV_COUNT 3
#endif
#elif defined(GD32H7)
#define SPIDEV_COUNT 6
#else
#define SPIDEV_COUNT 4
#endif

// Work around different check routines in the libraries for different MCU types
#define CHECK_SPI_RX_DATA_AVAILABLE(instance) LL_SPI_IsActiveFlag_RXNE(instance)
#define SPI_RX_DATA_REGISTER(base) ((base)->DR)

#if defined(GD32F4)
#define USE_TX_IRQ_HANDLER
#endif


#if defined(GD32F4)
#define UARTHARDWARE_MAX_PINS 4
#elif defined(GD32H7)
#define UARTHARDWARE_MAX_PINS 6
#endif

#if defined(GD32F4)
#define UART_REG_RXD(base) (USART_DATA((uint32_t)base))
#define UART_REG_TXD(base) (USART_DATA((uint32_t)base))
#elif defined(GD32H7)
#define UART_REG_RXD(base) (USART_RDATA((uint32_t)base))
#define UART_REG_TXD(base) (USART_TDATA((uint32_t)base))
#endif

#define SERIAL_TRAIT_PIN_CONFIG 1
#define USB_DP_PIN PA12

// Select UART prefix according to UART_DEV
#define _UART_GET_PREFIX(dev) _UART_GET_PREFIX_##dev

#define _UART_GET_PREFIX_UARTDEV_0 USART
#define _UART_GET_PREFIX_UARTDEV_1 USART
#define _UART_GET_PREFIX_UARTDEV_2 USART
#define _UART_GET_PREFIX_UARTDEV_3 UART
#define _UART_GET_PREFIX_UARTDEV_4 UART
#define _UART_GET_PREFIX_UARTDEV_5 USART
#define _UART_GET_PREFIX_UARTDEV_6 UART
#define _UART_GET_PREFIX_UARTDEV_7 UART
#define _UART_GET_PREFIX_UARTDEV_8 UART
#define _UART_GET_PREFIX_UARTDEV_9 UART
#define _UART_GET_PREFIX_UARTDEV_10 USART
#define _UART_GET_PREFIX_UARTDEV_LP1 LPUART

void systemClockSetHSEValue(uint32_t frequency);
extern void timerOCModeConfig(void *tim, uint8_t channel, uint16_t ocmode);
