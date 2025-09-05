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

/*
 * jflyper - Refactoring, cleanup and made pin-configurable
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_UART

#include "build/debug.h"

#include "drivers/system.h"
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
#ifdef USE_UART1
    {
        .device = UARTDEV_1,
        .reg = (void *)USART0,
        .rxDMAChannel = DMA_SUBPERI4,
        .txDMAChannel = DMA_SUBPERI4,

#ifdef USE_UART1_RX_DMA
        .rxDMAResource = &(dmaResource_t){DMA1, DMA_CH5},
#endif
#ifdef USE_UART1_TX_DMA
        .txDMAResource = &(dmaResource_t){DMA1, DMA_CH7},
#endif
        .rxPins = { { DEFIO_TAG_E(PA10) }, { DEFIO_TAG_E(PB3) }, { DEFIO_TAG_E(PB7) }, },
        .txPins = { { DEFIO_TAG_E(PA9) }, { DEFIO_TAG_E(PA15) }, { DEFIO_TAG_E(PB6) }, },
        .af = GPIO_AF_7,
        .rcc = RCC_APB2(USART0),
        .irqn = USART0_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART1_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART1,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
#endif

#ifdef USE_UART2
    {
        .device = UARTDEV_2,
        .reg = (void *)USART1,
        .rxDMAChannel = DMA_SUBPERI4,
        .txDMAChannel = DMA_SUBPERI4,
#ifdef USE_UART2_RX_DMA
        .rxDMAResource = &(dmaResource_t){DMA0, DMA_CH5},
#endif
#ifdef USE_UART2_TX_DMA
        .txDMAResource = &(dmaResource_t){DMA0, DMA_CH6},
#endif
        .rxPins = { { DEFIO_TAG_E(PA3) }, { DEFIO_TAG_E(PD6) } },
        .txPins = { { DEFIO_TAG_E(PA2) }, { DEFIO_TAG_E(PD5) } },
        .af = GPIO_AF_7,
        .rcc = RCC_APB1(USART1),
        .irqn = USART1_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART2_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART2,
        .txBuffer = uart2TxBuffer,
        .rxBuffer = uart2RxBuffer,
        .txBufferSize = sizeof(uart2TxBuffer),
        .rxBufferSize = sizeof(uart2RxBuffer),
    },
#endif

#ifdef USE_UART3
    {
        .device = UARTDEV_3,
        .reg = (void *)USART2,
        .rxDMAChannel = DMA_SUBPERI4,
        .txDMAChannel = DMA_SUBPERI4,
#ifdef USE_UART3_RX_DMA
        .rxDMAResource = &(dmaResource_t){DMA0, DMA_CH1},
#endif
#ifdef USE_UART3_TX_DMA
        .txDMAResource = &(dmaResource_t){DMA0, DMA_CH3},
#endif
        .rxPins = { { DEFIO_TAG_E(PB11) }, { DEFIO_TAG_E(PC11) }, { DEFIO_TAG_E(PD9) } },
        .txPins = { { DEFIO_TAG_E(PB10) }, { DEFIO_TAG_E(PC10) }, { DEFIO_TAG_E(PD8) } },
        .af = GPIO_AF_7,
        .rcc = RCC_APB1(USART2),
        .irqn = USART2_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART3_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART3,
        .txBuffer = uart3TxBuffer,
        .rxBuffer = uart3RxBuffer,
        .txBufferSize = sizeof(uart3TxBuffer),
        .rxBufferSize = sizeof(uart3RxBuffer),
    },
#endif

#ifdef USE_UART4
    {
        .device = UARTDEV_4,
        .reg = (void *)UART3,
        .rxDMAChannel = DMA_SUBPERI4,
        .txDMAChannel = DMA_SUBPERI4,
#ifdef USE_UART4_RX_DMA
        .rxDMAResource = &(dmaResource_t){DMA0, DMA_CH2},
#endif
#ifdef USE_UART4_TX_DMA
        .txDMAResource = &(dmaResource_t){DMA0, DMA_CH4},
#endif
        .rxPins = { { DEFIO_TAG_E(PA1) }, { DEFIO_TAG_E(PC11) } },
        .txPins = { { DEFIO_TAG_E(PA0) }, { DEFIO_TAG_E(PC10) } },
        .af = GPIO_AF_8,
        .rcc = RCC_APB1(UART3),
        .irqn = UART3_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART4_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART4,
        .txBuffer = uart4TxBuffer,
        .rxBuffer = uart4RxBuffer,
        .txBufferSize = sizeof(uart4TxBuffer),
        .rxBufferSize = sizeof(uart4RxBuffer),
    },
#endif

#ifdef USE_UART5
    {
        .device = UARTDEV_5,
        .reg = (void *)UART4,
        .rxDMAChannel = DMA_SUBPERI4,
        .txDMAChannel = DMA_SUBPERI4,
#ifdef USE_UART5_RX_DMA
        .rxDMAResource = &(dmaResource_t){DMA0, DMA_CH0},
#endif
#ifdef USE_UART5_TX_DMA
        .txDMAResource = &(dmaResource_t){DMA0, DMA_CH7},
#endif
        .rxPins = { { DEFIO_TAG_E(PD2) } },
        .txPins = { { DEFIO_TAG_E(PC12) } },
        .af = GPIO_AF_8,
        .rcc = RCC_APB1(UART4),
        .irqn = UART4_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART5_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART5,
        .txBuffer = uart5TxBuffer,
        .rxBuffer = uart5RxBuffer,
        .txBufferSize = sizeof(uart5TxBuffer),
        .rxBufferSize = sizeof(uart5RxBuffer),
    },
#endif

#ifdef USE_UART6
    {
        .device = UARTDEV_6,
        .reg = (void *)USART5,
        .rxDMAChannel = DMA_SUBPERI5,
        .txDMAChannel = DMA_SUBPERI5,
#ifdef USE_UART6_RX_DMA
        .rxDMAResource = &(dmaResource_t){DMA1, DMA_CH1},
#endif
#ifdef USE_UART6_TX_DMA
        .txDMAResource = &(dmaResource_t){DMA1, DMA_CH6},
#endif
        .rxPins = { { DEFIO_TAG_E(PC7) }, { DEFIO_TAG_E(PG9) } },
        .txPins = { { DEFIO_TAG_E(PC6) }, { DEFIO_TAG_E(PG14) } },
        .af = GPIO_AF_8,
        .rcc = RCC_APB2(USART5),
        .irqn = USART5_IRQn,
        .txPriority = NVIC_PRIO_SERIALUART6_TXDMA,
        .rxPriority = NVIC_PRIO_SERIALUART6,
        .txBuffer = uart6TxBuffer,
        .rxBuffer = uart6RxBuffer,
        .txBufferSize = sizeof(uart6TxBuffer),
        .rxBufferSize = sizeof(uart6RxBuffer),
    },
#endif

};


bool checkUsartTxOutput(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);
    IO_t txIO = IOGetByTag(uart->tx.pin);

    if ((uart->txPinState == TX_PIN_MONITOR) && txIO) {
        if (IORead(txIO)) {
            
            uart->txPinState = TX_PIN_ACTIVE;
            IOConfigGPIOAF(txIO, IOCFG_AF_PP, uart->hardware->af);

            usart_transmit_config((uint32_t)s->USARTx, USART_TRANSMIT_ENABLE);

            return true;
        } else {
            return false;
        }
    }

    return true;
}

void uartTxMonitor(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);

    if (uart->txPinState == TX_PIN_ACTIVE) {
        IO_t txIO = IOGetByTag(uart->tx.pin);

        usart_transmit_config((uint32_t)s->USARTx, USART_TRANSMIT_DISABLE);

        uart->txPinState = TX_PIN_MONITOR;
        IOConfigGPIO(txIO, IOCFG_IPU);
    }
}

static void handleUsartTxDma(uartPort_t *s)
{
    uartDevice_t *uart = container_of(s, uartDevice_t, port);

    uartTryStartTxDMA(s);

    if (s->txDMAEmpty && (uart->txPinState != TX_PIN_IGNORE)) {
        uartTxMonitor(s);
    }
}

void uartDmaIrqHandler(dmaChannelDescriptor_t* descriptor)
{
    uartPort_t *s = &(((uartDevice_t*)(descriptor->userParam))->port);
    
    if (dma_interrupt_flag_get((uint32_t)descriptor->dma, descriptor->stream, DMA_INT_FLAG_FTF))
    {
        dma_interrupt_flag_clear((uint32_t)descriptor->dma, descriptor->stream, DMA_INT_FLAG_FTF);
        dma_interrupt_flag_clear((uint32_t)descriptor->dma, descriptor->stream, DMA_INT_FLAG_HTF);
        
        if (dma_interrupt_flag_get((uint32_t)descriptor->dma, descriptor->stream, DMA_INT_FLAG_FEE))
        {
            dma_interrupt_flag_clear((uint32_t)descriptor->dma, descriptor->stream, DMA_INT_FLAG_FEE);
        }
        
        handleUsartTxDma(s);
    }
    
    if (dma_interrupt_flag_get((uint32_t)descriptor->dma, descriptor->stream, DMA_INT_FLAG_TAE))
    {
        dma_interrupt_flag_clear((uint32_t)descriptor->dma, descriptor->stream, DMA_INT_FLAG_TAE);
    }
    
    if (dma_interrupt_flag_get((uint32_t)descriptor->dma, descriptor->stream, DMA_INT_FLAG_SDE))
    {
        dma_interrupt_flag_clear((uint32_t)descriptor->dma, descriptor->stream, DMA_INT_FLAG_SDE);
    }
}


// XXX Should serialUART be consolidated?

uartPort_t *serialUART(UARTDevice_e device, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    uartDevice_t *uart = uartDevmap[device];
    if (!uart) return NULL;

    const uartHardware_t *hardware = uart->hardware;

    if (!hardware) return NULL; // XXX Can't happen !?

    uartPort_t *s = &(uart->port);
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = hardware->rxBuffer;
    s->port.txBuffer = hardware->txBuffer;
    s->port.rxBufferSize = hardware->rxBufferSize;
    s->port.txBufferSize = hardware->txBufferSize;

    s->USARTx = hardware->reg;

    s->checkUsartTxOutput = checkUsartTxOutput;

#ifdef USE_DMA
    uartConfigureDma(uart);
#endif

    IO_t txIO = IOGetByTag(uart->tx.pin);
    IO_t rxIO = IOGetByTag(uart->rx.pin);

    if (hardware->rcc) {
        RCC_ClockCmd(hardware->rcc, ENABLE);
    }

    uart->txPinState = TX_PIN_IGNORE;

    if (options & SERIAL_BIDIR) {
        IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));
        IOConfigGPIOAF(txIO, (options & SERIAL_BIDIR_PP_PD) ? IOCFG_AF_PP_PD 
                             : (options & SERIAL_BIDIR_PP) ? IOCFG_AF_PP
                             : IOCFG_AF_OD, hardware->af);
    } else {
        if ((mode & MODE_TX) && txIO) {
            IOInit(txIO, OWNER_SERIAL_TX, RESOURCE_INDEX(device));

            if (options & SERIAL_CHECK_TX) {
                uart->txPinState = TX_PIN_ACTIVE;
                // Switch TX to an input with pullup so it's state can be monitored
                uartTxMonitor(s);
            } else {
                IOConfigGPIOAF(txIO, IOCFG_AF_PP_UP, hardware->af);
            }
        }

        if ((mode & MODE_RX) && rxIO) {
            IOInit(rxIO, OWNER_SERIAL_RX, RESOURCE_INDEX(device));
            IOConfigGPIOAF(rxIO, IOCFG_AF_PP_UP, hardware->af);
        }
    }

#ifdef USE_DMA
    if (!(s->rxDMAResource)) {
        nvic_irq_enable(hardware->irqn, NVIC_PRIORITY_BASE(hardware->rxPriority), NVIC_PRIORITY_SUB(hardware->rxPriority));

    }
#endif

    return s;
}

void uartIrqHandler(uartPort_t *s)
{
    if (!s->rxDMAResource && (usart_interrupt_flag_get((uint32_t)s->USARTx, USART_INT_FLAG_RBNE) == SET)) {
        if (s->port.rxCallback) {
            s->port.rxCallback(usart_data_receive((uint32_t)s->USARTx), s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead] = (uint8_t)usart_data_receive((uint32_t)s->USARTx);
            s->port.rxBufferHead = (s->port.rxBufferHead + 1) % s->port.rxBufferSize;
        }
    }

    if (usart_interrupt_flag_get((uint32_t)s->USARTx, USART_INT_FLAG_TC) == SET) {
        uartTxMonitor(s);

        usart_interrupt_flag_clear((uint32_t)s->USARTx, USART_INT_FLAG_TC);
    }

    if (!s->txDMAResource && (usart_interrupt_flag_get((uint32_t)s->USARTx, USART_INT_FLAG_TBE) == SET)) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            usart_data_transmit((uint32_t)s->USARTx, s->port.txBuffer[s->port.txBufferTail]);
            s->port.txBufferTail = (s->port.txBufferTail + 1) % s->port.txBufferSize;
        } else {
            usart_interrupt_disable((uint32_t)s->USARTx, USART_INT_TBE);
        }
    }

    if (usart_interrupt_flag_get((uint32_t)s->USARTx, USART_INT_FLAG_ERR_ORERR) == SET) {
        usart_interrupt_flag_clear((uint32_t)s->USARTx, USART_INT_FLAG_ERR_ORERR);
    }

    if (usart_interrupt_flag_get((uint32_t)s->USARTx, USART_INT_FLAG_IDLE) == SET) {
        if (s->port.idleCallback) {
            s->port.idleCallback();
        }

        /* clear IDLE flag */
        USART_STAT0((uint32_t)s->USARTx);
        USART_DATA((uint32_t)s->USARTx);
    }

}
#endif // USE_UART
