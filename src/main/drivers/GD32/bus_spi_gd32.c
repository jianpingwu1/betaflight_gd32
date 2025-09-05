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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SPI

// GD32F405 can't DMA to/from TCMSRAM
#define IS_TCM(p) (((uint32_t)p & 0xffff0000) == 0x10000000)

#include "common/maths.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/rcc.h"

// Use DMA if possible if this many bytes are to be transferred
#define SPI_DMA_THRESHOLD 8

static spi_parameter_struct defaultInit = {
    .device_mode = SPI_MASTER,
    .trans_mode = SPI_TRANSMODE_FULLDUPLEX,
    .frame_size = SPI_FRAMESIZE_8BIT,
    .nss = SPI_NSS_SOFT,
    .endian = SPI_ENDIAN_MSB,
    .clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE,
    .prescale = SPI_PSC_8
};

static uint32_t spiDivisorToBRbits(const SPI_TypeDef *instance, uint16_t divisor)
{
    // SPI1 and SPI2 are on APB1/AHB1 which PCLK is half that of APB2/AHB2.
    if ((uint32_t)instance == SPI1 || (uint32_t)instance == SPI2) {
        divisor /= 2; // Safe for divisor == 0 or 1
    }

    divisor = constrain(divisor, 2, 256);

    return ((uint32_t)(ffs(divisor) - 2) << 3); // SPI_CR1_BR_Pos
}

static void spiSetDivisorBRreg(SPI_TypeDef *instance, uint16_t divisor)
{
#define BR_BITS ((BIT(5) | BIT(4) | BIT(3)))
    const uint32_t tempRegister = (SPI_CTL0((uint32_t)instance) & ~BR_BITS);
    SPI_CTL0((uint32_t)instance) = (tempRegister | (spiDivisorToBRbits(instance, divisor) & BR_BITS));
#undef BR_BITS
}

void spiInitDevice(SPIDevice device)
{
    spiDevice_t *spi = &(spiDevice[device]);

    if (!spi->dev) {
        return;
    }

    // Enable SPI clock
    RCC_ClockCmd(spi->rcc, ENABLE);
    RCC_ResetCmd(spi->rcc, ENABLE);

    IOInit(IOGetByTag(spi->sck),  OWNER_SPI_SCK,  RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->miso), OWNER_SPI_SDI, RESOURCE_INDEX(device));
    IOInit(IOGetByTag(spi->mosi), OWNER_SPI_SDO, RESOURCE_INDEX(device));

    IOConfigGPIOAF(IOGetByTag(spi->sck),  SPI_IO_AF_SCK_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->miso), SPI_IO_AF_SDI_CFG, spi->af);
    IOConfigGPIOAF(IOGetByTag(spi->mosi), SPI_IO_AF_CFG, spi->af);

    // Init SPI hardware
    spi_i2s_deinit((uint32_t)spi->dev);

    spi_dma_disable((uint32_t)spi->dev, SPI_DMA_TRANSMIT);
    spi_dma_disable((uint32_t)spi->dev, SPI_DMA_RECEIVE);
    spi_init((uint32_t)spi->dev, &defaultInit);
    spi_crc_off((uint32_t)spi->dev);
    spi_enable((uint32_t)spi->dev);
}

void spiInternalResetDescriptors(busDevice_t *bus)
{
    DMA_InitTypeDef *dmaInitTx = bus->initTx;

    dma_single_data_para_struct_init(dmaInitTx);
    // dmaInitTx->DMA_Channel = bus->dmaTx->channel;
    dmaInitTx->direction = DMA_MEMORY_TO_PERIPH;
    dmaInitTx->circular_mode = DMA_CIRCULAR_MODE_DISABLE;
    dmaInitTx->periph_addr = (uint32_t)&SPI_DATA((uint32_t)(bus->busType_u.spi.instance));
    dmaInitTx->priority = DMA_PRIORITY_LOW;
    dmaInitTx->periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dmaInitTx->periph_memory_width = DMA_PERIPH_WIDTH_8BIT;

    if (bus->dmaRx) {
        DMA_InitTypeDef *dmaInitRx = bus->initRx;

        dma_single_data_para_struct_init(dmaInitRx);
    //     dmaInitRx->DMA_Channel = bus->dmaRx->channel;
        dmaInitRx->direction = DMA_PERIPH_TO_MEMORY;
        dmaInitRx->circular_mode = DMA_CIRCULAR_MODE_DISABLE;
        dmaInitRx->periph_addr = (uint32_t)&SPI_DATA((uint32_t)(bus->busType_u.spi.instance));
        dmaInitRx->priority = DMA_PRIORITY_LOW;
        dmaInitRx->periph_inc = DMA_PERIPH_INCREASE_DISABLE;
        dmaInitRx->periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    }
}

void spiInternalResetStream(dmaChannelDescriptor_t *descriptor)
{
    DMA_Stream_TypeDef *streamRegs = (DMA_Stream_TypeDef *)descriptor->ref;

    // Disable the stream
    REG32(streamRegs) = 0U;

    // Clear any pending interrupt flags
    dma_flag_clear((uint32_t)descriptor->dma, descriptor->stream, DMA_FLAG_FEE | DMA_FLAG_SDE | DMA_FLAG_TAE | DMA_FLAG_HTF | DMA_FLAG_FTF);
}

static bool spiInternalReadWriteBufPolled(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
    uint8_t b;

    while (len--) {
        b = txData ? *(txData++) : 0xFF;
        while (spi_i2s_flag_get((uint32_t)instance, I2S_FLAG_TBE) == RESET);
        spi_i2s_data_transmit((uint32_t)instance, b);

        while (spi_i2s_flag_get((uint32_t)instance, I2S_FLAG_RBNE) == RESET);
        b = spi_i2s_data_receive((uint32_t)instance);
        if (rxData) {
            *(rxData++) = b;
        }
    }

    return true;
}

void spiInternalInitStream(const extDevice_t *dev, bool preInit)
{
    STATIC_DMA_DATA_AUTO uint8_t dummyTxByte = 0xff;
    STATIC_DMA_DATA_AUTO uint8_t dummyRxByte;
    busDevice_t *bus = dev->bus;

    volatile busSegment_t *segment = bus->curSegment;

    if (preInit) {
        // Prepare the init structure for the next segment to reduce inter-segment interval
        segment++;
        if(segment->len == 0) {
            // There's no following segment
            return;
        }
    }

    int len = segment->len;

    uint8_t *txData = segment->u.buffers.txData;
    DMA_InitTypeDef *dmaInitTx = bus->initTx;

    if (txData) {
        dmaInitTx->memory0_addr = (uint32_t)txData;
        dmaInitTx->memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    } else {
        dummyTxByte = 0xff;
        dmaInitTx->memory0_addr = (uint32_t)&dummyTxByte;
        dmaInitTx->memory_inc = DMA_MEMORY_INCREASE_DISABLE;
    }
    dmaInitTx->number = len;

    if (dev->bus->dmaRx) {
        uint8_t *rxData = segment->u.buffers.rxData;
        DMA_InitTypeDef *dmaInitRx = bus->initRx;

        if (rxData) {
            dmaInitRx->memory0_addr = (uint32_t)rxData;
            dmaInitRx->memory_inc = DMA_MEMORY_INCREASE_ENABLE;
        } else {
            dmaInitRx->memory0_addr = (uint32_t)&dummyRxByte;
            dmaInitRx->memory_inc = DMA_MEMORY_INCREASE_DISABLE;
        }
        // If possible use 16 bit memory writes to prevent atomic access issues on gyro data
        if ((dmaInitRx->memory0_addr & 0x1) || (len & 0x1)) {
            dmaInitRx->periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
        } else {
            dmaInitRx->periph_memory_width = DMA_PERIPH_WIDTH_16BIT;
        }
        dmaInitRx->number = len;
    }
}

void spiInternalStartDMA(const extDevice_t *dev)
{
    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;
    DMA_Stream_TypeDef *streamRegsTx = (DMA_Stream_TypeDef *)dmaTx->ref;
    if (dmaRx) {
        DMA_Stream_TypeDef *streamRegsRx = (DMA_Stream_TypeDef *)dmaRx->ref;

        // Use the correct callback argument
        dmaRx->userParam = (uint32_t)dev;

        // Clear transfer flags
        dma_flag_clear((uint32_t)dmaTx->dma, dmaTx->stream, DMA_FLAG_FEE | DMA_FLAG_SDE | DMA_FLAG_TAE | DMA_FLAG_HTF | DMA_FLAG_FTF);
        dma_flag_clear((uint32_t)dmaRx->dma, dmaRx->stream, DMA_FLAG_FEE | DMA_FLAG_SDE | DMA_FLAG_TAE | DMA_FLAG_HTF | DMA_FLAG_FTF);

        // Disable streams to enable update
        REG32(streamRegsTx) = 0U;
        REG32(streamRegsRx) = 0U;

        /* Use the Rx interrupt as this occurs once the SPI operation is complete whereas the Tx interrupt
         * occurs earlier when the Tx FIFO is empty, but the SPI operation is still in progress
         */
        xDMA_ITConfig(streamRegsRx, DMA_INT_FTF, ENABLE);

        // Update streams
        
        xDMA_Init(streamRegsTx, dev->bus->initTx);
        dma_channel_subperipheral_select((uint32_t)(dmaTx->dma), dmaTx->stream, dmaTx->channel);
        xDMA_Init(streamRegsRx, dev->bus->initRx);
        dma_channel_subperipheral_select((uint32_t)(dmaRx->dma), dmaRx->stream, dmaRx->channel);

        /* Note from AN4031
         *
         * If the user enables the used peripheral before the corresponding DMA stream, a “FEIF”
         * (FIFO Error Interrupt Flag) may be set due to the fact the DMA is not ready to provide
         * the first required data to the peripheral (in case of memory-to-peripheral transfer).
         */

        // Enable streams
        dma_channel_enable((uint32_t)(dmaTx->dma), dmaTx->stream);
        dma_channel_enable((uint32_t)(dmaRx->dma), dmaRx->stream);

        /* Enable the SPI DMA Tx & Rx requests */
        spi_dma_enable((uint32_t)(dev->bus->busType_u.spi.instance), SPI_DMA_TRANSMIT);
        spi_dma_enable((uint32_t)(dev->bus->busType_u.spi.instance), SPI_DMA_RECEIVE);
    } else {
        // Use the correct callback argument
        dmaTx->userParam = (uint32_t)dev;

        // Clear transfer flags
        dma_flag_clear((uint32_t)dmaTx->dma, dmaTx->stream, DMA_FLAG_FEE | DMA_FLAG_SDE | DMA_FLAG_TAE | DMA_FLAG_HTF | DMA_FLAG_FTF);

        // Disable stream to enable update
        REG32(streamRegsTx) = 0U;

        xDMA_ITConfig(streamRegsTx, DMA_INT_FTF, ENABLE);

        // Update stream
        xDMA_Init(streamRegsTx, dev->bus->initTx);

        /* Note from AN4031
         *
         * If the user enables the used peripheral before the corresponding DMA stream, a “FEIF”
         * (FIFO Error Interrupt Flag) may be set due to the fact the DMA is not ready to provide
         * the first required data to the peripheral (in case of memory-to-peripheral transfer).
         */

        // Enable stream
        dma_channel_enable((uint32_t)(dmaTx->dma), dmaTx->stream);

        /* Enable the SPI DMA Tx request */
        spi_dma_enable((uint32_t)(dev->bus->busType_u.spi.instance), SPI_DMA_TRANSMIT);
    }
}

void spiInternalStopDMA (const extDevice_t *dev)
{
    dmaChannelDescriptor_t *dmaTx = dev->bus->dmaTx;
    dmaChannelDescriptor_t *dmaRx = dev->bus->dmaRx;
    SPI_TypeDef *instance = dev->bus->busType_u.spi.instance;
    DMA_Stream_TypeDef *streamRegsTx = (DMA_Stream_TypeDef *)dmaTx->ref;

    if (dmaRx) {
        DMA_Stream_TypeDef *streamRegsRx = (DMA_Stream_TypeDef *)dmaRx->ref;

        // Disable streams
        REG32(streamRegsTx) = 0U;
        REG32(streamRegsRx) = 0U;

        spi_dma_disable((uint32_t)instance, SPI_DMA_TRANSMIT);
        spi_dma_disable((uint32_t)instance, SPI_DMA_RECEIVE);
    } else {
        // Ensure the current transmission is complete
        while (spi_i2s_flag_get((uint32_t)instance, I2S_FLAG_TRANS));

        // Drain the RX buffer
        while (spi_i2s_flag_get((uint32_t)instance, I2S_FLAG_RBNE)) {
            SPI_DATA((uint32_t)instance);
        }

        // Disable stream
        REG32(streamRegsTx) = 0U;

        spi_dma_disable((uint32_t)instance, SPI_DMA_TRANSMIT);
    }
}

// DMA transfer setup and start
void spiSequenceStart(const extDevice_t *dev)
{
    busDevice_t *bus = dev->bus;
    SPI_TypeDef *instance = bus->busType_u.spi.instance;
    bool dmaSafe = dev->useDMA;
    uint32_t xferLen = 0;
    uint32_t segmentCount = 0;

    dev->bus->initSegment = true;

    spi_disable((uint32_t)instance);

    // Switch bus speed
    if (dev->busType_u.spi.speed != bus->busType_u.spi.speed) {
        spiSetDivisorBRreg(bus->busType_u.spi.instance, dev->busType_u.spi.speed);
        bus->busType_u.spi.speed = dev->busType_u.spi.speed;
    }

    if (dev->busType_u.spi.leadingEdge != bus->busType_u.spi.leadingEdge) {
        // Switch SPI clock polarity/phase
        SPI_CTL0((uint32_t)instance) &= ~(SPI_CTL0_CKPL | SPI_CTL0_CKPH);

        // Apply setting
        if (dev->busType_u.spi.leadingEdge) {
            SPI_CTL0((uint32_t)instance) |= SPI_CK_PL_LOW_PH_1EDGE;
        } else
        {
            SPI_CTL0((uint32_t)instance) |= SPI_CK_PL_HIGH_PH_2EDGE;
        }
        bus->busType_u.spi.leadingEdge = dev->busType_u.spi.leadingEdge;
    }

    spi_enable((uint32_t)instance);

    // Check that any there are no attempts to DMA to/from CCD SRAM
    for (busSegment_t *checkSegment = (busSegment_t *)bus->curSegment; checkSegment->len; checkSegment++) {
        // Check there is no receive data as only transmit DMA is available
        if (((checkSegment->u.buffers.rxData) && (IS_TCM(checkSegment->u.buffers.rxData) || (bus->dmaRx == (dmaChannelDescriptor_t *)NULL))) ||
            ((checkSegment->u.buffers.txData) && IS_TCM(checkSegment->u.buffers.txData))) {
            dmaSafe = false;
            break;
        }
        // Note that these counts are only valid if dmaSafe is true
        segmentCount++;
        xferLen += checkSegment->len;
    }
    // Use DMA if possible
    // If there are more than one segments, or a single segment with negateCS negated in the list terminator then force DMA irrespective of length
    if (bus->useDMA && dmaSafe && ((segmentCount > 1) ||
                                   (xferLen >= SPI_DMA_THRESHOLD) ||
                                   !bus->curSegment[segmentCount].negateCS)) {
        // Intialise the init structures for the first transfer
        spiInternalInitStream(dev, false);

        // Assert Chip Select
        IOLo(dev->busType_u.spi.csnPin);

        // Start the transfers
        spiInternalStartDMA(dev);
    } else {
        busSegment_t *lastSegment = NULL;
        bool segmentComplete;

        // Manually work through the segment list performing a transfer for each
        while (bus->curSegment->len) {
            if (!lastSegment || lastSegment->negateCS) {
                // Assert Chip Select if necessary - it's costly so only do so if necessary
                IOLo(dev->busType_u.spi.csnPin);
            }

            spiInternalReadWriteBufPolled(
                    bus->busType_u.spi.instance,
                    bus->curSegment->u.buffers.txData,
                    bus->curSegment->u.buffers.rxData,
                    bus->curSegment->len);

            if (bus->curSegment->negateCS) {
                // Negate Chip Select
                IOHi(dev->busType_u.spi.csnPin);
            }

            segmentComplete = true;
            if (bus->curSegment->callback) {
                switch(bus->curSegment->callback(dev->callbackArg)) {
                case BUS_BUSY:
                    // Repeat the last DMA segment
                    segmentComplete = false;
                    break;

                case BUS_ABORT:
                    bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
                    segmentComplete = false;
                    return;

                case BUS_READY:
                default:
                    // Advance to the next DMA segment
                    break;
                }
            }
            if (segmentComplete) {
                lastSegment = (busSegment_t *)bus->curSegment;
                bus->curSegment++;
            }
        }

        // If a following transaction has been linked, start it
        if (bus->curSegment->u.link.dev) {
            busSegment_t *endSegment = (busSegment_t *)bus->curSegment;
            const extDevice_t *nextDev = endSegment->u.link.dev;
            busSegment_t *nextSegments = (busSegment_t *)endSegment->u.link.segments;
            bus->curSegment = nextSegments;
            endSegment->u.link.dev = NULL;
            endSegment->u.link.segments = NULL;
            spiSequenceStart(nextDev);
        } else {
            // The end of the segment list has been reached, so mark transactions as complete
            bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
        }
    }
}
#endif
