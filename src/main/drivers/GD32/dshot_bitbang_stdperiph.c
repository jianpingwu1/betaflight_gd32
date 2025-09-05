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

 #include <stdint.h>
 #include <math.h>
 #include <string.h>
 
 #include "platform.h"
 
 #ifdef USE_DSHOT_BITBANG
 
 #include "build/atomic.h"
 #include "build/debug.h"
 #include "build/debug_pin.h"
 
 #include "drivers/io.h"
 #include "drivers/io_impl.h"
 #include "drivers/dma.h"
 #include "drivers/dma_reqmap.h"
 #include "drivers/dshot.h"

#include "drivers/dshot_bitbang_impl.h"
#include "drivers/dshot_command.h"
#include "drivers/motor.h"
#include "drivers/nvic.h"
#include "drivers/time.h"
#include "drivers/timer.h"

#include "pg/motor.h"

 static uint32_t dshot_dma_periphs[MAX_SUPPORTED_MOTOR_PORTS];
 static int dshot_dma_channels[MAX_SUPPORTED_MOTOR_PORTS];
 
 void bbGpioSetup(bbMotor_t *bbMotor)
 {
     bbPort_t *bbPort = bbMotor->bbPort;
     int pinIndex = bbMotor->pinIndex;

     bbPort->gpioModeMask |= (CTL_CLTR(3) << (pinIndex * 2));
     bbPort->gpioModeInput |= (GPIO_MODE_INPUT << (pinIndex * 2));
     bbPort->gpioModeOutput |= (GPIO_MODE_OUTPUT << (pinIndex * 2));
 
 #ifdef USE_DSHOT_TELEMETRY
     if (useDshotTelemetry) {
         bbPort->gpioIdleBSRR |= (1 << pinIndex);         // BS (lower half)
     } else
 #endif
     {
         bbPort->gpioIdleBSRR |= (1 << (pinIndex + 16));  // BR (higher half)
     }
 
 #ifdef USE_DSHOT_TELEMETRY
     if (useDshotTelemetry) {
         IOWrite(bbMotor->io, 1);
     } else
 #endif
     {
         IOWrite(bbMotor->io, 0);
     }
 }
 
 void bbTimerChannelInit(bbPort_t *bbPort)
 {
     const timerHardware_t *timhw = bbPort->timhw;
 
     timer_oc_parameter_struct timer_ocintpara;
 
     timer_channel_output_struct_para_init(&timer_ocintpara);

     timer_channel_output_mode_config((uint32_t)timhw->tim, timhw->channel, TIMER_OC_MODE_PWM0);
 
     timer_ocintpara.ocidlestate = TIMER_OC_IDLE_STATE_HIGH;
     timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
     timer_ocintpara.ocpolarity  = TIMER_OC_POLARITY_LOW;       
 
    timer_channel_output_pulse_value_config((uint32_t)(timhw->tim), timhw->channel, 10);
 
    timer_disable((uint32_t)(bbPort->timhw->tim));
 
    timerOCInit(timhw->tim, timhw->channel, &timer_ocintpara);
    timerOCModeConfig(timhw->tim, timhw->channel, TIMER_OC_MODE_PWM0);
 
 
 #ifdef DEBUG_MONITOR_PACER
     if (timhw->tag) {
         IO_t io = IOGetByTag(timhw->tag);
         IOConfigGPIOAF(io, IOCFG_AF_PP, timhw->alternateFunction);
         IOInit(io, OWNER_DSHOT_BITBANG, 0);
         // TIM_CtrlPWMOutputs(timhw->tim, ENABLE);
         timer_primary_output_config((uint32_t)(timhw->tim), ENABLE);
     }
 #endif
 
     // Enable and keep it running
 
     timer_enable((uint32_t)(bbPort->timhw->tim));
 }
 
 #ifdef USE_DMA_REGISTER_CACHE


static void bbLoadDMARegs(uint32_t dma_periph, int dma_channel, dmaRegCache_t *dmaRegCache)
 {
#if 0
    DMA_CHCTL((uint32_t)dmaResource) = dmaRegCache->CHCTL;
    DMA_CHCNT((uint32_t)dmaResource) = dmaRegCache->CHCNT;
    DMA_CHPADDR((uint32_t)dmaResource) = dmaRegCache->CHPADDR;
    DMA_CHM0ADDR((uint32_t)dmaResource) = dmaRegCache->CHM0ADDR;
    DMA_CHFCTL((uint32_t)dmaResource) = dmaRegCache->CHFCTL;
#else
    DMA_CHCTL(dma_periph, dma_channel) = dmaRegCache->CHCTL;
    DMA_CHCNT(dma_periph, dma_channel) = dmaRegCache->CHCNT;
    DMA_CHPADDR(dma_periph, dma_channel) = dmaRegCache->CHPADDR;
    DMA_CHM0ADDR(dma_periph, dma_channel) = dmaRegCache->CHM0ADDR;
    DMA_CHFCTL(dma_periph, dma_channel) = dmaRegCache->CHFCTL;
#endif
 }
 
static void bbSaveDMARegs(uint32_t dma_periph, int dma_channel, dmaRegCache_t *dmaRegCache)
 {
#if 0
    dmaRegCache->CHCTL = DMA_CHCTL((uint32_t)dmaResource);
    dmaRegCache->CHCNT = DMA_CHCNT((uint32_t)dmaResource);
    dmaRegCache->CHPADDR = DMA_CHPADDR((uint32_t)dmaResource);
    dmaRegCache->CHM0ADDR = DMA_CHM0ADDR((uint32_t)dmaResource);
    dmaRegCache->CHFCTL = DMA_CHFCTL((uint32_t)dmaResource);
#else
    dmaRegCache->CHCTL = DMA_CHCTL(dma_periph, dma_channel);
    dmaRegCache->CHCNT = DMA_CHCNT(dma_periph, dma_channel);
    dmaRegCache->CHPADDR = DMA_CHPADDR(dma_periph, dma_channel);
    dmaRegCache->CHM0ADDR = DMA_CHM0ADDR(dma_periph, dma_channel);
    dmaRegCache->CHFCTL = DMA_CHFCTL(dma_periph, dma_channel);
#endif
 }
 
 
 #endif
 
 void bbSwitchToOutput(bbPort_t * bbPort)
 {
     dbgPinHi(1);
     // Output idle level before switching to output
     // Use BSRR register for this
     // Normal: Use BR (higher half)
     // Inverted: Use BS (lower half)
 
     WRITE_REG(GPIO_BOP((uint32_t)bbPort->gpio), bbPort->gpioIdleBSRR);
 
     // Set GPIO to output
     ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
         MODIFY_REG(GPIO_CTL((uint32_t)bbPort->gpio), bbPort->gpioModeMask, bbPort->gpioModeOutput);
     }
 
     // Reinitialize port group DMA for output
 
     dmaResource_t *dmaResource = bbPort->dmaResource;
 #ifdef USE_DMA_REGISTER_CACHE
    UNUSED(dmaResource);
     bbLoadDMARegs(dshot_dma_periphs[bbPort->portIndex], dshot_dma_channels[bbPort->portIndex], &bbPort->dmaRegOutput);
 #else
     xDMA_DeInit(dmaResource);
    dma_multi_data_mode_init(dshot_dma_periphs[bbPort->portIndex], dshot_dma_channels[bbPort->portIndex], pDmaInit);

     // Needs this, as it is DeInit'ed above...
     xDMA_ITConfig(dmaResource, DMA_INT_FTF, ENABLE);
 #endif
 
     // Reinitialize pacer timer for output
 
     TIMER_CAR((uint32_t)(bbPort->timhw->tim)) = bbPort->outputARR;
 
     bbPort->direction = DSHOT_BITBANG_DIRECTION_OUTPUT;
 
     dbgPinLo(1);
 }
 
 #ifdef USE_DSHOT_TELEMETRY
 void bbSwitchToInput(bbPort_t *bbPort)
 {
     dbgPinHi(1);
 
     // Set GPIO to input
 
     ATOMIC_BLOCK(NVIC_PRIO_TIMER) {
         MODIFY_REG(GPIO_CTL((uint32_t)bbPort->gpio), bbPort->gpioModeMask, bbPort->gpioModeInput);
     }
 
     // Reinitialize port group DMA for input
 
     dmaResource_t *dmaResource = bbPort->dmaResource;
 #ifdef USE_DMA_REGISTER_CACHE
    UNUSED(dmaResource);
     bbLoadDMARegs(dshot_dma_periphs[bbPort->portIndex], dshot_dma_channels[bbPort->portIndex], &bbPort->dmaRegInput);
 #else
     xDMA_DeInit(dmaResource);
    dma_multi_data_mode_init(dshot_dma_periphs[bbPort->portIndex], dshot_dma_channels[bbPort->portIndex], pDmaInit);

     // Needs this, as it is DeInit'ed above...
     xDMA_ITConfig(dmaResource, DMA_INT_FTF, ENABLE);
 #endif
 
     // Reinitialize pacer timer for input
 
     TIMER_CNT(&(bbPort->timhw->tim)) = 0;
     TIMER_CAR(&(bbPort->timhw->tim)) = bbPort->inputARR;
 
     bbDMA_Cmd(bbPort, ENABLE);
 
     bbPort->direction = DSHOT_BITBANG_DIRECTION_INPUT;
 
     dbgPinLo(1);
 }
 #endif
 
 void bbDMAPreconfigure(bbPort_t *bbPort, uint8_t direction)
 {
    gd32_dma_chbase_parse((uint32_t)bbPort->dmaResource, &dshot_dma_periphs[bbPort->portIndex], &dshot_dma_channels[bbPort->portIndex]);

    dma_multi_data_parameter_struct *dmainit = (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) ?  &bbPort->outputDmaInit : &bbPort->inputDmaInit;
 
    dma_multi_data_para_struct_init(dmainit);

    dma_channel_subperipheral_select(dshot_dma_periphs[bbPort->portIndex], dshot_dma_channels[bbPort->portIndex], bbPort->dmaChannel);
 
    dmainit->circular_mode = DMA_CIRCULAR_MODE_DISABLE;
    dmainit->periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dmainit->memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dmainit->critical_value = DMA_FIFO_STATUS_1_WORD;
    dmainit->memory_burst_width = DMA_MEMORY_BURST_SINGLE;
    dmainit->periph_burst_width = DMA_PERIPH_BURST_SINGLE;
 
 
 
    if (direction == DSHOT_BITBANG_DIRECTION_OUTPUT) {
        dmainit->priority = DMA_PRIORITY_HIGH;
        dmainit->direction = DMA_MEMORY_TO_PERIPH;
        dmainit->number = bbPort->portOutputCount;
        dmainit->periph_addr = (uint32_t)&GPIO_BOP((uint32_t)bbPort->gpio);
        dmainit->periph_width = DMA_PERIPH_WIDTH_32BIT;
        dmainit->memory0_addr = (uint32_t)bbPort->portOutputBuffer;
        dmainit->memory_width = DMA_MEMORY_WIDTH_32BIT;
 
 
 #ifdef USE_DMA_REGISTER_CACHE
        dma_multi_data_mode_init(dshot_dma_periphs[bbPort->portIndex], dshot_dma_channels[bbPort->portIndex], dmainit);
        bbSaveDMARegs(dshot_dma_periphs[bbPort->portIndex], dshot_dma_channels[bbPort->portIndex], &bbPort->dmaRegOutput);
 #endif
    } else {
        dmainit->priority = DMA_PRIORITY_ULTRA_HIGH;
        dmainit->direction = DMA_PERIPH_TO_MEMORY;
        dmainit->number = bbPort->portInputCount;
        dmainit->periph_addr = (uint32_t)&GPIO_ISTAT((uint32_t)bbPort->gpio);
        dmainit->periph_width = DMA_PERIPH_WIDTH_16BIT;
        dmainit->memory0_addr = (uint32_t)bbPort->portInputBuffer;
        dmainit->memory_width = DMA_MEMORY_WIDTH_32BIT;
 
 #ifdef USE_DMA_REGISTER_CACHE
        dma_multi_data_mode_init(dshot_dma_periphs[bbPort->portIndex], dshot_dma_channels[bbPort->portIndex], dmainit);
        bbSaveDMARegs(dshot_dma_periphs[bbPort->portIndex], dshot_dma_channels[bbPort->portIndex], &bbPort->dmaRegInput);
 #endif
    }
 }
 
 void bbTIM_TimeBaseInit(bbPort_t *bbPort, uint16_t period)
 {
    timer_parameter_struct *init = &bbPort->timeBaseInit;

    init->prescaler         = 0; // Feed raw timerClock
    init->clockdivision     = TIMER_CKDIV_DIV1;
    init->alignedmode       = TIMER_COUNTER_EDGE;
    init->counterdirection  = TIMER_COUNTER_UP;
    init->period            = period;
    init->repetitioncounter = 0;

    timer_init((uint32_t)(bbPort->timhw->tim), init);
    timer_auto_reload_shadow_enable((uint32_t)(bbPort->timhw->tim));

}
 
 void bbTIM_DMACmd(TIM_TypeDef* TIMx, uint16_t TIM_DMASource, FunctionalState NewState)
 {
    if(ENABLE == NewState){
        timer_dma_enable((uint32_t)TIMx, TIM_DMASource);
    }
    else {
        timer_dma_disable((uint32_t)TIMx, TIM_DMASource);
    }
 }
 
 void bbDMA_ITConfig(bbPort_t *bbPort)
 {
    xDMA_ITConfig(bbPort->dmaResource, DMA_INT_FTF, ENABLE);
 }
 
 void bbDMA_Cmd(bbPort_t *bbPort, FunctionalState NewState)
 {
    xDMA_Cmd(bbPort->dmaResource, NewState);
 }
 
 int bbDMA_Count(bbPort_t *bbPort)
 {
    return xDMA_GetCurrDataCounter(bbPort->dmaResource);
 }
 
 #endif // USE_DSHOT_BB
