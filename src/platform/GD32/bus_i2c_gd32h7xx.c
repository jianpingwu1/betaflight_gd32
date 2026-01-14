/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) && !defined(USE_SOFT_I2C)

#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "platform/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"
#include "drivers/bus_i2c_utils.h"

static void i2c_er_handler(i2cDevice_e device);
static void i2c_ev_handler(i2cDevice_e device);

#define IOCFG_I2C_PU IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_60MHZ, GPIO_OTYPE_OD, GPIO_PUPD_PULLUP)
#define IOCFG_I2C   IO_CONFIG(GPIO_MODE_AF, GPIO_OSPEED_60MHZ, GPIO_OTYPE_OD, GPIO_PUPD_NONE)

const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_0
    {
        .device = I2CDEV_0,
        .reg = (I2C_TypeDef *)I2C0,
        .sclPins = {
            I2CPINDEF(PB6, GPIO_AF_4),
            I2CPINDEF(PB8, GPIO_AF_4),
        },
         .sdaPins = {
            I2CPINDEF(PB7, GPIO_AF_4),
            I2CPINDEF(PB9, GPIO_AF_4),
         },
        .rcc = RCC_APB1(I2C0),
        .ev_irq = I2C0_EV_IRQn,
        .er_irq = I2C0_ER_IRQn,
    },
#endif
 #ifdef USE_I2C_DEVICE_1
     {
         .device = I2CDEV_1,
         .reg = (I2C_TypeDef *)I2C1,
         .sclPins = {
             I2CPINDEF(PB10, GPIO_AF_4),
             I2CPINDEF(PF1,  GPIO_AF_4),
         },
         .sdaPins = {
             I2CPINDEF(PB11, GPIO_AF_4),
             I2CPINDEF(PF0,  GPIO_AF_4),
         },
         .rcc = RCC_APB1(I2C1),
         .ev_irq = I2C1_EV_IRQn,
         .er_irq = I2C1_ER_IRQn,
     },
 #endif
#ifdef USE_I2C_DEVICE_2
     {
         .device = I2CDEV_2,
         .reg = (I2C_TypeDef *)I2C2,
         .sclPins = {
             I2CPINDEF(PA8, GPIO_AF_4),
         },
         .sdaPins = {
             I2CPINDEF(PC9, GPIO_AF_4),
         },
         .rcc = RCC_APB1(I2C2),
         .ev_irq = I2C2_EV_IRQn,
         .er_irq = I2C2_ER_IRQn,
     },
#endif
#ifdef USE_I2C_DEVICE_3
     {
         .device = I2CDEV_3,
         .reg = (I2C_TypeDef *)I2C3,
         .sclPins = {
             I2CPINDEF(PD12, GPIO_AF_4),
         },
         .sdaPins = {
             I2CPINDEF(PD13, GPIO_AF_4),
         },
         .rcc = RCC_APB1(I2C3),
         .ev_irq = I2C3_EV_IRQn,
         .er_irq = I2C3_ER_IRQn,
     },
#endif
};

i2cDevice_t i2cDevice[I2CDEV_COUNT];

// State used by event handler ISR
typedef struct {
    uint8_t subaddress_sent;    // flag to indicate if subaddess sent
    uint8_t final_stop;         // flag to indicate final bus condition
    int8_t index;               // index is signed -1 == send the subaddress
} i2cEvState_t;
static i2cEvState_t i2c_ev_state[I2CDEV_COUNT];

static volatile uint16_t i2cErrorCount = 0;

void I2C0_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_0);
}

void I2C0_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_0);
}

void I2C1_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_1);
}

void I2C1_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_1);
}

void I2C2_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_2);
}

void I2C2_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_2);
}

void I2C3_ER_IRQHandler(void)
{
    i2c_er_handler(I2CDEV_3);
}

void I2C3_EV_IRQHandler(void)
{
    i2c_ev_handler(I2CDEV_3);
}

 static bool i2cHandleHardwareFailure(i2cDevice_e device)
 {
     i2cErrorCount++;
     // reinit peripheral + clock out garbage
     i2cInit(device);
     return false;
 }

bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    uint32_t I2Cx = (uint32_t )(i2cDevice[device].reg);

    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &(i2cDevice[device].state);
    if (state->busy) {
        return false;
    }

    timeUs_t timeoutStartUs = microsISR();

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 1;
    state->reading = 0;
    state->write_p = data;
    state->read_p = data;
    state->bytes = len_;
    state->busy = 1;
    state->error = false;

    i2cEvState_t *ev_state = &i2c_ev_state[device];
    ev_state->subaddress_sent = 0;
    ev_state->final_stop = 0;
    ev_state->index = (reg_ != 0xFF) ? -1 : 0;

    // Ensure bus is free (check I2CBSY)
    while (I2C_STAT(I2Cx) & I2C_STAT_I2CBSY) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device);
        }
    }

    // Wait for the STOP bit to be cleared by hardware from the previous transaction.
    while (I2C_CTL1(I2Cx) & I2C_CTL1_STOP) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device);
        }
    }

    // Disable all interrupts first to ensure clean state
    I2C_CTL0(I2Cx) &= ~(I2C_CTL0_TIE | I2C_CTL0_RBNEIE | I2C_CTL0_TCIE | I2C_CTL0_NACKIE | I2C_CTL0_STPDETIE | I2C_CTL0_ERRIE);

    // Flush TDATA before starting
    I2C_STAT(I2Cx) |= I2C_STAT_TBE;

    // Configure Transfer
    I2C_CTL1(I2Cx) &= ~(I2C_CTL1_BYTENUM | I2C_CTL1_SADDRESS | I2C_CTL1_TRDIR | I2C_CTL1_RELOAD | I2C_CTL1_AUTOEND);
    
    uint32_t total_bytes = state->bytes;
    if (state->reg != 0xFF) total_bytes++; // Add subaddress byte

    uint32_t ctl1 = I2C_CTL1(I2Cx);
    ctl1 |= (total_bytes << 16); // BYTENUM
    ctl1 |= (state->addr & I2C_CTL1_SADDRESS); // SADDRESS
    ctl1 &= ~I2C_CTL1_TRDIR; // Master Transmit
    I2C_CTL1(I2Cx) = ctl1;

    // Enable Interrupts
    // H7: TIE (Tx), TCIE (Transfer Complete), NACKIE, STOPDETIE, ERRIE
    I2C_CTL0(I2Cx) |= (I2C_CTL0_TIE | I2C_CTL0_TCIE | I2C_CTL0_NACKIE | I2C_CTL0_STPDETIE | I2C_CTL0_ERRIE);

    // Send Start
    I2C_CTL1(I2Cx) |= I2C_CTL1_START;

    return true;
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
    i2cState_t *state = &i2cDevice[device].state;

    if (error) {
        *error = state->error;
    }
    return state->busy;
}

static bool i2cWait(i2cDevice_e device)
{
    i2cState_t *state = &(i2cDevice[device].state);
    timeUs_t timeoutStartUs = microsISR();

    while (state->busy) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device) && i2cWait(device);
        }
    }

    return !(state->error);
}

bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data) && i2cWait(device);
}

bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{

    if (device == I2CINVALID || device >= I2CDEV_COUNT) {
        return false;
    }

    uint32_t I2Cx = (uint32_t )(i2cDevice[device].reg);
    if (!I2Cx) {
        return false;
    }

    i2cState_t *state = &i2cDevice[device].state;
    if (state->busy) {
        return false;
    }

    timeUs_t timeoutStartUs = microsISR();

    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 0;
    state->reading = 1;
    state->read_p = buf;
    state->write_p = buf;
    state->bytes = len;
    state->busy = 1;
    state->error = false;

    i2cEvState_t *ev_state = &i2c_ev_state[device];
    ev_state->subaddress_sent = 0;
    // ev_state->final_stop = 0;
    ev_state->index = 0;

    I2C_STATC(I2Cx) |= (I2C_STATC_ADDSENDC | I2C_STATC_BERRC | I2C_STATC_LOSTARBC | I2C_STATC_OUERRC | \
    I2C_STATC_STPDETC | I2C_STATC_NACKC);

    // Ensure bus is free
    while (I2C_STAT(I2Cx) & I2C_STAT_I2CBSY) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device);
        }
    }

    // Wait for the STOP bit to be cleared by hardware from the previous transaction.
    while (I2C_CTL1(I2Cx) & I2C_CTL1_STOP) {
        if (cmpTimeUs(microsISR(), timeoutStartUs) >= I2C_TIMEOUT_US) {
            return i2cHandleHardwareFailure(device);
        }
    }
    
    // Disable all interrupts first
    I2C_CTL0(I2Cx) &= ~(I2C_CTL0_TIE | I2C_CTL0_RBNEIE | I2C_CTL0_TCIE | I2C_CTL0_NACKIE | I2C_CTL0_STPDETIE | I2C_CTL0_ERRIE);

    // Flush TDATA before starting
    I2C_STAT(I2Cx) |= I2C_STAT_TBE;

    // H7 Logic: 
    // If reg != 0xFF, we first Write the Register Address (1 byte), then Restart and Read.
    // If reg == 0xFF, we just Read.

    uint32_t ctl1 = I2C_CTL1(I2Cx);
    ctl1 &= ~(I2C_CTL1_BYTENUM | I2C_CTL1_SADDRESS | I2C_CTL1_TRDIR | I2C_CTL1_RELOAD | I2C_CTL1_AUTOEND);
    ctl1 |= (state->addr & I2C_CTL1_SADDRESS);

    if (state->reading && (ev_state->subaddress_sent || 0xFF == state->reg)) {              // we have sent the subaddr
        // Direct Read
        ctl1 |= ((uint32_t)len << 16); // BYTENUM = len
        ctl1 |= I2C_CTL1_TRDIR; // Master Receive
        ev_state->subaddress_sent = 1; // Skip subaddress phase
        
        I2C_CTL1(I2Cx) = ctl1;
        
        // Enable Interrupts for RX (RBNEIE)
        I2C_CTL1(I2Cx) |= I2C_CTL1_START;
        I2C_CTL0(I2Cx) |= (I2C_CTL0_RBNEIE | I2C_CTL0_TCIE | I2C_CTL0_NACKIE | I2C_CTL0_STPDETIE | I2C_CTL0_ERRIE);
    } else {                                                                                
        // Phase 1: Write Subaddress
        ctl1 |= ((uint32_t)(0x01) << 16); // BYTENUM = 1
        ctl1 &= ~I2C_CTL1_TRDIR; // Master Transmit
        if (state->reg != 0xFF)                                                            
                ev_state->index = -1;
        I2C_CTL1(I2Cx) = ctl1;
        
        // Enable Interrupts for TX (TIE)
        I2C_CTL1(I2Cx) |= I2C_CTL1_START;
        I2C_CTL0(I2Cx) |= (I2C_CTL0_TIE | I2C_CTL0_TCIE | I2C_CTL0_NACKIE | I2C_CTL0_STPDETIE | I2C_CTL0_ERRIE);
    }
   
    return true;
}
  
bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    return i2cReadBuffer(device, addr_, reg_, len, buf) && i2cWait(device);
}

static void i2c_er_handler(i2cDevice_e device)
{
    uint32_t I2Cx = (uint32_t )(i2cDevice[device].hardware->reg);

    i2cState_t *state = &i2cDevice[device].state;
    
    // Read the I2C status register
    volatile uint32_t status = I2C_STAT(I2Cx);

    // Map F4 errors to H7 errors
    // F4: BERR, LOSTARB, AERR (Ack Error), OUERR
    // H7: BERR, LOSTARB, NACK, OUERR, PECERR, TIMEOUT
    
    if (status & (I2C_STAT_BERR | I2C_STAT_LOSTARB | I2C_STAT_NACK | I2C_STAT_OUERR | I2C_STAT_PECERR | I2C_STAT_TIMEOUT)) {
        state->error = true;
        
        // Clear errors
        uint32_t clearMask = 0;
        if (status & I2C_STAT_BERR) clearMask |= I2C_STATC_BERRC;
        if (status & I2C_STAT_LOSTARB) clearMask |= I2C_STATC_LOSTARBC;
        if (status & I2C_STAT_NACK) clearMask |= I2C_STATC_NACKC;
        if (status & I2C_STAT_OUERR) clearMask |= I2C_STATC_OUERRC;
        if (status & I2C_STAT_PECERR) clearMask |= I2C_STATC_PECERRC;
        if (status & I2C_STAT_TIMEOUT) clearMask |= I2C_STATC_TIMEOUTC;
        
        I2C_STATC(I2Cx) = clearMask;

        // Stop and Disable Interrupts
        I2C_CTL1(I2Cx) |= I2C_CTL1_STOP;
        I2C_CTL0(I2Cx) &= ~(I2C_CTL0_TIE | I2C_CTL0_RBNEIE | I2C_CTL0_TCIE | I2C_CTL0_NACKIE | I2C_CTL0_STPDETIE | I2C_CTL0_ERRIE);
        
        state->busy = 0;
    }
}

void i2c_ev_handler(i2cDevice_e device)
{
    uint32_t I2Cx = (uint32_t)(i2cDevice[device].hardware->reg);

    i2cEvState_t *ev_state = &i2c_ev_state[device];
    i2cState_t *state = &i2cDevice[device].state;

    uint32_t status = I2C_STAT(I2Cx);
    uint32_t ctl0 = I2C_CTL0(I2Cx);

    // 1. NACK Received (Error) - Highest Priority
    if ((ctl0 & I2C_CTL0_NACKIE) && (status & I2C_STAT_NACK)) {
        I2C_STATC(I2Cx) |= I2C_STATC_NACKC;
        state->error = true;

        // state->busy = 0;
        I2C_CTL0(I2Cx) &= ~(I2C_CTL0_TIE | I2C_CTL0_RBNEIE | I2C_CTL0_TCIE | I2C_CTL0_NACKIE | I2C_CTL0_ERRIE);
        // IP automatically sends STOP on NACK, so we wait for STPDET.
        // Do not disable interrupts here, let STPDET handle cleanup.
    }
    // 2. Stop Detection (STPDET) - End of Transaction
    else if ((ctl0 & I2C_CTL0_STPDETIE) && (status & I2C_STAT_STPDET)) {
        I2C_STATC(I2Cx) |= I2C_STATC_STPDETC;

        // Disable ALL interrupts
        I2C_CTL0(I2Cx) &= ~(I2C_CTL0_TIE | I2C_CTL0_RBNEIE | I2C_CTL0_TCIE | I2C_CTL0_NACKIE | I2C_CTL0_STPDETIE | I2C_CTL0_ERRIE);
        
        state->busy = 0;
    }
    // 3. Receive Buffer Not Empty (RBNE) - Read Data
    // Must be handled before TC to ensure we read the last byte.
    else if ((ctl0 & I2C_CTL0_RBNEIE) && (status & I2C_STAT_RBNE)) {
        if (state->reading && ev_state->index < state->bytes) {
            state->read_p[ev_state->index++] = (uint8_t)I2C_RDATA(I2Cx);
        } else {
            // Flush garbage
            volatile uint8_t dummy = (uint8_t)I2C_RDATA(I2Cx);
            (void)dummy;
        }
    }
    // 4. Transfer Complete (TC) - End of Block (Restart or Stop)
    else if ((ctl0 & I2C_CTL0_TCIE) && (status & I2C_STAT_TC)) {
        // Check if we are in Phase 1 of Read (Write Reg Address done)
        if (state->reading && !ev_state->subaddress_sent) {
            // Phase 1 (Write Reg) complete. Start Phase 2 (Read Data).
            ev_state->subaddress_sent = 1;
            ev_state->index = 0;

            // CRITICAL: Switch Interrupts - Disable TIE, Enable RBNEIE
            // This prevents TI from firing again and causing the "re-send address" bug.
            uint32_t new_ctl0 = ctl0 & ~I2C_CTL0_TIE;
            new_ctl0 |= I2C_CTL0_RBNEIE;
            I2C_CTL0(I2Cx) = new_ctl0;

            // Reconfigure for Read and Restart
            uint32_t ctl1 = I2C_CTL1(I2Cx);
            ctl1 &= ~(I2C_CTL1_BYTENUM | I2C_CTL1_TRDIR); // Clear Len and Dir
            ctl1 |= ((uint32_t)(state->bytes) << 16); // BYTENUM = len
            ctl1 |= I2C_CTL1_TRDIR; // Master Receive
            ctl1 |= I2C_CTL1_START; // Set START bit (Restart)
            I2C_CTL1(I2Cx) = ctl1;
  
        } else {
            // Transaction Complete -> Send Stop
            I2C_CTL1(I2Cx) |= I2C_CTL1_STOP;
            // Wait for STPDET to finish up.
        }
    }
    // 5. Transmit Interrupt (TI) - Write Data
    // Lowest priority, only if we are not done and not in error/stop state.
    else if ((ctl0 & I2C_CTL0_TIE) && (status & I2C_STAT_TI)) {
        if (ev_state->index == -1) {
            // Sending Subaddress (Register Address)
            I2C_TDATA(I2Cx) = state->reg;
            ev_state->index = 0;
            // Note: TI clears when TDATA is written.
        } else if (state->writing && ev_state->index < state->bytes) {
            // Sending Data
            I2C_TDATA(I2Cx) = state->write_p[ev_state->index++];
        }
    }
}

void i2cInit(i2cDevice_e device)
{
    if (device == I2CINVALID)
        return;

    i2cDevice_t *pDev = &i2cDevice[device];
    const i2cHardware_t *hw = pDev->hardware;
    const IO_t scl = pDev->scl;
    const IO_t sda = pDev->sda;

    if (!hw || IOGetOwner(scl) || IOGetOwner(sda)) {
        return;
    }

    uint32_t I2Cx = (uint32_t )i2cDevice[device].reg;
    memset(&pDev->state, 0, sizeof(pDev->state));
    memset(&i2c_ev_state[device], 0, sizeof(i2cEvState_t));

    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    // Enable RCC
    RCC_ClockCmd(hw->rcc, ENABLE);

    i2cUnstick(scl, sda);

     // Init pins
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sclAF);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sdaAF);

    // Reset I2C
    i2c_deinit(I2Cx);
    
    // Configure timing for 400kHz
    pDev->clockSpeed = ((pDev->clockSpeed > 400) ? 400 : pDev->clockSpeed);
    
    // Configure I2C timing - values for 400kHz 
    i2c_timing_config(I2Cx, 0x1, 0x7, 0x0);
    i2c_master_clock_config(I2Cx, 0x2D, 0x87);
    
    // Configure address format
    i2c_address_config(I2Cx, 0, I2C_ADDFORMAT_7BITS);
    
    // Enable clock stretching and other features
    i2c_stretch_scl_low_enable(I2Cx);
    i2c_analog_noise_filter_enable(I2Cx);
    
    // Enable I2C
    i2c_enable(I2Cx);
    
    // I2C Interrupt
    nvic_irq_enable(hw->er_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER));
    nvic_irq_enable(hw->ev_irq, NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV), NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV));
}


uint16_t i2cGetErrorCounter(void)
{
    return i2cErrorCount;
}

#endif

