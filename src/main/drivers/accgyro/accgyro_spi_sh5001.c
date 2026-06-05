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

/*
 * SH5001 6-Axis IMU driver for Betaflight
 *
 * SH5001 is a 6-axis IMU (gyro + accel) from Senodia.
 * It is pin-to-pin compatible with ICM42688P.
 *
 * Key differences from ICM42688P:
 * - Different register map (no bank switching for main registers)
 * - Data registers: ACC at 0x00-0x05, GYRO at 0x06-0x0B (little-endian)
 * - CHIP_ID at register 0x1F (expected 0xA1)
 * - SPI max frequency: 10 MHz
 * - Complex initialization sequence required (ADC reset, CVA reset)
 * - Registers 0x80-0xFF accessible via page 2 (write 0x01 to reg 0x7F)
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_GYRO_SPI_SH5001

#include "common/axis.h"
#include "build/debug.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_spi_sh5001.h"
#include "drivers/bus_spi.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/sensor.h"
#include "drivers/time.h"
#include "drivers/pwm_output.h"

#include "sensors/gyro.h"
#include "pg/gyrodev.h"

// SH5001 SPI max frequency: 10 MHz (lower than ICM42688's 24 MHz)
#define SH5001_MAX_SPI_CLK_HZ   10000000

// Minimum EXTI count to confirm interrupt connectivity (same as other SPI gyro drivers)
#define GYRO_EXTI_DETECT_THRESHOLD 1000

// Forward declarations
bool sh5001GyroRead(gyroDev_t *gyro);
bool sh5001AccRead(accDev_t *acc);

// -------------------------------------------------------------------------
// Page 2 access helpers
// -------------------------------------------------------------------------

static void sh5001SelectPage2(const extDevice_t *dev)
{
    spiWriteReg(dev, SH5001_RA_PAGE_SEL, 0x01);
}

static void sh5001SelectPage1(const extDevice_t *dev)
{
    spiWriteReg(dev, SH5001_RA_PAGE_SEL, 0x00);
}

static void sh5001WriteRegPage2(const extDevice_t *dev, uint8_t reg, uint8_t data)
{
    sh5001SelectPage2(dev);
    spiWriteReg(dev, reg & 0x7F, data);
    sh5001SelectPage1(dev);
}

static uint8_t sh5001ReadRegPage2(const extDevice_t *dev, uint8_t reg)
{
    sh5001SelectPage2(dev);
    uint8_t val = spiReadRegMsk(dev, reg & 0x7F);
    sh5001SelectPage1(dev);
    return val;
}

static void sh5001WriteRegMasked(const extDevice_t *dev, uint8_t reg, uint8_t preserveMask, uint8_t value)
{
    uint8_t regData = spiReadRegMsk(dev, reg);
    regData = (regData & preserveMask) | value;
    spiWriteReg(dev, reg, regData);
}

static void sh5001SetRegBits(const extDevice_t *dev, uint8_t reg, uint8_t bits)
{
    const uint8_t regData = spiReadRegMsk(dev, reg) | bits;
    spiWriteReg(dev, reg, regData);
}

static void sh5001ClearRegBits(const extDevice_t *dev, uint8_t reg, uint8_t bits)
{
    const uint8_t regData = spiReadRegMsk(dev, reg) & ~bits;
    spiWriteReg(dev, reg, regData);
}

// -------------------------------------------------------------------------
// Initialization sub-routines (ported from Senodia reference driver)
// -------------------------------------------------------------------------

static void sh5001SoftReset(const extDevice_t *dev)
{
    // Unlock reset, then issue soft reset command
    spiWriteReg(dev, SH5001_RA_AGC_CONFIG1, 0x01);
    // Writing 0x73 to register 0x00 triggers soft reset (special hidden command)
    spiWriteReg(dev, SH5001_RA_ACC_DATA_XL, 0x73);
    delay(50);
}

static void sh5001DriveStart(const extDevice_t *dev)
{
    spiWriteReg(dev, SH5001_RA_AGC_CONFIG1, 0x01);
    delay(2);
    spiWriteReg(dev, SH5001_RA_AGC_CONFIG1, 0x00);
    delay(1);
}

static void sh5001ADCReset(const extDevice_t *dev)
{
    spiWriteReg(dev, SH5001_RA_POWER_MODE, 0x08);

    sh5001WriteRegPage2(dev, 0xD2, 0x00);
    sh5001WriteRegPage2(dev, 0xD1, 0x6B);
    sh5001WriteRegPage2(dev, 0xD5, 0x02);
    delay(5);

    sh5001WriteRegPage2(dev, 0xD1, 0x68);
    delay(2);

    sh5001WriteRegPage2(dev, 0xD5, 0x00);
    spiWriteReg(dev, SH5001_RA_POWER_MODE, 0x00);
    delay(50);
}

static void sh5001CVAReset(const extDevice_t *dev)
{
    uint8_t regDEData = sh5001ReadRegPage2(dev, 0xDE);

    sh5001WriteRegPage2(dev, 0xDE, regDEData & 0xC7);
    delay(5);
    sh5001WriteRegPage2(dev, 0xDE, regDEData | 0x38);
    delay(5);
    sh5001WriteRegPage2(dev, 0xDE, regDEData);
    delay(5);

    // CVA channel reset sequence
    sh5001WriteRegPage2(dev, 0xCD, 0x12);
    sh5001WriteRegPage2(dev, 0xCE, 0x12);
    sh5001WriteRegPage2(dev, 0xCF, 0x12);
    delay(1);

    sh5001WriteRegPage2(dev, 0xCD, 0x02);
    sh5001WriteRegPage2(dev, 0xCE, 0x02);
    sh5001WriteRegPage2(dev, 0xCF, 0x02);
}

// ACC reset for 3.3V VDD supply (SENODIA_VDD_3V3)
static void sh5001AccReset(const extDevice_t *dev)
{
    spiWriteReg(dev, SH5001_RA_POWER_MODE, 0x08);
    sh5001WriteRegPage2(dev, 0xD8, 0xE0);
    delay(5);
    sh5001WriteRegPage2(dev, 0xD8, 0x00);
    spiWriteReg(dev, SH5001_RA_POWER_MODE, 0x00);
}

// Enable dead-zone dither to reduce noise
static void sh5001DeadZoneDither(const extDevice_t *dev)
{
    uint8_t regData = spiReadRegMsk(dev, SH5001_RA_ACC_CONF0);
    regData |= 0x40;
    spiWriteReg(dev, SH5001_RA_ACC_CONF0, regData);

    regData = sh5001ReadRegPage2(dev, 0xBC);
    regData |= 0x01;
    sh5001WriteRegPage2(dev, 0xBC, regData);
}

static void sh5001AccConfig(const extDevice_t *dev, uint8_t odr, uint8_t range, uint8_t cutoff, uint8_t filter, uint8_t bypass)
{
    sh5001WriteRegMasked(dev, SH5001_RA_ACC_CONF0, 0xFC, filter | bypass);
    sh5001WriteRegMasked(dev, SH5001_RA_ACC_CONF1, 0x80, odr | range);
    sh5001WriteRegMasked(dev, SH5001_RA_ACC_CONF2, 0xF0, cutoff);
}

static void sh5001GyroConfig(const extDevice_t *dev, uint8_t odr, uint8_t range, uint8_t cutoff, uint8_t filter, uint8_t bypass)
{
    sh5001WriteRegMasked(dev, SH5001_RA_GYRO_CONF0, 0x7C, filter | bypass);
    sh5001WriteRegMasked(dev, SH5001_RA_GYRO_CONF1, 0x80, odr | range);
    sh5001WriteRegMasked(dev, SH5001_RA_GYRO_CONF2, 0xF0, cutoff);
}

// -------------------------------------------------------------------------
// Detection
// -------------------------------------------------------------------------

uint8_t sh5001SpiDetect(const extDevice_t *dev)
{
    // Ensure we are on page 1 before reading CHIP_ID
    spiWriteReg(dev, SH5001_RA_PAGE_SEL, 0x00);

    uint8_t attemptsRemaining = 5;
    do {
        delay(1);
        const uint8_t chipId = spiReadRegMsk(dev, SH5001_RA_CHIP_ID);
        if (chipId == SH5001_WHO_AM_I_CONST) {
            return SH5001_SPI;
        }
    } while (attemptsRemaining--);

    return MPU_NONE;
}

// -------------------------------------------------------------------------
// Gyroscope
// -------------------------------------------------------------------------

bool sh5001SpiGyroDetect(gyroDev_t *gyro)
{
    if (gyro->mpuDetectionResult.sensor != SH5001_SPI) {
        return false;
    }

    gyro->initFn = sh5001GyroInit;
    gyro->readFn = sh5001GyroRead;

    return true;
}

void sh5001GyroInit(gyroDev_t *gyro)
{
    const extDevice_t *dev = &gyro->dev;

    spiSetClkDivisor(dev, spiCalculateDivider(SH5001_MAX_SPI_CLK_HZ));

    mpuGyroInit(gyro);

    // Set acc and gyro data start registers (little-endian, low byte first)
    gyro->accDataReg  = SH5001_RA_ACC_DATA_XL;
    gyro->gyroDataReg = SH5001_RA_GYRO_DATA_XL;

    // Full module reset sequence (ported from Senodia reference driver)
    sh5001SoftReset(dev);     // soft reset, waits 50ms internally
    sh5001DriveStart(dev);
    sh5001ADCReset(dev);
    sh5001CVAReset(dev);
    delay(200);
    sh5001AccReset(dev);  // 3.3V VDD supply reset

    // Configure gyroscope: ODR 8kHz, ±2000 dps, LPF at 0.04 * ODR = 320 Hz.
    sh5001GyroConfig(dev,
        SH5001_GYRO_ODR_8000HZ,
        SH5001_GYRO_RANGE_2000DPS,
        SH5001_LPF_ODRX004,
        SH5001_FILTER_EN,
        0);
    gyro->scale = GYRO_SCALE_2000DPS;

    // Configure accelerometer: ODR 1kHz, ±16g, LPF at 0.40 * ODR = 400 Hz.
    sh5001AccConfig(dev,
        SH5001_ACC_ODR_1000HZ,
        SH5001_ACC_RANGE_16G,
        SH5001_LPF_ODRX040,
        SH5001_FILTER_EN,
        0);

    // Enable dead-zone dither (improves noise performance)
    sh5001DeadZoneDither(dev);

    // Configure INT1 as active high, auto-clear (pulsed), push-pull output; clear DRDY on data reads.
    spiWriteReg(dev, SH5001_RA_INT_CONF, SH5001_INT_ACTIVE_HIGH | SH5001_INT1_AUTO_CLEAR | SH5001_INT_CLEAR_ANY_READ | SH5001_INT1_PUSHPULL | SH5001_INT2_NO_OUTPUT);

    // Enable gyro data ready interrupt and map it to INT1 without disturbing other interrupt bits.
    sh5001SetRegBits(dev, SH5001_RA_INT_ENABLE1, SH5001_GYRO_DRDY_INT_EN);
    sh5001ClearRegBits(dev, SH5001_RA_INT_PIN_MAP1, SH5001_GYRO_DRDY_INT_EN);
}

// Gyro data read function (no byte-swap needed - SH5001 data is little-endian)
bool sh5001GyroRead(gyroDev_t *gyro)
{
    extDevice_t *dev = &gyro->dev;
    int16_t *gyroData = (int16_t *)dev->rxBuf;

    switch (gyro->gyroModeSPI) {
    case GYRO_EXTI_INIT:
    {
        memset(dev->txBuf, 0xff, 16);

        gyro->gyroDmaMaxDuration = 5;
        if (gyro->detectedEXTI > GYRO_EXTI_DETECT_THRESHOLD) {
#ifdef USE_DMA
            if (spiUseDMA(dev)) {
                dev->callbackArg = (uintptr_t)gyro;
                // Start burst read from ACC_XL (0x00), covering 6 ACC bytes + 6 GYRO bytes
                dev->txBuf[0] = gyro->accDataReg | 0x80;
                // len = gyroDataReg - accDataReg + 1 command + 6 gyro bytes
                gyro->segments[0].len = gyro->gyroDataReg - gyro->accDataReg + sizeof(uint8_t) + 3 * sizeof(int16_t);
                gyro->segments[0].callback = mpuIntCallback;
                gyro->segments[0].u.buffers.txData = dev->txBuf;
                gyro->segments[0].u.buffers.rxData = &dev->rxBuf[1];
                gyro->segments[0].negateCS = true;
                gyro->gyroModeSPI = GYRO_EXTI_INT_DMA;
            } else
#endif
            {
                gyro->gyroModeSPI = GYRO_EXTI_INT;
            }
        } else {
            gyro->gyroModeSPI = GYRO_EXTI_NO_INT;
        }
        break;
    }

    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        // Single-shot read of gyro registers
        dev->txBuf[0] = gyro->gyroDataReg | 0x80;

        busSegment_t segments[] = {
            {.u.buffers = {NULL, NULL}, 7, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = dev->txBuf;
        segments[0].u.buffers.rxData = &dev->rxBuf[1];

        spiSequence(dev, &segments[0]);
        spiWait(dev);

        // SH5001 data is little-endian: low byte at rxBuf[2], high byte at rxBuf[3]
        // As int16_t on ARM little-endian: gyroData[1] = rxBuf[2] | (rxBuf[3] << 8)
        // = GYRO_XL | (GYRO_XH << 8) = correct signed value, no bswap needed
        gyro->gyroADCRaw[X] = gyroData[1];
        gyro->gyroADCRaw[Y] = gyroData[2];
        gyro->gyroADCRaw[Z] = gyroData[3];
        break;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // Burst DMA read started from accDataReg (0x00), rxData = &rxBuf[1]
        // rxBuf layout:
        //   [0]: not used (SPI command sent phase)
        //   [1]: dummy (command response byte)
        //   [2]: ACC_XL, [3]: ACC_XH
        //   [4]: ACC_YL, [5]: ACC_YH
        //   [6]: ACC_ZL, [7]: ACC_ZH
        //   [8]: GYRO_XL, [9]: GYRO_XH
        //  [10]: GYRO_YL,[11]: GYRO_YH
        //  [12]: GYRO_ZL,[13]: GYRO_ZH
        //
        // As int16_t: gyroData[4] = rxBuf[9]<<8 | rxBuf[8] = GYRO_XL|(GYRO_XH<<8) -> correct
        const uint8_t gyroDataIndex = ((gyro->gyroDataReg - gyro->accDataReg) >> 1) + 1;
        gyro->gyroADCRaw[X] = gyroData[gyroDataIndex];
        gyro->gyroADCRaw[Y] = gyroData[gyroDataIndex + 1];
        gyro->gyroADCRaw[Z] = gyroData[gyroDataIndex + 2];
        break;
    }

    default:
        break;
    }

    return true;
}

// -------------------------------------------------------------------------
// Accelerometer
// -------------------------------------------------------------------------

bool sh5001SpiAccDetect(accDev_t *acc)
{
    if (acc->mpuDetectionResult.sensor != SH5001_SPI) {
        return false;
    }

    acc->initFn = sh5001AccInit;
    acc->readFn = sh5001AccRead;

    return true;
}

void sh5001AccInit(accDev_t *acc)
{
    // ±16g range: sensitivity = 2048 LSB/g → acc_1G = 2048
    // betaflight uses acc_1G = 512 * scale_factor
    // For 16g (2048 LSB/g): acc_1G = 2048 → 512 * 4
    acc->acc_1G = 512 * 4;
}

bool sh5001AccRead(accDev_t *acc)
{
    extDevice_t *dev = &acc->gyro->dev;

    switch (acc->gyro->gyroModeSPI) {
    case GYRO_EXTI_INT:
    case GYRO_EXTI_NO_INT:
    {
        dev->txBuf[0] = acc->gyro->accDataReg | 0x80;

        busSegment_t segments[] = {
            {.u.buffers = {NULL, NULL}, 7, true, NULL},
            {.u.link = {NULL, NULL}, 0, true, NULL},
        };
        segments[0].u.buffers.txData = dev->txBuf;
        segments[0].u.buffers.rxData = &dev->rxBuf[1];

        spiSequence(&acc->gyro->dev, &segments[0]);
        spiWait(&acc->gyro->dev);

        FALLTHROUGH;
    }

    case GYRO_EXTI_INT_DMA:
    {
        // SH5001 ACC data is little-endian, no byte-swap needed
        // rxBuf[2]=ACC_XL, rxBuf[3]=ACC_XH → accData[1] = ACC_XL|(ACC_XH<<8) = correct
        int16_t *accData = (int16_t *)acc->gyro->dev.rxBuf;
        acc->ADCRaw[X] = accData[1];
        acc->ADCRaw[Y] = accData[2];
        acc->ADCRaw[Z] = accData[3];
        break;
    }

    case GYRO_EXTI_INIT:
    default:
        break;
    }

    return true;
}

#endif // USE_GYRO_SPI_SH5001
