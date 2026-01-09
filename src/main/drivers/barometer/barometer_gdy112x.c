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

// GDY112X pressure sensor driver for Betaflight

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "common/utils.h"
#include "common/maths.h"

#include "drivers/io.h"
#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/time.h"
#include "drivers/barometer/barometer.h"
#include "drivers/barometer/barometer_gdy112x.h"
#include "drivers/resource.h"

// 10 MHz max SPI frequency
#define GDY112X_MAX_SPI_CLK_HZ 10000000

#if defined(USE_BARO) && (defined(USE_BARO_GDY112X) || defined(USE_BARO_SPI_GDY112X))

#define GDY112X_I2C_ADDR            0x3A

// GDY112X Register definitions
#define GDY112X_REG_CHIP_ID         0x00
#define GDY112X_REG_REV_ID          0x01
#define GDY112X_REG_ERR_MSG         0x02
#define GDY112X_REG_STATUS          0x03
#define GDY112X_REG_PRESS_XLSB      0x04
#define GDY112X_REG_PRESS_LSB       0x05
#define GDY112X_REG_PRESS_MSB       0x06
#define GDY112X_REG_TEMP_XLSB       0x07
#define GDY112X_REG_TEMP_LSB        0x08
#define GDY112X_REG_TEMP_MSB        0x09
#define GDY112X_REG_SENSOR_TIME_0   0x0C
#define GDY112X_REG_SENSOR_TIME_1   0x0D
#define GDY112X_REG_SENSOR_TIME_2   0x0E
#define GDY112X_REG_INT_STATUS      0x11
#define GDY112X_REG_FIFO_LENGTH_0   0x12
#define GDY112X_REG_FIFO_LENGTH_1   0x13
#define GDY112X_REG_FIFO_DATA       0x14
#define GDY112X_REG_FIFO_WM_0       0x15
#define GDY112X_REG_FIFO_WM_1       0x16
#define GDY112X_REG_FIFO_CONFIG_0   0x17
#define GDY112X_REG_FIFO_CONFIG_1   0x18
#define GDY112X_REG_INT_CTRL        0x19
#define GDY112X_REG_CONFIG          0x1A
#define GDY112X_REG_PWR_CTRL        0x1B
#define GDY112X_REG_OSR             0x1C
#define GDY112X_REG_ODR             0x1D
#define GDY112X_REG_FILTER          0x1F
#define GDY112X_REG_PRIMIF          0x22
#define GDY112X_REG_FILGAIN         0x30
#define GDY112X_REG_AGAIN           0x49
#define GDY112X_OTP_PWR             0x7A
#define GDY112X_OTP_ADDR            0x7B
#define GDY112X_OTP_DATA            0x7C
#define GDY112X_OTP_TRIG            0x7D
#define GDY112X_REG_RESET           0x7E

// GDY112X constants
#define GDY112X_PID                 0xA0
#define GDY112X_VER                 0x80

// Status register bit definitions
#define GDY112X_DRDY_TEMP_MASK      0x40
#define GDY112X_DRDY_PRESS_MASK     0x20

// Power mode definitions
#define GDY112X_SLEEP_MODE          0
#define GDY112X_FORCED_MODE         1
#define GDY112X_NORMAL_MODE         3

// OSR definitions
#define GDY112X_OSR_x1              0
#define GDY112X_OSR_x2              1
#define GDY112X_OSR_x4              2
#define GDY112X_OSR_x8              3
#define GDY112X_OSR_x16             4
#define GDY112X_OSR_x32             5
#define GDY112X_OSR_x64             6
#define GDY112X_OSR_x128            7

// ODR definitions
#define GDY112X_ODR_25ms            0x0E

// Filter definitions
#define GDY112X_FILTER_COE_3        2

// Sensitivity constants (from datasheet)
#define GDY112X_PRESS_SENSITIVITY   64U        // LSB/Pa
#define GDY112X_TEMP_SENSITIVITY    65536U     // LSB/°C

/**
 * Sensor state structure
 * Stores converted pressure and temperature values
 */
typedef struct {
    float        pressure;       // Pressure in Pa
    float        temperature;    // Temperature in °C
} baroState_t;

static baroState_t  baroState;

#define busReadBuf busReadRegisterBuffer
#define busWrite   busWriteRegister

static uint8_t buf[6];
static uint8_t chipId[1];

// Helper functions
static uint8_t registerRead(const extDevice_t *dev, uint8_t reg)
{
    return busReadRegister(dev, reg);
}

static void registerWrite(const extDevice_t *dev, uint8_t reg, uint8_t value)
{
    busWrite(dev, reg, value);
}

static void registerSetBits(const extDevice_t *dev, uint8_t reg, uint8_t setbits)
{
    uint8_t val = registerRead(dev, reg);

    if ((val & setbits) != setbits) {
        registerWrite(dev, reg, val | setbits);
    }
}

static int32_t getTwosComplement(uint32_t raw, uint8_t length)
{
    if (raw & ((int)1 << (length - 1))) {
        return ((int32_t)raw) - ((int32_t)1 << length);
    } else {
        return (int32_t)raw;
    }
}

static bool deviceConfigure(const extDevice_t *dev)
{
    uint8_t regval;

    // Trigger a chip reset
    registerWrite(dev, GDY112X_REG_RESET, 0xB6);

    // Wait for reset to complete
    delay(100);

    // Configure FIFO (disable by default)
    registerWrite(dev, GDY112X_REG_FIFO_CONFIG_0, 0x00);
    registerWrite(dev, GDY112X_REG_FIFO_CONFIG_1, 0x00);

    // Configure interrupt control (disable data ready interrupt)
    registerWrite(dev, GDY112X_REG_INT_CTRL, 0x00);

    // Configure oversampling ratio (pressure: x4, temperature: x2)
    registerWrite(dev, GDY112X_REG_OSR, (GDY112X_OSR_x2 << 4) | GDY112X_OSR_x4);

    // Configure output data rate (25ms = 40Hz)
    registerWrite(dev, GDY112X_REG_ODR, GDY112X_ODR_25ms);

    // Configure IIR filter (both pressure and temperature filter: 3)
    registerWrite(dev, GDY112X_REG_FILTER, (GDY112X_FILTER_COE_3 << 4) | GDY112X_FILTER_COE_3);

    // Enable sensors (0x03 = pressure_enable | temperature_enable)
    registerSetBits(dev, GDY112X_REG_PWR_CTRL, 0x03);

    // Set normal mode
    registerSetBits(dev, GDY112X_REG_PWR_CTRL, ((GDY112X_NORMAL_MODE & 0x03) << 4));

    // Wait for sensor to enter normal mode and stabilize
    delay(40);

    // Verify sensor is ready by checking data ready flags
    regval = registerRead(dev, GDY112X_REG_STATUS);

    // Check if both temperature and pressure data are ready
    // This indicates sensor is working properly
    if ((regval & (GDY112X_DRDY_TEMP_MASK | GDY112X_DRDY_PRESS_MASK)) == 0) {
        return false;
    }

    return true;
}

static bool gdy112xStartUP(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool gdy112xReadUP(baroDev_t *baro)
{
    // Check if bus is busy with another transaction
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // Read pressure and temperature data simultaneously (6 bytes)
    // Register order (little-endian): PRESS_XLSB, PRESS_LSB, PRESS_MSB,
    //                                 TEMP_XLSB, TEMP_LSB, TEMP_MSB
    // This is more efficient than reading them separately
    return busReadRegisterBufferStart(&baro->dev, GDY112X_REG_PRESS_XLSB, buf, 6);
}

static bool gdy112xGetUP(baroDev_t *baro)
{
    UNUSED(baro);

    // Extract pressure data (first 3 bytes: XLSB, LSB, MSB)
    // GDY112X uses little-endian byte order
    uint32_t pressureRaw = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[0];

    // Convert to signed 24-bit value (two's complement)
    int32_t pressureSigned = getTwosComplement(pressureRaw, 24);

    // Extract temperature data (next 3 bytes: XLSB, LSB, MSB)
    uint32_t temperatureRaw = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | (uint32_t)buf[3];

    // Convert to signed 24-bit value
    int32_t temperatureSigned = getTwosComplement(temperatureRaw, 24);

    // Convert raw counts to physical units
    // Pressure: raw_value / sensitivity = Pa
    // Temperature: raw_value / sensitivity = °C
    baroState.pressure = (float)pressureSigned / (float)GDY112X_PRESS_SENSITIVITY;
    baroState.temperature = (float)temperatureSigned / (float)GDY112X_TEMP_SENSITIVITY;

    return true;
}

static bool gdy112xStartUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool gdy112xReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static bool gdy112xGetUT(baroDev_t *baro)
{
    UNUSED(baro);
    return true;
}

static void deviceCalculate(int32_t *pressure, int32_t *temperature)
{
    if (pressure) {
        // Pressure in Pa (integer units, same as DPS310)
        *pressure = (int32_t)(baroState.pressure);
    }
    if (temperature) {
        // Temperature in centidegrees (0.01 °C units)
        *temperature = (int32_t)(baroState.temperature * 100.0f);
    }
}

#define DETECTION_MAX_RETRY_COUNT   5
static bool deviceDetect(const extDevice_t *dev)
{
    for (int retry = 0; retry < DETECTION_MAX_RETRY_COUNT; retry++) {
        delay(100);

        // Read chip ID register
        bool ack = busReadBuf(dev, GDY112X_REG_CHIP_ID, chipId, 1);

        // Verify communication success and correct chip ID (0xA0)
        if (ack && (chipId[0] == GDY112X_PID)) {
            return true;
        }
    }

    return false;
}

static void deviceInit(const extDevice_t *dev, resourceOwner_e owner)
{
#ifdef USE_BARO_SPI_GDY112X
    if (dev->bus->busType == BUS_TYPE_SPI) {
        IOHi(dev->busType_u.spi.csnPin); // Disable
        IOInit(dev->busType_u.spi.csnPin, owner, 0);
        IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_OUT_PP);
        spiSetClkDivisor(dev, spiCalculateDivider(GDY112X_MAX_SPI_CLK_HZ));
    }
#else
    UNUSED(dev);
    UNUSED(owner);
#endif
}

static void deviceDeInit(const extDevice_t *dev)
{
#ifdef USE_BARO_SPI_GDY112X
    if (dev->bus->busType == BUS_TYPE_SPI) {
        ioPreinitByIO(dev->busType_u.spi.csnPin, IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
    }
#else
    UNUSED(dev);
#endif
}

bool baroGDY112XDetect(baroDev_t *baro)
{
    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    deviceInit(&baro->dev, OWNER_BARO_CS);

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default address for GDY112X
        dev->busType_u.i2c.address = GDY112X_I2C_ADDR;
        defaultAddressApplied = true;
    }

    if (!deviceDetect(dev)) {
        deviceDeInit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
        return false;
    }

    if (!deviceConfigure(dev)) {
        deviceDeInit(dev);
        return false;
    }

    busDeviceRegister(dev);

    // Initialize baroDev_t structure
    // With ODR=25ms (40Hz), we should wait at least one measurement cycle
    // Adding margin: 25ms + 5ms = 30ms = 30000µs
    // Note: Betaflight scheduler will call read functions at ~20Hz (50ms interval)
    baro->ut_delay = 0;
    baro->up_delay = 30000;  // 30ms delay ensures fresh pressure data

    // Set up callback functions
    // These are called by Betaflight's barometer task in sequence:
    // 1. start_up/start_ut: Initiate measurement
    // 2. read_up/read_ut: Start bus read transaction
    // 3. get_up/get_ut: Process received data
    // 4. calculate: Convert to final units
    baro->start_ut = gdy112xStartUT;
    baro->read_ut = gdy112xReadUT;
    baro->get_ut = gdy112xGetUT;
    baro->start_up = gdy112xStartUP;
    baro->read_up = gdy112xReadUP;
    baro->get_up = gdy112xGetUP;
    baro->calculate = deviceCalculate;

    return true;
}

#endif
