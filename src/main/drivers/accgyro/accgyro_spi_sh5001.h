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
 * SH5001 6-Axis IMU (Gyroscope + Accelerometer) driver for Betaflight
 * SH5001 is pin-to-pin compatible with ICM42688P
 *
 * SH5001 SPI protocol: standard 4-wire SPI
 *   - Read:  first byte = (reg | 0x80), receive data in subsequent bytes
 *   - Write: first byte = (reg & 0x7F), send data in subsequent bytes
 *   - Data byte order: little-endian (low byte first)
 *   - Registers > 0x7F require page 2 selection via register 0x7F
 */

#pragma once

#include "drivers/bus.h"

// SH5001 data registers (Page 1, little-endian)
#define SH5001_RA_ACC_DATA_XL       0x00  // ACC X low byte
#define SH5001_RA_ACC_DATA_XH       0x01  // ACC X high byte
#define SH5001_RA_ACC_DATA_YL       0x02
#define SH5001_RA_ACC_DATA_YH       0x03
#define SH5001_RA_ACC_DATA_ZL       0x04
#define SH5001_RA_ACC_DATA_ZH       0x05
#define SH5001_RA_GYRO_DATA_XL      0x06  // GYRO X low byte
#define SH5001_RA_GYRO_DATA_XH      0x07
#define SH5001_RA_GYRO_DATA_YL      0x08
#define SH5001_RA_GYRO_DATA_YH      0x09
#define SH5001_RA_GYRO_DATA_ZL      0x0A
#define SH5001_RA_GYRO_DATA_ZH      0x0B
#define SH5001_RA_TEMP_DATA_L       0x0C
#define SH5001_RA_TEMP_DATA_H       0x0D

// Status registers
#define SH5001_RA_INT_STA0          0x16
#define SH5001_RA_INT_STA1          0x17

// CHIP_ID register (Page 1)
#define SH5001_RA_CHIP_ID           0x1F

// Accelerometer configuration registers (Page 1)
#define SH5001_RA_ACC_CONF0         0x20  // filter enable
#define SH5001_RA_ACC_CONF1         0x21  // range[6:4], ODR[3:0]
#define SH5001_RA_ACC_CONF2         0x22  // LPF cutoff factor[3:0]

// Gyroscope configuration registers (Page 1)
#define SH5001_RA_GYRO_CONF0        0x23  // filter enable
#define SH5001_RA_GYRO_CONF1        0x24  // range[6:4], ODR[3:0]
#define SH5001_RA_GYRO_CONF2        0x25  // LPF cutoff factor[3:0]

// Temperature/Timestamp config (Page 1)
#define SH5001_RA_TEMP_CONF1        0x29  // room temp 7:0
#define SH5001_RA_TEMP_CONF2        0x2A  // room temp 11:8, timestamp config

// AGC/Drive config (Page 1)
#define SH5001_RA_AGC_CONFIG1       0x2B

// Power mode register (Page 1)
#define SH5001_RA_POWER_MODE        0x30

// Interface config (Page 1)
#define SH5001_RA_INTF_CONF         0x34

// FIFO config (Page 1)
#define SH5001_RA_FIFO_CONF0        0x35

// Interrupt enable registers (Page 1)
#define SH5001_RA_INT_ENABLE0       0x40
#define SH5001_RA_INT_ENABLE1       0x41

// Interrupt configuration register (Page 1)
#define SH5001_RA_INT_CONF          0x42

// Interrupt pin mapping (Page 1)
#define SH5001_RA_INT_PIN_MAP0      0x66
#define SH5001_RA_INT_PIN_MAP1      0x67

// Page select register (selects page 1 or page 2 for registers > 0x7F)
#define SH5001_RA_PAGE_SEL          0x7F

// Page 2 register offsets (accessed as regAddr & 0x7F after selecting page 2)
#define SH5001_RA_P2_ADC_DRV        (0xBC & 0x7F)   // 0x3C: DeadZone/Dither
#define SH5001_RA_P2_CVA_CTRL       (0xDE & 0x7F)   // 0x5E: CVA control
#define SH5001_RA_P2_CVA_CD         (0xCD & 0x7F)   // 0x4D: CVA channel D X
#define SH5001_RA_P2_CVA_CE         (0xCE & 0x7F)   // 0x4E: CVA channel D Y
#define SH5001_RA_P2_CVA_CF         (0xCF & 0x7F)   // 0x4F: CVA channel D Z
#define SH5001_RA_P2_ADC_D2         (0xD2 & 0x7F)   // 0x52: ADC control
#define SH5001_RA_P2_ADC_D1         (0xD1 & 0x7F)   // 0x51: ADC control
#define SH5001_RA_P2_ADC_D5         (0xD5 & 0x7F)   // 0x55: ADC control
#define SH5001_RA_P2_ACC_RST        (0xD8 & 0x7F)   // 0x58: ACC reset (3.3V)

// ACC CONFIG1 range bits [6:4]
#define SH5001_ACC_RANGE_2G         (0x00 << 4)
#define SH5001_ACC_RANGE_4G         (0x01 << 4)
#define SH5001_ACC_RANGE_8G         (0x02 << 4)
#define SH5001_ACC_RANGE_16G        (0x03 << 4)

// ACC ODR bits [3:0] in ACC_CONF1
#define SH5001_ACC_ODR_1000HZ       0x00
#define SH5001_ACC_ODR_500HZ        0x01
#define SH5001_ACC_ODR_250HZ        0x02
#define SH5001_ACC_ODR_125HZ        0x03
#define SH5001_ACC_ODR_2000HZ       0x08
#define SH5001_ACC_ODR_4000HZ       0x09
#define SH5001_ACC_ODR_8000HZ       0x0A

// GYRO CONFIG1 range bits [6:4]
#define SH5001_GYRO_RANGE_31DPS     (0x00 << 4)
#define SH5001_GYRO_RANGE_62DPS     (0x01 << 4)
#define SH5001_GYRO_RANGE_125DPS    (0x02 << 4)
#define SH5001_GYRO_RANGE_250DPS    (0x03 << 4)
#define SH5001_GYRO_RANGE_500DPS    (0x04 << 4)
#define SH5001_GYRO_RANGE_1000DPS   (0x05 << 4)
#define SH5001_GYRO_RANGE_2000DPS   (0x06 << 4)

// GYRO ODR bits [3:0] in GYRO_CONF1
#define SH5001_GYRO_ODR_1000HZ      0x00
#define SH5001_GYRO_ODR_500HZ       0x01
#define SH5001_GYRO_ODR_250HZ       0x02
#define SH5001_GYRO_ODR_125HZ       0x03
#define SH5001_GYRO_ODR_2000HZ      0x08
#define SH5001_GYRO_ODR_4000HZ      0x09
#define SH5001_GYRO_ODR_8000HZ      0x0A
#define SH5001_GYRO_ODR_16000HZ     0x0B

// LPF cutoff frequency factor (N), cutoff = ODR * N
// At 8kHz ODR: min valid N is 0x0C (0.04 * 8000 = 320 Hz)
// At 1kHz ODR: min valid N is 0x00 (0.40 * 1000 = 400 Hz)
#define SH5001_LPF_ODRX040         0x00   // 0.40 * ODR
#define SH5001_LPF_ODRX004         0x0C   // 0.04 * ODR (use at 8kHz)

// ACC CONF0 / GYRO CONF0 filter bits
#define SH5001_FILTER_EN            0x01   // bit0: digital filter enable
#define SH5001_FILTER_BYPASS        0x02   // bit1: bypass LPF

// INT_CONF (0x42) bits
#define SH5001_INT_ACTIVE_HIGH      0x00   // bit7=0: active high
#define SH5001_INT_ACTIVE_LOW       0x80   // bit7=1: active low
#define SH5001_INT_NO_LATCH         0x40   // bit6=1: no latch (pulsed)
#define SH5001_INT1_PUSHPULL        0x00   // bit3=0: push-pull
#define SH5001_INT1_OPENDRAIN       0x08   // bit3=1: open-drain
#define SH5001_INT1_OUTPUT_EN       0x04   // bit2=1: INT1 output enable

// INT_ENABLE1 (0x41) bits
#define SH5001_GYRO_DRDY_INT_EN     0x04   // bit2: gyro data ready interrupt enable

// Exported functions
bool sh5001SpiAccDetect(accDev_t *acc);
bool sh5001SpiGyroDetect(gyroDev_t *gyro);

void sh5001AccInit(accDev_t *acc);
void sh5001GyroInit(gyroDev_t *gyro);

uint8_t sh5001SpiDetect(const extDevice_t *dev);
