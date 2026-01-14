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
 *
 * Author: Dominic Clifton
 */

/*
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_OCTOSPI
#include "drivers/system.h"

#include "drivers/bus_octospi.h"
#include "drivers/bus_octospi_impl.h"

#if !(defined(GD32H7))
#error MCU not supported.
#endif

#define OCTOSPI_INTERFACE_COUNT         1

#define OSPI_FUNCTIONAL_MODE_INDIRECT_WRITE ((uint32_t)0x00000000)
#define OSPI_FUNCTIONAL_MODE_INDIRECT_READ  ((uint32_t)OCTOSPI_CR_FMODE_0)
#define OSPI_FUNCTIONAL_MODE_AUTO_POLLING   ((uint32_t)OCTOSPI_CR_FMODE_1)
#define OSPI_FUNCTIONAL_MODE_MEMORY_MAPPED  ((uint32_t)OCTOSPI_CR_FMODE)

#define OSPI_DHQC_DISABLE                ((uint32_t)0x00000000U)
#define OSPI_DHQC_ENABLE                 ((uint32_t)OCTOSPI_TCR_DHQC)

#define OSPI_OPTYPE_COMMON_CFG           ((uint32_t)0x00000000U)

#define OSPI_OPTYPE_READ_CFG             ((uint32_t)0x00000001U)
#define OSPI_OPTYPE_WRITE_CFG            ((uint32_t)0x00000002U)
#define OSPI_OPTYPE_WRAP_CFG             ((uint32_t)0x00000003U)

// #define OSPI_INSTRUCTION_NONE            ((uint32_t)0x00000000U)
// #define OSPI_INSTRUCTION_1_LINE          ((uint32_t)OCTOSPI_CCR_IMODE_0)
// #define OSPI_INSTRUCTION_2_LINES         ((uint32_t)OCTOSPI_CCR_IMODE_1)
// #define OSPI_INSTRUCTION_4_LINES         ((uint32_t)(OCTOSPI_CCR_IMODE_0 | OCTOSPI_CCR_IMODE_1))
// #define OSPI_INSTRUCTION_8_LINES         ((uint32_t)OCTOSPI_CCR_IMODE_2)

// #define OSPI_INSTRUCTION_8_BITS          ((uint32_t)0x00000000U)
// #define OSPI_INSTRUCTION_16_BITS         ((uint32_t)OCTOSPI_CCR_ISIZE_0)
// #define OSPI_INSTRUCTION_24_BITS         ((uint32_t)OCTOSPI_CCR_ISIZE_1)
// #define OSPI_INSTRUCTION_32_BITS         ((uint32_t)OCTOSPI_CCR_ISIZE)

#define OSPI_INSTRUCTION_DTR_DISABLE     ((uint32_t)0x00000000U)
#define OSPI_INSTRUCTION_DTR_ENABLE      ((uint32_t)OCTOSPI_CCR_IDTR)

// #define OSPI_ADDRESS_NONE                ((uint32_t)0x00000000U)                                         /*!< No address               */
// #define OSPI_ADDRESS_1_LINE              ((uint32_t)OCTOSPI_CCR_ADMODE_0)                                /*!< Address on a single line */
// #define OSPI_ADDRESS_2_LINES             ((uint32_t)OCTOSPI_CCR_ADMODE_1)                                /*!< Address on two lines     */
// #define OSPI_ADDRESS_4_LINES             ((uint32_t)(OCTOSPI_CCR_ADMODE_0 | OCTOSPI_CCR_ADMODE_1))       /*!< Address on four lines    */
// #define OSPI_ADDRESS_8_LINES             ((uint32_t)OCTOSPI_CCR_ADMODE_2)                                /*!< Address on eight lines   */

// #define OSPI_ADDRESS_8_BITS              ((uint32_t)0x00000000U)                                         /*!< 8-bit address  */
// #define OSPI_ADDRESS_16_BITS             ((uint32_t)OCTOSPI_CCR_ADSIZE_0)                                /*!< 16-bit address */
// #define OSPI_ADDRESS_24_BITS             ((uint32_t)OCTOSPI_CCR_ADSIZE_1)                                /*!< 24-bit address */
// #define OSPI_ADDRESS_32_BITS             ((uint32_t)OCTOSPI_CCR_ADSIZE)                                  /*!< 32-bit address */

#define OSPI_ADDRESS_DTR_DISABLE         ((uint32_t)0x00000000U)                                         /*!< DTR mode disabled for address phase */
#define OSPI_ADDRESS_DTR_ENABLE          ((uint32_t)OCTOSPI_CCR_ADDTR)

// #define OSPI_DATA_NONE                   ((uint32_t)0x00000000U)
// #define OSPI_DATA_1_LINE                 ((uint32_t)OCTOSPI_CCR_DMODE_0)
// #define OSPI_DATA_2_LINES                ((uint32_t)OCTOSPI_CCR_DMODE_1)
// #define OSPI_DATA_4_LINES                ((uint32_t)(OCTOSPI_CCR_DMODE_0 | OCTOSPI_CCR_DMODE_1))
// #define OSPI_DATA_8_LINES                ((uint32_t)OCTOSPI_CCR_DMODE_2)

#define OSPI_DATA_DTR_DISABLE            ((uint32_t)0x00000000U)
#define OSPI_DATA_DTR_ENABLE             ((uint32_t)OCTOSPI_CCR_DDTR)

// #define OSPI_ALTERNATE_BYTES_NONE        ((uint32_t)0x00000000U)
// #define OSPI_ALTERNATE_BYTES_1_LINE      ((uint32_t)OCTOSPI_CCR_ABMODE_0)
// #define OSPI_ALTERNATE_BYTES_2_LINES     ((uint32_t)OCTOSPI_CCR_ABMODE_1)
// #define OSPI_ALTERNATE_BYTES_4_LINES     ((uint32_t)(OCTOSPI_CCR_ABMODE_0 | OCTOSPI_CCR_ABMODE_1))
// #define OSPI_ALTERNATE_BYTES_8_LINES     ((uint32_t)OCTOSPI_CCR_ABMODE_2)

// #define OSPI_ALTERNATE_BYTES_8_BITS      ((uint32_t)0x00000000U)
// #define OSPI_ALTERNATE_BYTES_16_BITS     ((uint32_t)OCTOSPI_CCR_ABSIZE_0)
// #define OSPI_ALTERNATE_BYTES_24_BITS     ((uint32_t)OCTOSPI_CCR_ABSIZE_1)
// #define OSPI_ALTERNATE_BYTES_32_BITS     ((uint32_t)OCTOSPI_CCR_ABSIZE)

#define OSPI_ALTERNATE_BYTES_DTR_DISABLE ((uint32_t)0x00000000U)
#define OSPI_ALTERNATE_BYTES_DTR_ENABLE  ((uint32_t)OCTOSPI_CCR_ABDTR)

#define OSPI_DQS_DISABLE                 ((uint32_t)0x00000000U)
#define OSPI_DQS_ENABLE                  ((uint32_t)OCTOSPI_CCR_DQSE)

#define OSPI_SIOO_INST_EVERY_CMD         ((uint32_t)0x00000000U)
#define OSPI_SIOO_INST_ONLY_FIRST_CMD    ((uint32_t)OCTOSPI_CCR_SIOO)

static MMFLASH_CODE_NOINLINE void Error_Handler(void) {
    while (1) {
        NOOP;
    }
}

#define __OSPI_GET_FLAG(__INSTANCE__, __FLAG__)           ((READ_BIT(OSPI_STAT(uint32_t __INSTANCE__), (__FLAG__)) != 0U) ? SET : RESET)
#define __OSPI_CLEAR_FLAG(__INSTANCE__, __FLAG__)           //WRITE_REG((__INSTANCE__)->FCR, (__FLAG__))
#define __OSPI_ENABLE(__INSTANCE__)                       //SET_BIT((__INSTANCE__)->CR, OCTOSPI_CR_EN)
#define __OSPI_DISABLE(__INSTANCE__)                      //CLEAR_BIT((__INSTANCE__)->CR, OCTOSPI_CR_EN)
#define __OSPI_IS_ENABLED(__INSTANCE__)                   //(READ_BIT((__INSTANCE__)->CR, OCTOSPI_CR_EN) != 0U)

MMFLASH_CODE_NOINLINE static void octoSpiAbort(OCTOSPI_TypeDef *instance)
{
    UNUSED(instance);
}

// MMFLASH_CODE_NOINLINE static void octoSpiWaitStatusFlags(OCTOSPI_TypeDef *instance, uint32_t mask, FlagStatus flagStatus)
// {
// //     uint32_t regval;

// //     switch (flagStatus) {
// //     case SET:
// //         while (!((regval = READ_REG(instance->SR)) & mask))
// //             {}
// //         break;
// //     case RESET:
// //         while (((regval = READ_REG(instance->SR)) & mask))
// //             {}
// //         break;
// //     }

//     uint32_t regval;
//     uint32_t ospi_periph = PERIPH_INT(instance);

//     switch (flagStatus) {
//     case SET:
//         while (!((regval = READ_REG(OSPI_STAT(ospi_periph))) & mask))
//             {}
//         break;
//     case RESET:
//         while (((regval = READ_REG(OSPI_STAT(ospi_periph))) & mask))
//             {}
//         break;
//     }
// }

typedef struct {
    uint32_t OperationType;

    uint32_t Instruction;
    uint32_t InstructionMode;
    uint32_t InstructionSize;
    uint32_t InstructionDtrMode;

    uint32_t Address;
    uint32_t AddressMode;
    uint32_t AddressSize;
    uint32_t AddressDtrMode;

    uint32_t AlternateBytes;
    uint32_t AlternateBytesMode;
    uint32_t AlternateBytesSize;
    uint32_t AlternateBytesDtrMode;

    uint32_t DummyCycles; // 0-31

    uint32_t DataMode;
    uint32_t DataDtrMode;
    uint32_t NbData;

    uint32_t DQSMode;
    uint32_t SIOOMode;
} OSPI_Command_t;

// TODO rename cmd to command
MMFLASH_CODE_NOINLINE static ErrStatus octoSpiConfigureCommand(OCTOSPI_TypeDef *instance, OSPI_Command_t *cmd)
{
    ErrStatus status = SUCCESS;
    uint32_t ospi_periph = PERIPH_INT(instance);
    ospi_regular_cmd_struct* cmd_struct = (ospi_regular_cmd_struct *)cmd;
    __IO uint32_t *tcfg_reg, *timcfg_reg, *ins_reg, *alte_reg;

    /* re-initialize the value of the functional mode */
    OSPI_CTL(ospi_periph) &= ~OSPI_CTL_FMOD;

    //instance->CCR = (cmd->DQSMode | cmd->SIOOMode);

    if(cmd_struct->operation_type == OSPI_OPTYPE_WRITE_CFG){
        tcfg_reg = &(OSPI_WTCFG(ospi_periph));
        timcfg_reg = &(OSPI_WTIMCFG(ospi_periph));
        ins_reg  = &(OSPI_WINS(ospi_periph));
        alte_reg = &(OSPI_WALTE(ospi_periph));
    }else if(cmd_struct->operation_type == OSPI_OPTYPE_WRAP_CFG){
        tcfg_reg = &(OSPI_WPTCFG(ospi_periph));
        timcfg_reg = &(OSPI_WPTIMCFG(ospi_periph));
        ins_reg  = &(OSPI_WPINS(ospi_periph));
        alte_reg = &(OSPI_WPALTE(ospi_periph));
    }else{
        tcfg_reg = &(OSPI_TCFG(ospi_periph));
        timcfg_reg = &(OSPI_TIMCFG(ospi_periph));
        ins_reg  = &(OSPI_INS(ospi_periph));
        alte_reg = &(OSPI_ALTE(ospi_periph));
    }

    if(cmd_struct->alter_bytes_mode != OSPI_ALTERNATE_BYTES_NONE){
        /* configure the ALTE register with alternate bytes value */
        *alte_reg = cmd_struct->alter_bytes;

        /* configure the TCFG register with alternate bytes communication parameters */
        *tcfg_reg = (*tcfg_reg & ~(OSPI_TCFG_ALTEMOD | OSPI_TCFG_ABDTR | OSPI_TCFG_ALTESZ)) | 
                    (cmd_struct->alter_bytes_mode | cmd_struct->alter_bytes_dtr_mode | cmd_struct->alter_bytes_size);
    }

    /* configure the TIMCFG register with the number of dummy cycles */
    *timcfg_reg = (*timcfg_reg & ~OSPI_TIMCFG_DUMYC) | cmd_struct->dummy_cycles;

    if(cmd_struct->data_mode != OSPI_DATA_NONE){
        if(cmd_struct->operation_type == OSPI_OPTYPE_COMMON_CFG){
            /* configure the DTLEN register with the number of data */
            OSPI_DTLEN(ospi_periph) = (cmd_struct->nbdata - 1U);
        }
    }

    if(cmd_struct->ins_mode != OSPI_INSTRUCTION_NONE) { 
        if(cmd_struct->addr_mode != OSPI_ADDRESS_NONE) {
            if(cmd_struct->data_mode != OSPI_DATA_NONE){
                /* command with instruction, address and data */
                /* configure the TCFG register with all communication parameters */
                *tcfg_reg &= ~(OSPI_TCFG_IMOD | OSPI_TCFG_INSSZ | 
                            OSPI_TCFG_ADDRMOD | OSPI_TCFG_ADDRDTR | OSPI_TCFG_ADDRSZ |
                            OSPI_TCFG_DATAMOD | OSPI_TCFG_DADTR);
                
                *tcfg_reg |= cmd_struct->ins_mode | cmd_struct->ins_size |
                            cmd_struct->addr_mode | cmd_struct->addr_dtr_mode | cmd_struct->addr_size |
                            cmd_struct->data_mode | cmd_struct->data_dtr_mode;
            }else{
                /* command with instruction and address */
                /* configure the TCFG register with all communication parameters */
                *tcfg_reg &= ~(OSPI_TCFG_IMOD | OSPI_TCFG_INSSZ | 
                            OSPI_TCFG_ADDRMOD | OSPI_TCFG_ADDRDTR | OSPI_TCFG_ADDRSZ |
                            OSPI_TCFG_DATAMOD | OSPI_TCFG_DADTR);

                *tcfg_reg |= cmd_struct->ins_mode | cmd_struct->ins_size |
                            cmd_struct->addr_mode | cmd_struct->addr_dtr_mode | cmd_struct->addr_size;

                /* the DHQC bit is linked with DDTR bit which should be activated */
                if(OSPI_DELAY_HOLD_QUARTER_CYCLE == (OSPI_TIMCFG(ospi_periph) & OSPI_TIMCFG_DEHQC)) {
                    *tcfg_reg = (*tcfg_reg & ~OSPI_DADTR_MODE_ENABLE) | OSPI_DADTR_MODE_ENABLE;
                }
            }
            /* configure the INS register with the instruction value */
            *ins_reg = cmd_struct->instruction;
            /* configure the ADDR register with the address value */
            OSPI_ADDR(ospi_periph) = cmd_struct->address;
        } else {
            if(cmd_struct->data_mode != OSPI_DATA_NONE){
                /* command with instruction and data */
                /* configure the TCFG register with all communication parameters */
                *tcfg_reg &= ~(OSPI_TCFG_IMOD | OSPI_TCFG_INSSZ | 
                            OSPI_TCFG_DATAMOD | OSPI_TCFG_DADTR);

                *tcfg_reg = cmd_struct->ins_mode | cmd_struct->ins_size |
                            cmd_struct->data_mode | cmd_struct->data_dtr_mode;
            }else{
                /* command with only instruction */
                /* configure the TCFG register with all communication parameters */
                *tcfg_reg &= ~(OSPI_TCFG_IMOD | OSPI_TCFG_INSSZ);
                
                *tcfg_reg = cmd_struct->ins_mode | cmd_struct->ins_size;

                /* the DEHQC bit is linked with DDTR bit which should be activated */
                if(OSPI_DELAY_HOLD_QUARTER_CYCLE == (OSPI_TIMCFG(ospi_periph) & OSPI_TIMCFG_DEHQC)) {
                    *tcfg_reg = (*tcfg_reg & ~OSPI_DADTR_MODE_ENABLE) | OSPI_DADTR_MODE_ENABLE;
                }
            }

            /* configure the INS register with the instruction value */
            *ins_reg = cmd_struct->instruction;  
        }
    } else {
        if(cmd_struct->addr_mode != OSPI_ADDRESS_NONE) {
            if(cmd_struct->data_mode != OSPI_DATA_NONE) {
                /* command with address and data */

                /* configure the TCFG register with all communication parameters */
                *tcfg_reg &= ~(OSPI_TCFG_ADDRMOD | OSPI_TCFG_ADDRDTR | OSPI_TCFG_ADDRSZ |
                               OSPI_TCFG_DATAMOD | OSPI_TCFG_DADTR);
                
                *tcfg_reg = cmd_struct->addr_mode | cmd_struct->addr_dtr_mode | cmd_struct->addr_size |
                            cmd_struct->data_mode | cmd_struct->data_dtr_mode;              
            } else{
                /* command with only address */

                /* configure the TCFG register with all communication parameters */
                *tcfg_reg &= ~(OSPI_TCFG_ADDRMOD | OSPI_TCFG_ADDRDTR | OSPI_TCFG_ADDRSZ);
                
                *tcfg_reg = cmd_struct->addr_mode | cmd_struct->addr_dtr_mode | cmd_struct->addr_size;                 
            }

            /* configure the ADDR register with the instruction value */
            OSPI_ADDR(ospi_periph) = cmd_struct->address;
        } else {
           /* no instruction, no address */
            status = ERROR;
        }
    }
    return status;
}

static MMFLASH_CODE_NOINLINE ErrStatus octoSpiCommand(OCTOSPI_TypeDef *instance, OSPI_Command_t *cmd)
{
    uint32_t ospi_periph = PERIPH_INT(instance);
    ospi_regular_cmd_struct *cmd_struct = (ospi_regular_cmd_struct *)cmd;
    ErrStatus status = ERROR;

    if(((cmd_struct->operation_type == OSPI_OPTYPE_WRITE_CFG) || (cmd_struct->operation_type == OSPI_OPTYPE_WRAP_CFG)) ||
        ((cmd_struct->operation_type == OSPI_OPTYPE_READ_CFG) || (cmd_struct->operation_type == OSPI_OPTYPE_COMMON_CFG))) {
        /* wait till busy flag is reset */
          while(RESET != (OSPI_STAT(ospi_periph) & OSPI_FLAG_BUSY)) {
          }

        /* configure the registers */
        status = octoSpiConfigureCommand(instance, (OSPI_Command_t *)cmd_struct);

        if(cmd_struct->data_mode == OSPI_DATA_NONE) {
            /* when there is no data phase, the transfer start as soon as the configuration is done
            so wait until TC flag is set to go back in idle state */
          while(RESET == (OSPI_STAT(ospi_periph) & OSPI_FLAG_TC)) {
          }

            OSPI_STATC(ospi_periph) = OSPI_STATC_TCC;
        }
    }
    return status;
}

/*
 * Transmit
 *
 * Call optoSpiCommand first to configure the transaction stages.
 */
static MMFLASH_CODE_NOINLINE ErrStatus octoSpiTransmit(OCTOSPI_TypeDef *instance, uint8_t *data)
{
    uint32_t ospi_periph = PERIPH_INT(instance);

    uint32_t txcounter;
    uint32_t address;
    /* configure counters and size */
    txcounter = OSPI_DTLEN(ospi_periph) + 1U;
    address = (uint32_t)data;

    /* configure CTL register with functional mode as indirect write */
    OSPI_CTL(ospi_periph) = (OSPI_CTL(ospi_periph) & ~OSPI_CTL_FMOD) | OSPI_INDIRECT_WRITE;

    do{
        /* wait till fifo threshold flag is set to send data */
      while(RESET != (OSPI_STAT(ospi_periph) & OSPI_FLAG_FT)){
      }
        *((__IO uint8_t *)&OSPI_DATA(ospi_periph)) = *(uint8_t *)address;
        address++;
        txcounter--;
    }while(txcounter > 0U);

    /* wait till transfer complete flag is set to go back in idle state */
    while(RESET == (OSPI_STAT(ospi_periph) & OSPI_FLAG_TC)){
    }

    /* clear transfer complete flag */
    OSPI_STATC(ospi_periph) = OSPI_STATC_TCC;

    return SUCCESS;
}

/*
 * Receive
 *
 * Call optoSpiCommand first to configure the transaction stages.
 */
static MMFLASH_CODE_NOINLINE ErrStatus octoSpiReceive(OCTOSPI_TypeDef *instance, uint8_t *data)
{
    uint32_t ospi_periph = PERIPH_INT(instance);
    uint32_t rxcounter;
    uint32_t address;
    uint32_t addr_reg = OSPI_ADDR(ospi_periph);
    uint32_t ins_reg = OSPI_INS(ospi_periph);

    /* configure counters and size */
    rxcounter = OSPI_DTLEN(ospi_periph) + 1U;
    address = (uint32_t)data;

    /* configure CTL register with functional mode as indirect read */
    OSPI_CTL(ospi_periph) = (OSPI_CTL(ospi_periph) & ~OSPI_CTL_FMOD) | OSPI_INDIRECT_READ;

    /* trigger the transfer by re-writing address or instruction register */
    if((OSPI_TCFG(ospi_periph) & OSPI_TCFG_ADDRMOD) != OSPI_ADDRESS_NONE){
        OSPI_ADDR(ospi_periph) = addr_reg;
    }else{
        OSPI_INS(ospi_periph) = ins_reg;
    }

    do{
        /* wait till fifo threshold or transfer complete flags are set to read received data */
      while(RESET == (OSPI_STAT(ospi_periph) & (OSPI_FLAG_FT | OSPI_FLAG_TC))){
      }
        
        *(uint8_t *)address = *((__IO uint8_t *)&OSPI_DATA(ospi_periph));
        address++;
        rxcounter--;
        
    }while(rxcounter > 0U);

    /* wait till transfer complete flag is set to go back in idle state */
    while(RESET == (OSPI_STAT(ospi_periph) & OSPI_FLAG_TC)){
    }

    /* clear transfer complete flag */
    OSPI_STATC(ospi_periph) = OSPI_STATC_TCC;

    return SUCCESS;
}

typedef struct
{
    // CR register contains the all-important FMODE bits.
    uint32_t CR;

    // flash chip specific configuration set by the bootloader
    uint32_t CCR;
    uint32_t TCR;
    uint32_t IR;
    uint32_t ABR;
    // address register - no meaning.
    // data length register no meaning.

} octoSpiMemoryMappedModeConfigurationRegisterBackup_t;

octoSpiMemoryMappedModeConfigurationRegisterBackup_t ospiMMMCRBackups[OCTOSPI_INTERFACE_COUNT];

static void octoSpiBackupMemoryMappedModeConfiguration(OCTOSPI_TypeDef *instance)
{
    uint32_t ospi_periph = PERIPH_INT(instance);

    octoSpiDevice_e device = octoSpiDeviceByInstance(instance);
    if (device == OCTOSPIINVALID) {
        return;
    }

    octoSpiMemoryMappedModeConfigurationRegisterBackup_t *ospiMMMCRBackup = &ospiMMMCRBackups[device];

    // backup all the registers used by memory mapped mode that:
    // a) the bootloader configured.
    // b) that other code in this implementation may have modified when memory mapped mode is disabled.

    ospiMMMCRBackup->CR = OSPI_CTL(ospi_periph);
    ospiMMMCRBackup->IR = OSPI_INS(ospi_periph);
    ospiMMMCRBackup->CCR = OSPI_TCFG(ospi_periph);
    ospiMMMCRBackup->TCR = OSPI_TIMCFG(ospi_periph);
    ospiMMMCRBackup->ABR = OSPI_ALTE(ospi_periph);
}

// static MMFLASH_CODE_NOINLINE void octoSpiRestoreMemoryMappedModeConfiguration(OCTOSPI_TypeDef *instance)
// {
//      uint32_t ospi_periph = PERIPH_INT(instance);
//     octoSpiDevice_e device = octoSpiDeviceByInstance(instance);
//     if (device == OCTOSPIINVALID) {
//         return;
//     }

//     octoSpiMemoryMappedModeConfigurationRegisterBackup_t *ospiMMMCRBackup = &ospiMMMCRBackups[device];

//     /* wait till busy flag is reset */
//     while(RESET != (OSPI_STAT(ospi_periph) & OSPI_FLAG_BUSY)) {
//     }

//     OSPI_ALTE(ospi_periph) = ospiMMMCRBackup->ABR;
//     OSPI_TIMCFG(ospi_periph) = ospiMMMCRBackup->TCR;

//     OSPI_DTLEN(ospi_periph) = 0; // "no meaning" in MM mode.

//     OSPI_TCFG(ospi_periph) = ospiMMMCRBackup->CCR;

//     OSPI_INS(ospi_periph) = ospiMMMCRBackup->IR;
//     OSPI_ADDR(ospi_periph) = 0; // "no meaning" in MM mode.

//     octoSpiAbort(instance);

//     /* wait till busy flag is reset */
//     while(RESET != (OSPI_STAT(ospi_periph) & OSPI_FLAG_BUSY)) {
//     }

//    OSPI_CTL(ospi_periph) = ospiMMMCRBackup->CR;
// }

/*
 * Disable memory mapped mode.
 *
 * @See octoSpiEnableMemoryMappedMode
 * @See MMFLASH_CODE_NOINLINE
 *
 * Once this is called any code or data in the memory mapped region cannot be accessed.
 * Thus, this function itself must be in RAM, and the caller's code and data should all be in RAM
 * and this requirement continues until octoSpiEnableMemoryMappedMode is called.
 * This applies to ISR code that runs from the memory mapped region, so likely the caller should
 * also disable IRQs before calling this. 
 */
MMFLASH_CODE_NOINLINE void octoSpiDisableMemoryMappedMode(OCTOSPI_TypeDef *instance)
{
     uint32_t ospi_periph = PERIPH_INT(instance);

    octoSpiAbort(instance);

    if(RESET != (OSPI_STAT(ospi_periph) & OSPI_FLAG_BUSY)) {
        ospi_disable(ospi_periph);
        octoSpiAbort(instance);
    }

    /* wait till busy flag is reset */
    while(RESET != (OSPI_STAT(ospi_periph) & OSPI_FLAG_BUSY)) {
    }

    /* OSPI indirect write mode */
    ospi_functional_mode_config(ospi_periph, OSPI_INDIRECT_WRITE);

    if ((OSPI_CTL(ospi_periph) & OSPI_CTL_FMOD) != OSPI_INDIRECT_WRITE) {
        Error_Handler();
    }

    if ((OSPI_CTL(ospi_periph) & OSPI_CTL_OSPIEN) != OSPI_CTL_OSPIEN) {
        ospi_enable(ospi_periph);
    }
}

/*
 * Enable memory mapped mode.
 *
 * @See octoSpiDisableMemoryMappedMode
 * @See MMFLASH_CODE_NOINLINE
 */

MMFLASH_CODE_NOINLINE void octoSpiEnableMemoryMappedMode(OCTOSPI_TypeDef *instance)
{
    // octoSpiAbort(instance);
    // octoSpiWaitStatusFlags(instance, OCTOSPI_SR_BUSY, RESET);

    //octoSpiRestoreMemoryMappedModeConfiguration(instance);
     uint32_t ospi_periph = PERIPH_INT(instance);
    /* wait till busy flag is reset */
    while(RESET != (OSPI_STAT(ospi_periph) & OSPI_FLAG_BUSY)); 
    /* configure OSPI memory mapped mode */
    ospi_functional_mode_config(ospi_periph, OSPI_MEMORY_MAPPED);
}

static MMFLASH_CODE_NOINLINE void octoSpiTestEnableDisableMemoryMappedMode(octoSpiDevice_t *octoSpi)
{
    OCTOSPI_TypeDef *instance = octoSpi->dev;

    __disable_irq();
    octoSpiDisableMemoryMappedMode(instance);
    octoSpiEnableMemoryMappedMode(instance);
    __enable_irq();
}

MMFLASH_DATA static const uint32_t octoSpi_addressSizeMap[] = {
    OSPI_ADDRESS_8_BITS,
    OSPI_ADDRESS_16_BITS,
    OSPI_ADDRESS_24_BITS,
    OSPI_ADDRESS_32_BITS
};

MMFLASH_CODE static uint32_t octoSpi_addressSizeFromValue(uint8_t addressSize)
{
    return octoSpi_addressSizeMap[((addressSize + 1) / 8) - 1]; // rounds to nearest OSPI_ADDRESS_* value that will hold the address.
}

MMFLASH_CODE_NOINLINE bool octoSpiTransmit1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, const uint8_t *out, int length)
{
     ospi_regular_cmd_struct  cmd_struct = {0}; // Can't initialise to zero as compiler optimization will use memset() which is not in RAM.

    cmd_struct.operation_type     = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.instruction        = instruction;
    cmd_struct.ins_mode           = OSPI_INSTRUCTION_1_LINE;
    cmd_struct.ins_size           = OSPI_INSTRUCTION_8_BITS;

    cmd_struct.addr_dtr_mode      = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.addr_mode          = OSPI_ADDRESS_NONE; 
    cmd_struct.addr_size          = OSPI_ADDRESS_32_BITS;   

    cmd_struct.dummy_cycles       = dummyCycles;
 
    cmd_struct.alter_bytes_mode   = OSPI_ALTERNATE_BYTES_NONE;

    cmd_struct.data_dtr_mode      = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.data_mode          = OSPI_DATA_NONE;
    cmd_struct.nbdata             = length;

    if (out) {
        cmd_struct.data_mode      = OSPI_DATA_1_LINE;
    }

    ErrStatus status = octoSpiCommand(instance, (OSPI_Command_t *)&cmd_struct);

    if (status == SUCCESS && out && length > 0) {
        status = octoSpiTransmit(instance, (uint8_t *)out);
    }
    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiReceive1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{
    ospi_regular_cmd_struct  cmd_struct = {0};

    cmd_struct.operation_type     = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.instruction        = instruction;
    cmd_struct.ins_mode           = OSPI_INSTRUCTION_1_LINE;
    cmd_struct.ins_size           = OSPI_INSTRUCTION_8_BITS;

    cmd_struct.addr_dtr_mode      = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.addr_mode          = OSPI_ADDRESS_NONE; 
    cmd_struct.addr_size          = OSPI_ADDRESS_32_BITS;   

    cmd_struct.dummy_cycles       = dummyCycles;
 
    cmd_struct.alter_bytes_mode   = OSPI_ALTERNATE_BYTES_NONE;

    cmd_struct.data_dtr_mode      = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.data_mode          = OSPI_DATA_1_LINE;
    cmd_struct.nbdata             = length;

    ErrStatus status = octoSpiCommand(instance, (OSPI_Command_t *)&cmd_struct);

    if (status == SUCCESS) {
        status = octoSpiReceive(instance, in);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiReceive4LINES(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint8_t *in, int length)
{
     ospi_regular_cmd_struct  cmd_struct = {0};

    cmd_struct.operation_type     = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.instruction        = instruction;
    cmd_struct.ins_mode           = OSPI_INSTRUCTION_4_LINES;
    cmd_struct.ins_size           = OSPI_INSTRUCTION_8_BITS;

    cmd_struct.addr_dtr_mode      = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.addr_mode          = OSPI_ADDRESS_NONE; 
    cmd_struct.addr_size          = OSPI_ADDRESS_32_BITS; 

    cmd_struct.dummy_cycles       = dummyCycles;
 
    cmd_struct.alter_bytes_mode   = OSPI_ALTERNATE_BYTES_NONE;

    cmd_struct.data_dtr_mode      = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.data_mode          = OSPI_DATA_4_LINES;
    cmd_struct.nbdata             = length;

    ErrStatus status = octoSpiCommand(instance, (OSPI_Command_t *)&cmd_struct);

    if (status == SUCCESS) {
        status = octoSpiReceive(instance, in);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiReceiveWithAddress1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{
     ospi_regular_cmd_struct  cmd_struct = {0};

    cmd_struct.operation_type     = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.instruction        = instruction;
    cmd_struct.ins_mode           = OSPI_INSTRUCTION_1_LINE;
    cmd_struct.ins_size           = OSPI_INSTRUCTION_8_BITS;

    cmd_struct.address            = address;
     cmd_struct.addr_dtr_mode      = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.addr_mode          = OSPI_ADDRESS_1_LINE; 
    cmd_struct.addr_size          = octoSpi_addressSizeFromValue(addressSize); 

    cmd_struct.dummy_cycles       = dummyCycles;
 
    cmd_struct.alter_bytes_mode   = OSPI_ALTERNATE_BYTES_NONE;

    cmd_struct.data_dtr_mode      = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.data_mode          = OSPI_DATA_1_LINE;
    cmd_struct.nbdata             = length;

    ErrStatus status = octoSpiCommand(instance, (OSPI_Command_t *)&cmd_struct);
    if (status == SUCCESS) {
        status = octoSpiReceive(instance, in);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiReceiveWithAddress4LINES(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, uint8_t *in, int length)
{
     ospi_regular_cmd_struct  cmd_struct = {0};

    cmd_struct.operation_type     = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.instruction        = instruction;
    cmd_struct.ins_mode           = OSPI_INSTRUCTION_1_LINE;
    cmd_struct.ins_size           = OSPI_INSTRUCTION_8_BITS;

    cmd_struct.address            = address;
    cmd_struct.addr_dtr_mode     = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.addr_mode          = OSPI_ADDRESS_1_LINE; 
    cmd_struct.addr_size          = octoSpi_addressSizeFromValue(addressSize); 

    cmd_struct.dummy_cycles       = dummyCycles;
 
    cmd_struct.alter_bytes_mode   = OSPI_ALTERNATE_BYTES_NONE;

    cmd_struct.data_dtr_mode      = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.data_mode          = OSPI_DATA_4_LINES;
    cmd_struct.nbdata             = length;

    ErrStatus status = octoSpiCommand(instance, (OSPI_Command_t *)&cmd_struct);
    if (status == SUCCESS) {
        status = octoSpiReceive(instance, in);
    }

    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiTransmitWithAddress1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{
     ospi_regular_cmd_struct  cmd_struct = {0};

    cmd_struct.operation_type     = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.instruction        = instruction;
    cmd_struct.ins_mode           = OSPI_INSTRUCTION_1_LINE;
    cmd_struct.ins_size           = OSPI_INSTRUCTION_8_BITS;

    cmd_struct.address            = address;
    cmd_struct.addr_dtr_mode      = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.addr_mode          = OSPI_ADDRESS_1_LINE; 
    cmd_struct.addr_size          = octoSpi_addressSizeFromValue(addressSize); 

    cmd_struct.dummy_cycles       = dummyCycles;
 
    cmd_struct.alter_bytes_mode   = OSPI_ALTERNATE_BYTES_NONE;

    cmd_struct.data_dtr_mode      = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.data_mode          = OSPI_DATA_1_LINE;
    cmd_struct.nbdata             = length;

    ErrStatus status = octoSpiCommand(instance, (OSPI_Command_t *)&cmd_struct);

    if (status == SUCCESS) {
        status = octoSpiTransmit(instance, (uint8_t *)out);
    }
    return status == SUCCESS;
}

MMFLASH_CODE_NOINLINE bool octoSpiTransmitWithAddress4LINES(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize, const uint8_t *out, int length)
{
     ospi_regular_cmd_struct  cmd_struct = {0};

    cmd_struct.operation_type     = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.instruction        = instruction;
    cmd_struct.ins_mode           = OSPI_INSTRUCTION_1_LINE;
    cmd_struct.ins_size           = OSPI_INSTRUCTION_8_BITS;

    cmd_struct.address            = address;
    cmd_struct.addr_dtr_mode      = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.addr_mode          = OSPI_ADDRESS_1_LINE; 
    cmd_struct.addr_size          = octoSpi_addressSizeFromValue(addressSize); 

    cmd_struct.dummy_cycles       = dummyCycles;
 
    cmd_struct.alter_bytes_mode   = OSPI_ALTERNATE_BYTES_NONE;

    cmd_struct.data_dtr_mode      = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.data_mode          = OSPI_DATA_4_LINES;
    cmd_struct.nbdata             = length;

    ErrStatus status = octoSpiCommand(instance, (OSPI_Command_t *)&cmd_struct);

    if (status == SUCCESS) {
        status = octoSpiTransmit(instance, (uint8_t *)out);
    }
    return status == SUCCESS;    return 0;
}

MMFLASH_CODE_NOINLINE bool octoSpiInstructionWithAddress1LINE(OCTOSPI_TypeDef *instance, uint8_t instruction, uint8_t dummyCycles, uint32_t address, uint8_t addressSize)
{
     ospi_regular_cmd_struct  cmd_struct = {0};

    cmd_struct.operation_type     = OSPI_OPTYPE_COMMON_CFG;
    cmd_struct.instruction        = instruction;
    cmd_struct.ins_mode           = OSPI_INSTRUCTION_1_LINE;
    cmd_struct.ins_size           = OSPI_INSTRUCTION_8_BITS;

    cmd_struct.address            = address;
    cmd_struct.addr_dtr_mode      = OSPI_ADDRDTR_MODE_DISABLE;
    cmd_struct.addr_mode          = OSPI_ADDRESS_1_LINE; 
    cmd_struct.addr_size          = octoSpi_addressSizeFromValue(addressSize); 

    cmd_struct.dummy_cycles       = dummyCycles;
 
    cmd_struct.alter_bytes_mode   = OSPI_ALTERNATE_BYTES_NONE;

    cmd_struct.data_dtr_mode      = OSPI_DADTR_MODE_DISABLE;
    cmd_struct.data_mode          = OSPI_DATA_NONE;
    cmd_struct.nbdata             = 0;

    ErrStatus status = octoSpiCommand(instance, (OSPI_Command_t *)&cmd_struct);

    return status == SUCCESS;
}

void octoSpiInitDevice(octoSpiDevice_e device)
{
    octoSpiDevice_t *octoSpi = &(octoSpiDevice[device]);

#if defined(GD32H7)
    if (isMemoryMappedModeEnabledOnBoot()) {
        // Bootloader has already configured the IO, clocks and peripherals.
        octoSpiBackupMemoryMappedModeConfiguration(octoSpi->dev);

        octoSpiTestEnableDisableMemoryMappedMode(octoSpi);
    } else {
        failureMode(FAILURE_DEVELOPER); // trying to use this implementation when memory mapped mode is not already enabled by a bootloader

        // Here is where we would configure the OCTOSPI1/2 and OCTOSPIM peripherals for the non-memory-mapped use case.
        
    }
#else
#error MCU not supported.
#endif

}

#endif
