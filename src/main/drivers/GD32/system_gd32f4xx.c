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

#include "platform.h"

#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/exti.h"
#include "drivers/nvic.h"
#include "drivers/system.h"
#include "drivers/persistent.h"

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)
void sys_clock_config(void);


typedef struct pllConfig_s {
  uint16_t mhz; // target SYSCLK
  uint16_t n;
  uint16_t p;
  uint16_t q;
} pllConfig_t;


uint32_t pll_src, pll_input, pll_m, pll_p, pll_n, pll_q;

// PLL parameters for PLL input = 1MHz.
// For PLL input = 2MHz, divide n by 2; see SystemInitPLLParameters below.

static const pllConfig_t overclockLevels[] = {
  { 168, 336, 2,  7 },  // 168 MHz
  { 200, 384, 2,  8 },  // 200 MHz
  { 240, 480, 2,  10 }  // 240 MHz
};


static void SystemInitPLLParameters(void)
{
    /* PLL setting for overclocking */

    uint32_t currentOverclockLevel = persistentObjectRead(PERSISTENT_OBJECT_OVERCLOCK_LEVEL);

    if (currentOverclockLevel >= ARRAYLEN(overclockLevels)) {
      return;
    }

    const pllConfig_t * const pll = overclockLevels + currentOverclockLevel;

    pll_n = pll->n / pll_input;
    pll_p = pll->p;
    pll_q = pll->q;
}


void OverclockRebootIfNecessary(uint32_t overclockLevel)
{
  if (overclockLevel >= ARRAYLEN(overclockLevels)) {
    return;
  }

  const pllConfig_t * const pll = overclockLevels + overclockLevel;

  // Reboot to adjust overclock frequency
  if (SystemCoreClock != pll->mhz * 1000000U) {
    persistentObjectWrite(PERSISTENT_OBJECT_OVERCLOCK_LEVEL, overclockLevel);
    __disable_irq();
    NVIC_SystemReset();
  }
}

void systemClockSetHSEValue(uint32_t frequency)
{
    uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);

    if (hse_value != frequency) {
        persistentObjectWrite(PERSISTENT_OBJECT_HSE_VALUE, frequency);
        __disable_irq();
        NVIC_SystemReset();
    }
}


void systemReset(void)
{
    __disable_irq();
    NVIC_SystemReset();
}

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    switch (requestType) {
    case BOOTLOADER_REQUEST_ROM:
    default:
        persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_BOOTLOADER_REQUEST_ROM);

        break;
    }

    __disable_irq();
    NVIC_SystemReset();
}

typedef void resetHandler_t(void);

typedef struct isrVector_s {
    __I uint32_t    stackEnd;
    resetHandler_t *resetHandler;
} isrVector_t;

// Used in the startup files for F4
void checkForBootLoaderRequest(void)
{
    uint32_t bootloaderRequest = persistentObjectRead(PERSISTENT_OBJECT_RESET_REASON);

    if (bootloaderRequest != RESET_BOOTLOADER_REQUEST_ROM) {
        return;
    }
    persistentObjectWrite(PERSISTENT_OBJECT_RESET_REASON, RESET_NONE);


    rcu_periph_clock_enable(RCU_SYSCFG);
    syscfg_bootmode_config(SYSCFG_BOOTMODE_BOOTLOADER);

    extern isrVector_t system_isr_vector_table_base;
    
    SCB->VTOR = (uint32_t)&system_isr_vector_table_base;
    __DSB();
    __DSB();


    __set_MSP(system_isr_vector_table_base.stackEnd);
    system_isr_vector_table_base.resetHandler();
    while (1);
}

void enableGPIOPowerUsageAndNoiseReductions(void)
{
    /* enable AHB1 peripherals clock */
    rcu_periph_clock_enable(RCU_BKPSRAM);
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_DMA1);

    /* enable APB1 peripherals clock */
    rcu_periph_clock_enable(RCU_TIMER1);
    rcu_periph_clock_enable(RCU_TIMER2);
    rcu_periph_clock_enable(RCU_TIMER3);
    rcu_periph_clock_enable(RCU_TIMER4);
    rcu_periph_clock_enable(RCU_TIMER5);
    rcu_periph_clock_enable(RCU_TIMER6);
    rcu_periph_clock_enable(RCU_TIMER11);
    rcu_periph_clock_enable(RCU_TIMER12);
    rcu_periph_clock_enable(RCU_TIMER13);
    rcu_periph_clock_enable(RCU_WWDGT);
    rcu_periph_clock_enable(RCU_SPI1);
    rcu_periph_clock_enable(RCU_SPI2);
    rcu_periph_clock_enable(RCU_USART1);
    rcu_periph_clock_enable(RCU_USART1);
    rcu_periph_clock_enable(RCU_UART3);
    rcu_periph_clock_enable(RCU_UART4);
    rcu_periph_clock_enable(RCU_I2C0);
    rcu_periph_clock_enable(RCU_I2C1);
    rcu_periph_clock_enable(RCU_I2C2);
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_CAN1);
    rcu_periph_clock_enable(RCU_PMU);
    rcu_periph_clock_enable(RCU_DAC);

    rcu_periph_clock_enable(RCU_TIMER0);
    rcu_periph_clock_enable(RCU_TIMER7);
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_USART5);
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_periph_clock_enable(RCU_ADC1);
    rcu_periph_clock_enable(RCU_ADC2);
    rcu_periph_clock_enable(RCU_SDIO);
    rcu_periph_clock_enable(RCU_SPI0);
    rcu_periph_clock_enable(RCU_SYSCFG);
    rcu_periph_clock_enable(RCU_TIMER8);
    rcu_periph_clock_enable(RCU_TIMER9);
    rcu_periph_clock_enable(RCU_TIMER10);

}


void sys_clock_config(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;
    __IO uint32_t reg_temp;

    uint32_t hse_value = persistentObjectRead(PERSISTENT_OBJECT_HSE_VALUE);
    uint32_t hse_mhz = hse_value / 1000000;

    if (hse_value == 0) {
        /* enable IRC16M */
        RCU_CTL |= RCU_CTL_IRC16MEN;
        /* wait until IRC16M is stable or the startup time is longer than IRC16M_STARTUP_TIMEOUT */
        do{
            timeout++;
            stab_flag = (RCU_CTL & RCU_CTL_IRC16MSTB);
        }while((0U == stab_flag) && (IRC16M_STARTUP_TIMEOUT != timeout));

        if(timeout >= IRC16M_STARTUP_TIMEOUT) {
            return;
        }

        pll_src = RCU_PLLSRC_IRC16M;

        // HSI is fixed at 16MHz.
        pll_m = 8;
        pll_input = 2;
    } else {
        /* enable HXTAL */
        RCU_CTL |= RCU_CTL_HXTALEN;
        /* wait until HXTAL is stable or the startup time is longer than HXTAL_STARTUP_TIMEOUT */
        do{
            timeout++;
            stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
        }while((0U == stab_flag) && (HXTAL_STARTUP_TIMEOUT != timeout));

        if(timeout >= HXTAL_STARTUP_TIMEOUT) {
            return;
        }

        pll_src = RCU_PLLSRC_HXTAL;

        pll_m = hse_mhz / 2;
        if (pll_m * 2 != hse_mhz) {
            pll_m = hse_mhz;
        }
        pll_input = hse_mhz / pll_m;
    }

    SystemInitPLLParameters();

    RCU_APB1EN |= RCU_APB1EN_PMUEN;
    PMU_CTL |= PMU_CTL_LDOVS;

    /* HXTAL is stable */
    /* AHB = SYSCLK */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    /* APB2 = AHB/2 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV2;
    /* APB1 = AHB/4 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV4;

    /* Configure the main PLL, PSC = 25, PLL_N = 400, PLL_P = 2, PLL_Q = 9 */ 
    RCU_PLL = (pll_m | (pll_n << 6U) | (((pll_p >> 1U) - 1U) << 16U) |
                   (pll_src) | (pll_q << 24U));

    /* enable PLL */
    RCU_CTL |= RCU_CTL_PLLEN;

    /* wait until PLL is stable */
    while(0U == (RCU_CTL & RCU_CTL_PLLSTB)){
    }
    
    /* Enable the high-drive to extend the clock frequency to 200 Mhz */
    PMU_CTL |= PMU_CTL_HDEN;
    while(0U == (PMU_CS & PMU_CS_HDRF)){
    }
    
    /* select the high-drive mode */
    PMU_CTL |= PMU_CTL_HDS;
    while(0U == (PMU_CS & PMU_CS_HDSRF)){
    } 

    reg_temp = RCU_CFG0;
    /* select PLL as system clock */
    reg_temp &= ~RCU_CFG0_SCS;
    reg_temp |= RCU_CKSYSSRC_PLLP;
    RCU_CFG0 = reg_temp;

    /* wait until PLL is selected as system clock */
    while(0U == (RCU_CFG0 & RCU_SCSS_PLLP)){
    }

    SystemCoreClockUpdate();
}


bool isMPUSoftReset(void)
{
    if (cachedRccCsrValue & RCU_RSTSCK_SWRSTF)
        return true;
    else
        return false;
}

void systemInit(void)
{
    persistentObjectInit();

#if 1
    checkForBootLoaderRequest();
#endif
    // sys_clock_config();

    // Configure NVIC preempt/priority groups
    nvic_priority_group_set(NVIC_PRIORITY_GROUPING);

    // cache RCU RSTSCK register value to use it in isMPUSoftReset() and others
    cachedRccCsrValue = RCU_RSTSCK;

    // Although VTOR is already loaded with a possible vector table in RAM,
    // removing the call to NVIC_SetVectorTable causes USB not to become active,

#ifdef VECT_TAB_SRAM
    extern uint8_t isr_vector_table_base;
    nvic_vector_table_set((uint32_t)&isr_vector_table_base, 0x0);
#endif

    rcu_periph_clock_disable(RCU_USBFS);

    rcu_all_reset_flag_clear();

    enableGPIOPowerUsageAndNoiseReductions();

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);
}
