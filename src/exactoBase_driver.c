/*
 * exactoBase_driver.c
 *
 *  Created on: 22 апр. 2019 г.
 *      Author: Temka
 */
#include "exactoBase_driver.h"

const uint8_t exacto_AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t exacto_APBPrescTable[8U] =  {0, 0, 0, 0, 1, 2, 3, 4};

#define __EXACTO_RCC_CALC_PLLCLK_FREQ(__INPUTFREQ__, __PLLMUL__) ((__INPUTFREQ__) * (((__PLLMUL__) >> RCC_CFGR_PLLMULL_Pos) + 2U))
#define __EXACTO_RCC_CALC_HCLK_FREQ(__SYSCLKFREQ__, __AHBPRESCALER__) ((__SYSCLKFREQ__) >> exacto_AHBPrescTable[((__AHBPRESCALER__) & RCC_CFGR_HPRE) >>  RCC_CFGR_HPRE_Pos])
#define __EXACTO_RCC_CALC_PCLK1_FREQ(__HCLKFREQ__, __APB1PRESCALER__) ((__HCLKFREQ__) >> exacto_APBPrescTable[(__APB1PRESCALER__) >>  RCC_CFGR_PPRE1_Pos])
#define __EXACTO_RCC_CALC_PCLK2_FREQ(__HCLKFREQ__, __APB2PRESCALER__) ((__HCLKFREQ__) >> exacto_APBPrescTable[(__APB2PRESCALER__) >>  RCC_CFGR_PPRE2_Pos])



#define __EXACTO_I2C_RISE_TIME(__FREQRANGE__, __SPEED__)                    (uint32_t)(((__SPEED__) <= 100000U) ? ((__FREQRANGE__) + 1U) : ((((__FREQRANGE__) * 300U) / 1000U) + 1U))
#define __EXACTO_I2C_SPEED_FAST_TO_CCR(__PCLK__, __SPEED__, __DUTYCYCLE__)  (uint32_t)(((__DUTYCYCLE__) == 0x00000000U)? \
                                                                            (((((__PCLK__) / ((__SPEED__) * 3U)) & I2C_CCR_CCR) == 0U)? 1U:((__PCLK__) / ((__SPEED__) * 3U))) : \
                                                                            (((((__PCLK__) / ((__SPEED__) * 25U)) & I2C_CCR_CCR) == 0U)? 1U:((__PCLK__) / ((__SPEED__) * 25U))))
#define __EXACTO_I2C_SPEED_STANDARD_TO_CCR(__PCLK__, __SPEED__)             (uint32_t)(((((__PCLK__)/((__SPEED__) << 1U)) & I2C_CCR_CCR) < 4U)? 4U:((__PCLK__) / ((__SPEED__) << 1U)))
void exacto_I2C_ConfigSpeed(I2C_TypeDef *I2Cx, uint32_t PeriphClock, uint32_t ClockSpeed,
                                        uint32_t DutyCycle)
{
  register uint32_t freqrange = 0x0U;
  register uint32_t clockconfig = 0x0U;

  /* Compute frequency range */
  freqrange = (uint32_t)((PeriphClock)/1000000U);

  /* Configure I2Cx: Frequency range register */
  MODIFY_REG(I2Cx->CR2, I2C_CR2_FREQ, freqrange);

  /* Configure I2Cx: Rise Time register */
  MODIFY_REG(I2Cx->TRISE, I2C_TRISE_TRISE, __EXACTO_I2C_RISE_TIME(freqrange, ClockSpeed));

  /* Configure Speed mode, Duty Cycle and Clock control register value */
  if (ClockSpeed > 100000U)
  {
    /* Set Speed mode at fast and duty cycle for Clock Speed request in fast clock range */
    clockconfig = I2C_CCR_FS                                          | \
    		__EXACTO_I2C_SPEED_FAST_TO_CCR(PeriphClock, ClockSpeed, DutyCycle)        | \
                  DutyCycle;
  }
  else
  {
    /* Set Speed mode at standard for Clock Speed request in standard clock range */
    clockconfig = 0x00000000U                                      | \
    		__EXACTO_I2C_SPEED_STANDARD_TO_CCR(PeriphClock, ClockSpeed);
  }

  /* Configure I2Cx: Clock control register */
  MODIFY_REG(I2Cx->CCR, (I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR), clockconfig);
}

 uint32_t exacto_RCC_PLL_GetMultiplicator(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PLLMULL));
}
 uint32_t exacto_RCC_PLL_GetMainSource(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PLLSRC));
}
 uint32_t exacto_RCC_PLL_GetPrediv(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PLLXTPRE) >> RCC_CFGR_PLLXTPRE_Pos);
}
uint32_t exacto_RCC_PLL_GetFreqDomain_SYS(void)
{
  uint32_t pllinputfreq = 0U, pllsource = 0U;

  /* PLL_VCO = (HSE_VALUE, HSI_VALUE or PLL2 / PLL Predivider) * PLL Multiplicator */

  /* Get PLL source */
  pllsource = exacto_RCC_PLL_GetMainSource();

  switch (pllsource)
  {
    case 0x00000000U: /* HSI used as PLL clock source */
      pllinputfreq = HSI_VALUE / 2U;
      break;

    case RCC_CFGR_PLLSRC:       /* HSE used as PLL clock source */
      pllinputfreq = HSE_VALUE / (exacto_RCC_PLL_GetPrediv() + 1U);
      break;

    default:
      pllinputfreq = HSI_VALUE / 2U;
      break;
  }
  return __EXACTO_RCC_CALC_PLLCLK_FREQ(pllinputfreq, exacto_RCC_PLL_GetMultiplicator());
}
 uint32_t exacto_RCC_GetSysClkSource(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_SWS));
}
uint32_t exacto_sysclk_RCC_GetSystemClockFreq(void)
{
  uint32_t frequency = 0U;

//  uint32_t frequency = 0U;
//
//  /* Get SYSCLK source -------------------------------------------------------*/
//  switch (LL_RCC_GetSysClkSource())
//  {
//    case LL_RCC_SYS_CLKSOURCE_STATUS_HSI:  /* HSI used as system clock  source */
//      frequency = HSI_VALUE;
//      break;
//
//    case LL_RCC_SYS_CLKSOURCE_STATUS_HSE:  /* HSE used as system clock  source */
//      frequency = HSE_VALUE;
//      break;
//
//    case LL_RCC_SYS_CLKSOURCE_STATUS_PLL:  /* PLL used as system clock  source */
//      frequency = RCC_PLL_GetFreqDomain_SYS();
//      break;
//
//    default:
//      frequency = HSI_VALUE;
//      break;
//  }

  /* Get SYSCLK source -------------------------------------------------------*/
  switch (exacto_RCC_GetSysClkSource())
  {
    case RCC_CFGR_SWS_HSI:  /* HSI used as system clock  source */
      frequency = HSI_VALUE;
      break;

    case RCC_CFGR_SWS_HSE:  /* HSE used as system clock  source */
      frequency = HSE_VALUE;
      break;

    case RCC_CFGR_SWS_PLL:  /* PLL used as system clock  source */
      frequency = exacto_RCC_PLL_GetFreqDomain_SYS();
      break;

    default:
      frequency = HSI_VALUE;
      break;
  }
  return frequency;
}

void exacto_RCC_GetSystemClocksFreq(exacto_RCC_ClocksTypeDef *RCC_Clocks)
{
  /* Get SYSCLK frequency */
  RCC_Clocks->SYSCLK_Frequency = exacto_sysclk_RCC_GetSystemClockFreq();

  /* HCLK clock frequency */
  RCC_Clocks->HCLK_Frequency   = exacto_RCC_GetHCLKClockFreq(RCC_Clocks->SYSCLK_Frequency);

  /* PCLK1 clock frequency */
  RCC_Clocks->PCLK1_Frequency  = exacto_RCC_GetPCLK1ClockFreq(RCC_Clocks->HCLK_Frequency);

  /* PCLK2 clock frequency */
  RCC_Clocks->PCLK2_Frequency  = exacto_RCC_GetPCLK2ClockFreq(RCC_Clocks->HCLK_Frequency);
}

#define __EXACTO_RCC_CALC_HCLK_FREQ(__SYSCLKFREQ__, __AHBPRESCALER__) ((__SYSCLKFREQ__) >> exacto_AHBPrescTable[((__AHBPRESCALER__) & RCC_CFGR_HPRE) >>  RCC_CFGR_HPRE_Pos])
#define __EXACTO_RCC_CALC_PCLK1_FREQ(__HCLKFREQ__, __APB1PRESCALER__) ((__HCLKFREQ__) >> exacto_APBPrescTable[(__APB1PRESCALER__) >>  RCC_CFGR_PPRE1_Pos])
 uint32_t exacto_RCC_GetAHBPrescaler(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_HPRE));
}
 uint32_t exacto_RCC_GetAPB1Prescaler(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PPRE1));
}
uint32_t exacto_RCC_GetAPB2Prescaler(void)
{
  return (uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PPRE2));
}
uint32_t exacto_RCC_GetHCLKClockFreq(uint32_t SYSCLK_Frequency)
{
  /* HCLK clock frequency */
  return __EXACTO_RCC_CALC_HCLK_FREQ(SYSCLK_Frequency, exacto_RCC_GetAHBPrescaler());
}
uint32_t exacto_RCC_GetPCLK1ClockFreq(uint32_t HCLK_Frequency)
{
  /* PCLK1 clock frequency */
  return __EXACTO_RCC_CALC_PCLK1_FREQ(HCLK_Frequency, exacto_RCC_GetAPB1Prescaler());
}
uint32_t exacto_RCC_GetPCLK2ClockFreq(uint32_t HCLK_Frequency)
{
  /* PCLK2 clock frequency */
  return __EXACTO_RCC_CALC_PCLK2_FREQ(HCLK_Frequency, exacto_RCC_GetAPB2Prescaler());
}
