/*
 * base.h
 *
 *  Created on: 18 апр. 2019 г.
 *      Author: Temka
 */

#ifndef EXACTOREGLIB_INC_EXACTOBASE_DRIVER_H_
#define EXACTOREGLIB_INC_EXACTOBASE_DRIVER_H_

//base include

//#define EXACTO_HAL
#define EXACTO_SPL
#ifdef EXACTO_HAL
#include "stm32f103xb.h"
#endif

#ifdef EXACTO_SPL
#include "stm32f10x.h"
#endif


//MACROS from HAL

#define GPIO_PIN_MASK_POS   8U

#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

#define RCC_CFGR_PLLXTPRE_Pos                (17U) 
#define RCC_CFGR_PLLMULL_Pos                 (18U) 
#define RCC_CFGR_HPRE_Pos                    (4U) 
#define RCC_CFGR_PPRE1_Pos                   (8U)
#define RCC_CFGR_PPRE2_Pos                   (11U)


typedef struct
{
  uint32_t SYSCLK_Frequency;        /*!< SYSCLK clock frequency */
  uint32_t HCLK_Frequency;          /*!< HCLK clock frequency */
  uint32_t PCLK1_Frequency;         /*!< PCLK1 clock frequency */
  uint32_t PCLK2_Frequency;         /*!< PCLK2 clock frequency */
} exacto_RCC_ClocksTypeDef;


void exacto_I2C_ConfigSpeed(I2C_TypeDef *I2Cx, uint32_t PeriphClock, uint32_t ClockSpeed,
                                        uint32_t DutyCycle);


 uint32_t exacto_RCC_GetAHBPrescaler(void);
 uint32_t exacto_RCC_GetAPB1Prescaler(void);
uint32_t exacto_RCC_GetAPB2Prescaler(void);
uint32_t exacto_RCC_GetHCLKClockFreq(uint32_t SYSCLK_Frequency);
uint32_t exacto_RCC_GetPCLK1ClockFreq(uint32_t HCLK_Frequency);
uint32_t exacto_RCC_GetPCLK2ClockFreq(uint32_t HCLK_Frequency);

 uint32_t exacto_RCC_PLL_GetMultiplicator(void);
 uint32_t exacto_RCC_PLL_GetMainSource(void);
 uint32_t exacto_RCC_PLL_GetPrediv(void);
uint32_t exacto_RCC_PLL_GetFreqDomain_SYS(void);
uint32_t exacto_RCC_GetSysClkSource(void);
uint32_t exacto_sysclk_RCC_GetSystemClockFreq(void);

void exacto_RCC_GetSystemClocksFreq(exacto_RCC_ClocksTypeDef *RCC_Clocks);




#endif /* EXACTOREGLIB_INC_EXACTOBASE_DRIVER_H_ */
