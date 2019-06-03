/*
 * bmp280_reglib.h
 *
 *  Created on: 22 апр. 2019 г.
 *      Author: Temka
 */

#ifndef EXACTOREGLIB_INC_BMP280_REGLIB_H_
#define EXACTOREGLIB_INC_BMP280_REGLIB_H_

#include "exactoBase_driver.h"
#include "bmp280_defs.h"

void ClockEnPinsI2C_bmp280(void);
void ConfigurePinsI2C_bmp280(void);
void ConfigureI2C_bmp280(void);
void ActivateI2C_bmp280(void);
uint8_t GetWhoami_bmp280(void);
//uint8_t GetPresTempValuesUint8_bmp280(uint8_t * data);

uint8_t  GetPresTempValues_bmp280(int32_t * pres, int32_t * temp);

uint8_t read_bmp280(uint8_t address);
void multiread_bmp280(uint8_t address, uint8_t * values, uint8_t cnt);
void write_bmp280(uint8_t address, uint8_t value);

void EnReset_bmp280(void);
void DsReset_bmp280(void);

__STATIC_INLINE uint8_t check_I2C_SB_bmp280(void)
{
    uint16_t i = 0;
	while(!	(READ_BIT(I2C2->SR1, I2C_SR1_SB) == (I2C_SR1_SB))	)
    {
        if(i > 1000)
            return 0;
        else
            i++;
    }
    return 1;
}
uint8_t check_I2C_ADDR_bmp280(void);
uint8_t check_I2C_RXNE_bmp280(void);
void clearFlag_I2C_ADDR_bmp280(void);
uint8_t check_I2C_TXE_bmp280(void);
uint8_t readI2C_bmp280(uint8_t address, uint8_t reg);
void multireadI2C_bmp280(uint8_t address, uint8_t reg, uint8_t * values, uint8_t cnt);
void writeI2C_bmp280(uint8_t address, uint8_t reg,uint8_t value);

__STATIC_INLINE uint8_t getval_bmp280( uint8_t reg)
{
	/* ST ; SAD + W */
	//LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
	//MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	//LL_I2C_GenerateStartCondition(I2Cx);
	SET_BIT(I2C2->CR1, I2C_CR1_START);
	if(!check_I2C_SB_bmp280())  return 0;
	uint8_t trg = (0x77<<1);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, (trg  | 0x00));
	if(!check_I2C_ADDR_bmp280()) return 0;
	clearFlag_I2C_ADDR_bmp280();
	if(!check_I2C_TXE_bmp280()) return 0;
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, reg);
	if(!check_I2C_TXE_bmp280()) return 0;
	SET_BIT(I2C2->CR1, I2C_CR1_START);
	if(!check_I2C_SB_bmp280()) return 0;
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, (trg  | 0x01));
	if(!check_I2C_ADDR_bmp280()) return 0;
	clearFlag_I2C_ADDR_bmp280();
	if(!check_I2C_RXNE_bmp280()) return 0;
	uint8_t res = (uint8_t)(READ_BIT(I2C2->DR, I2C_DR_DR));
	
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, 0x00000000U);
	if(!check_I2C_RXNE_bmp280()) return 0;
	SET_BIT(I2C2->CR1, I2C_CR1_STOP);
	return res;
}
__STATIC_INLINE uint8_t setval_bmp280( uint8_t reg,uint8_t value)
{
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	SET_BIT(I2C2->CR1, I2C_CR1_START);
	if(!check_I2C_SB_bmp280()) return 0;
	uint8_t trg = (0x77<<1);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, (trg  | 0x00));
	if(!check_I2C_ADDR_bmp280()) return 0;
    
	clearFlag_I2C_ADDR_bmp280();
	if(!check_I2C_TXE_bmp280()) return 0;
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, reg);
	if(!check_I2C_TXE_bmp280()) return 0;
  MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, value);
  if(!check_I2C_TXE_bmp280()) return 0;
    
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, 0x00000000U);
	SET_BIT(I2C2->CR1, I2C_CR1_STOP);
	return 1;
}
__STATIC_INLINE uint8_t GetPresTempValuesUint8_bmp280(uint8_t * data)
{
	if(!getval_bmp280(BMP280_STATUS_ADDR))
		return 0; 
	multiread_bmp280(BMP280_PRES_MSB_ADDR, data, 6);
	return 1;
}

__STATIC_INLINE uint8_t getMultiVal_bmp280( uint8_t reg, uint8_t * values, uint8_t cnt)
{
	SET_BIT(I2C2->CR1, I2C_CR1_START);
	if(!check_I2C_SB_bmp280())  return 0;
	uint8_t trg = (0x77<<1);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, (trg  | 0x00));
	if(!check_I2C_ADDR_bmp280()) return 0;
	clearFlag_I2C_ADDR_bmp280();
	if(!check_I2C_TXE_bmp280()) return 0;
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, reg);
	if(!check_I2C_TXE_bmp280()) return 0;
	SET_BIT(I2C2->CR1, I2C_CR1_START);
	if(!check_I2C_SB_bmp280()) return 0;
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, (trg  | 0x01));
	if(!check_I2C_ADDR_bmp280()) return 0;
	clearFlag_I2C_ADDR_bmp280();
	if(!check_I2C_RXNE_bmp280()) return 0;
	values[0] = (uint8_t)(READ_BIT(I2C2->DR, I2C_DR_DR));
	for(uint8_t i = 1; i < (cnt); i++)
	{
		MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
		if(!check_I2C_RXNE_bmp280()) return 0;
		values[i] = (uint8_t)(READ_BIT(I2C2->DR, I2C_DR_DR));
	}
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, 0x00000000U);
	if(!check_I2C_RXNE_bmp280()) return 0;
	SET_BIT(I2C2->CR1, I2C_CR1_STOP);
	return 1;
}

#endif /* EXACTOREGLIB_INC_BMP280_REGLIB_H_ */
