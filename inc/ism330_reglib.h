/*
 * ism330_reglib.h
 *
 *  Created on: 23 апр. 2019 г.
 *      Author: Temka
 */

#ifndef EXACTOREGLIB_INC_ISM330_REGLIB_H_
#define EXACTOREGLIB_INC_ISM330_REGLIB_H_

#include "exactoBase_driver.h"
#include "ism330dlc_reg.h"

void ConfigureSPI3_ism330(void);
void ActivateSPI3_ism330(void);
uint8_t Set3wireAndGetWhoami_ism330(void);
uint8_t GetTGXLData_ism330(uint8_t * data);
uint8_t GetGXLData_ism330(uint8_t * data);
uint8_t GetXLData_ism330(uint8_t * data);
uint8_t GetGData_ism330(uint8_t * data);

uint8_t Get_T_G_XL_uint8_ism330(uint8_t * data);

__STATIC_INLINE void set_halfduplexTX_ism330(void)
{
	MODIFY_REG(SPI2->CR1, (SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE), (SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE));
}
__STATIC_INLINE void set_halfduplexRX_ism330(void)
{
	MODIFY_REG(SPI2->CR1, (SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE), (SPI_CR1_BIDIMODE));
}

__STATIC_INLINE void disable_transmit_ism330(void)
{
	GPIOB->BSRR   =  GPIO_BSRR_BS12;
}
__STATIC_INLINE void enable_transmit_ism330(void)
{
	GPIOB->BSRR   =  GPIO_BSRR_BR12;
}
__STATIC_INLINE void check_TXE_ism330(void)
{
	while(! (READ_BIT(SPI2->SR, SPI_SR_TXE) == (SPI_SR_TXE))	);
}
__STATIC_INLINE void check_BSY_ism330(void)
{
	while(	(READ_BIT(SPI2->SR, SPI_SR_BSY) == (SPI_SR_BSY))	);
}
__STATIC_INLINE void check_RXNE_ism330(void)
{
	while(!	(READ_BIT(SPI2->SR, SPI_SR_RXNE) == (SPI_SR_RXNE))	);
}
__STATIC_INLINE void write_ism330(uint8_t address, uint8_t value)
{
	uint8_t mask = 0x7F ;//01111111b
	mask &= address; // delete first bit, set write bit to 0
	//LL_GPIO_ResetOutputPin(GPIOx,PinMask);
	enable_transmit_ism330();
	//while(!LL_SPI_IsActiveFlag_TXE(SPIx));
	check_TXE_ism330();
	//LL_SPI_TransmitData8(SPIx, mask);
	SPI2->DR = mask;
	//while(!LL_SPI_IsActiveFlag_TXE(SPIx));
	check_TXE_ism330();
	//LL_SPI_TransmitData8(SPIx, value);
	SPI2->DR = value;
	//while(!LL_SPI_IsActiveFlag_TXE(SPIx));
	check_TXE_ism330();
	//while(LL_SPI_IsActiveFlag_BSY(SPIx));
	check_BSY_ism330();
	//LL_GPIO_SetOutputPin(GPIOx,PinMask);
	disable_transmit_ism330();
}
__STATIC_INLINE uint8_t read_ism330(uint8_t address)
{
	uint8_t value = address | 0x80;
	//LL_GPIO_ResetOutputPin(GPIOx,PinMask);
	enable_transmit_ism330();
	//while(!LL_SPI_IsActiveFlag_TXE(SPIx));
	check_TXE_ism330();
	//LL_SPI_TransmitData8(SPIx, value);
	SPI2->DR = value;
	//while(!LL_SPI_IsActiveFlag_TXE(SPIx));
	check_TXE_ism330();
	//while(LL_SPI_IsActiveFlag_BSY(SPIx));
	check_BSY_ism330();
	//LL_SPI_SetTransferDirection(SPIx,LL_SPI_HALF_DUPLEX_RX);
	set_halfduplexRX_ism330();
	//while(!LL_SPI_IsActiveFlag_RXNE(SPIx));
	check_RXNE_ism330();
	//uint8_t result = 0;
	//result = LL_SPI_ReceiveData8(SPIx);
	uint8_t result = (uint8_t)SPI2->DR;
	//LL_SPI_SetTransferDirection(SPIx,LL_SPI_HALF_DUPLEX_TX);
	set_halfduplexTX_ism330();
	//LL_GPIO_SetOutputPin(GPIOx,PinMask);
	disable_transmit_ism330();
	return result;
}
__STATIC_INLINE void multiread_ism330( const uint8_t RegAdr, uint8_t * data, const uint32_t datalen )
{
	uint8_t value = RegAdr | 0x80;
	enable_transmit_ism330();
	check_TXE_ism330();
	SPI2->DR = value;
	check_TXE_ism330();
	check_BSY_ism330();
	set_halfduplexRX_ism330();
	for (uint32_t i = 0; i < datalen; i++)
	{
		check_RXNE_ism330();
		data[i] = (uint8_t)(READ_REG(SPI2->DR));
	}
	set_halfduplexTX_ism330();
	disable_transmit_ism330();
}

#endif /* EXACTOREGLIB_INC_ISM330_REGLIB_H_ */
