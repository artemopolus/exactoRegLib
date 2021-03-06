/*
 * lsm303ah_reglib.h
 *
 *  Created on: 19 ���. 2019 �.
 *      Author: Temka
 */

#ifndef EXACTOREGLIB_INC_LSM303AH_REGLIB_H_
#define EXACTOREGLIB_INC_LSM303AH_REGLIB_H_

#include "exactoBase_driver.h"

#include "lsm303ah_reg.h"

#define LSM303_SPI	SPI1
#define LSM303_TMOUT	UINT16_C(100)
//#define LSM303AH_STATUS_A	 		0x27
//#define LSM303AH_DATA_XL_X_L	0x28
#define LSM303AH_SELFTEST_ST_MIN	70
#define LSM303AH_SELFTEST_ST_MAX	1500

void ClockEnPins_lsm303ah(void);
void ConfigurePins_lsm303ah(void);
void ConfigureSPI3_lsm303ah(void);
void ActivateSPI3_lsm303ah(void);
uint8_t Set3wireAndGetWhoami_lsm303ah(void);

uint8_t GetReadyFlag_lsm303ah(void);
uint8_t GetXLallData_lsm303ah(int16_t * data);
uint8_t GetXLallDataUint8_lsm303ah(uint8_t * data);
uint8_t Get_XL_M_uint8_lsm303ah(uint8_t * XLdata, uint8_t * Mdata);
int16_t GetValue_lsm303ah(const uint8_t lsb, const uint8_t msb);


//    LL_SPI_SetTransferDirection(LSM303AH_SPI,LL_SPI_HALF_DUPLEX_TX);
//    MODIFY_REG(SPIx->CR1, SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE, TransferDirection);
//    LL_SPI_HALF_DUPLEX_TX              (SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE)
__STATIC_INLINE void set_halfduplexTX_lsm303ah(void)
{
	MODIFY_REG(SPI1->CR1, (SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE), (SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE));
}
__STATIC_INLINE void set_halfduplexRX_lsm303ah(void)
{
	MODIFY_REG(SPI1->CR1, (SPI_CR1_RXONLY | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE), (SPI_CR1_BIDIMODE));
}

__STATIC_INLINE void disable_transmit_lsm303ah(void)
{
	GPIOA->BSRR   =  GPIO_BSRR_BS4;
}
__STATIC_INLINE void enable_transmit_lsm303ah(void)
{
	GPIOA->BSRR   =  GPIO_BSRR_BR4;
}
__STATIC_INLINE void check_TXE_lsm303ah(void)
{
	while(! (READ_BIT(SPI1->SR, SPI_SR_TXE) == (SPI_SR_TXE))	);
}
__STATIC_INLINE uint8_t check_TXE_lsm303ah_fst(void)
{
	uint16_t i = 0;
	while(! (READ_BIT(LSM303_SPI->SR, SPI_SR_TXE) == (SPI_SR_TXE))	)
	{
		if(i < LSM303_TMOUT)
			i++;
		else 
			return 0;
	}
	return 1;
}
__STATIC_INLINE void check_BSY_lsm303ah(void)
{
	while(	(READ_BIT(SPI1->SR, SPI_SR_BSY) == (SPI_SR_BSY))	);
}
__STATIC_INLINE uint8_t check_BSY_lsm303ah_fst(void)
{
	uint16_t i = 0;
	while(	(READ_BIT(LSM303_SPI->SR, SPI_SR_BSY) == (SPI_SR_BSY))	)
	{
		if(i < LSM303_TMOUT)
			i++;
		else 
			return 0;
	}
	return 1;
}
__STATIC_INLINE void check_RXNE_lsm303ah(void)
{
	while(!	(READ_BIT(SPI1->SR, SPI_SR_RXNE) == (SPI_SR_RXNE))	);
}
__STATIC_INLINE uint8_t check_RXNE_lsm303ah_fst(void)
{
	uint16_t i = 0;
	while(!	(READ_BIT(LSM303_SPI->SR, SPI_SR_RXNE) == (SPI_SR_RXNE))	)
	{
		if(i < LSM303_TMOUT)
			i++;
		else 
			return 0;
	}
	return 1;
}
__STATIC_INLINE void write_lsm303ah(uint8_t address, uint8_t value)
{
	uint8_t mask = 0x7F ;//01111111b
	mask &= address; // delete first bit, set write bit to 0
	//LL_GPIO_ResetOutputPin(GPIOx,PinMask);
	enable_transmit_lsm303ah();
	//while(!LL_SPI_IsActiveFlag_TXE(SPIx));
	check_TXE_lsm303ah();
	//LL_SPI_TransmitData8(SPIx, mask);
	SPI1->DR = mask;
	//while(!LL_SPI_IsActiveFlag_TXE(SPIx));
	check_TXE_lsm303ah();
	//LL_SPI_TransmitData8(SPIx, value);
	SPI1->DR = value;
	//while(!LL_SPI_IsActiveFlag_TXE(SPIx));
	check_TXE_lsm303ah();
	//while(LL_SPI_IsActiveFlag_BSY(SPIx));
	check_BSY_lsm303ah();
	//LL_GPIO_SetOutputPin(GPIOx,PinMask);
	disable_transmit_lsm303ah();
}
__STATIC_INLINE uint8_t read_lsm303ah(uint8_t address)
{
	uint8_t value = address | 0x80;
	//LL_GPIO_ResetOutputPin(GPIOx,PinMask);
	enable_transmit_lsm303ah();
	//while(!LL_SPI_IsActiveFlag_TXE(SPIx));
	check_TXE_lsm303ah();
	//LL_SPI_TransmitData8(SPIx, value);
	SPI1->DR = value;
	//while(!LL_SPI_IsActiveFlag_TXE(SPIx));
	check_TXE_lsm303ah();
	//while(LL_SPI_IsActiveFlag_BSY(SPIx));
	check_BSY_lsm303ah();
	//LL_SPI_SetTransferDirection(SPIx,LL_SPI_HALF_DUPLEX_RX);
	set_halfduplexRX_lsm303ah();
	//while(!LL_SPI_IsActiveFlag_RXNE(SPIx));
	check_RXNE_lsm303ah();
	uint8_t result = 0;
	//result = LL_SPI_ReceiveData8(SPIx);
	result = (uint8_t)(READ_REG(SPI1->DR));
	//LL_SPI_SetTransferDirection(SPIx,LL_SPI_HALF_DUPLEX_TX);
	set_halfduplexTX_lsm303ah();
	//LL_GPIO_SetOutputPin(GPIOx,PinMask);
	disable_transmit_lsm303ah();
	return result;
}
__STATIC_INLINE void multiread_lsm303ah( const uint8_t RegAdr, uint8_t * data, const uint32_t datalen )
{
	/* ?????? */
	uint8_t value = RegAdr | 0x80;
//	LL_GPIO_ResetOutputPin(GPIOx,PinMask);
	enable_transmit_lsm303ah();
//	while(!LL_SPI_IsActiveFlag_TXE(SPIx));
	check_TXE_lsm303ah();
//	LL_SPI_TransmitData8(SPIx, value);
	SPI1->DR = value;
//	while(!LL_SPI_IsActiveFlag_TXE(SPIx));
	check_TXE_lsm303ah();
//	while(LL_SPI_IsActiveFlag_BSY(SPIx));
	check_BSY_lsm303ah();
//	/* ?????? */
//	LL_SPI_SetTransferDirection(SPIx,LL_SPI_HALF_DUPLEX_RX);
	set_halfduplexRX_lsm303ah();
//	for(uint32_t i = 0; i < val_count; i++)
//	{
//		while(!LL_SPI_IsActiveFlag_RXNE(SPIx));
//		val[i] = LL_SPI_ReceiveData8(SPIx);
//	}
	for (uint32_t i = 0; i < datalen; i++)
	{
		check_RXNE_lsm303ah();
		data[i] = (uint8_t)(READ_REG(SPI1->DR));
	}
//	LL_SPI_SetTransferDirection(SPIx,LL_SPI_HALF_DUPLEX_TX);
	set_halfduplexTX_lsm303ah();
//	LL_GPIO_SetOutputPin(GPIOx,PinMask);
	disable_transmit_lsm303ah();
}
__STATIC_INLINE uint8_t read_lsm303ah_fst(uint8_t address)
{
	uint8_t value = address | 0x80;
	//LL_GPIO_ResetOutputPin(GPIOx,PinMask);
	enable_transmit_lsm303ah();
	//while(!LL_SPI_IsActiveFlag_TXE(SPIx));
	if(!check_TXE_lsm303ah_fst()) return 0;
	//LL_SPI_TransmitData8(SPIx, value);
	LSM303_SPI->DR = value;
	//while(!LL_SPI_IsActiveFlag_TXE(SPIx));
	if(!check_TXE_lsm303ah_fst()) return 0;
	//while(LL_SPI_IsActiveFlag_BSY(SPIx));
	if(!check_BSY_lsm303ah_fst()) return 0;
	//LL_SPI_SetTransferDirection(SPIx,LL_SPI_HALF_DUPLEX_RX);
	set_halfduplexRX_lsm303ah();
	//while(!LL_SPI_IsActiveFlag_RXNE(SPIx));
	if(!check_RXNE_lsm303ah_fst()) return 0;
	uint8_t result = 0;
	//result = LL_SPI_ReceiveData8(SPIx);
	result = (uint8_t)(READ_REG(LSM303_SPI->DR));
	//LL_SPI_SetTransferDirection(SPIx,LL_SPI_HALF_DUPLEX_TX);
	set_halfduplexTX_lsm303ah();
	//LL_GPIO_SetOutputPin(GPIOx,PinMask);
	disable_transmit_lsm303ah();
	return result;
}
__STATIC_INLINE uint8_t multiread_lsm303ah_init( const uint8_t RegAdr )
{
    uint8_t value = RegAdr | 0x80;
	enable_transmit_lsm303ah();
	if(!check_TXE_lsm303ah_fst()) 
		return 0;
	LSM303_SPI->DR = value;
	if(!check_TXE_lsm303ah_fst()) 
		return 0;
	if(!check_BSY_lsm303ah_fst()) 
		return 0;
	set_halfduplexRX_lsm303ah();
    return 1;
}
__STATIC_INLINE uint8_t multiread_lsm303ah_data(uint8_t * data )
{
    if(!check_RXNE_lsm303ah_fst()) 
		return 0;
	*data = (uint8_t)(READ_REG(LSM303_SPI->DR));
    return 1;
}
__STATIC_INLINE void multiread_lsm303ah_rls(void)
{
    set_halfduplexTX_lsm303ah();
	disable_transmit_lsm303ah();
}
__STATIC_INLINE uint8_t multiread_lsm303ah_fst( const uint8_t RegAdr, uint8_t * data, const uint32_t datalen )
{
	uint8_t value = RegAdr | 0x80;
	enable_transmit_lsm303ah();
	if(!check_TXE_lsm303ah_fst()) 
		return 0;
	LSM303_SPI->DR = value;
	if(!check_TXE_lsm303ah_fst()) 
		return 0;
	if(!check_BSY_lsm303ah_fst()) 
		return 0;
	set_halfduplexRX_lsm303ah();
	for (uint32_t i = 0; i < datalen; i++)
	{
		if(!check_RXNE_lsm303ah_fst()) 
			return 0;
		data[i] = (uint8_t)(READ_REG(LSM303_SPI->DR));
	}
	set_halfduplexTX_lsm303ah();
	disable_transmit_lsm303ah();
	return 1;
}
__STATIC_INLINE uint8_t Get_XL_M_uint8_lsm303ah_fst(uint8_t * XLdata, uint8_t * Mdata)
{
	uint8_t res = read_lsm303ah_fst(LSM303AH_STATUS_A);
	if(!res) return 0;
  res &= 0x01;
	if(!res) return 0;
	if(!multiread_lsm303ah_fst(LSM303AH_OUT_X_L_A, XLdata, 3)) return 0;
//	if(!multiread_lsm303ah_fst(LSM303AH_OUTX_L_REG_M, Mdata, 6)) return 0;
	return 1;
}

#endif /* EXACTOREGLIB_INC_LSM303AH_REGLIB_H_ */
