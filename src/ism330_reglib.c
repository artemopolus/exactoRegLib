/*
 * ism330_reglib.c
 *
 *  Created on: 23 апр. 2019 г.
 *      Author: Temka
 */

#include "ism330_reglib.h"

//#define ISM330_GPIO_CLOCK_ENABLE	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB)
//#define ISM330_GPIO		GPIOB
//
//#define ISM330_SCK		LL_GPIO_PIN_13
//#define ISM330_MISO		LL_GPIO_PIN_14
//#define ISM330_MOSI		LL_GPIO_PIN_15
//#define ISM330_NSS		LL_GPIO_PIN_12
//
//#define ISM330_SPI_CLOCK_ENABLE		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2)
//#define ISM330_SPI	SPI2

void ConfigureSPI3_ism330(void)
{
//	ISM330_GPIO_CLOCK_ENABLE;
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);
//	LL_GPIO_SetPinMode	(ISM330_GPIO, ISM330_SCK, LL_GPIO_MODE_ALTERNATE);
//	LL_GPIO_SetPinSpeed	(ISM330_GPIO, ISM330_SCK, LL_GPIO_SPEED_FREQ_LOW);
//	LL_GPIO_SetPinPull	(ISM330_GPIO, ISM330_SCK, LL_GPIO_PULL_DOWN);
//	LL_GPIO_SetPinMode	(ISM330_GPIO, ISM330_MOSI, LL_GPIO_MODE_ALTERNATE);
//	LL_GPIO_SetPinSpeed	(ISM330_GPIO, ISM330_MOSI, LL_GPIO_SPEED_FREQ_LOW);
//	LL_GPIO_SetPinPull	(ISM330_GPIO, ISM330_MOSI, LL_GPIO_PULL_DOWN);
	GPIOB->CRH   |=  GPIO_CRH_MODE12;    //
	GPIOB->CRH   &= ~GPIO_CRH_CNF12;     //
	GPIOB->BSRR   =  GPIO_CRH_CNF12_0;     //

//	GPIOB->CRH   |=  GPIO_CRH_MODE13;    //
//	GPIOB->CRH   &= ~GPIO_CRH_CNF13;     //
//	GPIOB->CRH   |=  GPIO_CRH_CNF13_1;   //
	GPIOB->CRH |= 	(GPIO_CRH_CNF13	|	GPIO_CRH_MODE13);
	GPIOB->CRH &= ~	(GPIO_CRH_CNF13	|	GPIO_CRH_MODE13);
	GPIOB->CRH |= 	(GPIO_CRH_CNF13_1	|	GPIO_CRH_MODE13_1);

//	GPIOB->CRH   |=  GPIO_CRH_MODE15;    //
	//	GPIOB->CRH   &= ~GPIO_CRH_CNF15;     //
	//	GPIOB->CRH   |=  GPIO_CRH_CNF15_1;   //
	GPIOB->CRH |= 	(GPIO_CRH_CNF15	|	GPIO_CRH_MODE15);
	GPIOB->CRH &= ~	(GPIO_CRH_CNF15	|	GPIO_CRH_MODE15);
	GPIOB->CRH |= 	(GPIO_CRH_CNF15_1	|	GPIO_CRH_MODE15_1);
//	ISM330_SPI_CLOCK_ENABLE;
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
//	LL_SPI_SetBaudRatePrescaler	(ISM330_SPI, LL_SPI_BAUDRATEPRESCALER_DIV256);
//	LL_SPI_SetTransferDirection	(ISM330_SPI,LL_SPI_HALF_DUPLEX_TX);
//	LL_SPI_SetClockPhase		(ISM330_SPI, LL_SPI_PHASE_2EDGE);
//	LL_SPI_SetClockPolarity		(ISM330_SPI, LL_SPI_POLARITY_HIGH);
//	LL_SPI_SetDataWidth			(ISM330_SPI, LL_SPI_DATAWIDTH_8BIT);
//	LL_SPI_SetNSSMode			(ISM330_SPI, LL_SPI_NSS_SOFT);
//	LL_SPI_SetMode				(ISM330_SPI, LL_SPI_MODE_MASTER);
	MODIFY_REG(SPI2->CR1, SPI_CR1_BR, (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0));
	set_halfduplexTX_ism330();
    MODIFY_REG(SPI2->CR1, SPI_CR1_CPHA, SPI_CR1_CPHA);
    MODIFY_REG(SPI2->CR1, SPI_CR1_CPOL, SPI_CR1_CPOL);
    MODIFY_REG(SPI2->CR1, SPI_CR1_DFF, 0x00000000U);
    MODIFY_REG(SPI2->CR1, SPI_CR1_SSM,  SPI_CR1_SSM);
    MODIFY_REG(SPI2->CR2, SPI_CR2_SSOE, ((uint32_t)(SPI_CR1_SSM >> 16U)));
    MODIFY_REG(SPI2->CR1, SPI_CR1_MSTR | SPI_CR1_SSI, (SPI_CR1_MSTR | SPI_CR1_SSI));
}
void ActivateSPI3_ism330(void)
{
	SET_BIT(SPI2->CR1, SPI_CR1_SPE);
	disable_transmit_ism330();
}
uint8_t Set3wireAndGetWhoami_ism330(void)
{
//	writeSPI_lsm303(ISM330_SPI, ISM330_GPIO, ISM330_NSS, 0x12, 0x0C);
//	value = readWordSPI_lsm303(ISM330_SPI, ISM330_GPIO, ISM330_NSS, ISM330_WHOAMI_ADR);
//	res = (value == ISM330_WHOAMI_VAL) ? 1 : 0;
//#define ISM330_WHOAMI_ADR	UINT8_C(0x0F)
//#define ISM330_WHOAMI_VAL	UINT8_C(0x6A)
	write_ism330(0x12, 0x0C);
	uint8_t res = 0, value = 0;
	value = read_ism330(0x0F);
	res = (value == 0x6A) ? 1 : 0;
	return res;
}
uint8_t GetTGData_ism330(uint8_t * data)
{
//    if((read_ism330(ISM330DLC_STATUS_REG) & 0x05) != 0x05)
//        return 0;
//    multiread_ism330(ISM330DLC_OUT_TEMP_L, data, 8);
//    return 1;
		return multiread_ism330_fst(ISM330DLC_OUT_TEMP_L,data,8);
}
uint8_t GetGXLData_ism330(uint8_t * data)
{
//    if((read_ism330(ISM330DLC_STATUS_REG) & 0x03) != 0x03)
//        return 0;
//    multiread_ism330(ISM330DLC_OUTX_L_G, data, 6);
//    return 1;
		return multiread_ism330_fst(ISM330DLC_OUTX_L_G,data,6);
}
uint8_t GetXLData_ism330(uint8_t * data)
{
//    if(read_ism330(ISM330DLC_STATUS_REG) & 0x01)
//        return 0;
//    multiread_ism330(ISM330DLC_OUTX_L_XL, data, 3);
//    return 1;
		return multiread_ism330_fst(ISM330DLC_OUTX_L_XL,data,6);
}
uint8_t GetGData_ism330(uint8_t * data)
{
//    if(read_ism330(ISM330DLC_STATUS_REG) & 0x02)
//        return 0;
//    multiread_ism330(ISM330DLC_OUTX_L_G, data, 3);
//    return 1;
		return multiread_ism330_fst(ISM330DLC_OUTX_L_G,data,6);
}
uint8_t Get_T_G_XL_uint8_ism330(uint8_t * data)
{
//	if(!read_ism330(ISM330DLC_STATUS_REG))	return 0;
//	multiread_ism330(ISM330DLC_OUT_TEMP_L, data, 14);
//	return 1;
	return multiread_ism330_fst(ISM330DLC_OUT_TEMP_L,data,14);
}
uint8_t GetTData_ism330(uint8_t * data)
{
	return multiread_ism330_fst(ISM330DLC_OUT_TEMP_L,data,2);
}
uint8_t GetFlagDRDY_ism330(void)
{
	return read_ism330(ISM330DLC_STATUS_REG);
}
