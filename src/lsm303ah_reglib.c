/*
 * lsm303ah_reglib.c
 *
 *  Created on: 19 апр. 2019 г.
 *      Author: Temka
 */


#include "lsm303ah_reglib.h"

//#define LSM303AH_SPI                    SPI1
//#define	LSM303AH_CS_PIN                 LL_GPIO_PIN_4
//#define LSM303AH_GPIO                   GPIOA
//#define LSM303AH_SCK_PIN                LL_GPIO_PIN_5
//#define LSM303AH_SDA_PIN                LL_GPIO_PIN_7
//#define LSM303AH_APB2_GRP1_PERIPH_GPIO  LL_APB2_GRP1_PERIPH_GPIOA
//#define LSM303AH_APB2_GRP1_PERIPH_SPI   LL_APB2_GRP1_PERIPH_SPI1

void ClockEnPins_lsm303ah(void)
{
	RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN;
}
void ConfigurePins_lsm303ah(void)
{
// 	LL_GPIO_SetPinMode(LSM303AH_GPIO, LSM303AH_CS_PIN, LL_GPIO_MODE_OUTPUT);
//	LL_GPIO_SetPinMode(LSM303AH_GPIO, LSM303AH_SCK_PIN, LL_GPIO_MODE_ALTERNATE);
//	LL_GPIO_SetPinSpeed(LSM303AH_GPIO, LSM303AH_SCK_PIN, LL_GPIO_SPEED_FREQ_LOW);
//	LL_GPIO_SetPinPull(LSM303AH_GPIO, LSM303AH_SCK_PIN, LL_GPIO_PULL_DOWN);
//	LL_GPIO_SetPinMode(LSM303AH_GPIO, LSM303AH_SDA_PIN, LL_GPIO_MODE_ALTERNATE);
//	LL_GPIO_SetPinSpeed(LSM303AH_GPIO, LSM303AH_SDA_PIN, LL_GPIO_SPEED_FREQ_LOW);
//	LL_GPIO_SetPinPull(LSM303AH_GPIO, LSM303AH_SDA_PIN, LL_GPIO_PULL_DOWN);
	GPIOA->CRL   |=  GPIO_CRL_MODE4;    //
	GPIOA->CRL   &= ~GPIO_CRL_CNF4;     //
	GPIOA->BSRR   =  GPIO_CRL_CNF4_0;     //

	GPIOA->CRL   |=  GPIO_CRL_MODE5;    //
	GPIOA->CRL   &= ~GPIO_CRL_CNF5;     //
	GPIOA->CRL   |=  GPIO_CRL_CNF5_1;   //

	GPIOA->CRL   |=  GPIO_CRL_MODE7;    //
	GPIOA->CRL   &= ~GPIO_CRL_CNF7;     //
	GPIOA->CRL   |=  GPIO_CRL_CNF7_1;   //
}
void ConfigureSPI3_lsm303ah(void)
{
//	LL_APB2_GRP1_EnableClock(LSM303AH_APB2_GRP1_PERIPH_SPI);
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
//    LL_SPI_SetBaudRatePrescaler(LSM303AH_SPI, LL_SPI_BAUDRATEPRESCALER_DIV256);
//    MODIFY_REG(SPIx->CR1, SPI_CR1_BR, BaudRate);
//    LL_SPI_BAUDRATEPRESCALER_DIV256    (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)
//    MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
	MODIFY_REG(SPI1->CR1, SPI_CR1_BR, (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0));
	set_halfduplexTX_lsm303ah();
//    LL_SPI_SetClockPhase(LSM303AH_SPI, LL_SPI_PHASE_2EDGE);
//    MODIFY_REG(SPIx->CR1, SPI_CR1_CPHA, ClockPhase);
//    LL_SPI_PHASE_2EDGE                 (SPI_CR1_CPHA)
    MODIFY_REG(SPI1->CR1, SPI_CR1_CPHA, SPI_CR1_CPHA);
//    LL_SPI_SetClockPolarity(LSM303AH_SPI, LL_SPI_POLARITY_HIGH);
//    LL_SPI_POLARITY_HIGH               (SPI_CR1_CPOL)
    MODIFY_REG(SPI1->CR1, SPI_CR1_CPOL, SPI_CR1_CPOL);
    //LL_SPI_SetDataWidth(LSM303AH_SPI, LL_SPI_DATAWIDTH_8BIT);
    MODIFY_REG(SPI1->CR1, SPI_CR1_DFF, 0x00000000U);
    //LL_SPI_SetNSSMode(LSM303AH_SPI, LL_SPI_NSS_SOFT);
    MODIFY_REG(SPI1->CR1, SPI_CR1_SSM,  SPI_CR1_SSM);
    MODIFY_REG(SPI1->CR2, SPI_CR2_SSOE, ((uint32_t)(SPI_CR1_SSM >> 16U)));
    //LL_SPI_SetMode(LSM303AH_SPI, LL_SPI_MODE_MASTER);
    MODIFY_REG(SPI1->CR1, SPI_CR1_MSTR | SPI_CR1_SSI, (SPI_CR1_MSTR | SPI_CR1_SSI));

}
void ActivateSPI3_lsm303ah(void)
{
	//LL_SPI_Enable(LSM303AH_SPI);
	SET_BIT(SPI1->CR1, SPI_CR1_SPE);
	//LL_GPIO_SetOutputPin(LSM303AH_GPIO, LSM303AH_CS_PIN);
	disable_transmit_lsm303ah();
}
uint8_t Set3wireAndGetWhoami_lsm303ah(void)
{
	//writeSPI_lsm303(LSM303AH_SPI, LSM303AH_GPIO, LSM303AH_CS_PIN, LSM303AH_CTRL2_A_ADR, LSM303AH_CTRL2_A_SETUP);
	write_lsm303ah(0x21,0x05);
	uint8_t res = 0, value_xl = 0, value_mg = 0;
//	    value_xl = readWordSPI_lsm303(LSM303AH_SPI, LSM303AH_GPIO, LSM303AH_CS_PIN, LSM303AH_WHOAMI_XL_ADR);
	value_xl = read_lsm303ah(0x0f);
	if (value_xl == 0x43)
	{
		value_mg = read_lsm303ah(0x4f);
 		if(0x40 == value_mg)      res = 1;
		if(0x41 == value_mg)     res = 1;
	}

//	    if(res == 1)
//	    {
//	        sensorLSM303AH = WHOIAM_OK;
//	    }
//	    //res = ((LSM303AH_WHOAMI_XL_VAL == value_xl) && (LSM303AH_WHOAMI_MG_VAL2 == value_mg)) ? 1 : 0;
	return res;
}
uint8_t GetReadyFlag_lsm303ah(void)
{
    uint8_t res = 0;
		res = read_lsm303ah(LSM303AH_STATUS_A);
    uint8_t mask = 0x01;
    res = res & mask;
    return res;
}
uint8_t GetXLallData_lsm303ah(int16_t * data)
{
	if(!GetReadyFlag_lsm303ah())
		return 0;
	uint8_t src[6];
	multiread_lsm303ah(LSM303AH_OUT_X_L_A, src, 6);
	for (uint8_t i = 0; i < 3; i++)
	{
		data[i] = GetValue_lsm303ah(src[i*2],src[i*2+1]);
	}
	return 1;
}
uint8_t GetXLallDataUint8_lsm303ah(uint8_t * data)
{
    if(!GetReadyFlag_lsm303ah())
		return 0;
	multiread_lsm303ah(LSM303AH_OUT_X_L_A, data, 6);
	return 1;
}
uint8_t Get_XL_M_uint8_lsm303ah(uint8_t * XLdata, uint8_t * Mdata)
{
	if(!GetReadyFlag_lsm303ah()) return 0;
	multiread_lsm303ah(LSM303AH_OUT_X_L_A, XLdata, 6);
	multiread_lsm303ah(LSM303AH_OUTX_L_REG_M, Mdata, 6);
	return 1;
}
int16_t GetValue_lsm303ah(const uint8_t lsb, const uint8_t msb)
{
	int16_t res = 0;
	res = ((int16_t)msb << 8) | lsb;
	return res;
}

