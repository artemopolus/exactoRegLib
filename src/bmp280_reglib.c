/*
 * bmp280_reglib.c
 *
 *  Created on: 22 ���. 2019 �.
 *      Author: Temka
 */
#include "bmp280_reglib.h"

//#define BMP280_GPIO_CLOCK_ENABLE	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB)
//#define BMP280_I2C_CLOCK_ENABLE		LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2)
//#define BMP280_GPIO			GPIOB
//#define BMP280_SCL_PIN		LL_GPIO_PIN_10
//#define BMP280_SDA_PIN		LL_GPIO_PIN_11
//#define BMP280_I2C			I2C2
//#define I2C_SPEEDCLOCK           400000
//#define I2C_DUTYCYCLE            LL_I2C_DUTYCYCLE_2

#define I2C_REQUEST_WRITE                       0x00
#define I2C_REQUEST_READ                        0x01

void ClockEnPinsI2C_bmp280(void)
{
	//LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB)
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);
	//LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2)
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);
}
void ConfigurePinsI2C_bmp280(void)
{
//	LL_GPIO_SetPinMode			(BMP280_GPIO, BMP280_SCL_PIN, LL_GPIO_MODE_ALTERNATE);
//	MODIFY_REG(*pReg, ((GPIO_CRL_CNF0 | GPIO_CRL_MODE0) << (POSITION_VAL(Pin) * 4U)), (Mode << (POSITION_VAL(Pin) * 4U)));
//	LL_GPIO_MODE_ALTERNATE           (GPIO_CRL_CNF0_1 | GPIO_CRL_MODE0_0)
	MODIFY_REG(GPIOB->CRH,(GPIO_CRH_CNF10 | GPIO_CRH_MODE10),(GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0));

//	LL_GPIO_SetPinSpeed			(BMP280_GPIO, BMP280_SCL_PIN, LL_GPIO_SPEED_FREQ_HIGH);
//	  register uint32_t *pReg = (uint32_t *)((uint32_t)(&GPIOx->CRL) + (Pin >> 24));
//	  MODIFY_REG(*pReg, (GPIO_CRL_MODE0 << (POSITION_VAL(Pin) * 4U)),
//	             (Speed << (POSITION_VAL(Pin) * 4U)));
	MODIFY_REG(GPIOB->CRH,GPIO_CRH_MODE10,GPIO_CRH_MODE10);
//	LL_GPIO_SetPinOutputType	(BMP280_GPIO, BMP280_SCL_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
//	  register uint32_t *pReg = (uint32_t *)((uint32_t)(&GPIOx->CRL) + (Pin >> 24));
//	  MODIFY_REG(*pReg, (GPIO_CRL_CNF0_0 << (POSITION_VAL(Pin) * 4U)),
//	             (OutputType << (POSITION_VAL(Pin) * 4U)));
	MODIFY_REG(GPIOB->CRH,GPIO_CRH_CNF10_0,GPIO_CRH_CNF10_0);
//	LL_GPIO_SetPinPull			(BMP280_GPIO, BMP280_SCL_PIN, LL_GPIO_PULL_UP);
//	MODIFY_REG(GPIOx->ODR, (Pin >> GPIO_PIN_MASK_POS), Pull << (POSITION_VAL(Pin >> GPIO_PIN_MASK_POS)));
	MODIFY_REG(GPIOB->ODR, GPIO_ODR_ODR10, GPIO_ODR_ODR10);


//	LL_GPIO_SetPinMode			(BMP280_GPIO, BMP280_SDA_PIN, LL_GPIO_MODE_ALTERNATE);
//	LL_GPIO_SetPinSpeed			(BMP280_GPIO, BMP280_SDA_PIN, LL_GPIO_SPEED_FREQ_HIGH);
//	LL_GPIO_SetPinOutputType	(BMP280_GPIO, BMP280_SDA_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
//	LL_GPIO_SetPinPull			(BMP280_GPIO, BMP280_SDA_PIN, LL_GPIO_PULL_UP);
	MODIFY_REG(GPIOB->CRH,(GPIO_CRH_CNF11 | GPIO_CRH_MODE11),(GPIO_CRH_CNF11_1 | GPIO_CRH_MODE11_0));
	MODIFY_REG(GPIOB->CRH,GPIO_CRH_MODE11,GPIO_CRH_MODE11);
	MODIFY_REG(GPIOB->CRH,GPIO_CRH_CNF11_0,GPIO_CRH_CNF11_0);
	MODIFY_REG(GPIOB->ODR, GPIO_ODR_ODR11, GPIO_ODR_ODR11);
}
void ConfigureI2C_bmp280(void)
{

	//LL_I2C_Disable				(BMP280_I2C);
	CLEAR_BIT(I2C2->CR1, I2C_CR1_PE);
	exacto_RCC_ClocksTypeDef rcc_clocks;
	exacto_RCC_GetSystemClocksFreq	(&rcc_clocks);


	exacto_I2C_ConfigSpeed			(I2C2, rcc_clocks.PCLK1_Frequency, 400000, 0x00000000U);
	//LL_I2C_SetMode				(BMP280_I2C, LL_I2C_MODE_I2C);
	MODIFY_REG(I2C2->CR1, I2C_CR1_SMBUS | I2C_CR1_SMBTYPE | I2C_CR1_ENARP, 0x00000000U);
}
void ActivateI2C_bmp280(void)
{
	//LL_I2C_Enable(BMP280_I2C);
	SET_BIT(I2C2->CR1, I2C_CR1_PE);
}
uint8_t GetWhoami_bmp280(void)
{
	uint8_t value = readI2C_bmp280( 0x77, 0xD0);
	uint8_t res = (value == 0x58) ? 1 : 0;

	return res;
}
uint8_t read_bmp280(uint8_t address)
{
    return readI2C_bmp280( 0x77, address);
}
void multiread_bmp280(uint8_t address, uint8_t * values, uint8_t cnt)
{
	multireadI2C_bmp280(BMP280_I2C_ADDR_SEC, address, values, cnt);
}
uint8_t GetPresTempValuesUint8_bmp280(uint8_t * data)
{
	if(read_bmp280(BMP280_STATUS_ADDR))
		return 0; 
	multiread_bmp280(BMP280_PRES_MSB_ADDR, data, 6);
	return 1;
}
uint8_t GetPresTempValues_bmp280(int32_t * pres, int32_t * temp)
{
	if(read_bmp280(BMP280_STATUS_ADDR))
		return 0;
	uint8_t data[6];
	multiread_bmp280(BMP280_PRES_MSB_ADDR, data, 6);
	* pres = (int32_t) ((((uint32_t) 	(data[0])) << 12) | (((uint32_t) 	(data[1])) << 4) | ((uint32_t) 	data[2] >> 4));
  * temp = (int32_t) ((((int32_t) 	(data[3])) << 12) | (((int32_t) 	(data[4])) << 4) | (((int32_t) 	(data[5])) >> 4));
	return 1;
}
void write_bmp280(uint8_t address, uint8_t value)
{
    writeI2C_bmp280(0x77,address,value);
}
uint8_t readI2C_bmp280(uint8_t address, uint8_t reg)
{
	/* ST ; SAD + W */
	//LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	//LL_I2C_GenerateStartCondition(I2Cx);
	SET_BIT(I2C2->CR1, I2C_CR1_START);
//	while (!LL_I2C_IsActiveFlag_SB(I2Cx));
//	return (READ_BIT(I2Cx->SR1, I2C_SR1_SB) == (I2C_SR1_SB));
	check_I2C_SB_bmp280();
	//while(!	(READ_BIT(I2C2->SR1, I2C_SR1_SB) == (I2C_SR1_SB))	);
	uint8_t trg = (address<<1);
	//LL_I2C_TransmitData8(I2Cx, trg  | I2C_REQUEST_WRITE);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, (trg  | I2C_REQUEST_WRITE));
	/* wait SAK */
//	while(!LL_I2C_IsActiveFlag_ADDR(I2Cx));
//	return (READ_BIT(I2Cx->SR1, I2C_SR1_ADDR) == (I2C_SR1_ADDR));
	check_I2C_ADDR_bmp280();
	/* SUB */
//	LL_I2C_ClearFlag_ADDR(I2Cx);
	clearFlag_I2C_ADDR_bmp280();
	//while (!LL_I2C_IsActiveFlag_TXE(I2Cx));
	check_I2C_TXE_bmp280();
	//LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	//LL_I2C_TransmitData8(I2Cx, data);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, reg);
	//while (!LL_I2C_IsActiveFlag_TXE(I2Cx));
	check_I2C_TXE_bmp280();
	/* wait SAK */
//	while(!LL_I2C_IsActiveFlag_ADDR(I2Cx));
	/* SR ; SAD + R */
	//LL_I2C_GenerateStartCondition(I2Cx);
	SET_BIT(I2C2->CR1, I2C_CR1_START);
	//while (!LL_I2C_IsActiveFlag_SB(I2Cx));
	check_I2C_SB_bmp280();
	//LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_ACK);
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	//LL_I2C_TransmitData8(I2Cx, trg  | I2C_REQUEST_READ);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, (trg  | I2C_REQUEST_READ));
	/* wait SAK + DATA*/
	//while(!LL_I2C_IsActiveFlag_ADDR(I2Cx));
	check_I2C_ADDR_bmp280();
	//LL_I2C_ClearFlag_ADDR(I2Cx);
	clearFlag_I2C_ADDR_bmp280();
	//while (!LL_I2C_IsActiveFlag_RXNE(I2Cx));
	check_I2C_RXNE_bmp280();
	//uint8_t res = LL_I2C_ReceiveData8(I2Cx);
	uint8_t res = (uint8_t)(READ_BIT(I2C2->DR, I2C_DR_DR));
	//while (!LL_I2C_IsActiveFlag_RXNE(I2Cx));
	check_I2C_RXNE_bmp280();
	/* NMAK + SP */
	//LL_I2C_AcknowledgeNextData(I2Cx, LL_I2C_NACK);
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, 0x00000000U);
	//LL_I2C_GenerateStopCondition(I2Cx);
	SET_BIT(I2C2->CR1, I2C_CR1_STOP);
	return res;
}
void multireadI2C_bmp280(uint8_t address, uint8_t reg, uint8_t * values, uint8_t cnt)
{
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	SET_BIT(I2C2->CR1, I2C_CR1_START);
	check_I2C_SB_bmp280();
	uint8_t trg = (address<<1);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, (trg  | I2C_REQUEST_WRITE));
	check_I2C_ADDR_bmp280();
	clearFlag_I2C_ADDR_bmp280();
	check_I2C_TXE_bmp280();
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, reg);
	check_I2C_TXE_bmp280();
	SET_BIT(I2C2->CR1, I2C_CR1_START);
	check_I2C_SB_bmp280();
	
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, (trg  | I2C_REQUEST_READ));
	check_I2C_ADDR_bmp280();
	clearFlag_I2C_ADDR_bmp280();
	check_I2C_RXNE_bmp280();
	values[0] = (uint8_t)(READ_BIT(I2C2->DR, I2C_DR_DR));
	
	for (uint8_t i = 1; i < cnt; i++)
	{
		check_I2C_RXNE_bmp280();
		values[0] = (uint8_t)(READ_BIT(I2C2->DR, I2C_DR_DR));
	}
	
	check_I2C_RXNE_bmp280();
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, 0x00000000U);
	SET_BIT(I2C2->CR1, I2C_CR1_STOP);
}
void writeI2C_bmp280(uint8_t address, uint8_t reg,uint8_t value)
{
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	SET_BIT(I2C2->CR1, I2C_CR1_START);
	check_I2C_SB_bmp280();
	uint8_t trg = (address<<1);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, (trg  | I2C_REQUEST_WRITE));
	check_I2C_ADDR_bmp280();
    
	clearFlag_I2C_ADDR_bmp280();
	check_I2C_TXE_bmp280();
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, reg);
	check_I2C_TXE_bmp280();
    MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
	MODIFY_REG(I2C2->DR, I2C_DR_DR, value);
    check_I2C_TXE_bmp280();
    
	MODIFY_REG(I2C2->CR1, I2C_CR1_ACK, 0x00000000U);
	SET_BIT(I2C2->CR1, I2C_CR1_STOP);
}
void check_I2C_SB_bmp280(void)
{
	while(!	(READ_BIT(I2C2->SR1, I2C_SR1_SB) == (I2C_SR1_SB))	);
}
void check_I2C_ADDR_bmp280(void)
{
	while(!	(READ_BIT(I2C2->SR1, I2C_SR1_ADDR) == (I2C_SR1_ADDR))	);
}
void check_I2C_RXNE_bmp280(void)
{
	while(!	(READ_BIT(I2C2->SR1, I2C_SR1_RXNE) == (I2C_SR1_RXNE))	);
}
void clearFlag_I2C_ADDR_bmp280(void)
{
	__IO uint32_t tmpreg;
	tmpreg = I2C2->SR1;
	(void) tmpreg;
	tmpreg = I2C2->SR2;
	(void) tmpreg;
}
void check_I2C_TXE_bmp280(void)
{
	while(!	(READ_BIT(I2C2->SR1, I2C_SR1_TXE) == (I2C_SR1_TXE))	);
}

