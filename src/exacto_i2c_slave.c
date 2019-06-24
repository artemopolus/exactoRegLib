/*
 * exacto_i2c_slave.c
 *
 *  Created on: 22 апр. 2019 г.
 *      Author: Artem
 */

#include "exacto_i2c_slave.h"

#define SLAVE_OWN_ADDRESS                       0x5A

//#define	I2C_DMA_SLAVE							I2C2
//#define I2C_DMA_SLAVE_EV_IRQn					I2C2_EV_IRQn
//#define I2C_DMA_SLAVE_ER_IRQn					I2C2_ER_IRQn
//#define I2C_DMA_SLAVE_APB1_GRP1_PERIPH_I2C 		LL_APB1_GRP1_PERIPH_I2C2
//#define I2C_DMA_SLAVE_APB2_GRP1_PERIPH_GPIO		LL_APB2_GRP1_PERIPH_GPIOB
//#define I2C_DMA_SLAVE_GPIO						GPIOB
//#define I2C_DMA_SLAVE_SCL_PIN					LL_GPIO_PIN_10
//#define I2C_DMA_SLAVE_SDA_PIN					LL_GPIO_PIN_11
//
//#define I2C_DMA_DMA					DMA1
//#define I2C_DMA_DMA_RECEIVE			LL_DMA_CHANNEL_5

//#define I2C_DMA_DMA_TRANSMIT		LL_DMA_CHANNEL_4
#define I2C_DMA_DMA_RX_IRQnHandler	DMA1_Channel7_IRQHandler
#define I2C_DMA_DMA_TX_IRQnHandler	DMA1_Channel6_IRQHandler
#define I2C_DMA_DMA_RECEIVE_IRQn	DMA1_Channel7_IRQn
#define I2C_DMA_DMA_TRANSMIT_IRQn	DMA1_Channel6_IRQn
#define I2C_DMA_DMA_CHANNEL_RX	DMA1_Channel7
#define I2C_DMA_DMA_CHANNEL_TX	DMA1_Channel6
#define I2C_DMA_I2C	I2C1
#define I2C_DMA_DMA	DMA1
#define	I2C_DMA_I2C_EV_IRQn I2C1_EV_IRQn
#define	I2C_DMA_I2C_ER_IRQn I2C1_ER_IRQn

#define SLAVE_BYTE_TO_SEND       (uint8_t)0xA5

__IO uint8_t FlagTransmitBlocked = 0;

char Word2transmit_i2c_dma_slave[] = "hello";

#define MAXNBWORD2TRANSMIT  (EXACTOLBIDATASIZE + 4 + EXACTOLBIARRAYCNT)
char pTransmitBuffer_i2c_dma_slave[MAXNBWORD2TRANSMIT];
//__IO uint8_t nbWord2Transmit = 0;

//__IO uint8_t  ubNbDataToTransmit             = sizeof(word2transmit);

//uint32_t * pTransmitBufferDMA = (uint32_t*)(&word2transmit[0]);
uint32_t * ptTransmit_i2c_dma_slave = (uint32_t*)(&pTransmitBuffer_i2c_dma_slave[0]);
__IO uint32_t  uCountTransmit_i2c_dma_slave  = 0;
uint8_t       ptReceive_i2c_dma_slave[0x04] = {0};
__IO uint8_t  uCountReceive_i2c_dma_slave   = sizeof(ptReceive_i2c_dma_slave);

__IO uint8_t  flagReceiveTransferComplete_i2c_dma_slave        = 1;
__IO uint8_t  flagTransmitTransferComplete_i2c_dma_slave       = 1;

typedef enum {READ,WRITE,NONE}i2c_wire_status;
i2c_wire_status Status_i2c_dma_slave = NONE;
typedef enum {RECEIVETRANSMIT, TRANSMIT} i2c_tr_mode;
i2c_tr_mode Mode_i2c_dma_slave = RECEIVETRANSMIT;

void ConfigurePins_i2c_dma_slave(void);
void ConfigureMode_i2c_dma_slave(void)
{
	//LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	SET_BIT(RCC->AHBENR, RCC_AHBENR_DMA1EN);

	/* (2) Configure NVIC for DMA transfer complete/error interrupts */
	NVIC_SetPriority(I2C_DMA_DMA_RECEIVE_IRQn, 0); /* another var: 0x04 */
	NVIC_EnableIRQ(I2C_DMA_DMA_RECEIVE_IRQn);
	NVIC_SetPriority(I2C_DMA_DMA_TRANSMIT_IRQn, 0); /* 0x01 */
	NVIC_EnableIRQ(I2C_DMA_DMA_TRANSMIT_IRQn);


//	LL_DMA_ConfigTransfer(I2C_DMA_DMA, I2C_DMA_DMA_RECEIVE, LL_DMA_DIRECTION_PERIPH_TO_MEMORY | \
//			LL_DMA_PRIORITY_HIGH              | \
//			LL_DMA_MODE_NORMAL                | \
//			LL_DMA_PERIPH_NOINCREMENT           | \
//			LL_DMA_MEMORY_INCREMENT           | \
//			LL_DMA_PDATAALIGN_BYTE            | \
//			LL_DMA_MDATAALIGN_BYTE);
////	__STATIC_INLINE void LL_DMA_ConfigTransfer(DMA_TypeDef *DMAx, uint32_t Channel, uint32_t Configuration)
////	{
//	  MODIFY_REG(((DMA_Channel_TypeDef *)((uint32_t)((uint32_t)DMAx + CHANNEL_OFFSET_TAB[Channel - 1U])))->CCR,
//	             DMA_CCR_DIR | DMA_CCR_MEM2MEM | DMA_CCR_CIRC | DMA_CCR_PINC | DMA_CCR_MINC | DMA_CCR_PSIZE | DMA_CCR_MSIZE | DMA_CCR_PL,
//	             Configuration);

#ifdef EXACTO_SPL
       uint32_t conf = 0x00000000U | DMA_CCR1_PL_1 | 0x00000000U | 0x00000000U | DMA_CCR1_MINC | 0x00000000U | 0x00000000U;
#endif
#ifdef EXACTO_HAL
	  uint32_t conf = 0x00000000U | DMA_CCR_PL_1 | 0x00000000U | 0x00000000U | DMA_CCR_MINC | 0x00000000U | 0x00000000U;
#endif
#ifdef EXACTO_SPL
	MODIFY_REG(I2C_DMA_DMA_CHANNEL_RX->CCR, (DMA_CCR1_DIR | DMA_CCR1_MEM2MEM | DMA_CCR1_CIRC | DMA_CCR1_PINC | DMA_CCR1_MINC | DMA_CCR1_PSIZE | DMA_CCR1_MSIZE | DMA_CCR1_PL),conf);
#endif
#ifdef EXACTO_HAL
    MODIFY_REG(DMA1_Channel5->CCR, (DMA_CCR_DIR | DMA_CCR_MEM2MEM | DMA_CCR_CIRC | DMA_CCR_PINC | DMA_CCR_MINC | DMA_CCR_PSIZE | DMA_CCR_MSIZE | DMA_CCR_PL),conf);
#endif

	//LL_DMA_SetDataLength(I2C_DMA_DMA, I2C_DMA_DMA_RECEIVE, ubNbaReceiveBufferDMA);
    
#ifdef EXACTO_SPL
	MODIFY_REG(I2C_DMA_DMA_CHANNEL_RX->CNDTR,   DMA_CNDTR1_NDT, uCountReceive_i2c_dma_slave);
#endif
#ifdef EXACTO_HAL
    MODIFY_REG(DMA1_Channel5->CNDTR,   DMA_CNDTR_NDT, uCountReceive_i2c_dma_slave);
#endif

//	LL_DMA_ConfigAddresses(I2C_DMA_DMA, I2C_DMA_DMA_RECEIVE, (uint32_t)LL_I2C_DMA_GetRegAddr(I2C_DMA_SLAVE),
//			(uint32_t)&(aReceiveBufferDMA), LL_DMA_GetDataTransferDirection(I2C_DMA_DMA, I2C_DMA_DMA_RECEIVE));

	WRITE_REG(I2C_DMA_DMA_CHANNEL_RX->CPAR, (uint32_t) & (I2C_DMA_I2C->DR));
	WRITE_REG(I2C_DMA_DMA_CHANNEL_RX->CMAR, (uint32_t)&(ptReceive_i2c_dma_slave));

//	/* (4) Configure the DMA1_Channel6 functionnal parameters */
//	LL_DMA_ConfigTransfer(I2C_DMA_DMA, I2C_DMA_DMA_TRANSMIT, LL_DMA_DIRECTION_MEMORY_TO_PERIPH | \
//			LL_DMA_PRIORITY_HIGH              | \
//			LL_DMA_MODE_NORMAL                | \
//			LL_DMA_PERIPH_NOINCREMENT           | \
//			LL_DMA_MEMORY_INCREMENT           | \
//			LL_DMA_PDATAALIGN_BYTE            | \
//			LL_DMA_MDATAALIGN_BYTE);

#ifdef EXACTO_SPL
	conf = DMA_CCR1_DIR | DMA_CCR1_PL_1 | 0x00000000U | 0x00000000U | DMA_CCR1_MINC | 0x00000000U | 0x00000000U;
	MODIFY_REG(I2C_DMA_DMA_CHANNEL_TX->CCR, (DMA_CCR1_DIR | DMA_CCR1_MEM2MEM | DMA_CCR1_CIRC | DMA_CCR1_PINC | DMA_CCR1_MINC | DMA_CCR1_PSIZE | DMA_CCR1_MSIZE | DMA_CCR1_PL),conf);
	//LL_DMA_SetDataLength(I2C_DMA_DMA, I2C_DMA_DMA_TRANSMIT, ubNbTBDataToTransmit);
	MODIFY_REG(I2C_DMA_DMA_CHANNEL_TX->CNDTR,   DMA_CNDTR1_NDT, uCountTransmit_i2c_dma_slave);
#endif
#ifdef EXACTO_HAL
    conf = DMA_CCR_DIR | DMA_CCR_PL_1 | 0x00000000U | 0x00000000U | DMA_CCR_MINC | 0x00000000U | 0x00000000U;
	MODIFY_REG(DMA1_Channel4->CCR, (DMA_CCR_DIR | DMA_CCR_MEM2MEM | DMA_CCR_CIRC | DMA_CCR_PINC | DMA_CCR_MINC | DMA_CCR_PSIZE | DMA_CCR_MSIZE | DMA_CCR_PL),conf);
	//LL_DMA_SetDataLength(I2C_DMA_DMA, I2C_DMA_DMA_TRANSMIT, ubNbTBDataToTransmit);
	MODIFY_REG(DMA1_Channel4->CNDTR,   DMA_CNDTR_NDT, uCountTransmit_i2c_dma_slave);
#endif
	
//	LL_DMA_ConfigAddresses(I2C_DMA_DMA, I2C_DMA_DMA_TRANSMIT,
//			(uint32_t)pTransmitBufferDMA,(uint32_t)LL_I2C_DMA_GetRegAddr(I2C_DMA_SLAVE),
//			LL_DMA_GetDataTransferDirection(I2C_DMA_DMA, I2C_DMA_DMA_TRANSMIT));
	WRITE_REG(I2C_DMA_DMA_CHANNEL_TX->CMAR, (uint32_t)ptTransmit_i2c_dma_slave);
	WRITE_REG(I2C_DMA_DMA_CHANNEL_TX->CPAR, (uint32_t) & (I2C_DMA_I2C->DR));

	/* (4) Enable DMA interrupts complete/error */
	
#ifdef EXACTO_HAL
    //LL_DMA_EnableIT_TC(I2C_DMA_DMA, I2C_DMA_DMA_RECEIVE);
	SET_BIT(DMA1_Channel5->CCR, DMA_CCR_TCIE);
	//LL_DMA_EnableIT_TE(I2C_DMA_DMA, I2C_DMA_DMA_RECEIVE);
	SET_BIT(DMA1_Channel5->CCR, DMA_CCR_TEIE);
	//LL_DMA_EnableIT_TC(I2C_DMA_DMA, I2C_DMA_DMA_TRANSMIT);
	SET_BIT(DMA1_Channel4->CCR, DMA_CCR_TCIE);
	//LL_DMA_EnableIT_TE(I2C_DMA_DMA, I2C_DMA_DMA_TRANSMIT);
	SET_BIT(DMA1_Channel4->CCR, DMA_CCR_TEIE);
#endif
#ifdef EXACTO_SPL
	//LL_DMA_EnableIT_TC(I2C_DMA_DMA, I2C_DMA_DMA_RECEIVE);
	SET_BIT(I2C_DMA_DMA_CHANNEL_RX->CCR, DMA_CCR1_TCIE);
	//LL_DMA_EnableIT_TE(I2C_DMA_DMA, I2C_DMA_DMA_RECEIVE);
	SET_BIT(I2C_DMA_DMA_CHANNEL_RX->CCR, DMA_CCR1_TEIE);
	//LL_DMA_EnableIT_TC(I2C_DMA_DMA, I2C_DMA_DMA_TRANSMIT);
	SET_BIT(I2C_DMA_DMA_CHANNEL_TX->CCR, DMA_CCR1_TCIE);
	//LL_DMA_EnableIT_TE(I2C_DMA_DMA, I2C_DMA_DMA_TRANSMIT);
	SET_BIT(I2C_DMA_DMA_CHANNEL_TX->CCR, DMA_CCR1_TEIE);
#endif


	/* (1) Enables GPIO clock */
	/* Enable the peripheral clock of GPIOB */
	//LL_APB2_GRP1_EnableClock(I2C_DMA_SLAVE_APB2_GRP1_PERIPH_GPIO);
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);
	/* (2) Enable the I2C1 peripheral clock *************************************/
	/* Enable the peripheral clock for I2C1 */
	//LL_APB1_GRP1_EnableClock(I2C_DMA_SLAVE_APB1_GRP1_PERIPH_I2C);
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);
	/* Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up */
//	LL_GPIO_SetPinMode(			I2C_DMA_SLAVE_GPIO, I2C_DMA_SLAVE_SCL_PIN, LL_GPIO_MODE_ALTERNATE);
//	LL_GPIO_SetPinSpeed(		I2C_DMA_SLAVE_GPIO, I2C_DMA_SLAVE_SCL_PIN, LL_GPIO_SPEED_FREQ_HIGH);
//	LL_GPIO_SetPinOutputType(	I2C_DMA_SLAVE_GPIO, I2C_DMA_SLAVE_SCL_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
//	LL_GPIO_SetPinPull(			I2C_DMA_SLAVE_GPIO, I2C_DMA_SLAVE_SCL_PIN, LL_GPIO_PULL_UP);
	MODIFY_REG(GPIOB->CRL,(GPIO_CRL_CNF6 | GPIO_CRL_MODE6),(GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6_0));
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_MODE6,GPIO_CRL_MODE6);
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF6_0,GPIO_CRL_CNF6_0);
	MODIFY_REG(GPIOB->ODR, GPIO_ODR_ODR6, GPIO_ODR_ODR6);
	/* Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up */
//	LL_GPIO_SetPinMode(			I2C_DMA_SLAVE_GPIO, I2C_DMA_SLAVE_SDA_PIN, LL_GPIO_MODE_ALTERNATE);
//	LL_GPIO_SetPinSpeed(		I2C_DMA_SLAVE_GPIO, I2C_DMA_SLAVE_SDA_PIN, LL_GPIO_SPEED_FREQ_HIGH);
//	LL_GPIO_SetPinOutputType(	I2C_DMA_SLAVE_GPIO, I2C_DMA_SLAVE_SDA_PIN, LL_GPIO_OUTPUT_OPENDRAIN);
//	LL_GPIO_SetPinPull(			I2C_DMA_SLAVE_GPIO, I2C_DMA_SLAVE_SDA_PIN, LL_GPIO_PULL_UP);
	MODIFY_REG(GPIOB->CRL,(GPIO_CRL_CNF7 | GPIO_CRL_MODE7),(GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7_0));
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_MODE7,GPIO_CRL_MODE7);
	MODIFY_REG(GPIOB->CRL, GPIO_CRL_CNF7_0,GPIO_CRL_CNF7_0);
	MODIFY_REG(GPIOB->ODR, GPIO_ODR_ODR7, GPIO_ODR_ODR7);
	/* Configure interrupt */
	NVIC_SetPriority(I2C_DMA_I2C_EV_IRQn, 0);
	NVIC_EnableIRQ(I2C_DMA_I2C_EV_IRQn);
	NVIC_SetPriority(I2C_DMA_I2C_ER_IRQn, 0);
	NVIC_EnableIRQ(I2C_DMA_I2C_ER_IRQn);
	/* (3) Configure I2C1 functional parameters ********************************/
	//LL_I2C_Disable(I2C_DMA_SLAVE);
	CLEAR_BIT(I2C_DMA_I2C->CR1, I2C_CR1_PE);
	//LL_I2C_SetOwnAddress1(I2C_DMA_SLAVE, SLAVE_OWN_ADDRESS, LL_I2C_OWNADDRESS1_7BIT);
	MODIFY_REG(I2C_DMA_I2C->OAR1, I2C_OAR1_ADD0 | I2C_OAR1_ADD1_7 | I2C_OAR1_ADD8_9 | I2C_OAR1_ADDMODE, SLAVE_OWN_ADDRESS | 0x00004000U);
}
void Activate2work_i2c_dma_slave(void)
{
	//LL_I2C_Enable(I2C_DMA_SLAVE);
	SET_BIT(I2C_DMA_I2C->CR1, I2C_CR1_PE);
	//LL_I2C_EnableIT_EVT(I2C_DMA_SLAVE);
	SET_BIT(I2C_DMA_I2C->CR2, I2C_CR2_ITEVTEN);
	//LL_I2C_EnableIT_ERR(I2C_DMA_SLAVE);
	SET_BIT(I2C_DMA_I2C->CR2, I2C_CR2_ITERREN);
}
void Handle_i2c_dma_slave()
{
	/* Буффер передачи данных */
	uint8_t stringData[] = "TestedData";
	SetData2word2Transmit_i2c_dma_slave(stringData);
	/* Включаем буффер на прием данных */
#ifdef EXACTO_SPL
	MODIFY_REG(I2C_DMA_DMA_CHANNEL_RX->CNDTR, DMA_CNDTR1_NDT, uCountReceive_i2c_dma_slave);
	SET_BIT(I2C_DMA_DMA_CHANNEL_RX->CCR, DMA_CCR1_EN);
#endif
#ifdef EXACTO_HAL
    //LL_DMA_SetDataLength(I2C_DMA_DMA, I2C_DMA_DMA_RECEIVE, ubNbaReceiveBufferDMA);
	MODIFY_REG(DMA1_Channel5->CNDTR, DMA_CNDTR_NDT, uCountReceive_i2c_dma_slave);
	//LL_DMA_EnableChannel(I2C_DMA_DMA, I2C_DMA_DMA_RECEIVE);
	SET_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
#endif
	/* Создаем сигнал принятия для проверки адреса */
	//LL_I2C_AcknowledgeNextData(I2C_DMA_SLAVE, LL_I2C_ACK);
	MODIFY_REG(I2C_DMA_I2C->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
}
#ifdef EXACTO_SPL
uint8_t DMA_Body_TX_IRQHandler(void)
{
	//if(LL_DMA_IsActiveFlag_TC4(I2C_DMA_DMA))
	if((READ_BIT(I2C_DMA_DMA->ISR, DMA_ISR_TCIF6) == (DMA_ISR_TCIF6)))
	{
		//LL_DMA_ClearFlag_GI4(I2C_DMA_DMA);
		WRITE_REG(I2C_DMA_DMA->IFCR, DMA_IFCR_CGIF6);
		Transfer_Complete_i2c_dma_slave();
        return 1;
	}
//	else if(LL_DMA_IsActiveFlag_TE4(I2C_DMA_DMA))
	else if(READ_BIT(I2C_DMA_DMA->ISR, DMA_ISR_TEIF6) == (DMA_ISR_TEIF6))
	{
		Transfer_Error_i2c_dma_slave();
        return 0;
	}
    return 2;
}

uint8_t DMA_Body_RX_IRQHandler(void)
{
//	if(LL_DMA_IsActiveFlag_TC5(I2C_DMA_DMA))
	if (READ_BIT(I2C_DMA_DMA->ISR, DMA_ISR_TCIF7) == (DMA_ISR_TCIF7))
	{
		//LL_DMA_ClearFlag_GI5(I2C_DMA_DMA);
		WRITE_REG(I2C_DMA_DMA->IFCR, DMA_IFCR_CGIF7);
		Transfer_Complete_i2c_dma_slave();
        return 1;
	}
//	else if(LL_DMA_IsActiveFlag_TE5(I2C_DMA_DMA))
	else if (READ_BIT(I2C_DMA_DMA->ISR, DMA_ISR_TEIF7) == (DMA_ISR_TEIF7))
	{
		Transfer_Error_i2c_dma_slave();
        return 0;
	}
    return 2;
}
#endif
uint8_t Transfer_Complete_i2c_dma_slave()
{
	switch(Status_i2c_dma_slave)
	{
	case (WRITE):
        						/* передача данных по DMA окончена*/
		flagTransmitTransferComplete_i2c_dma_slave = 1;
        return 1;
	//break;
	case (READ):
        						/* DMA : приняты все данные */
		flagReceiveTransferComplete_i2c_dma_slave = 1;
		//getNewDataFromI2C_i2c_dma_slave();
    return 2;
		//break;
	case (NONE):
        						/* Непонятно что произошло! */
    return 0;
        //break;
	}
    return 0;
}
void Transfer_Error_i2c_dma_slave()
{
	/* Disable DMA1_Channel7_IRQn */
	NVIC_DisableIRQ(I2C_DMA_DMA_RECEIVE_IRQn);

	/* Disable DMA1_Channel6_IRQn */
	NVIC_DisableIRQ(I2C_DMA_DMA_TRANSMIT_IRQn);
}
void getNewDataFromI2C_i2c_dma_slave()
{
    /* Check data here */
	if(ptReceive_i2c_dma_slave[0] == 0x1F)
	{
		//printf("First: %d",ptReceive_i2c_dma_slave[0]);
		Word2transmit_i2c_dma_slave[0] = 't';
	}
	else
    {
        __NOP();
		//printf("None");
    }
}
uint8_t I2C_DMA_RXTX_EV_IRQHandler(void)
{
	if(READ_BIT(I2C_DMA_I2C->SR1, I2C_SR1_ADDR) == (I2C_SR1_ADDR))
	{
		switch(Mode_i2c_dma_slave)
		{
		case(RECEIVETRANSMIT):
			if(flagReceiveTransferComplete_i2c_dma_slave||flagTransmitTransferComplete_i2c_dma_slave)
			{
				switch((uint32_t)(READ_BIT(I2C_DMA_I2C->SR2, I2C_SR2_TRA)))
				{
				case (I2C_SR2_TRA):
						Transmit_Init_i2c_dma_slave();
						return 1;
				case (0x00000000U):
						Receive_Init_i2c_dma_slave();
						return 2;
				}
			}
		break;
		case (TRANSMIT):
			if ((((uint32_t)(READ_BIT(I2C_DMA_I2C->SR2, I2C_SR2_TRA))) == I2C_SR2_TRA)&&(flagTransmitTransferComplete_i2c_dma_slave))
			{
				Transmit_Init_i2c_dma_slave();
				return 1;
			}
		break;
		}
	}
	else if(READ_BIT(I2C_DMA_I2C->SR1, I2C_SR1_STOPF) == (I2C_SR1_STOPF))
	{
		//LL_I2C_ClearFlag_STOP(I2C_DMA_SLAVE);
		__IO uint32_t tmpreg;
		tmpreg = I2C_DMA_I2C->SR1;
		(void) tmpreg;
		SET_BIT(I2C_DMA_I2C->CR1, I2C_CR1_PE);
		Transfer_Complete_i2c_dma_slave();
		return 3;
	}
	return 0;
}
uint8_t I2C_DMA_Body_ER_IRQHandler(void)
{
//	__STATIC_INLINE uint32_t LL_I2C_IsActiveFlag_AF(I2C_TypeDef *I2Cx)
//{
//  return (READ_BIT(I2Cx->SR1, I2C_SR1_AF) == (I2C_SR1_AF));
//}
  /* Normal use case, if all bytes are sent and Acknowledge failure appears */
  /* This correspond to the end of communication */
//  if((ubSlaveNbDataToTransmit == 0) && \
//     (LL_I2C_IsActiveFlag_AF(I2C1)) && \
//     (LL_I2C_GetTransferDirection(I2C1) == LL_I2C_DIRECTION_WRITE))
//  {
//    /* Clear AF flag value in ISR register */
//    LL_I2C_ClearFlag_AF(I2C1);

//    /* Call function Slave Complete Callback */
//    Slave_Complete_Callback();
//  }
//  else
//  {
//    /* Call Error function */
//    Error_Callback();
//  }
//__STATIC_INLINE void LL_I2C_ClearFlag_AF(I2C_TypeDef *I2Cx)
//{
//  CLEAR_BIT(I2Cx->SR1, I2C_SR1_AF);
//}
//#define LL_I2C_DIRECTION_WRITE              I2C_SR2_TRA
	if((READ_BIT(I2C_DMA_I2C->SR1, I2C_SR1_AF) == (I2C_SR1_AF))&&	\
		((uint32_t)(READ_BIT(I2C_DMA_I2C->SR2, I2C_SR2_TRA)) == I2C_SR2_TRA))
	{
		CLEAR_BIT(I2C_DMA_I2C->SR1, I2C_SR1_AF);
		//acknoledgment
		//MODIFY_REG(I2C_DMA_I2C->CR1, I2C_CR1_ACK, I2C_CR1_ACK);
		return 1;
	}
	else{
		return 0;
	}
}
void I2C_DMA_Body_EV_IRQHandler(void)
{
	/* Check ADDR flag value in ISR register */
	//if(LL_I2C_IsActiveFlag_ADDR(I2C_DMA_SLAVE))
	if(READ_BIT(I2C_DMA_I2C->SR1, I2C_SR1_ADDR) == (I2C_SR1_ADDR))
	{
		switch(Mode_i2c_dma_slave)
		{
		case(RECEIVETRANSMIT):
			if(flagReceiveTransferComplete_i2c_dma_slave)
			{
				//switch(LL_I2C_GetTransferDirection(I2C_DMA_SLAVE))
				switch((uint32_t)(READ_BIT(I2C_DMA_I2C->SR2, I2C_SR2_TRA)))
				{
				case (I2C_SR2_TRA):
						Transmit_Init_i2c_dma_slave();
				break;
				//case (LL_I2C_DIRECTION_READ):
				case (0x00000000U):
						Receive_Init_i2c_dma_slave();
				break;
				}
			}
		break;
		case (TRANSMIT):
			//if ((LL_I2C_GetTransferDirection(I2C_DMA_SLAVE) == LL_I2C_DIRECTION_WRITE)&&(ubTransmitTransferComplete))
			if ((((uint32_t)(READ_BIT(I2C_DMA_I2C->SR2, I2C_SR2_TRA))) == I2C_SR2_TRA)&&(flagTransmitTransferComplete_i2c_dma_slave))
			{
				Transmit_Init_i2c_dma_slave();
			}
		break;
		}
	}
	//else if(LL_I2C_IsActiveFlag_STOP(I2C_DMA_SLAVE))
	else if(READ_BIT(I2C_DMA_I2C->SR1, I2C_SR1_STOPF) == (I2C_SR1_STOPF))
	{
		//LL_I2C_ClearFlag_STOP(I2C_DMA_SLAVE);
		__IO uint32_t tmpreg;
		tmpreg = I2C_DMA_I2C->SR1;
		(void) tmpreg;
		SET_BIT(I2C_DMA_I2C->CR1, I2C_CR1_PE);
		Transfer_Complete_i2c_dma_slave();
	}
}
void Block_TransmitInit_i2c_dma_slave(uint8_t mode)
{
	FlagTransmitBlocked = mode;
}

void Transmit_Init_i2c_dma_slave()
{
	if(!FlagTransmitBlocked) return;
    #ifdef EXACTO_HAL
    //	if(LL_DMA_IsEnabledChannel(I2C_DMA_DMA,I2C_DMA_DMA_RECEIVE)){
	if(READ_BIT(DMA1_Channel5->CCR, DMA_CCR_EN) == (DMA_CCR_EN))
	{
		//LL_DMA_DisableChannel(I2C_DMA_DMA, I2C_DMA_DMA_RECEIVE);
		CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
        //pTransmitBufferDMA    = (uint32_t*)(&word2transmit);
        //ubNbTBDataToTransmit  = strlen((char *)pTransmitBufferDMA);
        ptTransmit_i2c_dma_slave = (uint32_t*)(&pTransmitBuffer_i2c_dma_slave);
		//LL_DMA_SetDataLength(I2C_DMA_DMA, I2C_DMA_DMA_TRANSMIT, ubNbTBDataToTransmit);
		MODIFY_REG(DMA1_Channel4->CNDTR,  DMA_CNDTR_NDT, uCountTransmit_i2c_dma_slave);
		//LL_DMA_EnableChannel(I2C_DMA_DMA, I2C_DMA_DMA_TRANSMIT);
		SET_BIT(DMA1_Channel4->CCR, DMA_CCR_EN);
	}
    #endif
    #ifdef EXACTO_SPL
	if(READ_BIT(I2C_DMA_DMA_CHANNEL_RX->CCR, DMA_CCR1_EN) == (DMA_CCR1_EN))
	{
		CLEAR_BIT(I2C_DMA_DMA_CHANNEL_RX->CCR, DMA_CCR1_EN);  
	}else if (READ_BIT(I2C_DMA_DMA_CHANNEL_TX->CCR, DMA_CCR1_EN) == (DMA_CCR1_EN))
	{
		CLEAR_BIT(I2C_DMA_DMA_CHANNEL_TX->CCR, DMA_CCR1_EN);
	}
	else return;
	ptTransmit_i2c_dma_slave = (uint32_t*)(&pTransmitBuffer_i2c_dma_slave);
	MODIFY_REG(I2C_DMA_DMA_CHANNEL_TX->CNDTR,  DMA_CNDTR1_NDT, uCountTransmit_i2c_dma_slave);
	SET_BIT(I2C_DMA_DMA_CHANNEL_TX->CCR, DMA_CCR1_EN);
	
    #endif
	
	Status_i2c_dma_slave = WRITE;
	//LL_I2C_EnableDMAReq_TX(I2C_DMA_SLAVE);
	SET_BIT(I2C_DMA_I2C->CR2, I2C_CR2_DMAEN);
	//LL_I2C_ClearFlag_ADDR(I2C_DMA_SLAVE);
	__IO uint32_t tmpreg;
	tmpreg = I2C_DMA_I2C->SR1;
	(void) tmpreg;
	tmpreg = I2C_DMA_I2C->SR2;
	(void) tmpreg;
	flagTransmitTransferComplete_i2c_dma_slave = 0;
}
void Receive_Init_i2c_dma_slave()
{
#ifdef EXACTO_HAL
        //if(LL_DMA_IsEnabledChannel( I2C_DMA_DMA,  I2C_DMA_DMA_TRANSMIT))
    if	(READ_BIT(DMA1_Channel4->CCR, DMA_CCR_EN) == (DMA_CCR_EN))
    {
        //LL_DMA_DisableChannel(  I2C_DMA_DMA,  I2C_DMA_DMA_TRANSMIT);
        CLEAR_BIT(DMA1_Channel4->CCR, DMA_CCR_EN);
        //LL_DMA_SetDataLength(   I2C_DMA_DMA,  I2C_DMA_DMA_RECEIVE, ubNbaReceiveBufferDMA);
        MODIFY_REG(DMA1_Channel5->CNDTR,  DMA_CNDTR_NDT, uCountReceive_i2c_dma_slave);
        //LL_DMA_EnableChannel(   I2C_DMA_DMA,  I2C_DMA_DMA_RECEIVE);
        SET_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
	}
    else{
        //LL_DMA_DisableChannel(  I2C_DMA_DMA,  I2C_DMA_DMA_RECEIVE);
        CLEAR_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
        //LL_DMA_SetDataLength(   I2C_DMA_DMA,  I2C_DMA_DMA_RECEIVE, ubNbaReceiveBufferDMA);
        MODIFY_REG(DMA1_Channel5->CNDTR,  DMA_CNDTR_NDT, uCountReceive_i2c_dma_slave);
        //LL_DMA_EnableChannel(   I2C_DMA_DMA,  I2C_DMA_DMA_RECEIVE);
        SET_BIT(DMA1_Channel5->CCR, DMA_CCR_EN);
    }
#endif
#ifdef EXACTO_SPL
    if	(READ_BIT(I2C_DMA_DMA_CHANNEL_TX->CCR, DMA_CCR1_EN) == (DMA_CCR1_EN))
    {
        CLEAR_BIT(I2C_DMA_DMA_CHANNEL_TX->CCR, DMA_CCR1_EN);
        MODIFY_REG(I2C_DMA_DMA_CHANNEL_RX->CNDTR,  DMA_CNDTR1_NDT, uCountReceive_i2c_dma_slave);
        SET_BIT(I2C_DMA_DMA_CHANNEL_RX->CCR, DMA_CCR1_EN);
	}
    else{
        CLEAR_BIT(I2C_DMA_DMA_CHANNEL_RX->CCR, DMA_CCR1_EN);
        MODIFY_REG(I2C_DMA_DMA_CHANNEL_RX->CNDTR,  DMA_CNDTR1_NDT, uCountReceive_i2c_dma_slave);
        SET_BIT(I2C_DMA_DMA_CHANNEL_RX->CCR, DMA_CCR1_EN);
    }
#endif

	Status_i2c_dma_slave = READ;
	if(uCountReceive_i2c_dma_slave == 1){
		/* Prepare the generation of a Non ACKnowledge condition after next received byte */
		//LL_I2C_AcknowledgeNextData(I2C_DMA_SLAVE, LL_I2C_NACK);
		MODIFY_REG(I2C_DMA_I2C->CR1, I2C_CR1_ACK, 0x00000000U);
		/* Enable DMA transmission requests */
		//LL_I2C_EnableDMAReq_RX(I2C_DMA_SLAVE);
		SET_BIT(I2C_DMA_I2C->CR2, I2C_CR2_DMAEN);
	}
	else if(uCountReceive_i2c_dma_slave == 2){
		/* Prepare the generation of a Non ACKnowledge condition after next received byte */
		//LL_I2C_AcknowledgeNextData(I2C_DMA_SLAVE, LL_I2C_NACK);
		MODIFY_REG(I2C_DMA_I2C->CR1, I2C_CR1_ACK, 0x00000000U);
		/* Enable Pos */
		//LL_I2C_EnableBitPOS(I2C_DMA_SLAVE);
		SET_BIT(I2C_DMA_I2C->CR1, I2C_CR1_POS);
	}
	else{
		/* Enable Last DMA bit */
		//LL_I2C_EnableLastDMA(I2C_DMA_SLAVE);
		SET_BIT(I2C_DMA_I2C->CR2, I2C_CR2_LAST);
		/* Enable DMA transmission requests */
		//LL_I2C_EnableDMAReq_RX(I2C_DMA_SLAVE);
		SET_BIT(I2C_DMA_I2C->CR2, I2C_CR2_DMAEN);
	}
	//LL_I2C_ClearFlag_ADDR(I2C_DMA_SLAVE);
	  __IO uint32_t tmpreg;
	  tmpreg = I2C_DMA_I2C->SR1;
	  (void) tmpreg;
	  tmpreg = I2C_DMA_I2C->SR2;
	  (void) tmpreg;
	flagReceiveTransferComplete_i2c_dma_slave = 0;
}
void SetData2word2Transmit_i2c_dma_slave(uint8_t *pData)
{
    uint32_t index = 0;
    while((index < (MAXNBWORD2TRANSMIT - 1))&&(*pData))
    {
        pTransmitBuffer_i2c_dma_slave[index] = *pData;
        pData++;
        index++;
    }
    //pWord2Transmit[index] = '\0';
    uCountTransmit_i2c_dma_slave = index;
}
uint8_t SetDT2W2TR_fixlen_i2c_dma_slave(uint8_t * pData, const uint32_t datalen)
{
	uint32_t count = datalen;
	if(MAXNBWORD2TRANSMIT < count)	count = MAXNBWORD2TRANSMIT;
	for(uint32_t i = 0; i < count; i ++)
	{
		pTransmitBuffer_i2c_dma_slave[i] = pData[i];
	}
	uCountTransmit_i2c_dma_slave = count;
	return 1;
}
uint32_t GetMXW2TR_i2c_dma_slave(void)
{
	return MAXNBWORD2TRANSMIT;
}
uint8_t SetValue2W2TR_i2c_dma_slave(const uint8_t value)
{
	if(uCountTransmit_i2c_dma_slave == MAXNBWORD2TRANSMIT) return 0;
	pTransmitBuffer_i2c_dma_slave[uCountTransmit_i2c_dma_slave] = value;
	uCountTransmit_i2c_dma_slave++;
	return 1;
}
uint32_t GetCurW2TR_i2c_dma_slave(void)
{
	return uCountTransmit_i2c_dma_slave;
}
uint8_t GetReceiveBufferValue(const uint8_t index)
{
	return ptReceive_i2c_dma_slave[index];
}

