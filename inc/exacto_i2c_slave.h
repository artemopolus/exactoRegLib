/*
 * exacto_i2c_slave.h
 *
 *  Created on: 22 ���. 2019 �.
 *      Author: Artem
 */

#ifndef EXACTOREGLIB_INC_EXACTO_I2C_SLAVE_H_
#define EXACTOREGLIB_INC_EXACTO_I2C_SLAVE_H_

#include "exactoBase_driver.h"

void ConfigurePins_i2c_dma_slave(void);
void ConfigureMode_i2c_dma_slave(void);
void Activate2work_i2c_dma_slave(void);
void Handle_i2c_dma_slave(void);

void Transfer_Complete_i2c_dma_slave(void);
void Transfer_Error_i2c_dma_slave(void);
void getNewDataFromI2C_i2c_dma_slave(void);


void SetData2word2Transmit_i2c_dma_slave(uint8_t *pData);

void DMA1_Channel4_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);

void I2C1_EV_IRQHandler(void);

void Transmit_Init_i2c_dma_slave(void);
void Receive_Init_i2c_dma_slave(void);


#endif /* EXACTOREGLIB_INC_EXACTO_I2C_SLAVE_H_ */
