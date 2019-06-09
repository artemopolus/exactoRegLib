/*
 * exacto_i2c_slave.h
 *
 *  Created on: 22 апр. 2019 г.
 *      Author: Artem
 */

#ifndef EXACTOREGLIB_INC_EXACTO_I2C_SLAVE_H_
#define EXACTOREGLIB_INC_EXACTO_I2C_SLAVE_H_

#include "exactoBase_driver.h"

void ConfigurePins_i2c_dma_slave(void);
void ConfigureMode_i2c_dma_slave(void);
void Activate2work_i2c_dma_slave(void);
void Handle_i2c_dma_slave(void);

uint8_t Transfer_Complete_i2c_dma_slave(void);
void Transfer_Error_i2c_dma_slave(void);
void getNewDataFromI2C_i2c_dma_slave(void);


void SetData2word2Transmit_i2c_dma_slave(uint8_t *pData);

uint8_t DMA_Body_TX_IRQHandler(void);
uint8_t DMA_Body_RX_IRQHandler(void);

void I2C_DMA_Body_EV_IRQHandler(void);

void Transmit_Init_i2c_dma_slave(void);
void Receive_Init_i2c_dma_slave(void);


#endif /* EXACTOREGLIB_INC_EXACTO_I2C_SLAVE_H_ */
