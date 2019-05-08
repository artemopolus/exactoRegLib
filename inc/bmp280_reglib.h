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
uint8_t GetPresTempValuesUint8_bmp280(uint8_t * data);
uint8_t  GetPresTempValues_bmp280(int32_t * pres, int32_t * temp);

uint8_t read_bmp280(uint8_t address);
void multiread_bmp280(uint8_t address, uint8_t * values, uint8_t cnt);
void write_bmp280(uint8_t address, uint8_t value);



void check_I2C_SB_bmp280(void);
void check_I2C_ADDR_bmp280(void);
void check_I2C_RXNE_bmp280(void);
void clearFlag_I2C_ADDR_bmp280(void);
void check_I2C_TXE_bmp280(void);
uint8_t readI2C_bmp280(uint8_t address, uint8_t reg);
void multireadI2C_bmp280(uint8_t address, uint8_t reg, uint8_t * values, uint8_t cnt);
void writeI2C_bmp280(uint8_t address, uint8_t reg,uint8_t value);

#endif /* EXACTOREGLIB_INC_BMP280_REGLIB_H_ */
