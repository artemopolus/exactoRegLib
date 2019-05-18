#ifndef EXACTO_STRUCT_H_
#define EXACTO_STRUCT_H_

#include <stdint.h> 

typedef struct {
  uint16_t       TDiscr;     
  char Name[6];
    uint8_t Whoami;
    uint8_t flgSens;  
} ExactoSensorSet;

typedef struct{
    uint8_t pSensor;
    uint8_t s1[6];
		uint8_t s2[6];
}SensorData;

typedef struct{
    uint8_t adr;
    uint8_t val;
    uint8_t actiontype;
}CmdToStm32;

typedef enum{
    READSENSMODE_ES32A = 0,
    SENDFREQ_ES32A = 1,
    SET_LSM303 = 2,
    SET_BMP280 = 3,
    SET_ISM330 = 4
} ExactoStm32Adr;

typedef enum{
	ALLWAITING_ESM = 0,
	ONLYLSM303_ESM = 1,
	ALLRUNNING_ESM = 2
}ExactoSensMode;

#define	EXACTO_FREQ_100HZ  0x02U
#define	EXACTO_FREQ_10HZ 	 0x01U
#define	EXACTO_FREQ_1HZ 	 0x00U


#define EXACTOLBIDATASIZE		36
typedef struct{
    uint8_t lsm303[EXACTOLBIDATASIZE];
    uint8_t bmp280[EXACTOLBIDATASIZE];
    uint8_t ism330[EXACTOLBIDATASIZE];
    uint8_t cnt_lsm303;
    uint8_t cnt_bmp280;
    uint8_t cnt_ism330;
}ExactoLBIdata;

#endif
