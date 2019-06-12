#ifndef EXACTO_STRUCT_H_
#define EXACTO_STRUCT_H_

#include <stdint.h> 

typedef struct {
       
  char Name[6];
    uint8_t Whoami;
    uint8_t flgSens; 
		uint8_t initFreq;
	uint16_t       TDiscr;
} ExactoSensorSet;

typedef struct{
	uint8_t pSensor;
    uint8_t s1_status;
	uint8_t s1[6];
    uint8_t s2_status;
	uint8_t s2[6];
    uint8_t sL_status;
	uint8_t sL[14];
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
	ONLYBMP280_ESM = 2,
	ONLYISM330_ESM = 3,
	ALLRUNNING_ESM = 7,
	CNTLSM303_ESM = 9
}ExactoSensMode;

#define	EXACTO_FREQ_100HZ  0x02U
#define	EXACTO_FREQ_10HZ 	 0x01U
#define	EXACTO_FREQ_1HZ 	 0x00U


#define EXACTOLBIDATASIZE		512
#define EXACTOLSM303SZ		512
#define EXACTOBMP280SZ		512
#define EXACTOISM330SZ		512
typedef struct{
    uint8_t lsm303[EXACTOLSM303SZ];
    uint8_t bmp280[EXACTOBMP280SZ];
    uint8_t ism330[EXACTOISM330SZ];
    uint32_t cnt_lsm303;
    uint32_t cnt_bmp280;
    uint32_t cnt_ism330;
}ExactoLBIdata;

#endif
