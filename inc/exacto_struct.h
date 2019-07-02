#ifndef EXACTO_STRUCT_H_
#define EXACTO_STRUCT_H_

#include <stdint.h> 

typedef struct {
       
  char Name[6];
  uint8_t Whoami;
  uint8_t flgSens; 
	uint8_t initFreq;
	uint16_t TDiscr;
	uint8_t MultSens1;
	uint8_t MultSens2;
	uint8_t MultSens3;
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
    SET_ISM330 = 4,
    SET_SNSFRQ_MODE = 5
} ExactoStm32Adr;

typedef enum{
	ALLWAITING_ESM = 0,
	ONLYLSM303_ESM = 1,
	ONLYBMP280_ESM = 2,
	ONLYISM330_ESM = 3,
	ALLRUNNING_ESM = 7,
	CNTLSM303_ESM = 9,
	DISABLE_UART_ESM = 10,
	DISABLE_I2C_ESM = 11,
	ENABLE_TEST_ESM = 12
}ExactoSensMode;

#define	EXACTO_FREQ_100HZ  0x02U
#define	EXACTO_FREQ_10HZ 	 0x01U
#define	EXACTO_FREQ_1HZ 	 0x00U

#define EXACTOLBIMAXTRY 100

#define EXACTOLSM303SZ		500

#define EXACTOISM330SZ		512



#define EXACTOLSM303SZ_XL		(EXACTOLBIMAXTRY*6)
#define EXACTOLSM303SZ_M		(EXACTOLBIMAXTRY*6)

#define EXACTOBMP280SZ		    12

#define EXACTOISM330SZ_T		12
#define EXACTOISM330SZ_G		(EXACTOLBIMAXTRY*6)
#define EXACTOISM330SZ_XL		(EXACTOLBIMAXTRY*6)

#define EXACTOLBIDATASIZE		(EXACTOLSM303SZ_XL + EXACTOLSM303SZ_M + EXACTOBMP280SZ + EXACTOISM330SZ_T + EXACTOISM330SZ_G + EXACTOISM330SZ_XL)



#define EXACTOLBIARRAYCNT	9

#define MAXNBWORD2TRANSMIT  380

typedef struct{
    uint8_t lsm303_xl	[EXACTOLSM303SZ_XL];
	uint8_t lsm303_m	[EXACTOLSM303SZ_M];
    uint8_t bmp280		[EXACTOBMP280SZ];
    uint8_t ism330_t	[EXACTOISM330SZ_T];
	uint8_t ism330_g	[EXACTOISM330SZ_G];
	uint8_t ism330_xl	[EXACTOISM330SZ_XL];
    uint16_t    cnt_lsm303_xl;
	uint8_t     cnt_lsm303_m;
    uint8_t     cnt_bmp280;
    uint8_t     cnt_ism330_t;
	uint16_t    cnt_ism330_g;
	uint16_t    cnt_ism330_xl;
}ExactoLBIdata;

#endif
