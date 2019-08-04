#ifndef EXACTO_DEFS_H_
#define EXACTO_DEFS_H_

//#define ENABLE_FIFO_BUFFER
//#define ENABLE_SAFE_CP2BUFFER
//#define ENABLE_CRITSEC_CP2BUFFER
//#define ENABLE_TEST_MSG
//#define I2C_SEND_MSG
#define UART_SEND_MSG


#define ECHO_ALL
//#define PING_MODE
//#define FAKE_ISM330
//#define FAKE_BMP280
//#define FAKE_LSM303

#define ENABLE_LSM303_XL
#define ENABLE_LSM303_M
#define ENABLE_LSM303_SEND
#define ENABLE_LSM303_SAVE

#define ENABLE_TIME_MEAS

#define ENABLE_ISM330_T
#define ENABLE_ISM330_XL
#define ENABLE_ISM330_G

#define ENABLE_I2C_SLAVE
//#define ENABLE_I2C_PENDING

#define EN_TASK_ISM330
#define EN_TASK_BMP280
#define EN_TASK_LSM303

#define FLG_LSM303  (INT8U) 0x01
#define FLG_BMP280  (INT8U) 0x02
#define FLG_ISM330  (INT8U) 0x04
#define FLG_UART		(INT8U) 0x10
#define FLG_I2C			(INT8U) 0x20
#define FLG_TEST		(INT8U)	0x80

#define OS_TIME_1m2S   (INT16U) 12
#define OS_TIME_10mS 	(INT16U)((INT32U)OS_TICKS_PER_SEC * 10L / 1000L)       //100Hz
#define OS_TIME_09m6S (INT16U) 96
#define OS_TIME_100mS (INT16U)((INT32U)OS_TICKS_PER_SEC * 100L / 1000L)     //10 Hz
#define OS_TIME_500mS (INT16U)((INT32U)OS_TICKS_PER_SEC * 500L / 1000L)     //2 Hz
#define OS_TIME_2S 		(INT16U)(OS_TICKS_PER_SEC * 2)
#define OS_TIME_1mS 	(INT16U)((INT32U)OS_TICKS_PER_SEC * 1L / 1000L)       //1000Hz

#endif
