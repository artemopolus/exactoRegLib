#ifndef EXACTO_DEFS_H_
#define EXACTO_DEFS_H_

#define ENABLE_FIFO_BUFFER

#define FLG_LSM303  (INT8U) 0x01
#define FLG_BMP280  (INT8U) 0x02
#define FLG_ISM330  (INT8U) 0x04

#define OS_TIME_10mS 	(INT16U)((INT32U)OS_TICKS_PER_SEC * 10L / 1000L)       //100Hz
#define OS_TIME_09m6S (INT16U) 96
#define OS_TIME_100mS (INT16U)((INT32U)OS_TICKS_PER_SEC * 100L / 1000L)     //10 Hz
#define OS_TIME_500mS (INT16U)((INT32U)OS_TICKS_PER_SEC * 500L / 1000L)     //2 Hz
#define OS_TIME_2S 		(INT16U)(OS_TICKS_PER_SEC * 2)
#define OS_TIME_1mS 	(INT16U)((INT32U)OS_TICKS_PER_SEC * 1L / 1000L)       //1000Hz

#endif
