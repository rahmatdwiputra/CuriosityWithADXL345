/* 
 * File:   accel.h
 * Author: rputra
 *
 * Created on 21 February 2018, 1:51 PM
 */

#ifndef ACCEL_H
#define	ACCEL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <xc.h>
#include "pin_manager.h"

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

#define MMA8491Q_RETRY_MAX  100  // define the retry count
#define MMA8491Q_ADDRESS    0x55 // slave device address
#define MMA8491Q_STATUS     0x00  
#define MMA8491Q_OUTX_MSB   0x01
#define MMA8491Q_OUTX_LSB   0x02
#define MMA8491Q_OUTY_MSB   0x03
#define MMA8491Q_OUTY_LSB   0x04
#define MMA8491Q_OUTZ_MSB   0x05
#define MMA8491Q_OUTZ_LSB   0x06
#define AVERAGE   10


char AccelReadStatus(char SlaveAdd, char DataAdd);
char AccelReadData(char SlaveAdd, char DataAdd, char *DataMSB, char *DataLSB);
char AccelReadDataAll(char SlaveAdd, char DataAdd, char *DataXMSB, char *DataXLSB,char *DataYMSB, char *DataYLSB,char *DataZMSB, char *DataZLSB);
int16_t AccelConversion(char dataMSB, char dataLSB);

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif	/* ACCEL_H */

