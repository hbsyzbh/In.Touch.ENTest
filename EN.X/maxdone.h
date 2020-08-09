/* 
 * File:   maxdone.h
 * Author: Administrator
 *
 * Created on August 8, 2020, 11:53 AM
 */

#ifndef MAXDONE_H
#define	MAXDONE_H

#include <stdbool.h>

#ifdef	__cplusplus
extern "C" {
#endif

void UART2_Sendstr(char *str);
void doUartTask(void);
void doKeyTask(void);

bool I2C1_MasterOpen(void);

extern void on_uart1_error(void);
extern void on_uart1_receiveend(void);
extern unsigned char eusart1Want;
extern unsigned char * eusart1Data;

extern unsigned char ethInited;

void drv_initMRF89XA(void);
void drv_testMRF89XA(void);

#ifdef	__cplusplus
}
#endif

#endif	/* MAXDONE_H */

