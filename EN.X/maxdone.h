/* 
 * File:   maxdone.h
 * Author: Administrator
 *
 * Created on August 8, 2020, 11:53 AM
 */

#ifndef MAXDONE_H
#define	MAXDONE_H

#ifdef	__cplusplus
extern "C" {
#endif

void UART2_Sendstr(char *str);
void doUartTask(void);
extern void on_uart1_error(void);
extern void on_uart1_receiveend(void);
extern unsigned char eusart1Want;
extern unsigned char * eusart1Data;

#ifdef	__cplusplus
}
#endif

#endif	/* MAXDONE_H */
