/**
  ETH Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    eth.c

  @Summary
    This is the generated driver implementation file for the ETH driver using MPLAB(c) Code Configurator

  @Description
    This header file provides implementations for driver APIs for ETH.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.85.0
        Device            :  PIC18F67J60
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/
#include "eth.h"

/**
  Section: ETH APIs
*/

void ETH_Initialize(void)
{
    // Set the ETH module to the options selected in the user interface.

    // RXRST Normal; TXRTS Transmit; TXRST Normal; RXEN Receive; DMAST Idle; CSUMEN Disabled; 
    ECON1 = 0x00;

    // ETHEN Disabled; PKTDEC Unchanged; AUTOINC Do not automatically change ERDPT; 
    ECON2 = 0x00;

    // PHYRDY Not ready; BUFER No error; TXABRT Transmit ok; 
    ESTAT = 0x00;

    // PASSALL Control frames discarded; TXPAUS Disallow; MARXEN Disable; RXPAUS Ignore; 
    MACON1 = 0x00;

    // PHDREN Disable; TXCRCEN Disable; PADCFG No-padding; FULDPX Half-Duplex; FRMLNEN Disable; HFRMEN Disable; 
    MACON3 = 0xC0;

    // DEFER Disable; 
    MACON4 = 0x00;

    // MIIRD Disable; MIISCAN Disable; 
    MICMD = 0x00;

    // TXERIE Disable; TXERIE Disable; TXIE Disable; RXERIE Disable; LINKIE Disable; PKTIE Disable; 
    EIE = 0x00;

    // TXERIF No error; RXERIF No error; TXIF No error; TXERIF No error; 
    EIR = 0x00;

    // 
    MABBIPG = 0x00;

}


/**
  End of File
*/
