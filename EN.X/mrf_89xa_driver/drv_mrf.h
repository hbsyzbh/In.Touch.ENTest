////#ifndef __DRV_MRF_H
////    #define __DRV_MRF_H
////
/////*******************************************************************************
////Microchip MRF24J40 radio utility driver -drv_mrf file.
////    
////Company: Microchip Technology Inc.
////
////File Description:
////
////FileName:       drv_mrf.h
////Dependencies:   None
////Processor:      PIC18 Microcontroller/PIC24 Microcontroller
////Hardware:       The code is natively intended to be used on the following hardware platforms:
////                PIC18 Explorer Development Board/Explorer 16 Development Board
////                MRF24J40 PICtail/PICtail Plus boards
////Firmware:       version 4.00 and above
////Compiler:       Microchip XC8 V1.40/ XC16 V1.30
////IDE:            Microchip MPLAB X V3.50
//// 
////Summary:
////                Microchip MRF24J40 radio utility driver -drv_mrf file.
////                This file is supported for MRF24J40 modules
////                with firmware version 4.00(and above).
////
////*******************************************************************************/
////
/////********************************************************************
////Change History:
////Rev   Date         Description
////1.0   18/08/2016   Initial release
////********************************************************************/
////
////// DOM-IGNORE-BEGIN
/////*******************************************************************************
////Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.
////
////Microchip licenses to you the right to use, modify, copy and distribute
////Software only when embedded on a Microchip microcontroller or digital signal
////controller that is integrated into your product or third party product
////(pursuant to the sublicense terms in the accompanying license agreement).
////
////You should refer to the license agreement accompanying this Software for
////additional information regarding your rights and obligations.
////
////SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
////EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
////MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
////IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
////CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
////OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
////INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
////CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
////SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
////(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
////*******************************************************************************/
////// DOM-IGNORE-END
////
////
////
////    #include "system.h"
////    #include "system_config.h"
////
////    #include "driver/mrf_24j40_driver/drv_mrf_24j40.h"
////    
////
////    #define CHANNEL_ASSESSMENT_CARRIER_SENSE    0x00
////    #define CHANNEL_ASSESSMENT_ENERGY_DETECT    0x01
////    
////    #define POWER_STATE_DEEP_SLEEP              0x00
////    #define POWER_STATE_OPERATE                 0xFF
////
////
////    #define PACKET_TYPE_MASK        0x03
////    #define BROADCAST_MASK          0x04
////    #define SECURITY_MASK           0x08
////    #define REPEAT_MASK             0x10
////    #define ACK_MASK                0x20
////    #define DSTPRSNT_MASK           0x40
////    #define SRCPRSNT_MASK           0x80
////    
////    #define PACKET_TYPE_DATA        0x00
////    #define PACKET_TYPE_COMMAND     0x01
////    #define PACKET_TYPE_ACK         0x02
////    #define PACKET_TYPE_RESERVE     0x03
////
////
////    #define IEEE_802_15_4
////
////
////    // DOM-IGNORE-BEGIN
////    /*********************************************************************
////     Overview: Data types for drivers. This will facilitate easy
////               access smaller chunks of larger data types when sending
////               or receiving data (for example byte sized send/receive
////               over parallel 8-bit interface.
////    *********************************************************************/
////    // DOM-IGNORE-END
////    
////
////    typedef union _DRIVER_UINT8_UNION_
////    {
////        uint8_t Val;
////        struct
////        {
////        uint8_t b0:1;
////        uint8_t b1:1;
////        uint8_t b2:1;
////        uint8_t b3:1;
////        uint8_t b4:1;
////        uint8_t b5:1;
////        uint8_t b6:1;
////        uint8_t b7:1;
////        } bits;
////        
////    } DRIVER_UINT8_UNION;
////
////
////    #if defined(IEEE_802_15_4)
////        #undef MY_ADDRESS_LENGTH
////        #define MY_ADDRESS_LENGTH       8 
////    #endif
////    
////    void UserInterruptHandler(void);
////
////#endif
////
#include "../mcc_generated_files/pin_manager.h"
#include "../mcc_generated_files/ext_int.h"
#define PACKET_LEN 64
#define PAYLOAD_LEN 64

//declare MRF89XA function prototypes
typedef enum _BOOL { FALSE = 0, TRUE } BOOL;
typedef unsigned char		BYTE;				// 8-bit
typedef unsigned short int	WORD;				// 16-bit
typedef unsigned long		DWORD;				// 32-bit

void RegisterSet(BYTE address, BYTE value);			//Programs the MRF89XA register address with passing value
BYTE RegisterRead(BYTE address);					//returns register value (for that register address)
void Send_Packet(BYTE packetlen);					//Transmit packet of size packetlen
void ReceiveFrame(void);								//Receive Data packet
void WriteFIFO(BYTE);								//Write to FIFO (64 byte) or depending on configuration parameters
BYTE ReadFIFO(void);								//Read from FIFO (64 byte)
void SetRFMode(BYTE);								//Program RF mode


extern BOOL PHY_IRQ1_En;
#define Config_nCS      IO_RB0_LAT
#define Data_nCS        IO_RA5_LAT

#define SPI_SDI      IO_RB3_LAT
#define SPI_SDO      IO_RB4_PORT
#define SPI_SCK      IO_RB5_LAT

void SPIPut(unsigned char);
unsigned char SPIGet(void);