/*******************************************************************************
Microchip MRF89XA radio utility driver -main file.
    
Company: Microchip Technology Inc.

File Description:

FileName:       main.c
Dependencies:   None
Processor:      PIC18 Microcontroller/PIC24 Microcontroller
Hardware:       The code is natively intended to be used on the following hardware platforms:
                PIC18 Explorer Development Board/Explorer 16 Development Board
                MRF89XA PICtail/PICtail Plus boards
Firmware:       version 2.00 and above
Compiler:       Microchip XC8 V1.40/ XC16 V1.30
IDE:            Microchip MPLAB X V3.50
 
Summary:
                Microchip MRF89XA radio utility driver -main file.
                This file is supported for MRF89XA modules
                with firmware version 2.00(and above).

*******************************************************************************/

/********************************************************************
Change History:
Rev   Date         Description
1.0   18/08/2016   Initial release
********************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// DOM-IGNORE-END

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>

#include "drv_mrf.h"
#include "drv_mrf_89xa.h"
#include "../maxdone.h"

BOOL PHY_IRQ1_En;

BOOL IRQ0_Received;

static void SPI_delay(void)
{
    unsigned char delay = 100;
    
    while(delay--);
}

/*MRF89XA ??SPI ??0,0??????SCK ???????????
 * CS ????????SDI ?????17?????????SCK ?????18????????????
 * SDO ?????????SCK ?????????????????????????MostSignificant bit? MSb?
 */

void SPIPut(unsigned char data)
{
    int i;
    
    SPI_SCK = 0;
    SPI_delay();
    for(i = 7; i >= 0; i--)
    {
       SPI_SDO = ((data & (0x01 << i)) != 0);
       SPI_delay();
       SPI_SCK = 1;
       SPI_delay();
       SPI_SCK = 0;
    }
}

unsigned char SPIGet(void)
{
    unsigned char result = 0;
    int i;
    
    SPI_SCK = 0;
    SPI_delay();
    for(i = 7; i >= 0; i--)
    {
       if(SPI_SDI) {
           result &= (0x01 << i);
       }               
       SPI_SCK = 1;
       SPI_delay();
       SPI_SCK = 0;
       SPI_delay();
    }
    
    return result;
}
        
//Defines for RadioDriver.c file
#define PR_RX_MODE	0x00
#define PR_TX_MODE	0x01
#define PR_TX_RX_MODE	0x02

//variables
//initialized variable
BYTE RF_Mode = RF_STANDBY;
char PredefinedPacket[] = {0x01,0x08,0xC4,0xFF,0xFF,0xFF,0xFF,0x07,0x01,0x00,0x01,0x00,0x04,0x00,0x00,0x00};
char SynchronPacket[] = {0xFF,0xFF,0x00,0x00,0xFF,0x00,0x00,0xFF,0xFF,0x00,0x00,0xFF};
BYTE RandomBuffer[100];
BYTE PacketDelay = 1;
float Fxtal = 12800;			//crystal frequency 12.8 MHz
WORD RxCount = 0;				//Used for counting Rx packets received
BYTE RxPrintCount;				//Used for printing header for the displayed data
BYTE TX_FREQ_BW = FC_125;
BYTE RSSIRegVal = 0;			//For reading RSSI

//variables for Ping-Pong mode
WORD PingPong_Package = 100;	//Ping Pong Package size
WORD PingPong_Count = 0;		//Ping Pong package current count
BOOL PingPongSend = FALSE;
BOOL PingPongReceive = FALSE;
BOOL PingPongTest = FALSE;

//variables for PER test mode
BYTE PER_Packet_Size = 16;		//PER test packet size (choose from options - 16, 32, 64)
WORD PER_Count = 0;				//PER received packet counter
BOOL Synchron = FALSE;			//remove me
BOOL PERStatus = FALSE;
BOOL PERSend = FALSE;
BOOL PERReceive = FALSE;
BOOL PERTest = FALSE;
WORD PERMaxCount = 100;			//PER package count(choose from 10 or 100 or 1000) using this variable. Not an option from the program

//FHSS mode
BOOL FHSS_Enable_902 = FALSE;
BOOL FHSS_Enable_863 = FALSE;
BOOL FHSS_Master_Enabled = FALSE;
BOOL FHSS_Slave_Enabled = FALSE;
BOOL Channel_spacing_100 = FALSE;
BOOL Channel_spacing_50 = FALSE;
BOOL Channel_spacing_25 = FALSE;
BOOL FHSS_Sync_Detect = FALSE;
BOOL FHSS_Hop = FALSE;
extern BYTE Current_Channel;

//Duty Cycle
BOOL Enable_DutyCycle = FALSE;
WORD Duty_Cycle = 0x0000;

//LED status
BOOL LED_1_SET = FALSE;
BOOL LED_2_SET = FALSE;

//Interrupt flags
BYTE RX_STBY_IRQ1_SRC = IRQ1_RX_STDBY_CRCOK;
BYTE RX_STBY_IRQ0_SRC = IRQ0_RX_STDBY_FIFOEMPTY;
BYTE TX_IRQ1_SRC = IRQ1_TX_TXDONE;
BYTE TX_START_IRQ0_SRC = IRQ0_TX_START_FIFOTHRESH;

//receive flags
BOOL hasPacket = FALSE;			//indicates that data packet is available in RX buffer


//uninitialized variable - MRF89XA variables
WORD Rate_R;
WORD Rate_C;		//Data Rate
BYTE FREQ_BW;		//Frequenncy bandwidth - Receiver
BYTE FREQ_BAND;		//Frequency band - (902-915 / 915-928 / 950-960 or 863-870)
BYTE RVALUE;		//Register R value
BYTE PVALUE;		//Register P value
BYTE SVALUE;		//Register S value
BYTE TX_FSK;		//Frequency deviation register setting
BYTE IF_GAIN;		//IF filter gain register setting
BYTE TX_PWR;		//TX power register setting
BYTE PFILTER_SETTING;	//SIDEBAND FILTER SETTING
BOOL DATA_Whitening = FALSE;	//By default Data Whitening is disabled
BOOL Modulation_Type = TRUE;	//Implies FSK modulation ele OOK modulation
BOOL Data_out = 0;
BYTE Data_Mode = DATAMODE_PACKET;

//remove me ------
BYTE FREQ_BAND_RESET;
BYTE RVALUE_RESET;
BYTE PVALUE_RESET;
BYTE SVALUE_RESET;

//external variables
extern char InitConfigRegs[];	//Initial Configuration setting values

//used as extern in other files
BOOL IRQ1_Received = FALSE;		//indicates whether IRQ0 has been received or not
BYTE TxPacket[PACKET_LEN];		//TX buffer
BYTE RxPacket[PACKET_LEN];		//RX buffer
BYTE TxPacLen;					//Packet length for TX packet
BYTE RxPacketLen;				//Packet length for Received packet
BYTE dummy;

//Function prototypes
void BoardInit(void);
void PrintHeader(char*);
void PrintSubHeader(char*);
void MRF89XAInit(void);
void PrintStatus(void);
void PrintMiniStatus(BYTE);
void ResetMRF89XA(void);
void Setup(void);
	
void drv_testMRF89XA(void)
{
	BYTE TxPacket_Len;
	BYTE i;
	WORD j,k;

	for(i=0;i<sizeof(PredefinedPacket);i++)
		TxPacket[i] = PredefinedPacket[i];
	TxPacket_Len = sizeof(PredefinedPacket);

	Send_Packet(TxPacket_Len);
}

unsigned char Regs[32];

void onIRQ1(void)
{
	IRQ1_Received = 1;
}

void drv_initMRF89XA(void)
{
	unsigned char i;
	//call all the initialization routines
    INT2_SetInterruptHandler(onIRQ1);
    
	MRF89XAInit();			//initialize MRF89XA
	Setup();				//Configure the basic settings

	for (i = 0 ; i <= 31; i++)
	{
		Regs[i] = RegisterRead(i);
	}
}

void Setup(void)
{

	BYTE input;
	SetRFMode(RF_STANDBY);
										//Program the chip to standby mode before changing from FSK to OOK or OOK to FSK
	//FSK Modulation
	Modulation_Type = TRUE;
	//Program FSK Modulation parameters
	input = RegisterRead(REG_MCPARAM1);
	input = ((input & 0x3F) | MODSEL_FSK);
	RegisterSet(REG_MCPARAM1, input);
#if 0
	//c(" 2. Frequency Band = 915-928 MHz, Center Frequency = 916 MHz\r\n");
	//case '2':
	FREQ_BAND = FREQ_BAND_RESET = FREQBAND_915;
	RVALUE = RVALUE_RESET = 119;
	PVALUE = PVALUE_RESET = 100;
	SVALUE = SVALUE_RESET =	50;
#else
	//c(" 4. Frequency Band = 863-870 MHz, Center Frequency = 868 MHz\r\n");
	//case '4':
	FREQ_BAND = FREQ_BAND_RESET = FREQBAND_950;
	RVALUE = RVALUE_RESET = 125;
	PVALUE = PVALUE_RESET = 100;
	SVALUE = SVALUE_RESET = 20;
#endif

	RegisterSet(REG_MCPARAM0, ((InitConfigRegs[REG_MCPARAM0]&0xE7)|FREQ_BAND));
	if(FREQ_BAND == FREQBAND_950)
	{
		BYTE readback = RegisterRead(REG_MCPARAM0);
		readback = (readback & 0xF8);
		RegisterSet(REG_MCPARAM0, readback);
	}
	RegisterSet(REG_R1, RVALUE);
	RegisterSet(REG_P1, PVALUE);
	RegisterSet(REG_S1, SVALUE);

	//c(" 1. DataRate = 2 kbps, BW = 50 KHz, Frequency Deviation = 33 KHz\r\n");
	//	case '1':
	Rate_C = BITRATE_2; Rate_R = 2;
	FREQ_BW = RXFC_FOPLUS50;
	TX_FSK = FREGDEV_33;
	PFILTER_SETTING = PASSIVEFILT_157;
	RegisterSet(REG_BITRATE, Rate_C);
	RegisterSet(REG_RXPARAM0, (FREQ_BW | PFILTER_SETTING));
	RegisterSet(REG_FREGDEV, TX_FSK);

	//Perform frequency synthesization
	input = RegisterRead(REG_MCPARAM0);
	RegisterSet(REG_MCPARAM0, (input & 0x1F) | RF_SYNTHESIZER);
	RF_Mode = RF_SYNTHESIZER;

	//clear PLL_LOCK flag so we can see it restore on the new frequency
	input = RegisterRead(REG_IRQPARAM1);
	RegisterSet(REG_IRQPARAM1, (input | 0x02));

	SetRFMode(RF_RECEIVER);
	//transceiver will be set to FSK or OOK mode
	//Include delay and put the chip back to standby mode
	{
	WORD i;
	for(i=0;i<5000;i++)
	{
		;
	}
	}
	SetRFMode(RF_STANDBY);

}

/*********************************************************************
 * void MRF89XAInit(void)
 *
 * Overview:        
 *              This functions initialize the MRF89XA transceiver
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 ********************************************************************/     
void MRF89XAInit(void)
{
	BYTE input,i=0;

#if 0
	{
		DWORD delay;
		for(;;) {
			BYTE test = RegisterRead(0);
			if((test == 0x28) || (test == 0x30)) {
				break;
			}
			for(delay=0; delay<5000; delay++) {
				;
			}
		}

		for(;;) {
			RegisterSet(0, InitConfigRegs[0]);

			for(delay=0; delay<5000; delay++) {
				;
			}
		}
	}
#endif

	// configuring the MRF89XA radio
	
  	//Configure MRF89XA Initialization parameters
/*
	BYTE readback;
	do
	{
		RegisterSet(i, InitConfigRegs[i]);
		readback = RegisterRead(i);
	}while(readback != InitConfigRegs[i]);
*/

	
//c("\r\n Initializing MRF89XA with the following Register settings\r\n");
	for (i = 0 ; i <= 31; i++)
	{
	RegisterSet(i, InitConfigRegs[i]);
		//readback register values and print on hyperterminal
		/*
		{
		BYTE readback;
		{
		readback = RegisterRead(i);
		c("Register ");
		PrintDigit(i);
		c(" - ");
		PrintChar(readback);
		c("\r\n");
		}
		}*/ //readback code commented	
	}
	RF_Mode = RF_STANDBY;
	input = RegisterRead(REG_MCPARAM0);
	RegisterSet(REG_MCPARAM0, (input & 0x1F) | RF_SYNTHESIZER);        		
	RF_Mode = RF_SYNTHESIZER;

	/* clear PLL_LOCK flag so we can see it restore on the new frequency */\
	input = RegisterRead(REG_IRQPARAM1);
	RegisterSet(REG_IRQPARAM1, (input | 0x02));

	input = RegisterRead(REG_MCPARAM0);
	RegisterSet(REG_MCPARAM0, ((input & 0xE7) | FREQBAND_915));
	//Program R, P,S registers
	RegisterSet(REG_R1, 119);
    RegisterSet(REG_P1, 100);
	RegisterSet(REG_S1, 52);
	input = RegisterRead(REG_MCPARAM0);
	RegisterSet(REG_MCPARAM0, ((input & 0x1F) | RF_SYNTHESIZER));        		
	RF_Mode = RF_SYNTHESIZER;

	/* clear PLL_LOCK flag so we can see it restore on the new frequency */
	input = RegisterRead(REG_IRQPARAM1);
	RegisterSet(REG_IRQPARAM1, (input | 0x02));

	SetRFMode(RF_STANDBY);
	PHY_IRQ1_En = 1;
}


/*********************************************************************
 * Function:        void ResetMRF89XA(void)
 *
 * PreCondition:    BoardInit() and SPIInit()
 *
 * Input:           
 *
 * Output:          Resets MRF89XA configuration settings
 *                  
 *
 * Side Effects:    None
 *
 * Overview:        Resets MRF89XA Configuration Settings
 *                  
 *
 * Note:            
 ********************************************************************/
void ResetMRF89XA(void)
{
	PacketDelay = 1;				//resets all the variables to reset values
	RxCount = 0;
	PingPong_Package = 100;
	MRF89XAInit();
	//CONSOLE_PutString ((char *)"MRF89XA Device has been reset \r\n");
}
