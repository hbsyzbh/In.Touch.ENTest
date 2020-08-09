/*******************************************************************************
Microchip MRF89XA radio utility driver -drv_mrf_89xa file.
    
Company: Microchip Technology Inc.

File Description:

FileName:       drv_mrf_89xa.c
Dependencies:   None
Processor:      PIC18 Microcontroller/PIC24 Microcontroller
Hardware:       The code is natively intended to be used on the following hardware platforms:
                PIC18 Explorer Development Board/Explorer 16 Development Board
                MRF89XA PICtail/PICtail Plus boards
Firmware:       version 2.00 and above
Compiler:       Microchip XC8 V1.40/ XC16 V1.30
IDE:            Microchip MPLAB X V3.50
 
Summary:
                Microchip MRF89XA radio utility driver -drv_mrf_89xa file.
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

#include "drv_mrf_89xa.h"
#include "drv_mrf.h"

//First time configuration settings for MRF89XA
char InitConfigRegs[] = {
		/* 0 */				CHIPMODE_STBYMODE | FREQBAND_915 | VCO_TRIM_11, 
		/* 1 */				MODSEL_FSK | DATAMODE_PACKET | IFGAIN_0,
		/* 2 */				FREGDEV_80,
		/* 3 */				BITRATE_25,
		/* 4 */				OOKFLOORTHRESH_VALUE,
		/* 5 */				FIFOSIZE_64 | FIFO_THRSHOLD_1,
		/* 6 */				0,
		/* 7 */				0,
		/* 8 */				0,
		/* 9 */				0,
		/* 10 */			0,
		/* 11 */			0,
		/* 12 */			DEF_PARAMP | PA_RAMP_23,
		/* 13 */			IRQ0_RX_STDBY_SYNCADRS | IRQ1_RX_STDBY_CRCOK | IRQ1_TX_TXDONE,                     
		/* 14 */ 			DEF_IRQPARAM1 | IRQ0_TX_START_FIFOTHRESH | IRQ1_PLL_LOCK_PIN_ON,
		/* 15 */ 			RSSIIRQTHRESH_VALUE,
		/* 16 */ 			PASSIVEFILT_378 | RXFC_FOPLUS100,
		/* 17 */			DEF_RXPARAM1 | FO_100,
		/* 18 */			DEF_RXPARAM2 | POLYPFILT_OFF | SYNC_SIZE_32 | SYNC_ON | SYNC_ERRORS_0,
		/* 19 */			DEF_RXPARAM3,
		/* 20 */			0,
		/* 21 */			OOK_THRESH_DECSTEP_000 | OOK_THRESH_DECPERIOD_000 | OOK_THRESH_AVERAGING_00,
		/* 22 */ 			0x69, // 1st byte of Sync word,
		/* 23 */ 			0x81, // 2nd byte of Sync word,
		/* 24 */ 			0x7E, // 3rd byte of Sync word,
		/* 25 */ 			0x96, // 4th byte of Sync word,
		/* 26 */ 			FC_400 | TXPOWER_13,
		/* 27 */ 			CLKOUT_ON | CLKOUT_12800,
		/* 28 */ 			MANCHESTER_OFF | 64,
		/* 29 */ 			NODEADRS_VALUE,
		/* 30 */ 			PKT_FORMAT_VARIABLE | PREAMBLE_SIZE_4 | WHITENING_OFF | CRC_ON | ADRSFILT_NONE,
 		/* 31 */ 			FIFO_AUTOCLR_ON | FIFO_STBY_ACCESS_WRITE 
							};	
//declare external variables
extern BOOL IRQ0_Received;
extern BOOL IRQ1_Received;
extern BYTE RxPacketLen;
extern BOOL hasPacket;
extern BYTE RF_Mode;
extern BYTE TxPacket[PACKET_LEN];
extern BYTE RxPacket[PACKET_LEN];
extern BOOL Enable_DutyCycle;
extern WORD Duty_Cycle;

/*********************************************************************
 * void RegisterSet(BYTE address, BYTE value)
 *
 * Overview:        
 *              This function access the control register of MRF89XA.
 *              The register address and the register settings are
 *              the input
 *
 * PreCondition:    
 *            	 None
 *
 * Input:       
 *          WORD    setting     The address of the register and its
 *                              corresponding settings
 *
 * Output:  None    
 *
 * Side Effects:    Register settings have been modified
 *
 ********************************************************************/
void RegisterSet(BYTE address, BYTE value)
{
	volatile BYTE tmp0RFIE = PHY_IRQ1_En;

	PHY_IRQ1_En = 0;

    Config_nCS = 0;
	address = (address<<1);
    SPIPut(address);
    SPIPut(value);
    Config_nCS = 1;
	PHY_IRQ1_En = tmp0RFIE;
}

/*********************************************************************
 * WORD RegisterRead()
 *
 * Overview:        
 *              This function access the control register of MRF89XA.
 *              The register address and the register settings are
 *              the input
 *
 * PreCondition:    
 *              None
 *
 * Input:       
 *          WORD    setting     The address of the register and its
 *                              corresponding settings
 *
 * Output:  None    
 *
 * Side Effects:    Register settings have been modified
 *
 ********************************************************************/
BYTE RegisterRead(BYTE address)
{
	volatile BYTE tmp0RFIE = PHY_IRQ1_En;
	BYTE value;
    Config_nCS = 0;
	address = ((address<<1)|0x40);
    SPIPut(address);
	value = SPIGet();
    Config_nCS = 1;
	PHY_IRQ1_En = tmp0RFIE;
	return value;
}

/*********************************************************************
 * void Send_Packet(BYTE TxPacketLen)
 *
 * Overview:        
 *              This function send the packet in the buffer TxPacket
 *
 * PreCondition:    
 *              MRF89XA transceiver has been properly initialized
 *
 * Input:       
 *              BYTE    TxPacketLen     The length of the packet to be
 *                                      sent.
 *
 * Output:      None
 *
 * Side Effects:    
 *              The packet has been sent out
 *
 ********************************************************************/
void Send_Packet(BYTE TxPacketLen)
{
	WORD i;
	BYTE j;
	SetRFMode(RF_STANDBY);

	RegisterSet(REG_PKTPARAM3, ((InitConfigRegs[REG_PKTPARAM3] & 0xBF)| FIFO_STBY_ACCESS_WRITE));
	RegisterSet(REG_IRQPARAM0, (InitConfigRegs[REG_IRQPARAM0] | IRQ1_FIFO_OVERRUN_CLEAR ));
	RegisterSet(REG_IRQPARAM1, ((InitConfigRegs[REG_IRQPARAM1]) | 0x02));
	WriteFIFO(TxPacketLen+1);
	WriteFIFO(0x00);		//Node_adrs
	for(i=0; i< TxPacketLen; i++)
	{
		WriteFIFO(TxPacket[i]);
	}

	if(Enable_DutyCycle)
	{
	}

	SetRFMode(RF_TRANSMITTER);
	while(!IRQ1_Received);	//Wait until TX Done interrupt and restore the RF state to standby mode
	SetRFMode(RF_STANDBY);

	if(Enable_DutyCycle)
	{
	}

	IRQ1_Received = FALSE;

	//Reset FIFO
	j = RegisterRead(REG_IRQPARAM0);
	RegisterSet(REG_IRQPARAM0, (j | 0x01));
    
}
/*********************************************************************
 * void WriteFIFO(BYTE Data)
 *
 * Overview:        
 *              This function fills the FIFO
 *
 * PreCondition:    
 *              MRF89XA transceiver has been properly initialized
 *
 * Input:       
 *              BYTE   Data - Data to be sent to FIFO.
 *
 * Output:      None
 *
 * Side Effects:    
 *              The packet has been sent out
 *
 ********************************************************************/
void WriteFIFO(BYTE Data)
{
	volatile BYTE tmp0RFIE = PHY_IRQ1_En;
	PHY_IRQ1_En = 0;
    Data_nCS = 0;
    SPIPut(Data);
    Data_nCS = 1;
	PHY_IRQ1_En = tmp0RFIE;
}

/*********************************************************************
 * void ReceiveFrame(void)
 *
 * Overview:        
 *              This function reads the Reiceved frame from the fifo and sets hasPacket accordingly
 *
 * PreCondition:    
 *              MRF89XA transceiver has been properly initialized
 *
 * Input:       
 *              None
 *
 * Output:      None
 *
 * Side Effects:    
 *              The packet has been sent out
 *
 ********************************************************************/
void ReceiveFrame()
{
	BYTE data, node_adrs;
	BYTE i = 0;
	if(IRQ1_Received)
		{
		RxPacketLen = ReadFIFO();	
		IRQ0_Received = FALSE;
		node_adrs = ReadFIFO();
		RxPacketLen = (RxPacketLen-1);

		while(RxPacketLen--)
			{
			IRQ0_Received = FALSE;
			data = ReadFIFO();
			RxPacket[i] = data;
			i++;
			};
		RxPacketLen = i;
		}	
	IRQ1_Received = FALSE;
	hasPacket = TRUE;
	//Reset FIFO
	i = RegisterRead(REG_IRQPARAM0);
	RegisterSet(REG_IRQPARAM0, (i | 0x01));
}

/*********************************************************************
 * BYTE ReadFIFO(void)
 *
 * Overview:        
 *              This function reads the Reiceved frame from the fifo and sets hasPacket accordingly
 *
 * PreCondition:    
 *              MRF89XA transceiver has been properly initialized
 *
 * Input:       
 *              None
 *
 * Output:      Data from FIFO
 *
 * Side Effects:    
 *              The packet has been sent out
 *
 ********************************************************************/
BYTE ReadFIFO(void)
{
	volatile BYTE tmp0RFIE = PHY_IRQ1_En;
	BYTE value;
    Data_nCS = 0;
	value = SPIGet();
    Data_nCS = 1;
	PHY_IRQ1_En = tmp0RFIE;
	return value;

}

/*********************************************************************
 * void SetRFMode(BYTE mode)
 *
 * Overview:        
 *              This functions sets the MRF89XA transceiver operating mode to sleep, transmit, receive or standby
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
void SetRFMode(BYTE mode)
{
	BYTE	mcparam0_read;
	mcparam0_read = RegisterRead(REG_MCPARAM0);
	switch (mode) {
		case RF_TRANSMITTER:
			RegisterSet(REG_MCPARAM0, (mcparam0_read & 0x1F) | RF_TRANSMITTER);
			RF_Mode = RF_TRANSMITTER;				//RF in TX mode
			break;
		case RF_RECEIVER:
			RegisterSet(REG_MCPARAM0, (mcparam0_read & 0x1F) | RF_RECEIVER);
			RF_Mode = RF_RECEIVER;					//RF in RX mode
			break;
		case RF_SYNTHESIZER:
			RegisterSet(REG_MCPARAM0, (mcparam0_read & 0x1F) | RF_SYNTHESIZER);
			RF_Mode = RF_SYNTHESIZER;				//RF in Synthesizer mode
			break;
		case RF_STANDBY:
			RegisterSet(REG_MCPARAM0, (mcparam0_read & 0x1F) | RF_STANDBY);
			RF_Mode = RF_STANDBY;					//RF in standby mode
			break;
		case RF_SLEEP:
			RegisterSet(REG_MCPARAM0, (mcparam0_read & 0x1F) | RF_SLEEP);
			RF_Mode = RF_SLEEP;						//RF in sleep mode
			break;
	} /* end switch (mode) */

}
