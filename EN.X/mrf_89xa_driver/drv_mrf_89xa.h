#ifndef _DRV_MRF89XA_H_
  #define _DRV_MRF89XA_H_

/*******************************************************************************
Microchip MRF89XA radio utility driver -drv_mrf_89xa file.
    
Company: Microchip Technology Inc.

File Description:

FileName:       drv_mrf_89xa.h
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


/********************************************************************
MRF89XA Operating modes
**********************************************************************/
#define RF_SLEEP                         0x00
#define RF_STANDBY                       0x20
#define RF_SYNTHESIZER                   0x40
#define RF_RECEIVER                      0x60
#define RF_TRANSMITTER                   0x80

/*********************************************************************
MRF89XA Register Description
*********************************************************************/

#define REG_MCPARAM0                     0
	#define CHIPMODE_SLEEPMODE  0x00	//000 [7:5]
	#define CHIPMODE_STBYMODE 	0x20	//001 ;default
	#define CHIPMODE_FSMODE	 	0x40	//010
	#define CHIPMODE_RX		 	0x60	//011
	#define CHIPMODE_TX		 	0x80	//100

	#define FREQBAND_902		0x00   //902-915 00 [4:3]
	#define FREQBAND_915		0x08   //915-928 01 ;default
	#define FREQBAND_950		0x10	//950-960 or 863-870 10
	
	#define VCO_TRIM_00			0x00	// [2:1] Vtune determined by tank inductor values
	#define VCO_TRIM_01			0x02
	#define VCO_TRIM_10			0x04
	#define VCO_TRIM_11			0x06

#define REG_MCPARAM1                     1
	#define MODSEL_OOK			0x40   //01 [7:6]
	#define MODSEL_FSK			0x80   //10 ;default

	#define OOKTHRESHTYPE_FIXED 0x00   //00 [4:3]
	#define OOKTHRESHTYPE_PEAK	0x08	//01
	#define OOKTHRESHTYPE_AVG	0x10	//10

	#define DATAMODE_CONTINUOUS 0x00   //00 [Bit2,Bit5];default
	#define DATAMODE_BUFFERED	0x20	//01
	#define DATAMODE_PACKET	    0x04	//1x
	
	#define IFGAIN_0			0x00	//00 [1:0] 0dB ;default
	#define IFGAIN_45			0x01	//01 -4.5dB
 	#define IFGAIN_9			0x02	//10 -9dB
	#define IFGAIN_135			0x03	//11 -13.5dB

#define REG_FREGDEV                      2		//default D = 00000011, FDEV = 133 kHz
		//values may change - should be calculated based on crystal frequency - 12.8 MHz (fxtal/32/(D+1))
		#define FREGDEV_33		0x0B
		#define FREGDEV_40		0x09
		#define FREGDEV_50		0x07
		#define FREGDEV_67		0x05
		#define FREGDEV_80		0x04
		#define FREGDEV_100		0x03
		#define FREGDEV_133		0x02
		#define FREGDEV_200		0x01
	
#define REG_BITRATE                      3
												//default C = 0000111, BR = 25 Kbps
		#define BITRATE_200		0x00
		#define BITRATE_100		0x01
		#define BITRATE_66		0x02
		#define BITRATE_50		0x03
		#define BITRATE_40		0x04
		#define BITRATE_25		0x07
		#define BITRATE_20		0x09
		#define BITRATE_16		0x0B
		#define BITRATE_10		0x13
		#define BITRATE_5		0x27
		#define BITRATE_2		0x63
		#define BITRATE_1_56	0x7F
		#define BITRATE_2_41	0x52
		#define BITRATE_4_76	0x29
		#define BITRATE_8		0x18
		#define BITRATE_9_52	0x14
		#define BITRATE_12_5	0x0F
		#define BITRATE_16_67	0x0B

#define REG_OOKFLOORTHRESH               4
	// OOK threshold (reg4)
	#define OOKFLOORTHRESH_VALUE 0x0C 

	
#define REG_MCPARAM5                     5
	#define FIFOSIZE_16			0x00	//00 [7:6] - 16bytes - default
	#define FIFOSIZE_32			0x40
	#define FIFOSIZE_48			0x80
	#define FIFOSIZE_64			0xC0
	
	#define FIFO_THRSHOLD_31	0x0F	//31 bytes - default [5:0]
	#define FIFO_THRSHOLD_1		0x01

#define REG_R1                           6

#define REG_P1                           7

#define REG_S1                           8

#define REG_R2                           9

#define REG_P2                           10

#define REG_S2                           11

#define REG_MCPARAM12                    12
	#define PA_RAMP_3			0x00	//[4:3] - 3us
	#define PA_RAMP_85			0x08	//8.5us
	#define PA_RAMP_15			0x10	//15us
	#define PA_RAMP_23			0x18	//23us
	#define DEF_PARAMP			0x20
#define REG_IRQPARAM0                    13
	// IRQ Param 0 (reg13)
	// Select RX&STDBY IRQ_0 sources (Packet mode) (reg13)
	#define IRQ0_RX_STDBY_PAYLOADREADY    0x00
	#define IRQ0_RX_STDBY_WRITEBYTE       0x40
	#define IRQ0_RX_STDBY_FIFOEMPTY       0x80
	#define IRQ0_RX_STDBY_SYNCADRS        0xC0
	
	// Select RX&STDBY IRQ_1 sources (Packet mode) (reg13)
	#define IRQ1_RX_STDBY_CRCOK           0x00
	#define IRQ1_RX_STDBY_FIFOFULL        0x10
	#define IRQ1_RX_STDBY_RSSI            0x20
	#define IRQ1_RX_STDBY_FIFOTHRESH      0x30
	
	// Select TX IRQ_1 sources (Packet mode) (reg13)
	#define IRQ1_TX_FIFOFULL              0x00
	#define IRQ1_TX_TXDONE                0x08

	// Select TX IRQ_1 sources (Continuous mode) (reg13)
	#define IRQ1_TX_DCLK              	  0x00

	
	// FIFO overrun/clear  (reg13)
	#define IRQ1_FIFO_OVERRUN_CLEAR       0x01
	

#define REG_IRQPARAM1                    14
	// IRQ Param 1
	#define DEF_IRQPARAM1				  0x08
	// Select TX start condition and IRQ_0 source (Packet mode)
	#define IRQ0_TX_START_FIFOTHRESH      0x00
	#define IRQ0_TX_START_FIFONOTEMPTY    0x10
	
	// RSSI IRQ flag
	#define IRQ1_RSSI_IRQ_CLEAR           0x04
	
	// PLL_Locked flag
	#define IRQ1_PLL_LOCK_CLEAR           0x02
	
	// PLL_Locked pin
	#define IRQ1_PLL_LOCK_PIN_OFF         0x00
	#define IRQ1_PLL_LOCK_PIN_ON          0x01
	
	// RSSI threshold for interrupt
	#define RSSIIRQTHRESH_VALUE           0x00

#define NODEADRS_VALUE 			0x00


#define REG_RSSIIRQTHRESH                15

#define REG_RXPARAM0                     16
	// RX Param 0
	// Passive filter (kHz)
	#define PASSIVEFILT_65            0x00		//65 KHz
	#define PASSIVEFILT_82            0x10		//82 KHz
	#define PASSIVEFILT_109           0x20
	#define PASSIVEFILT_137           0x30
	#define PASSIVEFILT_157           0x40
	#define PASSIVEFILT_184           0x50
	#define PASSIVEFILT_211           0x60
	#define PASSIVEFILT_234           0x70
	#define PASSIVEFILT_262           0x80
	#define PASSIVEFILT_321           0x90
	#define PASSIVEFILT_378           0xA0
	#define PASSIVEFILT_414           0xB0
	#define PASSIVEFILT_458           0xC0
	#define PASSIVEFILT_514           0xD0
	#define PASSIVEFILT_676           0xE0
	#define PASSIVEFILT_987           0xF0
	
	// Butterworth filter (kHz)
	#define RXFC_VALUE                  0x03
	// !!! Values defined below only apply if XTAL frequency == 12.8MHz !!!
	#define RXFC_FOPLUS25               0x00
	#define RXFC_FOPLUS50               0x01
	#define RXFC_FOPLUS75               0x02
	#define RXFC_FOPLUS100              0x03
	#define RXFC_FOPLUS125              0x04
	#define RXFC_FOPLUS150              0x05
	#define RXFC_FOPLUS175              0x06
	#define RXFC_FOPLUS200              0x07
	#define RXFC_FOPLUS225              0x08
	#define RXFC_FOPLUS250              0x09
	#define RXFC_FOPLUS275				0x0A
	#define RXFC_FOPLUS300				0x0B
	#define RXFC_FOPLUS325				0x0C
	#define RXFC_FOPLUS350				0x0D
	#define RXFC_FOPLUS375				0x0E
	#define RXFC_FOPLUS400				0x0F




#define REG_RXPARAM1                     17
	// RX Param 1
	#define DEF_RXPARAM1			  0x08
	// Polyphase filter center value (kHz)
	#define FO_VALUE                  0x30

	// !!! Values defined below only apply if RFCLKREF = DEFAULT VALUE = 0x07 !!!
	#define FO_50                     0x10
	#define FO_75                     0x20
	#define FO_100                    0x30
	#define FO_125                    0x40
	#define FO_150                    0x50
	#define FO_175                    0x60
	#define FO_200                    0x70
	#define FO_225                    0x80
	#define FO_250                    0x90
	#define FO_275                    0xA0
	#define FO_300                    0xB0
	#define FO_325                    0xC0
	#define FO_350                    0xD0
	#define FO_375                    0xE0
	#define FO_400                    0xF0


#define REG_RXPARAM2                     18
	// Rx Param 2
	// Polyphase filter enable
	#define POLYPFILT_ON              0x80
	#define POLYPFILT_OFF             0x00

	#define DEF_RXPARAM2				0x20
	#define SYNC_ON						0x20	//[5] - 0 - off, 1 - on
	#define SYNC_SIZE_8					0x00	//[4:3] - 00 - 8 bits 
	#define SYNC_SIZE_16				0x08
	#define SYNC_SIZE_24				0x10
	#define SYNC_SIZE_32				0x18
	#define SYNC_ERRORS_0				0x00	//[2:1] - 00 - 0 error
	#define SYNC_ERRORS_1				0x02
	#define SYNC_ERRORS_2				0x04
	#define SYNC_ERROS_3				0x06
#define REG_RXPARAM3                     19				//Reserved
	#define DEF_RXPARAM3				0x07
#define REG_RSSIVALUE                    20
	//RSSI value (READ ONLY)
#define REG_OOKSETTINGS                  21
	// Rx Param 5
	// Decrement step of RSSI threshold in OOK demodulator (peak mode)
	#define OOK_THRESH_DECSTEP_000    0x00
	#define OOK_THRESH_DECSTEP_001    0x20
	#define OOK_THRESH_DECSTEP_010    0x40
	#define OOK_THRESH_DECSTEP_011    0x60
	#define OOK_THRESH_DECSTEP_100    0x80
	#define OOK_THRESH_DECSTEP_101    0xA0
	#define OOK_THRESH_DECSTEP_110    0xC0
	#define OOK_THRESH_DECSTEP_111    0xE0
	
	// Decrement period of RSSI threshold in OOK demodulator (peak mode)
	#define OOK_THRESH_DECPERIOD_000  0x00
	#define OOK_THRESH_DECPERIOD_001  0x04
	#define OOK_THRESH_DECPERIOD_010  0x08
	#define OOK_THRESH_DECPERIOD_011  0x0C
	#define OOK_THRESH_DECPERIOD_100  0x10
	#define OOK_THRESH_DECPERIOD_101  0x14
	#define OOK_THRESH_DECPERIOD_110  0x18
	#define OOK_THRESH_DECPERIOD_111  0x1C
	
	// Cutoff freq of average function of RSSI threshold in OOK demodulator (average mode)
	#define OOK_THRESH_AVERAGING_00   0x00
	#define OOK_THRESH_AVERAGING_11   0x03

#define REG_SYNCBYTE0                    22
#define REG_SYNCBYTE1                    23
#define REG_SYNCBYTE2                    24
#define REG_SYNCBYTE3                    25

#define REG_TXPARAM                      26
	// TX Param 
	// Interpolator filter Tx (kHz)
	#define FC_VALUE                   0x70
	// !!! Values defined below only apply if RFCLKREF = DEFAULT VALUE = 0x07 !!!
	#define FC_25                      0x00
	#define FC_50                      0x10
	#define FC_75                      0x20
	#define FC_100                     0x30
	#define FC_125                     0x40
	#define FC_150                     0x50
	#define FC_175                     0x60
	#define FC_200                     0x70
	#define FC_225                     0x80
	#define FC_250                     0x90
	#define FC_275                     0xA0
	#define FC_300                     0xB0
	#define FC_325                     0xC0
	#define FC_350                     0xD0
	#define FC_375                     0xE0
	#define FC_400                     0xF0

	#define TXPOWER_13					0x00	//[3:1] - 13dB
	#define TXPOWER_10					0x02	
	#define TXPOWER_7						0x04
	#define TXPOWER_4						0x06
	#define TXPOWER_1						0x08
	#define TXPOWER_2						0x0A
	#define TXPOWER_5						0x0C
	#define TXPOWER_8						0x0E

#define REG_OSCPARAM                     27
	// OSC Param
	// CLKOUT enable
	#define CLKOUT_ON                 0x80
	#define CLKOUT_OFF                0x00
	
	// CLKOUT frequency (kHz)
	#define CLKOUT_12800              0x00
	#define CLKOUTint64_t00           0x04
	#define CLKOUT_3200               0x08
	#define CLKOUT_2133               0x0C
	#define CLKOUT_1600               0x10
	#define CLKOUT_1280               0x14
	#define CLKOUT_1067               0x18
	#define CLKOUT_914                0x1C
	#define CLKOUT_800                0x20
	#define CLKOUT_711                0x24
	#define CLKOUT_64		          0x28
	#define CLKOUT_582                0x2C
	#define CLKOUT_533                0x30
	#define CLKOUT_492                0x34
	#define CLKOUT_457                0x38
	#define CLKOUT_427                0x3C


#define REG_PKTPARAM0                    28
	#define MANCHESTER_ON				0x80	//[7] 0 - off, 1 - on
	#define MANCHESTER_OFF				0x00
#define REG_NODEADDRESS                  29
#define REG_PKTPARAM2                    30
	#define PKT_FORMAT_FIXED			0x00	//[7] 0 - fixed length, 1- variable length
	#define PKT_FORMAT_VARIABLE			0x80
	#define PREAMBLE_SIZE_1				0x00 	//[6:5] 00 - 1 byte
	#define PREAMBLE_SIZE_2				0x20
	#define PREAMBLE_SIZE_3				0x40
	#define PREAMBLE_SIZE_4				0x60
	
	#define WHITENING_ON				0x10	//[4] 0 - off, 1 - On
	#define WHITENING_OFF				0x00	
	
	#define CRC_ON						0x08	//[3] 0 - off, 1 - On
	#define CRC_OFF						0x00	
	// Address filtering
	#define ADRSFILT_NONE				0x00	/* no filtering */
	#define ADRSFILT_ME_ONLY			0x02	/* only Node_adrs accepted (register 29) */
	#define ADRSFILT_ME_AND_00			0x04	/* Node_adrs and 0x00 accepted */
	#define ADRSFILT_ME_AND_00_AND_FF	0x06	/* Node_adrs and 0x00 and 0xff accepted */


#define REG_PKTPARAM3                    31
	#define FIFO_AUTOCLR_ON				0x00	//[7] 0 - On, 1 - Off
	#define FIFO_AUTOCLR_OFF			0x80

	#define FIFO_STBY_ACCESS_WRITE		0x00	//[6] 0 - Write, 1 - Read
	#define FIFO_STBY_ACCESS_READ		0x40

#endif
