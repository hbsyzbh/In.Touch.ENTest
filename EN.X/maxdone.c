#include "mcc_generated_files/mcc.h"
#include "maxdone.h"
unsigned char debug = 1;
void UART2_Sendstr(char *str)
{
	if(str == 0) return;
	if(! debug) return;

	while(*str != 0)
	{
		TXREG1 = *str++;
		while(! EUSART1_is_tx_done());
	}
}

void R_UART2_Receive(unsigned char *data, unsigned char len)
{
    eusart1Want = len;
    eusart1Data = data;
}

void R_UART2_Send(unsigned char *data, unsigned char len)
{    
    unsigned char i;
    
    for(i = 0; i < len; i++)
    {
		TXREG1 = data[i];
		while(! EUSART1_is_tx_done());        
    }
}

typedef enum {
	Cmd_getType,
	Cmd_syncRTC,
	Cmd_getRTC,
	Cmd_testSPIFlash,
	Cmd_testI2CPort,
	Cmd_testRF 			= 5,
	Cmd_testI2CE2prom,
	Cmd_testEthernet,
	Cmd_testLED,

	Cmd_initI2CPort = 0xF4
}	CMD_TYPE;

#define ACK_HEAD (0x4F)

#define ACK_CMD_POS		(1)
#define ACK_LEN_POS		(2)
#define ACK_DATA_POS	(3)

#define ACK_OK	(0xF0)
#define ACK_NG	(0xFF)

typedef enum {
	Uart_StartwaitHead,
	Uart_waittigHead,
	Uart_revHeadStartwaitlen,
	Uart_Waittinglen,
	Uart_revlenStartwaitData,
	Uart_waittingData,
	Uart_analysisData
}	UART_STATE;

UART_STATE UartState;

unsigned char cmd = 0;
unsigned char datalen = 0;

unsigned char revBuff[256] = {0};

static void startWaitForNewCmd()
{
	UartState = 0;
}

static char isCmdDatalenValied()
{
	switch(cmd) {
		case Cmd_syncRTC:
			return datalen == 6;
			break;
		case Cmd_getRTC:
		case Cmd_getType:
		case Cmd_testSPIFlash:
		case Cmd_testI2CPort:
		case Cmd_testRF:
		case Cmd_testI2CE2prom:
		case Cmd_testEthernet:
			return datalen == 0;
			break;
		case Cmd_testLED:
			return datalen == 1;
			break;

		case Cmd_initI2CPort:
			return datalen == 0;
			break;
	}

	return 0;
}

void SetLed(unsigned char leds)
{
	unsigned char b = leds & 0x1;
	unsigned char g = leds & 0x2;
	unsigned char r = leds & 0x4;

	IO_RE4_LAT = ! b;
	IO_RE3_LAT = ! g;
	IO_RE2_LAT = ! r;
}


const unsigned char notsupport[] = {0x40, 0, 0};
const unsigned char hardwareType[] = {ACK_HEAD, 0, 1, 0x0E};
unsigned char ackbuff[16] = {ACK_HEAD, 0};

static unsigned char TestMRF89XA()
{
	//drv_testMRF89XA();
	return ACK_NG;
}
/* I2C1 Register Level interfaces */
static inline bool I2C1_MasterOpen(void)
{
    if(!SSP1CON1bits.SSPEN)
    {
        SSP1STAT = 0x00;
        SSP1CON1 = 0x08;
        SSP1CON2 = 0x00;
        SSP1ADD = 0x3D;
        SSP1CON1bits.SSPEN = 1;
        return true;
    }
    return false;
}

static inline void I2C1_MasterClose(void)
{
    //Disable I2C1
    SSP1CON1bits.SSPEN = 0;
}

static inline uint8_t I2C1_MasterGetRxData(void)
{
    return SSP1BUF;
}

static inline void I2C1_MasterSendTxData(uint8_t data)
{
    SSP1BUF  = data;
}

static inline void I2C1_MasterEnableRestart(void)
{
    SSP1CON2bits.RSEN = 1;
}

static inline void I2C1_MasterDisableRestart(void)
{
    SSP1CON2bits.RSEN = 0;
}

static inline void I2C1_MasterStartRx(void)
{
    SSP1CON2bits.RCEN = 1;
}

static inline void I2C1_MasterStart(void)
{
    SSP1CON2bits.SEN = 1;
}

static inline void I2C1_MasterStop(void)
{
    SSP1CON2bits.PEN = 1;
}

static inline bool I2C1_MasterIsNack(void)
{
    return SSP1CON2bits.ACKSTAT;
}

static inline void I2C1_MasterSendAck(void)
{
    SSP1CON2bits.ACKDT = 0;
    SSP1CON2bits.ACKEN = 1;
}

static inline void I2C1_MasterSendNack(void)
{
    SSP1CON2bits.ACKDT = 1;
    SSP1CON2bits.ACKEN = 1;
}

static inline void I2C1_MasterClearBusCollision(void)
{
    PIR2bits.BCL1IF = 0;
}


static inline bool I2C1_MasterIsRxBufFull(void)
{
    return SSP1STATbits.BF;
}

static inline void I2C1_MasterEnableIrq(void)
{
    PIE1bits.SSP1IE = 1;
}

static inline bool I2C1_MasterIsIrqEnabled(void)
{
    return PIE1bits.SSP1IE;
}

static inline void I2C1_MasterDisableIrq(void)
{
    PIE1bits.SSP1IE = 0;
}

static inline void I2C1_MasterClearIrq(void)
{
    PIR1bits.SSP1IF = 0;
}

static inline void I2C1_MasterSetIrq(void)
{
    PIR1bits.SSP1IF = 1;
}

static inline void I2C1_MasterWaitForEvent(void)
{
    while(1)
    {
        if(PIR1bits.SSP1IF)
        {    
            break;
        }
    }
}

void I2C1_syncWrite(unsigned char data)
{
	I2C1_MasterClearIrq();
	I2C1_MasterSendTxData(data);
	I2C1_MasterWaitForEvent();
}

unsigned char I2C1_syncRead(void)
{
	unsigned char data;

	I2C1_MasterClearIrq();
	data = I2C1_MasterGetRxData();
	I2C1_MasterWaitForEvent();

	return data;
}

unsigned char TestI2CE2PROM()
{
	unsigned char MARK[] = " Maxdone";
	unsigned char buff[8] = {0};
	unsigned char i;
	MARK[0] = buff[0] = 0;

//	I2C1_WriteNBytes(0xA0, MARK, 8);
//	I2C1_ReadNBytes(0xA0, buff, 8);

	I2C1_MasterOpen();

//Ð´
	I2C1_MasterSendAck();
	I2C1_MasterStart();

	I2C1_syncWrite(0xA0);

	for(i = 0; i <8; i++)
	{
		I2C1_syncWrite(MARK[i]);
	}
	I2C1_MasterStop();

//¶Á
	I2C1_MasterStart();
	I2C1_syncWrite(0xA0);
	I2C1_syncWrite(0x00);

	I2C1_MasterStart();
	I2C1_syncWrite(0xA1);
	for(i = 1; i <7; i++)
	{
		buff[i] = I2C1_syncRead();
	}

	I2C1_MasterSendNack();
	buff[i] = I2C1_syncRead();	
	I2C1_MasterStop();


	I2C1_MasterClose();

	if( 0 == memcmp(MARK, buff, 8)) {
		return ACK_OK;
	}
	
	return ACK_NG;
}

unsigned char TestEthernet()
{
	return ACK_NG;
}

static void analysisCmd()
{

	ackbuff[ACK_CMD_POS] = cmd;
	ackbuff[ACK_LEN_POS] = 1;
	

	switch(cmd) {
	case Cmd_getType:
		R_UART2_Send(hardwareType, 4);
		break;

	case Cmd_testLED:
		SetLed(revBuff[ACK_DATA_POS]);
		ackbuff[ACK_DATA_POS] = revBuff[ACK_DATA_POS];
		R_UART2_Send(ackbuff, 	ACK_DATA_POS + 1);
		break;

	case Cmd_testRF:
		ackbuff[ACK_DATA_POS] = TestMRF89XA();
		R_UART2_Send(ackbuff, 	ACK_DATA_POS + 1);
		break;

	case Cmd_testI2CE2prom:
		ackbuff[ACK_DATA_POS] = TestI2CE2PROM();
		R_UART2_Send(ackbuff, 	ACK_DATA_POS + 1);
		break;
		
	case Cmd_testEthernet:
		ackbuff[ACK_DATA_POS] = TestEthernet();
		R_UART2_Send(ackbuff, 	ACK_DATA_POS + 1);
		break;

	default:
		R_UART2_Send(notsupport, 3);
		break;
	}
	startWaitForNewCmd();
}

void doUartTask(void)
{
	//TestMRF89XA();

	switch(UartState)
	{
		case Uart_StartwaitHead:
			R_UART2_Receive(revBuff, 1);
			UartState++;
			break;
		case Uart_revHeadStartwaitlen:
			R_UART2_Receive(&revBuff[1], 2);
			UartState++;
			break;
		case Uart_revlenStartwaitData:
			if(datalen > 0 ) {
				R_UART2_Receive(&revBuff[3], datalen);
				UartState++;
			}
			break;

		case Uart_analysisData:
			analysisCmd();
			break;

		default:
			break;
	}
}

void on_uart1_receiveend(void)
{
	switch(UartState)
	{
		case Uart_waittigHead:
			if((*revBuff) == 0xF4){
				UartState = Uart_revHeadStartwaitlen;
			} else {
				UartState = Uart_StartwaitHead;
			}
			break;

			break;
		case Uart_Waittinglen:
			cmd = revBuff[1];
			datalen = revBuff[2];

			if(isCmdDatalenValied())
			{
				if(datalen > 0) {
					UartState = Uart_revlenStartwaitData;
				} else {
					UartState = Uart_analysisData;
				}
			} else {
				UartState  = Uart_StartwaitHead;
			}
			break;
		case Uart_waittingData:
			UartState = Uart_analysisData;
			break;
	}
}

void on_uart1_error(void)
{
	startWaitForNewCmd();
}

void doKeyTask(void)
{
	static unsigned char keyState  = 0;
	static unsigned char lastKeyValue = 0xFF;

	unsigned char key = IO_RC5_GetValue();

	if(lastKeyValue != 0xFF)
	{
		if(lastKeyValue != key) {
			if(key == 0) {  //ÏÂ½µÑØ
				switch (keyState)
				{
					case 0:
						IO_RA0_SetHigh();
						IO_RA1_SetLow();
						keyState++;
						break;
					case 1:
						IO_RA0_SetLow();
						IO_RA1_SetHigh();
						keyState++;
						break;
					case 2:
						IO_RA0_SetLow();
						IO_RA1_SetLow();
						
						keyState++;
						break;
				}
			}
		}
	}

	lastKeyValue = key;	
}

