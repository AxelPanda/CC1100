/*
	CC1100.h - CC1100 Module Library for RaspberryPi
	Copyright (C) 2013-2014 Wu Huanjie.
	Author: Wu Huanjie, <panda.zjutv.com>
	Version: November 23, 2013

	This library is designed to use CC1100/CC1101 module on RaspberryPi platform.
	CC1100/CC1101 module is an useful wireless module. Using the functions of the
	library, you can easily send and receive data by the CC1100/CC1101 module.
	Just have fun!
	For the details, please refer to the datasheet of CC1100/CC1101.
*/
#include "CC1100.h"
#include <bcm2835.h>

#define WRITE_BURST     	0x40	//连续写入
#define READ_SINGLE     	0x80	//读
#define READ_BURST      	0xC0	//连续读
#define BYTES_IN_RXFIFO     0x7F  	//接收缓冲区的有效字节数
#define CRC_OK              0x80 	//CRC校验通过位标志

//***************更多功率参数设置可详细参考DATACC1100英文文档中第48-49页的参数表******************
//uint8_t PaTable[8] = {0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};  //-30dBm   功率最小
//uint8_t PaTable[8] = {0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60};  //0dBm
uint8_t PaTable[8] = { 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0 };   //10dBm     功率最大
//***********************************************************************************************

// RF_SETTINGS is a data structure which contains all relevant CCxxx0 registers
typedef struct S_RF_SETTINGS
{
	//uint8_t FSCTRL2; // Frequency synthesizer control.
	uint8_t FSCTRL1;   // Frequency synthesizer control.
	uint8_t FSCTRL0;   // Frequency synthesizer control.
	uint8_t FREQ2;     // Frequency control word, high INT8U.
	uint8_t FREQ1;     // Frequency control word, middle INT8U.
	uint8_t FREQ0;     // Frequency control word, low INT8U.
	uint8_t MDMCFG4;   // Modem configuration.         ********设置空中波特率
	uint8_t MDMCFG3;   // Modem configuration.         ********设置空中波特率
	uint8_t MDMCFG2;   // Modem configuration.
	uint8_t MDMCFG1;   // Modem configuration.
	uint8_t MDMCFG0;   // Modem configuration.
	uint8_t CHANNR;    // Channel number.
	uint8_t DEVIATN;   // Modem deviation setting (when FSK modulation is enabled).
	uint8_t FREND1;    // Front end RX configuration.
	uint8_t FREND0;    // Front end RX configuration.
	uint8_t MCSM0;     // Main Radio Control State Machine configuration.
	uint8_t FOCCFG;    // Frequency Offset Compensation Configuration.
	uint8_t BSCFG;     // Bit synchronization Configuration.
	uint8_t AGCCTRL2;  // AGC control.
	uint8_t AGCCTRL1;  // AGC control.
	uint8_t AGCCTRL0;  // AGC control.
	uint8_t FSCAL3;    // Frequency synthesizer calibration.
	uint8_t FSCAL2;    // Frequency synthesizer calibration.
	uint8_t FSCAL1;    // Frequency synthesizer calibration.
	uint8_t FSCAL0;    // Frequency synthesizer calibration.
	uint8_t FSTEST;    // Frequency synthesizer calibration control
	uint8_t TEST2;     // Various test settings.
	uint8_t TEST1;     // Various test settings.
	uint8_t TEST0;     // Various test settings.
	uint8_t IOCFG2;    // GDO2 output pin configuration
	uint8_t IOCFG0;    // GDO0 output pin configuration
	uint8_t PKTCTRL1;  // Packet automation control.
	uint8_t PKTCTRL0;  // Packet automation control.
	uint8_t ADDR;      // Device address.
	uint8_t PKTLEN;    // Packet length.
} RF_SETTINGS;

/////////////////////////////////////////////////////////////////
const RF_SETTINGS rfSettings =
{
	//0x00,			// FSCTRL2   Frequency synthesizer control.
	0x0A,	//0x08,	// FSCTRL1   Frequency synthesizer control.    for  38.4K
	0x00,			// FSCTRL0   Frequency synthesizer control.
	0x10,			// FREQ2     Frequency control word, high byte.
	0xB1,	//0xA7,	// FREQ1     Frequency control word, middle byte.
	0x3B,	//0x62,	// FREQ0     Frequency control word, low byte.
	0x2D,	//0xCA,	// MDMCFG4   Modem configuration.     for 38.4K
	0x3B,	//0x83,	// MDMCFG3   Modem configuration.     for 38.4K
	0x73,	//0x03,	// MDMCFG2   Modem configuration.     for 38.4K
	0x22,			// MDMCFG1   Modem configuration.
	0xF8,			// MDMCFG0   Modem configuration.

	0x00,			// CHANNR    Channel number.
	0x00,	//0x34,	// DEVIATN   Modem deviation setting (when FSK modulation is enabled).   for 38.4K
	0xB6,	//0x56,	// FREND1    Front end RX configuration.   for 38.4K
	0x10,			// FREND0    Front end RX configuration.
	0x18,	//0x18,	// MCSM0     Main Radio Control State Machine configuration.
	0x1D,	//0x16,	// FOCCFG    Frequency Offset Compensation Configuration.   for 38.4K
	0x1C,	//0x6C,	// BSCFG     Bit synchronization Configuration.             for 38.4K
	0xC7,	//0x43,	// AGCCTRL2  AGC control.       for 38.4K                              
	0x00,	//0x40,	// AGCCTRL1  AGC control.       for 38.4K
	0xB0,	//0x91,	// AGCCTRL0  AGC control.       for 38.4K

	0xEA,	//0xE9,	// FSCAL3    Frequency synthesizer calibration.   for 38.4k
	0x2A,			// FSCAL2    Frequency synthesizer calibration.   for 38.4K
	0x00,			// FSCAL1    Frequency synthesizer calibration.   for 38.4K
	0x1F,			// FSCAL0    Frequency synthesizer calibration.   for 38.4K
	0x59,			// FSTEST    Frequency synthesizer calibration.
	0x81,			// TEST2     Various test settings.
	0x35,			// TEST1     Various test settings.
	0x09,			// TEST0     Various test settings.
	0x06,	//0x0B,	// IOCFG2    GDO2 output pin configuration.
	0x06,			// IOCFG0D   GDO0 output pin configuration. Refer to SmartRF?Studio User Manual for detailed pseudo register explanation.

	0x04,			// PKTCTRL1  Packet automation control.
	0x05,			// PKTCTRL0  Packet automation control.
	0x00,			// ADDR      Device address.
	0x3D			// PKTLEN    Packet length.
};

/* SPIInit, SPI Initialization */
void SPIInit()
{
	bcm2835_gpio_clr(RPI_GPIO_P1_24);
	bcm2835_gpio_set(RPI_GPIO_P1_24);
}

/* CPUInit, CPU Initialization */
void CPUInit()
{
	SPIInit();		// SpiInit();
}

/* SPITxRxByte, SPI Send(Terminal) and Receive by Byte */
uint8_t SPITxRxByte(uint8_t dat)
{
	uint8_t temp = bcm2835_spi_transfer(dat);
	return temp;
}

/* Reset, Reset the CC1100 chip */
void Reset()
{
	while(bcm2835_gpio_lev(RPI_GPIO_P1_21));	// is there any need of while (MISO); ? this is used to avoid data confliction, I think.
	SPITxRxByte(CCxxx0_SRES);					// SpiTxRxByte(CCxxx0_SRES);
	while(bcm2835_gpio_lev(RPI_GPIO_P1_21));	// the same as above: while (MISO);
}

/* Power_Up_Reset, Power up reset the CC1100 chip (may not be used in this situation) */
void Power_Up_Reset()
{
	bcm2835_gpio_set(RPI_GPIO_P1_24);
	bcm2835_gpio_clr(RPI_GPIO_P1_24);
	bcm2835_gpio_set(RPI_GPIO_P1_24);
	Reset();										// RESET_CC1100();
}

void Init() {
	CPUInit();
	Power_Up_Reset();
	//Reset();
	halRFWriteRFSettings();
	halSPIWriteBurstReg(CCxxx0_PATABLE, PaTable, 8);
}

/* halSPIWriteReg, Write value into a register on CC1100 */
void halSPIWriteReg(uint8_t addr, uint8_t value)
{
	while(bcm2835_gpio_lev(RPI_GPIO_P1_21));	// is there any need of while (MISO); ?
	char tbuf[2] = {addr, value};
	char rbuf[2] = {0};
	uint32_t len = 2;
	bcm2835_spi_transfernb(tbuf, rbuf, len);
}

/* halSPIWriteBurstReg, Write buffered value into a register */
void halSPIWriteBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count)
{
	uint8_t i, temp;									// INT8U i, temp;
	temp = addr | WRITE_BURST;						// temp = addr | WRITE_BURST;
	char tbuf[256] = {temp};
	char rbuf[256] = {0};
	for(int j = 0; j < count; j++)
	{
		tbuf[j + 1] = (*buffer);
		buffer++;
	}
	uint32_t len = count + 1;
	while(bcm2835_gpio_lev(RPI_GPIO_P1_21));	// is there any need of while (MISO); ?
	bcm2835_spi_transfernb(tbuf, rbuf, len);
}

/* halSPIStrobe, write through SPI */
void halSPIStrobe(uint8_t strobe)
{
	while(bcm2835_gpio_lev(RPI_GPIO_P1_21));	// is there any need of while (MISO); ?
	SPITxRxByte(strobe);							// SpiTxRxByte(strobe);
}

/* halSPIReadReg, Read values from a register on CC1100 */
uint8_t halSPIReadReg(uint8_t addr)
{
	uint8_t temp, value;
	temp = addr | READ_SINGLE;								// temp = addr|READ_SINGLE;
	while(bcm2835_gpio_lev(RPI_GPIO_P1_21));	// is there any need of while (MISO); ?
	char tbuf[2] = {temp, 0};
	char rbuf[2] = {0};
	uint32_t len = 2;
	bcm2835_spi_transfernb(tbuf, rbuf, len);
	value = rbuf[1];
	return value;									// return value;
}

/* halSPIReadBurstReg, Read values from CC1100 */
void halSPIReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count)
{
	uint8_t i, temp;
	temp = addr | READ_BURST;								// temp = addr | READ_BURST;
	char tbuf[256] = {temp};
	char rbuf[256] = {0};
	for(int j = 0; j < count; j++)
	{
		tbuf[j + 1] = 0;
	}
	uint32_t len = count + 1;
	while(bcm2835_gpio_lev(RPI_GPIO_P1_21));	// is there any need of while (MISO); ?
	bcm2835_spi_transfernb(tbuf, rbuf, len);
	for(int j = 0; j < count; j++)
	{
		buffer[j] = rbuf[j + 1];
	}
}

/* halSPIReadStatus, Read status from register */
uint8_t halSPIReadStatus(uint8_t addr)
{
	uint8_t value, temp;
	temp = addr | READ_BURST;								// temp = addr | READ_BURST;
	while(bcm2835_gpio_lev(RPI_GPIO_P1_21));	// is there any need of while (MISO); ?
	char tbuf[2] = {temp, 0};
	char rbuf[2] = {0};
	uint32_t len = 2;
	bcm2835_spi_transfernb(tbuf, rbuf, len);
	value = rbuf[1];
	return value;
}

void halRFWriteRFSettings()
{
	// Write register settings
	//halSPIWriteReg(CCxxx0_FSCTRL0, rfSettings.FSCTRL2);
	halSPIWriteReg(CCxxx0_FSCTRL1, rfSettings.FSCTRL1);
	halSPIWriteReg(CCxxx0_FSCTRL0, rfSettings.FSCTRL0);
	halSPIWriteReg(CCxxx0_FREQ2, rfSettings.FREQ2);
	halSPIWriteReg(CCxxx0_FREQ1, rfSettings.FREQ1);
	halSPIWriteReg(CCxxx0_FREQ0, rfSettings.FREQ0);
	halSPIWriteReg(CCxxx0_MDMCFG4, rfSettings.MDMCFG4);
	halSPIWriteReg(CCxxx0_MDMCFG3, rfSettings.MDMCFG3);
	halSPIWriteReg(CCxxx0_MDMCFG2, rfSettings.MDMCFG2);
	halSPIWriteReg(CCxxx0_MDMCFG1, rfSettings.MDMCFG1);
	halSPIWriteReg(CCxxx0_MDMCFG0, rfSettings.MDMCFG0);
	halSPIWriteReg(CCxxx0_CHANNR, rfSettings.CHANNR);
	halSPIWriteReg(CCxxx0_DEVIATN, rfSettings.DEVIATN);
	halSPIWriteReg(CCxxx0_FREND1, rfSettings.FREND1);
	halSPIWriteReg(CCxxx0_FREND0, rfSettings.FREND0);
	halSPIWriteReg(CCxxx0_MCSM0, rfSettings.MCSM0);
	halSPIWriteReg(CCxxx0_FOCCFG, rfSettings.FOCCFG);
	halSPIWriteReg(CCxxx0_BSCFG, rfSettings.BSCFG);
	halSPIWriteReg(CCxxx0_AGCCTRL2, rfSettings.AGCCTRL2);
	halSPIWriteReg(CCxxx0_AGCCTRL1, rfSettings.AGCCTRL1);
	halSPIWriteReg(CCxxx0_AGCCTRL0, rfSettings.AGCCTRL0);
	halSPIWriteReg(CCxxx0_FSCAL3, rfSettings.FSCAL3);
	halSPIWriteReg(CCxxx0_FSCAL2, rfSettings.FSCAL2);
	halSPIWriteReg(CCxxx0_FSCAL1, rfSettings.FSCAL1);
	halSPIWriteReg(CCxxx0_FSCAL0, rfSettings.FSCAL0);
	halSPIWriteReg(CCxxx0_FSTEST, rfSettings.FSTEST);
	halSPIWriteReg(CCxxx0_TEST2, rfSettings.TEST2);
	halSPIWriteReg(CCxxx0_TEST1, rfSettings.TEST1);
	halSPIWriteReg(CCxxx0_TEST0, rfSettings.TEST0);
	halSPIWriteReg(CCxxx0_IOCFG2, rfSettings.IOCFG2);
	halSPIWriteReg(CCxxx0_IOCFG0, rfSettings.IOCFG0);
	halSPIWriteReg(CCxxx0_PKTCTRL1, rfSettings.PKTCTRL1);
	halSPIWriteReg(CCxxx0_PKTCTRL0, rfSettings.PKTCTRL0);
	halSPIWriteReg(CCxxx0_ADDR, rfSettings.ADDR);
	halSPIWriteReg(CCxxx0_PKTLEN, rfSettings.PKTLEN);
}

void halRFSendPacket(uint8_t *txBuffer, uint8_t size)
{
	halSPIWriteReg(CCxxx0_TXFIFO, size);
	halSPIWriteBurstReg(CCxxx0_TXFIFO, txBuffer, size);

	halSPIStrobe(CCxxx0_STX);
	while (!bcm2835_gpio_lev(RPI_GPIO_P1_18));
	while (bcm2835_gpio_lev(RPI_GPIO_P1_18));
	halSPIStrobe(CCxxx0_SFTX);
}

void setRxMode()
{
	halSPIStrobe(CCxxx0_SRX);
}

uint8_t checkReceiveFlag() {
	if(bcm2835_gpio_lev(RPI_GPIO_P1_18)) {
		while (bcm2835_gpio_lev(RPI_GPIO_P1_18));
		return 1;
	} else {
		return 0;
	}
}

uint8_t halRFReceivePacket(uint8_t *rxBuffer, uint8_t *length)
{
	uint8_t status[2];
	uint8_t packetLength;
	uint8_t i = (*length) * 4;

	halSPIStrobe(CCxxx0_SRX);
	/*delay(2);
	while(bcm2835_gpio_lev(RPI_GPIO_P1_18))
	{
		delay(2);
		i--;
		if(i < 1)
			return 0;
	}*/
	if ((halSPIReadStatus(CCxxx0_RXBYTES) & BYTES_IN_RXFIFO))
	{
		packetLength = halSPIReadReg(CCxxx0_RXFIFO);
		//if (packetLength <= *length)
		//{
		halSPIReadBurstReg(CCxxx0_RXFIFO, rxBuffer, packetLength);
		*length = packetLength;

		halSPIReadBurstReg(CCxxx0_RXFIFO, status, 2);
		halSPIStrobe(CCxxx0_SFRX);
		//return (status[1] & CRC_OK);
		return packetLength;
		//}
		/*else
		{
			*length = packetLength;
			halSPIStrobe(CCxxx0_SFRX);
			return 0;
		}*/
	}
	else{
		halSPIStrobe(CCxxx0_SFRX);
		return 0;
	}
}

//------------------------------------------------------------
// TX OPTION CC1100发射数据 用comm――data切换频道
//-------------------------------------------------------------
void TxCCxx00()
{
	unsigned char	J,i;
//	halSpiWriteReg(CCxxx0_FREND0, 0x12);//CC1100:POWER=10DBM
	//Dly01mS(25);	
	J=0;
	for(i=0;i<50;i++)	//发2次		
		{
		halSPIWriteReg(CCxxx0_CHANNR, J);//channr
		//Dly01mS(25);
		halSPIStrobe(CCxxx0_STX);
		//while(comm_data==0);
		J=J+2;
		//while(comm_data==1);
//			Dly01mS(25);
		halSPIStrobe(CCxxx0_SIDLE);
		}
}

//------------------------------------------------------------
// RX OPTION TESTING 
//
//-------------------------------------------------------------
unsigned char RxCCxx00()
{
	unsigned char	H,i;
		H=0;
	for(i=0;i<50;i++)	//发2次		
		{	
		halSPIWriteReg(CCxxx0_CHANNR, H);//channr
		halSPIStrobe(CCxxx0_SRX);
		//while(comm_data==0);
		H=H+2;
		//while(comm_data==1);
//		Dly01mS(25);
		}
	halSPIStrobe(CCxxx0_SIDLE);	
}

//===============================================================
//TESTING  mode
//===============================================================
void TestingCCxx00()
{
	unsigned char rfstate; 		
		 //test_rf_status();
		halSPIWriteReg(CCxxx0_IOCFG0, 0x0d);//Asynchronous transparent mode.
		halSPIWriteReg(CCxxx0_IOCFG2, 0x0b);
		halSPIWriteReg(CCxxx0_MDMCFG2, 0x08);
		halSPIWriteReg(CCxxx0_PKTCTRL0, 0x32);//NOTE:  Asynchronous transparent mode.
			//if(comm_data=1)
	    		TxCCxx00();				//发射信息
			//else 
				RxCCxx00();				//收数据
/*		halSpiWriteReg(CCxxx0_PKTCTRL0, 0x44);//NOTE: ABOUT CRC FIFOs
		halSpiWriteReg(CCxxx0_IOCFG2, 0x07);	
		halSpiWriteReg(CCxxx0_IOCFG0, 0x06);
		halSpiWriteReg(CCxxx0_MDMCFG2, 0x19);*/			
}

//CC1100 cc1100;