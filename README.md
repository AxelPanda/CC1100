# CC1101 Library
## for 8051 MCU
### usage
```cpp
#include "CC1100.c"	// Place CC1100.c in your work directory.
```
### initialization
```cpp
void CpuInit();	// SPI and CPU initialization are included.
void POWER_UP_RESET_CC1100();	// RESET instruction is sent through SPI.
void halRfWriteRfSettings();	// Congigure CC1100 registers.
void halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8);	// Write in RF Power settings.
```
### send packet
```cpp
void halRfSendPacket(INT8U *txBuffer, INT8U size);	// The content you want to send is stored where pointer *txBuffer points at and the length of the message is given by size.
```
### enter receiving mode
```cpp
void setRxMode();	// Call this function immediately after sending packet(s) or you will be unable to receive anything.
```
### check receiving flag
Read **GDO0**. If **GDO0** is high or true, there's something to be received, otherwise low or false.
### receive packet
```cpp
INT8U halRfReceivePacket(INT8U *rxBuffer, INT8U *length);	// The packet you receive will be stored where pointer *rxBuffer points at and the size of the packet will be stored in *length.
// NOTE: DO GIVE A LARGE ENOUGH *length, OR YOUR PACKET WILL BE FLUSHED!
```

## for RaspberryPi
**NOTE:**  BCM2835 (a library by [mikem@airspayce](http://www.airspayce.com/mikem/bcm2835/)) is required.
### usage
```cpp
#include "CC1100.h"	// Place CC1100.h and CC1100.cpp in your work directory.
```
### initialization
```cpp
void Init();	// CC1100/CC1101 is fully configured by calling this function, including SPI, CPU, executing POWER_UP_RESET_INSTRUCTION and configuring all the registers and RF power settings.
```
### send packet
```cpp
void halRFSendPacket(uint8_t *txBuffer, uint8_t size);	// The content you want to send is stored in the memory where is pointed by *txBuffer and size is the lengt you want to send.
```
### enter receiving mode
```cpp
void setRxMode();	// Call this immediately after sending packet(s) or you will be unable to receive anything until you call it.
```
### check receiving flag
```cpp
uint8_t checkReceiveFlag();	// Check whether there is any packet to be received.
```
### receive packet
```cpp
uint8_t halRFReceivePacket(uint8_t *rxBuffer, uint8_t *length);	// Receiving packet and storing it where the pointer *rxBuffer points at, and the length of the packet you received will be stored in *length as well as the returning value.
// NOTE: DO NOT USE THIS FUNCTION TO CHECK IF THERE IS PACKET TO BE RECEIVED!
```