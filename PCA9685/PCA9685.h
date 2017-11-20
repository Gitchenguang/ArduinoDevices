#include"Arduino.h"
#include"Wire.h"
//PCA9685 /OE端直接接地 

class PCA9685{
uint8_t Address;
uint8_t RegArray[256]={0};
double Freq;

public:
//一些public 变量
// 程序中有Led的函数可以认为是PCA9685的一个输出Port
	const uint8_t Mode1Reg = 0;
	const uint8_t Mode2Reg = 1;
	const uint8_t LedOffset = 6;
	const uint8_t LedSpace = 4;
	const uint8_t LedOn_L = 0; // 注意低位在低地址。
	const uint8_t LedOn_H = 1;
	const uint8_t LedOff_L = 2;
	const uint8_t LedOff_H = 3;

	const uint8_t LedFullOnMask = 0x10;
	const uint8_t LedFullOffMask = 0x10;

	const uint8_t Prescalar = 254;

	// Mode1 flags
	const uint8_t RestartMask = 0x80;
	const uint8_t EXTCLKMask = 0x40;
	const uint8_t AIMask = 0x20;
	const uint8_t SleepMask = 0x10;
	const uint8_t LowBitsDefault = 0x01; // Subaddr 1~3 disable allcall enable

	// Mode2 flags
	// bit 7-5 reserved, read only
	const uint8_t INVRTMask = 0x10;  // default is 0;
	const uint8_t OCHMask = 0x08; // default is 0; outputs change on stop command
	const uint8_t OUTDRVMask = 0x04; // default is 1:totem pole structure.     0:open-drain structure
	const uint8_t OUTEMask = 0x03;   // default 00: /OE=1,Output drivers not enabled. LEDn=0;其它情况和输出的驱动结构有关,具体看datasheet

public:
	PCA9685( uint8_t DevAddr );
	~PCA9685( );

	void PCA9685Init();
	
	uint8_t ReadReg( uint8_t TargetReg );
	void WriteReg( uint8_t TargetReg , uint8_t Dat );

	void WriteMode1( uint8_t Dat );
	uint8_t ReadMode1();

	void WriteMode2(uint8_t Dat );
	uint8_t ReadMode2();
	
	void WriteLed( uint8_t LedChannel , uint16_t LedOn , uint16_t LedOff );
	uint16_t ReadLed( uint8_t LedChannel , bool Status );

	void SetFreq( double f );
	double ReadFreq();
	
	void SetLedLow( uint8_t LedChannel );
	void SetLedHigh( uint8_t LedChannel );
};


