#ifndef ADS1115_H
#define ADS1115_H
#include"Arduino.h"
#include"Wire.h"

class ADS1115{
	uint8_t Channel;
	uint8_t Address;
	uint16_t ConfigInfo;
	uint16_t DataRate;
	unsigned int SampleDelay;
public:
	// 涉及到的Private 变量
	const uint8_t ConversionReg = 0x00;
	const uint8_t ConfigReg = 0x01;
	const uint8_t LoThreshReg = 0x02;
	const uint8_t HiThreshReg = 0x03;

	// Function masks
	const uint16_t OSMask = 0x8000;
	const uint16_t ChannelMask = 0x7000;
	const uint16_t PGAMask = 0x0E00;
	const uint16_t ModeMask = 0x0100;
	const uint16_t DataRateMask = 0x00E0;
	const uint16_t CompModeMask = 0x0010;
	const uint16_t CompPolMask = 0x0008;
	const uint16_t CompLatMask = 0x0004;
	const uint16_t CompQueMask = 0x0003;

	// Configurations
	//-Channel multiplexer
	const uint16_t	AN0SingleEnd = 0x4000;
	const uint16_t	AN1SingleEnd = 0x5000;
	const uint16_t	AN2SingleEnd = 0x6000;
	const uint16_t	AN3SingleEnd = 0x7000;
	const uint16_t	AN0AN1 = 0x0000;   //default
	const uint16_t	AN0AN3 = 0x1000;
	const uint16_t	AN1AN3 = 0x2000;
	const uint16_t	AN2AN3 = 0x3000;

	// Gain 除了下面的部分，其它均为+/-0.256V范围
	const uint16_t Gain_6dot144 = 0x0000;
	const uint16_t Gain_4dot096 = 0x0200;
	const uint16_t Gain_2dot048 = 0x0400;//default
	const uint16_t Gain_1dot024 = 0x0600;
	const uint16_t Gain_dot512 = 0x0800;
	const uint16_t Gain_dot256 = 0x0A00;

	// Mode  Continuous or single-shot
	const uint16_t ContinousMode = 0x0000;
	const uint16_t Singleshot = 0x0100;//default

	// Data rate
	const uint16_t	 SPS8 = 0x0000;
	const uint16_t	 SPS16 = 0x0020;
	const uint16_t	 SPS32 = 0x0040;
	const uint16_t	 SPS64 = 0x0060;
	const uint16_t	 SPS128 = 0x0080;//default
	const uint16_t	 SPS250 = 0x00A0;
	const uint16_t	 SPS475 = 0x00C0;
	const uint16_t	 SPS860 = 0x00E0;

	// Compare mode
	const uint16_t TraComp = 0x0000;//default
	const uint16_t WinComp = 0x0010;

	// Compare polarity
	const uint16_t ActiveLow = 0x0000;//default
	const uint16_t ActiveHigh = 0x0008;

	// Compare latching comparator
	const uint16_t NonLat = 0x0000;//default
	const uint16_t Lat = 0x0004;

	// Comparator queue and disable
	const uint16_t AssAfterOne = 0x0000;
	const uint16_t AssAfterTwo = 0x0001;
	const uint16_t AssAfterFour = 0x0002;
	const uint16_t DisComp = 0x0003;//default
	
public:
	ADS1115( uint8_t DevAddr);
	~ADS1115();
	void AdsBegin();
	void ConfigAds1115( uint16_t AdsConfig );
	int16_t ReadAds1115( uint8_t ch );
	void SetDataRate( uint16_t DR );
	uint16_t ReadConfigReg();
	
};
#endif
