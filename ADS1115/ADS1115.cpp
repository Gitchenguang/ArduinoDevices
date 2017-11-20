#include"ADS1115.h"
ADS1115::ADS1115( uint8_t DevAddr = 0x48){
	Address = DevAddr;
	Channel = 0;
	DataRate = 64;
	SampleDelay = 1000/DataRate*3; //防止不同通道数据读乱，2.5倍采样间隔即可，这里取整数3
	ConfigInfo = (AN0SingleEnd&ChannelMask)|(PGAMask&Gain_4dot096)|(ModeMask&ContinousMode)|(DataRateMask&SPS64)|(CompModeMask&TraComp)|(CompLatMask&NonLat)|(CompQueMask&DisComp);
}
ADS1115::~ADS1115(){

}
void ADS1115::AdsBegin(){
	Wire.begin();
}
void ADS1115::ConfigAds1115( uint16_t AdsConfig ){
	ConfigInfo = AdsConfig;
	Wire.beginTransmission(Address);
	Wire.write(ConfigReg );
	Wire.write((uint8_t)((ConfigInfo&0xff00)>>8));
	Wire.write((uint8_t)(ConfigInfo&0x00ff));
	Wire.endTransmission();	
}
int16_t  ADS1115::ReadAds1115( uint8_t ch ){
	int16_t CurrentAdc = 0;
	uint16_t CurConfig = 0;
	uint16_t ConfigChannel = 0;
	if(ch>3)ch=3;
	if( Channel == ch ){
		Wire.beginTransmission( Address );
		Wire.write( ConversionReg );
		Wire.endTransmission();

		Wire.requestFrom( (int)Address , 2 );
		while (Wire.available()<2) { 
		}
		CurrentAdc = (int16_t)Wire.read();
		CurrentAdc = (CurrentAdc<<8)|((int16_t)Wire.read());
	}else{
		// 将通道设置为要读取的通道
		Channel = ch;
		ConfigChannel = ((uint16_t)Channel)<<12;
		ConfigChannel = ConfigChannel + AN0SingleEnd;		
		ConfigInfo = (ConfigInfo&(~ChannelMask))|(ConfigChannel&ChannelMask);
		
		Wire.beginTransmission(Address);
		Wire.write(ConfigReg );
		Wire.write((uint8_t)((ConfigInfo&0xff00)>>8));
		Wire.write((uint8_t)(ConfigInfo&0x00ff));
		Wire.endTransmission(); 

		delay( SampleDelay );

		// 读取对应通道的值
		Wire.beginTransmission( Address );
		Wire.write( ConversionReg );
		Wire.endTransmission();
		Wire.requestFrom( (int)Address , 2 );
		while (Wire.available()<2) { 
		}
		CurrentAdc = (int16_t)Wire.read();
		CurrentAdc = (CurrentAdc<<8)|((int16_t)Wire.read());
	}
	return CurrentAdc;
}

void ADS1115::SetDataRate( uint16_t DR ){	
}
uint16_t ADS1115::ReadConfigReg(){
	uint16_t CurrentConfig;
	// 读取对应通道的值
	Wire.beginTransmission( Address );
	Wire.write( ConfigReg);
	Wire.endTransmission();
	Wire.requestFrom( (int)Address , 2 );
	while (Wire.available()<2) { 
	}
	CurrentConfig = (uint16_t)Wire.read();
	CurrentConfig = (CurrentConfig<<8)|((uint16_t)Wire.read());
	return CurrentConfig;
}


