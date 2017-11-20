#include"PCA9685.h"

uint8_t PCA9685::RegArray[256]={0};

PCA9685::PCA9685(uint8_t DevAddr = 0x40 ){
	Address = DevAddr;
	Freq = 500.0;
}
PCA9685::~PCA9685( ){

}

void PCA9685::PCA9685Init(){
	uint8_t i = 0;
	Wire.begin();
	
	RegArray[Mode1Reg] = 0x20; //Restart disable; Internal clock; Normal mode:Sleep disable;AI enable; subaddr diable; all-call addr disable;
	RegArray[Mode2Reg] = 0x06; // Output change on ACK; OUTDRV:totem;
	RegArray[Prescalar] = (uint8_t)(25000000.0/(4096.0*Freq)); // 设置频率为500Hz

	Wire.beginTransmission( Address );
	Wire.write( Mode1Reg );
	Wire.write( RegArray[Mode1Reg] );
	Wire.write( RegArray[Mode2Reg] );
	Wire.endTransmission();

	// 单次写64个Reg不知为啥写不了，可能是Arduino Wire 库函数的原因
	for( i=0; i<16; i++ ){		
		Wire.beginTransmission( Address );
		Wire.write( LedOffset + i*LedSpace+ LedOn_L );
		Wire.write( RegArray[ LedOffset + i*LedSpace+ LedOn_L ] );
		Wire.write( RegArray[ LedOffset + i*LedSpace + LedOn_H] );
		Wire.write( RegArray[ LedOffset + i*LedSpace + LedOff_L] );
		Wire.write( RegArray[ LedOffset + i*LedSpace + LedOff_H] );
		Wire.endTransmission();
	}

	RegArray[Mode1Reg] = 0x30; // 进入睡眠状态以更改频率
	WriteReg(Mode1Reg,RegArray[Mode1Reg]);

	WriteReg(Prescalar,RegArray[Prescalar]);

	RegArray[Mode1Reg] = 0x20; // 唤醒
	WriteReg(Mode1Reg,RegArray[Mode1Reg]);	
	
}

uint8_t PCA9685::ReadReg( uint8_t TargetReg ){
	Wire.beginTransmission( Address );
	Wire.write( TargetReg );
	Wire.endTransmission();	

	Wire.requestFrom( (int)Address , (int)1 );
	while( !Wire.available() ){

	}
	return( Wire.read() );
}
void PCA9685::WriteReg( uint8_t TargetReg , uint8_t Dat ){
	RegArray[ TargetReg ] = Dat;
	Wire.beginTransmission( Address );
	Wire.write( TargetReg );
	Wire.write( RegArray[ TargetReg ] );
	Wire.endTransmission();		
}

void PCA9685::WriteMode1( uint8_t Dat ){
	WriteReg(Mode1Reg, Dat);
}
uint8_t PCA9685::ReadMode1(){
	return( RegArray[Mode1Reg] );
}

void PCA9685::WriteMode2(uint8_t Dat ){
	WriteReg(Mode2Reg, Dat);
}
uint8_t PCA9685::ReadMode2(){
	return( RegArray[Mode2Reg] );
}

void PCA9685::WriteLed( uint8_t LedChannel , uint16_t LedOn , uint16_t LedOff ){
	uint8_t i;
	uint8_t LedIndex = LedOffset + LedChannel*LedSpace;
	RegArray[ LedIndex + LedOn_L ] = (uint8_t)(LedOn&0x00ff);
	RegArray[ LedIndex + LedOn_H ] = (uint8_t)((LedOn&0xff00)>>8);	
	RegArray[ LedIndex + LedOff_L ] = (uint8_t)(LedOff&0x00ff);
	RegArray[ LedIndex + LedOff_H ] = (uint8_t)((LedOff&0xff00)>>8);	
	
	Wire.beginTransmission( Address );
	Wire.write( LedIndex );
	for( i = LedIndex + LedOn_L;  i<(LedIndex+LedSpace); i++){
		Wire.write( RegArray[i]);
	}
	Wire.endTransmission();		
}
uint16_t PCA9685::ReadLed( uint8_t LedChannel , bool Status ){
	uint8_t LedIndex = LedOffset + LedChannel*LedSpace;
	if( Status == true ){
		return( (((uint16_t)RegArray[LedIndex + LedOn_H])<<8) + (uint16_t)RegArray[LedIndex + LedOn_L] );
	}else{
		return( (((uint16_t)RegArray[LedIndex + LedOff_H])<<8) + (uint16_t)RegArray[LedIndex + LedOff_L] );
	}
}

void PCA9685::SetFreq( double f ){
	Freq = f;
	RegArray[Prescalar] = (uint8_t)(25000000.0/(4096.0*Freq)); // 设置频率为500Hz
	uint8_t restartflag = 0;
    // 进入睡眠状态以更改频率
	WriteReg(Mode1Reg,RegArray[Mode1Reg]|SleepMask);
	WriteReg(Prescalar,RegArray[Prescalar]);
	// Clear Sleep bit	
	WriteReg(Mode1Reg,RegArray[Mode1Reg]);
	delay(1);

	Wire.beginTransmission( Address );
	Wire.write( Mode1Reg );
	Wire.endTransmission();	

	Wire.requestFrom( (int)Address , (int)1 );
	while( !Wire.available() ){

	}
	restartflag = Wire.read();
	if( restartflag & RestartMask ){ 
		WriteReg(Mode1Reg,RegArray[Mode1Reg]| RestartMask);
	}
	
}
double PCA9685::ReadFreq(){
	ReadReg( Prescalar );
	return( 25000000.0/(4096.0*((double)RegArray[Prescalar])));
}

void PCA9685::SetLedLow( uint8_t LedChannel ){
	uint8_t i = 0;
	uint8_t LedIndex = LedOffset + LedChannel*LedSpace;
	RegArray[ LedIndex + LedOn_H ] = RegArray[ LedIndex + LedOn_H ] & (~LedFullOnMask) ;
	RegArray[ LedIndex + LedOff_H ] = RegArray[ LedIndex + LedOff_H ] | LedFullOffMask ;
	Wire.beginTransmission( Address );
	Wire.write( LedIndex );
	for( i = LedIndex + LedOn_L;  i<(LedIndex+LedSpace); i++){
		Wire.write( RegArray[i]);
	}
	Wire.endTransmission();		

}

void PCA9685::SetLedHigh( uint8_t LedChannel ){
	uint8_t i = 0;
	uint8_t LedIndex = LedOffset + LedChannel*LedSpace;
	RegArray[ LedIndex + LedOn_H ] = RegArray[ LedIndex + LedOn_H ]|LedFullOnMask;
	RegArray[ LedIndex + LedOff_H ] = RegArray[ LedIndex + LedOff_H ] & (~LedFullOffMask) ;
	Wire.beginTransmission( Address );
	Wire.write( LedIndex );
	for( i = LedIndex + LedOn_L;  i<(LedIndex+LedSpace); i++){
		Wire.write( RegArray[i]);
	}
	Wire.endTransmission();		

}


