#include"TECDriver.h"

bool TECDriver::initflag = false;

TECDriver::TECDriver( uint8_t channel0 , uint8_t channel1):Ch0(channel0),Ch1(channel1){
	flag = 0; //0:空闲状态 1:制冷，2:加热  -1:刹车
	defaultMinDur = 62; // 以默认的500Hz的pwm频率算，最短高电平时间不超过30us   30/[1000000us/(500*4096)]
	defaultMaxDur = 4000;
	PwmController = new PCA9685(0x40);
}
TECDriver::~TECDriver(){

}
void TECDriver::Init(){
	if( initflag == false ){		
		initflag = true;
		PwmController->PCA9685Init();
		PwmController->WriteLed(15,0,2048); // 开启一路PWM防止其进入睡眠状态		
	}
	Stop();
}
void TECDriver::Idle(){
	PwmController->SetLedLow(Ch0);
	PwmController->SetLedLow(Ch1);
	flag = 0;	
	delay(100);
}
void TECDriver::Stop(){
	// 注意，Ch0必须是P口，且刹车时停止时必须先提升P口，刹车停止后需要延时100ms，使用注意事项有提
	PwmController->SetLedHigh(Ch0);
	PwmController->SetLedHigh(Ch1);
	flag = -1;
	delay(100);
}
void TECDriver::Cooling( double duration ){
	//正转  P=1 D=pwm : 制冷
	if( duration < defaultMinDur )duration = defaultMinDur;
	if( duration > defaultMaxDur) duration = defaultMaxDur;  //最大值4095，这里选4000就可以，当然也可以选其它上限值
	
	if( flag == 1 || flag == -1 ){
		PwmController->WriteLed( Ch1 , 0 , duration );
	}else{
		Stop();
		PwmController->WriteLed( Ch1 , 0 , duration );
	}	
	flag = 1;
}
void TECDriver::Heating( double duration ){
	//反转  P=pwm D=1 : 制热
	if( duration < defaultMinDur )duration = defaultMinDur;
	if( duration > defaultMaxDur) duration = defaultMaxDur;  //最大值4095，这里选4000就可以，当然也可以选其它上限值	
	
	if( flag == 2 || flag == -1){
		PwmController->WriteLed( Ch0 , 0 , (uint16_t)duration );
	}else{
		Stop();
		PwmController->WriteLed( Ch0 , 0 , (uint16_t)duration );
	}
	flag = 2;	
}


