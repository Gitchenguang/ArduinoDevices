#include"TECDriver.h"

bool TECDriver::initflag = false;

TECDriver::TECDriver( uint8_t channel0 , uint8_t channel1):Ch0(channel0),Ch1(channel1){
	flag = 0; //0:����״̬ 1:���䣬2:����  -1:ɲ��
	defaultMinDur = 62; // ��Ĭ�ϵ�500Hz��pwmƵ���㣬��̸ߵ�ƽʱ�䲻����30us   30/[1000000us/(500*4096)]
	defaultMaxDur = 4000;
	PwmController = new PCA9685(0x40);
}
TECDriver::~TECDriver(){

}
void TECDriver::Init(){
	if( initflag == false ){		
		initflag = true;
		PwmController->PCA9685Init();
		PwmController->WriteLed(15,0,2048); // ����һ·PWM��ֹ�����˯��״̬		
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
	// ע�⣬Ch0������P�ڣ���ɲ��ʱֹͣʱ����������P�ڣ�ɲ��ֹͣ����Ҫ��ʱ100ms��ʹ��ע����������
	PwmController->SetLedHigh(Ch0);
	PwmController->SetLedHigh(Ch1);
	flag = -1;
	delay(100);
}
void TECDriver::Cooling( double duration ){
	//��ת  P=1 D=pwm : ����
	if( duration < defaultMinDur )duration = defaultMinDur;
	if( duration > defaultMaxDur) duration = defaultMaxDur;  //���ֵ4095������ѡ4000�Ϳ��ԣ���ȻҲ����ѡ��������ֵ
	
	if( flag == 1 || flag == -1 ){
		PwmController->WriteLed( Ch1 , 0 , duration );
	}else{
		Stop();
		PwmController->WriteLed( Ch1 , 0 , duration );
	}	
	flag = 1;
}
void TECDriver::Heating( double duration ){
	//��ת  P=pwm D=1 : ����
	if( duration < defaultMinDur )duration = defaultMinDur;
	if( duration > defaultMaxDur) duration = defaultMaxDur;  //���ֵ4095������ѡ4000�Ϳ��ԣ���ȻҲ����ѡ��������ֵ	
	
	if( flag == 2 || flag == -1){
		PwmController->WriteLed( Ch0 , 0 , (uint16_t)duration );
	}else{
		Stop();
		PwmController->WriteLed( Ch0 , 0 , (uint16_t)duration );
	}
	flag = 2;	
}


