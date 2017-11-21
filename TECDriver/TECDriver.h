#ifndef TECDriver_H
#define TECDriver_H

#include"Arduino.h"
#include"../PCA9685/PCA9685.h"

class TECDriver{
	uint8_t Ch0, Ch1;
	static bool initflag;
	int8_t flag = 0;
	PCA9685 *PwmController;
	double defaultMinDur;
	double defaultMaxDur;
public:
	TECDriver( uint8_t channel0 , uint8_t channel1);
	~TECDriver();
	void Init();
	void Idle();
	void Stop();
	void Cooling( double duration );
	void Heating( double duration );

};

#endif
