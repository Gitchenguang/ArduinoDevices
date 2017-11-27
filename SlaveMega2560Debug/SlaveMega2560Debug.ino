#include <EEPROM.h>
#include <PID_v1.h>
#include <Wire.h>
#include <ADS1115.h>
#include <PCA9685.h>
#include <TECDriver.h>

#define DEBUG

// 系统全局变量
unsigned char SysState = 0;
unsigned char lastSysState=0;
const unsigned char Cell=0,Prism=1,WaterBox=2;
double PIDparamScale = 65536;
int16_t ADC_Cell=0;double CellTempValue=0;
int16_t ADC_Prism=0;double PrismTempValue=0;
int16_t ADC_WaterBox = 0; double WaterBoxTempValue=0;
//******蠕动泵********** Slave2560的31脚为pump的tone输出
unsigned int Pump = 49; 
unsigned int Speed=50;
double SpeedScale = 100.0/17.0;
//
////********ADS1115温度采集*******
//// 占用Uno的 A4 A5脚  另外增益设定为+/-4.096，所以进行量化时注意换算问题,不过换算最好在PC端，然后传过来只有ADC的值最好
int DevAddr = 0x48;
ADS1115 MyAds1115( DevAddr );
uint16_t AdsConfig=(MyAds1115.AN0SingleEnd&MyAds1115.ChannelMask)|(MyAds1115.PGAMask&MyAds1115.Gain_4dot096)|(MyAds1115.ModeMask&MyAds1115.ContinousMode)|(
  MyAds1115.DataRateMask&MyAds1115.SPS64)|(MyAds1115.CompModeMask&MyAds1115.TraComp)|(MyAds1115.CompLatMask&MyAds1115.NonLat)|(MyAds1115.CompQueMask&MyAds1115.DisComp);
//
////*******温度控制部分 配置与变量声明******
unsigned char CellD1 = 0, CellD2 = 1;
unsigned char PrismD1=2,PrismD2=3; 
unsigned char WaterBox1 = 4, WaterBox2 = 5;
double PIDOutMax=255.0;
unsigned char Kp=0,Ki=1,Kd=2;
const uint8_t ThreshHoldScale = 8;
const double Scale = 4096.0/32768.0; // 确定PID输入变量的单位，1000：输入变量为mV, ADS1115单端的话为0-32767
const double pwmScale = 16.0; // PCA9685的pwm为12bit，相对mega2560的8bit，其范围大了16倍

TECDriver CellTempDriver( CellD1, CellD2  );   //Tedriver 本来应该为TECdriver，这里把c去掉了
TECDriver PrismTempDriver( PrismD1 , PrismD2 );
TECDriver WaterBoxTempDriver( WaterBox1, WaterBox2 );
//---CellPID parameters
// eeprom存储信息
unsigned int eeAddress =0 ;
uint8_t CellAdaptiveFlag = 0;
uint16_t CellGap = 100;  //默认的gap
uint16_t CellThreshHold = 1;
double CellThreshHoldDouble = CellThreshHold/double(ThreshHoldScale);  //8 ADS1115对于电压的最小分辨率为0.125mV，PID的输入输出以mV为单位，这里除以8即可
uint16_t CellGivenADC = 19920; 
double CellPIDParams_Heating[3] = { 2.7 , 0.001 ,0.008 };
double CellPIDParams_Cooling[3] = { 2.7 , 0.001 ,0.008 };
double CellPIDAggParams_Heating[3] = { 2.7 , 0.0001 ,0.001 };
double CellPIDAggParams_Cooling[3] = { 2.7 , 0.0001 ,0.001 };
double CellPIDInput,CellPIDOutput_Cooling,CellPIDOutput_Heating, CellSetPoint = (double)(CellGivenADC*Scale);
PID CellPID_Heating(&CellPIDInput, &CellPIDOutput_Heating, &CellSetPoint, CellPIDParams_Heating[Kp], CellPIDParams_Heating[Ki], CellPIDParams_Heating[Kd], DIRECT);
PID CellPID_Cooling(&CellPIDInput, &CellPIDOutput_Cooling, &CellSetPoint, CellPIDParams_Cooling[Kp], CellPIDParams_Cooling[Ki], CellPIDParams_Cooling[Kd], REVERSE);
//---PrismPID Parameters
uint8_t PrismAdaptiveFlag= 0;
uint16_t PrismGap =100;
uint16_t PrismThreshHold = 1;
double PrismThreshHoldDouble = PrismThreshHold/double(ThreshHoldScale);
uint16_t PrismGivenADC = 19920;
double PrismPIDParams_Heating[3] = { 2.7 , 0.001 ,0.008 };
double PrismPIDParams_Cooling[3] = { 2.7 , 0.001 ,0.008 };
double PrismPIDAggParams_Heating[3] = { 2.7 , 0.001 ,0.001 };
double PrismPIDAggParams_Cooling[3] = { 2.7 , 0.001 ,0.001 };
double PrismPIDInput,PrismPIDOutput_Cooling,PrismPIDOutput_Heating, PrismSetPoint=(double)(PrismGivenADC*Scale);
PID PrismPID_Heating(&PrismPIDInput, &PrismPIDOutput_Heating, &PrismSetPoint, PrismPIDParams_Heating[Kp], PrismPIDParams_Heating[Ki], PrismPIDParams_Heating[Kd], DIRECT);
PID PrismPID_Cooling(&PrismPIDInput, &PrismPIDOutput_Cooling, &PrismSetPoint, PrismPIDParams_Cooling[Kp], PrismPIDParams_Cooling[Ki], PrismPIDParams_Cooling[Kd], REVERSE);

// WaterBoxPID Parameters(注：对于水槽温控的设置，这里将其与流室/棱镜部分的设置进行了分离，单独进行控制)
// WaterBox的工作方式与 Cell 和 Prism不同，WaterBox拥有自己的启动标志位进行温度控制的启停，不和Cell还有Prism一同受控制
bool WaterBoxWorkingFlag = false;
uint8_t WaterBoxAdaptiveFlag = 0;
uint16_t WaterBoxGap = 100;
uint16_t WaterBoxThreshHold = 1;
uint16_t WaterBoxGivenADC = 19920;
double WaterBoxThreshHoldDouble = WaterBoxThreshHold/double( ThreshHoldScale );
double WaterBoxPIDParams_Heating[3] = { 2.7 , 0.001 ,0.008 };
double WaterBoxPIDParams_Cooling[3] = { 2.7 , 0.001 ,0.008 };
double WaterBoxPIDAggParams_Heating[3] = { 2.7 , 0.001 ,0.001 };
double WaterBoxPIDAggParams_Cooling[3] = { 2.7 , 0.001 ,0.001 };
double WaterBoxPIDInput,WaterBoxPIDOutput_Cooling,WaterBoxPIDOutput_Heating, WaterBoxSetPoint=(double)(WaterBoxGivenADC*Scale);
PID WaterBoxPID_Heating(&WaterBoxPIDInput, &WaterBoxPIDOutput_Heating, &WaterBoxSetPoint, WaterBoxPIDParams_Heating[Kp], WaterBoxPIDParams_Heating[Ki], WaterBoxPIDParams_Heating[Kd], REVERSE);
PID WaterBoxPID_Cooling(&WaterBoxPIDInput, &WaterBoxPIDOutput_Cooling, &WaterBoxSetPoint, WaterBoxPIDParams_Cooling[Kp], WaterBoxPIDParams_Cooling[Ki], WaterBoxPIDParams_Cooling[Kd], DIRECT);

// PID设置参数索引变量
const uint8_t CellPIDParamsAddr = 0;
const uint8_t PrismPIDParamsAddr = 4*sizeof( CellPIDParams_Heating );
const uint8_t WaterBoxPIDParamsAddr= 8*sizeof( CellPIDParams_Heating );
const uint8_t PIDAdaptiveFlagAddr = 12*sizeof( CellPIDParams_Heating );
const uint8_t PIDGapAddr = 12*sizeof( CellPIDParams_Heating ) + 3*sizeof( CellAdaptiveFlag );
const uint8_t PIDThreshHoldAddr = 12*sizeof( CellPIDParams_Heating ) + 3*sizeof( CellAdaptiveFlag ) + 3*sizeof(CellThreshHold);

void WaterBoxTempReg( bool flag );

void setup() {

// EEPROM：//有历史信息就读取，没有的话就存放
  double tmp;
  tmp = EEPROM.get(0,tmp);
  unsigned int add=0;
  if( tmp== 0.0 ){
      EEPROM.put(add,CellPIDParams_Heating);
      add+=sizeof(CellPIDParams_Heating);
      EEPROM.put(add,CellPIDParams_Cooling);
      add+=sizeof(CellPIDParams_Cooling);
      EEPROM.put(add,CellPIDAggParams_Heating);
      add+=sizeof(CellPIDAggParams_Heating);
      EEPROM.put(add,CellPIDAggParams_Cooling );
      add+=sizeof(CellPIDAggParams_Cooling);
  
      EEPROM.put(add,PrismPIDParams_Heating);
      add+=sizeof(PrismPIDParams_Heating);
      EEPROM.put(add,PrismPIDParams_Cooling);
      add+=sizeof(PrismPIDParams_Cooling);
      EEPROM.put(add,PrismPIDAggParams_Heating);
      add+=sizeof(PrismPIDAggParams_Heating);
      EEPROM.put(add,PrismPIDAggParams_Cooling );      
      add+=sizeof(PrismPIDParams_Cooling);

      EEPROM.put(add,WaterBoxPIDParams_Heating);
      add+=sizeof(WaterBoxPIDParams_Heating);
      EEPROM.put(add,WaterBoxPIDParams_Cooling);
      add+=sizeof(WaterBoxPIDParams_Cooling);
      EEPROM.put(add,WaterBoxPIDAggParams_Heating);
      add+=sizeof(WaterBoxPIDAggParams_Heating);
      EEPROM.put(add,WaterBoxPIDAggParams_Cooling );      
      add+=sizeof( WaterBoxPIDAggParams_Cooling );
 // 零散设置     
      EEPROM.put( add ,CellAdaptiveFlag);
      add+=sizeof( CellAdaptiveFlag );
      EEPROM.put( add , PrismAdaptiveFlag );
      add+=sizeof( PrismAdaptiveFlag );
      EEPROM.put( add ,WaterBoxAdaptiveFlag);
      add+=sizeof( WaterBoxAdaptiveFlag );
      
      EEPROM.put( add , CellGap );
      add+=sizeof( CellGap );
      EEPROM.put( add ,PrismGap );
      add+=sizeof( PrismGap );
      EEPROM.put( add , WaterBoxGap );
      add+=sizeof( WaterBoxGap );
      
      EEPROM.put( add , CellThreshHold );
      add+=sizeof( PrismThreshHold );
      EEPROM.put( add , PrismThreshHold );
      add+= sizeof(  PrismThreshHold );
      EEPROM.put( add , WaterBoxThreshHold  );
  }else{
      EEPROM.get(add,CellPIDParams_Heating);
      add+=sizeof(CellPIDParams_Heating);
      EEPROM.get( add , CellPIDParams_Cooling );
      add+=sizeof(CellPIDParams_Cooling);
      EEPROM.get( add ,CellPIDAggParams_Heating );
      add+=sizeof( CellPIDAggParams_Heating );
      EEPROM.get( add , CellPIDAggParams_Cooling);
      add+=sizeof( CellPIDAggParams_Cooling );
      
      EEPROM.get(add,PrismPIDParams_Heating);
      add+=sizeof(PrismPIDParams_Heating);
      EEPROM.get(add,PrismPIDParams_Cooling);
      add+=sizeof(PrismPIDParams_Cooling);
      EEPROM.get(add,PrismPIDAggParams_Heating);
      add+=sizeof(PrismPIDAggParams_Heating);
      EEPROM.get(add,PrismPIDAggParams_Cooling );      
      add+=sizeof( PrismPIDAggParams_Cooling );
      
      EEPROM.get(add,WaterBoxPIDParams_Heating);
      add+=sizeof(WaterBoxPIDParams_Heating);
      EEPROM.get(add,WaterBoxPIDParams_Cooling);
      add+=sizeof(WaterBoxPIDParams_Cooling);
      EEPROM.get(add,WaterBoxPIDAggParams_Heating);
      add+=sizeof(WaterBoxPIDAggParams_Heating);
      EEPROM.get(add,WaterBoxPIDAggParams_Cooling );      
      add+=sizeof( WaterBoxPIDAggParams_Cooling );
 // 零散设置     
      EEPROM.get( add ,CellAdaptiveFlag);
      add+=sizeof( CellAdaptiveFlag );
      EEPROM.get( add , PrismAdaptiveFlag );
      add+=sizeof( PrismAdaptiveFlag );
      EEPROM.get( add ,WaterBoxAdaptiveFlag);
      add+=sizeof( WaterBoxAdaptiveFlag );
      
      EEPROM.get( add , CellGap );
      add+=sizeof( CellGap );
      EEPROM.get( add ,PrismGap );
      add+=sizeof( PrismGap );
      EEPROM.get( add , WaterBoxGap );
      add+=sizeof( WaterBoxGap );
      
      EEPROM.get( add, CellThreshHold );
      add+=sizeof( PrismThreshHold );
      EEPROM.get( add , PrismThreshHold );
      add+= sizeof(  PrismThreshHold );
      EEPROM.get( add , WaterBoxThreshHold  );     
  }
  CellThreshHoldDouble =CellThreshHold/double(ThreshHoldScale);
  PrismThreshHoldDouble = PrismThreshHold/double(ThreshHoldScale);
  WaterBoxThreshHoldDouble = WaterBoxThreshHold/double(ThreshHoldScale);
  // 初始设置就为保守值即可
  CellPID_Heating.SetTunings( CellPIDParams_Heating[Kp], CellPIDParams_Heating[Ki], CellPIDParams_Heating[Kd] );
  CellPID_Cooling.SetTunings( CellPIDParams_Cooling[Kp], CellPIDParams_Cooling[Ki], CellPIDParams_Cooling[Kd] );
  PrismPID_Heating.SetTunings( PrismPIDParams_Heating[Kp], PrismPIDParams_Heating[Ki], PrismPIDParams_Heating[Kd] );
  PrismPID_Cooling.SetTunings( PrismPIDParams_Cooling[Kp], PrismPIDParams_Cooling[Ki], PrismPIDParams_Cooling[Kd] );
  WaterBoxPID_Heating.SetTunings( WaterBoxPIDParams_Heating[Kp], WaterBoxPIDParams_Heating[Ki], WaterBoxPIDParams_Heating[Kd] );
  WaterBoxPID_Cooling.SetTunings( WaterBoxPIDParams_Cooling[Kp], WaterBoxPIDParams_Cooling[Ki], WaterBoxPIDParams_Cooling[Kd] );
  //Modules:
  pinMode( Pump , OUTPUT );
  Serial.begin(115200);
  MyAds1115.AdsBegin();
  MyAds1115.ConfigAds1115(AdsConfig);
  
  ADC_Cell = MyAds1115.ReadAds1115(Cell);
  CellPIDInput= (double)( ADC_Cell*Scale);
  ADC_Prism = MyAds1115.ReadAds1115(Prism);
  PrismPIDInput = (double)( ADC_Prism*Scale);
  ADC_WaterBox = MyAds1115.ReadAds1115( WaterBox );
  WaterBoxPIDInput = (double)( ADC_WaterBox*Scale );
  
  CellPID_Heating.SetMode(AUTOMATIC);
  CellPID_Cooling.SetMode(AUTOMATIC);
  PrismPID_Heating.SetMode(AUTOMATIC);
  PrismPID_Cooling.SetMode(AUTOMATIC);
  WaterBoxPID_Heating.SetMode(AUTOMATIC);
  WaterBoxPID_Cooling.SetMode(AUTOMATIC);
  
  CellTempDriver.Init();
  PrismTempDriver.Init();
  WaterBoxTempDriver.Init();
  
  CellTempDriver.Idle();
  PrismTempDriver.Idle();
  WaterBoxTempDriver.Idle();
}

void loop() {
      // SysState定义  0：空闲状态，关闭所有控温  
      // 设置状态：    1： 设置流速（流速为0即为关闭蠕动泵）     （此类状态在串口命令读取后立即执行）
      //               2： 同步设定温度 
      //               3:  异步设定温度 
      //               40: 水箱（ WaterBox）温度设定
      // 单次运行状态：4： Cell&Prism 查询温度  （此类状态在主循环中仅运行一次，然后恢复系统状态）
      //               5:  NULL
      //               11：设定 指定器件（Cell/Prism/WaterBox） PID参数Kp Ki Kd （****唯一可以设定三者的指令*****）
      //               12：打印指定PID参数（Cell/Prism/WaterBox）（****唯一可以读取三者的指令*****）
      //               13：设定Cell&Prism AdaptiveFlag 
      //               14：设定Cell&Prism Gap值  
      //               15：读取Cell & Prism AdaptiveFlag
      //               16：读取Cell & Prism Gap值
      //               17: NULL 
      //               18：设定Cell & Prism PIDthreshold
      //               19: 读取Cell & Prism ThreshHold值
      
       //              41：水箱温度查询
      //               42：设定WaterBox的WaterBoxAdaptiveFlag
      //               43：读取WaterBox的WaterBoxAdaptiveFlag
       //              44：设定WaterBox的WaterBoxGap值
       //              45：读取WaterBoxGap
       //              46：设定WaterBoxThreshHold
       //              47：读取WaterBoxThreshHold
      //               48：置位（true）WaterBoxWorkingFlag，使其处于工作状态
       //              49：置零（false）WaterBoxWorkingFlag，停止水箱的控温

       
      // 常运行状态：  6： Cell&Prism同步温度控制  （如果没有改变命令，将一直持续运行）
      //               7： Cell温度控制 
      //               8： Prism温度控制
      //               9： 关闭Cell控温
      //               10：关闭Prism控温 
  uint8_t i,j;
  uint32_t Param_Buff[3]={0};
  double Param_Buff_D[3]={0};      
  while(1){
    if(Serial.available()){
      lastSysState = SysState;
      SysState = Serial.read();
      if( SysState ==0 ){
        CellTempDriver.Idle();
        PrismTempDriver.Idle();
        WaterBoxWorkingFlag = 0;
      }
      if( SysState == 1){
        while(!Serial.available()){};
        Speed = Serial.read();
        if( Speed == 0 ){
          noTone( Pump );
          #ifdef DEBUG
          Serial.println("Off");
          #endif
        }else{
          tone( Pump , (unsigned int )( Speed*SpeedScale) );
          #ifdef DEBUG
          Serial.println("On");
          #endif
        } 
        SysState = lastSysState;
      }
      if( SysState == 2){
        while( 2>Serial.available() ){}
        CellGivenADC=Serial.read();
        CellGivenADC=(CellGivenADC<<8)|Serial.read();  
        PrismGivenADC = CellGivenADC;
        CellSetPoint = (double)(CellGivenADC*Scale);
        PrismSetPoint = (double)(PrismGivenADC*Scale);
        SysState = lastSysState;
      }  
      if( SysState == 3){
        while( 4>Serial.available() ){}
        CellGivenADC=Serial.read();
        CellGivenADC=(CellGivenADC<<8)|Serial.read();  
        PrismGivenADC = Serial.read();
        PrismGivenADC = (PrismGivenADC<<8)|Serial.read(); 
        CellSetPoint = (double)(CellGivenADC*Scale);
        PrismSetPoint = (double)(PrismGivenADC*Scale);
        SysState = lastSysState;
      }
      if( SysState == 40 ){ //设定水箱目标温度
        while( 2 > Serial.available() ){}
        WaterBoxGivenADC = Serial.read();
        WaterBoxGivenADC = ( WaterBoxGivenADC<<8)|Serial.read();
        WaterBoxSetPoint = ( double )( WaterBoxGivenADC*Scale );
        SysState = lastSysState;
      }
      if(SysState == 11){
        while( 13>Serial.available() ){}
        eeAddress = Serial.read();
        eeAddress = eeAddress*sizeof( Param_Buff_D );
        for(i=0;i<3;i++){
            Param_Buff[i]=Serial.read();
            Param_Buff[i]=( Param_Buff[i]<<8)|Serial.read();
            Param_Buff[i]=( Param_Buff[i]<<8)|Serial.read();
            Param_Buff[i]=( Param_Buff[i]<<8)|Serial.read();
            Param_Buff_D[i] = (double)(Param_Buff[i])/PIDparamScale; // 直接传double类型不容易，这里传一个32bit的整数，然后除以65536，可以获得0-65536之间的double类型
        }
        EEPROM.put( eeAddress , Param_Buff_D );
        //更新各个PID param的值
        eeAddress = 0 ;
        EEPROM.get(eeAddress,CellPIDParams_Heating);
        eeAddress+=sizeof(CellPIDParams_Heating);
        EEPROM.get( eeAddress , CellPIDParams_Cooling );
        eeAddress+=sizeof(CellPIDParams_Cooling);
        EEPROM.get( eeAddress ,CellPIDAggParams_Heating );
        eeAddress+=sizeof( CellPIDAggParams_Heating );
        EEPROM.get( eeAddress , CellPIDAggParams_Cooling);
        eeAddress+=sizeof( CellPIDAggParams_Cooling );
        
        EEPROM.get(eeAddress,PrismPIDParams_Heating);
        eeAddress+=sizeof(PrismPIDParams_Heating);
        EEPROM.get(eeAddress,PrismPIDParams_Cooling);
        eeAddress+=sizeof(PrismPIDParams_Cooling);
        EEPROM.get(eeAddress,PrismPIDAggParams_Heating);
        eeAddress+=sizeof(eeAddress,PrismPIDAggParams_Heating);
        EEPROM.get(eeAddress,PrismPIDAggParams_Cooling );

        EEPROM.get(eeAddress,WaterBoxPIDParams_Heating);
        eeAddress+=sizeof(WaterBoxPIDParams_Heating);
        EEPROM.get(eeAddress,WaterBoxPIDParams_Cooling);
        eeAddress+=sizeof(WaterBoxPIDParams_Cooling);
        EEPROM.get(eeAddress,WaterBoxPIDAggParams_Heating);
        eeAddress+=sizeof(WaterBoxPIDAggParams_Heating);
        EEPROM.get(eeAddress,WaterBoxPIDAggParams_Cooling );      
        eeAddress+=sizeof( WaterBoxPIDAggParams_Cooling );
        
        // 更新后使用保守值即可，运行时再判断自适应的条件
        CellPID_Heating.SetTunings( CellPIDParams_Heating[Kp], CellPIDParams_Heating[Ki], CellPIDParams_Heating[Kd] );
        CellPID_Cooling.SetTunings( CellPIDParams_Cooling[Kp], CellPIDParams_Cooling[Ki], CellPIDParams_Cooling[Kd] );
        PrismPID_Heating.SetTunings( PrismPIDParams_Heating[Kp], PrismPIDParams_Heating[Ki], PrismPIDParams_Heating[Kd] );
        PrismPID_Cooling.SetTunings( PrismPIDParams_Cooling[Kp], PrismPIDParams_Cooling[Ki], PrismPIDParams_Cooling[Kd] );  
        WaterBoxPID_Heating.SetTunings( WaterBoxPIDParams_Heating[Kp], WaterBoxPIDParams_Heating[Ki], WaterBoxPIDParams_Heating[Kd] );
        WaterBoxPID_Cooling.SetTunings( WaterBoxPIDParams_Cooling[Kp], WaterBoxPIDParams_Cooling[Ki], WaterBoxPIDParams_Cooling[Kd] );
        SysState = lastSysState;
      }  
      if( SysState == 12 ){   //打印各个参数用的
        while( !Serial.available()){};
        eeAddress = Serial.read();
        eeAddress = eeAddress*sizeof( Param_Buff_D );
        EEPROM.get( eeAddress , Param_Buff_D );
        for(i=0;i<3;i++)Serial.println(Param_Buff_D[i] );
        SysState = lastSysState;
      }
      if( SysState ==13 ){  // 设定 Cell & Prism AdaptiveFlag
        while( 2 > Serial.available()){}
        CellAdaptiveFlag = Serial.read();
        PrismAdaptiveFlag = Serial.read();
        eeAddress = PIDAdaptiveFlagAddr+Cell*sizeof( CellAdaptiveFlag );
        EEPROM.put( eeAddress , CellAdaptiveFlag );
        eeAddress = PIDAdaptiveFlagAddr+Prism*sizeof(PrismAdaptiveFlag);
        EEPROM.put( eeAddress , PrismAdaptiveFlag );
        SysState = lastSysState;
      }
      if( SysState ==14 ){ //设定Cell & Prism Gap值
        while( 4> Serial.available() ){}
        CellGap =Serial.read();
        CellGap = ( CellGap<<8)|Serial.read();
        PrismGap = Serial.read();
        PrismGap = ( PrismGap<<8)|Serial.read();
        eeAddress = PIDAdaptiveFlagAddr+Cell*sizeof( CellGap );
        EEPROM.put( eeAddress , CellGap ); 
        eeAddress += Prism*sizeof( CellGap );;
        EEPROM.put( eeAddress , PrismGap ); 
        SysState = lastSysState;
      }
      if( SysState ==17){ //设定Prism AdaptiveFlag
        SysState = lastSysState;        
      }
       if( SysState ==18 ){ //设定Cell & Prism ThreshHold 
        while( 4> Serial.available() ){}
        CellThreshHold =Serial.read();
        CellThreshHold = ( CellThreshHold<<8)|Serial.read();
        PrismThreshHold = Serial.read();
        PrismThreshHold = ( PrismThreshHold<<8)|Serial.read();
        CellThreshHoldDouble =CellThreshHold/double(ThreshHoldScale);
        PrismThreshHoldDouble = PrismThreshHold/double(ThreshHoldScale);
        eeAddress = PIDThreshHoldAddr + Cell*sizeof( CellThreshHold );
        EEPROM.put( eeAddress , CellThreshHold ); 
        eeAddress += Prism*sizeof( CellThreshHold );
        EEPROM.put( eeAddress , PrismThreshHold ); 
        SysState = lastSysState;
      }
      if( SysState == 42 ){      //               42：设定WaterBox的WaterBoxAdaptiveFlag
        while( 1 > Serial.available()){}
        WaterBoxAdaptiveFlag = Serial.read();
        eeAddress = PIDAdaptiveFlagAddr+WaterBox*sizeof( WaterBoxAdaptiveFlag );
        EEPROM.put( eeAddress , WaterBoxAdaptiveFlag );
        SysState = lastSysState; 
      }
      if( SysState == 44 ){       //              44：设定WaterBox的WaterBoxGap值
        while( 2> Serial.available() ){}
        WaterBoxGap =Serial.read();
        WaterBoxGap = ( WaterBoxGap<<8)|Serial.read();
        eeAddress = PIDAdaptiveFlagAddr+WaterBox*sizeof( CellGap );
        EEPROM.put( eeAddress , WaterBoxGap ); 
        SysState = lastSysState; 
      }
      if( SysState == 46 ){       //              46：设定WaterBoxThreshHold
        while( 2> Serial.available() ){}
        WaterBoxThreshHold =Serial.read();
        WaterBoxThreshHold = ( WaterBoxThreshHold<<8)|Serial.read();
        WaterBoxThreshHoldDouble =WaterBoxThreshHold/double(ThreshHoldScale);
        eeAddress = PIDThreshHoldAddr + WaterBox*sizeof( WaterBoxThreshHold );
        EEPROM.put( eeAddress , WaterBoxThreshHold ); 
        SysState = lastSysState; 
      }
    }
    //单次运行状态
    if( SysState == 4 ){
        // 这里按之前的直接传ADC就可以
        ADC_Cell = MyAds1115.ReadAds1115(Cell);
        Serial.write( (uint8_t)((ADC_Cell&0xff00)>>8) );//不管强制类型转换成uint8还是int8，传输的结果都是相同的
        Serial.write( (uint8_t)( ADC_Cell&0x00ff ) );
        ADC_Prism = MyAds1115.ReadAds1115(Prism);
        Serial.write( (uint8_t)((ADC_Prism&0xff00)>>8) );//不管强制类型转换成uint8还是int8，传输的结果都是相同的
        Serial.write( (uint8_t)( ADC_Prism&0x00ff ) );
        SysState = lastSysState;
    }
    if( SysState == 5 ){
      //分支5的程序转到分支4简化一下
        SysState = lastSysState;
    }
      if( SysState == 15 ){ //读AdaptiveFlag Cell & Prism
        eeAddress = PIDAdaptiveFlagAddr + Cell*sizeof( CellAdaptiveFlag );
        EEPROM.get( eeAddress , CellAdaptiveFlag );
        eeAddress += Prism+sizeof( CellAdaptiveFlag );
        EEPROM.get( eeAddress , PrismAdaptiveFlag );
        Serial.write( CellAdaptiveFlag );
        Serial.write( PrismAdaptiveFlag );
        SysState = lastSysState;
      }
      if(SysState==16){ //读Cell Prism Gap
         eeAddress = PIDGapAddr+Cell*sizeof( CellGap );
         EEPROM.get( eeAddress , CellGap );
         eeAddress+= Prism*sizeof( CellGap );
         EEPROM.get( eeAddress, PrismGap );
         Serial.write( (uint8_t)((CellGap&0xff00)>>8) );
         Serial.write( (uint8_t)( CellGap&0x00ff)); 
         Serial.write( (uint8_t)((PrismGap&0xff00)>>8) );
         Serial.write( (uint8_t)( PrismGap&0x00ff)); 
         SysState = lastSysState;
      }
       if( SysState ==19 ){ //读Cell & Prism ThreshHold 
        eeAddress = PIDThreshHoldAddr + Cell*sizeof( CellThreshHold ) ;
        EEPROM.get( eeAddress , CellThreshHold ); 
        eeAddress += Prism*sizeof( CellThreshHold );
        EEPROM.get( eeAddress , PrismThreshHold ); 
        Serial.write( (uint8_t)((CellThreshHold&0xff00)>>8) );
        Serial.write( (uint8_t)( CellThreshHold&0x00ff)); 
        Serial.write( (uint8_t)((PrismThreshHold&0xff00)>>8) );
        Serial.write( (uint8_t)( PrismThreshHold&0x00ff));        
        SysState = lastSysState;
      }       
    if( SysState == 41 ){
        // 这里按之前的直接传ADC就可以
        ADC_WaterBox = MyAds1115.ReadAds1115(WaterBox);
        Serial.write( (uint8_t)((ADC_WaterBox&0xff00)>>8) );//不管强制类型转换成uint8还是int8，传输的结果都是相同的
        Serial.write( (uint8_t)( ADC_WaterBox&0x00ff ) );
        SysState = lastSysState;
    }
    if( SysState == 43 ){ //               43：读取WaterBox的WaterBoxAdaptiveFlag   
      eeAddress = PIDAdaptiveFlagAddr + WaterBox*sizeof( WaterBoxAdaptiveFlag );
      EEPROM.get( eeAddress , WaterBoxAdaptiveFlag );
      Serial.write( WaterBoxAdaptiveFlag );
      SysState = lastSysState;
    }
    if(SysState== 45 ){ //               45：读取WaterBoxGap
       eeAddress = PIDGapAddr+WaterBox*sizeof( WaterBoxGap );
       EEPROM.get( eeAddress , WaterBoxGap );
       Serial.write( (uint8_t)((WaterBoxGap&0xff00)>>8) );
       Serial.write( (uint8_t)( WaterBoxGap&0x00ff)); 
       SysState = lastSysState;
    }
     if( SysState == 47 ){ //               47：读取WaterBoxThreshHold  
      eeAddress = PIDThreshHoldAddr + WaterBox*sizeof( WaterBoxThreshHold ) ;
      EEPROM.get( eeAddress , WaterBoxThreshHold ); 
      Serial.write( (uint8_t)((WaterBoxThreshHold&0xff00)>>8) );
      Serial.write( (uint8_t)( WaterBoxThreshHold&0x00ff));         
      SysState = lastSysState;
    }  
    if( SysState == 48 ){    // 开启水箱控温
        WaterBoxWorkingFlag = true;
        SysState = lastSysState;
    }
    if( SysState == 49 ){    // 关闭水箱控温
        WaterBoxWorkingFlag = false;
        SysState = lastSysState;
    } 
    // 常运行状态：  6：同步温度控制 7：Cell温度控制 8：Prism温度控制
    if( SysState == 0 ){
      CellTempDriver.Idle();
      PrismTempDriver.Idle();
    }
    if( SysState == 6 ){   // Cell 与Prism控温
          ADC_Cell = MyAds1115.ReadAds1115(Cell);
          CellPIDInput= (double)( ADC_Cell*Scale);
          ADC_Prism = MyAds1115.ReadAds1115(Prism);
          PrismPIDInput = (double)( ADC_Prism*Scale);
          if( CellAdaptiveFlag==1){
              if( abs(CellSetPoint-CellPIDInput)>(double)(CellGap)){
                CellPID_Heating.SetTunings( CellPIDAggParams_Heating[Kp], CellPIDAggParams_Heating[Ki], CellPIDAggParams_Heating[Kd] );
                CellPID_Cooling.SetTunings( CellPIDAggParams_Cooling[Kp], CellPIDAggParams_Cooling[Ki], CellPIDAggParams_Cooling[Kd] );
              }else{
                CellPID_Heating.SetTunings( CellPIDParams_Heating[Kp], CellPIDParams_Heating[Ki], CellPIDParams_Heating[Kd] );
                CellPID_Cooling.SetTunings( CellPIDParams_Cooling[Kp], CellPIDParams_Cooling[Ki], CellPIDParams_Cooling[Kd] );
              }
          }
          if( PrismAdaptiveFlag==1){
              if( abs(PrismSetPoint-PrismPIDInput)>(double)(PrismGap)){
                PrismPID_Heating.SetTunings( PrismPIDAggParams_Heating[Kp], PrismPIDAggParams_Heating[Ki], PrismPIDAggParams_Heating[Kd] );
                PrismPID_Cooling.SetTunings( PrismPIDAggParams_Cooling[Kp], PrismPIDAggParams_Cooling[Ki], PrismPIDAggParams_Cooling[Kd] );
              }else{
                PrismPID_Heating.SetTunings( PrismPIDParams_Heating[Kp], PrismPIDParams_Heating[Ki], PrismPIDParams_Heating[Kd] );
                PrismPID_Cooling.SetTunings( PrismPIDParams_Cooling[Kp], PrismPIDParams_Cooling[Ki], PrismPIDParams_Cooling[Kd] );
              }
           }
          CellPID_Heating.Compute();
          CellPID_Cooling.Compute();
          PrismPID_Heating.Compute();
          PrismPID_Cooling.Compute();
          if( (CellSetPoint - CellPIDInput) >=CellThreshHoldDouble ){
             CellTempDriver.Heating( pwmScale*CellPIDOutput_Heating  );
          }else if( (CellSetPoint - CellPIDInput)<(-CellThreshHoldDouble)){
             CellTempDriver.Cooling( pwmScale*CellPIDOutput_Cooling  );
          }
          if( (PrismSetPoint - PrismPIDInput) >= PrismThreshHoldDouble ){
             PrismTempDriver.Heating( pwmScale*PrismPIDOutput_Heating  );
          }else if( (PrismSetPoint - PrismPIDInput)<(-PrismThreshHoldDouble)){
             PrismTempDriver.Cooling( pwmScale*PrismPIDOutput_Cooling  );
          }
          #ifdef DEBUG
          Serial.print("CellSetPoint-CellPIDInput:");
          Serial.println( CellSetPoint - CellPIDInput );
          Serial.print( "ADC_Cell:");Serial.println( ADC_Cell);
          Serial.print("CellPIDInput:");Serial.println(CellPIDInput);
          Serial.print("CellSetPoint:");Serial.println(CellSetPoint);
          Serial.print("CellPIDOutput_Cooling:");Serial.println(CellPIDOutput_Cooling);
          Serial.print("CellPIDOutput_Heating:");Serial.println(CellPIDOutput_Heating);
                            
          Serial.print("PrismSetPoint-PrismPIDInput:");
          Serial.println( PrismSetPoint - PrismPIDInput );
          Serial.print( "ADC_Prism:");Serial.println( ADC_Prism);
          Serial.print("PrismPIDInput:");Serial.println(PrismPIDInput);
          Serial.print("PrismSetPoint:");Serial.println(PrismSetPoint);
          Serial.print("PrismPIDOutput_Cooling:");Serial.println(PrismPIDOutput_Cooling);
          Serial.print("PrismPIDOutput_Heating:");Serial.println(PrismPIDOutput_Heating);
          #endif
    }
    if( SysState == 7 ){  // Cell单独控温
          ADC_Cell = MyAds1115.ReadAds1115(Cell);
          CellPIDInput= (double)( ADC_Cell*Scale);
          if( CellAdaptiveFlag==1){
            if( abs(CellSetPoint-CellPIDInput)>(double)(CellGap)){
              CellPID_Heating.SetTunings( CellPIDAggParams_Heating[Kp], CellPIDAggParams_Heating[Ki], CellPIDAggParams_Heating[Kd] );
              CellPID_Cooling.SetTunings( CellPIDAggParams_Cooling[Kp], CellPIDAggParams_Cooling[Ki], CellPIDAggParams_Cooling[Kd] );
            }else{
              CellPID_Heating.SetTunings( CellPIDParams_Heating[Kp], CellPIDParams_Heating[Ki], CellPIDParams_Heating[Kd] );
              CellPID_Cooling.SetTunings( CellPIDParams_Cooling[Kp], CellPIDParams_Cooling[Ki], CellPIDParams_Cooling[Kd] );
            }
           }
          CellPID_Heating.Compute();
          CellPID_Cooling.Compute();
          if( (CellSetPoint - CellPIDInput) >=CellThreshHoldDouble ){
             CellTempDriver.Heating( pwmScale*CellPIDOutput_Heating  );
          }else if( (CellSetPoint - CellPIDInput)<(-CellThreshHoldDouble )){
            CellTempDriver.Cooling( pwmScale*CellPIDOutput_Cooling  );
          }
    }
    if( SysState == 8 ){   // Prsim单独控温
          ADC_Prism = MyAds1115.ReadAds1115(Prism);
          PrismPIDInput= (double)( ADC_Prism*Scale);
          if( PrismAdaptiveFlag==1){
            if( abs(PrismSetPoint-PrismPIDInput)>(double)(PrismGap)){
              PrismPID_Heating.SetTunings( PrismPIDAggParams_Heating[Kp], PrismPIDAggParams_Heating[Ki], PrismPIDAggParams_Heating[Kd] );
              PrismPID_Cooling.SetTunings( PrismPIDAggParams_Cooling[Kp], PrismPIDAggParams_Cooling[Ki], PrismPIDAggParams_Cooling[Kd] );
            }else{
              PrismPID_Heating.SetTunings( PrismPIDParams_Heating[Kp], PrismPIDParams_Heating[Ki], PrismPIDParams_Heating[Kd] );
              PrismPID_Cooling.SetTunings( PrismPIDParams_Cooling[Kp], PrismPIDParams_Cooling[Ki], PrismPIDParams_Cooling[Kd] );
            }
           }          
          PrismPID_Heating.Compute();
          PrismPID_Cooling.Compute();
          if( (PrismSetPoint - PrismPIDInput) >=PrismThreshHoldDouble){
             PrismTempDriver.Heating( pwmScale*PrismPIDOutput_Heating  );
          }else if( (PrismSetPoint - PrismPIDInput)<(-PrismThreshHoldDouble)){
            PrismTempDriver.Cooling( pwmScale*PrismPIDOutput_Cooling  );
          }  
    }
    // 单次运行状态---关闭模块
    if( SysState == 9 ){ //关Cell控温
      //noTone( Pump );      noTone( Pump );CellTempDriver.Idle();PrismTempDriver.Idle();
      CellTempDriver.Idle();
      if(lastSysState == 6 )SysState = 8;
      else SysState = 0; 
    }
    if( SysState ==10 ){
       PrismTempDriver.Idle();
       if( lastSysState == 6 )SysState = 7;
       else SysState = 0;  
    }
    WaterBoxTempReg( WaterBoxWorkingFlag );
    delay(10);
  }
}

void WaterBoxTempReg( bool flag ){
  if( flag ){
    ADC_WaterBox = MyAds1115.ReadAds1115( WaterBox );
    WaterBoxPIDInput= (double)( ADC_WaterBox*Scale);
    if( WaterBoxAdaptiveFlag==1){
      if( abs( WaterBoxSetPoint- WaterBoxPIDInput)>(double)( WaterBoxGap)){
         WaterBoxPID_Heating.SetTunings(  WaterBoxPIDAggParams_Heating[Kp],  WaterBoxPIDAggParams_Heating[Ki],  WaterBoxPIDAggParams_Heating[Kd] );
         WaterBoxPID_Cooling.SetTunings(  WaterBoxPIDAggParams_Cooling[Kp],  WaterBoxPIDAggParams_Cooling[Ki],  WaterBoxPIDAggParams_Cooling[Kd] );
      }else{
         WaterBoxPID_Heating.SetTunings(  WaterBoxPIDParams_Heating[Kp],  WaterBoxPIDParams_Heating[Ki],  WaterBoxPIDParams_Heating[Kd] );
         WaterBoxPID_Cooling.SetTunings(  WaterBoxPIDParams_Cooling[Kp],  WaterBoxPIDParams_Cooling[Ki],  WaterBoxPIDParams_Cooling[Kd] );
      }
     }          
    WaterBoxPID_Heating.Compute();
    WaterBoxPID_Cooling.Compute();
     #ifdef DEBUG
      Serial.println("WaterBoxThreshHoldDouble");
      Serial.println(WaterBoxThreshHoldDouble);
     #endif
    if( (WaterBoxSetPoint - WaterBoxPIDInput) >=WaterBoxThreshHoldDouble){
       WaterBoxTempDriver.Cooling( pwmScale*WaterBoxPIDOutput_Cooling  );
       #ifdef DEBUG
       Serial.println("Cooling");
       #endif
    }else if( (WaterBoxSetPoint - WaterBoxPIDInput)<(-WaterBoxThreshHoldDouble)){
       WaterBoxTempDriver.Heating( pwmScale*WaterBoxPIDOutput_Heating  );
       #ifdef DEBUG
        Serial.println("Heating");
       #endif
    }
    #ifdef DEBUG
      Serial.print("WaterBoxSetPoint-WaterBoxPIDInput:");
      Serial.println( WaterBoxSetPoint - WaterBoxPIDInput );
      Serial.print( "ADC_WaterBox:");Serial.println( ADC_WaterBox);
      Serial.print("WaterBoxPIDInput:");Serial.println(WaterBoxPIDInput);
      Serial.print("WaterBoxSetPoint:");Serial.println(WaterBoxSetPoint);
      Serial.print("WaterBoxPIDOutput_Cooling:");Serial.println(WaterBoxPIDOutput_Cooling);
      Serial.print("WaterBoxPIDOutput_Heating:");Serial.println(WaterBoxPIDOutput_Heating);
    #endif
  }else{
    WaterBoxTempDriver.Idle();  
  }   
}

