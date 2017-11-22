#include <FlexiTimer2.h>
// 2015 08 05 ADXL335 is added to this programm
// 1 ： 器件识别信息
// 2 ： 单次的数据采集
// 3 ： NULL
// 4 ： NULL
// 5 ： NULL
// 6 ： Pump set speed
// 7 ： NULL
// 8 ： NULL
// 9 ： 获取Cell & Prism 温度
// 10： SlaveMega2560进入空闲状态
// 11： 异步设定Cell & Prism 温度
// 12： Cell & Prism 控温启动
// 13： Cell控温启动
// 14： Prism控温启动
// 15： 关Cell控温
// 16： 关Prism控温
// 17： 关Pump
// 18： 转台归零
// 19： 转台控制指令直传
// 20： 获取转台位置信息
// 21： 旋转并测量
// 40:  设定水箱温度
// 41： 水箱温度查询
// 48： 启动水箱控温
// 49： 关闭水箱控温
// 50： PID参数调试命令
// 51： PID参数读取命令
// Pividi SGSP
unsigned char Pividi_SendBuffer[4];
unsigned char Pividi_RecBuffer[34];
unsigned char EmptyBuffer;
const unsigned char Pividi_OriginStart[ 4 ] = { 0 , 0 , 0 , 255 };  // 看Matlab代码的方向
const unsigned char Pividi_OriginBack[ 4 ] = { 2 , 0 , 0xE7 , 0  };
const unsigned char Pividi_OriginStop[ 4 ] = { 0 , 0 , 0 , 0 };     // 停止旋转
const unsigned char Pividi_QueryPosition[ 4 ] ={ 1, 1 , 1 , 1 };    // 查询位置
volatile bool TouchFlag = false;

int Photodiode=A0;
int ind=0;
int tempADC0val = 0;

unsigned int TimeCounter = 0;
unsigned int TimeDuration = 0;
unsigned int MeaDelayMicroSec = 0;

unsigned int PumpSpeed=0;
const byte interruptPin = 21; //2, 3, 18, 19, 20, 21

int i;
#define TimeOut 65535
#define TimeFactor 20
unsigned int DeadWhileAvoider = TimeOut;

void SampleAndTransfer(){
  tempADC0val = analogRead( Photodiode );
  Serial.write( tempADC0val/256 );
  Serial.write( tempADC0val%256 );  
  if(TimeCounter< TimeDuration ){
     TimeCounter++;
  }else{
     FlexiTimer2::stop();
  }
}
void Touch(){
  noInterrupts();
  TouchFlag = true;
  interrupts();
}
void setup(){
  // 串口0：电脑   串口1：SlaveMega2560  串口2：Pividi Controller
  Serial.begin(115200);
  Serial1.begin(115200);   
  Serial2.begin(57600);
 // Serial2.begin(57600);
  pinMode(Photodiode,INPUT);
  analogReference(EXTERNAL);
  FlexiTimer2::set(1, SampleAndTransfer);
  
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), Touch, FALLING);
}

void loop(){
  if( Serial.available() ){
    ind = Serial.read();
  }
  if( ind ==1 ){
    Serial.write(170);Serial.write(170);  // Indicate that I am the arduino board 
    ind=0;
  }
  if( ind == 2 ){ //单次测量Photodiode
    tempADC0val = analogRead( Photodiode );
    Serial.write( tempADC0val/256 );
    Serial.write( tempADC0val%256 );  
    ind = 0;  // To prevent looping
  }
  if( ind == 3 ){
    
    ind = 0;
  }  
  if( ind == 4 ){
    
    ind = 0;
  }
  if( ind == 5 ){ //5留作触碰开关使
      ind=0;
  }  
  if( ind ==6 ){ //Pump set speed
      while(!Serial.available());
      Serial1.write( (uint8_t)0x01 );
      Serial1.write( Serial.read());
      ind=0;
  }
  if( ind==7 ){
      FlexiTimer2::start();
      ind =0;
  }
  if( ind==8 ){
      FlexiTimer2::stop();
      ind =0;
  }
  if( ind == 9){  //获得cell与Prism的温度信息
      while(Serial1.available())Serial1.read();
      DeadWhileAvoider = TimeOut;
      Serial1.write((uint8_t)4);delay(10);
      while(Serial1.available()<4 && DeadWhileAvoider > 0){
        delayMicroseconds(TimeFactor);
        DeadWhileAvoider--;  
      }
      if( DeadWhileAvoider){
        for( i =0; i<4; i++)Serial.write( Serial1.read() );  
      }
      ind =0;
  }
  if( ind ==11){
    //设定CellPrism温度
    DeadWhileAvoider = TimeOut;
    Serial1.write(3);
    while(4>Serial.available() && DeadWhileAvoider > 0 ){
        delayMicroseconds(TimeFactor);
        DeadWhileAvoider--;   
    }
    if( DeadWhileAvoider ){
      for(i=0;i<4;i++)Serial1.write( Serial.read());
    }
    ind =0;
  }
  if( ind ==40){  // 40:  设定水箱温度
    //设定CellPrism温度
    DeadWhileAvoider = TimeOut;
    Serial1.write(40);
    while(4>Serial.available() && DeadWhileAvoider > 0 ){
        delayMicroseconds(TimeFactor);
        DeadWhileAvoider--;   
    }
    if( DeadWhileAvoider ){
      for(i=0;i<4;i++)Serial1.write( Serial.read());
    }
    ind =0;
  }
  if( ind == 41 ){  // 41： 水箱温度查询
      while(Serial1.available())Serial1.read();
      DeadWhileAvoider = TimeOut;
      Serial1.write((uint8_t)41);delay(10);
      while(Serial1.available()<2 && DeadWhileAvoider > 0){
        delayMicroseconds(TimeFactor);
        DeadWhileAvoider--;  
      }
      if( DeadWhileAvoider){
        for( i =0; i<2; i++)Serial.write( Serial1.read() );  
      }
      ind =0;
  }
  if( ind == 12 ){
    Serial1.write(6);  // Cell & Prism 控温启动
    ind =0;
  }
  if( ind ==13 ){
    Serial1.write(7);//Cell控温启动
    ind =0;  
  }
  if(ind==14){
    Serial1.write(8);//Prism  控温启动
    ind =0;
  }
  if( ind == 10 ){
    //Uno Idle
    Serial1.write(0);
    ind =0;
  }
  if( ind ==15 ){
    Serial1.write(9);  // 关cell控温
    ind =0;
  }
  if( ind == 16){
    Serial1.write( 10); // 关Prism控温
    ind =0;  
  }
  if( ind == 17 ){
    Serial1.write( 1 );
    Serial1.write( 0 ); // Close Pump;  
    ind =0;
  }
  if( ind == 48 ){ // 48： 启动水箱控温
    Serial1.write( 48 ); // Close Pump;  
    ind =0;
  }
  if( ind == 49 ){ // 49： 关闭水箱控温
    Serial1.write( 49 ); // Close Pump;  
    ind =0;
  }
  if( ind == 50 ){
    // 清除Slave2560缓冲区
    while(Serial1.available())Serial1.read();
    // 等待主机发送来所有参数，并防止while循环死掉
    DeadWhileAvoider = TimeOut; 
    while(13>Serial.available() && DeadWhileAvoider > 0 ){
        delayMicroseconds(TimeFactor);
        DeadWhileAvoider--;   
    }
    // 向Slave发送修改PID指令
    Serial1.write( 11 ); // Slave2560中 11：修改指定地址处的PID参数
    if( DeadWhileAvoider ){
      for(i=0;i<13;i++)Serial1.write( Serial.read());
    }
    delay(200);//延时等待EEPROM写入完成
    ind =0;
  }
  if( ind == 51 ){
    // 清除Slave2560缓冲区
    while(Serial1.available())Serial1.read();
    // 等待主机发送来所有参数，并防止while循环死掉
    DeadWhileAvoider = TimeOut; 
    while(1>Serial.available() && DeadWhileAvoider > 0 ){
        delayMicroseconds(TimeFactor);
        DeadWhileAvoider--;   
    }
    // 向Slave发送修改PID指令
    Serial1.write( 12 ); // Slave2560中 11：修改指定地址处的PID参数
    Serial1.write( Serial.read());
    delay(1000);//延时等待EEPROM读取完成
    while( Serial1.available() ){
      Serial.write( Serial1.read() );
    }
    ind =0;
  }
  if( ind ==18 ){     // -------------------------------转台归零
    TouchFlag = false;
    while( Serial2.available() ){ 
      EmptyBuffer=Serial2.read();  
    }
    for( i = 0 ; i < 4 ; i++ ){
      DeadWhileAvoider = TimeOut;
      Serial2.write( Pividi_OriginStart[ i ] );
      while( !Serial2.available() && DeadWhileAvoider > 0 ){
        delayMicroseconds( TimeFactor );  
        DeadWhileAvoider--;
      }
      if( DeadWhileAvoider ){
        EmptyBuffer=Serial2.read();
      }
    }
    while( !TouchFlag ){
    }
    for( i=0; i<4; i++ ){
      DeadWhileAvoider = TimeOut;
      Serial2.write( Pividi_OriginStop[ i ] );
      while( !Serial2.available() && DeadWhileAvoider > 0 ){
        delayMicroseconds( TimeFactor );  
        DeadWhileAvoider--;
      }
      if( DeadWhileAvoider ){
        EmptyBuffer=Serial2.read();
      }
    }
    delay(500);
    for( i = 0 ; i < 4 ; i++ ){
      DeadWhileAvoider = TimeOut;
      Serial2.write( Pividi_OriginBack[ i ] );
      while( !Serial2.available() && DeadWhileAvoider > 0 ){
        delayMicroseconds( TimeFactor );  
        DeadWhileAvoider--;
      }
      if( DeadWhileAvoider ){
        EmptyBuffer=Serial2.read();
        Serial.write( 55 ); //转台归零成功
      }else{
        Serial.write(44);  
      }
    }

    ind = 0;
  }
  if( ind == 19 ){       // ---------------------------转台控制指令直传
    while( 4> Serial.available() ){};
    while( Serial2.available() ){ 
      EmptyBuffer=Serial2.read();  
    }
    for( i=0 ; i< 4 ; i++ ){
        DeadWhileAvoider = TimeOut;
        Serial2.write( Serial.read() );
        while( !Serial2.available() && DeadWhileAvoider>0 ){
          delayMicroseconds( TimeFactor );
          DeadWhileAvoider--;  
        }
        if( DeadWhileAvoider ){
          EmptyBuffer=Serial2.read();
        }
    }
    ind = 0;
  }
  if( ind == 20 ){      //-----------------------------获取转台位置信息
    while( Serial2.available() ){ 
      EmptyBuffer=Serial2.read();  
    }  
    for( i = 0 ; i < 4 ; i++ ){
      DeadWhileAvoider = TimeOut;
      Serial2.write( Pividi_QueryPosition[ i ] );
      while( !Serial2.available() && DeadWhileAvoider>0  ){
        delayMicroseconds( TimeFactor );
        DeadWhileAvoider--;  
      }
      if( DeadWhileAvoider ){
        EmptyBuffer=Serial2.read(); 
      } 
    }
    DeadWhileAvoider = TimeOut;
    while( 33> Serial2.available() && DeadWhileAvoider > 0 ){
      delayMicroseconds( 2*TimeFactor );
      DeadWhileAvoider--;  
    } 
    if( DeadWhileAvoider ){
      for( i =0; i< 33 ; i++ ){  
        Serial.write( Serial2.read());  
      }
    }
    ind = 0;
  }
  if( ind == 21 ){        //----------------------------旋转并测量
    TimeCounter = 0;
    TimeDuration = 0;
    while( 8> Serial.available() ){}
    MeaDelayMicroSec = Serial.read();
    MeaDelayMicroSec = (MeaDelayMicroSec<<8)|Serial.read();
    if( MeaDelayMicroSec > 16383 )MeaDelayMicroSec = 16383;
    TimeDuration = Serial.read();
    TimeDuration = (TimeDuration<<8)|Serial.read(); 
    
    while( Serial2.available() ){ 
      EmptyBuffer=Serial2.read();  
    }
    for( i = 0 ; i < 4 ; i++ ){
      DeadWhileAvoider = TimeOut;
      Serial2.write( Serial.read() );
      while( !Serial2.available() && DeadWhileAvoider>0 ){
        delayMicroseconds( TimeFactor );
        DeadWhileAvoider--;  
      }
      if( DeadWhileAvoider ){
        if( i == 3 ){
            delayMicroseconds( MeaDelayMicroSec );
            FlexiTimer2::start();
//            while( TimeCounter < TimeDuration ){
//                delayMicroseconds( 1000 );
//                TimeCounter++;
//            }
//            FlexiTimer2::stop();
        }
        EmptyBuffer=Serial2.read();
      }
    }
    ind = 0;
  }
}
