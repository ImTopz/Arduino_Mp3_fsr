#include <SoftwareSerial.h>
#include <FastLED.h>


//已经用的DO接口 2 3 4 5 6 7 10 11

int bofang_1=0;
int bofang_2=0;
int bofang_3=0;
//接口定义初始化
#define ARDUINO_RX 5//should connect to TX of the Serial MP3 Player module
#define ARDUINO_TX 6//connect to RX of the module
SoftwareSerial mySerial(ARDUINO_RX, ARDUINO_TX);
unsigned char playmode = 1; 


//引脚定义
#define LED_PIN_3  2
#define LED_PIN_2  3
#define LED_PIN_1  7
#define NUM_LEDS   24
#define SENSOR_1   4
#define SENSOR_2   10
#define SENSOR_3   11
CRGB leds[NUM_LEDS];
int KEY_NUM_1=0;
int KEY_NUM_2=0;
int KEY_NUM_3=0;

//状态定义
  #define PLAY  1
  #define PAUSE 0

//发送缓冲区
static int8_t Send_buf[8] = {0} ;
  
/************Command byte**************************/
#define CMD_NEXT_SONG 0X01 //下一曲
#define CMD_PREV_SONG 0X02 //上一曲
#define CMD_PLAY_W_INDEX 0X03 //指定曲目
#define CMD_VOLUME_UP 0X04
#define CMD_VOLUME_DOWN 0X05
#define CMD_SET_VOLUME 0X06
#define CMD_SINGLE_CYCLE_PLAY 0X08 
#define CMD_SEL_DEV 0X09 //指定播放设备
#define DEV_TF 0X02
#define CMD_SLEEP_MODE 0X0A
#define CMD_WAKE_UP 0X0B
#define CMD_RESET 0X0C
#define CMD_PLAY 0X0D  //播放
#define CMD_PAUSE 0X0E
#define CMD_PLAY_FOLDER_FILE 0X0F
#define CMD_STOP_PLAY 0X16
#define CMD_FOLDER_CYCLE 0X17
#define CMD_SET_SINGLE_CYCLE 0X19
#define SINGLE_CYCLE_ON 0X00
#define SINGLE_CYCLE_OFF 0X01
#define CMD_SET_DAC 0X1A
#define DAC_ON  0X00
#define DAC_OFF 0X01

/*********************************************************************/
/*macro definitions of Rotary angle SENSOR_1 and LED pin*/
#define ROTARY_ANGLE_SENSOR A0

#define ADC_REF 5//reference voltage of ADC is 5v
#define VCC     5    //the default value of VCC of the control interface is 5v
#define FULL_ANGLE 280//full value of the rotary angle is 280 degrees

void setup() 
{
  Serial.begin(9600);
  mySerial.begin(9600);
  delay(500);
  attachInterrupt(0, playOrPause, RISING);//pin2 -> INT0, and the Touch SENSOR_1 
                                          //is connected with pin2 of Arduino
  FastLED.addLeds<WS2812,LED_PIN_1, GRB>(leds, NUM_LEDS); 
  FastLED.addLeds<WS2812,LED_PIN_2, GRB>(leds, NUM_LEDS);
  FastLED.addLeds<WS2812,LED_PIN_3, GRB>(leds, NUM_LEDS); 
  pinMode(SENSOR_1,INPUT);
  pinMode(SENSOR_2,INPUT);
  pinMode(SENSOR_3,INPUT);
  pinMode(LED_PIN_1,OUTPUT);
  pinMode(LED_PIN_2,OUTPUT);
  pinMode(LED_PIN_3,OUTPUT);
  sendCommand(CMD_SEL_DEV, DEV_TF);  
  delay(200);
  sendCommand(CMD_SET_VOLUME,0x1D);
  delay(200);//wait for 200ms
 

}
static int8_t pre_vol = 0x0f; 
void loop() 
{
  int degrees;
  degrees = getDegree();
  int8_t volume;
  /*The degrees is 0~280, should be converted to be 0~255 to control the*/
  /*brightness of LED */
  volume = map(degrees, 0, 280, 0, 30); 
  if(volume != pre_vol)
  {
    sendCommand(CMD_SET_VOLUME, volume);
    pre_vol = volume;
  }
  delay(100);
  scanSensor_1();
  scanSensor_2();
  scanSensor_3();
  if(KEY_NUM_1==1)
  {
    bofang_1=0;
    if(bofang_1==0)
    {
     sendCommand(0x0E,0x01);
    }
    for (int i = 0; i <= 23; i++)
    {
    leds[i] = CRGB ( 0, 255, 0);
    FastLED.show();
    delay(40);
     }
    
     
   
  }
  if(KEY_NUM_1>=2)
  {
    bofang_1++;
    if(bofang_1==1)
    {
      sendCommand(0x03,0x01);
    }
    for (int i = 0; i <= 23; i++)
    {
    leds[i] = CRGB ( 0, 0, 0);
    FastLED.show();
    delay(40);
    }
    KEY_NUM_1=0;
  }
  if(KEY_NUM_2==1)
  {
    bofang_2=0;
    if(bofang_2==0)
    {
     sendCommand(0x0E,0x01);
    }
    for (int i = 0; i <= 23; i++)
    {
    leds[i] = CRGB ( 255, 0, 0);
    FastLED.show();
    delay(40);
     }
   
  }
  if(KEY_NUM_2>=2)
  {
    bofang_2++;
    if(bofang_2==1)
    {
      sendCommand(0x03,0x02);
    }
    for (int i = 0; i <= 23; i++)
    {
    leds[i] = CRGB ( 0, 0, 0);
    FastLED.show();
    delay(40);
    }
    KEY_NUM_2=0;
  }
  if(KEY_NUM_3==1)
  {
    bofang_3=0;
    if(bofang_3==0)
    {
     sendCommand(0x0E,0x01);
    }
    for (int i = 0; i <= 23; i++)
    {
    leds[i] = CRGB ( 0, 0, 255);
    FastLED.show();
    delay(40);
     }
   
  }
  if(KEY_NUM_3>=2)
  {
    bofang_3++;
    if(bofang_3==1)
    {
      sendCommand(0x03,0x03);
    }
    for (int i = 0; i <= 23; i++)
    {
    leds[i] = CRGB ( 0, 0, 0);
    FastLED.show();
    delay(40);
    }
    KEY_NUM_3=0;
  }
}

void sendCommand(int8_t command, int16_t dat)
{
  delay(20);
  Send_buf[0] = 0x7e; //
  Send_buf[1] = 0xff; //
  Send_buf[2] = 0x06; //
  Send_buf[3] = command; //
  Send_buf[4] = 0x00;//
  Send_buf[5] = (int8_t)(dat >> 8);//datah
  Send_buf[6] = (int8_t)(dat); //datal
  Send_buf[7] = 0xef; //
  for(uint8_t i=0; i<8; i++)//
  {
    mySerial.write(Send_buf[i]) ;
  }
}
/********************************************************************************/
/*Function: Get the angle between the mark on the potentiometer cap and the starting position */
/*Parameter:-void                                                                                                          */
/*Return:     -int,the range of degrees is 0~280                                                                 */
int getDegree()
{
  int sensor_value = analogRead(ROTARY_ANGLE_SENSOR);
  float voltage;
  voltage = (float)sensor_value*ADC_REF/1023;
  float degrees = (voltage*FULL_ANGLE)/VCC;
  return degrees;
}

/*Interrupt service routine*/
void playOrPause()
{
  cli();
  if(playmode == PLAY)
  {
    playmode = PAUSE;
  sendCommand(CMD_PAUSE,0);
  }
  else
  {
    playmode = PLAY;
  sendCommand(CMD_PLAY,0);
  }
  sei();
}
void scanSensor_1()
{
  if(digitalRead(SENSOR_1) == HIGH)
  {
    delay(50);
    if(digitalRead(SENSOR_1) == HIGH) 
    {
      while(digitalRead(SENSOR_1) == HIGH);
      KEY_NUM_1 ++;
    } 
  }
}
void scanSensor_2()
{
  if(digitalRead(SENSOR_2) == HIGH)
  {
    delay(50);
    if(digitalRead(SENSOR_2) == HIGH) 
    {
      while(digitalRead(SENSOR_2) == HIGH);
      KEY_NUM_2 ++;
    } 
  }
}
void scanSensor_3() 
{
  if(digitalRead(SENSOR_3) == HIGH)
  {
    delay(50);
    if(digitalRead(SENSOR_3) == HIGH) 
    {
      while(digitalRead(SENSOR_3) == HIGH);
      KEY_NUM_3 ++;
    } 
  }
}
