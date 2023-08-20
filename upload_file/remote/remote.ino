#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define CE_PIN    5
#define CSN_PIN   6

//button
#define SWITCH_PIN  4
#define ROLL_PIN A0
#define PITCH_PIN A1
#define buttonUpPin     8     // 上按钮引脚
#define buttonDownPin   10   // 下按钮引脚
#define buttonLeftPin   7   // 左按钮引脚
#define buttonRightPin  9  // 右按钮引脚

#define kp1_ptr   1
#define ki1_ptr   2
#define kd1_ptr   3
#define kp2_ptr   4
#define ki2_ptr   5
#define kd2_ptr   6
#define base_ptr  7

#define DISABLE     0
#define ENABLE      1

#define ANGLE_MIN -40
#define ANGLE_MAX 40
#define ADC_MIN 0
#define ADC_MAX 1000

#define DebRes1 100
#define DebRes2 10

#define OLED_RESET  13

const uint64_t address[2] = {0x1,0x2};

Adafruit_SSD1306 display(OLED_RESET);

struct SensorData {
  int P1;
  int I1;
  int D1;
  int P2;
  int I2;
  int D2;
  int base;
  int motor;
  int roll;
  int pitch;
} data = {1100,0,0,70,0,0,0,0,0,0};

RF24 radio(CE_PIN, CSN_PIN);

int selectedParameter = kp1_ptr;

float angle[2] = {0,0};

void check_button(){
  
  if(digitalRead(SWITCH_PIN) == LOW)  data.motor=DISABLE;
  else                                data.motor=ENABLE;
  
  if(digitalRead(buttonUpPin) == LOW){
    delay(20);  // 延迟一段时间以避免连续切换
    if(digitalRead(buttonUpPin) == LOW){// 上按钮被按下，选择下一个参数
      selectedParameter++;
      if(selectedParameter > base_ptr)  selectedParameter=kp1_ptr;
    }
  }

  if (digitalRead(buttonLeftPin) == LOW) {
    delay(20);// 延迟一段时间以避免连续调整
    if (digitalRead(buttonLeftPin) == LOW) {// 左按钮被按下，减小参数值
    switch (selectedParameter){
      case kp1_ptr:
        data.P1-=DebRes1;
        break;
      case ki1_ptr:
        data.I1-=DebRes1;
        break; 
      case kd1_ptr:
        data.D1-=DebRes1;
        break; 
      case kp2_ptr:
        data.P2-=DebRes2;
        break;
      case ki2_ptr:
        data.I2-=DebRes2;
        break; 
      case kd2_ptr:
        data.D2-=DebRes2;
        break; 
      case base_ptr:
        data.base-=1;
      default: break;
      }
    }
  }

  if (digitalRead(buttonRightPin) == LOW) {
    delay(20);  // 延迟一段时间以避免连续调整
    if (digitalRead(buttonRightPin) == LOW) {// 右按钮被按下，增大参数值
    switch (selectedParameter){
      case kp1_ptr:
        data.P1+=DebRes1;
        break;
      case ki1_ptr:
        data.I1+=DebRes1;
        break; 
      case kd1_ptr:
        data.D1+=DebRes1;
        break; 
      case kp2_ptr:
        data.P2+=DebRes2;
        break;
      case ki2_ptr:
        data.I2+=DebRes2;
        break; 
      case kd2_ptr:
        data.D2+=DebRes2;
        break; 
      case base_ptr:
        data.base+=1;
      default: break;
      }
    }
  }

  //joystick
  float right_x = analogRead(ROLL_PIN);
  float right_y = analogRead(PITCH_PIN);

  data.pitch = (int) (map(right_x, ADC_MIN, ADC_MAX, ANGLE_MIN, ANGLE_MAX));
  data.roll  = (int) (map(right_y, ADC_MIN, ADC_MAX, ANGLE_MIN, ANGLE_MAX));

  if(data.roll>30)        data.roll=30;
  else if(data.roll<-30)  data.roll=-30;
  else  {}  
  
  if(data.pitch>30)        data.pitch=30;
  else if(data.pitch<-30)  data.pitch=-30;
  else  {}

  if(data.base>200)       data.base=200;
  else if(data.base<100)  data.base=100;
  else  {}

  if(data.P1<0) data.P1 = 0;
  if(data.I1<0) data.I1 = 0;
  if(data.D1<0) data.D1 = 0;
  if(data.P2<0) data.P2 = 0;
  if(data.I2<0) data.I2 = 0;
  if(data.D2<0) data.D2 = 0;
}

void oled_print(){
  display.clearDisplay();

  char  paraName[3];
  float paraValue;
 switch (selectedParameter){
      case kp1_ptr:
        strcpy(paraName,"P1:");
        paraValue = data.P1*0.001;
        break;
      case ki1_ptr:
        strcpy(paraName,"I1:");
        paraValue = data.I1*0.001;
        break; 
      case kd1_ptr:
        strcpy(paraName,"D1:");
        paraValue = data.D1*0.001;
        break; 
      case kp2_ptr:
        strcpy(paraName,"P2:");
        paraValue = data.P2*0.001;
        break;
      case ki2_ptr:
        strcpy(paraName,"I2:");
        paraValue = data.I2*0.001;
        break; 
      case kd2_ptr:
        strcpy(paraName,"D2:");
        paraValue = data.D2*0.001;
        break; 
      case base_ptr:
        strcpy(paraName,"bs:");
        paraValue = data.base*0.1;
      default: break;
      }
  
  display.setCursor(0,8);
  display.print(paraName);
  display.setCursor(25,8);
  display.print(paraValue);

  display.setCursor(65,8);
  char text1[] = "motor: on";
  char text2[] = "motor: off";
  if (data.motor == ENABLE)   display.print(text1);
  else                        display.print(text2);

//display roll
  display.setCursor(0, 20);
  char text3[] = "roll: ";
  display.print(text3);
  display.setCursor(30, 20);
  display.print(data.roll);
//display pitch
  display.setCursor(60, 20);
  char text4[] = "pitch: ";
  display.print(text4);
  display.setCursor(100, 20);
  display.print(data.pitch);

  display.display();
  delay(20);
}

void setup() {
  Serial.begin(9600);
  
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
//radio.openReadingPipe(1,address[0]);  //0x1
  radio.openWritingPipe(0x2);    //0x2
  radio.stopListening();

  //oled:
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);

  //button:
  pinMode(buttonUpPin, INPUT);
  pinMode(buttonDownPin, INPUT);
  pinMode(buttonLeftPin, INPUT);
  pinMode(buttonRightPin, INPUT);
  pinMode(SWITCH_PIN, INPUT);
  pinMode(ROLL_PIN , INPUT);
  pinMode(PITCH_PIN , INPUT);
}



void loop() {
  
  check_button(); //check button state

  oled_print();   //print info on oled

  int a[2];
  a[0] = data.roll;
  a[1] = data.pitch;

  radio.write(&data, sizeof(data));   //send data to raspi
  //delay(10);

  //radio.startListening();
  //while(!radio.available());
  //radio.read(&angle,sizeof(angle));
  //Serial.println("received");
  //Serial.print(angle[0]);
  }


