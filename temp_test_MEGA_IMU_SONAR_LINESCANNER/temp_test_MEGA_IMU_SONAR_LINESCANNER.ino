/*
传感器MEGA
KS103的I2C接
20-SDA
21-SCL
MEGA板上的SCL SDA与21/20号管脚实际上是相连的
IMU模块57600波特率
格式 #YPR=101.70,-1.66,17.54
接
18-TX1 
19-RX1

光感
前向8个：22 23 24 25 26 27 28 29
后向8个：30 31 32 33 34 35 36 37
*/
#include <Wire.h>

#include <ros.h>
#include <dependant_api/Int16Array.h>
#include <dependant_api/Int16Array16.h>
#include "std_msgs/String.h"

#define KS103ADDR 0x74    // Address of the 0xe8  选择串口地址，共有20个
#define Reg 0x02        // Command byte 选择02探测指令
byte highByte = 0x00;   // Stores high byte from ranging
byte lowByte = 0x00;   // Stored low byte from ranging

//for line scanner
int sensor_number = 16;
int print_value;
int led_pin = 12;
int led_sw = 0;

int forward_pin0 = 22;
int forward_pin1 = 23;
int forward_pin2 = 24;
int forward_pin3 = 25;
int forward_pin4 = 26;
int forward_pin5 = 27;
int forward_pin6 = 28;
int forward_pin7 = 29;

int backward_pin0 = 30;
int backward_pin1 = 31;
int backward_pin2 = 32;
int backward_pin3 = 33;
int backward_pin4 = 34;
int backward_pin5 = 35;
int backward_pin6 = 36;
int backward_pin7 = 37;

int coef_digitalread = 1000; //Make digital read compatible with previous analog read




  //callback function  led control
void messageCb(const dependant_api::Int16Array& msg) {
  if(msg.data[0] > 0)
    digitalWrite(led_pin,HIGH);
  else
    digitalWrite(led_pin,LOW);
}
ros::NodeHandle nh;
dependant_api::Int16Array16 analog;
ros::Publisher ir_line_scanner_pub("ir_raw_data", &analog);
ros::Subscriber<dependant_api::Int16Array> sub("led_duoduo_2", &messageCb);




void setup() {
  // 初始化串口:
  Serial.begin(57600);
  //Serial1.begin(57600); //for IMU
  //for SONAR
  Wire.begin();     //链接线路                          
  delay(100);                         // Waits to make sure everything is powered up before sending or receiving data
  Wire.beginTransmission(KS103ADDR);             // Start communticating with KS103
  Wire.write(Reg);                               // Send Reg
  Wire.write(0x71);  // Send 0x72 to set USB Power 三级降噪
  Wire.endTransmission();
  //for line scanner
  pinMode(led_pin, OUTPUT);
  
  pinMode(forward_pin0, INPUT_PULLUP);
  pinMode(forward_pin1, INPUT_PULLUP);
  pinMode(forward_pin2, INPUT_PULLUP);
  pinMode(forward_pin3, INPUT_PULLUP);
  pinMode(forward_pin4, INPUT_PULLUP);
  pinMode(forward_pin5, INPUT_PULLUP);
  pinMode(forward_pin6, INPUT_PULLUP);
  pinMode(forward_pin7, INPUT_PULLUP);
  
  pinMode(backward_pin0, INPUT_PULLUP);
  pinMode(backward_pin1, INPUT_PULLUP);
  pinMode(backward_pin2, INPUT_PULLUP);
  pinMode(backward_pin3, INPUT_PULLUP);
  pinMode(backward_pin4, INPUT_PULLUP);
  pinMode(backward_pin5, INPUT_PULLUP);
  pinMode(backward_pin6, INPUT_PULLUP);
  pinMode(backward_pin7, INPUT_PULLUP);
  
  nh.initNode();
  nh.advertise(ir_line_scanner_pub);
  nh.subscribe(sub);   
}

void loop() {
   
  //for SONAR
  //int rangeData = getRange();                      // Calls a function to get range
  //Serial.println(rangeData);
  
  //for LINESCANNER
  ros::Subscriber<dependant_api::Int16Array> sub("led_duoduo_2", &messageCb);
  //ros::spin();
  
  int i, j, k, a[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  
  a[0]  = coef_digitalread * (1 - digitalRead(backward_pin0));
  a[1]  = coef_digitalread * (1 - digitalRead(backward_pin1));
  a[2]  = coef_digitalread * (1 - digitalRead(backward_pin2));
  a[3]  = coef_digitalread * (1 - digitalRead(backward_pin3));
  a[4]  = coef_digitalread * (1 - digitalRead(backward_pin4));
  a[5]  = coef_digitalread * (1 - digitalRead(backward_pin5));
  a[6]  = coef_digitalread * (1 - digitalRead(backward_pin6));
  a[7]  = coef_digitalread * (1 - digitalRead(backward_pin7));
  
  a[8]  = coef_digitalread * (digitalRead(forward_pin0));
  a[9]  = coef_digitalread * (digitalRead(forward_pin1));
  a[10] = coef_digitalread * (digitalRead(forward_pin2));
  a[11] = coef_digitalread * (digitalRead(forward_pin3));
  a[12] = coef_digitalread * (digitalRead(forward_pin4));
  a[13] = coef_digitalread * (digitalRead(forward_pin5));
  a[14] = coef_digitalread * (digitalRead(forward_pin6));
  a[15] = coef_digitalread * (digitalRead(forward_pin7));
  
  for (k = 0; k < sensor_number; k++)
    analog.data[k] = a[k];
  ir_line_scanner_pub.publish(&analog);
  nh.spinOnce();
  //delay(18.5);  //使话题频率为50Hz
  
 
  print_value++;
  if (print_value >= 20)
  {
    for (k = 0; k < sensor_number; k++)
    {
      Serial.print(a[k]);
      Serial.print("\t");
    }
    Serial.println();
    print_value = 0;
  }

  
  delay(100); 


  
}


//for SONAR
int getRange(){                                    // This function gets a ranging from the SRF08
  
  int range = 0; 

  Wire.beginTransmission(KS103ADDR);               // Start communticating with SRF08
  Wire.write(Reg);                                // Send Command Byte
  Wire.write(0xb4);                                // Send 0xb0 to start a ranging(0-5m)
  Wire.endTransmission();
  
  delay(100);                                     // Wait for ranging to be complete
  
  Wire.beginTransmission(KS103ADDR);              // start communicating with SRFmodule
  Wire.write(Reg);                                // Call the register for start of ranging data
  Wire.endTransmission();

  Wire.requestFrom(KS103ADDR, 2);                 // Request 2 bytes from SRF module
  while(Wire.available() < 2);                    // Wait for data to arrive
  highByte = Wire.read();                         // Get high byte
  lowByte = Wire.read();                          // Get low byte
  //Serial.println (lowByte, DEC); 
  range = ((highByte << 8) + lowByte)/10;              // Put them together
  
  return(range);                                  // Returns Range
}

