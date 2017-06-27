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

//
#define KS103ADDR 0x74    // Address of the 0xe8  选择串口地址，共有20个
#define Reg 0x02        // Command byte 选择02探测指令
byte highByte = 0x00;   // Stores high byte from ranging
byte lowByte = 0x00;   // Stored low byte from ranging


void setup() {
  // 初始化串口:
  Serial.begin(57600);
  Serial1.begin(57600); //for IMU
  //for SONAR
  Wire.begin();     //链接线路                          
  delay(100);                         // Waits to make sure everything is powered up before sending or receiving data
  Wire.beginTransmission(KS103ADDR);             // Start communticating with KS103
  Wire.write(Reg);                               // Send Reg
  Wire.write(0x71);  // Send 0x72 to set USB Power 三级降噪
  Wire.endTransmission();
  
}

void loop() {
  //for IMU
  // 从串口1转发到串口0:  
  if (Serial1.available()) {
    int inByte = Serial1.read();
    Serial.write(inByte);
  }

  // 从串口0转发到串口1:
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial1.write(inByte);
  }

  //for SONAR
  int rangeData = getRange();                      // Calls a function to get range
  Serial.println(rangeData);
  
  //for LINESCANNER
  
  
  
  
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

