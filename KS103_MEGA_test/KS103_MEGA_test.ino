#include <Wire.h>
                              // Software serial pin for tx
#define KS103ADDR 0x74    // Address of the 0xe8  选择串口地址，共有20个
#define Reg 0x02        // Command byte 选择02探测指令
byte highByte = 0x00;   // Stores high byte from ranging
byte lowByte = 0x00;   // Stored low byte from ranging
void setup()
{
  
  Serial.begin(9600);
  Wire.begin();     //链接线路                          
  delay(100);                         // Waits to make sure everything is powered up before sending or receiving data

  Wire.beginTransmission(KS103ADDR);             // Start communticating with KS103
  Wire.write(Reg);                               // Send Reg
  Wire.write(0x71);  // Send 0x72 to set USB Power 三级降噪
  Wire.endTransmission();
   
}
void loop()
{
  
  int rangeData = getRange();                      // Calls a function to get range

  Serial.println(rangeData);
  delay(100);                                      // Wait before looping
}

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
