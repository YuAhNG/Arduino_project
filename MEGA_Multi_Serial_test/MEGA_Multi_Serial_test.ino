/*
  多串口[Mega]

 从主串口获取数据并且转发到其他串口。
 从串口1获取数据并且转发到主串口（串口0）。

 代码只能在串口>=2的设备上使用。比如Arduino Mega、Due、 Zero等。

 电路搭建:
 * 串口设备连接到串口1
 * 串口监视器在串口0打开

 代码是公开的。

 */


void setup() {
  // 初始化串口:
  Serial.begin(57600);
  Serial1.begin(57600);
}

void loop() {
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
}
