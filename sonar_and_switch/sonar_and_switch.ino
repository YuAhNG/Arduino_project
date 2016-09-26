
//#include <ros.h>
//#include <dependant_api/float32Array.h>
//#include <dependant_api/Int16Array.h>
//ros::NodeHandle nh;
//dependant_api::Int16Array sonar;
//ros::Publisher sonar_pub("sonar_data", &sonar);
const int TrigPin = 2;
const int EchoPin = 3;
const int SelectSW = A0;
const int StartSW =A1;
float cm = 0.0;
void setup()
{
  //nh.initNode();
  //nh.advertise(sonar_pub);
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
  pinMode(SelectSW, INPUT);
  pinMode(StartSW, INPUT);
  Serial.begin(9600);
}
void loop()
{
  int k;
  digitalWrite(TrigPin, LOW); //低高低电平发一个短时间脉冲去TrigPin
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  cm = pulseIn(EchoPin, HIGH) / 58.0; //将回波时间换算成cm
  cm = (int(cm * 100.0)) / 100.0; //保留两位小数
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  delay(10);
  //sonar.data[0] = cm;
  //sonar_pub.publish(&sonar);
  //nh.spinOnce();
  //////SwitchDetection
  select = analogRead(SelectSW);
  start = analogRead(StartSW);
  Serial.println(select);
  Serial.println(start);
}
