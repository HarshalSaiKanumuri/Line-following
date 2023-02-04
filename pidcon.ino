#define SLAVE_ADDRESS 4
#include <Wire.h>
#define Kp 3
#define Ki 0.6
#define Kd 0
double integral = 0;
double lastInput = 0;


int Output;

float PID(float setpoint, float input) {
  float error = setpoint - input;
  float Pout = Kp * error;
  integral += error * 0.1; 
  float Iout = Ki * integral;
  float derivative = (input - lastInput) / 0.1;
  float Dout = Kd * derivative;
  float output = Pout + Iout + Dout;
  lastInput = input;
  return output;
}


void sendData(int16_t leftMotorspeed, int16_t rightMotorspeed, int16_t servoAngle) {

  Wire.beginTransmission(4); // Call slave

  Wire.write((byte)((leftMotorspeed & 0x0000FF00) >> 8));    // first byte of leftMotor_speed, containing bits 16 to 9
  Wire.write((byte)(leftMotorspeed & 0x000000FF));           // second byte of leftMotor_speed, containing bits 8 to 1
  Wire.write((byte)((rightMotorspeed & 0x0000FF00) >> 8));   // rest follows the same logic
  Wire.write((byte)(rightMotorspeed & 0x000000FF));          
  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));         
  Wire.write((byte)(servoAngle & 0x000000FF));               
  Wire.endTransmission();   // stop transmitting
}

void setup() {
  // Set the sensor pins as inputs
  Serial.begin(9600);
  Wire.begin();
  pinMode(34, INPUT);
  pinMode(35, INPUT);
  pinMode(32, INPUT);
  pinMode(33, INPUT);
  pinMode(25, INPUT);
  pinMode(26, INPUT);


}

void loop() {
  Serial.begin(9600);
  // Calibrate the sensors by measuring the black and white values
  //calibrateSensors();

  // Read the sensor values
  int sensor1Value = analogRead(34);
  int sensor2Value = analogRead(35);
  int sensor3Value = analogRead(32);
  int sensor4Value = analogRead(33);
  int sensor5Value = analogRead(25);
  int sensor6Value = analogRead(26);
  

  sensor1Value = map(sensor1Value, 1861, 4095,100,0);
  sensor2Value = map(sensor2Value, 1511, 4095,100,0);
  sensor3Value = map(sensor3Value, 1770, 4095,100,0);
  sensor4Value = map(sensor4Value, 2756, 4095,100,0);
  sensor5Value = map(sensor5Value, 2147, 4095,100,0);
  sensor6Value = map(sensor6Value,  531, 4095,100,0);


  double Weighted_Average = ((sensor1Value * -10) + (sensor2Value * -5 ) + (sensor3Value * 0 ) + (sensor4Value * 5 ) + (sensor5Value  * 10)) / 5 ;
  Weighted_Average = map(Weighted_Average,-150,150,150,-150);
  Output = PID(0, Weighted_Average);
  Output = map(Output,-96,76,175,75);
  //int servo = PID(,Output );
/*
  Serial.print(sensor1Value);
  Serial.print(" "); 
  Serial.print(sensor2Value);
  Serial.print(" ");
  Serial.print(sensor3Value);
  Serial.print(" ");
  Serial.print(sensor4Value);
  Serial.print(" ");
  Serial.print(sensor5Value);
  //Serial.print(" ");
  //Serial.println(sensor6Value);

  
*/

  sendData(100,100,Output);  

  Serial.println(Output);
  delay(100);
}
