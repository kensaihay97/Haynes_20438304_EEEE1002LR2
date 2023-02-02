/*Student ID: 20438304
Name: Kenville Haynes*/

#include <Wire.h>  //Included for I2C communication
#include <Arduino.h> //Included Arduino library

#define NANO_ADDRESS 0x04  //I2C address of the Arduino Nano

// ADC Pin assignments for sensors
#define SENSOR1 35  //Left sensor
#define SENSOR2 32  //Left sensor
#define SENSOR3 33  //Middle left
#define SENSOR4 36  //Middle right
#define SENSOR5 39  //Right
#define SENSOR6 34  //Right

double P = 0, D = 0, U = 0, error = 0;  //PID formula variables

const double kp = 28.5, kd = 1.5, ki = 10;  //PID constants

int left_baseSpeed = 160;   //EEEBot motor stable speed
int right_baseSpeed = 160;  //EEEBot motor stable speed
int leftSpeed = 0, rightSpeed = 0, servoPos = 54;

int prevServo = 54;  //EEEBot previous servo position
int midServo = 54;   //EEEBot center steering

int setpoint = 0;  //Setpoint located on the center of the car

double K = 0.1666;  //Motor speed scaling factor

// Define the weights for each sensor, distance in mm from the center of the array of IR sensors
float weight1 = -37.5;
float weight2 = -22.5;
float weight3 = -7.5;
float weight4 = 7.5;
float weight5 = 22.5;
float weight6 = 37.5;

float input;  //PID input

// Calibration variables
int min_1, min_2, min_3, min_4, min_5, min_6;
int max_1, max_2, max_3, max_4, max_5, max_6;

unsigned long previous_time = 0;  // Initialize previous_time
float dt = 0;                     //PID dt term

void setup() {

  Serial.begin(115200);  //Begin serial communication

  Wire.begin();  //Begin I2C communication

  //Calibrated max and min values
  min_1 = 697;
  min_2 = 760;
  min_3 = 789;
  min_4 = 461;
  min_5 = 690;
  min_6 = 577;

  max_1 = 4095;
  max_2 = 4095;
  max_3 = 4095;
  max_4 = 4095;
  max_5 = 4095;
  max_6 = 4095;

  // Set the pin modes for the sensor pairs
  pinMode(SENSOR1, INPUT);
  pinMode(SENSOR2, INPUT);
  pinMode(SENSOR3, INPUT);
  pinMode(SENSOR4, INPUT);
  pinMode(SENSOR5, INPUT);
  pinMode(SENSOR6, INPUT);
}

void loop() {

  // Read the values from all 6 sensors
  int sensor1Value = analogRead(SENSOR1);
  int sensor2Value = analogRead(SENSOR2);
  int sensor3Value = analogRead(SENSOR3);
  int sensor4Value = analogRead(SENSOR4);
  int sensor5Value = analogRead(SENSOR5);
  int sensor6Value = analogRead(SENSOR6);

  if (sensor1Value >= max_1 && sensor2Value >= max_2 && sensor3Value >= max_3 && sensor4Value >= max_4 && sensor5Value >= max_5 && sensor6Value >= max_6) {
    
    //If all sensors are off the line, use the last servo position to reverse the EEEBot
    if (prevServo < 54) {
      //Turn right
      leftSpeed = -160;
      rightSpeed = -150;
      servoPos = 64;
    } else if (prevServo > 54) {
      //Turn left
      leftSpeed = -150;
      rightSpeed = -160;
      servoPos = 45;
    }

    //Constain values to within minimum and maximum values to prevent extremes
    leftSpeed = constrain(leftSpeed, -255, 0);
    rightSpeed = constrain(rightSpeed, -255, 0);
    servoPos = constrain(servoPos, 30, 80);

    transmitArduino(leftSpeed, rightSpeed, servoPos);  //Transmit to Arduino NANO which controls speed and steering

    delay(100);

  } else {

    //Constrains the values to a specified range, to disallow extremes
    sensor1Value = constrain(sensor1Value, min_1, max_1);
    sensor2Value = constrain(sensor2Value, min_2, max_2);
    sensor3Value = constrain(sensor3Value, min_3, max_3);
    sensor4Value = constrain(sensor4Value, min_4, max_4);
    sensor5Value = constrain(sensor5Value, min_5, max_5);
    sensor6Value = constrain(sensor6Value, min_6, max_6);

    //Maps the values into the 0 to 255 range
    sensor1Value = map(sensor1Value, min_1, max_1, 0, 255);
    sensor2Value = map(sensor2Value, min_2, max_2, 0, 255);
    sensor3Value = map(sensor3Value, min_3, max_3, 0, 255);
    sensor4Value = map(sensor4Value, min_4, max_4, 0, 255);
    sensor5Value = map(sensor5Value, min_5, max_5, 0, 255);
    sensor6Value = map(sensor6Value, min_6, max_6, 0, 255);

    // Calculate the weighted average of the sensor values
    float num = (sensor1Value * weight1 + sensor2Value * weight2 + sensor3Value * weight3 + sensor4Value * weight4 + sensor5Value * weight5 + sensor6Value * weight6);
    float denum = (sensor1Value + sensor2Value + sensor3Value + sensor4Value + sensor5Value + sensor6Value);

    input = (num / denum);  //Weighted average

    float error = setpoint - input;  // error = setpoint - weighted average

    //Serial.print("Error: ");
    //Serial.println(error);

    pid(error);  //Send value to PID function
  }
}

void pid(float error) {

  unsigned long current_time = millis();         //Initialize current_time in milli seconds
  dt = (current_time - previous_time) / 1000.0;  //Divided by 1000.0 to convert to seconds in floating point
  previous_time = current_time;                  //Initialize previous_time

  static float I = 0;              //Declare the integral term as a static floating point number
  static float previousError = 0;  //Declare the pervious error as a static floating point number

  P = error;                         //Error calculated by the weighted average formula
  I += error * dt;                   //Integral sum of errors
  D = (error - previousError) / dt;  //Differentiation of the error

  // Serial.print("I: ");
  // Serial.println(I);

  U = (kp * P) + (ki * I) + (kd * D);  //PID formula

  previousError = error;  //Storing the previous error

  leftSpeed = left_baseSpeed + K * U;    //Left motor speed is base speed + scaling factor K multiplied by the PID output
  rightSpeed = right_baseSpeed - K * U;  //Right motor speed is base speed - scaling factor K multiplied by the PID output
  servoPos = midServo + U;               //Servo angle is the center angle + the PID output

  //Constain values to within minimum and maximum values to prevent extremes
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  servoPos = constrain(servoPos, 30, 80);

  prevServo = servoPos;

  //Serial.print("Servo: ");
  //Serial.println(servoPos);

  //Serial.print("Left Speed:");
  //Serial.println(leftSpeed);

  //Serial.print("Right Speed:");
  //Serial.println(rightSpeed);

  transmitArduino(leftSpeed, rightSpeed, servoPos);  //Transmit to Arduino NANO over I2C which controls speed and servo
}

//Transmit to Arduino NANO
void transmitArduino(int leftSpeed, int rightSpeed, int servoPos)

{

  Wire.beginTransmission(NANO_ADDRESS);  //Transmit to device 0x04

  Wire.write((byte)((leftSpeed & 0x0000FF00) >> 8));  //First byte of leftMotor, containing bits 16 to 9

  Wire.write((byte)(leftSpeed & 0x000000FF));  //Second byte of leftMotor, containing the 8 LSB - bits 8 to 1

  Wire.write((byte)((rightSpeed & 0x0000FF00) >> 8));  //First byte of rightMotor, containing bits 16 to 9

  Wire.write((byte)(rightSpeed & 0x000000FF));  //Second byte of rightMotor, containing the 8 LSB - bits 8 to 1

  Wire.write((byte)((servoPos & 0x0000FF00) >> 8));  //First byte of rightMotor, containing bits 16 to 9

  Wire.write((byte)(servoPos & 0x000000FF));  //Second byte of rightMotor, containing the 8 LSB - bits 8 to 1

  Wire.endTransmission();  //Stop transmitting
}
