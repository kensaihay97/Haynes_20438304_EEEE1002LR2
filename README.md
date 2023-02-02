# Haynes_20438304_EEEE1002LR2
Lab Report 2 PID line following solution

This repository contains an Arduino sketch for a PID controlled line following car called the EEEBot, which uses I2C to communicate between ESP32 and an Arduino Nano. 

Getting Started

Prerequisites

Arduino IDE
ESP32
An Arduino Nano
A line following robot that is set up for I2C communication

Installing

Copy or download the repository to your computer
Open the Haynes_20438304_PID_Solution.ino file in the Arduino IDE
Connect your ESP32 to your computer using a USB cable
In the Arduino IDE, select the correct board type and port in the Tools menu
Upload the code to the ESP32
Running the Code
Power on your line following robot
Observe the behavior of the robot as it moves along the line

Code Explanation
The code uses a PID control loop to determine the speed and direction of the line following robot. The robot is equipped with 6 IR sensors which are used to detect the presence of a line. The input to the PID loop is the weighted sum of the sensor readings, which provides a measure of how far off the center of the line the robot is.

The PID loop uses the following formula to calculate the control output:

U = P + I + D

Where:

P is the proportional term, which is proportional to the error between the desired setpoint and the current line position
I is the integral term, which sums the error over time to eliminate any steady state error
D is the derivative term, which helps to reduce overshoot and oscillations in the system

The code also includes calibration variables for each of the 6 sensors, which can be used to adjust the sensitivity of the sensors to suit different line conditions.

Conclusion

This code provides a solid foundation for a PID controlled line following robot, and can be easily adapted and modified to suit different applications and requirements. With some additional modifications, this code could be used as the basis for more advanced line following.
