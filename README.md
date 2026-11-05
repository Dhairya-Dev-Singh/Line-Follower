Line Follower Robot
This is the autonomous robot designed for the maze solving. This robot currently follows a line path, identifies corners and junctions, and is being developed in stages to support exploring a maze.

It is currently using the ESP32, QTR-8RC sensor array and TB6612FNG motor driver. We are also using geared DC motors.


 Features
 
Follows a black line on a light/white background

Uses PID turning to follow line

Automatically calibrates sensors

Only starts moving when button is pressed

Robust to losing line

Can follow left hand rule, right hand rule and experimental DFS

Basic exploration events are stored in memory when using DFS.



Hardware

ESP32-WROOM-32 dev board

QTR-8RC Sensor Array (8 Channel Reflective Sensor)

TB6612FNG Motor Driver

2 geared DC motors (No Encoders)

2 wheels and 1 caster wheel

Battery pack

Power switch

Push-button connected to GPIO15

Chassis (Competition size limits)

Jumper cables

Connectors

Screws

Spacers

Motor Mounts



Full wiring diagram:



Wiring



QTR-8RC Sensor



Sensor Order: S1 (Right), S8 (Left)

 Sensor Pin | ESP32 Pin 

 S1 (Right) | GPIO32 

 S2 | GPIO33 

 S3 | GPIO18 

 S4 | GPIO19 

 S5 | GPIO4 

 S6 | GPIO21 

 S7 | GPIO23 

 S8 (Left) | GPIO22 

 VCC | 3.3V 

 GND | GND 

The order of sensors when reading is reversed so that S8 is the left most and S1 is the right most.



TB6612FNG Motor Driver



TB6612FNG Pin | ESP32 Connection 

 PWMA | GPIO25 

 AIN1 | GPIO26 

 AIN2 | GPIO27 

 PWMB | GPIO14 

 BIN1 | GPIO12 

 BIN2 | GPIO13 

 STBY | 3.3V 

 VCC | 3.3V 

 VM | Positive of battery (Switch) 

 GND | GND 

Motor Order: AO1/AO2 = Right, BO1/BO2 = Left

In the code you will have MOTORS_SWAPPED set to true.



Power Button



 Button Pin 1 | ESP32 GPIO15 

 Button Pin 2 | GND 

Pin GPIO15 is set to have the internal pull up resistor enabled. So button will be high when not pressed.



Power



Battery Positive | Power Switch | TB6612 VM 

Battery Negative | GND | GND |

Battery Negative, ESP32 GND, Sensor GND, Motor Driver GND, Button GND all need connecting together.

ESP32 needs to be powered using USB or a suitable power supply. 



Setup



Install Arduino IDE

Install ESP32 Board Package

Select the correct ESP32 board (usually ESP32 Dev Module)

Install the QTRSensors library using the Arduino Library Manager

Connect ESP32 and select the correct port

Upload the required sketch for line following or maze solving

Place robot on line to allow calibration to happen

Press the button connected to GPIO15

Currently we are using a black line on a light surface for testing. If the arena is using a white line on a black surface then the sensors will need to be set to read in that mode.



Algorithms

Basic Line Following

Robot finds the position of the line using the 8 sensors and uses PD control to change the speed of the 2 motors.



 Left Hand Rule & Right Hand Rule

Robot will decide which way to go at a junction based on a priority list:

Left Hand Rule: Left > Straight > Right > U-Turn

Right Hand Rule: Right > Straight > Left > U-Turn

These are useful for testing the maze with the robot but won’t give the shortest path.


 Testing Recommendations

Test each motor individually

Test both motors are moving together forwards

Test QTR sensor readings on black and white surfaces

Test auto-calibration

Fine tune basic line following on straight lines

Test on gentle curves and 90 degree turns

Test junction detection on small test track

Test left and right hand rules

Test DFS on simple non-looping maze

Once DFS is working reliably add graph and shortest path logic.
