# Line Follower Robot

This is an autonomous robot being developed for a maze-solving competition. The current version follows a line, handles corners and junctions, and is being developed step by step toward complete maze exploration.

The robot uses an ESP32, a QTR-8RC sensor array, a TB6612FNG motor driver, and two geared DC motors.

![Line follower robot](https://github.com/user-attachments/assets/aca75f50-7bf9-4db8-b976-befde283ada7)

## Features

- Follows a black line on a light or white surface
- Uses PD control for steering
- Automatically calibrates the sensors at startup
- Starts only after the push button is pressed
- Recovers when the line is temporarily lost
- Supports left-hand rule and right-hand rule navigation
- Includes an experimental DFS mode for maze exploration
- Stores basic exploration events in memory during DFS testing

## Hardware

- ESP32-WROOM-32 development board
- QTR-8RC 8-channel reflectance sensor array
- TB6612FNG dual motor driver
- Two geared DC motors without encoders
- Two wheels and one caster wheel
- Battery pack
- Main power switch
- Push button connected to GPIO15
- Chassis within the competition size limits
- Jumper wires and connectors
- Screws, spacers, and motor mounts

## Wiring diagram

![Complete wiring diagram](https://github.com/user-attachments/assets/c8efb6c1-3ef2-4c9a-bc6e-3ff2cd838459)

A vector version of the wiring diagram is also available in the repository:

## Connections

### QTR-8RC sensor array

The physical sensor order is S1 on the right and S8 on the left.

| Sensor pin | ESP32 pin |
|---|---:|
| S1, rightmost | GPIO32 |
| S2 | GPIO33 |
| S3 | GPIO18 |
| S4 | GPIO19 |
| S5 | GPIO4 |
| S6 | GPIO21 |
| S7 | GPIO23 |
| S8, leftmost | GPIO22 |
| VCC | 3.3 V |
| GND | Common GND |

In the code, the sensor order is reversed so the values are read from left to right: S8, S7, S6, S5, S4, S3, S2, S1.

### TB6612FNG motor driver

| TB6612FNG pin | ESP32 connection |
|---|---:|
| PWMA | GPIO25 |
| AIN1 | GPIO26 |
| AIN2 | GPIO27 |
| PWMB | GPIO14 |
| BIN1 | GPIO12 |
| BIN2 | GPIO13 |
| STBY | 3.3 V |
| VCC | 3.3 V |
| VM | Switched battery positive |
| GND | Common GND |


### Power connections

The following points must share a common ground:

- Battery negative
- ESP32 GND
- QTR-8RC GND
- TB6612FNG GND
- Push-button GND

Power the ESP32 using USB or a suitable regulated power supply. Do not connect raw battery voltage directly to the ESP32 3.3 V pin.

## Setup

1. Install the Arduino IDE.
2. Install the ESP32 board package.
3. Select the correct ESP32 board, usually **ESP32 Dev Module**.
4. Install the `QTRSensors` library through the Arduino Library Manager.
5. Connect the ESP32 and select the correct port.
6. Upload the required line-following or maze-solving sketch.
7. Place the robot over the line and let the automatic calibration finish.
8. Press the button connected to GPIO15 to start the robot.

The current setup is for a black line on a light surface. If the competition arena uses a white line on a black surface, the sensor-reading mode in the code must be changed.

## Algorithms

### Basic line following

The robot estimates the position of the line using all eight sensors. PD control then adjusts the speed of the two motors to keep the robot centred on the line.

### Left-hand and right-hand rules

At a junction, the robot selects a direction according to a fixed priority:

```text
Left-hand rule:  Left → Straight → Right → U-turn
Right-hand rule: Right → Straight → Left → U-turn
```

These rules are useful for initial maze testing, but they do not guarantee the shortest path.

### DFS exploration

The experimental DFS version explores branches recursively and backtracks after reaching a dead end or completing a branch. It is currently intended for testing the robot's exploration and backtracking behaviour.

The official maze contains loops, and the current robot does not have encoders or dedicated node markers. Because of this, identifying previously visited junctions reliably is still an open part of the project.

### Shortest path

Graph storage and A* shortest-path logic will be added after the basic line following and DFS behaviour are reliable.

## Testing plan

The robot is being tested in the following order:

1. Test each motor individually.
2. Confirm that both motors move forward together.
3. Check QTR sensor readings on black and white surfaces.
4. Test automatic calibration.
5. Tune basic line following on straight sections.
6. Test gentle curves and 90-degree turns.
7. Test junction detection on a small practice track.
8. Test left-hand and right-hand rule navigation.
9. Test DFS on a simple maze without loops.
10. Add graph and shortest-path logic after DFS is stable.

## CAD and mechanical design

### Rough paper sketch

![Rough robot sketch](https://github.com/user-attachments/assets/1fd800e3-ac10-4623-861c-8dc4847e2601)

### CAD development

![CAD model](https://github.com/user-attachments/assets/87d912c9-c08f-4041-b44f-044b0d22f7ad)

![Updated CAD model](https://github.com/user-attachments/assets/f9b46c34-9d3e-4519-bb60-989d0f8e3f0e)
