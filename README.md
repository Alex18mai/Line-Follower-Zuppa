# IntroductionToRobotics: Line Follower - Zuppa

## Introduction

🔥 Zuppa 🔥 is the greatest line follower to ever exist. 

With features such as:
- Automated calibration saved in EEPROM
- Fine-tuned PID controller
- Cool design
- FLAMES!
- Registration plate
- Headlights
- A lot of personality
- Love ❤️

🔥 Zuppa 🔥 has dominated the dinosaur race (taking 14.7 seconds in training), but decided to let other participants have a chance by only scoring 15.9 seconds in the final presentation.

## Our strategy

- We have implemented the calibration as a state machine based on the raw read information of the sensor.
- The configuration is saved in EEPROM and loaded each time. In order to start a new calibration we have added a button on the front of the robot for this.
- We have fine-tuned the PID by doing runs on the test tracks. The PID is based on the value of the readLineBlack function.

## Picture of the setup

<img src="https://github.com/Alex18mai/Line-Follower-Zuppa/blob/main/Assets/PictureOfSetup.jpg">

## Picture of the team

<img src="https://github.com/Alex18mai/Line-Follower-Zuppa/blob/main/Assets/PictureOfTeam.jpeg" width="500">

Big thanks to [Dana](https://github.com/danadascalescu00) for taking the picture and for being an awesome teacher! We love you! ❤️

## Video

[![](https://img.youtube.com/vi/RGH5YCr8QzM/0.jpg)](https://youtu.be/RGH5YCr8QzM)

## Task requirements

Given a kit with all the basic components and a boilerplate code, we needed to assemble the car and fine-tune the PID controller.

## Grading

- Implement calibration with automatic motor movement
- Follow a curved line (not exaggerated)
- Finish the line follower course
- Finish the line follower course in a fast manner with visible control (implementing P, D and maybe I)

## Components
- Arduino Uno
- Zip-ties
- Power source (can be of different shape). In our case, a LiPo battery
- Wheels (2)
- Wires for the line sensor (female - male)
- QTR-8A reflectance sensor, along with screws
- Ball caster
- Extra wires from the kit or lab
- Chassis
- Breadboard - medium (400pts)
- L293D motor driver
- DC motors (2)
- LED (2)
- Wires and resistors per logic
