# Autonomous line-following car for Mercedes Benz

- [Autonomous line-following car for Mercedes Benz](#autonomous-line-following-car-for-mercedes-benz)
  - [Intro](#intro)
  - [Components](#components)
  - [The Schematic](#the-schematic)


## Intro
In this project I will make an autonomous line-tracking car. The car is a prototype for the Mercedes Benz museum. Once done, the car will be able to perform the following actions:
1. Follow a white line
2. Detect obstacles in it's way using a HC-SR04 sensor
3. Detect if it has collided with anything
4. Show the battery voltage through a RGB-led indicator
5. Show the status of the vehicle through 4 different leds
6. It'll have a startup-tune
7. Web interface (grafana)
8. It'll also be able to stop at a traversal line at which point it either starts driving after 5sec or after the pressing of a button

## Components
1. ESP-32
2. 1 5xTCRT5000 IR sensor with Limit Switch
3. 2 5V Motor
4. 1 L293d Motor Driver
5. 1 LM 1117 T5,0 Voltage Regulator
6. 2 9V Batteries
7. 1 Switch
8. 1 DC-female jack
9. 1 RGB-led
10. 1 MCP23016 I/O Expander
11. 1 HC-SR04 Ultrasone Sensor
12. 4 leds (green, orange, red, blue) 
13. 1 Button
14. 1 Buzzer
15. 1 Castor Wheel
16. Wires

## The Schematic
![Schematic](https://imgur.com/wjlqvnd.png)
