# Autonomous line-following car for Mercedes Benz

- [Autonomous line-following car for Mercedes Benz](#autonomous-line-following-car-for-mercedes-benz)
  - [Intro](#intro)
  - [Components](#components)
  - [Body Plate](#body-plate)
  - [The Schematic \& PCB](#the-schematic--pcb)
  - [The Code](#the-code)
    - [Without MQTT](#without-mqtt)
      - [MCP23016 I/O Expander](#mcp23016-io-expander)
      - [Status Leds](#status-leds)
      - [Sensors](#sensors)
      - [Sensor State Handling](#sensor-state-handling)
      - [Motor Controls](#motor-controls)
      - [Distance Sensor](#distance-sensor)
      - [Battery Voltage](#battery-voltage)
      - [Startup-Tune](#startup-tune)
    - [With MQTT](#with-mqtt)


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

## Body Plate
![Body-Plate](./Autonomous%20line%20tracking%20car%20Design%20Drawing%20v4.png)
## The Schematic & PCB
The Schematic (.sch eagle file) can be found in the [./PCB&Schematic/](./PCB%26Schematic/) Folder
![Schematic](https://imgur.com/wjlqvnd.png)
An Image of the PCB and the gerber files can also be found there.
![PCB](./PCB&Schematic/PCB.png)

## The Code
### Without MQTT
The full .ino codefile can be found under [./Arduino/Final](./Arduino/Final/Final.ino).
This code has a couple different elements/sensors and I will be explaining the full code per element.
#### MCP23016 I/O Expander
##### Pre-Setup
In the pre-setup we are including the library that is necesary to use the MCP23016. This code also changes the call of the MCP23016 to just MCP which saves space on the microcontroller and makes writing the code easier. We also define the port A (0.0-0.7) and port B (0.1-0.7) data register addresses so that the sketch can interact with both registers. And at last we declare some initial variables of the library that it will need.
```
//------MCP23016------
#include <MCP23016.h>

MCP23016 MCP;

#define GPA 0x00  //port A data register address
#define GPB 0x01  //port B data register address

uint8_t mcpA;  //internal variable of the library - required to be declared as is
uint8_t mcpB;  //internal variable of the library - required to be declared as is

```
##### Setup
In the setup we initialise the expander library with it's I2C address (0x20) and we also initialise Wire for the I2C connection. We also define the pinmodes of all the connected sensors (INPUT) and leds (OUTPUT).
```
  //------MCP23016 I/O Expander------
  Wire.begin();
  Wire.setClock(100000);

  MCP.init(0x20);

  MCP.pinMode(lineSensorLL, INPUT);
  MCP.pinMode(lineSensorL, INPUT);
  MCP.pinMode(lineSensorC, INPUT);
  MCP.pinMode(lineSensorR, INPUT);
  MCP.pinMode(lineSensorRR, INPUT);

  MCP.pinMode(collisionSensor, INPUT);

  MCP.pinMode(problem, OUTPUT);
  MCP.pinMode(obstacle, OUTPUT);
  MCP.pinMode(line, OUTPUT);
  MCP.pinMode(onOFF, OUTPUT);
```
##### Loop
Inside of the loop we use the MCP mainly for getting sensor data and changing the status leds state. The majority of this will be explained later in this document but I'll give some examples.
<br>

To put a led on we do:
`MCP.digitalWrite(onOFF, HIGH);`
Here `onOFF` is the led from which we want to change the state and `HIGH` means that we want to put the led on. If we would want to put the led off, then we would use `LOW` here.
<br>

To read a sensor's value we would do:
`MCP.digitalRead(sensor1);`
In which `sensor1` is the sensor we want to read the value from.

#### Status Leds
##### Pre-Setup
In the Pre-setup we define the pins to which the leds are connected. This is to the MCP in which a0-a7 is 0.0-0.7 and b0-b7 is 1.0-1.7.
```
#define problem a0
#define obstacle a1
#define line a2
#define onOFF a3
```
##### Setup
As said in the [MCP23016](#mcp23016-io-expander) setup section I set the pinmodes here. For the code look at the MCP23016 Setup code.
##### Loop
In the loop there is a function that checks for the state of the error code statuses and adjusts the status leds accordingly.
```
  //------Changing the status leds------
  if (value[5] == false && value[6] == false && value[7] == false) {
    MCP.digitalWrite(problem, HIGH);
  } else {
    MCP.digitalWrite(problem, LOW);
  }

  if (value[5] == true || value[7] == true) {
    MCP.digitalWrite(obstacle, HIGH);
  } else {
    MCP.digitalWrite(obstacle, LOW);
  }

  if (value[6] == true) {
    MCP.digitalWrite(line, HIGH);
  } else {
    MCP.digitalWrite(line, LOW);
  }
```
#### Sensors
##### Pre-Setup
Just like the status leds, the IR line sensors are also connected to the MCP. Here the pins for those sensors is defined in the same way as the status leds, check [here](#status-leds) for more information.
```
//------Sensor Definition------
#define lineSensorLL b0
#define lineSensorL b1
#define lineSensorC b2
#define lineSensorR b3
#define lineSensorRR b4

#define collisionSensor b5
```
##### Setup
Just like the status leds in setup, the sensors' pinmode is also getting defined here. More information can be found [here](#mcp23016-io-expander).
##### Loop
In the loop, the sensor values will be read as explained with the [MCP23016-explenation](#mcp23016-io-expander). For the logic behind the reading of the sensor values, take a look [here](#sensor-state-handling).

#### Sensor State Handling
##### Pre-Setup
Before I started to write the sensor state handling, I thought about what I needed and seeing that there are a fair amount of value's that need to be changed I thought that always using a for-loop would be more benefitial. So I did and the following code is what's needed for that.
```
int sensors[6] = { lineSensorLL, lineSensorL, lineSensorC, lineSensorR, lineSensorRR, collisionSensor };

String sensornames[8] = { "LL", "L", "C", "R", "RR", "Col", "Line", "Dist" };

bool value[8] = { false, false, false, false, false, false, false, false };

bool middle = false;
bool left = false;
bool right = false;
```
I first define the line sensors here. This is so that I can reference them by index and not always have to change them individually. After that I added an array of string to also reference the abbreviated names of the values by index which is very beneficial for printing the values sincce you can just use 1 for loop.
After that I put the values of the sensors in a boolean array.
The last 3 booleans have been added, because the line was way smaller than the gap between the sensors and so the vehicle needed to remember that it just had a line visible at a specified sensor which would help it to correct itself. This will be explained more clearly later on.
##### Setup
In the setup I don't have anything, because there are just values which don't need to be initialised.
##### Loop
The loop has several pieces which are considered in the sensor handling proces.
###### Distance
This is a simple if statement that checks if the current value of the distance sensor is greater than the set trigger distance and adjusts the distance value accordingly.
```
  if (cm < triggerDistance) {
    value[7] = true;
  } else {
    value[7] = false;
  }
```
###### Whiteline
For testing purposes and in order to be versatile, I implemented a simple value that reads the values of the sensors according to the preset mode which can change between following a white line or a black line.
```
  if (whiteLine) {
    //------Sensor data acquisition for a white line----
    for (int i = 0; i < 6; i++) {
      value[i] = MCP.digitalRead(sensors[i]);
    }
  } else {
    //------Sensor data acquisition for a black line----
    for (int i = 0; i < 6; i++) {
      value[i] = !MCP.digitalRead(sensors[i]);
    }
    value[5] = !value[5];
  }
```
###### Debug Printing
I also implemented a simple debug print of the sensor states with a for-loop.
```
  for (int i = 0; i < 8; i++) {
    Serial.print(String(sensornames[i]));
    Serial.print(": ");
    Serial.print(value[i]);
    Serial.print(" | ");
  }
  Serial.println();
```
#### Motor Controls
##### Pre-Setup
In the pre-setup we define the motor pins, the speed of the motors and some variables to controll the halting of the vehicle at a traverse line.
###### Motor Pins
This vehicle has 2 5V DC motors called M1 and M2. Each of these motors are connected to the motor driver which has 3 inputs called EN, F and R. EN is short for Enable, which turns the engines on or off. F is short for Forward which makes the motor spin forward relative to the vehicle. R is short for Reverse which makes the motor spin in reverse relative to the vehicle.
```
#define M1_EN 15
#define M1_F 2
#define M1_R 4

#define M2_EN 23
#define M2_F 18
#define M2_R 19
```
###### Motor Speed
The motor speed is changed using PWM. In this case of the motor direction should not spin, then the value would be `low` of 0 and if the motor direction should spin, then the value would be `high` or 120.
```
#define high 120
#define low 0
```
###### Halt Control
For the vehicle to be able to take halt at a traverse line, we need to have a couple different values. First of all we need a state value to check if the vehicle is in halt. Then we need some variables to control the time that the vehicle should halt. At last we add a button to be able to skip the halting and move forward straight away.
```
bool haltControl = true;
int haltLast = 0;
int elapsedHalt = 0;
int currentHalt = 0;

#define haltButton 32
```
##### Setup
In the setup we only need to add the correct pin modes to the motor pins.
```
  //------Motor pins------
  pinMode(M1_EN, OUTPUT);
  pinMode(M1_F, OUTPUT);
  pinMode(M1_R, OUTPUT);

  pinMode(M2_EN, OUTPUT);
  pinMode(M2_F, OUTPUT);
  pinMode(M2_R, OUTPUT);
```
##### Loop
In the loop we control the behavior of the car so that it follows the line. This has several if/else if/else statements which will be explained one by one hereunder.
###### Stand still at obstacle
When the vehicle detects an obstacle either through it's distance sensor or through it's collision sensor, then all motors will be shut down.
```
  //------Stand still at obstacle------
  if (value[5] == HIGH || value[7] == HIGH) {
    digitalWrite(M1_EN, LOW);
    analogWrite(M1_F, low);
    analogWrite(M1_R, low);
    digitalWrite(M2_EN, LOW);
    analogWrite(M2_F, low);
    analogWrite(M2_R, low);
    middle = false;
    left = false;
    right = false;
    value[6] = false;
  }
```
###### Travese white line control
#### Distance Sensor
##### Pre-Setup
##### Setup
##### Loop
#### Battery Voltage
##### Pre-Setup
##### Setup
##### Loop
#### Startup-Tune
##### Pre-Setup
##### Setup
##### Loop
### With MQTT
The full .ino codefile can be found under [./Arduino/Final-MQTT](./Arduino/Final-MQTT/Final-MQTT.ino).

