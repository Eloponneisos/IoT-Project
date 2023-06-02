# Build Tutorial
In this tutorial I will tell you step by step how to make this line tracking vehicle.
## Step 1: Components
First of all you need to buy all the components listed below:
1. ESP-32
2. 1 5xTCRT5000 IR sensor with Limit Switch
3. 2 5V DC-Motor with corresponding wheels
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

Depending on your preferences you can also buy the pcb or you can use a perfboard on which you attach everything.

## Step 2: Soldering
In this step you have to solder all components either to the pcb or to the perfboard. I would suggest that you use header pins for the esp-32, the l293d motor driver IC and the MCP23016 I/O Expander as to not damage them when soldering and for easy removal and replacement.

## Step 3: Body plate
You can make this out of your material of choice. I've made a simple diagram for how you should cut it, for the castorwheel I just put 1 hole which should be the turning point of the castorwheel.
![Diagram](./Autonomous%20line%20tracking%20car%20Design%20Drawing%20v4.png)
The original file can be found [here](./Autonomous%20line%20tracking%20car%20Design%20Drawing%20v4.png).

## Step 4: Combining
In this step you are going to build up the car. 
-   First start by attaching the castor wheel and the motors with their wheels.
-   After that you want to attach the TCRT sensor which has to be placed almost on the ground, but not touching the ground. 
-   After that we can attach the HC-SR04 module who's connectors should fit snuggly inside of the premade hole.
-   Then we need to secure the pcb or perfboard to the bodyplate.
-   Lastly we need to run the cables through the hole also prescribed in the drawing.

## Step 5: Code
You can either download and upload my code using the arduino IDE. In which case I would recommend for you to first test the individual pieces who's code can be found [here](./Arduino/Individual%20test/) afterwhich you can upload the full final sketch. If you decide that you want to develop your own code which I highly recommend, then you should have a look at my code explanation [here]() for inspiration.