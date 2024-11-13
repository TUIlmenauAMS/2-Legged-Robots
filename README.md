# 2-Legged-Robots

Program and 3D printing parts and description for little 2-legged robots
The "stl" parts are for my Raspirobo7 robot.
The "..bin.stl" files are binare stl files, which are needed for a MuJoCo simulation.
(see: https://colab.research.google.com/github/deepmind/mujoco/blob/main/python/tutorial.ipynb)

It needs:

- 2 foot_round075.stl

- 1 hueftservos.stl

- 2 knieservo_scad.stl

- 1 ruecken2.stl

Further parts:

- It is controlled by a Raspberry Pi Zero WH.

- Terratec P1 Powerbank Li-Ion 2600 mAh 171649, 5.49eur

- 5 servo motors REELY 36 ANALOG- SERVO 0008
(Nr. 1365555 / 4016138986273 or successor 2148502) at 4.99 eur (each)

- 3 AXIS DIGITAL ACCELERATION SENSOR MO, e.g. MMA8452Q
650514 / 4053199509867 , 6.99 eur.
Alternative MAKERFACTORY MF-6402108 accelerometer 
https://www.conrad.de/de/p/makerfactory-mf-6402108-beschleunigungssensor-1-st-2134036.html
7.49 eur.

- Elektrolytic-Capacitor 2200 uF, 16V,
Nr.: 1328835 - 62
Conrad Nr.: KSE228M016S1GBH25KFT, 0.20 eur

- 1 or 2 Ohm resistor 

- 4 wires, blue, yello, red, green.

- Ada3416  Adafruit 16-channel PWM/Servo.. 10.25
https://shop.pimoroni.com/products/adafruit-16-channel-pwm-servo-bonnet-for-raspberry-pi?variant=2821150736394 

- Micro SD Card, 32 GB, Noobs, 9.73 eur
https://shop.pimoroni.com/products/noobs-32gb-microsd-card-3-1?variant=31703694245971

The 2 ohm resistor is for the low pass filtered power suppy of the servo board. Since the servos can draw high power peaks, the 2200 uF capacitor is needed to supply thos power peaks. The resistor limits the current from the battery, to keep its volatge stable, to avoid the raspberry pi rebooting from a voltage drop. The 2 ohm resistor is solered in the servo board from the upper 5V soldering hole (viewed from looking at the front of the robot) to the left round soldering hole of the (unused) power plug. 

This can be seen in Raspirobo7_PXL_20241113_201906613.jpg.

The acceleration sensor has the connections GND, scl, sda, 3V. They need to be connected, using flexible wires and soldering, to the corresponding soldering holes on the upper part ot the servo board.

As a first test after completing the hardware: after connecting with power, the LED's of both, the raspberry pi zero and also the servo shield, should light up. If not, the power is missing there.

The program "raspirobo7servoset90_only.py" sets all servos to the 90 degree position, for adjusting the legs and back to be straight at that position. This can also be used for testing the servos.

The program "raspirobo7servosetVertTilt.py" is for testing the accelerometer and find a vertical tilt angle that lets the robot stand in rest. This angel is then to be entered in the walking program "raspirobo7_balancing_learning2.py".

The program to let the robot walk in a learning manner is: "raspirobo7_balancing_learning2.py"

It uses our probabilistc optimization of "Random Directions" for learning and optimizing the walk. Hence it might take a while in the beginning until it learns to balance and walk.

Also see the information and videos on our Robotics website:

https://www.tu-ilmenau.de/en/university/departments/department-of-electrical-engineering-and-information-technology/profile/institutes-and-groups/applied-media-systems-group/research-and-study-projects/research-projects/robotic-bipedal-robots
