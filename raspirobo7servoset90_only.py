#Program to set the "Raspirobo7" Robot servos to zero.
#For tuning the offsets until the legs are straight down.
#Run it with:
#python3 raspirobo6servoset90_only.py
#
#Activate I2C bus on Raspi with "sudo raspi-config"
#Then:
#sudo apt-get install build-essential libi2c-dev i2c-tools python-dev libffi-dev
#sudo pip install cffi smbus2 (for python2, for python3 it should be smbus and pip3)
#sudo pip3 install adafruit-circuitpython-servokit
#pip3 install lsm303d
#name: raspirobo7
#Gerald Schuller, October 2022

#import smbus2 as smbus #for Python2
#import smbus   #for Python3
import time

# Get I2C bus
#bus = smbus.SMBus(1)
#For the servos:
from adafruit_servokit import ServoKit
from lsm303d import LSM303D


#Testing:
if __name__ == '__main__':
   import numpy as np
   
   # Set channels to the number of servo channels on your kit.
   # 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
   kit = ServoKit(channels=16)
   
   #tune to balance upright:
   verttiltdeg = 0 #tilt forward in degrees
   hortiltdeg = 0 #tilt left in degrees (right tilt: negative)
   huefterechts_offset=0.0 - verttiltdeg #-60 tilt forward
   hueftelinks_offset=0.0 + verttiltdeg #+60 tilt forward
   knierechts_offset=0.0 + hortiltdeg  #+1 #spread lower legs, tilt right
   knielinks_offset=0.0 + hortiltdeg #-1 spread lower legs
   spine_offset =0  #offset for the spine servo

   huefterechts=kit.servo[8]
   huefterechts.angle=100.0 + huefterechts_offset #+45 #-60 #tilt forward
   hueftelinks=kit.servo[9]
   hueftelinks.angle=90.0 + hueftelinks_offset #-15 #+60
   knierechts=kit.servo[10]
   knierechts.angle=90.0  + knierechts_offset  # +135: all the way back  #below 90: move lower leg forward
   knielinks=kit.servo[11]
   knielinks.angle=90.0  + knielinks_offset  #below 90: move lower leg backwards
   spine=kit.servo[12]
   spine.angle= 90 + spine_offset #+30 #below 90: tilt more right
   
