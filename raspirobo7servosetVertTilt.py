#Program to set the "Raspirobo7" Robot servos to a suitable vertical tilt.
#It measures the vertical angle given to the robot by hand, and then assigns that angel to the hip servos.
#Run it with:
#python3 raspirobo6servosetVertTilt.py
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

lsm = LSM303D(0x1d)  # Change to 0x1e if you have soldered the address jumper

def readaccelerometer():
   #Read current accelerometer values
   #returns:
   #List of 3 float acceleration-values, [x,y,z]
   xyz = lsm.accelerometer()
   
   return xyz

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
   
   knierechts=kit.servo[10]
   knierechts.angle=90.0  + knierechts_offset  # +135: all the way back  #below 90: move lower leg forward
   knielinks=kit.servo[11]
   knielinks.angle=90.0  + knielinks_offset  #below 90: move lower leg backwards
   spine=kit.servo[12]
   spine.angle= 90 + spine_offset #+30 #below 90: tilt more right
   huefterechts=kit.servo[8]
   hueftelinks=kit.servo[9]
   
   while True:
      (X,Y,Z)=readaccelerometer()
      X=-X; Z=-Z #sign change for rotated accelerometer
      #verttilt= np.sin(verttiltdeg/180*3.14)
      #verttilt from Y component of accelerrometer
      #verttilt=(Y)
      print("Y=", Y)
      verttiltdeg= (180/np.pi*np.arcsin(np.clip(-1,1,Y*1.0))/2)*2 #conversion to degrees from accelerometer, quantized
      print("verttiltdeg=", verttiltdeg) #40 Grad scheint gut zu sein
      huefterechts_offset = verttiltdeg
      #huefterechts_offset += 5*np.sign(verttiltdeg-huefterechts_offset)
      hueftelinks_offset = - verttiltdeg
      #print("huefterechts_offset=", huefterechts_offset)
      #hueftelinks_offset +=  5*np.sign(hueftelinks_offset-verttiltdeg)
      huefterechts.angle=np.clip(0, 180, 100.0 + huefterechts_offset) #+45 #-60 #tilt forward
      hueftelinks.angle=np.clip(0, 180, 90.0 + hueftelinks_offset) #-15 #+60
      time.sleep(0.5)
      
