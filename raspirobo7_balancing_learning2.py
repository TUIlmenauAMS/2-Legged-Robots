#Program to test the Raspirobo7 Robot with accelerometer and servo movement.
#For learning to balance itself using the random directions optimization.
#With side learning and storing of the optimized coefficients.
#Seems to be too wild after a while.
#Optimize for minimum variation for accelerometer measures despite disturbances from knee movements.
#Run it with:
#python3 raspirobo7_balancing_learning2.py
#
#You can check the I2C addresses of the devices connected to the Raspberry Pi using:
#sudo i2cdetect -y 1
#
#Includes a
#function to read accelerometer sensor MMA8452Q from the Raspberry Pi I2C port.
#See: http://ozzmaker.com/guide-to-interfacing-a-gyro-and-accelerometer-with-a-raspberry-pi/
#Based on:
#https://github.com/ControlEverythingCommunity/MMA8452Q/blob/master/Python/MMA8452Q.py
#Connections:
#http://www.pibits.net/code/mma8452q-accelerometer-and-raspberry-pi-example.php
#
#Data Sheet of MMA8452Q:
#https://www.robotshop.com/media/files/pdf/triple-axis-accelerometer-breakout-board-mma8452q-headers-datasheet.pdf
#Activate I2C bus on Raspi with "sudo raspi-config"
#Then:
#sudo apt-get install build-essential libi2c-dev i2c-tools python-dev libffi-dev
#sudo pip install cffi smbus2 (for python2, for python3 it should be smbus and pip3)
#sudo pip3 install adafruit-circuitpython-servokit
#name: raspirobo7
#Gerald Schuller, November 2022

#import smbus2 as smbus #for Python2
import smbus   #for Python3, install with pip
from lsm303d import LSM303D #install with pip
import time
import numpy as np
#from scipy.signal import medfilt
#import optimrandomdir as opt
import optimrandomdir_timevarying as opt

# Get I2C bus
bus = smbus.SMBus(1)
#For the servos:
from adafruit_servokit import ServoKit
lsm = LSM303D(0x1d)  # Change to 0x1e if you have soldered the address jumper


def readaccelerometer():
   #Read current accelerometer values
   #returns:
   #List of 3 float acceleration-values, [x,y,z]
   xyz = lsm.accelerometer()
   
   return xyz

#def balancing_obj(kcoeffs, tilts): #optimizing only side balance
#def balancing_obj(kfcoeffs, tilts): #optimizing only forward balance
def balancing_obj(coeffs, tilts):  #optimiizing both, side and forward balance
   #Objective function for balancing the robot such that the accelerometer is aligned to verttilt and hortilt, 
   #which are the sin of the angles to the vertical axis, vertical (front/back) and horizontally (left/right).
   #Arguments: kcoeffs: coefficients for the control lop
   #tilts: (verttilt, hortilt) target directions for balancing
   #Returns the error or objective function to minimize
   
   global YLP, YLPold, ydist, yold, Yvar, ZLP, ZLP1, ZLPold, ZLP1old, zdist, zold, Z, Zvar, phase, ampRH, ampLH;
   
   (verttilt, hortilt) = tilts  #hortilt is optimized
   kcoeffs=coeffs[0,:]
   kfcoeffs=coeffs[1,:]
   #hortilt=kcoeffs[3] #learned horizontal tilt
   #verttilt=kfcoeffs[3] #learned forward tilt
   
   for balancetrials in range(100): #Number of iterations for a set of control coefficients from optimizer, 
      #objective functions is returned at the end, to have a clearer picture if it is good.
      (X,Y,Z)=readaccelerometer()
      X=-X; Z=-Z #sign change for rotated accelerometer
      #print("(X,Y,Z)=", (X,Y,Z))
      #Y's:
      ydist+= (Y-verttilt)*(timestep);        #integral values (distance traveled, without artificial factor 100)
      ydist=np.clip(ydist,-0.1,0.1);
      YLPold=YLP
      YLP=0.99*YLP+0.01*Y
      yspeed=(Y-yold)/(timestep);  #differential values (speed, without artificial factor 100)
      ylpspeed=(YLP-YLPold)/(timestep)  #vert. speed from low passed version

      #Z's:
      """
      Zmem[0:(Lmed-1)]=Zmem[1:Lmed] #shift samples 
      Zmem[Lmed-1]=Z  #write latest sample, for median filter
      Z=medfilt(Zmem,kernel_size=Lmed)[int((Lmed-1)/2)]
      """
      
      zdist+= (Z-hortilt)*(timestep);
      zdist=np.clip(zdist,-0.1,0.1);

      #Sideways tilt:
      zspeed=(Z-zold)/(timestep);
      yold=Y;
      zold=Z;
      ZLPold=ZLP
      ZLP=0.99*ZLP+0.01*Z
      ZLP1old=ZLP1
      ZLP1=0.95*ZLP1+0.5*Z  
      zlpspeed=(ZLP-ZLPold)/(timestep)  #hor. speed from low passed version
      zlpspeed1=(ZLP1-ZLP1old)/(timestep) #hor. speed estimation from less low pass filtering
      
      #----------------------
      #PID control loop:
      #frontbalance = 180/3.14* 2.6*(ydist + 0.05*(Y-verttilt) -4*2.0e-2*ylpspeed) #2: schneller, without artificial factor 100
      #kf=np.array([1.3e+02, 6.3e+00, -1.2e+01, 5.6e-01]) #from learning robot 4, seems reasonably stable
      #kf=np.array([100, 100.0, -120])
      #kf=np.array([0, 6, 0])
      #kf=np.array([350, 20.0, -30])
      kf=kfcoeffs
      frontbalance = kf[0]*ydist + np.clip(kf[1]*(Y-verttilt),-10,10) + np.clip(kf[2]*ylpspeed,-10,10) 
      #sidebalance = -180/3.14* 2.9*(zdist + 0.05*(Z-hortilt) -2*4.0e-2*zlpspeed)*1.0;# k2=4e-4 #2: schneller, without artificial factor 100. k1 needs to be larger for faster steps, ca. 0.7 fuer speed 0.25. k2 was originally with - sign, but with plus seems to be better!
      #k=np.array([-3.2e+02, -1.5e+01, 3.4e+00, 1.4e-01]) #This set makes it oscillate! From learning robot4
      #Manual tuning of the PID controller: https://en.wikipedia.org/wiki/PID_controller
      #Increase each coeff until oscillation, then half it, first propoertional, then integral, then differential. 
      #k=np.array([-200, -1.5e+01, 28])
      #k=np.array([-350, -20.0, 30])
      #k=np.array([-1000, -1000.0, 1200]) #P: 30 makes it oscillate, so half: 15, I: 400 oscillates, so half, 200
      #k=np.array([-1000, -100.0, 10])
      #k=np.array([-1000, -100.0, 400])
      k=kcoeffs; #Read control coefficients from function argument
      #k=np.array([ 10.89118321,   4.93782397, -10.27297858,   3.8025568 ]); hortilt=k[3] #optimized with step excitation
      #k=np.array([ 10.89118321,   4.93782397, -10.27297858,   -0.15 ]); hortilt=k[3] #optimized with step excitation, k[3] needs to be manually tuned for back upright, more negative values tilt more left, as seen from the Robot. -0.3, -0.15. But attention: Changes because it is part of the optimization, which makes the robot sometimes turn or go in circles. So it could also be left at 0.0. Not really, kcoeffs is not used, only the preset above!
      sidebalance = k[0]*zdist + np.clip(k[1]*(Z-hortilt), -10, 10) + np.clip(k[2]*zlpspeed, -10,10) #+ np.clip(k[3]*zlpspeed1, -10,10)
      sidebalance=np.clip(sidebalance,-50,50);
      #print("sidebalance=", sidebalance)
      #print("k[0]*zdist=", k[0]*zdist, "k[1]*(Z-hortilt)=", k[1]*(Z-hortilt), "k[2]*zlpspeed=", k[2]*zlpspeed, "Z=", Z)
      #----------------------------
      #Servo control:
      angle=-frontbalance+110+1*ampRH; #sign change because of flipping hip servos, with tuning offsets. The number (110) needs to be manually tune for the legs being parallel in the beginning.
      huefterechts.angle=np.clip(int(angle + huefterechts_offset),0, 180); 
      angle=frontbalance+90+1*ampLH; #sign change because of flipping hip servos
      hueftelinks.angle=np.clip(int(angle + hueftelinks_offset),0,180); 
      #print("ampRK=", ampRK, "ampLK=", ampLK)
      
      #Independent Knee movements:
      #"""
      angle=0.4*sidebalance+knierechtsbalance+90-0-ampRK + knierechts_offset; #0.5
      knierechts.angle=np.clip(int(angle),0,180);
      #print("r=",np.clip(int(angle),0,180))
      angle=0.4*sidebalance+knielinksbalance+90+0-ampLK + knielinks_offset; #0.5
      knielinks.angle=np.clip(int(angle),0,180); 
      #print("l=",np.clip(int(angle),0,180))
      #"""
      
      #horangle=np.arcsin(hortilt)/3.14*180 #convert from radians to degrees
      #spine.angle = np.clip(int(90 + spine_offset - 0.0*horangle),0,180);
      #spine.angle = np.clip(int(90 + 0.6*sidebalance),0,180);
      spine.angle = np.clip(int(90 + 4.0*sidebalance),45,135); #2.0
      #spine.angle = 90 + spine_offset + hortiltdeg;
      #print("int(90 + spine_offset )=", int(90 + spine_offset ))
      #print("horangle=", horangle)
      #print("0.5*sidebalance=", 0.5*sidebalance)
      
      #Perturbation:
      #Move knees:
      """
      ctr+=1
      if ctr%50==0: #every second change horizontal tilt for control loop testing:
         hortiltoffset=-hortiltoffset
         #Perturbation at knees: Doesn't reall work because the balancing function overwrites it! Turned off the knee control
         angleknee=(np.random.rand()-0.5)*20 #+-5 Grad
         angleknee = -angleknee #flip direction
      """
      #Knee moves:
      """ 
      angleknee =-amptilt*np.sign(np.cos(phase))*(abs(np.cos(phase))**(0.5))
      #angleknee=(np.random.rand()-0.5)*20 #+-10 Grad
      #print("angleknee=", angleknee)
      knierechts.angle=90.0 + knierechts_offset+angleknee 
      knielinks.angle=90.0 + knielinks_offset + angleknee
      #walking movement back-forth:
      ampLH=-3.0*(np.sin(phase))
      ampRH=-3.0*(np.sin(phase))
      """
      #"""
      #Steps according to tilt:
      #hortiltoffset=-amptilt*np.cos(phase) #amptilt=-0.2,-0.08
      #angleknee=(np.random.rand()-0.5)*2 #+-1 Grad random excitation
      ampLH += np.sign(Z -hortiltadj)*5 #stepincrement
      ampLH= np.clip(ampLH, -5, 5)
      ampRH += np.sign(Z -hortiltadj)*5
      ampRH = np.clip(ampRH, -5, 5)
      #"""
      Yvar= 0.99*Yvar+0.01*((Y-YLP)**2)
      Zvar= 0.99*Zvar+0.01*((Z-ZLP)**2)
      phase+=speed
      time.sleep(timestep)
   print("Zvar=", Zvar, "Yvar=", Yvar, "Y=", Y)
   #print("kfcoeffs=", kfcoeffs)
   #print("fcoeffs=", coeffs)
   #return Zvar +Yvar
   #return Yvar
   #return Zvar
   return abs(Zvar-0.1) #0.06, 0.1, 0.15

#--------------------------------------------------------------


#Testing:
if __name__ == '__main__':
   
   #Initializations:
   # Set channels to the number of servo channels on your kit.
   # 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
   kit = ServoKit(channels=16)
   
   Lmed=3 #should be odd
   phase=0.0

   schrittzaehler=0
   ampRK=0 #right knee step
   ampLK=0 #Left knee step
   huefterechts=kit.servo[8]
   hueftelinks=kit.servo[9]
   knierechts=kit.servo[10]
   knielinks=kit.servo[11]
   spine=kit.servo[12]
         
   """
   for s in range(16):
      #test if accelerometer works, move accelerometer, the printed values should change:
      (X,Y,Z)=readaccelerometer()
      #print("Accelerometer X,Y,Z=", round(X,2),round(Y,2),round(Z,2))  
      
      #makes servos go first to positive, then to negative direction, relative to 90 degrees,
      #servos connected to connector 0 to 15, 1 period for each:
      print("Servo:", s)
      kit.servo[s].angle = 90 
   """
   #tune to balance upright:
   verttiltdeg = 32 #tilt forward in degrees
   hortiltdeg = 0 #5, +/- 10 for balancing on one foot, tilt left in degrees (right tilt: negative), should be balanced with this
   verttilt= np.sin(verttiltdeg/180*3.14) #0.5
   hortiltadj= np.sin(hortiltdeg/180*3.14) #adjusted horizontal tilt, from balancing in rest
   
   hueftelinks_offset= -(verttiltdeg) #+60 tilt forward, negative: move leg backward, negative sign because robo7 has turned around hip servos
   huefterechts_offset= -( -verttiltdeg) #-60 tilt forward, negative: move leg forward, negative sign because robo7 has turned around hip servos
   knielinks_offset= 5.0  + hortiltdeg #-1 spread lower legs, positive: move leg left (seen from the robot)
   knierechts_offset= 0.0  + hortiltdeg  #+1 #spread lower legs, tilt right, negative: move leg right
   #spine_offset = 0  #offset for the spine servo, negative: to the right (seen from robot), should be straight up (accelerometer data=0) for hortiltdeg=0 and straight legs
   


   
   ampRH=0.0;ampLH=0.0; ampLK=0.0; ampRK=0.0 #amplitudes for the steps
   knierechtsbalance=0.0
   knielinksbalance=0.0
   #Global low pass filter variables for the balancing control loop:
   ydist=0
   yold=0
   zdist=0.0;
   zold=0.0
   ZLP=0.0
   ZLPold=0.0
   ZLP1=0.0 #for less low pass filtering of Z
   ZLP1old=0.0
   YLP=0.0
   YLPold=0.0
   Yvar=0.0; Zvar=0;
   Zmem=np.zeros(Lmed) #For median filter of length Lmed
   Z=0;
   
   timestep=0.01 #time step in seconds, larger for optimization?
   hortiltoffset=0.0 #Horizontal tilt variable for walking
   #hortiltoffset=0
   ctr=0
   angleknee=6 #degrees
   phase=0;
   amptilt=8 #6 #amplitude of knees in degrees
   speed=0.1;  #phase increment in radiants for servo, speed is forward
   
   """
   #setting before control loop not really necessary:
   huefterechts.angle=100.0 + huefterechts_offset #-60 #tilt forward
   hueftelinks.angle=90.0 + hueftelinks_offset #+60
   angleknee=0
   knierechts.angle=90.0 + knierechts_offset+angleknee #+1 #spread lower legs
   knielinks.angle=90.0 + knielinks_offset + angleknee #-1
   spine.angle= 90 + hortiltdeg#
   """
   
   #initx=np.array([-350, -20.0, 30])
   #initx=(np.random.rand(4)-0.5) #optimizing only side balance
   """
   try:
      #use loadtxt instead of pickle for easier editing:
      print("Load robocoeffs7.txt")
      initx= np.loadtxt("/home/schuller/Robots/robocoeffs7.txt")
   except IOError:
      print("robocoeff.pickle file not found, fresh start")
      #k=np.array([-166.2, -8.31, -0.066, 0.1])
      initx=(np.random.rand(2,4)-0.5) #optimizing both side and forward balance, and hor tilt
   """
   initx=(np.random.rand(2,4)-0.5) #optimizing both side and forward balance, and hor tilt
   args=(verttilt , hortiltadj+hortiltoffset)
   xmin= opt.optimrandomdir(balancing_obj, initx, args=args, iterations=100, startingscale=1.0, endscale=0.0) #100 iterations should be about 100 seconds.
   
   #use savetxt instead of pickle for easier editing:
   """
   os.system('cp -p robocoeffs7.txt robocoeffs7_o.txt') #backup in case of losses
   np.savetxt("robocoeffs7.txt", xmin)
   print("Wrote k and kf to file robocoeffs7.txt")
   """

   """
   while True:
      tilts=(verttilt , hortiltadj+hortiltoffset)
      kcoeffs=np.array([-350, -20.0, 30])
      balancing_obj(kcoeffs, tilts)
      
      #print("next setp")
      
      phase+=speed
      #time.sleep(timestep)
   """
      
