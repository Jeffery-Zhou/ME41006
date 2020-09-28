from __future__ import division
import Adafruit_PCA9685
import math
import serial
import codecs
import sys
import time
import os

#initialize 
XP = 90
YP = 90
ZP = 90
ax0 = 0
ay0 = 0
az0 = 0
last_ax = 0
last_ay = 0
last_az = 0
flag = 0 #Used in storing the initial position
pwm = Adafruit_PCA9685.PCA9685(0x41) #the i2c address of the device to drive the motor
pwm.set_pwm_freq(50) #set the frequency
#the funtion to rotate the servo motor
def set_servo_angle(channel,angle):
    if angle>180:
        print("out of range!")
        angle = 180
    elif angle<0:
        print("out of range!")
        angle = 0
    angle=4096*((angle*11)+500)/20000
    pwm.set_pwm(channel,0,int(angle))
#convert the hex value into decimal number
def convert(hexVal):
    return int(codecs.encode(hexVal,'hex'), 16)
#initialize the position of motor
set_servo_angle(1,XP)
set_servo_angle(3,YP)
set_servo_angle(2,ZP)
print("wait for 5 sec")
time.sleep(5)
#open the sensor
sensor = serial.Serial(port='/dev/ttyUSB0', baudrate='9600', timeout = 1)


while True:
    #look for the start byte
    data = sensor.read(size = 1)
    if data == b'\x55':
        print('data recieved')
        sensor.read(size = 10)
        break
print('trying',data)

try:
    while True:
        #read the data from the IMU
        data = sensor.read(size = 11)
        if not len(data)==11:
            print("Byte error!")
            break
        if data[1]== b'\x50':
            print('pass')
            pass
        # read the position 
        if data[1]== b'\x53':
            hexVal = []
            for i in range(11):
                hexVal.append(convert(data[i]))
            ax = ((hexVal[3]<<8)|(hexVal[2]&0xff)) / 32768 *180
            ay = ((hexVal[5]<<8)|(hexVal[4]&0xff)) / 32768 *180
            az = ((hexVal[7]<<8)|(hexVal[6]&0xff)) / 32768 *180
            if ax > 180:
                ax -= 360
            if ay > 180:
                ay -= 360
            if az > 180:
                az -= 360
            #store the first set of values as the initial position 
            if flag == 0:
                ax0 = ax
                ay0 = ay
                az0 = az
                flag += 1
            #move the servo motor according to the current position
            else:
                if (ay - ay0)>0.5 or (az - az0)>0.5 or (ay - ay0)<-0.5 or (az - az0)<-0.5:
                    
                    YP = YP + (ay - ay0)*1.5/2
                    XP = XP - (az - az0)*1.5/2
                    print('XP:',XP)
                    print('YP:',YP)
                    set_servo_angle(1,XP)
                    set_servo_angle(3,YP)
            
            #print(round(ax, 3), round(ay, 3), round(az, 3))
            
except KeyboardInterrupt:
    sensor.close()
    print('Sensor closed!')