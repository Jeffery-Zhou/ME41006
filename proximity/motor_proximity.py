from __future__ import print_function
import time
import serial
import numpy as np
import Adafruit_PCA9685
import threading

def set_servo_angle(channel, angle):
    angle = 4096 * ((angle * 11) + 500) / 20000
    pwm.set_pwm(channel, 0, int(angle))
    
this_error=0
last_error=0
Y_P=90
Z_P=90
YP=0
ZP=0

sensor = serial.Serial (port='/dev/ttyUSB0',baudrate='9600',timeout=1)    #Open port with baud rate

pwm = Adafruit_PCA9685.PCA9685(0x41)
pwm.set_pwm_freq(50)
set_servo_angle(2,90)
set_servo_angle(3,90)
print('Ctrl-C to stop')
def xx(Y_P, Z_P):
    set_servo_angle(2,Y_P)
    
    set_servo_angle(3,Z_P)
   


def get_lenth():
    string=''
    while True:
        data=sensor.read(size=1)
        if data=='\r':
            pass
        elif data=='\n':
            print(string)
            get_angle(string)
            break
        else:
            string=string+data
        
def get_angle(lenth):
    global Y_P
    global Z_P
    global last_error
    lenth=lenth.replace('m','')
    l=int(lenth)
    print('l=',l)
    #if distance is bigger than 300mm, it will not move
    if l<300:
        this_error=150-l #keep distance 150mm
        pwm=0.05*this_error
        print('pwm=',pwm)
        YP=-pwm
        ZP=pwm
        Y_P=Y_P+YP
        Z_P=Z_P+ZP
        if Y_P >= 160:
            Y_P= 160
        if Z_P >= 160:
            Z_P = 160
        if Y_P <= 20:
            Y_P = 20
        if Z_P <= 20:
            Z_P = 20
        print("angle_y=",Y_P)
        print("angle_z=",Z_P)
        last_error=this_error
        tid = threading.Thread(target=xx, args=(Y_P,Z_P))
        tid.setDaemon(True)
        tid.start()
        
try:
    while True:
        get_lenth()
        
          
except KeyboardInterrupt:
    sensor.close()
    print('Sencor closed!')
