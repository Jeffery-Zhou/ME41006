import Adafruit_PCA9685
import cv2
import numpy as np

#the function control the motion of the motor 
def set_servo_angle(channel,angle):
    if angle>180:
        print("out of range!")
        angle = 180
    elif angle<0:
        print("out of range!")
        angle = 0
    angle=4096*((angle*11)+500)/20000
    pwm.set_pwm(channel,0,int(angle))
    

#set up the working environment of the servo motor
pwm = Adafruit_PCA9685.PCA9685(0x41)
pwm.set_pwm_freq(50)

while True:
    i = raw_input("Please input the position of the motor (0-180),enter q to quit:")
    if i == 'q':#input 'q' in the command window, or python will treat it as a variable
        break
    else:
        i=np.float(i)
        set_servo_angle(2,i)
