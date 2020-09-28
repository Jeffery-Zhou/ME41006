import Adafruit_PCA9685
import cv2
import math
import time
#open camera
cap = cv2.VideoCapture(0)
#set window size
cap.set(3, 640)
cap.set(4, 480)
def set_servo_angle(channel,angle):
    if angle>180:
        print("out of range!")
        angle = 180
    elif angle<0:
        print("out of range!")
        angle = 0
    angle=4096*((angle*11)+500)/20000
    pwm.set_pwm(channel,0,int(angle))
    
pwm = Adafruit_PCA9685.PCA9685(0x41)
pwm.set_pwm_freq(50)

#The fuction control the end effector to move along a line
def move_along_line(angle):
    beta = 0
    angles_y = 0
    print("ditrection: %d"%angle)
    while True:
        print("beta: %f"%beta)
        l = (14.5* math.sin(float(angle)/180.0*math.pi))/(math.sin(float(angle+beta)/180.0*math.pi))
        print("l:%f"%l)
        cosa = (81+5.5*5.5-l*l)/99.0
        print("cosA: %f"%cosa)
        A = math.degrees(math.acos(cosa))
        print("A: %f"%A)
        angle_z = 90+180-A
        print("angle_z: %f"%angle_z)
        
        B = beta + math.degrees(math.acos((81+l*l-5.5*5.5)/(18*l)))
        print("B: %f"%B)
        angle_y = 90-B
        print("angle_y: %f"%angle_y)
        set_servo_angle(3,angle_y)
        set_servo_angle(2,angle_z)
        beta += 3
        time.sleep(1)
        print(' ')
        if (beta + 2*angle) > 180:
            break
#current coordinate of the end effector
current_x = 0
current_y = 14.5
current_z = 0
    
X_P = 90
Y_P = 90
Z_P = 90
set_servo_angle(1,X_P)
set_servo_angle(2,Z_P)
set_servo_angle(3,Y_P)
#control the end effector to move to the certain position
def move_to_position(x,y,z):
    if x*z>0 :
        X_P = 180 - math.degrees(math.atan(z/x))
    elif x*z<0 :
        X_P = 180 - (math.degrees(math.atan(z/x)) + 180)
    else:
        if x==0:
            X_P = 90
        elif z==0:
            if x>0:
                X_P = 180
            else:
                X_P = 0
    if z>=0:
        if x==0:
            alpha = 90
        else:
            alpha = math.degrees(math.atan(float(y)/math.sqrt(math.pow(x,2)+math.pow(z,2))))
    else:
        alpha = 180 + math.degrees(math.atan(float(-y)/math.sqrt(math.pow(x,2)+math.pow(z,2))))
    
    delta = math.degrees(math.acos((81+math.pow(x,2)+math.pow(y,2)+
                                    math.pow(z,2)-30.25)/(18*math.sqrt(math.pow(x,2)+math.pow(y,2)+math.pow(z,2)))))
    Y_P = alpha-delta
    
    beta = math.degrees(math.acos((111.25 - (math.pow(x,2)+math.pow(y,2)+math.pow(z,2)))/99.0))
    Z_P = 270 - beta
   
    set_servo_angle(1,X_P)
    set_servo_angle(2,Z_P)
    set_servo_angle(3,Y_P)
    
    
                        
while True:
    ret,frame = cap.read()
    cv2.imshow("capture", frame)
    key = cv2.waitKey(10)
    #press s to control the servo motor to move backward
    if key == ord('s'):
        print("backward")
        Y_P += 3
        if Y_P >= 180:
            Y_P = 180
        set_servo_angle(3,Y_P)
    #press w to control the servo motor to move backward
    elif key == ord('w'):
        print("forward")
        Y_P -= 3
        if Y_P <= 0:
            Y_P = 0
        set_servo_angle(3,Y_P)
    #press aj to control the servo motor to move left
    elif key == ord('a'):
        print("look left")
        X_P += 3
        if X_P >= 180:
            X_P = 180
        set_servo_angle(1,X_P)
    #press d to control the servo motor to move right
    elif key == ord('d'):
        print("look right")
        X_P -= 3
        if X_P <= 0:
            X_P = 0
        set_servo_angle(1,X_P)
    #press j to control the servo motor to look down
    elif key == ord('j'):
        print("low")
        Z_P -= 3
        if Z_P <= 0:
            Z_P = 0
        set_servo_angle(2,Z_P)
    #press k to control the servo motor to look up
    elif key == ord('k'):
        print("rise")
        Z_P += 3
        if Z_P >= 180:
            Z_P = 180
        set_servo_angle(2,Z_P)
    #press l to control the end effector to move along a line
    elif key == ord('l'):
        ang = int(input("input the angle(51.85-90): "))
        if ang<51.85 or ang>90:
            print('Out of range!')
        else:
            move_along_line(ang)
        time.sleep(3)
        set_servo_angle(2,90)
        set_servo_angle(3,90)
    #press p to control the end effector to move to a certain coordinate
    elif key == ord('p'):
        x = float(input("input the x position: "))
        y = float(input("input the y (y>0) position: "))
        z = float(input("input the z position: "))
        if (math.pow(x,2)+math.pow(y,2)+math.pow(z,2)) > 210.25 or (math.pow(x,2)+math.pow(y,2)+math.pow(z,2)) < 111.25 or y<0:
            print("out of range")
        else:
            move_to_position(x,y,z)
    elif key == ord('q'):
        print('exit')
        break
    
    
cap.release()
cv2.destroyAllWindows()
