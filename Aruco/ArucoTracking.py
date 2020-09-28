import numpy as np
import time
import cv2
import cv2.aruco as aruco
import Adafruit_PCA9685
import math
#initialize the working encironment of the servo motor
pwm = Adafruit_PCA9685.PCA9685(0x41)
pwm.set_pwm_freq(50)

def set_servo_angle(channel,angle):
    angle=4096*((angle*11)+500)/20000
    pwm.set_pwm(channel,0,int(angle))


#initialize the position of the motor
set_servo_angle(1,90)
set_servo_angle(2,90)
set_servo_angle(3,90)
#open camera
cap = cv2.VideoCapture(0)
#set window size
cap.set(3, 640)
cap.set(4, 480)
#intrinsic matrix and distortion of the camera
'''mtx = np.array([[2650.70471, 0.00000000, 332.562678],
       [0.00000000, 2742.03111, 187.132868],
       [0.00000000e+00, 0.00000000e+00, 1.00000000]])
dist = np.array([-1.34244525e+01,  1.02171517e+03,  7.32151963e-02,
        -1.62646951e-02, -3.38738968e+04])'''
mtx = np.array([[733.84324593,   0.        , 336.22571254],
       [  0.        , 725.41228837, 244.72227331],
       [  0.        ,   0.        ,   1.        ]])
dist = np.array([[-0.43567675, -0.10998935, -0.0024313 , -0.00220639, -0.14920207]])
font = cv2.FONT_HERSHEY_SIMPLEX

Y_P=90
X_P=90
Z_P=90
index = 0
distance = 0
z2 = 0


while True:
    ret, frame = cap.read()
    frame = cv2.GaussianBlur(frame,(5,5),0)
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    parameters = aruco.DetectorParameters_create()
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250) #get the aruco marker dictionary
    corners, ids, rIP = aruco.detectMarkers(gray, aruco_dict, parameters = parameters) #detect Aruco markers
    if ids != None:
        #estimate pose
        aruco.drawDetectedMarkers(frame,corners)
        rvec, tvec = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        #print('rvec:',rvec)
        #print('tvec:',tvec[0][0])
        (rvec-tvec).any()
        for i in range(rvec.shape[0]):
            aruco.drawAxis(frame, mtx, dist, rvec[i,:,:],tvec[i,:,:],0.03)
        #calaulate the position of the servo motors
        x = tvec[0][0][0]
        y = tvec[0][0][1]
        z = tvec[0][0][2]
        X_P = X_P - x/z/math.pi*30
        Y_P = Y_P - y/z/math.pi*30
        #use the pose in the fifth frame as the initial pose
        if index == 5:
            distance = z
            z2 = z
        if index >= 5:
            Z_P = Z_P + (distance - z)*100/180/math.pi*120
        index = index + 1
        
        if X_P>175:
            X_P=174
        if X_P<0:
            X_P=0
        if Y_P>175:
            Y_P=174
        if Y_P<0:
            Y_P=0
        if Z_P < 30:
            Z_P = 30
        if Z_P>150:
            Z_P = 150
        print("index: ", index)
        print("distance: ",distance)
        print("z:", z)
        print("Z_P: ", Z_P)
        print("Y_P", Y_P + (90 - Z_P))
        print('\n')
        
    set_servo_angle(1,X_P)
    #set_servo_angle(2,Y_P)
    if index >= 5:
        set_servo_angle(3,Z_P)
        set_servo_angle(2,Y_P + (90 - Z_P))
    cv2.imshow("Capture",frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
