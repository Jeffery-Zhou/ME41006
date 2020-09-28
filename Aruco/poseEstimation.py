import numpy as np
import time
import cv2
import cv2.aruco as aruco
import math

#open camera
cap = cv2.VideoCapture(0)
#set the window size
cap.set(3, 640)
cap.set(4, 480)
#intrinsic parameters of the camera, which can be obtained from the calibration program
mtx = np.array([[733.84324593,   0.        , 336.22571254],
       [  0.        , 725.41228837, 244.72227331],
       [  0.        ,   0.        ,   1.        ]])
dist = np.array([[-0.43567675, -0.10998935, -0.0024313 , -0.00220639, -0.14920207]])

font = cv2.FONT_HERSHEY_SIMPLEX


while True:
    #show the framesget rid of that nasty numpy value array error
    ret, frame = cap.read()
    #Gaussian blur
    frame = cv2.GaussianBlur(frame,(5,5),0)
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) #change the frame into grayscale picture
    parameters = aruco.DetectorParameters_create()
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250) #get the aruco marker dictionary
    #invoke the function that detect the aruco marker
    corners, ids, rIP = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    if ids != None:#Aruco marker found
        #estimate the pose
        aruco.drawDetectedMarkers(frame,corners)
        rvec, tvec= aruco.estimatePoseSingleMarkers(corners, 0.158, mtx, dist)
        #get rid of that nasty numpy value array error
        (rvec-tvec).any()
        for i in range(rvec.shape[0]):
            aruco.drawAxis(frame, mtx, dist, rvec[i,:,:],tvec[i,:,:],0.1)    
        print("rvec","%f "%rvec[0][0][0]+"%f "%tvec[0][0][1]+"%f "%tvec[0][0][2])
        print("tvec: "+"%fm "%tvec[0][0][0]+"%fm "%tvec[0][0][1]+"%fm "%tvec[0][0][2])
        aruco.drawDetectedMarkers(frame,corners,ids,(0,0,255))
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    cv2.imshow("Capture",frame)
cap.release()
cv2.destroyAllWindows()
