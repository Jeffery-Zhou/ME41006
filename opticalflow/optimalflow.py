import cv2
import numpy as np
import time
cap=cv2.VideoCapture(0)
cap.set(3,640) #set width
cap.set(4,480) #set high
time.sleep(0.5)


while True:
    ret,frame1=cap.read()
    ret,frame2=cap.read()
    if not ret:
        break
    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    blur1 = cv2.GaussianBlur(frame1, (15, 15), 0)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    blur2 = cv2.GaussianBlur(frame2, (15, 15), 0)
    #difference between two images
    diff=cv2.absdiff(blur1,blur2)
    #print("diff",diff)
    cv2.imshow('optimal flow', diff)
    cv2.imshow('original',frame1)
    frame1=frame2
    x=cv2.waitKey(1)
    if x==ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
