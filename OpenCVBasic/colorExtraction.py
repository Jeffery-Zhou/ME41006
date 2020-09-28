import cv2
import numpy as np

cap = cv2.VideoCapture(0)

green_lower=np.array([35,43,46])
green_upper=np.array([77,255,255])
cap.set(3, 640)
cap.set(4, 480)
print("Press 's' to save image")
print("Press 'q' to exit program")
while True:    
    ret,frame = cap.read()
    original = frame.copy()
    frame=cv2.GaussianBlur(frame,(5,5),0)
    hsv= cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,green_lower,green_upper)
    mask=cv2.erode(mask,None,iterations=2)
    mask=cv2.dilate(mask,None,iterations=2)
    mask=cv2.GaussianBlur(mask,(3,3),0)
    res=cv2.bitwise_and(frame,frame,mask=mask)
    cv2.imshow("original", frame)
    cv2.imshow("mask", mask)
    cv2.imshow("res", res)
    k=cv2.waitKey(1)
    if k == ord('s'):
        cv2.imwrite('original.jpg',original)
        cv2.imwrite('mask.jpg',mask)
        cv2.imwrite('result.jpg',res)
        print("Save!")
        break
    elif k== ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
