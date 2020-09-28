import cv2
import numpy as np

cap = cv2.VideoCapture(0)

#set frame size 640x480
cap.set(3, 640)
cap.set(4, 480)
print("Press 'q' to exit the program")
while True:    
    ret,frame = cap.read()
    cv2.imshow("capture", frame)
    k=cv2.waitKey(1)
    if k == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
