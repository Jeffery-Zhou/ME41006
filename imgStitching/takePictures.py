import cv2
import numpy as np

cap0 = cv2.VideoCapture(0)

cap0.set(3, 640)
cap0.set(4, 480)
i = 1
while True:    
    ret0,frame0 = cap0.read()
    
    cv2.imshow("capture0", frame0)
    
    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    if key == ord('s'):#press 's' to take pictures
        cv2.imwrite('pictures/pic' + str(i) + '.jpg',frame0)
        i += 1
        print("image saved")
    
cap0.release()
cv2.destroyAllWindows()
