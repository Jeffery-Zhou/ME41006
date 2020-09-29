import cv2
import numpy as np

# Using camera(0) to capture the video
cap0 = cv2.VideoCapture(0) 


cap0.set(3, 640) # set width for the video
cap0.set(4, 480) # set height for the video
i = 1
while True:    
    # return flag of whether get image or not, and the each image in each frame
    ret0,frame0 = cap0.read() 
    
    # show each image captured and genete an image flow, AKA. video
    cv2.imshow("capture0", frame0)
    
    key = cv2.waitKey(1)
    # press q to quit
    if key == ord('q'):
        break
    #press 's' to take pictures and save them
    if key == ord('s'):
        cv2.imwrite('pictures/pic' + str(i) + '.jpg',frame0)
        i += 1
        print("image saved")
    
cap0.release()
cv2.destroyAllWindows()
