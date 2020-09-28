import cv2
import numpy as np

def nothing(x):
    pass

# Create a black image, a window
img = np.zeros((480,640,3), np.uint8)
cv2.namedWindow('image')
print("Press 'q' to exit the program")
# create trackbars for color change
cv2.createTrackbar('H','image',0,255,nothing)
cv2.createTrackbar('S','image',0,255,nothing)
cv2.createTrackbar('V','image',0,255,nothing)

# create switch for ON/OFF functionality
while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(1) 
    if k == ord('q'):
        break

    # get current positions of four trackbars
    h = cv2.getTrackbarPos('H','image')
    s = cv2.getTrackbarPos('S','image')
    v = cv2.getTrackbarPos('V','image')



    img[:] = [h,s,v]
    img = cv2.cvtColor(img,cv2.COLOR_HSV2BGR)

cv2.destroyAllWindows()
