import cv2
import numpy as np

def nothing(x):
    pass

# Create a black image, a window
img = np.zeros((480,640,3), np.uint8)
cv2.namedWindow('image')
print("Press 'q' to exit the program")
# create trackbars for color change
cv2.createTrackbar('B','image',0,255,nothing)
cv2.createTrackbar('G','image',0,255,nothing)
cv2.createTrackbar('R','image',0,255,nothing)

# create switch for ON/OFF functionality
while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(1) 
    if k == ord('q'):
        break
    # get current positions of four trackbars
    B = cv2.getTrackbarPos('B','image')
    G = cv2.getTrackbarPos('G','image')
    R = cv2.getTrackbarPos('R','image')



    img[:] = [B,G,R]
    

cv2.destroyAllWindows()
