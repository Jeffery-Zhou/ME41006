#capture picture 
import cv2
import time
cap=cv2.VideoCapture(0)
print("start camera...")
time.sleep(1)
print("Press 'c' to captrue picture")
print("Press 'q' to exit the program")
while True:
    #read frame
    ret,frame=cap.read()
    
    k=cv2.waitKey(1)
    cv2.imshow("frame",frame)
    if k==ord('c'):
        print("capture!")
        cv2.imwrite("pic.png",frame)
    elif k==ord('q'):
        break

#release the resource, importnt after read video
cap.release()
cv2.destroyAllWindows()
