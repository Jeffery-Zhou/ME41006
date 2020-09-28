import cv2
import time
i=0
cap=cv2.VideoCapture(0)
print("start camera...")
time.sleep(1)
print("Press 'c' to captrue picture for calibration")
print("**Take 20-25 picture with different angles of chessboard for calibration (see example pictures in the 'Example' folder)")
print("Press 'q' to exit thr program")
while True:
    ret,frame=cap.read()
    k=cv2.waitKey(1)
    cv2.imshow("frame",frame)
    if k==ord('c'):
        w="pic"
        x=".png"
        file="calibrate_picture/"
        name=file+w+str(i)+x
        cv2.imwrite(name,frame)
        print(name)
        i=i+1
    elif k==ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()
