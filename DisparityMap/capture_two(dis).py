import cv2
import time
l=0
r=0
capl=cv2.VideoCapture(2)
capr=cv2.VideoCapture(0)
capl.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
capl.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
capr.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
capr.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
capl.set(cv2.CAP_PROP_FPS,5)
capr.set(cv2.CAP_PROP_FPS,5)
print("start camera...")
time.sleep(1)
print("Press 'c' to capture pictures")
print("Press 'q' to exit the program")
while True:
    ret,framel=capl.read()
    ret,framer=capr.read()
    k=cv2.waitKey(1)
    cv2.imshow("left_eye",framel)
    cv2.imshow("right_eye",framer)
    if k==ord('c'):
        w="lefteye"
        t="righteye"
        x=".png"
        namel=w+str(l)+x
        namer=t+str(r)+x
        
        
        cv2.imwrite(namel,framel)
        cv2.imwrite(namer,framer)
        print(namel,"  ",namer)
        l=l+1
        r=r+1

    elif k==ord('q'):
        break

capl.release()
capr.release()
cv2.destroyAllWindows()
