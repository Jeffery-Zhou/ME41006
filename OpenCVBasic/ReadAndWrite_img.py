import cv2

#Read image
picture=cv2.imread("picture.jpeg")
cv2.imshow("picture",picture)
cv2.putText(picture,'1',(1,20),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,0),2,cv2.LINE_AA)
#Show image
cv2.imshow("picture1.jpg",picture)
#Write image
cv2.imwrite("picture1.jpg",picture)
print("Write!")

cv2.waitKey(0)
cv2.destroyAllWindows()
