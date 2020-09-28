import cv2
import numpy as np

#create a black image
black_pic=np.zeros((300,300,3),np.uint8)
print('black_pic matrix',black_pic)
cv2.imshow('black_pic',black_pic)
colored_pic=black_pic.copy()
colored_pic[0:100,:]=[255,0,0] #blue
colored_pic[100:200,:]=[0,255,0] #green
colored_pic[200:300,:]=[0,0,255] #red
print('colored_pic matrix',colored_pic)
cv2.imshow('colored_pic',colored_pic)
print("Press 'q' to exit the program")
cv2.waitKey(0)
cv2.destroyAllWindows()
