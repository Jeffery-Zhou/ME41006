import cv2
import numpy as np

##**copy and paste 'mtx' and 'dist' from the result of CameraCalibrate.py to here
mtx=np.array([[769.91451428,   0.        , 329.97018993],
       [  0.        , 762.60109019, 222.46135299],
       [  0.        ,   0.        ,   1.        ]])
dist=np.array([[-5.87181123e-01,  1.08227147e+00, -1.98149204e-02,
         3.40721662e-04, -2.64198265e+00]])

##import distort picture here
img=cv2.imread("calibrate_picture/pic0.png")
h,w=img.shape[:2]
newcameramtx,roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
#undistorted picture
map1, map2 =cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
dst=cv2.remap(img,map1,map2,cv2.INTER_LINEAR)


cv2.imshow("distortion",img)
cv2.imshow("undistortion",dst)
cv2.imwrite("undistort.png",dst)

cv2.waitKey(0)
cv2.destroyAllWindows()
