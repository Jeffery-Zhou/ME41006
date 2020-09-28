import numpy as np
import cv2
import glob
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:, :2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints_l = []  # 2d points in image plane.
imgpoints_r = []  # 2d points in image plane.

images_right = glob.glob('320X240_twoeyes/right*.png')
images_left = glob.glob('320X240_twoeyes/left*.png')
images_left.sort()
images_right.sort()

for i, fname in enumerate(images_right):
    img_l = cv2.imread(images_left[i])
    img_r = cv2.imread(images_right[i])

    h, w = img_l.shape[:2]
    size = (w, h)

    gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret_l, corners_l = cv2.findChessboardCorners(gray_l, (9, 6), None)
    ret_r, corners_r = cv2.findChessboardCorners(gray_r, (9, 6), None)

    objpoints.append(objp)
    if ret_l is True:
        rt = cv2.cornerSubPix(gray_l, corners_l, (11, 11),
                              (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,30,0.1))
        imgpoints_l.append(corners_l)
        # Draw and display the corners
        ret_l = cv2.drawChessboardCorners(img_l, (9, 6),corners_l, ret_l)
        cv2.imshow(images_left[i], img_l)
        cv2.waitKey(500)

    if ret_r is True:
        rt = cv2.cornerSubPix(gray_r, corners_r, (11, 11),
                              (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
        imgpoints_r.append(corners_r)
        # Draw and display the corners
        ret_r = cv2.drawChessboardCorners(img_r, (9, 6), corners_r, ret_r)
        cv2.imshow(images_right[i], img_r)
        cv2.waitKey(500)
cv2.destroyAllWindows()
print("Calculate parameters...")

img_shape = gray_l.shape[::-1]
rt, M1, d1, r1, t1 = cv2.calibrateCamera(objpoints, imgpoints_l, img_shape, None, None)
rt, M2, d2, r2, t2 = cv2.calibrateCamera(objpoints, imgpoints_r, img_shape, None, None)



ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(
objpoints, imgpoints_l,imgpoints_r, M1, d1, M2,d2, img_shape,
criteria=(cv2.TERM_CRITERIA_MAX_ITER+cv2.TERM_CRITERIA_EPS,100,1e-5),
flags=cv2.CALIB_SAME_FOCAL_LENGTH+cv2.CALIB_ZERO_TANGENT_DIST)

print('ret',ret)
print('Intrinsic_mtx_1', M1) #intrinsic matrix left
print('dist_1', d1) #distortion coefficient left
print('Intrinsic_mtx_2', M2) #intrinsic matrix right
print('dist_2', d2) #distortion coefficient right
print('R', R)
print('T', T)
print('E', E)
print('F', F)

#Rectified
imgLeft=cv2.imread("320X240_twoeyes/lefteye3.png")
imgRight=cv2.imread("320X240_twoeyes/righteye3.png")



R1,R2,P1,P2,Q,validPixROI1,validPixROI2=cv2.stereoRectify(M1,d1,M2,d2,size,R,T)

left_map1,left_map2=cv2.initUndistortRectifyMap(M1,d1,R1,P1,size,cv2.CV_16SC2)
right_map1,right_map2=cv2.initUndistortRectifyMap(M2,d2,R2,P2,size,cv2.CV_16SC2)

grayL=cv2.cvtColor(imgLeft,cv2.COLOR_BGR2GRAY)
grayR=cv2.cvtColor(imgRight,cv2.COLOR_BGR2GRAY)

img1_rectified=cv2.remap(grayL,left_map1,left_map2,cv2.INTER_LINEAR)
img2_rectified=cv2.remap(grayR,right_map1,right_map2,cv2.INTER_LINEAR)
result = np.concatenate((img1_rectified, img2_rectified), axis=1)
result[::20, :] = 0
original= np.concatenate((imgLeft, imgRight), axis=1)
original[::20, :] = 0
cv2.imshow("left",imgLeft)
cv2.imshow("right",imgRight)
cv2.imshow("rectified", result)
cv2.imshow("original", original)
cv2.imwrite("rectified.jpg",result)
cv2.imwrite("original.jpg", original)


cv2.waitKey()
cv2.destroyAllWindows()

