#**if error occur, check every pictures have been detected chessboard corners.
#**Please delete pictures that not been detected (both eyes need to be deleted) 
import numpy as np
import cv2
import glob
import argparse

print("start stereo callibration...")

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS +
             cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
criteria_cal = (cv2.TERM_CRITERIA_EPS +
                 cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:, :2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints_l = []  # 2d points in image plane.
imgpoints_r = []  # 2d points in image plane.



##for 320X240 
images_right = glob.glob('320X240_twoeyes_calibration/right*.png')
images_left = glob.glob('320X240_twoeyes_calibration/left*.png')



images_left.sort()
images_right.sort()

for i, fname in enumerate(images_right):
            img_l = cv2.imread(images_left[i])
            img_r = cv2.imread(images_right[i])

            gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret_l, corners_l = cv2.findChessboardCorners(gray_l, (9, 6), None)
            ret_r, corners_r = cv2.findChessboardCorners(gray_r, (9, 6), None)

            # If found, add object points, image points (after refining them)
            objpoints.append(objp)

            if ret_l is True:
                rt = cv2.cornerSubPix(gray_l, corners_l, (11,11),
                                      (-1, -1), criteria)
                imgpoints_l.append(corners_l)

                # Draw and display the corners
                ret_l = cv2.drawChessboardCorners(img_l, (9, 6),
                                                  corners_l, ret_l)
                cv2.imshow(images_left[i], img_l)
                cv2.waitKey(500)

            if ret_r is True:
                rt = cv2.cornerSubPix(gray_r, corners_r, (11,11),
                                      (-1, -1), criteria)
                imgpoints_r.append(corners_r)

                # Draw and display the corners
                ret_r = cv2.drawChessboardCorners(img_r, (9, 6),
                                                  corners_r, ret_r)
                cv2.imshow(images_right[i], img_r)
                cv2.waitKey(500)
    
            img_shape = gray_l.shape[::-1]

rt, M1, d1, r1, t1 = cv2.calibrateCamera(objpoints, imgpoints_l, img_shape, None, None)
rt, M2, d2, r2, t2 = cv2.calibrateCamera(objpoints, imgpoints_r, img_shape, None, None)



flags = 0
flags |= cv2.CALIB_FIX_INTRINSIC


flags |= cv2.CALIB_USE_INTRINSIC_GUESS

flags |= cv2.CALIB_FIX_FOCAL_LENGTH

flags |= cv2.CALIB_ZERO_TANGENT_DIST


stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER +
                    cv2.TERM_CRITERIA_EPS, 100, 1e-5)
ret, M1, d1, M2, d2, R, T, E, F = cv2.stereoCalibrate(objpoints, imgpoints_l,imgpoints_r, M1, d1, M2,
d2,img_shape,criteria=stereocalib_criteria, flags=flags)

print('Intrinsic_mtx_left(K_left)', M1) #intrinsic matrix left
print('dist_left', d1) #distortion coefficient left
print('Intrinsic_mtx_right(K_right)', M2) #intrinsic matrix right
print('dist_right', d2) #distortion coefficient right
print('R', R)
print('T', T)
print('E', E)
print('F', F)


print('')

camera_model = dict([('M1', M1), ('M2', M2), ('dist1', d1),
                ('dist2', d2), ('rvecs1', r1),
                ('rvecs2', r2), ('R', R), ('T', T),
                ('E', E), ('F', F)])

cv2.destroyAllWindows()

