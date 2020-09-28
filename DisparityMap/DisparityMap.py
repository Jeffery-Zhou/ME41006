import numpy as np
import cv2
##Stereo 3D reconstruction


sad_windowSize=5


#camera parameters
#parameter for 320x240
#**paste result of stereo_callibration.py
K_left =np.array([[397.57163209,   0.        , 178.12148514],
       [  0.        , 392.44985343, 102.41233454],
       [  0.        ,   0.        ,   1.        ]])
dist_left = np.array([[-4.16782160e-01, -7.10449667e-01,  2.26107705e-03,
        -6.81215687e-03,  5.05280240e+00]])
K_right=np.array([[410.30620761,   0.        , 173.74718886],
       [  0.        , 402.43191081,  61.91498964],
       [  0.        ,   0.        ,   1.        ]])
dist_right=np.array([[-0.42287717,  0.09728812,  0.00750983, -0.00287711, -0.06335423]])
R=np.array([[ 0.99989143, -0.00899753, -0.01166946],
       [ 0.00933738,  0.99952402,  0.02940328],
       [ 0.01139935, -0.02950905,  0.99949951]])
T=np.array([[-2.26105305],
       [ 0.17822233],
       [ 0.65133303]])
#Load pictures
img_left=cv2.imread("lefteye0.png")
img_right=cv2.imread("righteye0.png")
h,w = img_left.shape[:2]
size=(w,h)

#undistorted and rctified
R1,R2,P1,P2,Q,validPixROI1,validPixROI2=cv2.stereoRectify(K_left,dist_left,K_right,dist_right,size,R,T)
left_map1,left_map2=cv2.initUndistortRectifyMap(K_left,dist_left,R1,P1,size,cv2.CV_16SC2)
right_map1,right_map2=cv2.initUndistortRectifyMap(K_right,dist_right,R2,P2,size,cv2.CV_16SC2)

#convert to grayscale
grayL=cv2.cvtColor(img_left,cv2.COLOR_BGR2GRAY)
grayR=cv2.cvtColor(img_right,cv2.COLOR_BGR2GRAY)


img1_rectified=cv2.remap(grayL,left_map1,left_map2,cv2.INTER_LINEAR)
img2_rectified=cv2.remap(grayR,right_map1,right_map2,cv2.INTER_LINEAR)


# create disparity map (StereoSGBM)

left_matcher = cv2.StereoSGBM_create(
    minDisparity=0,
    # Maximum disparity minus minimum disparity. The value is always greater than zero.
    # In the current implementation, this parameter must be divisible by 16.
    numDisparities=96,
    blockSize=sad_windowSize,
    P1=8*3*sad_windowSize*sad_windowSize, #8*number_of_image_channels*blockSize*blockSize
    #sad_windowsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
    P2=32*3*sad_windowSize*sad_windowSize, #32*number_of_image_channels*blockSize*blockSize
    disp12MaxDiff=1,
    #uniquenessRatio
    #Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider
    # the found match correct.
    # Normally, a value within the 5-15 range is good enough.
    uniquenessRatio=15,
    speckleWindowSize=0,
    speckleRange=2,
    preFilterCap=0,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)
right_matcher=cv2.ximgproc.createRightMatcher(left_matcher)

#post filtering

wls_filter=cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
#Lambda is a parameter defining the amount of regularization during filtering.
# Larger values force filtered disparity map edges to adhere more to source image edges. Typical value is 8000.
wls_filter.setLambda(8000)
#SigmaColor is a parameter defining how sensitive the filtering process is to source image edges.
# Large values can lead to disparity leakage through low-contrast edges.
# Small values can make the filter too sensitive to noise and textures in the source image. Typical values range from 0.8 to 2.0.
wls_filter.setSigmaColor(1.2)

print('computing disparity...') 
displ = left_matcher.compute(img1_rectified, img2_rectified)  # .astype(np.float32)/16
print("displ=",displ)
dispr = right_matcher.compute(img2_rectified, img1_rectified)  # .astype(np.float32)/16
displ = np.int16(displ)
print("np.int16.displ=",displ)

filteredImg = wls_filter.filter(displ, img1_rectified, None, dispr)  # important to put "img1_rectified" here!!!
filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
filteredImg = np.uint8(filteredImg)
cv2.imshow('Disparity Map', filteredImg)
cv2.imshow('lefteye',img_left)
cv2.imshow('righteye',img_right)
cv2.imshow('left_rectified',img1_rectified)
cv2.imshow('right_rectified',img2_rectified)
cv2.imwrite('undistorted_left.png',img1_rectified)
cv2.imwrite('undistorted_right.png',img2_rectified)
cv2.imwrite('dispartymap.png', filteredImg)
cv2.waitKey()
cv2.destroyAllWindows()
