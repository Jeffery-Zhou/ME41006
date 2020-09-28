# import the necessary packages
import numpy as np
import cv2

# grab the paths to the input images and initialize our images list
print("[INFO] loading images...")

img1 = cv2.imread('./pictures/pic1.jpg')
img2 = cv2.imread('./pictures/pic2.jpg')
images = []
images.append(img1)
images.append(img2)
cv2.imshow('pic1',img1)
cv2.imshow('pic2',img2)
# initialize OpenCV's image stitcher object and then perform the image
# stitching
print("[INFO] stitching images...")
stitcher = cv2.createStitcher()
status,stitched = stitcher.stitch(images)

# if the status is '0', then OpenCV successfully performed image stitching
if status == 0:
    # write the output stitched image to disk
    cv2.imwrite('stitched.jpg', stitched)
    # display the output stitched image to our screen
    cv2.imshow("Stitched", stitched)
    cv2.waitKey(0)
# otherwise the stitching failed, likely due to not enough keypoints)
# being detected
else:
    print("[INFO] image stitching failed ({})".format(status))
