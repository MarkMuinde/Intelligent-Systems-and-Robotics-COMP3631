import cv2
import numpy as np


image = cv2.imread("red.png", 1)
cv2.imshow("test", image)

hsv_green_lower = np.array([0, 100, 20])
hsv_green_upper = np.array([10 , 255, 255])

hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
mask_green = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

hsv_img_mask_green = cv2.bitwise_and(image, image, mask=mask_green)

img_green_contour = cv2.cvtColor(hsv_img_mask_green, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(img_green_contour, 50, 255, 0)
greencontours, gHierarchy = cv2.findContours(img_green_contour, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

print("1")
if len(greencontours)>0:

    # There are a few different methods for identifying which contour is the biggest
    # Loop through the list and keep track of which contour is biggest or
    # Use the max() method to find the largest contour
    c = max(greencontours, key=cv2.contourArea)

    M = cv2.moments(c)
    cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

    #Check if the area of the shape you want is big enough to be considered
    # If it is then change the flag for that colour to be True(1)
    if cv2.contourArea(c) > 500: #<What do you think is a suitable area?>:
        # Alter the value of the flag
        (cx, cy), radius = cv2.minEnclosingCircle(c)
        radius = int(radius)
        cv2.circle(image,(int(cx),int(cy)),radius,(0, 0, 255),2)


cv2.imshow("mask", mask_green)
cv2.imshow("mask", hsv_img_mask_green)

cv2.imshow("mask", image)
cv2.waitKey(0)
