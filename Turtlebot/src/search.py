import cv2
import numpy as np

red = np.uint8([[[41,54,133]]])
hsv_red = cv2.cvtColor(red, cv2.COLOR_BGR2HSV)
print(hsv_red)