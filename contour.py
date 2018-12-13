import cv2
import numpy as np

image = cv2.imread('leftarr.png')
blurred = cv2.GaussianBlur(image, (5,5), 0)
hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

lower_red = np.array([-10, 100, 100])
upper_red = np.array([10, 255, 255])
mask = cv2.inRange(hsv, lower_red, upper_red)


image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

cv2.drawContours(image, contours, -1, (0,255,0), 3)

cv2.namedWindow('Display', cv2.WINDOW_NORMAL)
cv2.imshow('Display', image)
cv2.waitKey()

