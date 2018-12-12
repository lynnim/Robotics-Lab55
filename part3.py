#!/usr/bin/env python
import cv2 
import numpy as np 

# Read the main image 
img = cv2.imread("shape.png") 

# Convert to grayscale 
gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 

# Read the template 
template1 = cv2.imread("left.png", cv2.IMREAD_GRAYSCALE) 
#template2 = cv2.imread("right.png", cv2.IMREAD_GRAYSCALE)

result1 = cv2.matchTemplate(gray_img, template1, cv2.TM_CCOEFF_NORMED)
#result2 = cv2.matchTemplate(gray_img, template2, cv2.TM_CCOEFF_NORMED)

cv2.imshow("img", img)
cv2.imshow("result1", result1)
cv2.waitKey(0)
cv2.destroyAllWindows()
