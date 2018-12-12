import cv2
import numpy as np

# Read the main image 
img = cv2.imread("shape.png")

# Convert it to grayscale 
gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
template = cv2.imread("star.png", cv2.IMREAD_GRAYSCALE) 

result = cv2.matchTemplate(gray_img,template,cv2.TM_CCOEFF_NORMED) 


cv2.imshow("img", img)
cv2.imshow("result", result)
cv2.waitKey(0)
cv2.destroyAllWindows()
