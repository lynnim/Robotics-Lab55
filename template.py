# Python program to illustrate 
# multiscaling in template matching 
# import cv2 
# import numpy as np 

# # Read the main image 
# img_rgb = cv2.imread("shape.png") 

# # Convert to grayscale 
# img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY) 

# # Read the template 
# template = cv2.imread("leftred.png",0) 

# # Store width and heigth of template in w and h 
# w, h = template.shape[::-1] 
# found = None

# for scale in np.linspace(0.2, 1.0, 20)[::-1]: 

# 	# resize the image according to the scale, and keep track 
# 	# of the ratio of the resizing 
# 	resized = imutils.resize(img_gray, width = int(img_gray.shape[1] * scale)) 
# 	r = img_gray.shape[1] / float(resized.shape[1]) 

# 	# if the resized image is smaller than the template, then break 
# 	# from the loop 
# 	# detect edges in the resized, grayscale image and apply template 
# 	# matching to find the template in the image edged 
# 	# = cv2.Canny(resized, 50, 200) result = cv2.matchTemplate(edged, template, 
# 	# cv2.TM_CCOEFF) (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result) 
# 	# if we have found a new maximum correlation value, then update 
# 	# the found variable if found is None or maxVal > found[0]: 
# 	if resized.shape[0] < h or resized.shape[1] < w: 
# 			break
# 	found = (maxVal, maxLoc, r) 

# # unpack the found varaible and compute the (x, y) coordinates 
# # of the bounding box based on the resized ratio 
# (_, maxLoc, r) = found 
# (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r)) 
# (endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r)) 

# # draw a bounding box around the detected result and display the image 
# cv2.rectangle(image, (startX, startY), (endX, endY), (0, 0, 255), 2) 
# cv2.imshow("Image", image) 
# cv2.waitKey(0) 




import cv2
import numpy as np
 
img = cv2.imread("shape.png")
gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
template = cv2.imread("leftred.png", 0)
w, h = template.shape[::-1]
 
result = cv2.matchTemplate(gray_img, template, cv2.TM_CCOEFF_NORMED)
loc = np.where(result >= 0.4)
 
for pt in zip(*loc[::-1]):
    cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 3)
 

cv2.namedWindow("img", cv2.WINDOW_NORMAL)
cv2.resizeWindow("img", 600, 600) 
cv2.imshow("img", img)
#cv2.imshow("img", img)
 
 
cv2.waitKey(0)
cv2.destroyAllWindows()
 
