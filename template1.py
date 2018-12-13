#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([19, 100, 100])
    upper_yellow = numpy.array([39, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    h, w, d = image.shape
    search_top = 5*h/6 #put the frame 5/6 of the way down
    search_bot = search_top + 20 #use the 20 units in front of the robot from that 5/6 of the way down
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
		#calculate the centriod
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
		# BEGIN CONTROL
		err = cx - w/2
		self.twist.linear.x = 0.2
		self.twist.angular.z = -float(err) / 100
		self.cmd_vel_pub.publish(self.twist)
		# END CONTROL

    else:
	    img_rgb = cv2.imread(image) 
		img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY) 
		
		# template matching
		template = cv2.imread("leftred.png",0) 
		w, h = template.shape[::-1]
		 
		result = cv2.matchTemplate(gray_img, template, cv2.TM_CCOEFF_NORMED)
		loc = numpy.where(result >= 0.4)
		 
		for pt in zip(*loc[::-1]):
		    cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 3)
    cv2.imshow("window", img_rgb)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL








#     image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#     img_rgb = cv2.imread(image) 
# 	img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY) 
	
# 	# template matching
# 	template = cv2.imread("leftred.png",0) 
# 	w, h = template.shape[::-1]
	 
# 	result = cv2.matchTemplate(gray_img, template, cv2.TM_CCOEFF_NORMED)
# 	loc = numpy.where(result >= 0.4)
	 
# 	for pt in zip(*loc[::-1]):
# 	    cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 3)
 
#     cv2.imshow("window", image)
#     cv2.waitKey(3)
#     # cv2.namedWindow("img", cv2.WINDOW_NORMAL)
# 	# cv2.resizeWindow("img", 600, 600) 
# 	# cv2.imshow("img", image)

# rospy.init_node('follower')
# follower = Follower()
# rospy.spin()

 


 
