#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Follower:
	def __init__(self):
 		self.bridge = cv_bridge.CvBridge()
 		# cv2.namedWindow("window", 1)

 		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback)
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
		self.twist = Twist()
		self.M = None
		self.current_red = False
		self.old_red = False
		self.current_blue = False
		self.old_blue = False
		self.current_green = False
		self.old_green = False
		self.state = "FOLLOW"

	# def is_red(self, hsv, center):
	# 	x,y = center
	# 	lower_color = numpy.array([16,17,163])
	# 	upper_color = numpy.array([18,19,165])
	# 	mask = cv2.inRange(hsv, lower_color, upper_color)
	# 	cv2.imshow("window", mask)
	# 	return mask[int(y), int(x)] == 255

	def callback(self, msg):
		print(msg.pose.pose)

	def is_color(self, image, center, color):
		cx, cy = center
		return (image[int(cx), int(cy), 2],image[int(cx), int(cy), 1], image[int(cx), int(cy), 0]) == color
		
	def is_red(self, image, center, color = (164, 18, 17)):
		cx, cy = center
		#cy -= 40
		#print(image[int(cx), int(cy), 2],image[int(cx), int(cy), 1], image[int(cx), int(cy), 0])
		return self.is_color(image, (cx, cy), color)

	def is_blue(self, image, center, color = (0, 0, 169)):
		cx, cy = center
		#cy -= 40
		return self.is_color(image, (cx, cy), color)

	def is_green(self, image, center, color = (170,168,42)):
		cx, cy = center
		#cy -= 140
		return self.is_color(image, (cx, cy), color)

	def turn_left(self):
		self.twist.linear.x = 0.2
		self.twist.angular.z = 5 / 100
		self.cmd_vel_pub.publish(self.twist)

	def stop(self):
		self.twist.linear.x = 0
		self.twist.angular.z = 0
		self.cmd_vel_pub.publish(self.twist)

	def image_callback(self, msg):
		if self.state == "FOLLOW":
			image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
			hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
			lower_yellow = numpy.array([10,10,10]) # 30 30 29
			upper_yellow = numpy.array([255, 255, 250])
			mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

			# cv2.imshow("window", mask)
			h, w, d = image.shape
			search_top = 3*h/4
			search_bot = 3*h/4 + 20
			mask[0:search_top, 0:w] = 0
			mask[search_bot:h, 0:w] = 0
			M = cv2.moments(mask)
			self.M = M

			if M['m00'] > 0:
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])
				cv2.circle(image, (cx, cy), 4, (0,0,255), -1)
				err = cx - w/2
				self.current_red = self.is_red(image, (cx,cy))
				self.current_blue = self.is_blue(image, (cx,cy))
				self.current_green = self.is_green(image, (cx,cy))

				if self.current_red:
					print("RED")
				elif self.current_blue:
					print("BLUE")
				elif self.current_green:
					print("GREEN")
				if self.old_red and not self.current_red:
					self.state = "STOP"
				elif self.old_blue and not self.current_blue:
					self.state = "RIGHT"
				elif self.old_green and not self.current_green:
					self.state = "LEFT"
				else:
					self.state = "FOLLOW"
					print("FOLLOW")

				self.twist.linear.x = 0.2
				self.twist.angular.z = -float(err) / 100
				self.cmd_vel_pub.publish(self.twist)

				self.old_red = self.current_red
				self.old_green = self.current_green
				self.old_blue = self.current_blue
			
			cv2.imshow("window", image)
			cv2.waitKey(3)
		
		elif self.state == "LEFT":
			print("leaving GREEN")
			self.turn_left()
			self.state = "FOLLOW"
		elif self.state == "RIGHT":
			print("leaving BLUE")
			self.state = "FOLLOW"
		elif self.state == "STOP":
			print("leaving RED")
			self.stop()

if __name__ == "__main__":
	rospy.init_node('follower')
	follower = Follower()
	rospy.spin()
	# while (not rospy.is_shutdown()):
		# print(follower.M)

# {'mu02': 21328724.24581909, 'mu03': -2263635.88671875, 'm11': 77474830830.0, 'nu02': 5.223028374669191e-05, 'm12': 28623657702240.0, 'mu21': 75136156.71569824, 'mu20': 931719212.8731079, 'nu20': 0.002281616016961075, 'm30': 23514535222590.0, 'nu21': 2.3016848492542736e-07, 'mu11': -44877487.03909302, 'mu12': 16908234.21757126, 'nu11': -0.00010989705032872682, 'nu12': 5.179587062655204e-08, 'm02': 87306512910.0, 'm03': 32282591954610.0, 'm00': 639030.0, 'm01': 236173350.0, 'mu30': -663540141.6640625, 'nu30': -2.032656922710442e-06, 'nu03': -6.934313188792959e-09, 'm10': 209750250.0, 'm20': 69778514160.0, 'm21': 25759428091320.0}
