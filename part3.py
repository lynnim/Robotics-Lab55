#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


"""
Yellow
[[[ 29 191 171]]]
Blue
[[[120 255 169]]]
Red
[[[  0 229 164]]]
Green
[[[ 60 193 169]]]
Take [H-10, 100, 100] and [H+10, 255, 255] as lower and upper bounds, respectively 
"""

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_yellow = numpy.array([19, 100, 100])
    upper_yellow = numpy.array([39, 255, 255])
    y_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    lower_green = numpy.array([50, 100, 100])
    upper_green = numpy.array([70, 255, 255])
    g_mask = cv2.inRange(hsv, lower_green, upper_green)

    lower_blue = numpy.array([110, 100, 100])
    upper_blue = numpy.array([130, 255, 255])
    b_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    lower_red = numpy.array([-10, 100, 100])
    upper_red = numpy.array([10, 255, 255])
    r_mask = cv2.inRange(hsv, lower_red, upper_red)

    h, w, d = image.shape

    #for the yellow
    search_top = 5 * h / 6  # put the frame 5/6 of the way down
    search_bot = search_top + 20  # use the 20 units in front of the robot from that 5/6 of the way down
    y_mask[0:search_top, 0:w] = 0
    y_mask[search_bot:h, 0:w] = 0

    #for the green
    g_top = 4 * h / 5  
    g_bot = g_top + 20  # use the 20 units in front of the robot from that 5/6 of the way down
    g_mask[0:g_top, 0:w] = 0
    g_mask[g_bot:h, 0:w] = 0
    G = cv2.moments(g_mask)

    #for the blue
    b_top = 4 * h / 5  
    b_bot = b_top + 20  # use the 20 units in front of the robot from that 5/6 of the way down
    b_mask[0:b_top, 0:w] = 0
    b_mask[b_bot:h, 0:w] = 0
    B = cv2.moments(b_mask)

    #for the red
    r_top = 4 * h / 5  
    r_bot = r_top + 20  # use the 20 units in front of the robot from that 5/6 of the way down
    r_mask[0:r_top, 0:w] = 0
    r_mask[r_bot:h, 0:w] = 0
    R = cv2.moments(r_mask)

    M = cv2.moments(y_mask)
    if M['m00'] > 0:

        if G['m00'] > 0:
            # calculate the centriod
            g_cx = int(G['m10'] / G['m00'])
            g_cy = int(G['m01'] / G['m00'])
            cv2.circle(image, (g_cx, g_cy), 10, (0, 225, 0), -1)
            print("green = left")
            self.twist.linear.x = .2
            self.twist.angular.z = .05
            self.cmd_vel_pub.publish(self.twist)

        elif B['m00'] > 0:
            # calculate the centriod
            b_cx = int(B['m10'] / B['m00'])
            b_cy = int(B['m01'] / B['m00'])
            cv2.circle(image, (b_cx, b_cy), 10, (225, 0, 0), -1)
            print("blue = right")
            self.twist.linear.x = .2
            self.twist.angular.z = -.06
            self.cmd_vel_pub.publish(self.twist)

        elif R['m00'] > 0:
            # calculate the centriod
            r_cx = int(R['m10'] / R['m00'])
            r_cy = int(R['m01'] / R['m00'])
            cv2.circle(image, (r_cx, r_cy), 10, (0, 0, 225), -1)
            print("red = stop")
            self.twist.linear.x = 0.3
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

        else:
            # calculate the centriod
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)

            err = cx - w / 2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            #print("twisting:")
            #print(str(-float(err) / 100))
            self.cmd_vel_pub.publish(self.twist)
    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
