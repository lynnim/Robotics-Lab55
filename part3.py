#!/usr/bin/env python
# BEGIN ALL
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
Take [H-10, 100, 100] and [H-10, 255, 255] as lower and upper bounds, respectively 
"""
class Follower:
    template1 = cv2.imread("left-triangle.jpg")
    template2 = cv2.imread("right-triangle.jpg")
    template3 = cv2.imread("triangle-up.jpg")

    gray_temp1 = cv2.cvtColor(template1, cv2.COLOR_BGR2GRAY)
    gray_temp2 = cv2.cvtColor(template2, cv2.COLOR_BGR2GRAY)
    gray_temp3 = cv2.cvtColor(template3, cv2.COLOR_BGR2GRAY)

    template1_res = cv2.resize(gray_temp1, (0,0), fx=0.3, fy=0.3)
    template2_res = cv2.resize(gray_temp2, (0,0), fx=0.3, fy=0.3)
    template3_res = cv2.resize(gray_temp3, (0,0), fx=0.3, fy=0.3)

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()
        self.STOP = False

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_resize = cv2.resize(gray_image, (0,0), fx=0.5, fy=0.5)

        lower_yellow = numpy.array([19, 100, 100])
        upper_yellow = numpy.array([39, 255, 255])
        y_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        lower_red = numpy.array([-10, 100, 100])
        upper_red = numpy.array([10, 255, 255])
        r_mask = cv2.inRange(hsv, lower_red, upper_red)

        h, w, d = image.shape

        #for the yellow
        search_top = 5 * h / 6  # put the frame 5/6 of the way down
        search_bot = search_top + 20  # use the 20 units in front of the robot from that 5/6 of the way down
        y_mask[0:search_top, 0:w] = 0
        y_mask[search_bot:h, 0:w] = 0

        #for the red
        r_top = 4 * h / 5  # put the frame 5/6 of the way down
        r_bot = r_top + 20  # use the 20 units in front of the robot from that 5/6 of the way down
        r_mask[0:r_top, 0:w] = 0
        r_mask[r_bot:h, 0:w] = 0
        R = cv2.moments(r_mask)

        M = cv2.moments(y_mask)
        if M['m00'] > 0:

            if R['m00'] > 0:
                # calculate the centriod
                r_cx = int(R['m10'] / R['m00'])
                r_cy = int(R['m01'] / R['m00'])
                cv2.circle(image, (r_cx, r_cy), 10, (0, 0, 255), -1)
                
                image_result1 = cv2.matchTemplate(gray_image, gray_temp1, cv2.TM_CCOEFF_NORMED)
                image_result2 = cv2.matchTemplate(gray_image, gray_temp2, cv2.TM_CCOEFF_NORMED)
                image_result3 = cv2.matchTemplate(gray_image, gray_temp3, cv2.TM_CCOEFF_NORMED)

                min_val1, max_val1, min_loc1, max_loc1 = cv2.minMaxLoc(image_result1)
                min_val2, max_val2, min_loc2, max_loc2 = cv2.minMaxLoc(image_result2)
                min_val3, max_val3, min_loc3, max_loc3 = cv2.minMaxLoc(image_result3)

                print("left arrow: " + str(min_val1))
                print("right arrow: " + str(min_val2))
                print("star: " + str(min_val3))

                # if min_val1 > min_val2 and min_val1 >= -0.16:
                #     print("turning left")
                #     self.twist.linear.x = 0.2
                #     self.twist.angular.z = -0.06
                #     self.cmd_vel_pub.publish(self.twist)
                
                # elif min_val1 < min_val2 and min_val2 >= -0.17:
                #     print("turning right")
                #     self.twist.linear.x = 0.2
                #     self.twist.angular.z = .05
                #     self.cmd_vel_pub.publish(self.twist)
                # else:
                #     print("not turning") 

        # calculate the centriod
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)

        # BEGIN CONTROL
        err = cx - w / 2
        self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 100
        #print("twisting:")
        #print(str(-float(err) / 100))
        self.cmd_vel_pub.publish(self.twist)
        # END CONTROL
        print(M['m00'])
    cv2.imshow("window", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL

