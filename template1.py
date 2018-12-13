# Python program to illustrate 
# multiscaling in template matching 
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
    image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    img_rgb = cv2.imread(image) 
    img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY) 

    # template matching
    template = cv2.imread("leftred.png",0) 
    w, h = template.shape[::-1]

    result = cv2.matchTemplate(gray_img, template, cv2.TM_CCOEFF_NORMED)
    loc = numpy.where(result >= 0.4)

    for pt in zip(*loc[::-1]):
    cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 3)

    cv2.imshow("window", image)
    cv2.waitKey(3)
    # cv2.namedWindow("img", cv2.WINDOW_NORMAL)
    # cv2.resizeWindow("img", 600, 600) 
    # cv2.imshow("img", image)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
