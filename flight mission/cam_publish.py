import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node("usb_cam_publisher")
pub = rospy.Publisher("firecam", Image, queue_size=10)
bridge = CvBridge()

cap = cv2.VideoCapture("/dev/video1")
rate = rospy.Rate(30)

while not rospy.is_shutdown():
    _, frame = cap.read()
    msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    pub.publish(msg)
    rate.sleep()

cap.release()
