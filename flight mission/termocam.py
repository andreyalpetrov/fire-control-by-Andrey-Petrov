import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

rospy.init_node('termocam_publisher')
pub = rospy.Publisher('termocam', Image, queue_size=10)
bridge = CvBridge()
cap = cv2.VideoCapture(1)

rate = rospy.Rate(25)
while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        rospy.logwarn("Нет кадра с камеры")
        continue

    image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    pub.publish(image_msg)
    rate.sleep()

cap.release()
