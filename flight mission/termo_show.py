import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

def apply_iron_palette(frame):
    colored = cv2.applyColorMap(frame, cv2.COLORMAP_INFERNO)
    return colored

def callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, "mono8")
    frame_rotated = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    colored_frame = apply_iron_palette(frame_rotated)
    height, width = colored_frame.shape[:2]
    right_half = colored_frame[:, width // 2:]
    out_msg = bridge.cv2_to_imgmsg(right_half, "bgr8")
    pub.publish(out_msg)

rospy.init_node('termocam_iron_red_viewer')
pub = rospy.Publisher("/termo_show", Image, queue_size=10)
sub = rospy.Subscriber("/termocam", Image, callback)
rospy.spin()
