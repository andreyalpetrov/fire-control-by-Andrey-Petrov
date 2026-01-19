import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge

rospy.init_node('termocam_calibrated')
bridge = CvBridge()
pub_temp_map = rospy.Publisher('temperature_map', Float32MultiArray, queue_size=10)
pub_visual = rospy.Publisher('termocam', Image, queue_size=10)

cap = cv2.VideoCapture(1)
rate = rospy.Rate(25)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    temp_celsius = (gray.astype(np.float32) * 420.0 / 255.0) - 20.0

    temp_msg = Float32MultiArray()
    temp_msg.data = temp_celsius.flatten().tolist()

    h, w = temp_celsius.shape
    temp_msg.layout.dim.append(MultiArrayDimension(label='height', size=h, stride=h*w))
    temp_msg.layout.dim.append(MultiArrayDimension(label='width', size=w, stride=w))

    pub_temp_map.publish(temp_msg)

    visual_norm = ((temp_celsius - temp_celsius.min()) * 255 / (temp_celsius.max() - temp_celsius.min() + 1)).astype(np.uint8)
    visual_msg = bridge.cv2_to_imgmsg(visual_norm, "mono8")
    pub_visual.publish(visual_msg)

    rospy.loginfo_throttle(2, f"Temp: {temp_celsius.min():.1f}°C .. {temp_celsius.max():.1f}°C")

    rate.sleep()

cap.release()
