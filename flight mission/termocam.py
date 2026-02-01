import rospy
from sensor_msgs.msg import Image, Temperature
from cv_bridge import CvBridge
import numpy as np

from p2pro_wrapper import P2Pro   # файл, где лежит класс выше

rospy.init_node('termocam_publisher')
pub_image = rospy.Publisher('/termocam_image', Image, queue_size=10)
pub_temp  = rospy.Publisher('/termocam_temp', Temperature, queue_size=10)
bridge = CvBridge()

cam_id = '/dev/video1'  # лучше явно путь
cam = P2Pro(cam_id)

rate = rospy.Rate(25)
while not rospy.is_shutdown():
    temp_celsius = cam.temperature()
    if temp_celsius is None:
        rospy.logwarn("Нет кадра с камеры")
        continue

    print(f"min={temp_celsius.min():.1f}°C  max={temp_celsius.max():.1f}°C")

    # нормализация в 0..255 и uint8
    display = (temp_celsius - temp_celsius.min()) / (temp_celsius.max() - temp_celsius.min())
    display_u8 = (display * 255).astype(np.uint8)

    # 1‑канальное изображение
    image_msg = bridge.cv2_to_imgmsg(display_u8, encoding="mono8")
    pub_image.publish(image_msg)
    rate.sleep()

    temp_max_msg = Temperature()
    temp_max_msg.header.stamp = rospy.Time.now()
    temp_max_msg.header.frame_id = "/termocam"
    temp_max_msg.temperature = temp_celsius.max()
    temp_max_msg.variance = 0.0
    pub_temp.publish(temp_max_msg)
