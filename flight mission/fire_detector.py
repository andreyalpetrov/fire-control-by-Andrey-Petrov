import rospy
import requests
import os
import tempfile
import numpy as np
import cv2
from sensor_msgs.msg import Image, Temperature
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge, CvBridgeError


class FireAlertNode:
    def __init__(self):
        """Инициализация ROS ноды и подписок"""
        rospy.init_node('fire_alert_node', anonymous=True)
        
        # Параметры (адаптируйте под вашу систему)
        self.temp_meas_topic = '/termocam_temp'      # Топик температуры
        self.temp_image_topic = '/termocam_image'    # Тепловизор
        self.cam_image_topic = '/main_camera/image_raw'  # Нижняя камера
        self.alert_url = 'http://androproject:5000/api/alerts'

        self.max_temp_threshold = 100.0
        self.current_max_temp = 0.0
        self.bridge = CvBridge()
        
        self.latest_cam_image = None
        self.latest_temp_image = None
        
        self.alert_sent = False  # Флаг для избежания спама
        
        # Подписки на топики
        rospy.Subscriber(self.temp_meas_topic, Temperature, self.temp_meas_callback)
        rospy.Subscriber(self.temp_image_topic, Image, self.temp_image_callback)
        rospy.Subscriber(self.cam_image_topic, Image, self.cam_image_callback)
        
        rospy.loginfo(f"Подписка на {self.temp_meas_topic} (темп.), "
                      f"{self.temp_image_topic} и {self.cam_image_topic} (изобр.). "
                      f"Порог: {self.max_temp_threshold}°C")
        
        # ГЛАВНОЕ: rospy.spin() держит ноду активной и обрабатывает входящие сообщения
        rospy.spin()

    def temp_meas_callback(self, msg):
        """Колбэк для обработки сообщений температуры"""
        self.current_max_temp = msg.data
        rospy.loginfo(f"Макс. температура: {self.current_max_temp:.1f}°C")
