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
        self.alert_url = 'http://androprojects.ru:5000/api/alerts'

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
        self.current_max_temp = msg.temperature
        rospy.loginfo(f"Макс. температура: {self.current_max_temp:.1f}°C")
        
        # Проверка на пожар
        if self.current_max_temp > self.max_temp_threshold and \
           self.latest_temp_image is not None and \
           not self.alert_sent:
            self.send_alert()
            self.alert_sent = True

    def temp_image_callback(self, msg):
        """Колбэк для получения изображения с тепловизора"""
        try:
            self.latest_temp_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.logdebug("Получено изображение с тепловизора")
        except CvBridgeError as e:
            rospy.logerr(f"Ошибка cv_bridge (тепловизор): {e}")

    def cam_image_callback(self, msg):
        """Колбэк для получения изображения с обычной камеры"""
        try:
            self.latest_cam_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.logdebug("Получено изображение с фронтальной камеры")
        except CvBridgeError as e:
            rospy.logerr(f"Ошибка cv_bridge (камера): {e}")
            
    def send_alert(self):
        """Отправка алерта на веб-сервер с изображениями"""
        if self.latest_temp_image is None:
            rospy.logwarn("Нет изображения тепловизора для отправки")
            return
        
        rospy.loginfo("Отправка алерта о пожаре...")
        
        # Сохраняем изображения временно
        with tempfile.TemporaryDirectory() as tmpdir:
            photo_temp_path = os.path.join(tmpdir, 'photo_thermal.png')
            photo_cam_path = os.path.join(tmpdir, 'photo_camera.png')
            
            cv2.imwrite(photo_temp_path, self.latest_temp_image)
            if self.latest_cam_image is not None:
                cv2.imwrite(photo_cam_path, self.latest_cam_image)
            
            # Правильная подготовка файлов для отправки
            try:
                with open(photo_temp_path, 'rb') as f_temp:
                    files_to_send = [
                        ('images', ('thermal.png', f_temp, 'image/png'))
                    ]
                    
                    # Если есть изображение с камеры, добавляем его
                    if self.latest_cam_image is not None:
                        with open(photo_cam_path, 'rb') as f_cam:
                            files_to_send.append(
                                ('images', ('camera.png', f_cam, 'image/png'))
                            )
                            
                            data = {
                                'fire_detected': 'true',
                                'comment': f'Обнаружен огонь (темп. {self.current_max_temp:.1f}°C)'
                            }
                            
                            response = requests.post(
                                self.alert_url,
                                files=files_to_send,
                                data=data,
                                timeout=10
                            )
                    else:
                        data = {
                            'fire_detected': 'true',
                            'comment': f'Обнаружен огонь (темп. {self.current_max_temp:.1f}°C)'
                        }
                        
                        response = requests.post(
                            self.alert_url,
                            files=files_to_send,
                            data=data,
                            timeout=10
                        )
                
                rospy.loginfo(f"Alert отправлен. Код ответа: {response.status_code}")
                if response.status_code == 200:
                    rospy.loginfo("Алерт успешно доставлен на сервер!")
            except Exception as e:
                rospy.logerr(f"Ошибка отправки алерта: {e}")


if __name__ == '__main__':
    try:
        node = FireAlertNode()  # ГЛАВНОЕ: создаём объект и запускаем его
    except rospy.ROSInterruptException:
        rospy.loginfo("Нода завершена пользователем (Ctrl+C)")
