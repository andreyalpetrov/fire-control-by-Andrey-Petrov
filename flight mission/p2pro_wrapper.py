import cv2
import numpy as np

class P2Pro:
    def __init__(self, cam_id):
        self.cap = cv2.VideoCapture(cam_id)
        # критично: не конвертировать в RGB
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        # иногда полезно явно задать режим 256x384
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 256)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 384)

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        # на Linux reshape делаем прямо по всему массиву
        frame = np.reshape(frame, (2, 192, 256, 2))  # см. оригинальный код автора p2pro.py[web:60]
        return frame

    def raw16(self):
        frame = self.get_frame()
        if frame is None:
            return None
        raw = frame[1, :, :, :].astype(np.intc)          # нижняя половина[web:60]
        raw = (raw[:, :, 1] << 8) + raw[:, :, 0]         # собираем 16‑бит[web:60]
        return raw

    def temperature(self):
        raw = self.raw16()
        if raw is None:
            return None
        temp_c = raw / 64 - 273.2                        # формула автора[web:60]
        return temp_c

    def __del__(self):
        self.cap.release()
