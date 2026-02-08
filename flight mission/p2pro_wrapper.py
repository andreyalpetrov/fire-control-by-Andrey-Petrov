import cv2
import numpy as np

class P2Pro:
    def __init__(self, cam_id):
        self.cap = cv2.VideoCapture(cam_id)
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 256)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 384)

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        frame = np.reshape(frame, (2, 192, 256, 2))
        return frame

    def raw16(self):
        frame = self.get_frame()
        if frame is None:
            return None
        raw = frame[1, :, :, :].astype(np.intc)
        raw = (raw[:, :, 1] << 8) + raw[:, :, 0]
        return raw

    def temperature(self):
        raw = self.raw16()
        if raw is None:
            return None
        temp_c = raw / 64 - 273.2
        return temp_c

    def __del__(self):
        self.cap.release()
