"""Webcam frames and MediaPipe hand landmarks (one hand, VIDEO mode)."""
import time

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks.python import BaseOptions, vision

from .features import HandSample


class HandTracker:
    def __init__(self, model, camera=0, width=640, height=480):
        self.capture = cv2.VideoCapture(camera)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        options = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(model)), running_mode=vision.RunningMode.VIDEO,
            num_hands=1, min_hand_detection_confidence=0.6, min_hand_presence_confidence=0.6,
            min_tracking_confidence=0.6)
        self.landmarker = vision.HandLandmarker.create_from_options(options)
        self.start = time.monotonic()

    def read(self):
        """Raw BGR frame and the hand in it, or None."""
        ok, frame = self.capture.read()
        if not ok:
            raise RuntimeError('웹캠 프레임을 읽지 못했습니다 (macOS 카메라 권한, --camera 번호 확인)')
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        result = self.landmarker.detect_for_video(image, int((time.monotonic() - self.start) * 1000))
        if not result.hand_landmarks:
            return frame, None
        height, width = frame.shape[:2]
        pixels = np.array([[p.x * width, p.y * height] for p in result.hand_landmarks[0]])
        world = np.array([[p.x, p.y, p.z] for p in result.hand_world_landmarks[0]])
        return frame, HandSample.from_landmarks(pixels, world)

    def close(self):
        self.landmarker.close()
        self.capture.release()
