# 用于调试相机

import cv2
import pyrealsense2 as rs
import numpy as np
import socket
from ultralytics import YOLO
import time
import threading
from pymodbus.client import ModbusTcpClient
import queue



pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

while True:
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue

    annotated_frame = np.asanyarray(color_frame.get_data())
    cv2.imshow('frame', annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break