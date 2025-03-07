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

# 修改后的调试程序（保持与主程序一致）
while True:
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue

    # 获取原始BGR数据
    bgr_frame = np.asanyarray(color_frame.get_data())
    
    # 转换为RGB用于显示比对
    rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
    
    # 并排显示两种格式
    combined = np.hstack((bgr_frame, rgb_frame))
    cv2.imshow('BGR vs RGB', combined)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break