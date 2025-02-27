# camera.py 定义了一些与摄像头相关的操作

import pyrealsense2 as rs
import numpy as np
from config import COLOR_WIDTH, COLOR_HEIGHT, DEPTH_WIDTH, DEPTH_HEIGHT, FPS

def initialize_camera():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, COLOR_WIDTH, COLOR_HEIGHT, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, FPS)
    pipeline.start(config)
    return pipeline

def get_camera_intrinsics(color_frame):
    return color_frame.profile.as_video_stream_profile().intrinsics

def stop_camera(pipeline):
    pipeline.stop()
