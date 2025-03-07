"""
摄像头控制模块

功能：
- 提供摄像头的初始化、内参获取和停止操作
- 支持颜色和深度流的配置和启动

接口说明：
1. Camera 类：主控制类
   - __init__(): 初始化摄像头，配置颜色和深度流，并启动管道
   - get_intrinsics(color_frame): 获取颜色帧的相机内参
   - stop(): 停止摄像头管道

"""

import pyrealsense2 as rs
import numpy as np
from config import COLOR_WIDTH, COLOR_HEIGHT, DEPTH_WIDTH, DEPTH_HEIGHT, FPS

class Camera:
    def __init__(self):
        """
        初始化摄像头，配置颜色和深度流，并启动管道
        """
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, COLOR_WIDTH, COLOR_HEIGHT, rs.format.bgr8, FPS)
        self.config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, FPS)
        self.pipeline.start(self.config)

    def get_intrinsics(self, color_frame):
        """
        获取颜色帧的相机内参
        
        :param color_frame: 颜色帧
        :return: 相机内参
        """
        return color_frame.profile.as_video_stream_profile().intrinsics

    def stop(self):
        """
        停止摄像头管道
        """
        self.pipeline.stop()

# 使用示例
if __name__ == "__main__":
    camera = Camera()
    try:
        # 获取帧数据
        frames = camera.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        intrinsics = camera.get_intrinsics(color_frame)
        print("相机内参:", intrinsics)
    finally:
        camera.stop()