"""
增强版摄像头控制模块

功能：
- 提供摄像头的初始化、内参获取和停止操作
- 支持颜色和深度流的配置和启动
- 增加自动曝光控制
- 添加帧同步功能
- 提供深度-颜色对齐功能
- 增加异常处理和状态检查

接口说明：
1. Camera 类：主控制类
   - __init__(): 初始化摄像头，配置颜色和深度流，并启动管道
   - get_intrinsics(): 获取颜色帧的相机内参
   - get_frames(): 获取同步的颜色和深度帧
   - align_frames(): 获取对齐的深度-颜色帧
   - stop(): 停止摄像头管道
   - is_running(): 检查摄像头是否正在运行
"""

import pyrealsense2 as rs
import numpy as np
from config import COLOR_WIDTH, COLOR_HEIGHT, DEPTH_WIDTH, DEPTH_HEIGHT, FPS

class Camera:
    def __init__(self):
        """
        初始化摄像头，配置颜色和深度流，并启动管道
        """
        self.pipeline = None
        self.align = None
        self.running = False
        
        try:
            # 创建管道和配置
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # 配置流参数
            self.config.enable_stream(rs.stream.color, 
                                    COLOR_WIDTH, 
                                    COLOR_HEIGHT, 
                                    rs.format.bgr8, 
                                    FPS)
            self.config.enable_stream(rs.stream.depth, 
                                    DEPTH_WIDTH, 
                                    DEPTH_HEIGHT, 
                                    rs.format.z16, 
                                    FPS)
            
            # 创建对齐对象（深度到颜色）
            self.align = rs.align(rs.stream.color)
            
            # 启动管道
            pipeline_profile = self.pipeline.start(self.config)
            
            # 获取设备并设置自动曝光
            device = pipeline_profile.get_device()
            color_sensor = device.first_color_sensor()
            if color_sensor.supports(rs.option.enable_auto_exposure):
                color_sensor.set_option(rs.option.enable_auto_exposure, 1)
            
            self.running = True
            print("摄像头初始化成功")
            
        except Exception as e:
            print(f"摄像头初始化失败: {str(e)}")
            self.stop()
            raise RuntimeError("无法初始化摄像头")

    def get_intrinsics(self):
        """
        获取颜色帧的相机内参
        
        :return: 相机内参对象
        """
        if not self.running:
            raise RuntimeError("摄像头未运行")
            
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                raise RuntimeError("无法获取颜色帧")
                
            return color_frame.profile.as_video_stream_profile().intrinsics
            
        except Exception as e:
            print(f"获取内参失败: {str(e)}")
            return None

    def get_frames(self, timeout=5000):
        """
        获取同步的颜色和深度帧
        
        :param timeout: 等待帧的超时时间（毫秒）
        :return: (color_frame, depth_frame) 元组
        """
        if not self.running:
            raise RuntimeError("摄像头未运行")
            
        try:
            # 等待一组同步的帧
            frames = self.pipeline.wait_for_frames(timeout_ms=timeout)
            
            # 获取颜色和深度帧
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                raise RuntimeError("无法获取同步帧")
                
            return color_frame, depth_frame
            
        except Exception as e:
            print(f"获取帧失败: {str(e)}")
            return None, None

    def align_frames(self, timeout=5000):
        """
        获取对齐的深度-颜色帧
        
        :param timeout: 等待帧的超时时间（毫秒）
        :return: (aligned_color_frame, aligned_depth_frame) 元组
        """
        if not self.running:
            raise RuntimeError("摄像头未运行")
            
        try:
            # 等待一组同步的帧
            frames = self.pipeline.wait_for_frames(timeout_ms=timeout)
            
            # 对齐深度到颜色帧
            aligned_frames = self.align.process(frames)
            
            # 获取对齐后的帧
            aligned_color_frame = aligned_frames.get_color_frame()
            aligned_depth_frame = aligned_frames.get_depth_frame()
            
            if not aligned_color_frame or not aligned_depth_frame:
                raise RuntimeError("无法获取对齐帧")
                
            return aligned_color_frame, aligned_depth_frame
            
        except Exception as e:
            print(f"获取对齐帧失败: {str(e)}")
            return None, None

    def is_running(self):
        """
        检查摄像头是否正在运行
        
        :return: 运行状态 (True/False)
        """
        return self.running

    def stop(self):
        """
        停止摄像头管道
        """
        if self.pipeline:
            try:
                self.pipeline.stop()
                self.running = False
                print("摄像头已停止")
            except Exception as e:
                print(f"停止摄像头失败: {str(e)}")
        else:
            self.running = False

# 使用示例
if __name__ == "__main__":
    try:
        camera = Camera()
        
        if camera.is_running():
            # 获取内参
            intrinsics = camera.get_intrinsics()
            if intrinsics:
                print("相机内参:")
                print(f"  焦距: ({intrinsics.fx}, {intrinsics.fy})")
                print(f"  主点: ({intrinsics.ppx}, {intrinsics.ppy})")
            
            # 获取对齐帧
            color_frame, depth_frame = camera.align_frames()
            if color_frame and depth_frame:
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
                print(f"获取帧成功: 颜色帧大小={color_image.shape}, 深度帧大小={depth_image.shape}")
                
    except Exception as e:
        print(f"测试失败: {str(e)}")
    finally:
        if 'camera' in locals():
            camera.stop()