"""
UR5机械臂控制模块

功能：
- 提供UR5机械臂的TCP/IP通信控制
- 支持点位运动控制、脚本发送等基础功能
- 包含安全连接管理和异常处理

接口说明：
1. UR5Controller 类：主控制类
   - connect(): 建立连接
   - disconnect(): 断开连接
   - move_to(position, speed): 移动到指定位置
   - send_script(script): 发送URScript脚本
   - reset(): 机械臂复位（需预先定义复位位姿）

"""

import socket
import time
from typing import List, Tuple
from config import UR5_IP, UR5_PORT, UR5_SPEED, UR5_ACCEL, UR5_FAULT_POS
class UR5ConnectionError(Exception):
    """UR5连接错误异常类"""
    pass

class UR5CommandError(Exception):
    """UR5指令发送错误异常类"""
    pass

class UR5Controller:
    def __init__(self, ip = UR5_IP, port = UR5_PORT, speed = UR5_SPEED, accel = UR5_ACCEL):
        """
        初始化UR5控制器
    
        """
        self.ip = ip
        self.port = port
        self._socket = None
        self._default_speed = speed 
        self._default_accel = accel
        
        # 预定义位姿（单位：米/弧度）
        self._preset_poses = {
            'home': UR5_FAULT_POS,
            'handeye': [-0.1, -0.6, 0.3, 0.0, 2.0, -2.0]
        }

    def connect(self, timeout: float = 5.0) -> None:
        """
        建立与UR5的TCP连接
        
        :param timeout: 连接超时时间（秒）
        :raises UR5ConnectionError: 连接失败时抛出
        """
        if self.is_connected:
            return

        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.settimeout(timeout)
            self._socket.connect((self.ip, self.port))
            print(f"成功连接到UR5@{self.ip}:{self.port}")
        except (socket.timeout, ConnectionRefusedError) as e:
            raise UR5ConnectionError(f"连接失败: {str(e)}") from e
        finally:
            self._socket.settimeout(None)  # 重置超时设置

    @property
    def is_connected(self) -> bool:
        """检查当前是否保持连接"""
        return self._socket is not None and not self._socket._closed

    def disconnect(self) -> None:
        """安全关闭连接"""
        if self.is_connected:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
                self._socket.close()
                print("已安全断开UR5连接")
            except OSError as e:
                raise UR5ConnectionError(f"断开连接时出错: {str(e)}") from e
            finally:
                self._socket = None

    def send_script(self, script: str, wait_time: float = 0) -> None:
        """
        发送URScript脚本到机械臂
        
        :param script: URScript程序字符串
        :param wait_time: 发送后的等待时间（秒）
        :raises UR5CommandError: 发送失败时抛出
        """
        if not self.is_connected:
            raise UR5ConnectionError("未建立连接")

        try:
            # 添加程序头尾标识
            formatted_script = f"def execute():\n{script}\nend\n"
            sent = self._socket.send(formatted_script.encode('utf-8'))
            
            if sent != len(formatted_script):
                raise UR5CommandError("指令未完整发送")
                
            if wait_time > 0:
                time.sleep(wait_time)
        except socket.error as e:
            raise UR5CommandError(f"指令发送失败: {str(e)}") from e

    def move_to(self, position: List[float], 
               speed: float = None, 
               accel: float = None) -> None:
        """
        移动到指定笛卡尔位姿
        
        :param position: 目标位姿 [x, y, z, rx, ry, rz]（单位：米/弧度）
        :param speed: 运动速度（m/s），默认使用初始化速度
        :param accel: 加速度（m/s²），默认使用初始化加速度
        """
        if not self.is_connected:
            raise UR5ConnectionError("未建立连接")

        speed = speed or self._default_speed
        accel = accel or self._default_accel
        
        script = f"""
movej(p{position}, a={accel}, v={speed})
"""
        self.send_script(script, wait_time=0.1)

    def reset(self, pose_name: str = 'home', wait: bool = True) -> None:
        """
        复位到预定义位姿
        
        :param pose_name: 预定义位姿名称（home/handeye）
        :param wait: 是否等待复位完成
        :raises ValueError: 无效位姿名称
        """
        if pose_name not in self._preset_poses:
            raise ValueError(f"无效预定义位姿: {pose_name}。可选: {list(self._preset_poses.keys())}")
            
        target = self._preset_poses[pose_name]
        self.move_to(target)
        
        if wait:
            time.sleep(5)  # 根据实际运动时间调整
