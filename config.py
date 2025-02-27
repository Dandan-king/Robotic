# config.py 保存了一些参数

# YOLO模型路径
MODEL_PATH = 'yolo11m-seg.pt'

# UR5 机械臂 IP 和端口
UR5_IP = '192.168.11.125'
UR5_PORT = 30003

# 机械手 IP 和端口
HAND_IP = '192.168.11.210'
HAND_PORT = 6000

# RealSense 相机设置
COLOR_WIDTH = 640
COLOR_HEIGHT = 480
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
FPS = 30

# 摄像头坐标系到机械臂坐标系的转换矩阵
TRANSFORMATION_MATRIX = [
    [ 0.34281861,  0.42765676, -0.83641204,  0.71507414],
    [ 0.93912141, -0.13427398,  0.31626173, -0.78430701],
    [ 0.02294309, -0.89391286, -0.44765323,  0.38923032],
    [ 0.0,         0.0,         0.0,         1.0       ]
]