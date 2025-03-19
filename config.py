# config.py 保存了一些参数

# YOLO模型路径
MODEL_PATH = 'yolo11m-seg.pt'

# UR5 机械臂 IP 和端口
UR5_IP = '192.168.11.125'
UR5_SPEED = 0.25
UR5_ACCEL = 1.2
UR5_FAULT_JOINT = [-1.5, -1.2, -2.0, 0.0, 1.5, 1.5]

# 机械手 IP 和端口
HAND_IP = '192.168.11.210'
HAND_PORT = 6000
HAND_SPEED = 200
HAND_FORCE = 500

# RealSense 相机设置
COLOR_WIDTH = 640
COLOR_HEIGHT = 480
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 480
FPS = 30

# 摄像头坐标系到机械臂坐标系的转换矩阵


# [[ 0.99707235 -0.01555624  0.07486475 -0.09904975]
#  [-0.06571732 -0.6748597   0.73501403 -1.09044378]
#  [ 0.03908915 -0.73778208 -0.67390626  0.5792831 ]
#  [ 0.          0.          0.          1.        ]]


TRANSFORMATION_MATRIX = [
    [0.99707235, -0.01555624, 0.07486475, -0.09904975],
    [-0.06571732, -0.6748597, 0.73501403, -1.09044378],
    [0.03908915, -0.73778208, -0.67390626, 0.5792831],
    [0, 0, 0, 1]
]