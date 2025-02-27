# utils.py 定义了公用的辅助函数，在这里是从相机坐标系到机械臂坐标系的转移函数

import numpy as np
from config import TRANSFORMATION_MATRIX

def transform_to_arm_coordinates(camera_position):
    T = np.array(TRANSFORMATION_MATRIX)

    camera_position_homogeneous = np.append(camera_position, 1)  # 转换为齐次坐标
    transformed_position = np.dot(T, camera_position_homogeneous)

    return transformed_position[:3]  # 返回转换后的位置（x, y, z）