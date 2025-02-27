# 测试物体的坐标变换是否正确的

import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO

model_path = 'yolo11s.pt'

# 加载YOLOv11模型
model = YOLO(model_path)

# 创建RealSense管道
pipeline = rs.pipeline()
config = rs.config()

# 配置并启用RGB和深度流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 启动管道
pipeline.start(config)

# 获取相机内参
while True:
    # 捕获一帧图像
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        continue

    # 获取图像数据并转换为NumPy数组
    frame = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    # 使用YOLOv11进行目标检测
    results = model(frame, stream=True)  # 传入OpenCV图像（BGR格式），返回结果生成器
    
    # 获取相机内参
    intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
    
    for result in results:
        boxes = result.boxes  # 获取检测框
        annotated_frame = result.plot()  # 绘制检测结果
        names = result.names  # 获取物体名称字典

        # 打印boxes的信息
        for i, box in enumerate(boxes):
            # 获取每个框的四个角坐标（xyxy）
            xyxy = box.xyxy[0].cpu().numpy()  # 转换为NumPy数组
            conf = box.conf[0].cpu().numpy()  # 获取置信度
            class_id = box.cls[0].cpu().numpy()  # 获取类别ID
            name = names[int(class_id)]  # 获取物体名称
            
            # 只有当物体的名称是bottle时才输出坐标
            if name == 'bottle':
                # 输出每个物体的检测框信息
                print(f"物体 {i + 1}: 边界框坐标 (x1, y1, x2, y2) = {xyxy}, 置信度 = {conf}")
                
                # 计算物体中心点坐标
                center_x = (xyxy[0] + xyxy[2]) / 2
                center_y = (xyxy[1] + xyxy[3]) / 2
                print(f"物体 {i + 1}: 物体中心点坐标 = ({center_x}, {center_y})")
    
                # 通过深度图获取物体中心的深度
                depth = depth_frame.get_distance(int(center_x), int(center_y))  # 获取该点的深度值
    
                # 将像素坐标（center_x, center_y）转化为相机坐标系中的三维坐标
                x_cam = (center_x - intrinsics.ppx) * depth / intrinsics.fx
                y_cam = (center_y - intrinsics.ppy) * depth / intrinsics.fy
                z_cam = depth
    
                # 得到相机坐标系下的物体中心位置
                camera_position = np.array([x_cam, y_cam, z_cam, 1.0])
    
                # 使用转移矩阵T将物体中心从相机坐标系转换到目标坐标系
                T = np.array([
                    [ 0.34281861,  0.42765676, -0.83641204,  0.71507414],
                    [ 0.93912141, -0.13427398,  0.31626173, -0.78430701],
                    [ 0.02294309, -0.89391286, -0.44765323,  0.38923032],
                    [ 0.0,         0.0,         0.0,         1.0       ]
                ])

                
                transformed_position = np.dot(T, camera_position)
                transformed_position = transformed_position[:3]  # 只取前三个坐标（x, y, z）

                print(f"物体 {i + 1}: 物体在目标坐标系下的中心位置 = {transformed_position}")

    # 显示检测结果
    cv2.imshow('frame', annotated_frame)
    
    # 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 停止管道并清理资源
pipeline.stop()
cv2.destroyAllWindows()




# 物体 1: 边界框坐标 (x1, y1, x2, y2) = [     336.11      28.476      416.67      307.49], 置信度 = 0.9373940229415894
# 物体 1: 物体中心点坐标 = (376.3913269042969, 167.98110961914062)
# 物体 1: 物体在目标坐标系下的中心位置 = [    -1.7271     0.39928    -0.53564]

# 物体 1: 边界框坐标 (x1, y1, x2, y2) = [      300.1      25.945      383.12      310.03], 置信度 = 0.9358943104743958
# 物体 1: 物体中心点坐标 = (341.61114501953125, 167.98629760742188)
# 物体 1: 物体在目标坐标系下的中心位置 = [    0.27477    -0.60264     0.22555]





# 目标物体位置在相机坐标系: [   0.012827   -0.055993     0.42569]
# 目标物体位置在机械臂坐标系: [    0.33947    -0.63011     0.24901]


