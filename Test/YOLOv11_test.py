import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO

model_path = 'yolo11m-seg.pt'

# 加载YOLOv11s模型
model = YOLO(model_path)

# 创建RealSense管道
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # 配置相机分辨率和帧率
pipeline.start(config)

while True:
    # 捕获一帧图像
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue

    # 获取图像数据并转换为NumPy数组
    frame = np.asanyarray(color_frame.get_data())

    # 创建一个空白图像，用于绘制掩膜
    mask_frame = np.zeros_like(frame)

    # 使用YOLOv11进行目标检测
    results = model(frame, stream=True)  # 传入OpenCV图像（BGR格式），返回结果生成器
    
    # 处理结果生成器
    for result in results:
        boxes = result.boxes  # 检测框
        masks = result.masks  # 分割掩码
        keypoints = result.keypoints 
        probs = result.probs  # 类别概率
        obb = result.obb  # 旋转矩形
        annotated_frame = result.plot()  # 绘制检测结果

        # if masks is not None:
        #     # 返回GPU内tensor掩码
        #     mask_arrays = masks.cuda()
        #     # 将tensor转换为numpy数组
        #     mask_arrays = mask_arrays.cpu()
        #     mask_arrays = mask_arrays.numpy()
                
        #     for mask_array in mask_arrays:
        #         # 将掩膜绘制到空白图像上
        #         mask_frame = mask_array

    # # 将numpy数组转换为Mat格式
    # mask_frame = cv2.cvtColor(mask_frame, cv2.COLOR_GRAY2BGR)
    # # 显示检测结果
    # cv2.imshow('mask_frame', mask_frame)


    cv2.imshow('frame',annotated_frame)
    # 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 清理资源
pipeline.stop()
cv2.destroyAllWindows()