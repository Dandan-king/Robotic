# model.py 定义了一些与视觉物体识别模型相关的操作

from ultralytics import YOLO

def initialize_model(model_path):
    model = YOLO(model_path)
    return model