import cv2
import torch

# 加载训练好的 YOLOv5 模型
model_path = '/home/fwb/yolov5/yolov5-master/runs/train/exp7/weights/best.pt'
yolov5_path = '/home/fwb/yolov5/yolov5-master'
model = torch.hub.load(yolov5_path, 'custom', path=model_path, source='local')

# 打开 USB 摄像头
cap = cv2.VideoCapture(0)

# 设置摄像头分辨率，可根据实际情况调整
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

while True:
    # 逐帧读取画面
    ret, frame = cap.read()

    # 检查是否成功读取帧
    if not ret:
        print("无法获取帧，退出...")
        break

    # 使用 YOLOv5 模型进行推理
    results = model(frame)

    # 获取检测结果中的类别信息
    detections = results.pandas().xyxy[0]
    for _, detection in detections.iterrows():
        class_name = detection['name']
        print(f"检测到的类别: {class_name}")

    # 显示当前帧及检测结果
    cv2.imshow('USB Camera Feed', results.render()[0])

    # 按 'q' 键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头资源
cap.release()
# 关闭所有 OpenCV 窗口
cv2.destroyAllWindows()
