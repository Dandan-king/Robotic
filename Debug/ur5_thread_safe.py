import threading
from rtde_control import RTDEControlInterface
import time

UR_IP = "192.168.11.125"  # 替换为实际IP

# 共享控制接口实例
shared_control = RTDEControlInterface(UR_IP)

def move_task():
    try:
        for i in range(5):
            print(f"{threading.current_thread().name} 发送移动指令 {i}")
            # 尝试发送不同方向的移动指令
            target = [0.3 * (-1)**i, -0.3, 0.2, 0, 3.14, 0]
            shared_control.moveL(target, 0.1, 0.2)
            time.sleep(0.5)
    except Exception as e:
        print(f"{threading.current_thread().name} 发生错误: {str(e)}")

if __name__ == "__main__":
    # 创建并启动多个线程
    threads = []
    for i in range(3):
        t = threading.Thread(target=move_task, name=f"Thread-{i+1}")
        threads.append(t)
        t.start()

    # 等待所有线程完成
    for t in threads:
        t.join()

    shared_control.disconnect()