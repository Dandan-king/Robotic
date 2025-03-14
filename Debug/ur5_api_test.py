"""
UR5机械臂端口测试脚本
依赖库：ur_rtde (安装命令: pip install ur-rtde)
三维中分别是第一维+向左，第二维+向后，第三维+向上
"""
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import UR5_IP, UR5_FAULT_POS, UR5_FAULT_JOINT
import rtde_control
import rtde_receive

rtde_c = rtde_control.RTDEControlInterface(UR5_IP)
rtde_r = rtde_receive.RTDEReceiveInterface(UR5_IP)


rtde_c.moveJ(UR5_FAULT_JOINT)

target_pos = rtde_r.getTargetTCPPose()
current_pos = rtde_r.getActualTCPPose()

print(f"机械臂当前位置：{current_pos}")
print(f"机械臂目标位置：{target_pos}")

# pos1 = [current_pos[0], current_pos[1]+0.1, current_pos[2], current_pos[3], current_pos[4], current_pos[5]]

# rtde_c.moveL(pos1)

# 检测是否完成移动
while True:
    if rtde_c.isSteady():
        print("机械臂移动完成")
        break
    else:
        print("机械臂移动中...")
        continue

actual_q = rtde_r.getActualQ()
print(f"机械臂当前关节角度：{actual_q}")
actual_tcp = rtde_r.getActualTCPPose()
print(f"机械臂当前位置：{actual_tcp}")

speed = [0, 0, 0.100, 0, 0, 0]
rtde_c.moveUntilContact(speed)

rtde_c.stopScript()

rtde_c.disconnect()
rtde_r.disconnect()

