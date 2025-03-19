"""
UR5机械臂的RTDE控制依赖于官方定义的接口
依赖库：ur_rtde (安装命令: pip install ur-rtde)
示例：
import rtde_control
import rtde_receive

rtde_c = rtde_control.RTDEControlInterface(UR5_IP)
rtde_r = rtde_receive.RTDEReceiveInterface(UR5_IP)


rtde_c.moveJ(UR5_FAULT_JOINT)

actual_q = rtde_r.getActualQ()
print(f"机械臂当前关节角度：{actual_q}")
actual_tcp = rtde_r.getActualTCPPose()
print(f"机械臂当前位置：{actual_tcp}")


rtde_c.stopScript()

rtde_c.disconnect()
rtde_r.disconnect()
"""
