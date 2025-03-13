"""
灵巧手控制模块（Inspire Hand）

功能：
- 提供基于Modbus TCP的灵巧手控制接口
- 支持位置、速度、力度控制
- 提供预设抓取动作和状态读取

接口说明：
1. InspireHand 类：主控制类
   - connect(): 建立连接
   - disconnect(): 断开连接
   - set_positions(positions): 设置各关节目标位置
   - set_speed(speeds): 设置运动速度
   - set_force(forces): 设置力度限制
   - grasp(): 执行抓取动作
   - release(): 执行释放动作
   - get_status(): 获取当前状态
   - read_register(address, count): 读取寄存器值
   - read6(reg_name): 读取6个寄存器值

"""

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException
from typing import List, Optional, Dict
from config import HAND_IP, HAND_PORT, HAND_SPEED, HAND_FORCE
import time

# 预定义抓取和松开的位姿

# 抓取位姿
GRASP_POSITIONS = [400, 400, 400, 400, 400, 400]
REALEASE_POSITIONS = [1000, 1000, 1000, 1000, 1000, 400]
INITIAL_POSITIONS = [1000, 1000, 1000, 1000, 1000, 400]

class HandConnectionError(Exception):
    """灵巧手连接相关异常基类"""
    pass

class HandCommandError(Exception):
    """灵巧手指令执行异常"""
    pass

class InspireHand:
    # 保留原有的寄存器字典
    regdict = {
        'ID': 1000,
        'baudrate': 1001,
        'clearErr': 1004,
        'forceClb': 1009,
        'angleSet': 1486,
        'forceSet': 1498,
        'speedSet': 1522,
        'angleAct': 1546,
        'forceAct': 1582,
        'errCode': 1606,
        'statusCode': 1612,
        'temp': 1618,
        'actionSeq': 2320,
        'actionRun': 2322
    }

    # 参数范围限制
    _POS_RANGE = (0, 1000)    # 位置范围 0-1000
    _SPEED_RANGE = (0, 1000)   # 速度范围 0-1000
    _FORCE_RANGE = (0, 1000)  # 力度范围 0-1000

    def __init__(self, ip=HAND_IP, port=HAND_PORT, speed=HAND_SPEED, force=HAND_FORCE):
        """
        初始化灵巧手控制器
        
        :param ip: 灵巧手IP地址
        :param port: 端口号，默认6000
        :param speed: 默认速度
        :param force: 默认力度
        """
        self.ip = ip
        self.port = port
        self._client: Optional[ModbusTcpClient] = None
        self._num_DOF = 6  # 6个自由度
        self._default_speed = speed
        self._default_force = force

    def connect(self, timeout: float = 3.0) -> None:
        """
        建立Modbus TCP连接
        
        :param timeout: 连接超时时间（秒）
        :raises HandConnectionError: 连接失败时抛出
        """
        if self.is_connected:
            return

        try:
            self._client = ModbusTcpClient(
                host=self.ip,
                port=self.port,
                timeout=timeout
            )
            if not self._client.connect():
                raise HandConnectionError("连接被拒绝")
            
            # 验证连接
            if not self._verify_connection():
                raise HandConnectionError("设备响应验证失败")
                
            print(f"成功连接到灵巧手@{self.ip}:{self.port}")
        except ModbusException as e:
            raise HandConnectionError(f"Modbus通信错误: {str(e)}") from e

    def _verify_connection(self) -> bool:
        """验证连接有效性"""
        try:
            return self.read_register(self.regdict['statusCode'], 1) is not None
        except HandCommandError:
            return False

    @property
    def is_connected(self) -> bool:
        """检查当前是否保持连接"""
        return self._client is not None and self._client.connected

    def disconnect(self) -> None:
        """安全关闭连接"""
        if self.is_connected:
            try:
                self._client.close()
                print("已断开灵巧手连接")
            except ModbusException as e:
                raise HandConnectionError(f"断开连接时出错: {str(e)}") from e
            finally:
                self._client = None

    def initialize(self) -> None:
        """初始化灵巧手"""
        self.set_speed([self._default_speed] * self._num_DOF)
        self.set_force([self._default_force] * self._num_DOF)
        self.set_positions(INITIAL_POSITIONS)


    def set_positions(self, positions: List[int]) -> None:
        """
        设置各关节目标位置
        
        :param positions: 各手指目标位置列表（长度需匹配手指数量）
        :raises HandCommandError: 参数错误或写入失败时抛出
        """
        self._validate_params(positions, self._POS_RANGE)
        self._write_registers(self.regdict['angleSet'], positions)

    def set_speed(self, speeds: List[int]) -> None:
        """
        设置各关节运动速度
        
        :param speeds: 各手指运动速度列表（长度需匹配手指数量）
        """
        self._validate_params(speeds, self._SPEED_RANGE)
        self._write_registers(self.regdict['speedSet'], speeds)

    def set_force(self, forces: List[int]) -> None:
        """
        设置各关节力度限制
        
        :param forces: 各手指力度限制列表（长度需匹配手指数量）
        """
        self._validate_params(forces, self._FORCE_RANGE)
        self._write_registers(self.regdict['forceSet'], forces)

    def grasp(self) -> None:
        """
        执行抓取动作（预设）
        """
        # 设置抓取速度和力度
        self.set_speed([self._default_speed] * self._num_DOF)
        self.set_force([self._default_force] * self._num_DOF)
        # 闭合
        self.set_positions(GRASP_POSITIONS)

    def release(self) -> None:
        """执行释放动作（全开）"""

        self.set_speed([self._default_speed] * self._num_DOF)
        self.set_force([0] * self._num_DOF)
        self.set_positions(REALEASE_POSITIONS)

    def get_status(self) -> dict:
        """
        获取当前状态信息
        
        :return: 包含各状态信息的字典
        """
        return {
            'positions': self.read_register(self.regdict['angleAct'], self._num_DOF),
            'forces': self.read_register(self.regdict['forceAct'], self._num_DOF),
            'status_code': self.read_register(self.regdict['statusCode'], 1)[0]
        }

    def read_register(self, address: int, count: int) -> Optional[List[int]]:
        """
        读取多个寄存器值（兼容旧版）
        
        :param address: 寄存器起始地址
        :param count: 读取数量
        :return: 寄存器值列表
        """
        if not self.is_connected:
            raise HandConnectionError("未建立连接")

        try:
            response = self._client.read_holding_registers(
                address=address,
                count=count,
            )
            if response.isError():
                raise HandCommandError(f"读取寄存器失败: {response}")
            return response.registers
        except ModbusException as e:
            raise HandCommandError(f"Modbus读取错误: {str(e)}") from e

    def read6(self, reg_name: str) -> None:
        """
        读取6个寄存器值（兼容旧版）
        
        :param reg_name: 寄存器名称（angleSet/forceSet/speedSet/angleAct/forceAct/errCode/statusCode/temp）
        """
        if reg_name not in self.regdict:
            print(f"无效寄存器名称: {reg_name}")
            return

        if reg_name in ['angleSet', 'forceSet', 'speedSet', 'angleAct', 'forceAct']:
            val = self.read_register(self.regdict[reg_name], 6)
            if val:
                print(reg_name,'的值依次为：', end='')
                for v in val:
                    print(v, end=' ')
                print()
            else:
                print('没有读到数据')

        elif reg_name in ['errCode', 'statusCode', 'temp']:
            val_act = self.read_register(self.regdict[reg_name], 3)
            if val_act:
                results = []
                for i in range(len(val_act)):
                    low_byte = val_act[i] & 0xFF
                    high_byte = (val_act[i] >> 8) & 0xFF
                    results.append(low_byte)
                    results.append(high_byte)
                print(reg_name,'的值依次为：', end='')
                for v in results:
                    print(v, end=' ')
                print()
            else:
                print('没有读到数据')
        else:
            print('函数调用错误，正确方式：str的值为\'angleSet\'/\'forceSet\'/\'speedSet\'/\'angleAct\'/\'forceAct\'/\'errCode\'/\'statusCode\'/\'temp\'')

    def _validate_params(self, values: List[int], valid_range: tuple) -> None:
        """参数验证"""
        if len(values) != self._num_DOF:
            raise HandCommandError(f"需要{self._num_DOF}个参数")
            
        if any(not (valid_range[0] <= v <= valid_range[1]) for v in values):
            raise HandCommandError(f"参数超出范围{valid_range}")

    def _write_registers(self, address: int, values: List[int]) -> None:
        """写入多个寄存器"""
        if not self.is_connected:
            raise HandConnectionError("未建立连接")

        try:
            response = self._client.write_registers(
                address=address,
                values=values,
            )
            if response.isError():
                raise HandCommandError(f"写入寄存器失败: {response}")
        except ModbusException as e:
            raise HandCommandError(f"Modbus写入错误: {str(e)}") from e


def main():
    hand = InspireHand()
    try:
        hand.connect()
        hand.initialize()

        hand.release()
        time.sleep(3)
        hand.grasp()
        time.sleep(3)
        hand.release()

        # 读取寄存器并输出

        # while True:
        #     hand.read6('angleAct')
        #     hand.read6('forceAct')
        #     time.sleep(2)

    finally:
        hand.disconnect()

if __name__ == '__main__':
    main()