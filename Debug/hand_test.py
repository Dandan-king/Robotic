# 测试机械手

from pymodbus.client import ModbusTcpClient
import time

# -------------------- 4. 连接机械手 --------------------

# 寄存器字典
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

# 定义设置,读取机械手位置、速度和力度的函数以及连接和断开机械手的函数

def connect_hand(ip, port):
    client = ModbusTcpClient(host=ip, port=port)
    client.connect()
    return client

def set_hand_position(client, angle_values):
    client.write_registers(regdict['angleSet'], angle_values)

def set_hand_speed(client, speed_values):
    client.write_registers(regdict['speedSet'], speed_values)

def set_hand_force(client, force_values):
    client.write_registers(regdict['forceSet'], force_values)

def read_register(client, address, count):
    # Modbus 读取寄存器
    response = client.read_holding_registers(address=address, count=count)
    return response.registers if response.isError() is False else []


def read6(client, reg_name):
    # 检查寄存器名称是否在允许的范围内
    if reg_name in ['angleSet', 'forceSet', 'speedSet', 'angleAct', 'forceAct']:
        # 直接读取与reg_name对应的寄存器，读取的数量为6
        val = read_register(client, regdict[reg_name], 6)
        if len(val) < 6:
            print('没有读到数据')
            return
        print('读到的值依次为：', end='')
        for v in val:
            print(v, end=' ')
        print()
    
    elif reg_name in ['errCode', 'statusCode', 'temp']:
        # 读取错误代码、状态代码或温度，每次读取3个寄存器
        val_act = read_register(client, regdict[reg_name], 3)
        if len(val_act) < 3:
            print('没有读到数据')
            return
            
        # 初始化存储高低位的数组
        results = []
        
        # 将每个寄存器的高位和低位分开存储
        for i in range(len(val_act)):
            # 读取当前寄存器和下一个寄存器
            low_byte = val_act[i] & 0xFF            # 低八位
            high_byte = (val_act[i] >> 8) & 0xFF     # 高八位
        
            results.append(low_byte)  # 存储低八位
            results.append(high_byte)  # 存储高八位

        print('读到的值依次为：', end='')
        for v in results:
            print(v, end=' ')
        print()
    
    else:
        print('函数调用错误，正确方式：str的值为\'angleSet\'/\'forceSet\'/\'speedSet\'/\'angleAct\'/\'forceAct\'/\'errCode\'/\'statusCode\'/\'temp\'')


def read_hand_position(client):
    print("读取机械手位置...")
    read6(client, 'angleAct')

def read_hand_force(client):
    print("读取机械手力度...")
    read6(client, 'forceAct')

def disconnect_hand(client):
    client.close()

if __name__ == "__main__":
    hand_ip = '192.168.11.210'
    hand_port = 6000

    # 连接机械手
    hand_client = connect_hand(hand_ip, hand_port)


    # try:
    #     # 控制机械手完全张开
    #     print("抓取物体...")
    #     set_hand_speed(hand_client, [200, 200, 200, 200, 200, 200])
    #     time.sleep(1)
    #     set_hand_force(hand_client, [500, 500, 500, 500, 500, 500])
    #     time.sleep(1)
    #     set_hand_position(hand_client, [1000, 1000, 1000, 1000, 1000, 1000])
    #     time.sleep(2)

    #     # 读取机械手的位置和力度
    #     position = read_hand_position(hand_client)
    #     force = read_hand_force(hand_client)

    #     # 控制机械手抓取物体（半闭合）
    #     print("抓取物体...")
    #     set_hand_position(hand_client, [500, 500, 500, 500, 500, 200])
    #     time.sleep(2)

    #     # 读取机械手的位置、速度和力度
    #     position = read_hand_position(hand_client)
    #     force = read_hand_force(hand_client)

    try:
        set_hand_speed(hand_client, [1000, 1000, 1000, 1000, 1000, 1000])
        time.sleep(1)
        set_hand_force(hand_client, [1000, 1000, 1000, 1000, 1000, 1000])

        time.sleep(1)

        set_hand_position(hand_client, [500, 500, 500, 600, 600, 0])
        time.sleep(3)
        set_hand_position(hand_client, [500, 500, 500, 400, 400, 0])

    finally:
        # 断开机械手的连接
        disconnect_hand(hand_client)