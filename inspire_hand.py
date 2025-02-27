# hand.py 定义了一些与机械手相关的操作

from pymodbus.client import ModbusTcpClient
from config import HAND_IP, HAND_PORT

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
    response = client.read_holding_registers(address=address, count=count)
    return response.registers if not response.isError() else []

def read6(client, reg_name):
    if reg_name in ['angleSet', 'forceSet', 'speedSet', 'angleAct', 'forceAct']:
        val = read_register(client, regdict[reg_name], 6)
        return val if len(val) == 6 else None
    elif reg_name in ['errCode', 'statusCode', 'temp']:
        val_act = read_register(client, regdict[reg_name], 3)
        if len(val_act) < 3:
            return None
        results = []
        for i in range(len(val_act)):
            low_byte = val_act[i] & 0xFF
            high_byte = (val_act[i] >> 8) & 0xFF
            results.append(low_byte)
            results.append(high_byte)
        return results
    else:
        return None

def read_hand_position(client):
    print("读取机械手位置...")
    val = read6(client, 'angleAct')
    if val:
        print(' '.join(map(str, val)))
    else:
        print('没有读到数据')

def read_hand_force(client):
    print("读取机械手力度...")
    val = read6(client, 'forceAct')
    if val:
        print(' '.join(map(str, val)))
    else:
        print('没有读到数据')

def disconnect_hand(client):
    client.close()
