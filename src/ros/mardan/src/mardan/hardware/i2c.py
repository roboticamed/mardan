import smbus

__all__ = ['init_i2c_bus', 'send_speed']

bus = None
address = None

def init_i2c_bus(i2c_address=0x11):
    global bus
    global address
    bus = smbus.SMBus(1)
    address = i2c_address

def send_speed(speed_l, speed_r, log=False):
    var_l = __int_to_bytes_array(speed_l)
    var_r = __int_to_bytes_array(speed_r)

    values = var_l + var_r

    __send_values(values)
    if log:
        print("RPI: Hi Arduino, I sent you" + str(values))

    return values

def __int_to_bytes_array(value):
    result = []
    intBytes = 2
    mask = 0xFF

    for i in range(0, intBytes):
        result.insert(0, value & mask)
        value >>= 8

    return result

def __send_values(value):
    bus.write_i2c_block_data(address, 0, value)
    return -1