import struct

def ble_rx_stream(ble, uuid_key):
    data_buffer = bytearray()
    next_packet = 1
    while next_packet != 0:
        data = ble.read(ble.uuid[uuid_key])
        if data[0] == 0:
            next_packet = 0
        else:
            assert(data[0] == next_packet)
            next_packet += 1
        data_buffer += data[1:]
    return data_buffer

def unpack_stream(format, buffer):
    if format[-1] == '*':
        unpack = struct.iter_unpack
        format = '<' + format[:-1]
    else:
        unpack = struct.unpack
        format = '<' + format
    return unpack(format, buffer)