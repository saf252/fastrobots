import asyncio
import struct
from base_ble import LOG

def ble_rx_stream(ble, uuid_key, debug_verbose=False):
    loop = asyncio.get_event_loop()
    fut = loop.create_future()
    data_buffer = bytearray()
    next_packet = 1
    def rx_stream_notification_handler(sender, data):
        nonlocal data_buffer
        nonlocal next_packet
        if debug_verbose:
            LOG.debug(f'{sender} -> {data.hex()}')
        try:
            data_buffer += data[1:]
            if data[0] == 0:
                LOG.debug(f'Received {data_buffer.hex()}')
                fut.set_result(data_buffer)
                asyncio.create_task(ble.device._stop_notify(ble.uuid[uuid_key]))
            else:
                assert(data[0] == next_packet)
                next_packet += 1
        except Exception as exc:
            fut.set_exception(exc)
    ble.start_notify(ble.uuid[uuid_key], rx_stream_notification_handler)
    return fut

def unpack_stream(format, buffer):
    if format[-1] == '*':
        unpack = struct.iter_unpack
        format = '<' + format[:-1]
    else:
        unpack = struct.unpack
        format = '<' + format
    return unpack(format, buffer)