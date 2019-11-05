#!/usr/bin/python3

import can

VESC_BYTE_ORDER = 'big'

MSG_VESC_STATUS = 9
MSG_VESC_STATUS_2 = 14
MSG_VESC_STATUS_3 = 15
MSG_VESC_STATUS_4 = 16
MSG_VESC_STATUS_5 = 27

def pop_int(buf, begin, length):
  return int.from_bytes(data[begin:(begin+length)], byteorder=VESC_BYTE_ORDER, signed=True)

if __name__ == '__main__':
  bus = can.Bus(channel="can0", interface="socketcan")
  while True:
    # Continuously receive frames
    msg = bus.recv()
    if msg is None:
      continue
    msg_id = msg.arbitration_id
    data = msg.data
    # Split arb id according to arb_id = (message code) << 8 | (source device)
    # This is how the VESC arb ids are made
    source_dev = msg_id & 0xFF
    msg_type = msg_id >> 8
    print(f"{source_dev}\t{msg_type}\t", end="")
    if msg_type == MSG_VESC_STATUS:
      rpm = pop_int(data, 0, 4)
      current = pop_int(data, 4, 2) / 10.0
      duty = pop_int(data, 6, 2) / 1e3
      print(f'RPM: {rpm} Current: {current} Duty: {duty}')
    elif msg_type == MSG_VESC_STATUS_2:
      amp_hour = pop_int(data, 0, 4) / 1e4
      amp_hour_charged = pop_int(data, 4, 4) / 1e4
      print(f'AH: {amp_hour} AH Charged: {amp_hour_charged}')
    elif msg_type == MSG_VESC_STATUS_3:
      watt_hour = pop_int(data, 0, 4) / 1e4
      watt_hour_charged = pop_int(data, 4, 4) / 1e4
      print(f'WH: {watt_hour} WH Charged: {watt_hour_charged}')
    elif msg_type == MSG_VESC_STATUS_4:
      temp_fet = pop_int(data, 0, 2) / 10.0
      temp_motor = pop_int(data, 2, 2) / 10.0
      current_in = pop_int(data, 4, 2) / 10.0
      pid_pos_now = pop_int(data, 6, 2) / 50.0
      print(f'FET temp: {temp_fet} Motor temp: {temp_motor} Current in: {current_in} PID Pos Now: {pid_pos_now}')
    elif msg_type == MSG_VESC_STATUS_5:
      tacho_val = pop_int(data, 0, 4)
      v_in = pop_int(data, 4, 2) / 10.0
      print(f'Tacho Val: {tacho_val} Voltage In: {v_in}')
    else: # Unknown message, just print data
      for b in data:
        b_ = f'{b:X}'
        if len(b_) == 1:
          b_ = "0"+b_
        print(b_, end=" ")
      print()
