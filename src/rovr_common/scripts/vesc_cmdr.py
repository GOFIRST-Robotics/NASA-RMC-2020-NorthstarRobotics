#!/usr/bin/python3

import can
import sys
import time

if __name__ == '__main__':
  if len(sys.argv) >= 4:
    vesc = int(sys.argv[1])
    if sys.argv[2] == 'd':
      msg_type = 0
      value = int(float(sys.argv[3]) * 100000.0)
    if sys.argv[2] == 'r':
      msg_type = 3
      value = int(sys.argv[3])
    arb_id = vesc | (msg_type << 8)
    bus = can.Bus(channel="can0", interface="socketcan")
    msg = can.Message(is_extended_id=True, arbitration_id=arb_id, data=value.to_bytes(4, byteorder="big", signed=True))
    while True:
        bus.send(msg)
        time.sleep(100/1000)