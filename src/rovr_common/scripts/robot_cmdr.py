#!/usr/bin/python3

import can
import sys
import time

if __name__ == '__main__':
  if len(sys.argv) >= 3:
    system = sys.argv[1]
    has_msg = False
    data = None
    if system == "achoo":
      if sys.argv[2] == "kneel":
        data = bytes([1])
      else:
        data = bytes([0])
      arb_id = (40 << 8) | 101
    if system == "drive":
      data = int(sys.argv[2]).to_bytes(4, byteorder='big', signed=True) + int(sys.argv[2]).to_bytes(4, byteorder='big', signed=True)
      arb_id = (35 << 8) | 100
    if data is not None:
      bus = can.Bus(channel="can0", interface="socketcan")
      msg = can.Message(is_extended_id=True, arbitration_id=arb_id, data=data)
      print(data)
      while True:
          bus.send(msg)
          time.sleep(100/1000)