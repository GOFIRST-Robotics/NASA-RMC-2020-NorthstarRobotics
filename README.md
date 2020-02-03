NASA-RMC-2020-RT
===
The code for the RT controller on the Lunabotics robot for the 2020 season.

Interface
===
The controller takes extended CAN messages. 
The arbitration ID of these messages should be formatted as follows:

| Bytes 31-8                      | Bytes 7-0                |
|---------------------------------|--------------------------|
| Message ID (subsystem-specific) |  Controller subsystem ID |

Below is a table of controller subsystem IDs:

| Subsystem  | ID   |
|------------|------|
| Drivetrain | 100  |
| ACHOO      | 101  |
| GESUNDHEIT | 102  |
| Digger     | 103  |

These IDs start at 100 to avoid interaction with VESCs.
See the subsystem tables below for message IDs.
Message IDs are required to begin above 40 to avoid interaction with VESC command IDs, which go up to 27.

Drivetrain
---
| Command    | ID   | Broadcasts/Listens  | Length | Data |
|------------|------|---------------------|--------|------|
| Twist      |  40  | Listens             |   8    | Bits 7-4: Linear velocity (int32) mm/sBits 3-0: Angular Velocity (int32) urad/s |

ACHOO
---
| Command    | ID   | Broadcasts/Listens  | Length | Data |
|------------|------|---------------------|--------|------|
| Set kneel  |  40  | Listens             |   1    | 0: Stand<br>1: Kneel |
| Kneel state|  41  | Broadcasts (10 Hz)  |   1    | 0: Standing<br>1: Moving up<br>2: Moving Down<br>3: Kneeling |

GESUNDHEIT
---
| Command    | ID   | Broadcasts/Listens    | Length | Data |
|------------|------|-----------------------|--------|------|
| Set extension  |  50  | Listens           |   1    | 0: Stow<br>1: Extend |
| Conveyor speed |  51  | Listens           |   4    | Speed (int32) rpm |
| Set door       |  52  | Listens           |   1    | 0: Close<br>1: Open |
| Extension state | 53 | Broadcasts (10 Hz) |   6    | Byte 0: <br>&nbsp;0: Stowed<br>&nbsp;1: Moving to stow<br>&nbsp;2: Moving to extend<br>&nbsp;3: Extended<br>Byte 1:<br>&nbsp;0: Door closed<br>&nbsp;1: Door open<br>Byte 2-5: Speed (int32) rpm |

