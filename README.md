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
| Drivetrain |  100 |
| ACHOO      |  101 |
| Elevator   | 102  |
| Digger     | 103  |

These IDs start at 100 to avoid interaction with VESCs.
See the subsystem tables below for message IDs.
Message IDs are encouraged to begin above 40 to avoid interaction with VESC command IDs, which go up to 27.

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
| Kneel state|  41  | Broadcasts (10 Hz)  |   2    | 00: Standing<br>01: Moving up<br>10: Moving Down<br>11: Kneeling |

