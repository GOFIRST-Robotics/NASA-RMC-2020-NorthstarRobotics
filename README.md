NASA-RMC-2020-RT
===
The code for the RT controller on the Lunabotics robot for the 2020 season.

Interface
===
The controller takes CAN messages. 
The arbitration ID of these messages should be formatted as follows:

| Bytes 31-8                      | Bytes 7-0                |
|---------------------------------|--------------------------|
| Command ID (subsystem-specific) |  Controller subsystem ID |

Below is a table of controller subsystem IDs:

| Subsystem  | ID   |
|------------|------|
| Drivetrain |  100 |
| STANDR     |  101 |
| Elevator   | 102  |
| Digger     | 103  |

These IDs start at 100 to avoid interaction with VESCs.
See the subsystem tables below for command IDs.
Command IDs are encouraged to begin above 40 to avoid interaction with VESC command IDs, which go up to 27.

Drivetrain
---
| Command    | ID   | Data |
|------------|------|------|
| Twist      |  40  | Bits 7-4: Linear velocity (int32) mm/s, Bits 3-0: Angular Velocity (int32) urad/s | 