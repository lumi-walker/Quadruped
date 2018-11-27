#ifndef LX16A_DEFINES_H
#define LX16A_DEFINES_H

//baud rate for Serial object
#define LX16A_BAUDRATE 115200

// header frame
#define LX16A_FRAME_HEADER 0x55

// broadcast ID to command all servos
#define LX16A_BROADCAST_ID 0xFE
#define LX16A_DEFAULT_ID 0x01

// COMMAND IDs
#define LX16A_COMMID_MOVE_TIME_WRITE 1
#define LX16A_COMMID_SERVO_ID_WRITE 13

#define LX16A_DATLEN_MOVE_TIME_WRITE 7
#define LX16A_DATLEN_SERVO_ID_WRITE 4

#endif