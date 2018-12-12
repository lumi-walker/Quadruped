#ifndef LX16A_H
#define LX16A_H

#include "LX16A_DEFINES.h"
#include <HardwareSerial.h>
#include "Arduino.h"

#define GET_LOW_BYTE(A) (uint8_t)((A))
//Macro function  get lower 8 bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//Macro function  get higher 8 bits of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//Macro Function  put A as higher 8 bits   B as lower 8 bits   which amalgamated into 16 bits integer


enum{
	POSITION_CONTROL,
	VELOCITY_CONTROL
} typedef ServoMode;


namespace LX16A {

	void WRITE_ID(HardwareSerial& SerialX, const uint8_t& old_ID, const uint8_t& new_ID);
	void WRITE_MOVE_TIME(HardwareSerial& SerialX, const uint8_t& servo_ID, uint16_t angle, uint16_t duration);
	void WRITE_MODE(HardwareSerial& SerialX, const uint8_t& servo_ID, uint8_t mode, int16_t speed);


}



#endif
