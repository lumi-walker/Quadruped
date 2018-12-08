#include "LX16A.h"

byte check_sum(byte buf[]){
      byte i;
      uint16_t temp = 0;
      for (i = 2; i < buf[3] + 2; i++) {
        temp += buf[i];
      }
      temp = ~temp;
      i = (byte)temp;
      return i;
}

int ReceiveHandle(HardwareSerial &SerialX, byte *ret) {
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  while (SerialX.available()) {
    rxBuf = SerialX.read();
    delayMicroseconds(100);
    if (!frameStarted) {
      if (rxBuf == LX16A_FRAME_HEADER) {
        frameCount++;
        if (frameCount == 2) {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted) {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3) {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7) {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3) {

#ifdef LOBOT_DEBUG
        Serial.print("RECEIVE DATA:");
        for (i = 0; i < dataCount; i++) {
          Serial.print(recvBuf[i], HEX);
          Serial.print(":");
        }
        Serial.println(" ");
#endif

        if (check_sum(recvBuf) == recvBuf[dataCount - 1]) {

#ifdef LOBOT_DEBUG
          Serial.println("Check SUM OK!!");
          Serial.println("");
#endif

          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
}


void LX16A::WRITE_ID(HardwareSerial& SerialX, const uint8_t& old_ID, const uint8_t& new_ID) {
  byte buf[7];
  buf[0] = buf[1] = LX16A_FRAME_HEADER;
  buf[2] = old_ID;
  buf[3] = LX16A_DATLEN_SERVO_ID_WRITE;
  buf[4] = LX16A_COMMID_SERVO_ID_WRITE;
  buf[5] = new_ID;
  buf[6] = check_sum(buf);
  SerialX.write(buf, 7);
}

void LX16A::WRITE_MOVE_TIME(HardwareSerial& SerialX, const uint8_t& servo_ID, uint16_t angle, uint16_t duration) {
  byte buf[10];

  if(angle < 0)
    angle = 0;
  if(angle > 1000)
    angle = 1000;

  buf[0] = buf[1] = LX16A_FRAME_HEADER;
  buf[2] = servo_ID;
  buf[3] = LX16A_DATLEN_MOVE_TIME_WRITE;
  buf[4] = LX16A_COMMID_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(angle);
  buf[6] = GET_HIGH_BYTE(angle);
  buf[7] = GET_LOW_BYTE(duration);
  buf[8] = GET_HIGH_BYTE(duration);
  buf[9] = check_sum(buf);
  SerialX.write(buf, 10);
}

void LX16A::WRITE_MODE(HardwareSerial& SerialX, const uint8_t& servo_ID, uint8_t mode, int16_t speed) {
	byte buf[10];

	buf[0] = buf[1] = LX16A_FRAME_HEADER;
	buf[2] = servo_ID;
	buf[3] = LX16A_DATLEN_MODE_WRITE;
	buf[4] = LX16A_COMMID_MODE_WRITE;
	buf[5] = mode;
	buf[6] = LX16A_NULL;
	buf[7] = GET_LOW_BYTE((uint16_t)speed);
	buf[8] = GET_HIGH_BYTE((uint16_t)speed);
	buf[9] = check_sum(buf);
	SerialX.write(buf,10);
}

/*
void LX16A::WRITE_VELOCITY_MODE(HardwareSerial& SerialX, const uint8_t& servo_ID, uint16_t speed) {
	LX16A::WRITE_MODE(SerialX,servo_ID,ServoMode::VELOCITY_CONTROL,speed);
}

void LX16A::WRITE_POSITION_CONTROL(HardwareSerial& SerialX, const uint8_t& servo_ID, uint16_t angle, uint16_t duration) {
	LX16A::WRITE_MODE(SerialX,servo_ID,ServoMode::POSITION_CONTROL,angle);
	LX16A::WRITE_MOVE_TIME(SerialX,servo_ID,angle,duration);
}
*/
// TEST THESE LAST TWO FUNCTIONS ON HARDWARE
/*
void LX16A::WRITE_TRIGGERED_MOVE_TIME(HardwareSerial& SerialX, const uint8_t& servo_ID, uint16_t angle, uint16_t duration) {
	byte buf[10];

	buf[0] = buf[1] = LX16A_FRAME_HEADER;
	buf[2] = servo_ID;
	buf[3] = LX16A_DATLEN_TRIGGERED_MOVE_TIME_WRITE;
	buf[4] = LX16A_COMMID_TRIGGERED_MOVE_TIME_WRITE;
 	buf[5] = GET_LOW_BYTE(angle);
 	buf[6] = GET_HIGH_BYTE(angle);
 	buf[7] = GET_LOW_BYTE(duration);
 	buf[8] = GET_HIGH_BYTE(duration);
 	buf[9] = check_sum(buf);
 	SerialX.write(buf, 10);
}

void LX16A::WRITE_MOVE_TIME_TRIGGER(HardwareSerial& SerialX,  const uint8_t& servo_ID) {
	byte buf[6];

	buf[0] = buf[1] = LX16A_FRAME_HEADER;
	buf[2] = servo_ID;
	buf[3] = LX16A_DATLEN_MOVE_TIME_TRIGGER_WRITE;
	buf[4] = LX16A_COMMID_MOVE_TIME_TRIGGER_WRITE;
 	buf[5] = check_sum(buf);
 	SerialX.write(buf,6);

}
*/
