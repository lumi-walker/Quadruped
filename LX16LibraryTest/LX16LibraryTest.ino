#include <LX16A.h>


#define SERVO_ID 3

void setup() {
  // put your setup code here, to run once:
  Serial.begin(LX16A_BAUDRATE);
  LX16A::WRITE_ID(Serial, LX16A_BROADCAST_ID, LX16A_DEFAULT_ID);
  LX16A::WRITE_ID(Serial, LX16A_DEFAULT_ID, SERVO_ID);
}


uint16_t angles[4] = {100,500,900,500};
uint16_t dur = 500;
void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0; i < 4; i++) {
    LX16A::WRITE_MOVE_TIME(Serial,SERVO_ID,angles[i],dur);
    delay(1000);
  }
}
