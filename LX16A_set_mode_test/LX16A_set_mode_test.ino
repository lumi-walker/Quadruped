#include <LX16A.h>
#include <LX16A_DEFINES.h>

#define SERVO_ID 4
int16_t mspeeds[6] = {-1000,-700,-400,400,700,1000};
void setup() {
  // put your setup code here, to run once:
  Serial.begin(LX16A_BAUDRATE);
  LX16A::WRITE_ID(Serial,LX16A_BROADCAST_ID,SERVO_ID);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  for(int i = 0; i < 6; i++) {
     LX16A::WRITE_MODE(Serial,SERVO_ID,ServoMode::VELOCITY_CONTROL,mspeeds[i]);
    delay(3000);
  }  
}
