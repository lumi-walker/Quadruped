//State Machine Logic
#include "ui_functions_init.h"

void StateMachine(State currState) {
  switch(currStage) {
    case stateDE:
      DefaultMode();
      break;
    case stateAA:
      ActiveAssistMode();
      break;
    case stateJS:
      JoystickMode();
      break;
    case stateSS:
      SetSpeedMode();
      break;
    case stateCE:
      CriticalErrorMode();
      break;
    case stateTurnAA:
      TurningMode();
      break;
    case stateTurnSS:
      TurningMode();
      break;
    case StateSit:
      SittingMode();
      break;
    case default:
      DefaultMode();
      break;
  }
}

void DefaultMode() {

}

void ActiveAssistMode() {

}

void JoystickMode() {

}

void SetSpeedMode() {

}

void TurningMode() {

}

void CriticalErrorMode(){

}

void SittingMode() {

}

void buttonBlink(int whichLED) {
  if (millis() % 2000 < 1000) {
    digitalWrite(whichLED,HIGH);
  }
  else {
    digitalWrite(whichLED,LOW);
  }
}

void readJoystick() {
  int xRead = -(analogRead(xJS)-512);
  int yRead = (analogRead(yJS)-512);
  angRead = atan2(yRead,xRead);
  double xR = (double)xRead;
  double yR = (double)yRead;
  xR = xR/512;
  yR = yR/512;
  rRead = sqrt(pow(xR,2) + pow(yR,2));
}

void ShutDownProcedure(Shutdown stage) {
  switch(stage) {
    case TEENSY_TO_DUE:
      stage = DUE_TURN_OFF_MOTOR;
      break;
    case DUE_TURN_OFF_MOTOR:
      stage = DUE_TO_TEENSY;
      break;
    case DUE_TO_TEENSY;
      stage = TEENSY_TO_DUE2;
      break;
    case TEENSY_TO_DUE2:
      stage = DUE_PRINT_LCD;
      break;
    case DUE_PRINT_LCD:
        //print to LCD
      break;
    case default:
      break;
  }
}
