//Main Logic Functions and Initialization
#include "LCD.h"
//#include "global_include.h"

void UI_setup() {
  Serial.begin(9600);
  tft.begin();

  pinMode(bAA,INPUT_PULLUP);
  pinMode(lAA,OUTPUT);
  pinMode(bJS,INPUT_PULLUP);
  pinMode(lJS,OUTPUT);
  pinMode(bSS,INPUT_PULLUP);
  pinMode(lSS,OUTPUT);
  pinMode(bUP,INPUT_PULLUP);
  pinMode(bDN,INPUT_PULLUP);
  pinMode(bTN,INPUT_PULLUP);
  pinMode(lTN,OUTPUT);

  prevAA = millis();
  prevJS = millis();
  prevSS = millis();
  prevUP = millis();
  prevDN = millis();
  prevTN = millis();


  bool debounceCheck(long& prevTime) {
    if(millis() - prevTime >= debounceThresh) {
      prevTime = millis();
      return true;
    }
    else {
      return false;
    }
  }

  void ISR_AA() {
    if(debounceCheck(prevAA)) {
      if (currState == stateDE) {
        currState = stateAA;
      }
      else if (currState == stateAA) {
        currState = stateDE;
      }
      else {
        //if in neither do nothing haha
      }
    }
  }

  void ISR_JS() {
    if(debounceCheck(prevJS)) {
      if (currState == stateDE) {
        currState = stateJS;
      }
      else if (currState == stateJS) {
        currState = stateDE;
      }
      else {
        //if in neither do nothing haha
      }
    }
  }

  void ISR_SS() {
    if(debounceCheck(prevSS)) {
      if (currState == stateDE) {
        currState = stateSS;
      }
      else if (currState == stateSS) {
        currState = stateDE;
      }
      else {
        //if in neither do nothing haha
      }
    }
  }

  void ISR_UP() {
    if(debounceCheck(prevUP)) {
      if (currState == stateSS && setSpeed < maxSpeed) {
        setSpeed = setSpeed + dSpeed;
      }
      else {
        //if in neither do nothing haha
      }
    }
  }

  void ISR_DN() {
    if(debounceCheck(prevDN)) {
      if (currState == stateSS && setSpeed > minSpeed) {
        setSpeed = setSpeed - dSpeed;
      }
      else {
        //if in neither do nothing haha
      }
    }
  }

  void ISR_TN() {
    if(debounceCheck(prevTN)) {
      if (currState == stateSS) {
        currState = stateTurnSS;
      }
      else if (currState == stateAA) {
        currState = stateTurnAA;
      }
      else if (currState == stateTurnSS) {
        currState = stateSS;
      }
      else if (currState == stateTurnAA) {
        currState = stateAA;
      }
      else {
        //if in neither do nothing haha
      }
    }
  }

  void pingHandler() {
    while(Serial1.available()<= 0){
      incomingByte = Serial1.read();
    }
    bool isError = incomingByte && PING_TYPE_MASK;

    if(isBooting) {
      if(incomingByte == 255) {
        isBootUp = 1;
        isBooting = 0;
      }
    }
    else if (isError) {
      checkErrors(incomingByte);
      LCD.writeErrorPanel();
    }
    else if (!isError) {
      batterylvl = (int)incomingByte;
      LCD.writeBatteryLevel(batterylvl);
    }
  }

  void checkErrors(byte incomingByte) {
    is_FAULTY_CURRENT_SENSOR = incomingByte && (1 << FAULTY_CURRENT_SENSOR);
    is_FAULTY_TEMP_SENSOR =  incomingByte && (1 << FAULTY_TEMPERATURE_SENSOR);
    is_OVER_TEMP = incomingByte && (1 << OVER_TEMPERATURE);
    is_OVER_CURRENT = incomingByte && (1 << OVER_CURRENT);
    is_OVER_VOLT = incomingByte && (1 << OVER_VOLTAGE);
    is_LOW_VOLT = incomingByte && (1 << LOW_VOLTAGE);

    bool is_CRITICAL = incomingByte && CRITICAL_ERROR_MASK;
    if(is_CRITICAL) {
      currState = stateCE;
    }
  }

  attachInterrupts(digitalPinToInterrupt(bAA),ISR_AA,FALLING);
  attachInterrupts(digitalPinToInterrupt(bJS),ISR_JS,FALLING);
  attachInterrupts(digitalPinToInterrupt(bSS),ISR_SS,FALLING);
  attachInterrupts(digitalPinToInterrupt(bUP),ISR_UP,FALLING);
  attachInterrupts(digitalPinToInterrupt(bDN),ISR_DN,FALLING);
  attachInterrupts(digitalPinToInterrupt(bTN),ISR_TN,FALLING);
  attachInterrupts(digitalPinToInterrupt(18),pingHandler,RISING);

  interrupts();
}
