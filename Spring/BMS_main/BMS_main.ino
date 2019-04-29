

#include "BMS.h"
#include <vector>
#include<SD.h>
#define _DEBUG 1

File errorLog;

#define OKAY_TO_UNPLUG  0xFF

byte ERROR_MESSAGE_MASK = (1 << 7);
byte CRITICAL_ERROR_MASK = (1 << OVER_TEMPERATURE) | (1 << OVER_CURRENT) | (1 << OVER_VOLTAGE) | (1 << LOW_VOLTAGE);
byte CLEAR_CRITICAL_ERROR  = ~CRITICAL_ERROR_MASK;
bool BMSError = false;

CurrentMonitor currentMonitor;
TemperatureMonitor temperatureMonitor;
RelayController relayLifts(LIFTS_PWR_PIN_1, LIFTS_PWR_PIN_2);
RelayController relayMotors(MOTOR_PWR_PIN_1, MOTOR_PWR_PIN_2);
ErrorLogger errorLogger;

bool CALL_TO_DUE_SENT = false;
volatile bool DUE_REPLIED = false;
uint16_t printCount = 0;
void DueReplied() {
  DUE_REPLIED = true;
}
bool DEBUG_LOG = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial1.setTX(26);
  Serial1.setRX(27);

  attachInterrupt(digitalPinToInterrupt(DUE2TEENSY_CALL_PIN),DueReplied,FALLING);
  interrupts();
  delay(500);


  Serial.println("done");
}

byte prevErrorMessage = 0;

void loop() {

  byte errorMessage = 0;
  float current;
  float temp;
  float voltage;

  // read current
  ErrorStatus currentErrStatus;
  std::vector<ErrorStatus> tempErrStatus;
  std::vector<ErrorStatus> errors2send;
  
  bool SUCCESS = currentMonitor.readCurrent(current,currentErrStatus);

  if(!SUCCESS) {
    errors2send.push_back(currentErrStatus);
    errorMessage |= (1 << currentErrStatus.errMsg) | ERROR_MESSAGE_MASK;
  }
  
  // read temperature
  SUCCESS &= temperatureMonitor.readTemperature(temp,tempErrStatus);

  if(!tempErrStatus.empty()) {
    errorMessage |= ERROR_MESSAGE_MASK;
    for(int i = 0; i < tempErrStatus.size(); i++) {
      errors2send.push_back(tempErrStatus[i]);
      errorMessage |= (1<< tempErrStatus[i].errMsg);
    }
  }

  // read voltage
  // DOESN'T EXIST YET
  // SUCCESS &= voltageMonitor.readVoltage(voltage, voltageErrStatus)

  
   if(errorMessage != prevErrorMessage) {
      // different error message, ping Due
       Serial.println("WRITE");
       digitalWrite(TEENSY2DUE_CALL_PIN,HIGH);
       delay(5);
       digitalWrite(TEENSY2DUE_CALL_PIN,LOW);
       Serial.println(errorMessage,BIN);
       Serial1.write(errorMessage);
       prevErrorMessage = errorMessage;
   } else {
      // do not ping Due  
   }
   
  if(!SUCCESS) {
    /*
     * Tell Due there is an error so that it stops the motor
     * Finish all current, temp, and voltage reads to get all errors
     * wait for Due to send OKAY back to Teensy, then shut off relays
     */
      
       if(errorMessage & CRITICAL_ERROR_MASK) {
          Serial.println(errorMessage & CRITICAL_ERROR_MASK, BIN);
          
          while(!DUE_REPLIED);
          
          relayMotors.disconnect();
          relayLifts.disconnect();  
          Serial.println("Power Disconnected");
         
          errorLogger.log(true, errors2send); // critical log
  
          digitalWrite(TEENSY2DUE_CALL_PIN,HIGH);
          delay(5);
          digitalWrite(TEENSY2DUE_CALL_PIN,LOW);
          Serial1.write(OKAY_TO_UNPLUG);
          
       } else {
           if(DEBUG_LOG) {
              errorLogger.log(false,errors2send); // not critical log (debug log)
           }
       }  
  }
     
}

  
  
  
  
