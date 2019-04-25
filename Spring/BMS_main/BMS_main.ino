

#include "BMS.h"
#include <vector>
#include<SD.h>
#define _DEBUG 1

File errorLog;



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


void loop() {
  errorLogger.updateTime();
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
    errorLogger.log(CURRENT_ERROR,errors2send);
    errors2send.clear();
  }
  
  // read temperature
  SUCCESS |= temperatureMonitor.readTemperature(temp,tempErrStatus);

  if(!tempErrStatus.empty()) {
    errorMessage |= ERROR_MESSAGE_MASK;
    for(int i = 0; i < tempErrStatus.size(); i++) {
      errors2send.push_back(tempErrStatus[i]);
      errorMessage |= (1<< tempErrStatus[i].errMsg);
    }
    errorLogger.log(TEMPERATURE_ERROR,errors2send);
    errors2send.clear();
    
  }

  // read voltage
  // DOESN'T EXIST YET
  // SUCCESS |= voltageMonitor.readVoltage(voltage, voltageErrStatus)
  
  if(!SUCCESS) {
    /*
     * Tell Due there is an error so that it stops the motor
     * Finish all current, temp, and voltage reads to get all errors
     * wait for Due to send OKAY back to Teensy, then shut off relays
     */
      
    
     digitalWrite(TEENSY2DUE_CALL_PIN,HIGH);
     delay(5);
     digitalWrite(TEENSY2DUE_CALL_PIN,LOW);
     Serial1.write(errorMessage);

     if(errorMessage & CRITICAL_ERROR_MASK) {
        Serial.println(errorMessage & CRITICAL_ERROR_MASK, BIN);
        
        while(!DUE_REPLIED);
        
        relayMotors.disconnect();
        relayLifts.disconnect();  
        Serial.println("Power Disconnected");
       
        /*
         * log error message
         */
         //errorLogger.log(errors2send);

         /*
          * signal to Due to display that it's okay to disconnect battery now
          */
     } else {
        /*
         * log error message
         */
     }



        
      }
     
  }

  
  
  
  
