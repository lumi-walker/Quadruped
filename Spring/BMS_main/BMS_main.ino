

#include "BMS.h"
#include <vector>
#define _DEBUG 1

bool BMSError = false;

CurrentMonitor currentMonitor;
TemperatureMonitor temperatureMonitor;
RelayController relayLifts(LIFTS_PWR_PIN_1, LIFTS_PWR_PIN_2);
RelayController relayMotors(MOTOR_PWR_PIN_1, MOTOR_PWR_PIN_2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(500);
}

void loop() {
  float current;
  float temp;
  float voltage;

  // read current
  ErrorStatus currentErrStatus;
  std::vector<ErrorStatus> tempErrStatus;
  bool SUCCESS = currentMonitor.readCurrent(current,currentErrStatus);

  // read temperature
  SUCCESS |= temperatureMonitor.readTemperature(temp,tempErrStatus);


  // read voltage
  
  if(!SUCCESS) {
    /*
     * Tell Due there is an error so that it stops the motor
     * Finish all current, temp, and voltage reads to get all errors
     * wait for Due to send OKAY back to Teensy, then shut off relays
     */

     // wait for response from Due first
     relayMotors.disconnect();
     relayLifts.disconnect();
     
  }

  
  
  
  

}
