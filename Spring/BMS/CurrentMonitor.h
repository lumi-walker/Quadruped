#ifndef CURRENT_MONITOR_H
#define CURRENT_MONITOR_H

#include "ACS722.h"
#include <Arduino.h>
#include "BMSUtils.h"


#define _DEBUG_PRINT_CURRENTS 1
#define _PRINT_PERCENT_ERROR 1


// in Amps
#define OVER_CURRENT_THRESHOLD 1.f //A

/*
	ACS722 datasheet specifies total output error to be +/- 2% error
	so we will define our redundancy check threshold to be 5% 
*/
#define CURRENT_MONITOR_REDUNDANCY_CHECK_THRESHOLD 1.f

static const BMSErrorCode CurrentErrorCode[4] = {OVER_CURRENT_ERROR, 
												FAULTY_SENSOR_ERROR, 
												OVER_CURRENT_AND_FAULTY_SENSOR, 
												NO_ERROR};

class CurrentMonitor {
public:
	CurrentMonitor()  {

	}

	~CurrentMonitor() {

	}

	bool readCurrent(float& current, ErrorStatus& errStatus) {
		// note - do not return SUCCESS too early, need to check for faulty sensor error AND over current error
		bool SUCCESS = true;

		// add filtering?
		for(int i = 0; i < ALL_SENSORS; i++) {
			currentReadings[i] = iSense[i].readCurrent();

			#ifdef _DEBUG_PRINT_CURRENTS
			Serial.print("Sensor " + String(i) + " : " + String(currentReadings[i]));
			if(i == ALL_SENSORS-1) {
				Serial.println();
			} else {
				Serial.print(" // ");
			}
			#endif
		}

		// check for faulty sensor
		SensorIndex faultySensorIndex;
		bool isErrors = checkForFaultySensor(&currentReadings[0], faultySensorIndex, CURRENT_MONITOR_REDUNDANCY_CHECK_THRESHOLD);


		if(isErrors) {
			SUCCESS = false;
			errStatus.errCode = FAULTY_SENSOR_ERROR;
			errStatus.faultySensorIndex = faultySensorIndex;
		} else {
			faultySensorIndex = (SensorIndex)-1;
		}

		if(faultySensorIndex == ALL_SENSORS) {
			// check if any 1 of them has over current to warn the user
			// assume the worst
			for(int i = 0; i < ALL_SENSORS; i++) {
				if(currentReadings[i] > OVER_CURRENT_THRESHOLD) {
					current = currentReadings[i];
					errStatus.errCode = OVER_CURRENT_AND_FAULTY_SENSOR;
					break;
				}
			}
		} else {
			current = 0;
			for(int i = 0; i < ALL_SENSORS; i++) {
				if(i != faultySensorIndex) {
					current += currentReadings[i];
				}
			}
			// take average and multiply by 3 
			// we assume that the current still flows fine through faulty sensor but sensor just can't measure it
			// therefore the current is still roughly distributed evenly amongst the 3
			if(faultySensorIndex == -1) {
				// all sensors are okay... do nothing
			} else {
				current *= 3/2;
			}

			if(current > OVER_CURRENT_THRESHOLD) {
				SUCCESS = false;
				errStatus.errCode = OVER_CURRENT_ERROR;
			}
		}

		return SUCCESS;
	}


private:

	ACS722 iSense[ALL_SENSORS] = {ACS722(I_SENSE_READ_PIN_1), 
								  ACS722(I_SENSE_READ_PIN_2), 
								  ACS722(I_SENSE_READ_PIN_3)};

	float currentReadings[ALL_SENSORS];
};

#endif