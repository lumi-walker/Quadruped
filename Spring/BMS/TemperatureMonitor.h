#ifndef TEMPERATURE_MONITOR_H
#define TEMPERATURE_MONITOR_H



#include <Adafruit_MAX31855.h>
#include "BMS_pin_assignments.h"
#include <vector>
#include "BMSUtils.h"
#include <Arduino.h>

#define TEMPERATURE_MONITOR_REDUNDANCY_CHECK_THRESHOLD 0.10f
#define OVER_TEMPERATURE_THRESHOLD	60.f // degrees C
static const BMSErrorCode TemperatureErrorCode[5] = {OVER_TEMPERATURE_ERROR, 
								   					OPEN_CIRCUIT_ERROR, 
								   					SHORT_TO_GND_ERROR, 
								   					SHORT_TO_VCC_ERROR, 
								   					SHORT_TO_VCC_AND_SHORT_TO_GND_ERROR};



class TemperatureMonitor {
public:

	TemperatureMonitor() {

	}

	~TemperatureMonitor() {
		
	}

	bool readTemperature(float& temp, std::vector<ErrorStatus>& errStatus) {
		bool SUCCESS = true;
		float tmp[3] = {0, 0 , 0};

		uint8_t errCode;
		if(!errStatus.empty()) {
			errStatus.clear();
		}

		ErrorStatus t;

		for(int i = 0; i < ALL_SENSORS; i++) {
			tempReadings[i] = tSense[i].readCelsius();
			if(isnan(tempReadings[i])) {
				//error occured!
				
				SUCCESS = false;
				errCode = tSense[i].readError();
				for(BMSErrorCode err : TemperatureErrorCode) {
					if((BMSErrorCode)errCode == err) {
						t.errCode = err;
						t.faultySensorIndex = (SensorIndex)i;
						errStatus.push_back(t);
						break;
					}
				}
			} else{
				tmp[i] = tempReadings[i];
			}
		}
		SensorIndex faultySensorIndex;

		bool isErrors = checkForFaultySensor(&tempReadings[0],faultySensorIndex,TEMPERATURE_MONITOR_REDUNDANCY_CHECK_THRESHOLD);

		if(isErrors) {
			SUCCESS = false;
			t.errCode = FAULTY_SENSOR_ERROR;
			t.faultySensorIndex = faultySensorIndex;
			errStatus.push_back(t);

			if(faultySensorIndex != ALL_SENSORS) {
				tmp[faultySensorIndex] = 0;

				for(int i = 0; i < ALL_SENSORS; i++) {
					temp += tmp[i];
				}
				temp /= 2;

			} else {
				float maxtemp = tmp[0];
				for(int i = 0; i < ALL_SENSORS; i++) {
					if(tmp[i] > maxtemp) {
						maxtemp = tmp[i];
					}
				}
				temp = maxtemp;
			}
			
		} else {
			temp = (tmp[0] + tmp[1] + tmp[2])/3; 
		}

		if(temp > OVER_TEMPERATURE_THRESHOLD) {
			SUCCESS = false;
			t.errCode = OVER_TEMPERATURE_ERROR;
			t.faultySensorIndex = ALL_SENSORS;
			errStatus.push_back	(t);
		}
		

			Serial.println("_____________________________________________________________________________________");
			Serial.println("TEMPERATURE READINGS");
			Serial.println("_____________________________________________________________________________________");
			Serial.println();
		for(int i = 0; i < 3; i++) {
			Serial.print("Sensor " + String(i) + " : " + String(tempReadings[i]));
			if(i != 2) {
				Serial.print(" | ");
			} else {
				Serial.println(" | Estimated Temp: " + String(temp));
				if(!errStatus.empty()) {
					Serial.println("TEMPERATURE READINGS ERROR");

					for(int i = 0; i < errStatus.size(); i++) {
        				Serial.println("SENSOR : " + String(errStatus[i].faultySensorIndex) + " | ERROR CODE : " + String(errStatus[i].errCode));
   					 }		
				}
				Serial.println("_____________________________________________________________________________________");
				Serial.println();
			}
		}
		return SUCCESS;
	}

private:

	Adafruit_MAX31855 tSense[ALL_SENSORS] = {Adafruit_MAX31855(SPI_CLK_PIN, T_SENSE_CHIP_SEL_1, SPI_MISO_PIN),
											 Adafruit_MAX31855(SPI_CLK_PIN, T_SENSE_CHIP_SEL_2, SPI_MISO_PIN),
											 Adafruit_MAX31855(SPI_CLK_PIN, T_SENSE_CHIP_SEL_3, SPI_MISO_PIN)};

	float tempReadings[ALL_SENSORS];

};


#endif