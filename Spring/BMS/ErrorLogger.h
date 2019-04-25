#ifndef ERROR_LOGGER_H
#define ERROR_LOGGER_H

#include<Arduino.h>
#include<string>
#include "BMSUtils.h"

class ErrorLogger {
public:
	ErrorLogger() {
		// init file name to "log-YY/MM/DD-index.txt", check if file name exists first
		// initialize time
	}

	~ErrorLogger() {

	}

	void updateTime() {
		if(micros() - prevT > 60000000) {
			minutes += 1;
			prevT = micros();
		}

		if(minutes >= 60) {
			hours += 1;
			minutes = 0;
		}
	}


	void log(std::vector<ErrorStatus> errors) {
		// convert to verbose error message format
		// see if it matches with previous
		// if it does, then do not log
		// otherwise, log and update with (hours < 15 (4 bits) | minutes < 60 (6 bits) | seconds < 60 (6 bits) | verboseErrorMessage 16 bits) = 32 bit word
		/*
		  SD.begin(SD_CARD_CS);
      errorLog = SD.open("log1.txt",FILE_WRITE);
      if(errorLog){
        errorLog.println("Error Occured");
        errorLog.close();

        delay(1500);
        SD.begin(SD_CARD_CS);
        errorLog = SD.open("log1.txt");
        Serial.println("file content:");

        while(errorLog.available())
        Serial.write(errorLog.read());
        errorLog.close();
        Serial.println();
      }
		*/
	}



private:

	/*
	16 bits
	first 3 bits indicate the sensor
	next 11 bits indicate the error code
	last 2 bits not used
	*/
	int16_t verboseErrorMessage;
	int16_t previousVerboseErrorMessage;

	std::string fileName;
	int32_t completeLog;

	// store time since file opened
	long prevT;
	long minutes = 0;
	long hours = 0;

};


#endif