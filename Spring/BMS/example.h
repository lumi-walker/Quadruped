
bool isCurrentSensorFaulty = byteReceived && (1 << FAULTY_CURRENT_SENSOR);
bool isTemperatureSensorFaulty = byteReceived && (1 << FAULTY_TEMPERATURE_SENSOR);
bool isOverTemperature = byteReceived && (1 << OVER_TEMPERATURE);


volatile byte incomingByte;

#define CRITICAL_ERROR_MASK (1 << OVER_TEMPERATURE) | (1 << OVER_CURRENT) | (1 << OVER_VOLTAGE) | (1 << LOW_VOLTAGE)


bool isBootUpStage = true;
bool bootUpComplete = false;
bool isBootDownStage = false;

// ISR
void pingHandler() {
	while(Serial1.available() <= 0);
	incomingByte = Serial1.read();

	if(isBootUpStage) {
		if(incomingByte == 254) {
			bootUpComplete = true;
			isBootUpStage = false;
		}
	}
}

void checkErrors(byte incomingByte) {
	bool isCritical = incomingByte && CRITICAL_ERROR_MASK;

	if(isCritical) {
		shutDownMotors();
	} 

	displayErrorMessages(incomingByte);
}

void setup() {
	attachInterrupt(digitalPinToInterrupt(18),pingHandler,RISING); // RISING, FALLING, CHANGING
	interrupts();

	while(bootUpComplete == false);
}

void loop() {
	checkErrors(incomingByte);
}


