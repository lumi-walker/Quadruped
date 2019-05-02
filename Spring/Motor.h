

#include "SMi21.h"
#include "motor_pin_assignments.h"


void motor_init() {
	pinMode(M1_IN1, OUTPUT);
	pinMode(M1_IN2, OUTPUT);
	pinMode(M1_IN3, OUTPUT);
	pinMode(M1_IN4, OUTPUT);
  pinMode(M1_IN5, OUTPUT);
  pinMode(M1_IN6, OUTPUT);

  pinMode(M1_OUT1, INPUT);
	pinMode(M1_OUT2, INPUT);
	pinMode(M1_OUT3, INPUT);
	pinMode(M1_OUT4, INPUT);

  pinMode(M2_IN1, OUTPUT);
	pinMode(M2_IN2, OUTPUT);
	pinMode(M2_IN3, OUTPUT);
	pinMode(M2_IN4, OUTPUT);
  pinMode(M2_IN5, OUTPUT);
  pinMode(M2_IN6, OUTPUT);

  pinMode(M2_OUT1, INPUT);
  pinMode(M2_OUT2, INPUT);
  pinMode(M2_OUT3, INPUT);
  pinMode(M2_OUT4, INPUT);
}

SMi21 M1(M1_IN1,M1_IN2,M1_IN3,M1_IN4,M1_IN5,M1_IN6);
SMi21 M2(M2_IN1,M2_IN2,M2_IN3,M2_IN4,M2_IN5,M2_IN6);
//,M1_OUT1,M1_OUT2,M1_OUT3,M1_OUT4

//example of higher level function
void setspeed(float speed,bool direc){
  M1.setvel(speed,direc);
  M2.setvel(speed,!direc);
}

void turnRight(){
	M1.setvel(0.7,1);
	M2.setvel(0.7,1);
}

void turnLeft(){
	M1.setvel(0.7,0);
	M2.setvel(0.7,0);
}
