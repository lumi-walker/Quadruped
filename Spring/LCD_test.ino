#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "math.h"
#include "motor.h"

// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

//String myInput;
uint16_t  dx = tft.width()/5;
uint16_t  dy = tft.height()/9;
uint16_t xmax = tft.width();
uint16_t  ymax = tft.height();

#define bAA 42 //double-check ordering
#define bJS 44
#define bSS 46
#define bUP 48
#define bDN 50
#define bTN 52

#define lAA 23
#define lJS 25
#define lSS 27
#define lTN 29

#define xJS A4
#define yJS A5
#define PI 3.1415926535897932384626433832795

bool modeAA = 0;
bool modeJS = 0;
bool modeSS = 0;
bool modeTN = 0;
bool modeDef = 1;

int valAA = 1;
int valJS = 1;
int valSS = 1;
int valUP = 1;
int valDN = 1;
int valTN = 1;

double angRead = 0;
double rRead = 0;

float SetSpeed = 0.0;
float minSpeed = 0.1;
float maxSpeed = 1.5;

int lastJS =  5;
int timer = 0;
int timer_SS = 0;
int timer_JS = 0;
int prevMillis = 0;
int prevMillis_SS = 0;
int prevMillis_JS = 0;

void setup() {
  Serial.begin(9600);
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

  motor_init();
  M1.turnon();
  M2.turnon();
  
  M1.faststopoff();
  M1.holdingoff();
  
  M2.faststopoff();
  M2.holdingoff();
    
  M1.setacc(100);
  M2.setacc(100);
  
  tft.begin();
  loadingScreen();
  startup();
  defaultMode();
}
int batterylvl = 100;

void loop() {
  // put your main code here, to run repeatedly:
   checkMode();
  if(millis() % 3000 < 10) {
    batterylvl = batterylvl - 10;
    if (batterylvl <= 10) {
      batterylvl = 100;
    }
    testBattery(batterylvl);
  }
//if (Serial.available() > 1) {
//    float vel = Serial.parseFloat();
//    Serial.println(testSpeed(vel));
//}
}

void loadingScreen() {
  tft.fillScreen(ILI9341_WHITE);
  tft.setCursor(10,dy*4);
  tft.setTextColor(ILI9341_BLACK);  tft.setTextSize(5);
  tft.println("LEGtrek");
  tft.setCursor(10,dy*5+7);
  tft.setTextColor(ILI9341_BLUE);  tft.setTextSize(2);
  tft.println("   Walk empowered");
  
  for(int i = 0; i <= 1; i++) {
  tft.fillCircle(dx,dy*7,10,ILI9341_BLACK);
  yield();
  delay(500);
  tft.fillCircle(2*dx,dy*7,10,ILI9341_BLACK);
  yield();
  delay(500);
  tft.fillCircle(3*dx,dy*7,10,ILI9341_BLACK);
  yield();
  delay(500);
  tft.fillCircle(4*dx,dy*7,10,ILI9341_BLACK);
  yield();
  delay(500);
  tft.fillCircle(1*dx,dy*7,10,ILI9341_WHITE);
  tft.fillCircle(2*dx,dy*7,10,ILI9341_WHITE);
  tft.fillCircle(3*dx,dy*7,10,ILI9341_WHITE);
  tft.fillCircle(4*dx,dy*7,10,ILI9341_WHITE);
  }
}

void startup() {
  tft.fillRect(0,0,xmax,4*dy,ILI9341_CYAN);
  tft.fillRect(0,4*dy,xmax,6*dy,ILI9341_BLUE);
  tft.setCursor(40,20);
  tft.setTextColor(ILI9341_BLACK);  tft.setTextSize(4);
  tft.println("SPEED");
  tft.setCursor(dx/2, 2*dy+5);
  tft.setTextColor(ILI9341_BLACK);  tft.setTextSize(4);
  tft.println(0.00);
  tft.setCursor(2.5*dx+10,2.3*dy);
  tft.setTextColor(ILI9341_BLACK);  tft.setTextSize(3);
  tft.println("mph");
  tft.setCursor(65,4*dy+10);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
  tft.println("MODE");
  tft.setCursor(40,5.5*dy+10);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
  tft.println("Default");
  testBattery(100);
}

void testBattery(uint16_t voltage) {
tft.fillRect(xmax-dx,0,dx,4*dy,ILI9341_CYAN);
tft.fillRect(xmax-dx,4*dy,dx,5*dy,ILI9341_BLUE);
if (voltage >= 90) {
tft.fillRect(xmax-dx,0,dx,dy, ILI9341_GREEN);}
if (voltage >= 80) {
tft.fillRect(xmax-dx,dy,dx,dy, ILI9341_GREEN);}
if (voltage >= 70) {
tft.fillRect(xmax-dx,2*dy,dx,dy, ILI9341_GREEN);}
if (voltage >= 60) {
tft.fillRect(xmax-dx,3*dy,dx,dy, ILI9341_ORANGE);}
if (voltage >= 50) {
tft.fillRect(xmax-dx,4*dy,dx,dy, ILI9341_ORANGE);}
if (voltage >= 40) {
tft.fillRect(xmax-dx,5*dy,dx,dy, ILI9341_ORANGE);}
if (voltage >= 30) {
  tft.fillRect(xmax-dx,6*dy,dx,dy, ILI9341_RED);}
if (voltage >= 20) {
  tft.fillRect(xmax-dx,7*dy,dx,dy, ILI9341_RED);}
if (voltage >= 10) {
  tft.fillRect(xmax-dx,8*dy,dx,dy+5, ILI9341_RED);}
tft.drawRect(xmax-dx,0,dx,ymax,ILI9341_BLACK);
}

void testSpeed(float vel) {
  if (vel > 10) {
    vel = 9.99;
  }
  if (vel < 0) {
    vel = 0.00;
  }
  tft.fillRect(0, 2*dy, 2.5*dx, 2*dy, ILI9341_CYAN);
  tft.setCursor(dx/2, 2*dy+5);
  tft.setTextColor(ILI9341_BLACK);  tft.setTextSize(4);
  tft.println(vel);
}

void checkMode(){
  valAA = digitalRead(bAA);
  valJS = digitalRead(bJS);
  valSS = digitalRead(bSS);
  valTN = digitalRead(bTN);
  
  timer = millis() - prevMillis;
  
  if (modeDef == 1 && modeAA == 0 && modeSS == 0 && modeSS == 0) {
    if (valAA == LOW && timer > 500) {
     modeAA = 1;
     modeDef = 0; 
     writeAA();
     prevMillis = millis();
    }
    else if (valJS == LOW && timer > 500) {
     modeJS = 1;
     modeDef = 0; 
     writeJS();
     prevMillis = millis();
    }
    else if (valSS == LOW && timer > 500) {
     modeSS = 1;
     modeDef = 0; 
     writeSS();
     prevMillis = millis();
    }
    else {
      modeDef = 1;
    }
  }
  else if (modeAA == 1 && modeDef == 0) {
    if (valAA == LOW && timer > 500) { //go back to def
      modeAA = 0;
      modeDef = 1;
      testSpeed(0.00);
      writeDefault();
      prevMillis = millis();
    }
  }
  else if (modeJS == 1 && modeDef == 0) {
    if (valJS == LOW && timer > 500) { //go back to def
      modeJS = 0;
      modeDef = 1;
      testSpeed(0.00);
      writeDefault();
      prevMillis = millis();
    }
  }
  else if (modeSS == 1 && modeDef == 0) {
    if (valSS == LOW && timer > 500) { //go back to def
      modeSS = 0;
      modeDef = 1;
      testSpeed(0.00);
      writeDefault();
      prevMillis = millis();
    }
  }
  if (modeAA == 1) {
    buttonBlink(lAA);
    }
  else if (modeJS == 1) {
    buttonBlink(lJS);
    readJoystick();
    timer_JS = millis() - prevMillis_JS;
    //if (timer_JS > 500) {
    if (rRead > 0.1) {
      if (angRead > PI/4 && angRead < 3*PI/4) {
        //Write FORWARD and SPEED
        if (timer_JS > 200) {
        testSpeed(rRead*1.5 - (abs(PI/2 - angRead)));
        setspeed(rRead*1.5 - (abs(PI/2 - angRead)),1);
        prevMillis_JS = millis();
        }
        writeTurnMode(1);
        lastJS = 1;
      }
      else if (angRead > -PI/8 && angRead < PI/8) {
        //Write Turn R
        writeTurnMode(2);
        turnRight();
        lastJS = 2;
      }
      else if (angRead > (PI-(PI/8)) && angRead < (PI + (PI/8))) {
        //Write Turn L
        writeTurnMode(3);
        turnLeft();
        lastJS = 3;
      }
      else {
        //Write Braked
        //SPEED = 0
        writeTurnMode(0);
        if (lastJS != 0) {
        testSpeed(0.00);
        setspeed(0,1);
        lastJS = 0;
        }
      }
    }
    else {
      //Write Braked
      if (lastJS != 0) {
      testSpeed(0.00);
      setspeed(0,1);
      lastJS = 0;
      }
      writeTurnMode(0);
    }
    //prevMillis_JS = millis();
    //}
  }
  else if (modeSS == 1) {
    buttonBlink(lSS);
    if (modeTN == 0) { //SS but not TUURN
       if (valTN == LOW && timer > 500) {
          modeTN = 1;
          testSpeed(0.00);
          prevMillis = millis();
       }
       SettingSpeed();
     }
     else if(modeTN == 1) { //SS but TURN
       buttonBlink(lTN);
      if (valTN == LOW && timer > 500) {
          modeTN = 0;
          digitalWrite(lTN,LOW);
          tft.fillRect(0,7.5*dy,4*dx,3*dy,ILI9341_BLUE);
          prevMillis = millis();
       }
       readJoystick();
      if (rRead > 0.1) {
        if (angRead > -PI/4 && angRead < PI/4) {
          
          //Write Turn R
          writeTurnMode(2);
          turnRight();
          lastJS = 2;
        }
        else if (angRead > (PI-(PI/4)) && angRead < (PI + (PI/4))) {
          //Write Turn L
          writeTurnMode(3);
          turnLeft();
          lastJS = 3;
        }
      else {
        //Write Braked
        //SPEED = 0
        writeTurnMode(0);
        if (lastJS != 0) {
          testSpeed(0.00);
          setspeed(0,1);
          lastJS = 0;
          }
        }
      }
      else {
        //Write Braked
        if (lastJS != 0) {
        testSpeed(0.00);
        setspeed(0,1);
        lastJS = 0;
        }
        writeTurnMode(0);
      }
     }
  }
  else {
    digitalWrite(lAA,LOW);
    digitalWrite(lJS,LOW);
    digitalWrite(lSS,LOW);
    digitalWrite(lTN,LOW);
    defaultMode();
    modeTN = 0;
    SetSpeed = 0.00;
    tft.fillRect(0,7.5*dy,4*dx,3*dy,ILI9341_BLUE);
    //estSpeed(0.00);
    }
  }


void defaultMode() {
  setspeed(0,1);
}

int preVar = 0;
void writeTurnMode(int var) {
  if (var != preVar) {
      tft.fillRect(0,7.5*dy,4*dx,3*dy,ILI9341_BLUE);
  switch(var) {
    case 1: 
      tft.setCursor(35,7.5*dy+10);
      tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
      tft.println("Forward"); 
      break;
    case 2:
      tft.setCursor(10,7.5*dy+10);
      tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
      tft.println("Turn Right"); 
      break;
    case 3:
      tft.setCursor(15,7.5*dy+10);
      tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
      tft.println("Turn Left"); 
      break;
    default:
      tft.setCursor(50,7.5*dy+10);
      tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
      tft.println("Braked"); 
      break;
  }
  }
  preVar = var;
}

void writeDefault() {
  tft.fillRect(0,5.5*dy,4*dx,3*dy,ILI9341_BLUE);
  tft.setCursor(40,5.5*dy+10);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
  tft.println("Default"); 
}

void writeAA() {
  tft.fillRect(0,5.5*dy,4*dx,3*dy,ILI9341_BLUE);
  tft.setCursor(45,5.5*dy+10);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
  tft.println("Active");
  tft.setCursor(45,6.5*dy+10);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
  tft.println("Assist");
  //write Active Assist
}

void writeJS() {
  tft.fillRect(0,5.5*dy,4*dx,3*dy,ILI9341_BLUE);
  tft.setCursor(30,5.5*dy+10);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
  tft.println("Joystick");
}

void writeSS() {
  tft.fillRect(0,5.5*dy,4*dx,3*dy,ILI9341_BLUE);
  tft.setCursor(65,5.5*dy+10);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
  tft.println("Set");
  tft.setCursor(50,6.5*dy+10);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
  tft.println("Speed");
  //write Active Assist
}

void buttonBlink(int whichLED) {
  if (millis() % 2000 < 1000) {
      digitalWrite(whichLED,HIGH);
    }
    else {
      digitalWrite(whichLED,LOW);
    }
}

uint16_t prevSpeed = 0.00;
void SettingSpeed() {
    valUP = digitalRead(bUP);
    valDN = digitalRead(bDN);
    timer_SS = millis() - prevMillis_SS;
    if (valUP == LOW && timer_SS > 500) {
      if (SetSpeed < maxSpeed) {
        SetSpeed = SetSpeed + 0.1;
      }
      else {
        SetSpeed = maxSpeed;
      }
      prevMillis_SS = millis();
      testSpeed(SetSpeed);
      //set wheel speed
    }
    else if (valDN == LOW && timer_SS > 500) {
      if (SetSpeed > minSpeed) { 
        SetSpeed = SetSpeed - 0.1;
      }
      else {
        SetSpeed = 0;
      }
      prevMillis_SS = millis();
      testSpeed(SetSpeed);      
    }
    //digitalWrite(lAA,!valUP);
    //digitalWrite(lJS,!valDN);
    if(SetSpeed != prevSpeed) {
      setspeed(SetSpeed,1); //if 1 is forward
      prevSpeed = SetSpeed;
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
