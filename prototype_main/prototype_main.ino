#include <QUAD.h>
enum {
  STAND_STATE,
  SIT_STATE,
  WALK_STATE,
  TURN_STATE,
} typedef states;

states my_state;
states my_old_state;
String readString;
String walk_direction;
int walk_speed;
String turn_direction;
int turn_angle;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (! Serial); // Wait untilSerial is ready
  Serial.println("Enter stand/sit/walk/turn command with comma ',' ; or 'x' to clear");
  Serial.println("Expected input format:  '1stand,'/'1sit,'/'x,' or");
  Serial.println("Walk command '100walkf'/'50walkb', number are speed");
  Serial.println("Turn command '90turnclw'/'30turnccw', number are angle");
  my_state = STAND_STATE; // initialize the legs to standing configuration
  my_old_state = STAND_STATE;
}


void loop() {
  // put your main code here, to run repeatedly:
  //run state machine (function)
  state_machine();
  //  checkinput();
  delay(10);

}


void state_machine() {
  switch (my_state) {

    case STAND_STATE:
      if (my_old_state == SIT_STATE) {
        //sit2stand();
        Serial.println("Reconfig done");
      }
      else if (my_old_state == WALK_STATE) {
        //hard_halt();
        Serial.println("Stopped");
      }
      checkinput();
      my_old_state = STAND_STATE;
      break;

    case SIT_STATE:
      if (my_old_state == STAND_STATE) {
        //stand2sit();
        my_old_state = SIT_STATE;
        //        Serial.println("Reconfig done");
      }
      else if (my_old_state == SIT_STATE) {
        /// do nothing since already in sitting mode
      }
      else {
        Serial.println("Return to stand to reconfig!");
        my_old_state = STAND_STATE;
      }
      checkinput();
      break;

    case WALK_STATE:
      //can only go from stand to walk
      if (my_old_state == STAND_STATE) {
        //walk(walk_speed,walk_direction)
      }
      else {
        Serial.println("Return to stand to reconfig!");
        my_old_state = STAND_STATE;
      }
      checkinput();
      break;

    case TURN_STATE:
      //maybe: checkoldstate -> do corresponding conversion
      //stand2turn
      //sit2stand -> stand2turn

      if (my_old_state == STAND_STATE) {
        //turn(turn_angle,turn_direction);
        my_state = STAND_STATE;
      }
      else {
        Serial.println("Return to stand to reconfig!");
        my_old_state = STAND_STATE;
      }
      break;
      //      checkinput();
  }
}

void checkinput() {
  // take value from serial monitor and determine states accordingly
  // !!!all input from serial monitor must start with a number and end with a comma!!!
  if (Serial.available() > 0) { //if there's a input from serial monitor
    char c = Serial.read();  //gets one byte from serial buffer

    if (c == ',') {
      if (readString.length() > 1) {
        Serial.println(readString); //prints string to serial port out

        int n = readString.toInt();  //convert readString into a number

        if (readString.indexOf("stand") > 0) {
          my_state = STAND_STATE;
          Serial.println("standing");
        }
        else if (readString.indexOf("sit") > 0) {
          my_state = SIT_STATE;
          Serial.println("sitting");
        }
        else if (readString.indexOf("walk") > 0) {
          Serial.println("walk");
          my_state = WALK_STATE;
          walk_speed = n;
          if (readString.indexOf("f") > 0) {
            Serial.println("forward");
            walk_direction = "forward";
          }

          if (readString.indexOf('b') > 0) {
            Serial.println("backward");
            walk_direction = "backward";
          }
        }
        else if (readString.indexOf("turn") > 0) {
          Serial.println("turn");
          my_state = TURN_STATE;
          turn_angle = n;
          if (readString.indexOf("clw") > 0) {
            Serial.println("clockwise");
            walk_direction = "clockwise";
          }

          if (readString.indexOf("ccw") > 0) {
            Serial.println("counterclockwise");
            walk_direction = "counterclockwise";
          }
        }
        else {
          Serial.println("ERROR! Follow input Format!!! Previous input cleared!");
          readString = ""; //clears variable for new input
        }
        readString = ""; //clears variable for new input
      }
    }
    else {
      readString += c; //makes the string readString
    }

  }

}


