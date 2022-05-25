#include <Servo.h>
#define SERVO_NUMBER 4

Servo myservo;
Servo myservo2;
Servo myservo3;
Servo myservo4;

int pos = 45;
int positions[SERVO_NUMBER];
int maxPos[SERVO_NUMBER] = {180,180,180,180};
int minPos[SERVO_NUMBER] = {0,0,0,0};

bool isServoGoingUp = true;

void setTheServos(){
  myservo.write(180-positions[0]);
  myservo2.write(180-positions[1]);
  myservo3.write(180-positions[2]);
  myservo4.write(180-positions[3]);
}

void setup() {
  Serial.begin(9600);
  myservo.attach(9);
  myservo2.attach(6);
  myservo3.attach(5);
  myservo4.attach(3);

  for(int i=0;i<SERVO_NUMBER;i++){
    positions[i] = pos;
  }
}

void loop() {
  pos = 90;
  //set positions
  for(int i=0;i<SERVO_NUMBER;i++){
    positions[i] = pos;
  }
  //check if pos aint too big or small
  for(int i=0;i<SERVO_NUMBER;i++){
    positions[i] = constrain(positions[i],minPos[i],maxPos[i]);
  }

  setTheServos();
  delay (100);
}