#include <Servo.h>
#define SERVO_NUMBER 4

Servo myservo;
Servo myservo2;
Servo myservo3;
Servo myservo4;

int positions[SERVO_NUMBER];
int maxPos[SERVO_NUMBER] = {180,180,180,180};
int minPos[SERVO_NUMBER] = {0,0,0,0};

int data1 = 90;
int data2 = 90;
int data3 = 90;
int data4 = 90;

void setTheServos(){
  myservo.write(positions[0]);
  myservo2.write(positions[1]);
  myservo3.write(positions[2]);
  myservo4.write(positions[3]);
}

void setup() {
  Serial.begin(9600);
  myservo.attach(9);
  myservo2.attach(6);
  myservo3.attach(5);
  myservo4.attach(3);
}

void loop() {
  if(Serial.available() > 0) {
    data1 = (Serial.readStringUntil('\n')).toInt();
    data2 = (Serial.readStringUntil('\n')).toInt();
    data3 = (Serial.readStringUntil('\n')).toInt();
    data4 = (Serial.readStringUntil('\n')).toInt();
    Serial.println("got the data");
  }

  
  //set positions
  //first servo
  positions[2] = data1+25;
  //second servo
  positions[1] = data2;
  //third servo
  positions[0] = data3+15;
  //base servo
  positions[3] = 180-data4;
  
  //check if pos aint too big or small
  for(int i=0;i<SERVO_NUMBER;i++){
    positions[i] = constrain(positions[i],minPos[i],maxPos[i]);
  }

  setTheServos();
  delay (100);
}
