/*  Arduino Robotic Arm with Grab
 *  More info: http://www.ardumotive.com/
 *  Dev: Michalis Vasilakis Data: 12/12/2016 Ver: 1.1 */

#include <Servo.h>

//Create servo objects to control servo motors
Servo up_down;  
Servo forward_backward;
Servo rotate;
Servo grab;

//Constants
const int startBT = 7;
const int teachBT = 8;
const int buzzer = 9;
const int potForwardBackward = A3;
const int potUpDown = A2;
const int potRotate = A1;
const int potGrab = A0;
//Max and Min values for servos ! Change them to meet your setup !
const int minGrab=100;
const int maxGrab=179;
const int minRotate=10;
const int maxRotate=100;
const int minUpDown=10;
const int maxUpDown=100;
const int minForwardBackward=10;
const int maxForwardBackward=160;
//-------------------------------------//
//Variables
int readUpDown,readForwardBackward,readRotate,readGrab,readTeach,readStart;
int teachUpDown[100],teachForwardBackward[100],teachRotate[100],teachGrab[100];
boolean started =false;
int index = 1;
int stepSpeed = 20; //Change this to fo faster!

void setup() {
  Serial.begin(115200);
  //Attach Servo motors
  forward_backward.attach(3);
  up_down.attach(4);
  rotate.attach(5); //portokali xontro
  grab.attach(6);  
  //Inputs-Outputs
  pinMode(teachBT, INPUT_PULLUP);
  pinMode(startBT, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);
  //Do a smooth movement on startup, from home potion to pot position:
  readInputs();
  goHome();
}

void loop() {
  if (!started){
    readInputs();
    delay(50);
    moveMotores();
    if (readTeach==LOW){
      savePosition();
      tone(buzzer,500);
      delay(500);
      noTone(buzzer);
    }
    if (readStart==LOW){
      tone(buzzer,700);
      started=true;
      delay(1000);
      noTone(buzzer);
    }
  }
  else{
    goHome();
    runTeach();
  }
}

void readInputs(){
  //Read potentiometers
  readUpDown = analogRead(potUpDown);
  readUpDown = map(readUpDown,0,1023,minUpDown,maxUpDown);
  Serial.println(readUpDown);
  readForwardBackward = analogRead(potForwardBackward);
  readForwardBackward = map(readForwardBackward,0,1023,minForwardBackward,maxForwardBackward);
  readRotate = analogRead(potRotate);
  readRotate = map(readRotate,0,1023,minRotate,maxRotate);
  readGrab = analogRead(potGrab);
  readGrab = map(readGrab,0,1023,minGrab,maxGrab);
  //Read buttons
  readTeach = digitalRead(teachBT);
  readStart = digitalRead(startBT);
}
void moveMotores(){
  up_down.write(readUpDown);
  forward_backward.write(readForwardBackward);
  rotate.write(readRotate);
  grab.write(readGrab);
}

void savePosition(){
  teachUpDown[index] = readUpDown;
  teachForwardBackward[index] = readForwardBackward;
  teachRotate[index] = readRotate;
  teachGrab[index] = readGrab;
  index++;
}

void runTeach(){
  for (int i=0; i<index-1; i++){
    if (teachRotate[i] < teachRotate[i+1]){
      for (int j = teachRotate[i]; j<= teachRotate[i+1]; j++){
        rotate.write(j);
        delay(stepSpeed);
      }
    }
    else if (teachRotate[i] > teachRotate[i+1]){
      for (int j = teachRotate[i]; j>= teachRotate[i+1]; j--){
        rotate.write(j);
        delay(stepSpeed);
      }  
    }
    else{
      rotate.write(teachRotate[i]);
    }
    if (teachGrab[i] < teachGrab[i+1]){
      for (int j = teachGrab[i]; j<= teachGrab[i+1]; j++){
        grab.write(j);
        delay(stepSpeed);
      }
    }
    else if (teachGrab[i] > teachGrab[i+1]){
      for (int j = teachGrab[i]; j>= teachGrab[i+1]; j--){
        grab.write(j);
        delay(stepSpeed);
      } 
    }
    else{
      grab.write(teachGrab[i]);
    }
    if (teachForwardBackward[i] < teachForwardBackward[i+1]){
      for (int j = teachForwardBackward[i]; j<= teachForwardBackward[i+1]; j++){
        forward_backward.write(j);
        delay(stepSpeed);
      }
    }
    else if (teachForwardBackward[i] > teachForwardBackward[i+1]){
      for (int j = teachForwardBackward[i]; j>= teachForwardBackward[i+1]; j--){
        forward_backward.write(j);
        delay(stepSpeed);
      }
    }
    else{
      forward_backward.write(teachForwardBackward[i]);
    }
    if (teachUpDown[i] < teachUpDown[i+1]){
      for (int j = teachUpDown[i]; j<= teachUpDown[i+1]; j++){
        up_down.write(j);
        delay(stepSpeed);
      }
    }
    else if (teachUpDown[i] > teachUpDown[i+1]){
      for (int j = teachUpDown[i]; j>= teachUpDown[i+1]; j--){
        up_down.write(j);
        delay(stepSpeed);
      }
    }
    else{
      up_down.write(teachUpDown[i]);
    }
  }
  started=false;
}

void moveMotorSentidoHorario(Servo &motor, int anguloAtual, int anguloDesejado) {
  for (int j = anguloAtual; j <= anguloDesejado; j++) {
    motor.write(j);
    delay(stepSpeed);
  }
}

void moveMotorSentidoAntiHorario(Servo &motor, int anguloAtual, int anguloDesejado) {
  for (int j = anguloAtual; j >= anguloDesejado; j--) {
    motor.write(j);
    delay(stepSpeed);
  }
}

void moveMotor(Servo &motor, int anguloAtual, int anguloDesejado) {
  if (anguloAtual < anguloAtual) {
    moveMotorSentidoHorario(motor, anguloAtual, anguloDesejado);
  }
  else if (anguloAtual > anguloDesejado) {
    moveMotorSentidoAntiHorario(motor, anguloAtual, anguloDesejado);
  }
  else {
    motor.write(anguloAtual);
  }
}

//Change values if it's necessary...
void goHome() {
  moveMotor(forward_backward, readForwardBackward, minForwardBackward);
  moveMotor(up_down, readUpDown, minUpDown);
  moveMotor(rotate, readRotate, minRotate);
  moveMotor(grab, readGrab, minGrab);
  //Always start from home position
  teachForwardBackward[0]= minForwardBackward;
  teachUpDown[0]=minUpDown;
  teachRotate[0]=minRotate;
  teachGrab[0]=minGrab;
}
