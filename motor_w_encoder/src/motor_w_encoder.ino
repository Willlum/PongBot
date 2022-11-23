#include <Arduino.h>
#include <Encoder.h>

#define MAX_SPEED 255

// Motor connections
int enA = 9;
int in1 = 8;
int in2 = 7;
int limASwitchPower = 12;
int limASwitch = 11;
int limBSwitchPower = 5;
int limBSwitch = 6;
int ENCA = 3;
int ENCB = 2;
Encoder gantryMotor(ENCA,ENCB);
volatile int motorPosition = 0;
int desPosition = 0;
int prevMotorPosition = 0;
const byte numChars = 8;
char receivedChars[numChars];
int railLength = 0;
int maxPosition;
char strBuf[40];
int controlSignal = 0;

void setup() {
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(limASwitchPower, OUTPUT);
  pinMode(limASwitch, INPUT);
  pinMode(limBSwitchPower, OUTPUT);
  pinMode(limBSwitch, INPUT);
  // Turn off motors - Initial state
  directionControl(0);
  digitalWrite(limASwitchPower, HIGH);
  digitalWrite(limBSwitchPower, HIGH);
  speedControl(MAX_SPEED/4);
  Serial.begin(19200);
  calRailLength();
  speedControl(192);
}

void loop() {

  if(Serial.available() > 0){
    serialRecieve();
    float imageToMotorPos = (float)((atoi(receivedChars)));
    if(imageToMotorPos > 0) imageToMotorPos /= 640.0;
    else homeMotor();
    desPosition = (int)(imageToMotorPos * railLength);
    delay(1);
  }

  motorPosition = gantryMotor.read();
 
  if (motorPosition != desPosition) {
    //P controller
    controlSignal = abs(motorPosition - desPosition) + 64;
    if(controlSignal > 255) controlSignal = 255;
    speedControl(controlSignal);
    
    if (desPosition > motorPosition && motorPosition < railLength){
      directionControl(-1);
    }
    else if(desPosition < motorPosition && motorPosition > 0){
      directionControl(1);
    }
  }
  else {
      directionControl(0);
  }
  prevMotorPosition = motorPosition;
  //delay(10);
}

void directionControl(int dir) {
  if (dir == 1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    delay(1);
    // Turn on motor A & B
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    delay(1);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void speedControl(int speed) {
  analogWrite(enA, speed);
}

void homeMotor(){
  //Limit switches are normally closed
  if(digitalRead(limASwitch)){
    //Serial.println("HOMING...");
    speedControl(MAX_SPEED/5);
    directionControl(1);
    while(digitalRead(limASwitch));
    directionControl(0);
    delay(100);
    directionControl(1);
    while(digitalRead(limASwitch));
    directionControl(0);
    delay(100);
    //Serial.println("I'm HOME");
  }
  gantryMotor.write(0);
 }

void calRailLength(){
 homeMotor();
 directionControl(-1);
 while(digitalRead(limBSwitch));
 directionControl(0);
 railLength = gantryMotor.read();
 //sprintf(strBuf, "Max Distance: %d", railLength);
 //Serial.println(strBuf);
 homeMotor();
}

int serialRecieve(){
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    boolean newData = false;
    
    while (Serial.available() > 0 && newData == false) {
      rc = Serial.read();

      if (recvInProgress == true) {
          if (rc != endMarker) {
              receivedChars[ndx] = rc;
              ndx++;
              if (ndx >= numChars) {
                  ndx = numChars - 1;
              }
          }
          else {
              receivedChars[ndx] = '\0'; // terminate the string
              recvInProgress = false;
              ndx = 0;
              newData = true;
          }
      }

      else if (rc == startMarker) {
          recvInProgress = true;
      }
    }
    return 1;
}

// float computePID(){
//   //PID Control for motor
//   long currT = micros();
//   float deltaT = ((float) (currT - prevT))/( 1.0e6 );
//   prevT = currT;
//   err = motorPosition - desPosition;
//   dedt = (err-eprev)/(deltaT);
//   eintegral = eintegral + err*deltaT;
//   return (float)(kp*err + kd*dedt + ki*eintegral);
// }
