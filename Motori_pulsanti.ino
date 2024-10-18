#include <TimedAction.h>
#include <SoftwareSerial.h>

#define led_retro 2
#define motor1 3 //left
#define dir1 4 //left
#define motor2 5 //right
#define dir2 6 //right
#define led_stop 7
#define RXpin 8
#define TXpin 9
#define led_spin 10
#define joystick_signal 11
#define btn_forw 12
#define btn_prev 13

#define joystick_X A0
#define joystick_Y A1

String command;
char c;
int moveVelocity = 70;
int moveSpin = 40;
int intermLedsTotal = 5;
int motorsThreadDuration = 3000  ;
int intermittentLedsDuration = 700;
int whiteLedStatus = 0;
int x_dir = 0;
int y_dir = 0;
SoftwareSerial bluetooth(RXpin, TXpin);

void setup() {
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT); 
  pinMode(dir1, OUTPUT); 
  pinMode(dir2, OUTPUT); 
  pinMode(btn_forw, INPUT_PULLUP);
  pinMode(btn_prev, INPUT_PULLUP);
  pinMode(led_stop, OUTPUT);
  pinMode(led_spin, OUTPUT);
  pinMode(led_retro, OUTPUT);
  pinMode(joystick_signal, INPUT_PULLUP);
  
  Serial.begin(9600);
  bluetooth.begin(9600);
}

void moveMotors(int speed, bool isForw){
  analogWrite(motor1,speed);
  analogWrite(motor2,speed);
  /*whiteLedStatus = */digitalWrite(led_retro, LOW);
  if(speed > 0){
    digitalWrite(led_stop, LOW);
    if(isForw){
      /*whiteLedStatus = */digitalWrite(led_retro, HIGH);
    }
    digitalWrite(dir1, isForw ? HIGH : LOW);
    digitalWrite(dir2, isForw ? HIGH : LOW); 
  }
}

void spinToRight(){
  analogWrite(motor1,moveSpin);
  digitalWrite(dir1, HIGH);
  delay(5000
  );
  analogWrite(motor1,0);
}

void spinToLeft(){
  analogWrite(motor2,moveSpin);
  digitalWrite(dir2, HIGH);
  delay(1000);
  analogWrite(motor2,0);
}

void interLed(){
  for(int i=0; i<intermLedsTotal; i++){
    digitalWrite(led_spin, HIGH);
    //digitalWrite(buzzer,HIGH);
    delay(500);
    digitalWrite(led_spin, LOW);
    //digitalWrite(buzzer,LOW);
    delay(500);
  }
}

/*void activeBuzzer(){
  //while(digitalRead(btn_forw) == LOW || whiteLedStatus == 1){
  for(int i = 0; i < 5; i++){
    digitalWrite(buzzer,HIGH);
    delay(500);
    digitalWrite(buzzer,LOW);
    delay(500);
  }
}*/

void readFromBT(){
  c = bluetooth.read();
  command += c;
}

TimedAction spinToRightThread = TimedAction(motorsThreadDuration, spinToRight);
TimedAction spinToLeftThread = TimedAction(motorsThreadDuration, spinToLeft);
TimedAction intermLedsThread = TimedAction(intermittentLedsDuration, interLed);

void loop() { 
  delay(10);
  if(bluetooth.available()){
    c = bluetooth.read();
    command += c;
  } else {
    x_dir = analogRead(joystick_X);
    y_dir = analogRead(joystick_Y);
    Serial.print("\nComando: " + command);
    if (digitalRead(btn_forw) == LOW){
      Serial.print("\nAVANTI");
      while (digitalRead(btn_forw) == LOW || (x_dir == 0 && y_dir > 0)){
        moveMotors(moveVelocity, false);
      }
    } else if (command == "avanti") {
      Serial.print("\nAVANTI (BT)");
      moveMotors(moveVelocity, false);
      delay(5000);   
    } else if (digitalRead(btn_prev) == LOW) {
      Serial.print("\nINDIETRO");
      while (digitalRead(btn_prev) == LOW || (x_dir == 0 && y_dir < 0)) {
        //activeBuzzer();
        moveMotors(moveVelocity, true);   
      } 
    } else if (command == "indietro") {
      Serial.print("\nINDIETRO (BT)");
      //activeBuzzer();
      moveMotors(moveVelocity, true); 
      delay(5000);  
    } else if (x_dir > 0 && y_dir == 0) {
      Serial.print("\nDESTRA (JS)");
      intermLedsThread.check();
      spinToRightThread.check();
    } else if (command == "destra") {
      Serial.print("\nDESTRA (BT)");
      intermLedsThread.check();
      spinToRightThread.check();
    } else if (x_dir < 0 && y_dir == 0) {
      Serial.print("\nSINISTRA (JS)"); 
      intermLedsThread.check();
      spinToLeftThread.check();
    } else if (command == "sinistra") {
      Serial.print("\nSINISTRA (BT)"); 
      intermLedsThread.check();
      spinToLeftThread.check();
    } else {
      Serial.print("\nSTOP");
      digitalWrite(led_stop, HIGH);
      moveMotors(0, false);
    }
    command = "";
  }
  
  
}
