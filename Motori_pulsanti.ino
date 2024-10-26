#include <TimedAction.h>
#include <SoftwareSerial.h> 

#define rightMotor1 5 //motore di sx
#define leftMotor1 6 //motore di sx
#define rightMotor2 10 //motore di dx
#define leftMotor2 11 //motore di dx

#define freccia_sx 3
#define led_stop 7
#define RXpin 9
#define TXpin 8
#define freccia_dx 12

#define joystick_X A0
#define joystick_Y A1

#define moveVelocity 200  //PWM: 20 per test, 200 per sedia
#define moveSpin 40
#define intermLedsTotal 5
#define motorsThreadDuration 3000
#define intermittentLedsDuration 700
#define numberOfFlashes 2

SoftwareSerial bluetooth(RXpin, TXpin);

long time1, time2;
long interval = 800;
boolean pari = false;

String command;
char c;

void setup() {
  pinMode(rightMotor1, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(led_stop, OUTPUT);
  pinMode(freccia_dx, OUTPUT);
  pinMode(freccia_sx, OUTPUT);
  pinMode(joystick_X,INPUT_PULLUP);
  pinMode(joystick_Y,INPUT_PULLUP);

  Serial.begin(9600);
  bluetooth.begin(9600);

  digitalWrite(led_stop, LOW);
  time1=0.0;
  time2=0.0;
}

void moveMotors(int speed, bool isForw) {
  if (speed > 0) {
    digitalWrite(led_stop, HIGH);
    if (isForw) { //Indietro
      analogWrite(rightMotor1, speed); //sx
      analogWrite(leftMotor1, 0);
      analogWrite(rightMotor2, 0); //dx
      analogWrite(leftMotor2, speed);
    } else{
      analogWrite(rightMotor1, 0); //sx
      analogWrite(leftMotor1, speed);
      analogWrite(rightMotor2, speed); //dx
      analogWrite(leftMotor2, 0);
    }
    delay(1500);
    analogWrite(rightMotor1, 0); //sx
    analogWrite(leftMotor1, 0);
    analogWrite(rightMotor2, 0); //dx
    analogWrite(leftMotor2, 0);
  }
}

void spinToLeft() {
//   digitalWrite(freccia_sx, HIGH);
  digitalWrite(led_stop, HIGH);
  analogWrite(rightMotor1, 0); //sx
  analogWrite(leftMotor1, moveVelocity);
  analogWrite(rightMotor2, 0); //dx
  analogWrite(leftMotor2, moveVelocity);
  delay(1000);
  analogWrite(rightMotor1, 0); //sx
  analogWrite(leftMotor1, 0);
  analogWrite(rightMotor2, 0); //dx
  analogWrite(leftMotor2, 0);
}

void spinToRight() {
  digitalWrite(led_stop, HIGH);
  analogWrite(rightMotor1, moveVelocity); //sx
  analogWrite(leftMotor1, 0);
  analogWrite(rightMotor2, moveVelocity); //dx
  analogWrite(leftMotor2, 0);
  delay(1000);
  analogWrite(rightMotor1, 0); //sx
  analogWrite(leftMotor1, 0);
  analogWrite(rightMotor2, 0); //dx
  analogWrite(leftMotor2, 0);
}

void turnOnOrangeArrows() {
    time1=millis();
    if((time1-time2) >= interval) {
        if(pari) {
            digitalWrite(freccia_dx, HIGH);
            digitalWrite(freccia_sx, HIGH);
        } else {
            digitalWrite(freccia_dx, LOW);
            digitalWrite(freccia_sx, LOW);
        }
        time2=time1;
        Serial.println("time1= " + String(time1));
        Serial.println("time2= " + String(time2));
    }
    pari = not(pari);
}

TimedAction spinToRightThread = TimedAction(motorsThreadDuration, spinToRight);
TimedAction spinToLeftThread = TimedAction(motorsThreadDuration, spinToLeft);

void loop() {
  delay(10);
  if (bluetooth.available()) {
    c = bluetooth.read();
    command += c;
  } else {
    Serial.print("\nComando: " + command);
    Serial.print("\nx= "+String(analogRead(joystick_X)));
    Serial.print("\ny= "+String(analogRead(joystick_Y)));

    //joystick INDIETRO
    if (analogRead(joystick_Y) >= 1000) { 
      Serial.print(String(analogRead(joystick_Y)));
      digitalWrite(freccia_dx, HIGH);
      digitalWrite(freccia_sx, HIGH);
      while ((analogRead(joystick_Y))>=1000){
        moveMotors(moveVelocity, false);
        turnOnOrangeArrows();
      }
      digitalWrite(freccia_dx, LOW);
      digitalWrite(freccia_sx, LOW);
      
    //Bluetooth AVANTI
    } else if (command == "avanti") { 
      Serial.print(" AVANTI (BT)");
      moveMotors(moveVelocity, true);

    //joystick AVANTI
    } else if(analogRead(joystick_Y) < 50) { 
      Serial.print(String(analogRead(joystick_Y)));
      while ((analogRead(joystick_Y))<50) {
        moveMotors(moveVelocity, true);
      }

    //Bluetooth INDIETRO
    } else if (command == "indietro") { 
      Serial.print(" INDIETRO (BT)");
      for(int i = 0; i < numberOfFlashes; i++){
        digitalWrite(freccia_dx, HIGH);
        digitalWrite(freccia_sx, HIGH);
        delay(500);
        digitalWrite(freccia_dx, LOW);
        digitalWrite(freccia_sx, LOW);
        delay(500);
      }
      moveMotors(moveVelocity, false);

    //joystick DESTRA
    } else if (analogRead(joystick_X) > 1000) {
      Serial.print(" DESTRA (JS)");
      while (analogRead(joystick_X)>1000) {
        digitalWrite(freccia_dx, HIGH);
        digitalWrite(led_stop, HIGH);
        analogWrite(rightMotor1, moveVelocity); //sx
        analogWrite(leftMotor1, 0);
        analogWrite(rightMotor2, moveVelocity); //dx
        analogWrite(leftMotor2, 0);
      }
      digitalWrite(freccia_dx, LOW);
      digitalWrite(led_stop, LOW);
      analogWrite(rightMotor1, 0); //sx
      analogWrite(leftMotor1, 0);
      analogWrite(rightMotor2, 0); //dx
      analogWrite(leftMotor2, 0);

    //Bluetooth DESTRA
    } else if (command == "destra") {
      Serial.print(" DESTRA (BT)");
      for(int i = 0; i < numberOfFlashes; i++){
        digitalWrite(freccia_dx, HIGH);
        delay(500);
        digitalWrite(freccia_dx, LOW);
        delay(500);
      }
      spinToRight();

    //joystick SINISTRA
    } else if (analogRead(joystick_X) < 50) {
      Serial.print(" SINISTRA (JS)");
      while (analogRead(joystick_X) < 50) {
        digitalWrite(freccia_sx, HIGH);
        digitalWrite(led_stop, HIGH);
        analogWrite(rightMotor1, 0); //sx
        analogWrite(leftMotor1, moveVelocity);
        analogWrite(rightMotor2, 0); //dx
        analogWrite(leftMotor2, moveVelocity);
      }
      digitalWrite(freccia_sx, LOW);
      digitalWrite(led_stop, LOW);
      analogWrite(rightMotor1, 0); //sx
      analogWrite(leftMotor1, 0);
      analogWrite(rightMotor2, 0); //dx
      analogWrite(leftMotor2, 0);

    //Bluetooth SINISTRA
    } else if (command == "sinistra") {
      Serial.print(" SINISTRA (BT)");
      for(int i = 0; i < numberOfFlashes; i++){
        digitalWrite(freccia_sx, HIGH);
        delay(500);
        digitalWrite(freccia_sx, LOW);
        delay(500);
      }
      spinToLeft();

    //STOP
    } else { 
      Serial.print(" STOP");
      digitalWrite(led_stop, LOW);
      digitalWrite(freccia_dx, LOW);
      digitalWrite(freccia_sx, LOW);
      moveMotors(0, false);
      delay(1000);
    }
    command = "";
  }
}
