#include "Arduino.h"
#include "Motores.h"

Motores::Motores(int a_in1, int a_in2, int b_in1, int b_in2, int eepPin)
  : motorAIn1(a_in1), motorAIn2(a_in2), motorBIn1(b_in1), motorBIn2(b_in2), eep(eepPin) {}

void Motores::setup() {
  pinMode(motorAIn1, OUTPUT);
  pinMode(motorAIn2, OUTPUT);
  pinMode(motorBIn1, OUTPUT);
  pinMode(motorBIn2, OUTPUT);
  pinMode(eep, OUTPUT);
  lockMotors();
}

void Motores::lockMotors() {
  digitalWrite(motorAIn1, LOW); 
  digitalWrite(motorAIn2, LOW);
  digitalWrite(motorBIn1, LOW);
  digitalWrite(motorBIn2, LOW);
}

void Motores::moveForward() {
  digitalWrite(motorAIn1, HIGH);
  digitalWrite(motorAIn2, LOW);
  digitalWrite(motorBIn1, HIGH);
  digitalWrite(motorBIn2, LOW);
  Serial.println("Moving forward");
}

void Motores::moveBackward() {
  digitalWrite(motorAIn1, LOW);
  digitalWrite(motorAIn2, HIGH);
  digitalWrite(motorBIn1, LOW);
  digitalWrite(motorBIn2, HIGH);
  Serial.println("Moving backward");
}

void Motores::turnLeft() {
  digitalWrite(motorAIn1, HIGH);
  digitalWrite(motorAIn2, LOW);
  digitalWrite(motorBIn1, LOW);
  digitalWrite(motorBIn2, HIGH);
  Serial.println("Turning left");
}

void Motores::turnRight() {
  digitalWrite(motorAIn1, LOW);
  digitalWrite(motorAIn2, HIGH);
  digitalWrite(motorBIn1, HIGH);
  digitalWrite(motorBIn2, LOW);
  Serial.println("Turning right");
}
