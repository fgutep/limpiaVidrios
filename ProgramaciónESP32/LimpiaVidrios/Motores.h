#ifndef MOTORES_H
#define MOTORES_H

#include <Arduino.h>

class Motores {
  private:
    int motorAIn1, motorAIn2, motorBIn1, motorBIn2, eep;

  public:
    Motores(int a_in1, int a_in2, int b_in1, int b_in2, int eep);
    
    void setup();
    void lockMotors();
    void moveForward();
    void moveBackward();
    void turnLeft();
    void turnRight();
};

#endif
