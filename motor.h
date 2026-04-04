#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

bool motorBegin();
void motorSetNormalized(float command);
void motorStop();

#endif
