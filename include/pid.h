#ifndef PID_H
#define PID_H
#include <Arduino.h>
void checkPID();
void computePID(byte channel);  // Computes the PID for a given channel

#endif // PID_H