#ifndef CUSTOM_H
#define CUSTOM_H    
#include <Arduino.h>
#include "a0_data.h"

boolean chkTimer (unsigned long &startTime, unsigned long loopTime);
void customSetup();
void customLoop();

#endif