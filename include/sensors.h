#ifndef SENSORS_H
#define SENSORS_H
#include "a0_data.h"

// Function declarations
void getAllData();
int analogReadAverage(byte pin);
void readAllSensors();
#if (USE_PLOTS == 1)
void plotTick();
void scrollPlot(int vals[6][15], int newVal[6]);
#endif


#endif // SENSORS_H
