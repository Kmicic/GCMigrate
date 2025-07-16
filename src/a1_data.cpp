#include "a1_data.h"

channelsStruct activeChannel, setChannel;

MicroDS3231 rtc;

#if (USE_BMP280 == 1)
#include <BMP280.h>
BMP280 bmp280;
#endif

uint16_t thisDay;

long timerMillis[10];       // milisecond counter

uint32_t driveTimer;
byte driveState;
boolean lastDriveState;
boolean manualControl;
boolean manualPos;

float sensorVals[8];    // sensors ...
int8_t realTime[3];
byte servoPosServ[2];
int PWMactive[2];

int pwmVal[7];   // drive movement time for PID
boolean drivePidFlag; // drive movement for PID flag

boolean channelStates[10];
boolean channelStatesServ[10];
int8_t debugPage;

int8_t arrowPos;  // 0-3
int8_t navDepth;  // 0-2
int8_t currentChannel;  // -3 - 14
int8_t currentMode; // 0-3
int8_t thisH[2], thisM[2], thisS[2];
int8_t currentLine;
uint32_t commonTimer, backlTimer, plotTimer;

int sensMinute[6][15];

boolean serviceFlag;
boolean timeChanged;
boolean startFlagDawn;
uint32_t settingsTimer;
uint32_t driveTout;

byte thisMode;
byte curMode;
