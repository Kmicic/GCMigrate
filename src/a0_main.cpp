#include <Arduino.h>

#include "a0_data.h"



boolean backlState = true;  




#if (START_MENU == 1)
bool startService = false;
#endif

#if (USE_PLOTS == 1 || USE_PID == 1 || USE_DAWN == 1)

const char row8[] PROGMEM = {0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111};
const char row7[] PROGMEM = {0b00000,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111};
const char row6[] PROGMEM = {0b00000,  0b00000,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111};
const char row5[] PROGMEM = {0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111};
const char row4[] PROGMEM = {0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111,  0b11111};
const char row3[] PROGMEM = {0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111};
const char row2[] PROGMEM = {0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111};
const char row1[] PROGMEM = {0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111};
#endif


#if (USE_CO2 == 1)
#if (CO2_PIN == 1)
#define CO2_PIN_NAME SENS_1
#elif (CO2_PIN == 2)
#define CO2_PIN_NAME SENS_2
#else
#error "Wrong CO2 pin"
#endif

int CO2ppm = 0;
bool CO2_flag = false;
bool CO2_rst = false;

void CO2tick() {
  static uint32_t tmr;
  if (millis() - settingsTimer > 1000) {
    if (digitalRead(CO2_PIN_NAME)) {
      if (!CO2_flag) {
        tmr = millis();
        CO2_flag = true;
      }
    } else {
      if (CO2_flag) {
        
        if (!CO2_rst) {
          tmr = millis() - tmr;
          CO2ppm = (CO2_MAX / 1000) * (tmr - 2);
        }
        CO2_flag = false;
        CO2_rst = false;
      }
    }
  }
}
#endif

const char *settingsPageNames[]  = {
  "A-BKL",
  "BKL-time",
  "Drv",
  "Day",
  "Month",
  "Year",
  "Sens prd",
  "Plot prd",
#if (SMOOTH_SERVO == 1)
  "S1 sp",
  "S1 acc",
  "S2 sp",
  "S2 acc",
#endif
#if (PID_AUTOTUNE == 1)
  "Tuner",
  "Result",
  "Channel",
  "Sensor",
  "Manual",
  "Steady",
  "Step",
  "Window",
  "Kick time",
  "Delay",
  "Period",
#endif
};

#if (USE_PID == 1)
const char *pidNames[] = {
  "P",
  "I",
  "D",
  "Sens",
  "Set",
  "T",
  "Min",
  "Max",
};
#endif
#if (USE_DAWN == 1)
const char *dawnNames[]  = {
  "Start",
  "Dur. up",
  "Stop",
  "Dur. down",
  "Min",
  "Max",
};
#endif

const char *channelNames[] = {
  "Channel 1",
  "Channel 2",
  "Channel 3",
  "Channel 4",
  "Channel 5",
  "Channel 6",
  "Channel 7",
  "Servo 1",
  "Servo 2",
  "Drive",
};

const char *settingsNames[]  = {
  "Mode",
  "Direction",
  "Type",

  "Mode",
  "Direction",
  "Limits",

  "Mode",
  "Direct.",
  "Timeout",
};

const char *modeNames[]  = {
  "<Timer>",
  "<Timer RTC>",
  "<Week>",
  "<Sensor>",
  "<PID>",
  "<Dawn>",
};

const char *relayNames[]  = {
  "Relay",
  "Valve",
  "Common",
};

const char *modeSettingsNames[]  = {
  "Period",   // 0
  "Work",     // 1
  "Left",     // 2

  "Period",   // 3
  "Work",     // 4
  "Start hour", // 5

  "",    // 6
  "",     // 7

  "Period",   // 8
  "Sensor",   // 9
  //"Threshold",  // 10
};


const char *directionNames[]  = {
  "Off-On",
  "On-Off",
  "Min-Max",
  "Max-Min",
  "Close-Open",
  "Open-Close",
};

const uint8_t daysMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 0};

const char *sensorNames[]  = {
  "AirT",
  "AirH",

  SENS1_NAME,
  SENS2_NAME,
  SENS3_NAME,
  SENS4_NAME,
  "AirP"
};


byte curPWMchannel = 0; 
byte curSetMode = 0;   

boolean startPID = false;
float uptime = 0;
int8_t lastScreen = 0;
uint16_t thisDay = 0;
const int PWMperiod = (float)1000 / PWM_RELAY_HZ;

LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);
encMinim enc(CLK, DT, SW, true, false);
boolean controlState = false;  // true - control, false - read

// Global sensor objects
MicroDS3231 rtc;

#if (USE_BMP280 == 1)
BMP280 bmp280;
#endif

#if (USE_AHT20 == 1)
AHT20 aht20;
#endif

// Global variables - system state
long timerMillis[10];
uint32_t driveTimer = 0;
byte driveState = 0;
boolean lastDriveState = false;
boolean manualControl = false;
boolean manualPos = false;

// Global variables - sensor data
float sensorVals[8];
int8_t realTime[3];
byte servoPosServ[2];
int PWMactive[2];

// Global variables - PID and drive
int pwmVal[7];
boolean drivePidFlag = false;

// Global variables - channel states
boolean channelStates[10];
boolean channelStatesServ[10];
int8_t debugPage = 0;

// Global variables - UI state
int8_t arrowPos = 0;
int8_t navDepth = 0;
int8_t currentChannel = 0;
int8_t currentMode = 0;
int8_t thisH[2], thisM[2], thisS[2];
int8_t currentLine = 0;
uint32_t commonTimer = 0, backlTimer = 0, plotTimer = 0;

// Global variables - sensor history
int sensMinute[6][15];

// Global variables - flags and timers
boolean serviceFlag = false;
boolean timeChanged = false;
boolean startFlagDawn = false;
uint32_t settingsTimer = 0;
uint32_t driveTout = 0;

// Global variables - mode
byte thisMode = 0;
byte curMode = 0;  

