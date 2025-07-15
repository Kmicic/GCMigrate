#ifndef A1_DATA_H
#define A1_DATA_H


// -------------------- PINY ---------------------
#define SW        0
#define RELAY_0   1
#define DT        2
#define CLK       3
#define RELAY_1   4
#define RELAY_2   5
#define RELAY_3   6
#define RELAY_4   7
#define RELAY_5   8
#define RELAY_6   9
#define DRV_SIGNAL1 10
#define DRV_PWM     11
#define DRV_SIGNAL2 12
#define SERVO_0   13
#define SERVO_1   A0
#define SENS_VCC  A1
#define SENS_1    A2
#define SENS_2    A3
#define SENS_3    A6
#define SENS_4    A7
#define CO2_RX    A1

// ------------- PREPROCESOR -------------
#define DEBUG_ENABLE 1

#if (DEBUG_PID > 0)
#define DEBUG_ENABLE 1
#endif

// #if (DEBUG_ENABLE == 1)
// #include <GyverUART.h>
// #define DEBUG(x) uart.println(x)
// #else
// #define DEBUG(x)
// #endif

// force pid to be enabled when using schedule
#if (SCHEDULE_NUM > 0)
#define USE_PID 1
#endif

#if (SCHEDULE_NUM > 2)
#define SCHEDULE_NUM 2
#endif

#if (PID_AUTOTUNE == 1)
#define SMOOTH_SERVO 0
#endif

// jeśli odłączone oba serwa
#if (SERVO1_RELAY == 1 && SERVO2_RELAY == 1)
// force remove smoothness settings from menu
#define SMOOTH_SERVO 0
#endif

#define USE_BMP280 1

#define USE_AHT20 1

// odłączyć BME jeżeli włączony DHT
#if (DHT_SENS2 == 1)
#define USE_BME 0
#define USE_BMP280 0
#define USE_ 0
#endif


// maximum schedule points
#define SCHEDULE_MAX 30

// karta eeprom
#define EEPR_KEY_ADDR 1022
#define EEPR_KEY 121

#define EEPR_CH 0
#define EEPR_PID 320
#define EEPR_DAWN 460
#define EEPR_SETTINGS 502
#define EEPR_PLOT_D 524
#define EEPR_PLOT_H 704
#define EEPR_SHED 884

#define EEPR_CH_STEP 32
#define EEPR_PID_STEP 20
#define EEPR_DAWN_STEP 6
#define EEPR_SETTINGS_STEP 22
#define EEPR_SHED_STEP (6+SCHEDULE_MAX*2)

// -------------------- Biblioteki ---------------------
#include "encMinim.h"
encMinim enc(CLK, DT, SW, ENC_REVERSE, ENCODER_TYPE);

#if (SERVO1_RELAY == 0 || SERVO2_RELAY == 0)
#if (SMOOTH_SERVO == 1)
#include <ServoSmooth.h>
#else
#include <Servo.h>
#endif
#endif

#if (SERVO1_RELAY == 0)
#if (SMOOTH_SERVO == 1)
ServoSmooth servo1;
#else
Servo servo1;
#endif
#endif

#if (SERVO2_RELAY == 0)
#if (SMOOTH_SERVO == 1)
ServoSmooth servo2;
#else
Servo servo2;
#endif
#endif


#include <microWire.h>
#include <microLiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);

#include <EEPROM.h>

#include <microDS3231.h>
MicroDS3231 rtc;

// // bme
// #if (USE_BME == 1)
// #include <GyverBME280.h>
// GyverBME280 bme;
// #endif

// #if (USE_ADAFRUIT_SENSOR == 1)

// #include <Adafruit_Sensor.h>
// #include <Adafruit_AHTX0.h>
// #include <Adafruit_BMP280.h>

// // Create sensor instances
// Adafruit_AHTX0 aht;
// Adafruit_BMP280 bmp; // Default I2C address is 0x77

// #endif

#if (DALLAS_SENS1 == 1)
#include <microDS18B20.h>
#if (DALLAS_AMOUNT > 1)
MicroDS18B20 dallas[DALLAS_AMOUNT];
float dallasBuf[DALLAS_AMOUNT];
#else
MicroDS18B20 dallas(SENS_1);
#endif
#endif

#if (DHT_SENS2 == 1)
#include <DHT.h>
DHT dht(SENS_2, DHT_TYPE);
#endif

// #if (USE_HTU21D == 1)
// #include <microHTU21D.h>
// HTU21D myHTU21D(HTU21D_RES_RH12_TEMP14);
// #endif

#if (USE_BMP280 == 1)
#include <BMP280.h>
//#include <Adafruit_BMP280.h>
BMP280 bmp280;
#endif

#if (USE_AHT20 == 1)
#include <AHT20.h>
AHT20 aht20;
#endif

#if (WDT_ENABLE == 1)
#include <avr/wdt.h>
#endif
// -------------------- ПЕРЕМЕННЫЕ ---------------------
int8_t lastScreen = 0;

// struktura shedule
#if (SCHEDULE_NUM > 0)
struct scheduleStruct {
  byte pidChannel = 0;    // select canal
  int startDay = 1;  // day
  int endDay = 1;    // month
  byte pointAmount = 15;  // number points
  int setpoints[SCHEDULE_MAX];  // table of sheduler
};
scheduleStruct setSchedule, activeSchedule;
#define loadSchedule(x) scheduleStruct(EEPROM.get((x) * EEPR_SHED_STEP + EEPR_SHED, activeSchedule))
//6+SCHEDULE_NUM*2

const char *schedulePageNames[] = {
  "Channel",
  "Start",
  "End",
  "Amount",
};
#endif

#if (USE_CO2 == 1 && CO2_CALIB == 0)
uint16_t _tx_delay;
uint8_t *_tx_pin_reg;
uint8_t _tx_pin_mask;
#endif

// dawn
#define DAWN_SET_AMOUNT 6
#if (USE_DAWN == 1)
struct dawnStruct {
  int8_t start = 0;
  int8_t stop = 0;
  uint8_t dur1 = 30;
  uint8_t dur2 = 30;
  uint8_t minV = 0;
  uint8_t maxV = 255;
  // 6
};
dawnStruct setDawn, activeDawn;
#define loadDawn(x) dawnStruct(EEPROM.get((x) * EEPR_DAWN_STEP + EEPR_DAWN, activeDawn))
#endif

// PID
#define PID_CH_AMOUNT 7
#define PID_SET_AMOUNT 8
#if (USE_PID == 1)
struct PIDstruct {
  int8_t sensor = 0;      // sensor canal
  float kP = 0.0;         // P
  float kI = 0.0;         // I
  float kD = 0.0;         // D
  byte dT = 1;            // iterations time
  byte minSignal = 0;     // min signal
  byte maxSignal = 200;   // max signal
  float setpoint = 20;    // set vallue
  // 20
};
PIDstruct activePID, setPID;

#define loadPID(x) PIDstruct(EEPROM.get((x) * EEPR_PID_STEP + EEPR_PID, activePID))
#define savePID(x) (EEPROM.put((x) * EEPR_PID_STEP + EEPR_PID, activePID))

float integralSum[PID_CH_AMOUNT];
float prevInput[PID_CH_AMOUNT];
float input[PID_CH_AMOUNT];
int output[PID_CH_AMOUNT];
uint32_t PIDtimers[PID_CH_AMOUNT];
#endif

#if (USE_PID_RELAY == 1)
uint32_t tmr0, tmr1;
bool flag0, flag1;
#endif

// Settings
#define SETTINGS_AMOUNT 8
#if (SMOOTH_SERVO == 1)
#define SETTINGS_AMOUNT 12
#endif
#if (PID_AUTOTUNE == 1)
#define SETTINGS_AMOUNT 19
#endif

#if (PID_AUTOTUNE == 1)
struct {
  bool tuner = false;
  bool restart = true;
  bool result = false;
  byte channel = 0;
  byte sensor = 0;
  bool manual = false;
  byte steady = 50;
  byte step = 25;
  float window = 0.1;
  byte kickTime = 30;
  byte delay = 20;
  byte period = 1;
} tunerSettings;

struct {
  byte status = 0;
  uint32_t cycle = 0;
  float min = 0;
  float max = 0;
  float P = 0;
  float I = 0;
  float D = 0;
  byte value = 0;
  float input = 0;
} tuner;

const char *tuneNames[] = {
  "off",
  "wait steady",
  "wait kick",
  "step up",
  "step down",
};
#endif

struct {
  boolean backlight = 1;    // auto-off display backlight after idle timeout (1 - enable)
  byte backlTime = 60;      // Display timeout, seconds
  byte drvSpeed = 125;      // drive speed, 0-255
  byte srv1_Speed = 40;     // max servo1 speed, 0-255
  byte srv2_Speed = 40;     // max speed servo 2, 0-255
  float srv1_Acc = 0.2;     // servo acceleration 1, 0.0-1.0
  float srv2_Acc = 0.2;     // servo acceleration 2, 0.0-1.0
  int16_t comSensPeriod = 1;
  int8_t plotMode = 0;
  byte minAngle[2] = {0, 0};
  byte maxAngle[2] = {180, 180};
  int16_t driveTimeout = 50;
} settings; //21

// Channels
struct channelsStruct {
  boolean type = 0;
  boolean state = 0;          // channel on/off)
  boolean direction = true;   // direction
  boolean global = false;     // global day mode
  int8_t week = 0;            // week
  int8_t sensor;              // sensor type (air temp, air hum, mois1...)
  int8_t relayType;           // relais type (pump, valve, relais)
  int8_t mode;                // work mode (timer, rtc, time, датчик)
  int8_t startHour = 0;       // start time for RTC
  int8_t impulsePrd = 1;      // impulse period
  int16_t threshold = 30;     // min. response threshold
  int16_t thresholdMax = 30;  // max. response threshold
  int16_t sensPeriod = 2;     // sensor polling period (seconds)
  uint32_t period = 100;      // stert period
  uint32_t work = 10;          // work period
  uint32_t weekOn = 100;      // week on
  uint32_t weekOff = 10;       // week off
};
// 32
channelsStruct activeChannel, setChannel;

#define loadChannel(x) channelsStruct(EEPROM.get((x) * EEPR_CH_STEP, activeChannel))

uint32_t timerMillis[10];       // milisecond counter

extern uint32_t driveTimer;
extern byte driveState;
extern boolean lastDriveState;
extern boolean manualControl;
extern boolean manualPos;
extern boolean controlState;

const byte PIDchs[] = {0, 1, 2, 3, 7, 8, 9};
const byte channelToPWM[] = {0, 1, 2, 3, 0, 0, 0, 4, 5, 6};  // channel to pwm
const byte impulsePrds[] = {1, 5, 10, 15, 20, 30, 1, 2, 3, 4, 6, 8, 12, 1, 2, 3, 4, 5, 6, 7};
const byte relayPins[] = {RELAY_0, RELAY_1, RELAY_2, RELAY_3, RELAY_4, RELAY_5, RELAY_6, SERVO_0, SERVO_1};

float sensorVals[8];    // sensors ...
int8_t realTime[3];
float uptime = 0;
byte servoPosServ[2];
const int PWMperiod = (float)1000 / PWM_RELAY_HZ;
int PWMactive[2];

int pwmVal[7];   // drive movement time for PID
boolean drivePidFlag; // drive movement for PID flag

boolean channelStates[10];
boolean channelStatesServ[10];
int8_t debugPage;

int8_t arrowPos;  // 0-3
int8_t navDepth;  // 0-2
int8_t currentChannel = 0;  // -3 - 14
int8_t currentMode; // 0-3
int8_t thisH[2], thisM[2], thisS[2];
int8_t currentLine;
uint32_t commonTimer, backlTimer, plotTimer;
boolean backlState = true;

int sensMinute[6][15];

boolean serviceFlag;
boolean startPID = false;
boolean timeChanged;
boolean startFlagDawn;
uint32_t settingsTimer;
uint32_t driveTout;

byte thisMode;
byte curMode;

const byte daysMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 0};
uint16_t thisDay;

#if (START_MENU == 1)
bool startService = false;
#endif

#if (USE_PLOTS == 1 || USE_PID == 1 || USE_DAWN == 1)
// graphics
const char row8[] PROGMEM = {0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111};
const char row7[] PROGMEM = {0b00000,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111};
const char row6[] PROGMEM = {0b00000,  0b00000,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111};
const char row5[] PROGMEM = {0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111,  0b11111,  0b11111};
const char row4[] PROGMEM = {0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111,  0b11111};
const char row3[] PROGMEM = {0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111,  0b11111};
const char row2[] PROGMEM = {0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111,  0b11111};
const char row1[] PROGMEM = {0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b00000,  0b11111};
#endif

// co2
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
        // protection against measurement after display
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

// sensors names
#define SENS1_NAME "Sen1"
#define SENS2_NAME "Sen2"
#define SENS3_NAME "Sen3"
#define SENS4_NAME "Sen4"

#if (DALLAS_SENS1 == 1)
#define SENS1_NAME "Dall"
#endif

#if (THERM1 == 1)
#define SENS1_NAME "Tmp1"
#endif
#if (THERM2 == 1)
#define SENS2_NAME "Tmp2"
#endif
#if (THERM3 == 1)
#define SENS3_NAME "Tmp3"
#endif
#if (THERM4 == 1)
#define SENS4_NAME "Tmp4"
#endif

#if (USE_CO2 == 1)
#if (CO2_PIN == 1)
#define SENS1_NAME "CO2"
#elif (CO2_PIN == 2)
#define SENS2_NAME "CO2"
#endif
#endif

const char *sensorNames[]  = {
  "AirT",
  "AirH",

  SENS1_NAME,
  SENS2_NAME,
  SENS3_NAME,
  SENS4_NAME,
  "AirP"
};

#if (USE_PLOTS == 1)
const char *plotNames[]  = {
  "Min",
  "Hour",
  "Day",
};
#endif

// harmful functions
void smartArrow(bool state = true);

#endif