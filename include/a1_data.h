#ifndef A1_DATA_H
#define A1_DATA_H

#include <Arduino.h>
#include <stdint.h>

#pragma once

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

// // odłączyć BME jeżeli włączony DHT
// #if (DHT_SENS2 == 1)
// #define USE_BME 0
// #define USE_BMP280 0
// #define USE_ 0
// #endif


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

extern uint16_t thisDay;

// -------------------- Biblioteki ---------------------
#include "encMinim.h"
//extern encMinim enc(CLK, DT, SW, true, false);
extern encMinim enc;

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
extern LiquidCrystal_I2C lcd;

#include <EEPROM.h>

#include <microDS3231.h>
extern MicroDS3231 rtc;

// bme
// #if (USE_BME == 1)
// #include <GyverBME280.h>
// extern GyverBME280 bme;
// #endif

// #if (USE_ADAFRUIT_SENSOR == 1)

// #include <Adafruit_Sensor.h>
// #include <Adafruit_AHTX0.h>
// #include <Adafruit_BMP280.h>

// // Create sensor instances
// Adafruit_AHTX0 aht;
// extern Adafruit_BMP280 bmp; // Default I2C address is 0x77

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

// #if (DHT_SENS2 == 1)
// #include <DHT.h>
// extern DHT dht(SENS_2, DHT_TYPE);
// #endif

// #if (USE_HTU21D == 1)
// #include <microHTU21D.h>
// extern HTU21D myHTU21D(HTU21D_RES_RH12_TEMP14);
// #endif

#if (USE_BMP280 == 1)
#include <BMP280.h>
extern BMP280 bmp280;
#endif

#if (USE_AHT20 == 1)
#include <AHT20.h>
extern AHT20 aht20;
#endif

#if (WDT_ENABLE == 1)
#include <avr/wdt.h>
#endif

extern int8_t lastScreen;

// структура расписания
#if (SCHEDULE_NUM > 0)
struct scheduleStruct {
  byte pidChannel = 0;    // выбор канала
  int startDay = 1;  // день
  int endDay = 1;    // месяц
  byte pointAmount = 15;  // количество точек
  int setpoints[SCHEDULE_MAX];  // массив расписания
};
extern scheduleStruct setSchedule, activeSchedule;
#define loadSchedule(x) scheduleStruct(EEPROM.get((x) * EEPR_SHED_STEP + EEPR_SHED, activeSchedule))
//6+SCHEDULE_NUM*2

extern const char *schedulePageNames[];
#endif

// Channels
struct channelsStruct {
  boolean type;
  boolean state;          // состояние канала (вкл/выкл)
  boolean direction;   // направление работы
  boolean global;     // режим глобальных суток
  int8_t week;            // неделька
  int8_t sensor;              // тип датчика (air temp, air hum, mois1...)
  int8_t relayType;           // тип реле (помпа, клапан, реле)
  int8_t mode;                // режим работы (таймер, rtc, сутки, датчик)
  int8_t startHour;       // начальный час для таймера RTC
  int8_t impulsePrd;      // период импульса
  int16_t threshold;     // мин. порог срабатывания
  int16_t thresholdMax;  // макс. порог срабатывания
  int16_t sensPeriod;     // период опроса датчика (секунды)
  uint32_t period;      // период вызова
  uint32_t work;          // период работы
  uint32_t weekOn;      // неделька вкл
  uint32_t weekOff;       // неделька выкл

  channelsStruct() :
       type(0),
       state(0),
       direction(true),
       global(false),
       week(0),
       sensor(0),
       relayType(0),
       mode(0),
       startHour(0),
       impulsePrd(1),
       threshold(30),
       thresholdMax(30),
       sensPeriod(2),
       period(100),
       work(10),
       weekOn(100),
       weekOff(10) {}
};
// 32
extern channelsStruct activeChannel, setChannel;

#define loadChannel(x) channelsStruct(EEPROM.get((x) * EEPR_CH_STEP, activeChannel))




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
extern dawnStruct setDawn, activeDawn;
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
extern PIDstruct activePID, setPID;

#define loadPID(x) PIDstruct(EEPROM.get((x) * EEPR_PID_STEP + EEPR_PID, activePID))
#define savePID(x) (EEPROM.put((x) * EEPR_PID_STEP + EEPR_PID, activePID))

extern float integralSum[PID_CH_AMOUNT];
extern float prevInput[PID_CH_AMOUNT];
extern float input[PID_CH_AMOUNT];
extern int output[PID_CH_AMOUNT];
extern uint32_t PIDtimers[PID_CH_AMOUNT];
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

extern const char *tuneNames[];
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

extern long timerMillis[10];       // milisecond counter

extern uint32_t driveTimer;
extern byte driveState;
extern boolean lastDriveState;
extern boolean manualControl;
extern boolean manualPos;
extern boolean controlState;

extern const byte PIDchs[];
extern const byte channelToPWM[];  // channel to pwm
extern const byte impulsePrds[];
extern const byte relayPins[];

extern float sensorVals[8];    // sensors ...
extern int8_t realTime[3];
extern float uptime;
extern byte servoPosServ[2];
extern const int PWMperiod;
extern int PWMactive[2];

extern int pwmVal[7];   // drive movement time for PID
extern boolean drivePidFlag; // drive movement for PID flag

extern boolean channelStates[10];
extern boolean channelStatesServ[10];
extern int8_t debugPage;

extern int8_t arrowPos;  // 0-3
extern int8_t navDepth;  // 0-2
extern int8_t currentChannel;  // -3 - 14
extern int8_t currentMode; // 0-3
extern int8_t thisH[2], thisM[2], thisS[2];
extern int8_t currentLine;
extern uint32_t commonTimer, backlTimer, plotTimer;
extern boolean backlState;

extern int sensMinute[6][15];

extern boolean serviceFlag;
extern boolean startPID;
extern boolean timeChanged;
extern boolean startFlagDawn;
extern uint32_t settingsTimer;
extern uint32_t driveTout;

extern byte thisMode;
extern byte curMode;



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

extern const char *sensorNames[];

#if (USE_PLOTS == 1)
extern const char *plotNames[];
#endif

// harmful functions
void smartArrow(bool state = true);


void clearEEPROM();
void applySettings();
void customSetup();
void customLoop();

#endif