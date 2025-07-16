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

// Servo objects
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

// Dallas sensor objects
#if (DALLAS_SENS1 == 1)
#if (DALLAS_AMOUNT > 1)
MicroDS18B20 dallas[DALLAS_AMOUNT];
float dallasBuf[DALLAS_AMOUNT];
#else
MicroDS18B20 dallas(SENS_1);
#endif
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

// PID variables (only defined if PID is enabled)
#if (USE_PID == 1)
PIDstruct activePID, setPID;
float integralSum[PID_CH_AMOUNT];
float prevInput[PID_CH_AMOUNT];
float input[PID_CH_AMOUNT];
int output[PID_CH_AMOUNT];
uint32_t PIDtimers[PID_CH_AMOUNT];
#endif

// PID relay variables  
#if (USE_PID_RELAY == 1)
uint32_t tmr0, tmr1;
bool flag0, flag1;
#endif

// Schedule variables
#if (SCHEDULE_NUM > 0)
scheduleStruct setSchedule, activeSchedule;
#endif

// Dawn variables
#if (USE_DAWN == 1)
dawnStruct setDawn, activeDawn;
#endif

// Channel variables
channelsStruct activeChannel, setChannel;

// Constant arrays  
const byte PIDchs[] = {0, 1, 2, 3, 7, 8, 9};
const byte channelToPWM[] = {0, 1, 2, 3, 0, 0, 0, 4, 5, 6};
const byte impulsePrds[] = {1, 5, 10, 15, 20, 30, 1, 2, 3, 4, 6, 8, 12, 1, 2, 3, 4, 5, 6, 7};
const byte relayPins[] = {RELAY_0, RELAY_1, RELAY_2, RELAY_3, RELAY_4, RELAY_5, RELAY_6, SERVO_0, SERVO_1};

// Dallas sensor addresses
#if (DALLAS_AMOUNT > 1)
const uint8_t dsAddress[][8] = {
  {0x28, 0xFF, 0x42, 0x5A, 0x51, 0x17, 0x4, 0xD2}, 
  {0x28, 0xFF, 0x53, 0xE5, 0x50, 0x17, 0x4, 0xC3}, 
  {0x28, 0xFF, 0x99, 0x80, 0x50, 0x17, 0x4, 0x4D}, 
};
#endif

// Schedule page names
#if (SCHEDULE_NUM > 0)
const char *schedulePageNames[] = {
  "Channel",
  "Start",
  "End",
  "Amount",
};
#endif

// PID tuner names
#if (PID_AUTOTUNE == 1)
const char *tuneNames[] = {
  "off",
  "wait steady",
  "wait kick",
  "step up",
  "step down",
};
#endif

// Plot names
#if (USE_PLOTS == 1)
const char *plotNames[] = {
  "Min",
  "Hour",
  "Day",
};
#endif

// PID Autotune variables
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
#endif

// Settings structure
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
} settings;

// CO2 sensor variables
#if (USE_CO2 == 1 && CO2_CALIB == 0)
uint16_t _tx_delay;
uint8_t *_tx_pin_reg;
uint8_t _tx_pin_mask;
#endif  

