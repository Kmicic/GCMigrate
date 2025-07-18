#include "a0_data.h"
#include "menu.h"
#include "redrawScreen.h"

#if (USE_AHT20 == 1)
#include <AHT20.h>
AHT20 aht20;
#endif  

void initHardware() {
#if (WDT_ENABLE == 1)
  wdt_disable();
  delay(3000); 
  wdt_enable (WDTO_8S); 
#endif

  
  lcd.init();
  lcd.backlight();
  lcd.clear();
#if (USE_PLOTS == 1 || USE_PID == 1 || USE_DAWN == 1)
  lcd.createChar(0, row8);
  lcd.createChar(1, row1);
  lcd.createChar(2, row2);
  lcd.createChar(3, row3);
  lcd.createChar(4, row4);
  lcd.createChar(5, row5);
  lcd.createChar(6, row6);
  lcd.createChar(7, row7);
#endif

  
  if (rtc.lostPower()) {  
    rtc.setTime(COMPILE_TIME);  
  }

  
#if (USE_BME == 1)
  bme.begin(BME_ADDR);
#endif

  
#if (WIRE_OVERCLOCK == 1)
  Wire.setClock(400000);
#endif

  
// #if (DHT_SENS2 == 1)
//   dht.begin();
// #endif

#if (USE_HTU21D == 1)
  myHTU21D.begin();
#endif

#if (USE_BMP280 == 1)
  bmp280.begin();
#endif

#if (USE_AHT20 == 1)
  if (aht20.begin() == false)
  {
    Serial.println("AHT20 not detected. Please check wiring. Freezing.");
    while(true);
  }
#endif

#if (USE_DRIVE == 1)
  
  
  TCCR2A |= _BV(WGM20);
  TCCR2B = TCCR2B & 0b11111000 | 0x01;

  /*
    TCCR2A |= _BV(WGM20) | _BV(WGM21);
    TCCR2B = TCCR2B & 0b11111000 | 0x02;*/
  pinMode(DRV_PWM, OUTPUT);
  pinMode(DRV_SIGNAL1, OUTPUT);
  pinMode(DRV_SIGNAL2, OUTPUT);
  digitalWrite(DRV_SIGNAL1, DRIVER_LEVEL);
  digitalWrite(DRV_SIGNAL2, DRIVER_LEVEL);
  analogWrite(DRV_PWM, settings.drvSpeed);
#endif

  
  EICRA = (EICRA & 0x0C) | 1;  
  bitSet(EIMSK, INT0);            
  EICRA = (EICRA & 0x03) | (1 << 2);
  bitSet(EIMSK, INT1);

  pinMode(SENS_VCC, OUTPUT);

#if (SERVO1_RELAY == 1)
  pinMode(SERVO_0, OUTPUT);
#endif

#if (SERVO2_RELAY == 1)
  pinMode(SERVO_1, OUTPUT);
#endif

  for (byte i = 0; i < 7; i++) pinMode(relayPins[i], OUTPUT);

#if (SERVO1_RELAY == 0)
  servo1.attach(SERVO_0, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
#endif
#if (SERVO2_RELAY == 0)
  servo2.attach(SERVO_1, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
#endif
}

void applySettings() {
  

  channelStates[7] = !loadChannel(7).direction;
  channelStates[8] = !loadChannel(8).direction;

#if (USE_DRIVE == 1)
  analogWrite(DRV_PWM, settings.drvSpeed);
  lastDriveState = !loadChannel(9).direction;
  channelStates[9] = lastDriveState;
  manualPos = !lastDriveState;
#endif
  startFlagDawn = true;

#if (SERVO2_RELAY == 1)
  digitalWrite(SERVO_1, channelStates[8]);
#endif
#if (SERVO1_RELAY == 1)
  digitalWrite(SERVO_0, channelStates[7]);
#endif

  
  for (byte i = 0; i < 7; i++) {
    channelsStruct temp = loadChannel(i);
    channelStates[i] = !temp.direction;        
    if (temp.mode < 4) digitalWrite(relayPins[i], channelStates[i]);     
  }

#if (SERVO1_RELAY == 0)
  if (!loadChannel(7).direction) pwmVal[4] = settings.minAngle[0];
  else pwmVal[4] = settings.maxAngle[0];

#if (SMOOTH_SERVO == 1)
  servo1.attach(SERVO_0, SERVO_MIN_PULSE, SERVO_MAX_PULSE, pwmVal[4]); 
  if (!loadChannel(7).state) servo1.stop();

  servo1.setSpeed(settings.srv1_Speed);    
  servo1.setAccel(settings.srv1_Acc);      
  servo1.setCurrentDeg(pwmVal[4]);
  servo1.setTargetDeg(pwmVal[4]);
#else
  servo1.attach(SERVO_0, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  servo1.write(pwmVal[4]);
#endif
#endif

#if (SERVO2_RELAY == 0)
  if (!loadChannel(8).direction) pwmVal[5] = settings.minAngle[1];
  else pwmVal[5] = settings.maxAngle[1];

#if (SMOOTH_SERVO == 1)
  servo2.attach(SERVO_1, SERVO_MIN_PULSE, SERVO_MAX_PULSE, pwmVal[5]); 
  if (!loadChannel(8).state) servo2.stop();

  servo2.setSpeed(settings.srv2_Speed);    
  servo2.setAccel(settings.srv2_Acc);      
  servo2.setCurrentDeg(pwmVal[5]);
  servo2.setTargetDeg(pwmVal[5]);
#else
  servo2.attach(SERVO_1, SERVO_MIN_PULSE, SERVO_MAX_PULSE); 
  servo2.write(pwmVal[5]);
#endif
#endif

#if (DALLAS_AMOUNT > 1)
  for (byte i = 0; i < DALLAS_AMOUNT; i++) {
    dallas[i].setPin(SENS_1);
    dallas[i].setAddress(dsAddress[i]);
  }
#endif
}


void debTick() {
  if ( (currentChannel >= 0 || currentChannel == -2) && millis() - settingsTimer > SETT_TIMEOUT * 1000L) {   
    settingsTimer = millis();
    changeChannel(-1);
    currentChannel = -1;
    navDepth = 0;
    arrowPos = 0;
    debugPage = 0;
    currentLine = 4;
    redrawScreen();
  }
}

void backlOn() {
  backlState = true;
  backlTimer = millis();
  lcd.backlight();
}

void backlTick() {
  if (backlState && settings.backlight && millis() - backlTimer >= (long)settings.backlTime * 1000) {
    backlState = false;
    lcd.noBacklight();
  }
}


void disableABC() {
#if (USE_CO2 == 1 && CO2_CALIB == 0)
  _tx_delay = 1000000UL / 9600;
  _tx_pin_reg = portOutputRegister(digitalPinToPort(CO2_RX));
  _tx_pin_mask = digitalPinToBitMask(CO2_RX);
  uint8_t command[] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87};

  for (int i = 0; i < 9; i++) {
    uint8_t data = command[i];
    *_tx_pin_reg &= ~ _tx_pin_mask;
    delayMicroseconds(_tx_delay);
    for (uint8_t i = 0; i < 8; i++) {
      if (data & 0x01) {
        *_tx_pin_reg |= _tx_pin_mask;
        delayMicroseconds(_tx_delay);
      } else {
        *_tx_pin_reg &= ~ _tx_pin_mask;
        delayMicroseconds(_tx_delay);
      }
      data >>= 1;
    }
    *_tx_pin_reg |= _tx_pin_mask;
    delayMicroseconds(_tx_delay);
  }
#endif
}
