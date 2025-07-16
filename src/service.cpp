#include "a0_data.h"
#include "service.h"

void serviceTick() {
  if (millis() - commonTimer > 500) {
    commonTimer = millis();
    
    for (byte i = 0; i < 7; i++) {
      digitalWrite(relayPins[i], channelStatesServ[i]);
    }
    if (SERVO1_RELAY) {
      digitalWrite(SERVO_0, channelStatesServ[7]);
    }
    if (SERVO2_RELAY) {
      digitalWrite(SERVO_1, channelStatesServ[8]);
    }
    
#if (SERVO1_RELAY == 0)
#if (SMOOTH_SERVO == 1)
    servo1.setTargetDeg(servoPosServ[0]);
#else
    servo1.write(servoPosServ[0]);
#endif
#endif

#if (SERVO2_RELAY == 0)
#if (SMOOTH_SERVO == 1)
    servo2.setTargetDeg(servoPosServ[1]);
#else
    servo2.write(servoPosServ[1]);
#endif
#endif
  }
}

void serviceIN() {    
#if (SERVO1_RELAY == 0)
#if (SMOOTH_SERVO == 1)
  servo1.start();
#else
  servo1.attach(SERVO_0, 600, 2400);
#endif

#endif
#if (SERVO2_RELAY == 0)
#if (SMOOTH_SERVO == 1)
  servo2.start();
#else
  servo2.attach(SERVO_1, 600, 2400);
#endif

#endif

  servoPosServ[0] = pwmVal[4];
  servoPosServ[1] = pwmVal[5];

  serviceFlag = true;
  for (byte i = 0; i < 10; i++) channelStatesServ[i] = channelStates[i];
  
  realTime[0] = rtc.getHours();
  realTime[1] = rtc.getMinutes();
  realTime[2] = rtc.getSeconds();
}

void serviceOUT() {   
  serviceFlag = false;
  startFlagDawn = true;
#if (START_MENU == 0)
  for (byte i = 0; i < 7; i++) {
    if (loadChannel(i).state) digitalWrite(relayPins[i], channelStates[i]); 
  }

#if (SERVO1_RELAY == 1)
  digitalWrite(SERVO_0, channelStates[7]);
#endif
#if (SERVO2_RELAY == 1)
  digitalWrite(SERVO_1, channelStates[8]);
#endif

#if (USE_DAWN == 1)
  
  if (loadChannel(2).mode == 5) {
    analogWrite(RELAY_2, pwmVal[2]);
  }
  if (loadChannel(3).mode == 5) {
    analogWrite(RELAY_3, pwmVal[3]);
  }
#endif

  
#if (SERVO1_RELAY == 0)
#if (SMOOTH_SERVO == 1)
  if (!loadChannel(7).state) servo1.stop();
  servo1.setTargetDeg(pwmVal[4]);
#else
  servo1.write(pwmVal[4]);
  if (!loadChannel(7).state) servo1.detach();
#endif

#endif
#if (SERVO2_RELAY == 0)
#if (SMOOTH_SERVO == 1)
  if (!loadChannel(8).state) servo2.stop();
  servo2.setTargetDeg(pwmVal[5]);
#else
  servo2.write(pwmVal[5]);
  if (!loadChannel(8).state) servo2.detach();
#endif
#endif

#if (USE_DRIVE == 1)
  
  if (channelStatesServ[9] != channelStates[9])
    driveState = 1;
#endif
#endif
  if (timeChanged) {
    timeChanged = false;
    rtc.setTime(realTime[2], realTime[1], realTime[0], rtc.getDate(), rtc.getMonth(), rtc.getYear());
  }
}

#if (START_MENU == 1)
void drawStartMenu(int8_t pos) {
  lcd.setCursor(0, 0);
  smartArrow(pos == 0);
  lcd.print(F("Reset settings"));
  lcd.setCursor(0, 1);
  smartArrow(pos == 1);
  lcd.print(F("Service"));
  lcd.setCursor(0, 2);
  smartArrow(pos == 2);
  lcd.print(F("Back"));
}
void startupService() {
  currentChannel = -3;
  serviceIN();
  currentLine = 4;
  startService = true;
  redrawScreen();
  for (;;) {
    controlTick();
    serviceTick();
    if (!startService) {
      serviceFlag = false;
      startFlagDawn = true;
      if (timeChanged) {
        timeChanged = false;
        rtc.setTime(realTime[2], realTime[1], realTime[0], rtc.getDate(), rtc.getMonth(), rtc.getYear());
      }
      lcd.clear();
      drawStartMenu(1);
      return;
    }
  }
}
void startMenu() {
  int8_t cursorPos = 0;
  bool serviceWorkFlag = false;
  for (;;) {
    enc.tick();
    if (enc.isClick()) {
      switch (cursorPos) {
        case 0: clearEEPROM(); lcd.setCursor(16, 0); lcd.print(F("OK")); break;
        case 1: startupService(); break;
        case 2: return;
      }
    }
    if (enc.isTurn()) {
      if (enc.isRight()) {
        cursorPos++;
      }
      if (enc.isLeft()) {
        cursorPos--;
      }
      cursorPos = constrain(cursorPos, 0, 2);
      drawStartMenu(cursorPos);
    }
  }
}
#endif
