#include "a0_data.h"

boolean chkTimer (unsigned long &startTime, unsigned long loopTime) 
{ if (millis()-startTime >= loopTime)  {      
        startTime = millis();          
        return true;  }                
  else return false;  
}

void customSetup() {
timerMillis[0] = millis();
timerMillis[1] = millis();
  

}

void customLoop() {

  
#if (DEBUG_ENABLE==1)  
  if (chkTimer(timerMillis[1], 5000)) {
    Serial.print("currentChannel:"); Serial.println(currentChannel);    
    Serial.print("Sensor        :"); Serial.println(sensorNames[setChannel.sensor]);
    Serial.print("Relay Type    :"); Serial.println(setChannel.relayType);
    Serial.print("threnshold    :"); Serial.println(setChannel.threshold);
    Serial.print("thrensholdMax :"); Serial.println(setChannel.thresholdMax);
    Serial.print("Value         :"); Serial.println(sensorVals[setChannel.sensor]);  
  timerMillis[1]=millis();  
  }

 
#endif
}
