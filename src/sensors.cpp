#include "a0_data.h"
#include "sensors.h"



void getAllData() {
  sensorVals[0] = 0;
  sensorVals[1] = 0;

#if (USE_BME == 1)
  sensorVals[0] = bme.readTemperature();
  sensorVals[1] = bme.readHumidity();
#endif


// #if (DHT_SENS2 == 1)
//   sensorVals[0] = dht.readTemperature();
//   sensorVals[1] = dht.readHumidity();
// #endif


#if (USE_BMP280 == 1)
  sensorVals[0] = bmp280.getTemperature();
  sensorVals[6] = bmp280.getPressure();
#endif

#if (USE_AHT20 == 1)
sensorVals[0] = aht20.getTemperature();
sensorVals[1] = aht20.getHumidity();
#endif


#if (USE_HTU21D == 1)
  sensorVals[1] = myHTU21D.readHumidity();
  sensorVals[0] = myHTU21D.readTemperature();
#endif

  sensorVals[1] = constrain(sensorVals[1], 0, 99);

  sensorVals[2] = analogReadAverage(SENS_1) / 4;
  sensorVals[3] = analogReadAverage(SENS_2) / 4;
  sensorVals[4] = analogReadAverage(SENS_3) / 4;
  sensorVals[5] = analogReadAverage(SENS_4) / 4;

#if (DALLAS_SENS1 == 1)
#if (DALLAS_AMOUNT > 1)
  for (byte i = 0; i < DALLAS_AMOUNT; i++) {
    float thisDal = dallas[i].getTemp();
    if (thisDal != 0) dallasBuf[i] = thisDal;
    dallas[i].requestTemp();
  }
  float thisMin = 200.0, thisMax = -200.0, thisSum = 0.0;
  for (byte i = 0; i < DALLAS_AMOUNT; i++) {
    thisSum += dallasBuf[i];
    if (dallasBuf[i] > thisMax) thisMax = dallasBuf[i];
    if (dallasBuf[i] < thisMin) thisMin = dallasBuf[i];
  }

#if (DALLAS_MODE == 0)
  sensorVals[2] = thisSum / DALLAS_AMOUNT;
#elif (DALLAS_MODE == 1)
  sensorVals[2] = thisMax;
#elif (DALLAS_MODE == 2)
  sensorVals[2] = thisMin;
#endif

#else
  sensorVals[2] = dallas.getTemp();
  dallas.requestTemp();
#endif
#endif

#if (USE_CO2 == 1)
#if (CO2_PIN == 1)
  sensorVals[2] = CO2ppm;
#elif (CO2_PIN == 2)
  sensorVals[3] = CO2ppm;
#endif
#endif

#if (THERM1 == 1)
  sensorVals[2] = getThermTemp(analogReadAverage(SENS_1), BETA_COEF1);
#endif
#if (THERM2 == 1)
  sensorVals[3] = getThermTemp(analogReadAverage(SENS_2), BETA_COEF2);
#endif
#if (THERM3 == 1)
  sensorVals[4] = getThermTemp(analogReadAverage(SENS_3), BETA_COEF3);
#endif
#if (THERM4 == 1)
  sensorVals[5] = getThermTemp(analogReadAverage(SENS_4), BETA_COEF4);
#endif
}

#if (THERM1 == 1 || THERM2 == 1 || THERM3 == 1 || THERM4 == 1)
#define RESIST_BASE 10000   
#define TEMP_BASE 25        
#define RESIST_10K 10000    

float getThermTemp(int resistance, int B_COEF) {
  float thermistor;
  thermistor = RESIST_10K / ((float)1024 / resistance - 1);
  thermistor /= RESIST_BASE;                        
  thermistor = log(thermistor) / B_COEF;            
  thermistor += (float)1.0 / (TEMP_BASE + 273.15);  
  thermistor = (float)1.0 / thermistor - 273.15;    
  return thermistor;
}
#endif

int analogReadAverage(byte pin) {
  
  for (byte i = 0; i < 10; i++)
    analogRead(pin);

  
  int sum = 0;
  for (byte i = 0; i < 10; i++)
    sum += analogRead(pin);
  return (sum / 10);
}

uint32_t sensorTimer = 0;
uint32_t period = 1000;
byte sensorMode = 0;
void readAllSensors() {
  if (millis() - sensorTimer >= period) {
    sensorTimer = millis();
    switch (sensorMode) {
      case 0:   
        sensorMode = 1;
        period = 100;
        digitalWrite(SENS_VCC, 1);
        break;
      case 1:   
        sensorMode = 2;
        period = 25;
        getAllData();
        break;
      case 2:   
        sensorMode = 0;
        period = (long)settings.comSensPeriod * 1000;
        digitalWrite(SENS_VCC, 0);
        break;
    }
  }
}

#if (USE_PLOTS == 1)
void plotTick() {
  if (millis() - plotTimer >= 60000L) {     
    plotTimer += 60000L;
    static uint16_t counter = 0;
    static int averH[6] = {0, 0, 0, 0, 0, 0};
    static int averD[6] = {0, 0, 0, 0, 0, 0};
    int tempPlot[6][15];
    counter++;                              

    int ints[6];
    
    for (byte i = 0; i < 6; i++) ints[i] = (float)sensorVals[i] * 7;

    scrollPlot(sensMinute, ints);           

    if (counter % 10 == 0) {                
      for (byte i = 0; i < 6; i++) averH[i] += ints[i];
    }
    if (counter % 240 == 0) {               
      for (byte i = 0; i < 6; i++) averD[i] += ints[i];
    }
    if (counter % 60 == 0) {                      
      for (byte i = 0; i < 6; i++) averH[i] /= 6; 
      EEPROM.get(EEPR_PLOT_H, tempPlot);          
      scrollPlot(tempPlot, averH);                
      EEPROM.put(EEPR_PLOT_H, tempPlot);          
      for (byte i = 0; i < 6; i++) averH[i] = 0;  
    }

    if (counter % 1440L == 0) {                   
      for (byte i = 0; i < 6; i++) averD[i] /= 6; 
      EEPROM.get(EEPR_PLOT_D, tempPlot);          
      scrollPlot(tempPlot, averD);                
      EEPROM.put(EEPR_PLOT_D, tempPlot);          
      for (byte i = 0; i < 6; i++) averD[i] = 0;  
      counter = 0;
    }
  }

}
void scrollPlot(int vals[6][15], int newVal[6]) {
  for (byte i = 0; i < 6; i++) {
    for (byte j = 0; j < 14; j++)
      vals[i][j] = vals[i][j + 1];

    
    vals[i][14] = newVal[i];
  }
}
#endif
