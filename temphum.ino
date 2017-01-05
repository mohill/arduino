/*

Temperature and humidity sensor with a DHT22
Uses a Arduino pro micro 3,3V

*/
#include <MySensor.h>  
#include <dht.h>
//#include "LowPower.h"

MySensor gw;

dht DHT;

#define CHILD_ID_HUM 2
#define CHILD_ID_TEMP 3
#define DHT22_PIN 4
#define MY_RF24_PA_LEVEL RF24_PA_LOW

int BATTERY_SENSE_PIN = A0;
int oldBatteryPcnt = 0;

float lastTemp;
float lastHum;
boolean metric = true; 
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);

int reportCount = 1;
int sanityBailoutTemp = 1;
int sanityBailoutHum = 1;

void setup()  
{ 
  analogReference(INTERNAL);
  gw.begin();

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Indoor temperature", "2.0");

  // Register all sensors to gateway (they will be created as child devices)
  gw.present(CHILD_ID_HUM, S_HUM);
  gw.present(CHILD_ID_TEMP, S_TEMP);
  
  metric = gw.getConfig().isMetric;

  // Initialize serial with 9600 baud
  Serial.begin(9600);
}

void loop()      
{  
  // Sleep for 8 s with ADC module and BOD module off
  //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

  // Battery level
/*  int sensorValue = analogRead(BATTERY_SENSE_PIN);
  Serial.print("Battery level: ");
  Serial.println(sensorValue);
  // Reference
  // 1M, 470K divider across battery and using internal ADC ref of 1.1V
  // Sense point is bypassed with 0.1 uF cap to reduce noise at that point
  // ((1e6+470e3)/470e3)*1.1 = Vmax = 3.3 Volts
  // 3.3/1023 = Volts per bit = 0.003363075
  // My values:
  // 810k, 405K divider
  // ((810e3+405e3)/405e3)*1.1 = Vmax = 3.3 Volts
  // 3.3/1023 = Volts per bit = 0.003363075
  float batteryV  = sensorValue * 0.003363075;
  int batteryPcnt = sensorValue / 10;
  Serial.print("Battery Voltage: ");
  Serial.print(batteryV);
  Serial.println(" V");
  Serial.print("Battery percent: ");
  Serial.print(batteryPcnt);
  Serial.println(" %");
  if (oldBatteryPcnt != batteryPcnt) {
    // Power up radio after sleep
    gw.sendBatteryLevel(batteryPcnt);
    oldBatteryPcnt = batteryPcnt;
  }
*/
  // more info: https://forum.pimatic.org/topic/457/mysensors-plugin-battery-level/7
  // http://lygte-info.dk/info/BatteryChargePercent%20UK.html
  #define MIN_V 2700 // origianl value was 2700
  #define MAX_V 4200 // Origianl value was 3200
  int batteryPcnt = min(map(readVcc(), MIN_V, MAX_V, 0, 100), 100); // Convert voltage to percentage
  Serial.print("battery: ");
  Serial.println(batteryPcnt);
  if (oldBatteryPcnt != batteryPcnt) {
    oldBatteryPcnt = batteryPcnt;
    gw.sendBatteryLevel(batteryPcnt); // Send battery percentage to gateway
    Serial.println("Sent battery percentage!");
  }
 
  Serial.print("DHT22, \t");
  int chk = DHT.read22(DHT22_PIN);
  switch (chk)
  {
    case DHTLIB_OK:  
    Serial.print("OK,\t"); 
    break;
    case DHTLIB_ERROR_CHECKSUM: 
    Serial.print("Checksum error,\t"); 
    break;
    case DHTLIB_ERROR_TIMEOUT: 
    Serial.print("Time out error,\t"); 
    break;
    default: 
    Serial.print("Unknown error,\t"); 
    break;
  }
  // DISPLAY DATA
  Serial.print(DHT.humidity, 1);
  Serial.print(",\t");
  Serial.println(DHT.temperature, 1);

  float temperature = DHT.temperature;
  if (isnan(temperature)) {
      Serial.println("Failed reading temperature from DHT");
  } 
  else {
    Serial.print("T: ");
    Serial.println(temperature);
    if(lastTemp != temperature || reportCount == 10) {
      float sanityVal = lastTemp - temperature;
      float sensVal = constrain(sanityVal, -5, 5);
      if (sensVal == sanityVal || sanityBailoutTemp == 10) {
        lastTemp = temperature;
        Serial.println("Sent temperature!");
        gw.send(msgTemp.set(temperature, 1));
        sanityBailoutTemp = 1;    
      }
      sanityBailoutTemp++;
    }
  }

  // Humidity
  float humidity = DHT.humidity;
  if (isnan(humidity)) {
      Serial.println("Failed reading humidity from DHT");
  } 
  else {
    Serial.print("H: ");
    Serial.println(humidity);
    if(lastHum != humidity || reportCount == 10) { 
      float sanityVal = lastHum - humidity;
      float sensVal = constrain(sanityVal, -20, 20);
      if (sensVal == sanityVal || sanityBailoutHum == 10) {
        lastHum = humidity;
        Serial.println("Sent humidity");
        gw.send(msgHum.set(humidity, 1));
        sanityBailoutHum = 1;
      }
      sanityBailoutHum++;
    }
  }

  if(reportCount == 10) {
    reportCount = 1;
  }
  else {
    reportCount++;
  }
  
  gw.sleep(90000); // production mode
  //gw.sleep(3000); // debug mode
}

long readVcc() {
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
      ADMUX = _BV(MUX5) | _BV(MUX0);
    #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
      ADMUX = _BV(MUX3) | _BV(MUX2);
    #else
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #endif  
   
    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring
   
    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
    uint8_t high = ADCH; // unlocks both
   
    long result = (high<<8) | low;
   
    result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    Serial.print("battery: ");
    Serial.print(result);
    Serial.println(" mV");
    return result; // Vcc in millivolts
  }
