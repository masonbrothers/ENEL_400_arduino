#define DEPLOY_MODE  //Note that to set the time on the Real Time Clock, we need to run it with DEPLOY_MODE off.

//#define SERIAL_DEBUG_MODE

#define LCD_MODE
#define WEB_INTERFACE_MODE


// For Water Temperature Probe
#include <OneWire.h>
#include <DallasTemperature.h>

// For Air Temperature and Humidity Sensor
#include "DHT.h" 

// For SD Card
#include <SPI.h> 
#include <SD.h>

// For Real Time Clock (RTC)
#include <Wire.h>
#include "RTClib.h"

#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"

#include <SoftwareSerial.h>

#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

#define BUZZER_PIN                                        4 // ???
#define PUMP_PIN                                          12                           
#define ESP8266_RESET_PIN                                 
#define WATER_THERMOMETER_DALLAS_ONE_WIRE_PIN             5
#define WATER_LEVEL_ANALOG_PIN                            A1 // ???
#define WATER_LEVEL_RESISTOR_VALUE                        560 // Value of the resistor added in ohms    
#define MAX_RESOLUTION                                    1023
#define ADAFRUIT_DATA_LOGGING_SHIELD_CHIP_SELECT          10
// Note that this pin needs a 10K pullup Resistor 
#define AMBIENT_TEMPERATURE_HUMIDITY_SENSOR_PIN           4
#define AMBIENT_TEMPERATURE_HUMIDITY_SENSOR_TYPE          DHT22
#define LUMINOSITY_SCL_PIN                                A5
#define LUMINOSITY_SDA_PIN                                A4
#define DATA_FILE_NAME                                    "DATA.TXT"
#define LOG_FILE_NAME                                     "LOG.TXT"
#define TSL2591_NUMBER                                    2591

#define UNSIGNED_LONG_MAX                                 2147483647

#define SOFTWARE_SERIAL_ARDUINO_TX_OTHER_RX               9
#define SOFTWARE_SERIAL_ARDUINO_RX_OTHER_TX               8

#define C_STRING_BUFFER_SIZE                              250
#include <assert.h>


OneWire oneWire(WATER_THERMOMETER_DALLAS_ONE_WIRE_PIN);

DallasTemperature waterTemperatureSensor(&oneWire);

DHT dht(AMBIENT_TEMPERATURE_HUMIDITY_SENSOR_PIN, AMBIENT_TEMPERATURE_HUMIDITY_SENSOR_TYPE);

RTC_PCF8523 rtc;

Adafruit_TSL2591 tsl = Adafruit_TSL2591(TSL2591_NUMBER);

SoftwareSerial esp8266Serial(SOFTWARE_SERIAL_ARDUINO_RX_OTHER_TX, SOFTWARE_SERIAL_ARDUINO_TX_OTHER_RX); // RX, TX

struct AmbientTemperatureHumidity
{
  float ambientTemperature;
  float ambientHumidity;
  boolean valid;
};

// unsigned long counter = 0;

boolean validAmbientTemperatureHumidity = true; //Boolean to detect whether or not Reading was valid

boolean pumpIsOn;

boolean pumpShouldBeOn;

#ifdef LCD_MODE
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
#endif

#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

#include <avr/pgmspace.h>

const char aquaponicsClubString[] PROGMEM = {"Aquaponics Club"};
const char initializedSDCardString[] PROGMEM = {"Initialized SD Card."};
const char initializedRTCString[] PROGMEM = {"Initialized the RTC."};
const char failedInitializeSDCardString[] PROGMEM = {"Failed to initialize SD Card."};
const char failedInitializeRTCString[] PROGMEM = {"Failed to initialize the RTC."};
const char failedTemperatureHumidityString[] PROGMEM = {"Ambient Temperature and Humidity Sensor Not Working!!!"};
const char failedLogWriteString[] PROGMEM = {"Cannot write to " LOG_FILE_NAME};

void setup() {
  Serial.begin(9600);
  esp8266Serial.begin(9600);

#ifdef LCD_MODE
  lcd.begin(16, 2);
#endif

  setupBuzzer();
  stopBuzzer();
  
  setupPump();
  startPump();

  setupLightSensor();
  
  setupWaterTemperatureSensor();
#ifdef LCD_MODE
  //lcd.print((String)counter);
  lcd.setBacklight(WHITE);
#endif
  #ifdef SERIAL_DEBUG_MODE
  if (setupSDCard())
    Serial.println(initializedSDCardString);
  else
    Serial.println(failedInitializeSDCardString);
  #else
  setupSDCard();
  #endif
  
  
  setupAmbientTemperatureAndHumidity();
  
  if (setupRTC())
  {
    #ifdef SERIAL_DEBUG_MODE
    Serial.println(initializedRTCString);
    #endif
    #ifndef DEPLOY_MODE
    setRTCTime(); //TODO -- ONLY RUN THIS WHEN TESTING. AFTER LOADING UP, need to get rid of this.
    #endif
  }
  else
  {
    #ifdef SERIAL_DEBUG_MODE
    Serial.println(failedInitializeRTCString);
    #endif
    writeToSDCardLog(failedInitializeRTCString);
  }

}

void loop() {

  delay(2000);


  
  float waterTemperature = getWaterTemperature();

  int visibleLight = getVisibleLight();
  int waterLevelResistance = waterLevelSensorResistance();
  

  AmbientTemperatureHumidity ambientTemperatureHumidity = readAmbientTemperatureAndHumidity();

  printToScreens("Water: ", waterTemperature, "C");
  

  
  
  if (ambientTemperatureHumidity.valid)
  {
    printToScreens("Air Temp: ", ambientTemperatureHumidity.ambientTemperature, "C");
    printToScreens("Air Hum: ", ambientTemperatureHumidity.ambientHumidity, "%");
  }
  #ifdef SERIAL_DEBUG_MODE
  else
  {
    Serial.println(failedTemperatureHumidityString);
    writeToSDCardLog(failedTemperatureHumidityString);
  }
  #endif
  
  printToScreens("Light: ", visibleLight, "lux");

  //Serial.println("IR light: " + (String)getIRLight() + " lux");



  unsigned long unixTime = rtc.now().unixtime();
  
#ifdef WEB_INTERFACE_MODE
  //printToESP((String)deviceName);

  printToESP("p:" + (String)(int)pumpIsOn); //pumpIsOn
  
  printToESP("h:" + (String)ambientTemperatureHumidity.ambientHumidity); //ambientHumidity
  
  printToESP("l:" + (String)visibleLight); //ambientLight
  
  printToESP("a:" + (String)ambientTemperatureHumidity.ambientTemperature); //ambientTemperature
  
  //printToESP("spillSensor:" + (String)(int)spillSensor);
  
  printToESP("t:" + (String)unixTime); //time
  
  printToESP("e:" + (String)waterLevelResistance); //waterLevel
  
  printToESP("w:" + (String)waterTemperature); // waterTemperature
  printTimeToScreens();
  getFromESP();
  

#endif
  writeToSDCard(DATA_FILE_NAME, pumpIsOn + '\t' + (String)ambientTemperatureHumidity.ambientHumidity + '\t' + (String)visibleLight + '\t' + (String)visibleLight + '\t' + (String)ambientTemperatureHumidity.ambientTemperature + '\t' + (String)unixTime + '\t' + (String)waterLevelResistance + '\t' + (String)waterTemperature);
 // counter++;
 // if (counter == UNSIGNED_LONG_MAX)
 //   counter = 0;
}

void getFromESP()
{
  esp8266Serial.print("P");
  if (esp8266Serial.available()) {
    pumpShouldBeOn = (boolean)getESPSerialInt();
  }
  if (pumpShouldBeOn)
    startPump();
  else
    stopPump();

  /*
  esp8266Serial.print("C");
  if (esp8266Serial.available()) {
    counter = getESPSerialInt();
  }
  */
}

int getESPSerialInt()
{
  return esp8266Serial.readString().toInt();
}

void printTimeToScreens()
{
  String Time = getRTCStringTime();

  #ifdef SERIAL_DEBUG_MODE
  Serial.println(Time);
  #endif
  #ifdef LCD_MODE
  lcd.clear();
  lcd.print(Time);
  #endif
}
void printToScreens(String input)
{
  #ifdef SERIAL_DEBUG_MODE
  Serial.println(input);
  #endif
  #ifdef LCD_MODE
  lcd.clear();
  lcd.print(input);
  #endif
}

void printToScreens(char *type, float input, char *unit)
{
  String output = type + (String)input + unit;

  #ifdef SERIAL_DEBUG_MODE
  Serial.println(output);
  #endif
  #ifdef LCD_MODE
  lcd.clear();
  lcd.print(output);
  #endif
}

bool printToESP(String input) {
  /*int count = 100;
  while (!esp8266Serial.available() && count > 0 )
  {
    count --;
    delay(10);// Wait until Serial is available
  }
  if (count > 0)
  {
    esp8266Serial.println(input);
    return 1;
  }
  */
  delay(4000);
  esp8266Serial.println(input);
  
  
  return 0;
}

void setupLightSensor()
{
  tsl.setGain(TSL2591_GAIN_MED);      // 25x gain
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
}

int getVisibleLight()
{
  return tsl.getLuminosity(TSL2591_VISIBLE);
}

int getIRLight()
{
  return tsl.getLuminosity(TSL2591_INFRARED);
}

bool writeToSDCardLog(String input)
{
  if (writeToSDCard(LOG_FILE_NAME, input))
    return true;
  Serial.println(failedLogWriteString);
  return false;
}

void setupWaterTemperatureSensor()
{
  waterTemperatureSensor.begin();
}

float getWaterTemperature()
{
  waterTemperatureSensor.requestTemperatures();
  return waterTemperatureSensor.getTempCByIndex(0);
}

boolean setupRTC()
{
  //rtc.initialized();
  return rtc.begin();
}

void setRTCTime()
{
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

DateTime getRTCTime()
{
  return rtc.now();
}

String getRTCStringTime()
{
  DateTime now = getRTCTime();
  String output = (int)now.year() + (String)"-" + addPrecedingZero(now.dayOfTheWeek()) + (String)"-" + addPrecedingZero(now.day()) + (String)"T" + addPrecedingZero(now.hour()) + (String)":" + addPrecedingZero(now.minute()) + (String)":" + addPrecedingZero(now.second());
  return output;
}

String addPrecedingZero(int input)
{
  String output;
  if (input > 10)
    output = (String) input;
  else
    output = "0" + (String) input;
  return output;
}

boolean setupSDCard()
{
  // see if the card is present and can be initialized:
  if (!SD.begin(ADAFRUIT_DATA_LOGGING_SHIELD_CHIP_SELECT)) {
    return false;
  }
  return true;
}

void setupAmbientTemperatureAndHumidity()
{
  dht.begin();
}


AmbientTemperatureHumidity readAmbientTemperatureAndHumidity()
{
  //Sensor requires a delay of atleast 2 seconds to work
  delay(2000);
  AmbientTemperatureHumidity ambientTemperatureHumidity;
  ambientTemperatureHumidity.ambientTemperature = dht.readTemperature();
  ambientTemperatureHumidity.ambientHumidity = dht.readHumidity();
  
  //If Ambient Temperature and Humidity Readings are garbage output an error message
  if (isnan(ambientTemperatureHumidity.ambientTemperature) || isnan(ambientTemperatureHumidity.ambientHumidity))
  {
    validAmbientTemperatureHumidity = false;
  }
  else
  {
    ambientTemperatureHumidity.valid = true;
  }

  if (isnan(ambientTemperatureHumidity.ambientTemperature))
  {
    ambientTemperatureHumidity.ambientTemperature = -300;
  }

  
  if (isnan(ambientTemperatureHumidity.ambientHumidity))
  {
    ambientTemperatureHumidity.ambientHumidity = -1;
  }
  
  return ambientTemperatureHumidity;
} 

float waterLevelSensorResistance()
{
  float value;
  value = analogRead(WATER_LEVEL_ANALOG_PIN); // This is the raw voltage
  value = (MAX_RESOLUTION / value)  - 1;
  value = WATER_LEVEL_RESISTOR_VALUE / value; // This is the resistance of the meter
  return value;
}

void setupBuzzer()
{
  pinMode(BUZZER_PIN, OUTPUT);
}

void startBuzzer()
{
  digitalWrite(BUZZER_PIN, HIGH);
}

void stopBuzzer()
{
  digitalWrite(BUZZER_PIN, LOW);
}

void setupPump()
{
  pinMode(PUMP_PIN, OUTPUT);
}

void startPump()
{
  digitalWrite(PUMP_PIN, HIGH);
  pumpIsOn = 1;
}

void stopPump()
{
  digitalWrite(PUMP_PIN, LOW);
  pumpIsOn = 0;
}

boolean writeToSDCard(String fileName, String dataToWrite)
{
  File dataFile = SD.open(fileName.c_str(), FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataToWrite);
    dataFile.close();
    return true;
  }
  #ifdef SERIAL_DEBUG_MODE
  else
  {
    Serial.println("Cannot write to " + (String)DATA_FILE_NAME);
  }
  #endif
  
  return false;
}
