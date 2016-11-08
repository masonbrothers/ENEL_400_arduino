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
#define DATA_FILE_NAME                                    "AQUADATA.TXT"
#define LOG_FILE_NAME                                     "AQUALOG.TXT"
#define TSL2591_NUMBER                                    2591

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


boolean validAmbientTemperatureHumidity = true; //Boolean to detect whether or not Reading was valid

boolean pumpIsOn;

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

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

  
  setupBuzzer();
  stopBuzzer();
  
  setupPump();
  startPump();

  setupLightSensor();
  
  setupWaterTemperatureSensor();

  lcd.print(aquaponicsClubString);
  lcd.setBacklight(WHITE);
  
  if (setupSDCard())
    Serial.println(initializedSDCardString);
  else
    Serial.println(failedInitializeSDCardString);

  setupAmbientTemperatureAndHumidity();
  
  if (setupRTC())
  {
    Serial.println(initializedRTCString);
    setRTCTime();
  }
  else
  {
    Serial.println(failedInitializeRTCString);
    writeToSDCardLog(failedInitializeRTCString);
  }

}

void loop() {

  delay(2000);

  float waterTemperature = getWaterTemperature();
  //Serial.println("Water Temp:\t" + (String)waterTemperature + "degC");

  AmbientTemperatureHumidity ambientTemperatureHumidity = readAmbientTemperatureAndHumidity();
  if (ambientTemperatureHumidity.valid)
  {
    Serial.println((String)ambientTemperatureHumidity.ambientTemperature + "degC\t" + (String)ambientTemperatureHumidity.ambientHumidity + "%");
  }
  else
  {
    Serial.println(failedTemperatureHumidityString);
    writeToSDCardLog(failedTemperatureHumidityString);
  }
  
  String Time = getRTCStringTime();
  //Serial.println(Time);
  
  if (!writeToSDCard(DATA_FILE_NAME, "Greatness!!!"))
  {
    Serial.println("Cannot write to " + (String)DATA_FILE_NAME);
  }
  
  int visibleLight = getVisibleLight();
  
  Serial.println("Visible light: " + (String)visibleLight + " lux");

  Serial.println("IR light: " + (String)getIRLight() + " lux");
  
  int waterLevelResistance = waterLevelSensorResistance();

  
  printToESP("pumpIsOn:" + (String)(int)pumpIsOn);
  printToESP("realTimeAmbientHumidity:" + (String)ambientTemperatureHumidity.ambientHumidity);
  printToESP("realTimeAmbientLight:" + (String)visibleLight);
  printToESP("realTimeAmbientTemperature:" + (String)ambientTemperatureHumidity.ambientTemperature);
  //esp8266Serial.print("realTimeSpillSensor:" + (String)visibleLight);
  printToESP("realTimeTime:" + Time);
  printToESP("realTimeWaterLevel:" + (String)waterLevelResistance);
  printToESP("realTimeWaterTemperature:" + (String)waterTemperature);
  esp8266Serial.print("PUMP");
  if (esp8266Serial.available()) {
    Serial.write(esp8266Serial.read());
  }
  
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
  return false;
}
