#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h" //Temperature and Humidity Sensor

#include <SPI.h>
#include <SD.h>


#define BUZZER_PIN                                        5
#define PUMP_PIN                                          4
#define ESP8266_TX_ARDUINO_RX_PIN                         
#define ESP8266_RX_ARDUINO_TX                             
#define ESP8266_RESET                                     
#define WATER_THERMOMETER_DALLAS_ONE_WIRE_PIN             2
#define AMBIENT_THERMOMETER_DALLAS_ONE_WIRE_PIN           
#define WATER_LEVEL_ANALOG_PIN                            A0
#define WATER_LEVEL_RESISTOR_VALUE                        560 // Value of the resistor added in ohms    

// What pin to connect the sensor to
#define WATER_LEVEL_ANALOG_PIN                            A0 
#define MAX_RESOLUTION                                    1023
#define ADAFRUIT_DATA_LOGGING_SHIELD_CHIP_SELECT          10
// Note that this pin needs a 10K pullup Resistor 
#define DHTPIN                                            3
#define DHTTYPE                                           DHT22
#define LUMINOSITY_I2C_PIN                                    

OneWire oneWire(WATER_THERMOMETER_DALLAS_ONE_WIRE_PIN);

DallasTemperature sensors(&oneWire);

DHT dht(DHTPIN, DHTTYPE);

float AmbientTemperature;
float AmbientHumidity;
boolean validAmbientTemperatureHumidity = true; //Boolean to detect whether or not Reading was valid


float waterLevelSensorResistance()
{
  float value;
  value = analogRead(WATER_LEVEL_ANALOG_PIN); // This is the raw voltage
  value = (MAX_RESOLUTION / value)  - 1;
  value = WATER_LEVEL_RESISTOR_VALUE / value; // This is the resistance of the meter
  return value;
}

void setup() {
  // put your setup code here, to run once:
  /*
  pinMode(BUZZER_PIN, OUTPUT);
  stopBuzzer();
  pinMode(PUMP_PIN, OUTPUT);
  startPump();
  */
  sensors.begin();
  Serial.begin(9600);

  setupSDCard();

  dht.begin();

  
}

boolean setupSDCard()
{
  Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(ADAFRUIT_DATA_LOGGING_SHIELD_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return false;
  }
  Serial.println("card initialized.");
  return true;
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors.requestTemperatures();
  Serial.print((String)sensors.getTempCByIndex(0) + "\n");
  delay(1000);
  ReadAmbientTemperatureAndHumidity();
  Serial.println((String)AmbientTemperature + "degC\t" + (String)AmbientHumidity + "%");
  writeToSDCard("Aqua.txt", "Greatness!!!");
}




void ReadAmbientTemperatureAndHumidity()
{
  //Sensor requires a delay of atleast 2 seconds to work
  delay(2000);
  AmbientTemperature = dht.readTemperature();
  AmbientHumidity = dht.readHumidity();
  
  //If Ambient Temperature and Humidity Readings are garbage output an error message
  if (isnan(AmbientTemperature) || isnan(AmbientHumidity))
  {
    Serial.println("Ambient Temperature and Humidity Sensor Failed!");
    validAmbientTemperatureHumidity = false;
    return;
  }
} 


/*
bool saveData(SensorData sensorData) {
  
}
*/
void startBuzzer() {
  digitalWrite(BUZZER_PIN, HIGH);
}

void stopBuzzer() {
  digitalWrite(BUZZER_PIN, LOW);
}

void startPump() {
  digitalWrite(PUMP_PIN, HIGH);
}

void stopPump() {
  digitalWrite(PUMP_PIN, LOW);
}

boolean writeToSDCard(String fileName, String dataToWrite)
{
  File dataFile = SD.open(fileName.c_str(), FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataToWrite);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataToWrite);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening" + fileName);
    return false;
  }
  return true;
}


