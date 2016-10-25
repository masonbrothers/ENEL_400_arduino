#define BUZZER_PIN                                        1
#define PUMP_PIN                                          2
#define ESP8266_TX_ARDUINO_RX_PIN                         
#define ESP8266_RX_ARDUINO_TX                             
#define ESP8266_RESET                                     
#define WATER_THERMOMETER_DALLAS_ONE_WIRE_PIN             
#define AMBIENT_THERMOMETER_DALLAS_ONE_WIRE_PIN           
#define WATER_LEVEL_ANALOG_PIN                            A0
#define WATER_LEVEL_RESISTOR_VALUE                        560 // Value of the resistor added in ohms    

// What pin to connect the sensor to
#define WATER_LEVEL_ANALOG_PIN                            A0 
#define MAX_RESOLUTION                                    1023

#define LUMINOSITY_I2C_PIN                                    

float waterLevelSensorResistance()
{
  float value;
  value = analogRead(WATER_LEVEL_ANALOG_PIN); // This is the raw voltage
  value = (MAX_RESOLUTION / value)  - 1;
  value = WATER_LEVEL_SENSOR_RESISTOR_VALUE / reading; // This is the resistance of the meter
  return value;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(BUZZER_PIN, OUTPUT);
  stopBuzzer();
  pinMode(PUMP_PIN, OUTPUT);
  startPump();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

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


