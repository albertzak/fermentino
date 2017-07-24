#include <OneWire.h>
#include <RCSwitch.h>
#include <SoftwareSerial.h>
#include <serLCD.h>

#define TEMPERATURE_MIN 15.0
#define TEMPERATURE_MAX 90.0

// See this link on how to set the following constants
// https://github.com/sui77/rc-switch/wiki/HowTo_OperateLowCostOutlets
#define REMOTE_SWITCH_GROUP "10000"
#define REMOTE_SWITCH_OUTLET "10000"

#define TEMPERATURE_PIN 10
#define REMOTE_SWITCH_PIN 9
#define LCD_PIN 2
#define POTENTIOMETER_PIN 0

OneWire ds(TEMPERATURE_PIN);
RCSwitch rc = RCSwitch();
serLCD lcd(LCD_PIN);

float getTemperature(void);
float getSetpoint(void);
void startHeater(void);
void stopHeater(void);
void printStatus(float temperature, float setpoint, char isHeating);
void printStatusLCD(float temperature, float setpoint, char isHeating);
void printStatusSerial(float temperature, float setpoint, char isHeating);
float mapFloat(float input, float input_start, float input_end, float output_start, float output_end);

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  rc.enableTransmit(REMOTE_SWITCH_PIN);
}

void loop(void) {
  while (true) {
    float temperature = getTemperature();
    float setpoint = getSetpoint();
    char isHeating;

    if (temperature < setpoint) {
      startHeater();
      isHeating = 1;
    } else {
      stopHeater();
      isHeating = 0;
    }

    printStatus(temperature, setpoint, isHeating);
  }
}

void printStatus(float temperature, float setpoint, char isHeating) {
  printStatusLCD(temperature, setpoint, isHeating);
  printStatusSerial(temperature, setpoint, isHeating);
}

void printStatusSerial(float temperature, float setpoint, char isHeating) {
  if (isHeating) {
    Serial.print("HEATING");
  } else {
    Serial.print("IDLE");
  }

  Serial.print(" - ");

  Serial.print("Current: ");
  Serial.print(temperature, 2);

  Serial.print(" - ");

  Serial.print("Setpoint: ");
  Serial.println(setpoint, 1);
}

void printStatusLCD(float temperature, float setpoint, char isHeating) {
  lcd.clear();

  // This is a visualization of the 16x2 LCD
  //        [current      min]
  //        [22.55  ->  35.97]
  lcd.print("current      min");
  lcd.print(temperature, 2);
  lcd.print("  ");

  if (isHeating) {
    lcd.print("->");
  } else {
    lcd.print("  ");
  }

  lcd.print("  ");
  lcd.print(setpoint, 2);
}

void startHeater(void) {
  digitalWrite(LED_BUILTIN, HIGH);

  rc.switchOn(REMOTE_SWITCH_GROUP, REMOTE_SWITCH_OUTLET);
}

void stopHeater(void) {
  digitalWrite(LED_BUILTIN, LOW);

  rc.switchOff(REMOTE_SWITCH_GROUP, REMOTE_SWITCH_OUTLET);
}

float getSetpoint(void) {
  int reading = analogRead(POTENTIOMETER_PIN);
  return mapFloat(reading, 0, 1023, TEMPERATURE_MIN, TEMPERATURE_MAX);
}

float mapFloat(float input, float input_start, float input_end, float output_start, float output_end) {
  float slope = 1.0 * (output_end - output_start) / (input_end - input_start);
  return output_start + slope * (input - input_start);
}

// Returns the temperature from one DS18S20 in Degrees Celsius
// from http://bildr.org/2011/07/ds18b20-arduino/
float getTemperature(void) {
  byte data[12];
  byte addr[8];

  if (!ds.search(addr)) {
    ds.reset_search();
    return 9999;
  }

  if (OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return 9999;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return 9999;
  }

  ds.reset();
  ds.select(addr);

  // Start conversion, with parasite power on at the end
  ds.write(0x44,1); 

  // Wait for temperature conversion to complete
  delay(750); 

  byte present = ds.reset();
  ds.select(addr);

  // Read 
  ds.write(0xBE); 

  // We need 9 bytes
  for (int i = 0; i < 9; i++) {
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  // Using Two's Complement
  float tempRead = ((MSB << 8) | LSB); 
  float temperature = tempRead / 16;

  return temperature;
}

