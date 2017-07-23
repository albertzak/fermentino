#include <OneWire.h>
#include <RCSwitch.h>
#include <SoftwareSerial.h>
#include <serLCD.h>

#define TEMPERATURE_PIN 10
#define REMOTE_SWITCH_PIN 9
#define LCD_PIN 2

// See this link on how to set the following constants
// https://github.com/sui77/rc-switch/wiki/HowTo_OperateLowCostOutlets
#define REMOTE_SWITCH_GROUP "00001"
#define REMOTE_SWITCH_OUTLET "01000"

OneWire ds(TEMPERATURE_PIN);
RCSwitch rc = RCSwitch();
serLCD lcd(LCD_PIN);

float getTemperature(void);
void startHeater(void);
void stopHeater(void);
void printTemperature(void);

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  rc.enableTransmit(REMOTE_SWITCH_PIN);
}


int flip = 1;

void loop(void) {
  while (true) {
    float temperature = getTemperature();
    Serial.println(temperature, 2);

    printTemperature(temperature);

    flip = 1 - flip;
    if (flip) {
      startHeater();
    } else {
      stopHeater();
    }
  }
}

void startHeater(void) {
  Serial.println("STARTING HEATER");
  digitalWrite(LED_BUILTIN, HIGH);

  rc.switchOn(REMOTE_SWITCH_GROUP, REMOTE_SWITCH_OUTLET);
}

void stopHeater(void) {
  Serial.println("STOPPING HEATER");
  digitalWrite(LED_BUILTIN, LOW);

  rc.switchOff(REMOTE_SWITCH_GROUP, REMOTE_SWITCH_OUTLET);
}

void printTemperature(float temperature) {
  lcd.clear();
  lcd.print(temperature, 2);

  // Degree symbol
  lcd.print((char) 223);
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

  // start conversion, with parasite power on at the end
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

