#include <OneWire.h>

#define TEMPERATURE_PIN 10

OneWire ds(TEMPERATURE_PIN);

float getTemperature(void);

void setup(void) {
  Serial.begin(9600);
}

void loop(void) {
  while (true) {
    float temperature = getTemperature();
    Serial.println(temperature, 2);
  }
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
