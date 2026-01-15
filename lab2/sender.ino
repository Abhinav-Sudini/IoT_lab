#include <RadioLib.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// Define I2C pins explicitly for Heltec V3 headers
#define BME_SDA 41
#define BME_SCL 42

Adafruit_BME680 bme; 

struct __attribute__((packed)) SensorData {
  float temperature;    // 4 bytes
  float humidity;     // 4 bytes
  uint32_t pressure;       // 4 bytes
  uint32_t gas;          // 4 bytes
  uint32_t TimeStamp;    // 4 bytes
};
SensorData dataPacket;


void init_bme(){

  // Initialize I2C with specific pins (SDA, SCL)
  Serial.println(F("BME680 init"));
  Wire.begin(BME_SDA, BME_SCL);

  // Initialize BME680
  // 0x77 is the default address. If it fails, try 0x76.
  if (!bme.begin(0x77)) { 
    Serial.println(F("Could not find a valid BME680 sensor, check wiring or try address 0x76!"));
    while (1);
  }

  // Set up oversampling and filter initialization (Standard settings)
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

SX1262 radio = new Module(8, 14, 12, 13);
void setup() {
  Serial.begin(115200);
  while (!Serial); 

  init_bme();

  // Initialize the radio
  Serial.print(F("[SX1262] Initializing ... "));
  int state = radio.begin(915.0, 125.0, 10, 7, 0x12, 17, 8);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
}

void loop() {
  if (! bme.performReading()) {
    Serial.println(F("Failed to perform reading :("));
    return;
  }

  dataPacket.temperature = bme.temperature;
  dataPacket.pressure = bme.pressure/ 100;
  dataPacket.humidity = bme.humidity;
  dataPacket.gas = bme.gas_resistance/ 1000.0;
  dataPacket.TimeStamp = millis() / 1000;

  // Serial.println(bme.temperature);
  // Serial.println(bme.pressure / 100);
  // Serial.println(bme.gas_resistance / 1000.0);
  // Serial.print(F("[SX1262] Transmitting packet ... "));

  int state = radio.transmit((uint8_t*)&dataPacket, sizeof(dataPacket));

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    Serial.println(F("too long!"));
  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    Serial.println(F("timeout!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  delay(1000);
}