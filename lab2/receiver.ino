#include <RadioLib.h>

struct __attribute__((packed)) SensorData {
  float temperature;    // 4 bytes
  float humidity;     // 4 bytes
  uint32_t pressure;       // 4 bytes
  uint32_t gas;          // 4 bytes
  uint32_t TimeStamp;
};
SensorData receivedPacket;

// Module(NSS, DIO1, RST, BUSY)
SX1262 radio = new Module(8, 14, 12, 13);

void setup() {
  Serial.begin(115200);
  
  Serial.print(F("[SX1262] Initializing ... "));
  // Ensure these parameters match the sender exactly
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
  // Listen for a packet
  int state = radio.receive((uint8_t*)&receivedPacket, sizeof(receivedPacket));

  if (state == RADIOLIB_ERR_NONE) {
    char buffer[50]; // Create a buffer
    sprintf(buffer, "%.2f,%.2f,%d,%d",receivedPacket.temperature, receivedPacket.humidity, receivedPacket.pressure, receivedPacket.gas);
    // Serial.print("Data :");
    Serial.println(buffer);
    
    // Serial.print(receivedPacket.temperature);
    // Serial.println(receivedPacket.humidity);
    // Serial.println(receivedPacket.gas);
    // Serial.print(F("[SX1262] RSSI: "));
    // Serial.print(radio.getRSSI());
    // Serial.print(F(" dBm, SNR: "));
    // Serial.print(radio.getSNR());
    // Serial.println(F(" dB"));

  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // Timeout is expected if no packet is received
  } else {
    Serial.print(F("Receive failed, code "));
    Serial.println(state);
  }
}