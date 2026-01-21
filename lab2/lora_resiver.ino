// #include <SPI.h>
// #include <LoRa.h>

// #define LORA_SCK   9
// #define LORA_MISO  11
// #define LORA_MOSI  10
// #define LORA_SS    8
// #define LORA_RST   12
// #define LORA_DIO0  14
// #define LORA_PWR   21

// void setup() {
//   Serial.begin(115200);
//   delay(2000);
//   Serial.println("LoRa Receiver");

//   SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
//   LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

//   Serial.println("LoRa init start");
//   while (!LoRa.begin(915E6)) {
//     Serial.print(".");
//     delay(250);
//   }

//   Serial.println("LoRa init OK");
// }

// void loop() {
//   int packetSize = LoRa.parsePacket();
//   if (packetSize) {
//     Serial.print("Received: ");

//     while (LoRa.available()) {
//       Serial.print((char)LoRa.read());
//     }

//     Serial.print(" | RSSI: ");
//     Serial.println(LoRa.packetRssi());
//   }
// }

// // #include <SPI.h>
// // #include <LoRa.h>

// // //define the pins used by the transceiver module
// // #define ss 5
// // #define rst 14
// // #define dio0 2

// // void setup() {
// //   //initialize Serial Monitor
// //   Serial.begin(115200);
// //   while (!Serial);
// //   Serial.println("LoRa Receiver");

// //   //setup LoRa transceiver module
// //   LoRa.setPins(ss, rst, dio0);
  
// //   //replace the LoRa.begin(---E-) argument with your location's frequency 
// //   //433E6 for Asia
// //   //868E6 for Europe
// //   //915E6 for North America
// //   while (!LoRa.begin(868E6)) {
// //     Serial.println(".");
// //     delay(500);
// //   }
// //    // Change sync word (0xF3) to match the receiver
// //   // The sync word assures you don't get LoRa messages from other LoRa transceivers
// //   // ranges from 0-0xFF
// //   LoRa.setSyncWord(0xF3);
// //   Serial.println("LoRa Initializing OK!");
// // }

// // void loop() {
// //   // try to parse packet
// //   int packetSize = LoRa.parsePacket();
// //   if (packetSize) {
// //     // received a packet
// //     Serial.print("Received packet '");

// //     // read packet
// //     while (LoRa.available()) {
// //       String LoRaData = LoRa.readString();
// //       Serial.print(LoRaData); 
// //     }

// //     // print RSSI of packet
// //     Serial.print("' with RSSI ");
// //     Serial.println(LoRa.packetRssi());
// //   }
// // }

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
  int state = radio.begin(865.5, 125.0, 10, 7, 0x13, 17, 8);

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
    sprintf(buffer, "%.2f,%.2f,%d,%d,%d",receivedPacket.temperature, receivedPacket.humidity, receivedPacket.pressure, receivedPacket.gas, receivedPacket.TimeStamp);
    Serial.print("Data :");
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