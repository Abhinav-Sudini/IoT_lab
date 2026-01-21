

#include <RadioLib.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#include <WiFi.h>
#include <HTTPClient.h>

// Define I2C pins explicitly for Heltec V3 headers
#define BME_SDA 41
#define BME_SCL 42

#define WHITE_LED_PIN 35

Adafruit_BME680 bme; 

// ------------------- CONFIGURATION -------------------
// 1. Wi-Fi Credentials
const char* ssid     = "IITP-WiFi";
// const char* password = "YOUR_WIFI_PASSWORD";

const char* serverUrl = "http://74.225.137.156/iot/api/";

struct __attribute__((packed)) SensorData {
  float temperature;    // 4 bytes
  float humidity;     // 4 bytes
  uint32_t pressure;       // 4 bytes
  uint32_t gas;          // 4 bytes
  uint32_t TimeStamp;    // 4 bytes
};
SensorData dataPacket;


void init_wifi() {
  Serial.print(F("[WiFi] Connecting to "));
  Serial.println(ssid);
  
  WiFi.begin(ssid);

  // Wait for connection (timeout after 10s to avoid hanging forever)
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("\n[WiFi] Connected!"));
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("\n[WiFi] Connection Failed. Will continue with LoRa only."));
  }
}

void blink_led(){
  digitalWrite(WHITE_LED_PIN, HIGH);  // Turn the LED on (Voltage Level = 3.3V)
  delay(100);                        // Wait for a second
  digitalWrite(WHITE_LED_PIN, LOW);   // Turn the LED off (Voltage Level = 0V)
}

void send_http_post() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // Start connection
    http.begin(serverUrl);
    
    // Set headers (MIME type for CSV)
    http.addHeader("Content-Type", "text/csv");

    // Construct CSV String: "temp,pressure,humidity,gas"
    String payload = String(dataPacket.temperature) + "," + 
                     String(dataPacket.pressure) + "," + 
                     String(dataPacket.humidity) + "," + 
                     String(dataPacket.gas) + "," +
                     String(dataPacket.TimeStamp);

    Serial.print(F("[HTTP] Posting CSV: "));
    Serial.println(payload);

    // Send POST
    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0) {
      Serial.print(F("[HTTP] Response code: "));
      Serial.println(httpResponseCode);
      if (httpResponseCode == 200 ){
        blink_led();
      }
    } else {
      Serial.print(F("[HTTP] Error code: "));
      Serial.println(httpResponseCode);
    }
    
    // Free resources
    http.end();
  } else {
    // while(WiFi.status() != WL_CONNECTED){
      delay(1000);
      Serial.println(F("[HTTP] WiFi Disconnected . trying to reconect"));
      init_wifi();
    // }
  }
}


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
  // init_wifi();
  pinMode(WHITE_LED_PIN, OUTPUT);

  // Initialize the radio
  Serial.print(F("[SX1262] Initializing ... "));
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
    // Serial.println(F("success!"));
  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    Serial.println(F("too long!"));
  } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    Serial.println(F("timeout!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  // send_http_post();

  delay(1000);
}