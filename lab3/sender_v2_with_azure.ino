#include "LoRaWan_APP.h"
#include "Arduino.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#include <WiFi.h>
#include <HTTPClient.h>

uint32_t license[4] = {0x1D04EBF8,0x4CCA6B9F,0xB411D049,0x083CF19A};
// AT+CDKEY=1D04EBF84CCA6B9FB411D049083CF19A


#define RF_FREQUENCY                                433500000 // Hz

#define TX_OUTPUT_POWER                             14 //14        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       12 //12         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;

bool lora_idle=true;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );


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

  //Aqi
  float ppmCO2;   //4 bytes
};
SensorData dataPacket;



#define MQ135_PIN 7
#define RL_VALUE 1.0    // Load resistance in kohms
#define RO_CLEAN_AIR_FACTOR 3.59 // RO_CLEAR_AIR_FACTOR=(RLOAD+RS)/RS (from datasheet)

// Curve constants for  
// These are approximations derived from the MQ-135 datasheet
const float CO2_a = 110.47,  CO2_b = -2.86;



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
                     String(dataPacket.TimeStamp) + "," +
                     String(dataPacket.ppmCO2);

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

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);

    init_bme();
    // init_wifi();
    pinMode(WHITE_LED_PIN, OUTPUT);
    analogReadResolution(12);

    txNumber=0;

    // Initialize the radio
    Serial.print(F("[SX1262] Initializing ... "));
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
   }



void loop()
{
	if(lora_idle == true)
	{
    delay(1000);


    if (! bme.performReading()) {
      Serial.println(F("Failed to perform reading :("));
      return;
    }

    dataPacket.temperature = bme.temperature;
    dataPacket.pressure = bme.pressure/ 100;
    dataPacket.humidity = bme.humidity;
    dataPacket.gas = bme.gas_resistance/ 1000.0;
    dataPacket.TimeStamp = millis() / 1000;

    Serial.println(bme.temperature);
    Serial.println(bme.pressure / 100);
    // Serial.println(bme.gas_resistance / 1000.0);
    

    int rawValue = analogRead(MQ135_PIN);

  float vOut = rawValue * (3.3 / 4095.0);
  // 1. Calculate Rs (Sensor Resistance)
  // Formula: Rs = ((Vcc * RL) / Vout) - RL
  // We use 3.3V for Vcc logic, but sensor heater must be on 5V
  float rs = ((3.3 - vOut)*RL_VALUE) / vOut;
  // 2. Calculate Ro (Resistance in clean air)
  // Ideally, you should calibrate this in fresh air and hardcode the result
  float ro = 2.7;
  float ratio = rs / ro;
  // 3. Calculate PPMs
  dataPacket.ppmCO2 = CO2_a * pow(ratio, CO2_b);
  // dataPacket.ppmNH3 = NH3_a * pow(ratio, NH3_b);
  // dataPacket.ppmBenzene = Benzene_a * pow(ratio, Benzene_b);
  // dataPacket.ppmNOx = NOx_a * pow(ratio, NOx_b);

  // 4. Output to Serial
  Serial.printf("Gas Readings raw:%d \n",rawValue);
  Serial.printf("CO2: %.2f ppm\n", dataPacket.ppmCO2);
  // Serial.printf("NH3 (Ammonia): %.2f ppm\n", dataPacket.ppmNH3);
  // Serial.printf("Benzene: %.2f ppm\n", dataPacket.ppmBenzene);
  // Serial.printf("NOx: %.2f ppm\n", dataPacket.ppmNOx);
  Serial.print(F("[SX1262] Transmitting packet ... "));
  Serial.println("--------------------\n");

		Radio.Send( (uint8_t*)&dataPacket, sizeof(dataPacket) ); //send the package out	
    // send_http_post();

    lora_idle = false;
	}
  Radio.IrqProcess( );
}

void OnTxDone( void )
{
	Serial.println("TX done......");
	lora_idle = true;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}
