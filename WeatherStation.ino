/**
  WeatherStation 
  v1.0 - 2023.12.22 - Nicolas Christophe - Initial version
  v1.1 - 2024.11.11 - Board ESP8266, add OLED display

  Based on the weather station kit. 
  
  Main functions :
  - Establish a WiFi connection
  - Measure the temperature, humidity and pressure
  - Calculate the pressure delta vs expected
  - Push the measures to Arduino Cloud
  - Push the measures to ThingSpeak
  - Display data on the OLED internal screen of the ESP8266

*/

/***************************
 * Include
 **************************/

#include <Wire.h> 
#include <Arduino.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <coredecls.h>                  // settimeofday_cb()
#else
#include <WiFi.h>
#endif
#include <ESPHTTPClient.h>
#include <JsonListener.h>
#include <time.h>                       // time() ctime()
#include <sys/time.h>                   // struct timeval
#include "SSD1306Wire.h"
#include "OLEDDisplayUi.h"
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "thingProperties.h"
#include <U8g2lib.h>

/***************************
 * Settings
 **************************/

const int UPDATE_INTERVAL_AC = 20; // Update every Arduino Cloud 20s 
const int UPDATE_INTERVAL_TS = 3600; // Update Thingspeak every 1h 
const int I2C_DISPLAY_ADDRESS = 0x3c;

const boolean IS_METRIC = true;
const char* host = "api.thingspeak.com";
const char* THINGSPEAK_API_KEY = "YOUR KEY";
const float p0 = 1013.25; // reference pressure at sea level
const float currentAltitude = 142.00; // current altitude in meters in Strasbourg

// Sensors measures
float humidity = 0;
float temperature = 0;
float mPressure = 0;
float cPressure = 0;
float sPressure = 0;
float dPressure = 0;

const int SDA_PIN = 4; // D2 GPIO 4 
const int SDC_PIN = 5; // D1 GPIO 5 

const int DELTA_MIN = -10; // delta pressure
const int DELTA_MAX = 10; // delta pressure

#define TZ              2       // (utc+) TZ in hours
#define DST_MN          60      // use 60mn for summer time in some countries
#define TZ_MN           ((TZ)*60)
#define TZ_SEC          ((TZ)*3600)
#define DST_SEC         ((DST_MN)*60)
#define DHTPIN 16    // D0 GPIO16 
#define DHTTYPE DHT22
#define SCL 12 // D6 GPIO12
#define SDA 14 // D5 GPIO14

/***************************
 * Initialize
 **************************/

// Flag changed in the ticker function every 10 minutes
time_t now;
bool readyForWeatherUpdate = false;
String lastUpdate = "--";
long timeSinceLastWUpdate = 0;
long timeSinceLastACUpdate = 0;
long timeSinceLastTSUpdate = 0;
 
// Initialize the temperature/humidity sensor, pressure, screen
DHT dht(DHTPIN, DHTTYPE); // temperature
Adafruit_BMP085 bmp; // pressure
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0,SCL,SDA, U8X8_PIN_NONE); // OLED

/***************************
 * Setup
 **************************/

void setup() {
  Serial.begin(115200);
  Serial.println();

  //Pressure sensor
  if (!bmp.begin()) {
    Serial.println("[Sensor] Could not find BMP180 or BMP085 sensor at 0x77");
    while (1) {}

  }

  WiFi.begin(SSID, PASS); // WIFI connection

  int counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("... ");
    counter++;

  }

  //updateData(&display);
  u8g2.setBusClock(600000);
  u8g2.setI2CAddress(0x78);
  u8g2.begin(); 
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x14B_tf);
  u8g2.drawStr(20, 14," Loading "); // Display resolution is 64 x 128
  u8g2.sendBuffer();
  delay(3000);

  // Arduino Cloud : connect
  initProperties(); // Defined in thingProperties.h
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(4);
  ArduinoCloud.printDebugInfo();

}

/***************************
 * Loop
 **************************/

void loop() {
  if (millis() - timeSinceLastACUpdate > (1000L*UPDATE_INTERVAL_AC)) {
      setTempHumidityPressure(); 
      updateArduinoCloud();
      timeSinceLastACUpdate = millis();

    }

  if (millis() - timeSinceLastTSUpdate > (1000L*UPDATE_INTERVAL_TS)) {
      setTempHumidityPressure(); 
      updateThingspeak();
      timeSinceLastTSUpdate = millis();
      
    }
}

void setTempHumidityPressure() {
    // Sensors : read values
    humidity = dht.readHumidity() / 1024.0 * 69;
    temperature = dht.readTemperature(!IS_METRIC) / 1024.0 * 50.0 * 0.8;
    mPressure = bmp.readPressure(); // measured pressure at altitude
    cPressure = mPressure / 100 * 0.501 ; // conversion
    sPressure = cPressure / pow((1-currentAltitude/44330), 5.255); // sea level equivalent pressure
    dPressure = sPressure - p0; // pressure difference at sea level

    // forecast
    String f = "Forecast :-!";
    if ( dPressure >= DELTA_MAX ) {
      f = "Forecast :-)";
    } 

    if ( dPressure <= DELTA_MIN ) {
      f = "Forecast :-(";
    } 

    // Sensors : inform the user
    Serial.println("");
    Serial.print("[Sensor] Humidity: ");
    Serial.print(dht.readHumidity());
    Serial.print(" converted to ");  
    Serial.println(humidity);
    Serial.print("[Sensor] Temperature: ");
    Serial.print(dht.readTemperature(!IS_METRIC));
    Serial.print(" converted to "); 
    Serial.println(temperature);
    Serial.print("[Sensor] mPressure: ");
    Serial.println(mPressure);
    Serial.print("[Sensor] cPressure: ");
    Serial.print(cPressure);  
    Serial.print(" converted to sea level ");
    Serial.println(sPressure);
    Serial.print("[Sensor] dPressure: ");
    Serial.println(dPressure);
    
    // header OLED
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.setCursor(20,14);
    u8g2.print(f);
    u8g2.setCursor(30,28);
    
    // measures OLED
    u8g2.print(humidity);
    u8g2.print(" %");
    u8g2.setCursor(30,42);
    u8g2.print(temperature);
    u8g2.print(" C");
    u8g2.setCursor(30,60);
    u8g2.print(cPressure);
    u8g2.print(" hPa");
    u8g2.sendBuffer();
  
}

void updateArduinoCloud() {
    // Arduino Cloud : sensor update
    acIMessenger = "Hey from Bella :) Connection done. Will update the temperature, humidity and pressure";        
    acHumidity = humidity;
    acTemperature = temperature;
    acPressure = cPressure;
    acCtemperature = temperature;

    ArduinoCloud.update();
    delay(1000);
    Serial.println("[Arduino Cloud ] Update done");
    ArduinoCloud.printDebugInfo();

}

void updateThingspeak() {
    // ThingSpeak : use WiFiClient class to create TCP connections
    WiFiClient client;
    const int httpPort = 80;
    if (!client.connect(host, httpPort)) {
      Serial.println("[Wifi] Connection failed");
      return;

    }

    // ThingSpeak : we now create a URI for the request
    String url = "/update?api_key=";
    url += THINGSPEAK_API_KEY;
    url += "&field1=";
    url += String(temperature);
    url += "&field2=";
    url += String(humidity);
    url += "&field3=";
    url += String(cPressure);
    url += "&field4=";
    url += String(dPressure);

    Serial.print("[ThingSpeak] Requesting URL: ");
    Serial.println(url);
    
    // ThingSpeak : this will send the request to the server
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" + 
                 "Connection: close\r\n\r\n");
    delay(10);
    while(!client.available()){
      delay(100);
      Serial.print("... ");

    }

    // ThingSpeak : read all the lines of the reply from server and print them to Serial
    while(client.available()){
      String line = client.readStringUntil('\r');
      Serial.print(line);

    }
      
    Serial.println("");
    Serial.println("[ThingSpeak] Closing connection");

}

