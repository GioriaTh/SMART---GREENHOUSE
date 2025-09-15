#include <WiFi.h>
#include <Wire.h>
#include <time.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>
#include <CHT8305.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include "FS.h"
#include "SPIFFS.h"

// WiFi & MQTT Configuration
const char* ssid = "";
const char* password = "";

// NTP Client Setup
const long utcOffsetInSeconds = 2 * 3600;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "0.gr.pool.ntp.org", utcOffsetInSeconds, 60000); // Sync every 60 sec



// Define I2C pins for the first bus
#define I2C_SDA1 21
#define I2C_SCL1 22

// Define I2C pins for the second bus
#define I2C_SDA2 16
#define I2C_SCL2 17

// NTC Thermistor Pins (ADC1 only)
const int thermistor_pins[12] = { 39, 36, 35, 34, 32, 33, 25, 26, 27, 4, 13, 15 };

// Magnetic door sensor pin
#define DOOR_SENSOR_PIN 5

// Define anemometer pin
#define ANEMOMETER_PIN 14

// I2C address for the CHT8305 sensor
#define CHT8305_ADDRESS 0x40

// NTC Constants
const float BETA = 3950;  // Beta coefficient
const float ROOM_TEMP = 298.15;
const float R_NOMINAL = 10000;  // 10kΩ at 25°C


// variables  Calculated wind speed
float windSpeed = 0;
const float voltageMin = 0.0;  // Minimum voltage output at 0 m/s
const float voltageMax = 5;    // Maximum voltage output at max wind speed (e.g., 30 m/s)
const float speedMin = 0.0;    // Minimum wind speed (0 m/s)
const float speedMax = 30.0;   // Maximum wind speed (30 m/s)


// Initialize TwoWire instances for each I2C bus
TwoWire I2C_1 = Wire;        // Default I2C instance for bus 1
TwoWire I2C_2 = TwoWire(1);  // Second I2C instance for bus 2


WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_TSL2591 tsl1 = Adafruit_TSL2591(2591);  // Initialize 1st TSL2591 sensor
Adafruit_TSL2591 tsl2 = Adafruit_TSL2591(2591);  // Initialize 2nd TSL2591 sensor



// Timing
struct tm timeinfo;
unsigned long lastMillis = 0;
unsigned long lastNTPUpdate = 0;
unsigned long lastSaveTime = 0;
unsigned long lastUploadTime = 0;
const unsigned long SAVE_INTERVAL = 3* 1000;         // Save every 1 min
const unsigned long UPLOAD_INTERVAL = 1* 60 * 1000;  // Upload every 5 minutes

void setup() {
  Serial.begin(115200);
  connectWiFi();
  syncTime();

  // Initialize I2C communication on the first bus
  I2C_1.begin(I2C_SDA1, I2C_SCL1);
  I2C_1.setClock(50000);
  i2cScan(I2C_1);

  // Initialize I2C communication on the second bus
  I2C_2.begin(I2C_SDA2, I2C_SCL2);
  I2C_2.setClock(25000);
  delay(100);
  i2cScan(I2C_2);

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed!");
    return;
  }
  Serial.println("SPIFFS Mounted Successfully");



  // Configure both CHT8305 sensors
  configureSensor(I2C_1);
  configureSensor(I2C_2);
  delay(100);


  // Initialize the TSL2591 sensor
  if (!tsl1.begin(&I2C_1)) {
    Serial.println("Failed to find TSL2591 sensor!");
    while (1)
      ;
  }
  configureTSL2591(tsl1);  // Configure TSL2591 settings

  // Initialize the second TSL2591 sensor
  if (!tsl2.begin(&I2C_2)) {
    Serial.println("Failed to find second TSL2591 sensor!");
    while (1)
      ;
  }
  configureTSL2591(tsl2);

  // Initialize magnetic door sensor pin
  pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);

  //Initalize wind sensor
  analogReadResolution(12);  // Set ADC resolution to 12 bits (0-4095)



  // Start disconnected
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi OFF, starting data collection...");
}

// Sync Time from NTP
void syncTime() {
    timeClient.begin();
    timeClient.update();

    time_t rawTime = timeClient.getEpochTime();
    struct timeval tv = { rawTime, 0 };
    settimeofday(&tv, NULL);  // Set ESP32 RTC

    Serial.println(" Time synchronized with NTP!");
    lastNTPUpdate = millis(); // Save last sync time
}

void configureSensor(TwoWire& i2c_bus) {
  Serial.println("Configuring CHT8305...");
  i2c_bus.beginTransmission(CHT8305_ADDRESS);
  i2c_bus.write(0x03);
  i2c_bus.write(0x10);
  int error = i2c_bus.endTransmission();

  if (error == 0) {
    Serial.println("CHT8305 successfully configured.");
  } else {
    Serial.printf("CHT8305 I2C error: %d\n", error);
    if (error == 2) Serial.println("NACK received (wrong address?)");
    if (error == 3) Serial.println("I2C bus is busy (try resetting)");
  }
}


void configureTSL2591(Adafruit_TSL2591& tsl) {
  tsl.setGain(TSL2591_GAIN_MED);                 // Medium gain (recommended for indoor lighting)
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // Shorter integration time for faster readings
  Serial.println("TSL2591 sensor configured successfully");
}

void i2cScan(TwoWire& i2c_bus) {
  byte error, address;
  int nDevices = 0;
  Serial.println("Scanning for I2C devices...");
  for (address = 1; address < 127; address++) {
    i2c_bus.beginTransmission(address);
    error = i2c_bus.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found");
  else
    Serial.println("Done scanning.");
}


void resetI2C(TwoWire& i2c_bus) {
  i2c_bus.end();
  delay(10);
  i2c_bus.begin();
  i2c_bus.setClock(50000);
  Serial.println("I2C bus reset");
}

void loop() {


  unsigned long currentMillis = millis();

  // Update Time Every Second
    if (currentMillis - lastMillis >= 1000) {
        lastMillis = currentMillis;
        Serial.println(getFormattedTime());  // Show updated time
    }

  //  Collect and save sensor data every second
  if (currentMillis - lastSaveTime >= SAVE_INTERVAL) {
    lastSaveTime = currentMillis;
    saveSensorData();
  }

  //  Every 5 minutes, connect to WiFi, upload, and disconnect
  if (currentMillis - lastUploadTime >= UPLOAD_INTERVAL) {
    lastUploadTime = currentMillis;
    uploadDataFromSPIFFS();
  }
}


void saveSensorData() {
  File file = SPIFFS.open("/ntc_data.txt", FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for writing!");
    return;
  }

  StaticJsonDocument<512> jsonDoc;            
  jsonDoc["timestamp"] = getFormattedTime();  

  //  Fix: Declare `i` inside the loop
  for (int i = 0; i < 12; i++) {
    float temp = readThermistorTemperature(thermistor_pins[i]);  
    jsonDoc[String("temp") + String(i + 1)] = temp;               
    Serial.printf("Thermistor %d: %.2f°C\n", i + 1, temp);
  }


  // Read CHT8305 sensors
  float temperature_in, humidity_in, temperature_out, humidity_out;
  if (measureTemperature(I2C_1, temperature_in)) jsonDoc["chtIn_temp"] = round(temperature_in * 10) / 10;
  if (measureHumidity(I2C_1, humidity_in)) jsonDoc["chtIn_humidity"] = round(humidity_in * 10) / 10;
  ;
  if (measureTemperature(I2C_2, temperature_out)) jsonDoc["chtOut_temp"] = round(temperature_out * 10) / 10;
  if (measureHumidity(I2C_2, humidity_out)) jsonDoc["chtOut_humidity"] = round(humidity_out * 10) / 10;

  //Read doosr State
  int doorState = digitalRead(DOOR_SENSOR_PIN);
  jsonDoc["Door"] = doorState;

  // Read TSL2591 sensors
  uint16_t ir_in, full1, ir_out, full2;
  ir_in = measureTSL2591(tsl1, ir_in, full1);
  ir_out = measureTSL2591(tsl2, ir_out, full2);
  jsonDoc["ir_in"] = ir_in;
  jsonDoc["ir_out"] = ir_out;

  //Read Wind
  float wind = calculateWindSpeed();
  jsonDoc["wind"] = wind;

  // Print JSON to Serial Monitor before saving
  Serial.println("Generated JSON:");
  serializeJsonPretty(jsonDoc, Serial);
  Serial.println();

  // Save to SPIFFS
  String jsonData;
  serializeJson(jsonDoc, jsonData);
  file.println(jsonData);
  file.close();

  Serial.println("Data saved to SPIFFS!\n");
}

void uploadDataFromSPIFFS() {
  if (!SPIFFS.exists("/ntc_data.txt")) {
    Serial.println("No stored data to upload.");
    return;
  }

  File file = SPIFFS.open("/ntc_data.txt", FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading!");
    return;
  }

  Serial.println("Uploading all data to MySQL...");

  connectWiFi();  // Ensure WiFi is connected

  String postData = "[";
  bool firstEntry = true;

  while (file.available()) {
    String line = file.readStringUntil('\n');
    StaticJsonDocument<512> jsonDoc;
    DeserializationError error = deserializeJson(jsonDoc, line);

    if (error) {
      Serial.println("JSON parsing failed!");
      continue;
    }

    if (!firstEntry) postData += ",";
    firstEntry = false;

    String entry = "{";
    entry += "\"timestamp\":\"" + jsonDoc["timestamp"].as<String>() + "\",";
    entry += "\"temp1\":" + String(jsonDoc["temp1"].as<float>()) + ",";
    entry += "\"temp2\":" + String(jsonDoc["temp2"].as<float>()) + ",";
    entry += "\"temp3\":" + String(jsonDoc["temp3"].as<float>()) + ",";
    entry += "\"temp4\":" + String(jsonDoc["temp4"].as<float>()) + ",";
    entry += "\"temp5\":" + String(jsonDoc["temp5"].as<float>()) + ",";
    entry += "\"temp6\":" + String(jsonDoc["temp6"].as<float>()) + ",";
    entry += "\"temp7\":" + String(jsonDoc["temp7"].as<float>()) + ",";
    entry += "\"temp8\":" + String(jsonDoc["temp8"].as<float>()) + ",";
    entry += "\"temp9\":" + String(jsonDoc["temp9"].as<float>()) + ",";
    entry += "\"temp10\":" + String(jsonDoc["temp10"].as<float>()) + ",";
    entry += "\"temp11\":" + String(jsonDoc["temp11"].as<float>()) + ",";
    entry += "\"temp12\":" + String(jsonDoc["temp12"].as<float>()) + ",";
    entry += "\"chtIn_humidity\":" + String(jsonDoc["chtIn_humidity"].as<float>()) + ",";
    entry += "\"chtIn_temp\":" + String(jsonDoc["chtIn_temp"].as<float>()) + ",";
    entry += "\"chtOut_humidity\":" + String(jsonDoc["chtOut_humidity"].as<float>()) + ",";
    entry += "\"chtOut_temp\":" + String(jsonDoc["chtOut_temp"].as<float>()) + ",";
    entry += "\"ir_in\":" + String(jsonDoc["ir_in"].as<uint16_t>()) + ",";
    entry += "\"ir_out\":" + String(jsonDoc["ir_out"].as<uint16_t>()) + ",";
    entry += "\"door\":" + String(jsonDoc["door"].as<int>()) + ",";
    entry += "\"wind\":" + String(jsonDoc["wind"].as<float>()) + "}";
    
    postData += entry;
  }

  postData += "]";  // Close JSON array
  file.close();

  // Send bulk data to MySQL
  HTTPClient http;
  http.begin("http://192.x.x.x/db_sensor_data.php");  // Change to your server URL
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.POST(postData);
  if (httpResponseCode > 0) {
    Serial.print(" MySQL Upload Response: ");
    Serial.println(httpResponseCode);
    SPIFFS.remove("/ntc_data.txt");  // Delete after successful upload
    Serial.println(" Data uploaded & file deleted.");
  } else {
    Serial.print(" HTTP Error: ");
    Serial.println(httpResponseCode);
  }

  http.end();
  disconnectWiFi();  // Disconnect after sending
}


// Read thermistor temperature
float readThermistorTemperature(int pin) {
  WiFi.setSleep(true);  // Ensure WiFi is sleeping so ADC2 works
  int analogValue = analogRead(pin);
  WiFi.setSleep(false);

  if (analogValue == 0) return -273.15;

  float resistance = R_NOMINAL * (4095.0 / analogValue - 1.0);
  float temperatureK = 1.0 / (1.0 / ROOM_TEMP + (1.0 / BETA) * log(resistance / R_NOMINAL));
  return temperatureK - 273.15;
}

// Measure humidity
bool measureHumidity(TwoWire& i2c_bus, float& humidity) {
  i2c_bus.beginTransmission(CHT8305_ADDRESS);
  i2c_bus.write(0x01);
  if (i2c_bus.endTransmission() != 0) {
    Serial.println("Failed to trigger humidity measurement");
    return false;
  }
  delay(20);

  i2c_bus.requestFrom(CHT8305_ADDRESS, 2);
  if (i2c_bus.available() == 2) {
    uint16_t rawHumidity = (i2c_bus.read() << 8) | i2c_bus.read();
    humidity = ((125.0 * rawHumidity) / 65536.0) - 6.0;
    return true;
  } else {
    Serial.println("Failed to read humidity data");
    return false;
  }
}


// Measure temperature CH8305
bool measureTemperature(TwoWire& i2c_bus, float& temperature) {
  i2c_bus.beginTransmission(CHT8305_ADDRESS);
  i2c_bus.write(0x00);
  if (i2c_bus.endTransmission() != 0) {
    Serial.println("Failed to trigger temperature measurement");
    return false;
  }
  delay(20);

  i2c_bus.requestFrom(CHT8305_ADDRESS, 2);
  if (i2c_bus.available() == 2) {
    uint16_t rawTemperature = (i2c_bus.read() << 8) | i2c_bus.read();
    temperature = ((175.72 * rawTemperature) / 65536.0) - 46.85;
    return true;
  } else {
    Serial.println("Failed to read temperature data");
    return false;
  }
}


// Measure lux and infrared values from TSL2591
float measureTSL2591(Adafruit_TSL2591& sensor, uint16_t& ir, uint16_t& full) {
  full = sensor.getFullLuminosity();
  ir = full >> 16;  // Get IR part
  uint16_t visible = full & 0xFFFF;
  return sensor.calculateLux(visible, ir);
}

float calculateWindSpeed() {
  // Read the analog voltage
  int analogValue = analogRead(ANEMOMETER_PIN);
  float voltage = analogValue * (3.3 / 4095.0);  // Convert ADC value to voltage (3.3V reference)

  // Calculate wind speed based on the voltage
  float windSpeed = (voltage - voltageMin) * (speedMax - speedMin) / (voltageMax - voltageMin) + speedMin;

  return windSpeed;  // Return the calculated wind speed
}

// Get Formatted Time After WiFi Disconnects
String getFormattedTime() {
    time_t now;
    time(&now);
    localtime_r(&now, &timeinfo);

    char timeBuffer[20];
    snprintf(timeBuffer, sizeof(timeBuffer), "%04d-%02d-%02d %02d:%02d:%02d",
             timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    return String(timeBuffer);
}

// Connect to WiFi
void connectWiFi() {
  Serial.println("Reconnecting to WiFi...");
  WiFi.disconnect();
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
  } else {
    Serial.println("\nWiFi Failed!");
  }

}

// Disconnect WiFi
void disconnectWiFi() {
  Serial.println("Disconnecting WiFi...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("WiFi OFF, back to collecting data...");
}

