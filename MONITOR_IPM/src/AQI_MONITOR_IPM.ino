//AirBuddi2
//GPSGPRS  —  AQI Monitor IPM
//  OTA + AWS wired to match Airbuddimax pattern.

#include "bsec.h"
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include "FreeRTOS.h"
#include <WiFiManager.h>
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include "esp_mac.h"
#include <Preferences.h>
#include <time.h>
#include <HTTPClient.h>
#include <Update.h>


// =====================================================
// FIRMWARE INFORMATION
// =====================================================

#define DEVICE_MODEL    "Monitor"
#define FIRMWARE_VERSION "1.0.0"

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// =====================================================
// HTTPS OTA
// =====================================================

volatile bool otaInProgress = false;
volatile bool otaRequested   = false;

String otaFirmwareUrl = "";

TaskHandle_t otaTaskHandle = NULL;

#define OTA_TASK_PRIORITY  5
#define OTA_TASK_STACK_SIZE 12288

void performOTA(String firmwareUrl);
void otaTask(void *pvParameters);

// ===================== AWS IoT =====================

String AWS_IOT_PUBLISH_TOPIC;
String AWS_IOT_SUBSCRIBE_TOPIC;
String AWS_IOT_STATUS_TOPIC;

String mac = "";
int connectedFlag = 0;
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

bool connectAWS();
bool syncTimeForTLS();
void printAwsNetworkDiagnostics();
const char* mqttStateMessage(int state);
void messageHandler(char* topic, byte* payload, unsigned int length);
void publishMessage();
void publishStatus();
bool statusChanged();
void prepareHardwareForOTA();
void AWSTask(void *pvParameters);
void WifiManagerTask(void *pvParameters);

HardwareSerial dwin(2);
HardwareSerial ipm(1); // Renamed to 'ipm' for clarity. Also, you used Serial2 but defined dwin as port 2. Using a new HardwareSerial instance on port 1 for the IPM sensor is a better practice.



unsigned char Buffer[9];
uint8_t p = 5;



//BME 688
void checkIaqSensorStatus(void);
void errLeds(void);
Bsec iaqSensor;
String output;

//HPMA
#define RXD2 16
#define DWIN_RX   23    // ESP32 pin wired to the display's TX
#define DWIN_TX   13    // ESP32 pin wired to the display's RX
#define DWIN_TEST 0    // 1 = send fixed test frame, 0 = normal operation
#define TXD2 17
int PM1;
int PM25;
int PM4;
int PM10;

//RGB
#define LED_PIN 5   // 5 for new code (prevents conflict with DWIN_TX on GPIO 13)
#define LED_COUNT 16
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

//LCD
float pressure;
float temperature;
float Humidity;
float Gasresistance;
float IAQ;
float Co2;
float Vocs;

void bmeTask(void *pvParameters);
void ipmTask(void *pvParameters);
void displayTask(void *pvParameters);
void ledTask(void *pvParameters);
void WifiManagerTask(void *pvParameters);
void AWSTask(void *pvParameters);
void displaycontrol(void *parameter);
void startMeasurement();
void requestPMData();
void sendMacToDwin();

// Task handles
TaskHandle_t bmeTaskHandle;
TaskHandle_t ipmTaskHandle;
TaskHandle_t displayTaskHandle;
TaskHandle_t ledTaskHandle;
TaskHandle_t WifiManagerTaskHandle = NULL;
TaskHandle_t AWSTaskHandle         = NULL;
TaskHandle_t controltaskhandle;

String getDefaultMacAddress() {
    uint8_t mac_base[6] = {0};
    if (esp_efuse_mac_get_default(mac_base) != ESP_OK) return "";
    String mac_str = "";
    for (int i = 0; i < 6; i++) {
        if (i > 0) mac_str += ":";
        if (mac_base[i] < 0x10) mac_str += "0";
        mac_str += String(mac_base[i], HEX);
    }
    mac_str.toUpperCase();
    return mac_str;
}

// BME 688
// Helper functions declarations
void checkIaqSensorStatus(void) {
  if (iaqSensor.bsecStatus != BSEC_OK) {
    if (iaqSensor.bsecStatus < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.bsecStatus);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.bsecStatus);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme68xStatus != BME68X_OK) {
    if (iaqSensor.bme68xStatus < BME68X_OK) {
      output = "BME68X error code : " + String(iaqSensor.bme68xStatus);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME68X warning code : " + String(iaqSensor.bme68xStatus);
      Serial.println(output);
    }
  }
}

void errLeds(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

//VARIABLES TO SEND DATA TO DWINN DISPLAY
unsigned char a[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X20, 0X00, 0X00  //Serial output prefix for gas resistance
};
unsigned char b[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X21, 0X00, 0X00  //Serial output prefix for pressure
};
unsigned char c[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X30, 0X00, 0X00  //Serial output prefix for temprature
};
unsigned char d[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X22, 0X00, 0X00  //Serial output prefix for humidity
};
unsigned char e[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X24, 0X00, 0X00  //Serial output prefix for IAQ
};
unsigned char f[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X25, 0X00, 0X00  //Serial output prefix for CO2
};
unsigned char k[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X26, 0X00, 0X00  //Serial output prefix for VOC
};
unsigned char l[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X27, 0X00, 0X00  //Serial output prefix for PM 2.5
};
unsigned char m[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X28, 0X00, 0X00  //Serial output prefix for PM 1.0
};
unsigned char n[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X23, 0X00, 0X00  //Serial output prefix to PM1
};
unsigned char o[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X29, 0X00, 0X00  //Serial output prefix to PM4
};

// =====================================================
// OTA TASK – stop tasks, then perform update
// =====================================================

void prepareHardwareForOTA()
{
    Serial.println("Turning OFF device outputs...");
    strip.clear();
    strip.show();
    Serial.println("Device outputs are OFF.");
}

void stopNormalTasksForOTA()
{
    Serial.println();
    Serial.println("====================================");
    Serial.println("      PREPARING DEVICE FOR OTA");
    Serial.println("====================================");

    if (controltaskhandle != NULL) vTaskSuspend(controltaskhandle);
    if (ledTaskHandle     != NULL) vTaskSuspend(ledTaskHandle);
    if (displayTaskHandle != NULL) vTaskSuspend(displayTaskHandle);
    if (bmeTaskHandle     != NULL) vTaskSuspend(bmeTaskHandle);
    if (ipmTaskHandle     != NULL) vTaskSuspend(ipmTaskHandle);

    Serial.println("Normal application tasks suspended.");
}

void otaTask(void *pvParameters)
{
    Serial.println();
    Serial.println("====================================");
    Serial.println("       OTA TASK STARTED");
    Serial.println("====================================");

    while (true)
    {
        if (otaRequested && !otaInProgress)
        {
            otaRequested  = false;
            otaInProgress = true;

            Serial.println();
            Serial.println("====================================");
            Serial.println("       OTA REQUEST ACCEPTED");
            Serial.println("====================================");

            stopNormalTasksForOTA();
            prepareHardwareForOTA();
            vTaskDelay(pdMS_TO_TICKS(500));

            String url = otaFirmwareUrl;
            if (url.length() == 0) {
                Serial.println("OTA FAILED: Empty firmware URL.");
                otaInProgress = false;
                continue;
            }

            performOTA(url);
            otaInProgress = false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// =====================================================
// SETUP
// ================= ===================================
void setup() {
    // Serial.begin(115200);
    Serial.println(getDefaultMacAddress());
    // ipm.begin(1200, SERIAL_8N1, RXD2, TXD2); // Use the new 'ipm' HardwareSerial instance
    // dwin.begin(115200, SERIAL_8N1, 3, 1); // RX first, then TX(23,13) for new code

    Serial.begin(115200);
    ipm.begin(1200, SERIAL_8N1, RXD2, TXD2);
    dwin.begin(115200, SERIAL_8N1, DWIN_RX, DWIN_TX);

    strip.begin();
    strip.show();

    mac = getDefaultMacAddress();
    Serial.print("Device MAC: "); Serial.println(mac);

    AWS_IOT_PUBLISH_TOPIC   = "airbuddi/device/" + mac + "/telemetry";
    AWS_IOT_SUBSCRIBE_TOPIC = "airbuddi/device/" + mac + "/control";
    AWS_IOT_STATUS_TOPIC    = "airbuddi/device/" + mac + "/status";

    Serial.print("Telemetry topic: ");  Serial.println(AWS_IOT_PUBLISH_TOPIC);
    Serial.print("Control topic:   "); Serial.println(AWS_IOT_SUBSCRIBE_TOPIC);
    Serial.print("Firmware version: "); Serial.println(FIRMWARE_VERSION);

    xTaskCreate(displayTask,     "Display Task",     4096,  NULL, 1, &displayTaskHandle);
    xTaskCreate(WifiManagerTask, "WifiManager Task", 4096,  NULL, 1, &WifiManagerTaskHandle);
    xTaskCreate(ipmTask,         "IPM Task",         2048,  NULL, 1, &ipmTaskHandle);
    // xTaskCreate(bmeTask,         "BME Task",         8192,  NULL, 1, &bmeTaskHandle);
    xTaskCreate(bmeTask,         "BME Task",         4096,  NULL, 1, &bmeTaskHandle);
    xTaskCreate(ledTask,         "LED Task",         4096,  NULL, 1, &ledTaskHandle);
    xTaskCreate(AWSTask,         "AWS Task",         16384, NULL, 3, &AWSTaskHandle);
    xTaskCreate(displaycontrol,  "Display Control",  10000, NULL, 1, &controltaskhandle);
    // HIGH PRIORITY OTA task
    xTaskCreate(otaTask, "OTA Task", OTA_TASK_STACK_SIZE, NULL, OTA_TASK_PRIORITY, &otaTaskHandle);
}

void publishMessage() {
    JsonDocument doc;
    doc["NAME"]           = "AQI MONITOR";
    doc["id"]             = mac;
    doc["IAQ"]            = isnan(IAQ) ? 0.0f : IAQ;
    doc["Humidity"]       = isnan(Humidity) ? 0.0f : Humidity;
    doc["PM 1.0"]         = PM1;
    doc["PM 2.5"]         = PM25;
    doc["PM 4.0"]         = PM4;
    doc["PM 10"]          = PM10;
    doc["Temperature"]    = isnan(temperature) ? 0.0f : temperature;
    doc["Pressure"]       = isnan(pressure) ? 0.0f : pressure;
    doc["C02 Equivalent"] = isnan(Co2) ? 0.0f : Co2;
    doc["VOC's"]          = isnan(Vocs) ? 0.0f : Vocs;
    doc["Gas Resistance"] = isnan(Gasresistance) ? 0.0f : Gasresistance;

    char jsonBuffer[512];
    size_t len = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
    if (len >= sizeof(jsonBuffer)) {
        Serial.println("JSON buffer too small; message not published");
        return;
    }
    bool ok = client.publish(AWS_IOT_PUBLISH_TOPIC.c_str(), jsonBuffer);
    if (ok) { Serial.println("Published to AWS IoT OK"); }
    else    { Serial.println("Publish FAILED"); }
}

void loop() {}

// Task functions
void bmeTask(void *pvParameters) {
  pinMode(LED_BUILTIN, OUTPUT);
  iaqSensor.begin(BME68X_I2C_ADDR_LOW, Wire);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();
  bsec_virtual_sensor_t sensorList[13] = {
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_GAS_PERCENTAGE
  };
  iaqSensor.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();
  // Print the header
  output = "Timestamp [ms], IAQ, IAQ accuracy, Static IAQ, CO2 equivalent, breath VOC equivalent, raw temp[°C], pressure [hPa], raw relative humidity [%], gas [Ohm], Stab Status, run in status, comp temp[°C], comp humidity [%], gas percentage";
  //  Serial.println(output);

  while (1) {
    unsigned long time_trigger = millis();
    if (iaqSensor.run()) { // If new data is available
      digitalWrite(LED_BUILTIN, LOW);
      pressure = iaqSensor.pressure;
      temperature = iaqSensor.temperature;
      Humidity = iaqSensor.humidity;
      Gasresistance = iaqSensor.gasResistance;
      IAQ = iaqSensor.iaq;
      Co2 = iaqSensor.co2Equivalent;
      Vocs = iaqSensor.breathVocEquivalent;
      output = String(time_trigger);
      output += ", " + String(iaqSensor.iaq);
      output += ", " + String(iaqSensor.iaqAccuracy);
      output += ", " + String(iaqSensor.staticIaq);
      output += ", " + String(iaqSensor.co2Equivalent);
      output += ", " + String(iaqSensor.breathVocEquivalent);
      output += ", " + String(iaqSensor.rawTemperature);
      output += ", " + String(iaqSensor.pressure);
      output += ", " + String(iaqSensor.rawHumidity);
      output += ", " + String(iaqSensor.gasResistance);
      output += ", " + String(iaqSensor.stabStatus);
      output += ", " + String(iaqSensor.runInStatus);
      output += ", " + String(iaqSensor.temperature);
      output += ", " + String(iaqSensor.humidity);
      output += ", " + String(iaqSensor.gasPercentage);
      Serial.println(output);
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      checkIaqSensorStatus();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void ipmTask(void *pvParameters) {
  startMeasurement(); // Start measurement at the beginning of the task
  while (1) {
    requestPMData();
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Use vTaskDelay instead of delay()
  }
}


void displayTask(void *pvParameters) {
  while (1) {
  #if DWIN_TEST
    uint8_t t[] = {0x5A,0xA5,0x07,0x82,0x24,0x00, 0x00,0x00,0x00,0x7B};  // IAQ = 123
    while (dwin.available()) dwin.read();      // clear old bytes
    dwin.write(t, sizeof(t));
    vTaskDelay(pdMS_TO_TICKS(300));
    Serial.print("DWIN reply: ");
    while (dwin.available()) Serial.printf("%02X ", dwin.read());
    Serial.println();
    vTaskDelay(pdMS_TO_TICKS(200));
    continue;
  #endif
    long g;
    long h;
    long i;
    long j;

    g = Gasresistance;
    i = g >> 8;
    j = g >> 16;
    dwin.write(a, 7);
    dwin.write(j);
    dwin.write(i & 0x0000FF);
    dwin.write(g & 0x0000FF);

    g = pressure;
    i = g >> 8;
    j = g >> 16;
    dwin.write(b, 7);
    dwin.write(j);
    dwin.write(i & 0x0000FF);
    dwin.write(g & 0x0000FF);

    g = temperature;
    i = g >> 8;
    j = g >> 16;
    dwin.write(c, 7);
    dwin.write(j);
    dwin.write(i & 0x0000FF);
    dwin.write(g & 0x0000FF);

    g = Humidity;
    i = g >> 8;
    j = g >> 16;
    dwin.write(d, 7);
    dwin.write(j);
    dwin.write(i & 0x0000FF);
    dwin.write(g & 0x0000FF);

    g = IAQ;
    i = g >> 8;
    j = g >> 16;
    dwin.write(e, 7);
    dwin.write(j);
    dwin.write(i & 0x0000FF);
    dwin.write(g & 0x0000FF);

    g = Co2;
    i = g >> 8;
    j = g >> 16;
    dwin.write(f, 7);
    dwin.write(j);
    dwin.write(i & 0x0000FF);
    dwin.write(g & 0x0000FF);

    float vocs = Vocs;
    int16_t vocs_int = (int16_t)(vocs * 100);  // Assuming 2 decimal places precision
    i = vocs_int >> 8;
    j = vocs_int >> 16;
    dwin.write(k, 7);
    dwin.write(j);
    dwin.write(i & 0x00FF);
    dwin.write(vocs_int & 0x00FF);

    g = PM25;
    i = g >> 8;
    j = g >> 16;
    dwin.write(l, 7);
    dwin.write(j);
    dwin.write(i & 0x0000FF);
    dwin.write(g & 0x0000FF);

    g = PM10;
    i = g >> 8;
    j = g >> 16;
    dwin.write(m, 7);
    dwin.write(j);
    dwin.write(i & 0x0000FF);
    dwin.write(g & 0x0000FF);

    g = PM1;
    i = g >> 8;
    j = g >> 16;
    dwin.write(n, 7);
    dwin.write(j);
    dwin.write(i & 0x0000FF);
    dwin.write(g & 0x0000FF);

    g = PM4;
    i = g >> 8;
    j = g >> 16;
    dwin.write(o, 7);
    dwin.write(j);
    dwin.write(i & 0x0000FF);
    dwin.write(g & 0x0000FF);

    sendMacToDwin();
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

#define DWIN_MAC_VP 0x3200

void sendMacToDwin()
{
    if (mac.length() == 0) {
        mac = getDefaultMacAddress();
    }
    if (mac.length() == 0) return;

    const char *macText = mac.c_str();
    uint8_t macLen = mac.length();

    // DWIN DGUS frame format for writing string:
    // 0x5A 0xA5 [Len] 0x82 [VP_H] [VP_L] [ASCII characters...] 0xFF 0xFF
    // Len = 1 (0x82) + 2 (VP) + macLen + 2 (0xFFFF terminator)
    uint8_t frameLen = 1 + 2 + macLen + 2;

    dwin.write((uint8_t)0x5A);
    dwin.write((uint8_t)0xA5);
    dwin.write(frameLen);
    dwin.write((uint8_t)0x82);

    // VP 0x3200
    dwin.write((uint8_t)((DWIN_MAC_VP >> 8) & 0xFF));
    dwin.write((uint8_t)(DWIN_MAC_VP & 0xFF));

    // ASCII MAC characters
    for (size_t i = 0; i < macLen; i++)
    {
        dwin.write((uint8_t)macText[i]);
    }

    // DWIN DGUS Text terminator
    dwin.write((uint8_t)0xFF);
    dwin.write((uint8_t)0xFF);
}

void displaycontrol(void *parameter) {
  while (1) 
  {
    memset(Buffer, 0, 9);


    if (dwin.available()) {
      for (int i = 0; i <= 8; i++) {
        Buffer[i] = dwin.read();
      }
    }

    switch (Buffer[5]) {

      case 0X11:  //POWER ON
        switch (Buffer[8]) {
          case 0X01:
            p = 7;
            break;
        }
      case 0X12:  //POWER OFF
        switch (Buffer[8]) {
          case 0X05:
            p = 5;
            break;
        }


    }
  }
}  

void ledTask(void *pvParameters) {
  while (1) {
    if (p != 5) {
      // Condition: if p is NOT 5, the LEDs will be controlled by PM2.5 data.
      if (PM25 != 0) {
        uint32_t color;

        if (PM25 > 500) {
          color = strip.Color(255, 0, 0); // Red
        } else if (PM25 > 100 && PM25 <= 500) {
          color = strip.Color(0, 0, 255); // Blue
        } else { // PM25 <= 100
          color = strip.Color(0, 255, 0); // Green
        }
        
        for (int i = 0; i < LED_COUNT; i++) {
          strip.setPixelColor(i, color);
        }
        strip.show();
      }
    } else {
      // Condition: if p IS 5, turn off all the LEDs.
      for (int i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 0)); // Set color to black (off)
      }
      strip.show();
    }
    
    // Delay the task to prevent hogging the CPU
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void WifiManagerTask(void *pvParameters) {
  WiFi.mode(WIFI_STA);
  WiFiManager wm;
  // wm.resetSettings();
  bool res;
  res = wm.autoConnect("AirBuddi4", "password");
  if (!res) {
    Serial.println("Failed to connect");
  } else {
    Serial.println("connected...yeey :)");
  }
  vTaskDelete(NULL);
}

// =====================================================
// AWS HELPER FUNCTIONS  (ported from Airbuddimax)
// =====================================================

const char* mqttStateMessage(int state) {
    switch (state) {
        case MQTT_CONNECTION_TIMEOUT:    return "connection timeout";
        case MQTT_CONNECTION_LOST:       return "connection lost";
        case MQTT_CONNECT_FAILED:        return "TCP/TLS connect failed";
        case MQTT_DISCONNECTED:          return "disconnected";
        case MQTT_CONNECTED:             return "connected";
        case MQTT_CONNECT_BAD_PROTOCOL:  return "bad MQTT protocol";
        case MQTT_CONNECT_BAD_CLIENT_ID: return "bad client ID / thing name";
        case MQTT_CONNECT_UNAVAILABLE:   return "broker unavailable";
        case MQTT_CONNECT_BAD_CREDENTIALS: return "bad credentials";
        case MQTT_CONNECT_UNAUTHORIZED:  return "unauthorized - check AWS IoT policy/certificate";
        default:                         return "unknown";
    }
}

bool syncTimeForTLS() {
    time_t now = time(nullptr);
    if (now > 1700000000) return true;
    Serial.println("Syncing time for TLS...");
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    for (int i = 0; i < 30; i++) {
        now = time(nullptr);
        if (now > 1700000000) {
            Serial.print("Time synced: "); Serial.println(ctime(&now));
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    Serial.println("Time sync failed. TLS certificate validation may fail.");
    return false;
}

void printAwsNetworkDiagnostics() {
    Serial.print("WiFi IP: ");   Serial.println(WiFi.localIP());
    Serial.print("WiFi RSSI: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    IPAddress awsIp;
    if (WiFi.hostByName(AWS_IOT_ENDPOINT, awsIp)) {
        Serial.print("AWS endpoint resolved to: "); Serial.println(awsIp);
    } else {
        Serial.println("DNS failed for AWS IoT endpoint.");
    }
}

bool lastPowerState = false;
bool statusInitialized = false;

bool statusChanged()
{
    bool currentPower = (p == 7);

    if (!statusInitialized) {
        lastPowerState = currentPower;
        statusInitialized = true;
        return true;
    }

    if (currentPower != lastPowerState) {
        lastPowerState = currentPower;
        return true;
    }

    return false;
}

void publishStatus()
{
    JsonDocument doc;
    doc["deviceId"]        = mac;
    doc["power"]           = (p == 7);
    doc["firmwareVersion"] = FIRMWARE_VERSION;
    doc["model"]           = DEVICE_MODEL;

    char jsonBuffer[512];
    size_t len = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));
    if (len >= sizeof(jsonBuffer)) {
        Serial.println("Status JSON too large");
        return;
    }

    if (client.publish(AWS_IOT_STATUS_TOPIC.c_str(), jsonBuffer)) {
        Serial.println("===== STATUS PUBLISHED =====");
        Serial.println(jsonBuffer);
        Serial.println("============================");
    } else {
        Serial.println("STATUS PUBLISH FAILED");
    }
}

void messageHandler(char* topic, byte* payload, unsigned int length)
{
    String mqttMsg;
    for (unsigned int i = 0; i < length; i++) mqttMsg += (char)payload[i];
    mqttMsg.trim();

    Serial.println();
    Serial.println("===== MQTT MESSAGE RECEIVED =====");
    Serial.print("Topic: ");   Serial.println(topic);
    Serial.print("Payload: "); Serial.println(mqttMsg);
    Serial.println("=================================");

    if (String(topic) != AWS_IOT_SUBSCRIBE_TOPIC) {
        Serial.println("Unexpected MQTT topic!"); return;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, mqttMsg);
    if (error) { Serial.print("JSON Error: "); Serial.println(error.c_str()); return; }

    String message = doc["message"] | "";
    message.trim();
    Serial.print("Command: "); Serial.println(message);

    // =========================================================
    // HTTPS OTA UPDATE
    // =========================================================
    if (message == "firmware_update")
    {
        Serial.println("===== FIRMWARE UPDATE REQUEST =====");
        String firmwareUrl = doc["url"] | "";
        firmwareUrl.trim();
        if (firmwareUrl.length() == 0) { Serial.println("OTA REJECTED: Missing firmware URL."); return; }
        if (otaInProgress || otaRequested) { Serial.println("OTA REJECTED: OTA already active."); return; }

        String requestedModel = doc["model"] | "";
        if (requestedModel.length() > 0) {
            requestedModel.trim();
            if (requestedModel != DEVICE_MODEL) {
                Serial.println("OTA REJECTED: Firmware model mismatch.");
                Serial.print("Device model: ");    Serial.println(DEVICE_MODEL);
                Serial.print("Requested model: "); Serial.println(requestedModel);
                return;
            }
        }

        String requestedVersion = doc["version"] | "";
        Serial.println("OTA request accepted.");
        Serial.print("Device model: ");     Serial.println(DEVICE_MODEL);
        Serial.print("Current firmware: "); Serial.println(FIRMWARE_VERSION);
        if (requestedVersion.length() > 0) { Serial.print("Requested firmware: "); Serial.println(requestedVersion); }

        otaFirmwareUrl = firmwareUrl;
        otaRequested   = true;
        Serial.println("OTA task notified."); Serial.println("==================================");
        return;
    }

    // Monitor control commands
    if (message == "power_on") {
        p = 7;
        publishStatus();
        return;
    }

    if (message == "power_off") {
        p = 5;
        publishStatus();
        return;
    }

    if (message == "reboot") {
        Serial.println("Reboot command received. Restarting...");
        delay(500); ESP.restart();
    }

    Serial.print("Unknown command: "); Serial.println(message);
    Serial.println("=================================");
}

bool connectAWS()
{
    if (client.connected()) return true;
    if (WiFi.status() != WL_CONNECTED) { Serial.println("WiFi not connected!"); return false; }

    syncTimeForTLS();
    printAwsNetworkDiagnostics();
    Serial.println("Connecting to AWS IoT...");

    for (int attempt = 1; attempt <= 5 && !client.connected(); attempt++)
    {
        Serial.print("AWS connect attempt "); Serial.println(attempt);
        if (client.connect(mac.c_str()))
        {
            Serial.println("AWS IoT Connected!");
            publishStatus();
            if (client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC.c_str())) {
                Serial.println("MQTT SUBSCRIBE OK");
                Serial.print("Subscribed topic: "); Serial.println(AWS_IOT_SUBSCRIBE_TOPIC);
            } else { Serial.println("MQTT SUBSCRIBE FAILED"); }
            return true;
        }
        int state = client.state();
        Serial.print("AWS connect failed, state = "); Serial.print(state);
        Serial.print(" ("); Serial.print(mqttStateMessage(state)); Serial.println(")");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

void AWSTask(void *pvParameters)
{
    while (WiFi.status() != WL_CONNECTED) { vTaskDelay(pdMS_TO_TICKS(1000)); }

    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    client.setServer(AWS_IOT_ENDPOINT, 8883);
    client.setCallback(messageHandler);
    client.setBufferSize(16384);
    client.setKeepAlive(60);
    client.setSocketTimeout(15);

    while (!connectAWS()) { vTaskDelay(pdMS_TO_TICKS(5000)); }
    publishStatus();

    for (int i = 0; i < 20; i++) { client.loop(); vTaskDelay(pdMS_TO_TICKS(10)); }

    TickType_t lastPublish = xTaskGetTickCount();
    int reconnectDelay = 5000;

    while (true)
    {
        if (otaInProgress || otaRequested) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

        client.loop();

        if (!client.connected())
        {
            Serial.println("MQTT Lost. Reconnecting...");
            vTaskDelay(pdMS_TO_TICKS(reconnectDelay));
            if (connectAWS()) {
                reconnectDelay = 5000;
                for (int i = 0; i < 20; i++) { client.loop(); vTaskDelay(pdMS_TO_TICKS(100)); }
                lastPublish = xTaskGetTickCount();
            } else {
                reconnectDelay = min(reconnectDelay * 2, 60000);
            }
            continue;
        }

        if (statusChanged())
        {
            publishStatus();
        }

        if ((xTaskGetTickCount() - lastPublish) >= pdMS_TO_TICKS(5000))
        {
            lastPublish = xTaskGetTickCount();
            publishMessage();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void startMeasurement() {
  byte cmd[] = { 0xFE, 0xA5, 0x00, 0x11, 0xB6 };  // Start measurement command
  ipm.write(cmd, sizeof(cmd)); // Use 'ipm' serial port
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  while (ipm.available()) ipm.read();  // Clear input buffer
}

void requestPMData() {
  byte cmd[] = { 0xFE, 0xA5, 0x00, 0x07, 0xAC };  // Request PM1, PM2.5, PM4, PM10
  ipm.write(cmd, sizeof(cmd)); // Use 'ipm' serial port

  unsigned long start_time = millis();
  while (ipm.available() < 13 && (millis() - start_time) < 1000) { // Wait for response with a timeout
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  if (ipm.available() >= 13) {
    byte buffer[13];
    ipm.readBytes(buffer, 13);
    
    // Validate response header
    if (buffer[0] == 0xFE && buffer[1] == 0xA5 && buffer[3] == 0x00) {
      PM1 = buffer[4] << 8 | buffer[5];
      PM25 = buffer[6] << 8 | buffer[7];
      PM4 = buffer[8] << 8 | buffer[9];
      PM10 = buffer[10] << 8 | buffer[11];
      
      Serial.println("----- Air Quality Readings -----");
      Serial.print("PM1.0: ");
      Serial.print(PM1);
      Serial.println(" mg/m³");
      Serial.print("PM2.5: ");
      Serial.print(PM25);
      Serial.println(" mg/m³");
      Serial.print("PM4.0: ");
      Serial.print(PM4);
      Serial.println(" mg/m³");
      Serial.print("PM10 : ");
      Serial.print(PM10);
      Serial.println(" mg/m³");
      Serial.println("--------------------------------");
    } else {
      Serial.println("Invalid response received.");
    }
  } else {
    Serial.println("Sensor response timed out or incomplete.");
  }
}

// =====================================================
// HTTPS OTA FUNCTION  (ported from Airbuddimax)
// =====================================================
void performOTA(String firmwareUrl)
{
    Serial.println();
    Serial.println("====================================");
    Serial.println("       AQI MONITOR HTTPS OTA");
    Serial.println("====================================");

    Serial.printf("Free heap before OTA: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("Largest free block:   %u bytes\n", ESP.getMaxAllocHeap());

    client.disconnect();
    delay(500);
    Serial.printf("Free heap after MQTT disconnect: %u bytes\n", ESP.getFreeHeap());

    WiFiClientSecure otaClient;
    otaClient.setInsecure(); // DEVELOPMENT ONLY

    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.setTimeout(30000);
    Serial.println("Connecting to firmware server...");

    if (!http.begin(otaClient, firmwareUrl)) {
        Serial.println("OTA FAILED: HTTP begin failed"); return;
    }

    int httpCode = http.GET();
    Serial.printf("HTTP response: %d\n", httpCode);
    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("OTA FAILED: HTTP error = %d\n", httpCode);
        http.end(); return;
    }

    int contentLength = http.getSize();
    Serial.printf("Firmware size: %d bytes\n", contentLength);
    if (contentLength <= 0) {
        Serial.println("OTA FAILED: Invalid firmware size"); http.end(); return;
    }

    if (!Update.begin(contentLength)) {
        Serial.printf("OTA FAILED: Update.begin error: %s\n", Update.errorString());
        http.end(); return;
    }

    WiFiClient *stream = http.getStreamPtr();
    uint8_t buffer[1024];
    size_t written = 0;
    unsigned long lastDataTime = millis();

    while (http.connected() && written < (size_t)contentLength)
    {
        size_t available = stream->available();
        if (available) {
            size_t readSize = (available > sizeof(buffer)) ? sizeof(buffer) : available;
            int bytesRead = stream->readBytes(buffer, readSize);
            if (bytesRead > 0) {
                size_t bytesWritten = Update.write(buffer, bytesRead);
                if (bytesWritten != (size_t)bytesRead) {
                    Serial.println("OTA FAILED: Flash write error");
                    Update.abort(); http.end(); return;
                }
                written += bytesWritten;
                lastDataTime = millis();
                Serial.printf("OTA progress: %u / %u bytes (%u%%)\n",
                    (unsigned)written, (unsigned)contentLength,
                    (unsigned)((written * 100) / contentLength));
            }
        }
        if (millis() - lastDataTime > 15000) {
            Serial.println("OTA FAILED: Download timeout");
            Update.abort(); http.end(); return;
        }
        delay(1);
    }

    if (written != (size_t)contentLength) {
        Serial.println("OTA FAILED: Incomplete firmware download");
        Update.abort(); http.end(); return;
    }
    if (!Update.end()) {
        Serial.printf("OTA FAILED: Update.end error: %s\n", Update.errorString());
        http.end(); return;
    }
    if (!Update.isFinished()) {
        Serial.println("OTA FAILED: Update not finished"); http.end(); return;
    }

    Serial.println();
    Serial.println("====================================");
    Serial.println("      OTA UPDATE SUCCESSFUL");
    Serial.println("====================================");

    http.end();
    delay(1000);
    ESP.restart();
}