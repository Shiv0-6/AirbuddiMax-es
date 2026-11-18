#include "bsec.h"
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include "FreeRTOS.h"
#include <freertos/semphr.h>
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

#define DEVICE_MODEL "AIRBUDDI_MINI"
#define FIRMWARE_VERSION "1.0.3"

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// =====================================================
// RTOS SYNCHRONIZATION
// =====================================================
SemaphoreHandle_t dwinMutex = NULL;

// =====================================================
// HTTPS OTA
// =====================================================

volatile bool otaInProgress = false;
volatile bool otaRequested = false;

String otaFirmwareUrl = "";

TaskHandle_t otaTaskHandle = NULL;

#define OTA_TASK_PRIORITY 5
#define OTA_TASK_STACK_SIZE 12288

void performOTA(String firmwareUrl);
void otaTask(void *pvParameters);

// ===================== AWS IoT =====================

String AWS_IOT_PUBLISH_TOPIC;
String AWS_IOT_SUBSCRIBE_TOPIC;
String AWS_IOT_STATUS_TOPIC;

String mac = "";
int connectedFlag = 0;
String getDefaultMacAddress();
WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

bool connectAWS();
bool syncTimeForTLS();
void printAwsNetworkDiagnostics();
const char* mqttStateMessage(int state);
void messageHandler(char* topic, byte* payload, unsigned int length);
void publishMessage();
void publishStatus();
void AWSTask(void *pvParameters);
void WifiManagerTask(void *pvParameters);

//BME 688
void checkIaqSensorStatus(void);
void errLeds(void);
Bsec iaqSensor;
void colorWipe(uint32_t color, int wait);
void reset(void);
bool readDwinFrame(uint8_t *buf, size_t maxLen);
void dwinWriteBytes(const uint8_t *data, size_t len);
void dwinSendSensorValue(const unsigned char prefix[7], int32_t val);
int readMoisturePercent(uint8_t pin, float fallbackPercent);

// Sensor readings (volatile for multi-core safety)
volatile int PM25 = 0;
volatile int PM10 = 0;

volatile int soilmoistsetting1 = 50;
volatile int soilmoistsetting2 = 51;
volatile int sl1 = 0;
volatile int sl2 = 0;
volatile int moist = 0;

volatile float pressure = 0.0f;
volatile float temperature = 0.0f;
volatile float Humidity = 0.0f;
volatile float Gasresistance = 0.0f;
volatile float IAQ = 0.0f;
volatile float Co2 = 0.0f;
volatile float Vocs = 0.0f;

volatile float UPPER_CHAMBER = 0.0f;
volatile float LOWER_CHAMBER = 0.0f;

//DEFINIG WHICH SERIAL PORT SHOULD BE USED FOR WHICH DEVICE/SENSOR
HardwareSerial dwin(2);
HardwareSerial HPMA115S0(1);

bool start_autosend(void);
bool receive_measurement(void);

// =====================================================
// PIN DEFINITIONS
// =====================================================
#define RGB_PIN 5
#define LED_COUNT 16

#define RXD2 16
#define TXD2 17
#define UV_PROTECTION 19
#define UPPER_CHAMBER_PIN 15
#define LOWER_CHAMBER_PIN 12
#define SOIL_MOISTURE_SENSOR_1 25
#define SOIL_MOISTURE_SENSOR_2 32
#define Watersprinkler1 26
#define Watersprinkler2 33
#define speed1 4
#define speed2 18
#define speed3 27

// Device Control State (volatile for multi-core safety)
volatile int mod = 0;
volatile int text = 0;
volatile int mine = 5;
volatile uint8_t p = 5;

bool powerState = false;
int fanSpeed = 0;
bool uvState = false;
bool upperChamberState = false;
bool lowerChamberState = false;
bool sleepMode = false;
bool autoMode = false;
bool sleepStatusChange = false;

// NeoPixel
Adafruit_NeoPixel strip(LED_COUNT, RGB_PIN, NEO_GRB + NEO_KHZ800);
int pixelInterval = 50;
uint16_t pixelCurrent = 0;
const uint16_t pixelNumber = LED_COUNT;

#define DWIN_MAC_VP 0x3200

void sendMacToDwin()
{
    const char *macText = mac.c_str();
    uint8_t pkt[23];
    pkt[0] = 0x5A;
    pkt[1] = 0xA5;
    pkt[2] = 0x14;
    pkt[3] = 0x82;
    pkt[4] = 0x32;
    pkt[5] = 0x00;
    for (int i = 0; i < 17; i++)
    {
        pkt[6 + i] = (uint8_t)macText[i];
    }
    dwinWriteBytes(pkt, 23);
}

// =====================================================
// DWIN COMMAND DEFINITIONS
// =====================================================
const unsigned char A[8] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X06, 0x00, 0X01 };  //(UV ON)
const unsigned char B[8] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X06, 0x00, 0X00 };  //(UV OFF)
const unsigned char C[8] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X07, 0x00, 0X05 };  //(SLEEP ON)
const unsigned char D[8] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X03, 0x00, 0X07 };  //(SLEEP OFF)
const unsigned char T[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X00, 0X84, 0X5A, 0X01, 0X00, 0X09 };  //(GO TO PAGE 9)
const unsigned char U[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X00, 0X84, 0X5A, 0X01, 0X00, 0X0C };  //(GO TO PAGE 12)
const unsigned char E[8] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X08, 0x00, 0X01 };  //(UPPER CHAMBER ON)
const unsigned char F[8] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X08, 0x00, 0X00 };  //(UPPER CHAMBER OFF)
const unsigned char G[8] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X09, 0x00, 0X01 };  //(LOWER CHAMBER ON)
const unsigned char H[8] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X09, 0x00, 0X00 };  //(LOWER CHAMBER OFF)
const unsigned char I[8] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X05, 0x00, 0X02 };  //(FAN LOW)
const unsigned char J[8] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X05, 0x00, 0X00 };  //(FAN MEDIUM)
const unsigned char K[8] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X05, 0x00, 0X01 };  //(FAN HIGH)
const unsigned char W[8] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X05, 0x00, 0X03 };  //(FAN OFF)
const unsigned char X[8] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X10, 0x00, 0X01 };  //AUTO MODE
const unsigned char Y[8] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X10, 0x00, 0X00 };  //(AUTO MODE OFF)
const unsigned char V[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X00, 0X84, 0X5A, 0X01, 0X00, 0X16 };  //(GO TO PAGE 22 - AUTO MODE ACTIVE SCREEN)
const unsigned char L[8] = { 0X5A, 0XA5, 0X05, 0X82, 0X10, 0X11, 0X00, 0X01 };  //(TRIGGER POWER ON BUTTON)
const unsigned char M[8] = { 0X5A, 0XA5, 0X05, 0X82, 0X10, 0X12, 0X00, 0X05 };  //(TRIGGER POWER OFF BUTTON)
const unsigned char R[10]  = { 0X5A, 0XA5, 0X07, 0X82, 0X00, 0X84, 0X5A, 0X01, 0X00, 0X00 };  //(GO TO PAGE 0)
const unsigned char S[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X00, 0X84, 0X5A, 0X01, 0X00, 0X0C };  //(GO TO PAGE 12)

// Sensor VP prefixes
const unsigned char a[7] = { 0X5A, 0XA5, 0X07, 0X82, 0X20, 0X00, 0X00 };
const unsigned char b[7] = { 0X5A, 0XA5, 0X07, 0X82, 0X21, 0X00, 0X00 };
const unsigned char c[7] = { 0X5A, 0XA5, 0X07, 0X82, 0X30, 0X00, 0X00 };
const unsigned char d[7] = { 0X5A, 0XA5, 0X07, 0X82, 0X22, 0X00, 0X00 };
const unsigned char e[7] = { 0X5A, 0XA5, 0X07, 0X82, 0X24, 0X00, 0X00 };
const unsigned char f[7] = { 0X5A, 0XA5, 0X07, 0X82, 0X25, 0X00, 0X00 };
const unsigned char k[7] = { 0X5A, 0XA5, 0X07, 0X82, 0X26, 0X00, 0X00 };
const unsigned char l[7] = { 0X5A, 0XA5, 0X07, 0X82, 0X27, 0X00, 0X00 };
const unsigned char m[7] = { 0X5A, 0XA5, 0X07, 0X82, 0X28, 0X00, 0X00 };
const unsigned char n[7] = { 0X5A, 0XA5, 0X07, 0X82, 0X23, 0X00, 0X00 };
const unsigned char o[7] = { 0X5A, 0XA5, 0X07, 0X82, 0X29, 0X00, 0X00 };
const unsigned char q[7] = { 0X5A, 0XA5, 0X07, 0X82, 0X31, 0X00, 0X00 };

// =====================================================
// FUNCTION PROTOTYPES
// =====================================================
void bmeTask(void *pvParameters);
void hpmaTask(void *pvParameters);
void displayTask(void *pvParameters);
void displaycontrol(void *parameter);
void ledlight(void *parameter);
void SoilMoisture_1(void *pvParameters);
void SoilMoisture_2(void *pvParameters);

// =====================================================
// TASK HANDLES
// =====================================================
TaskHandle_t bmeTaskHandle = NULL;
TaskHandle_t hpmaTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
TaskHandle_t controltaskhandle = NULL;
TaskHandle_t ledlighttaskhandle = NULL;
TaskHandle_t SoilMoisture_1_Handle = NULL;
TaskHandle_t SoilMoisture_2_Handle = NULL;
TaskHandle_t AWSTaskHandle = NULL;
TaskHandle_t WifiManagerTaskHandle = NULL;
TaskHandle_t builtinLedTaskHandle = NULL;

volatile bool builtinLedRunning = true;
volatile bool led12Running = false;

// =====================================================
// THREAD-SAFE DWIN UART HELPERS
// =====================================================
void dwinWriteBytes(const uint8_t *data, size_t len) {
  if (dwinMutex != NULL && xSemaphoreTake(dwinMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    dwin.write(data, len);
    xSemaphoreGive(dwinMutex);
  }
}

void dwinSendSensorValue(const unsigned char prefix[7], int32_t val) {
  uint8_t pkt[10];
  memcpy(pkt, prefix, 7);
  pkt[7] = (uint8_t)((val >> 16) & 0xFF);
  pkt[8] = (uint8_t)((val >> 8) & 0xFF);
  pkt[9] = (uint8_t)(val & 0xFF);
  dwinWriteBytes(pkt, 10);
}

int readMoisturePercent(uint8_t pin, float fallbackPercent) {
  int raw = analogRead(pin);
  if (raw < 0 || raw > 4095) {
    return (int)fallbackPercent;
  }
  int pct = map(raw, 4095, 0, 0, 100);
  return constrain(pct, 0, 100);
}

// =====================================================
// OTA TASK CONTROL
// =====================================================

void stopNormalTasksForOTA()
{
    Serial.println();
    Serial.println("====================================");
    Serial.println("      PREPARING DEVICE FOR OTA");
    Serial.println("====================================");

    if (controltaskhandle != NULL)
    {
        vTaskSuspend(controltaskhandle);
    }

    if (ledlighttaskhandle != NULL)
    {
        vTaskSuspend(ledlighttaskhandle);
    }

    if (displayTaskHandle != NULL)
    {
        vTaskSuspend(displayTaskHandle);
    }

    if (SoilMoisture_1_Handle != NULL)
    {
        vTaskSuspend(SoilMoisture_1_Handle);
    }

    if (SoilMoisture_2_Handle != NULL)
    {
        vTaskSuspend(SoilMoisture_2_Handle);
    }

    if (bmeTaskHandle != NULL)
    {
        vTaskSuspend(bmeTaskHandle);
    }

    if (hpmaTaskHandle != NULL)
    {
        vTaskSuspend(hpmaTaskHandle);
    }

    Serial.println("Normal application tasks suspended.");
}

void resumeNormalTasksAfterOTA()
{
    Serial.println();
    Serial.println("====================================");
    Serial.println("    OTA FAILED - RESUMING TASKS");
    Serial.println("====================================");

    if (controltaskhandle != NULL)
    {
        vTaskResume(controltaskhandle);
    }

    if (ledlighttaskhandle != NULL)
    {
        vTaskResume(ledlighttaskhandle);
    }

    if (displayTaskHandle != NULL)
    {
        vTaskResume(displayTaskHandle);
    }

    if (SoilMoisture_1_Handle != NULL)
    {
        vTaskResume(SoilMoisture_1_Handle);
    }

    if (SoilMoisture_2_Handle != NULL)
    {
        vTaskResume(SoilMoisture_2_Handle);
    }

    if (bmeTaskHandle != NULL)
    {
        vTaskResume(bmeTaskHandle);
    }

    if (hpmaTaskHandle != NULL)
    {
        vTaskResume(hpmaTaskHandle);
    }

    Serial.println("Normal application tasks resumed.");
}

void prepareHardwareForOTA()
{
    Serial.println("Turning OFF device outputs...");

    digitalWrite(speed1, LOW);
    digitalWrite(speed2, LOW);
    digitalWrite(speed3, LOW);
    digitalWrite(UV_PROTECTION, LOW);
    digitalWrite(UPPER_CHAMBER_PIN, LOW);
    digitalWrite(LOWER_CHAMBER_PIN, LOW);
    digitalWrite(Watersprinkler1, LOW);
    digitalWrite(Watersprinkler2, LOW);

    Serial.println("Device outputs are OFF.");
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
            otaRequested = false;
            otaInProgress = true;

            Serial.println();
            Serial.println("====================================");
            Serial.println("       OTA REQUEST ACCEPTED");
            Serial.println("====================================");

            stopNormalTasksForOTA();
            prepareHardwareForOTA();

            vTaskDelay(pdMS_TO_TICKS(500));

            String url = otaFirmwareUrl;

            if (url.length() == 0)
            {
                Serial.println("OTA FAILED: Empty firmware URL.");
                resumeNormalTasksAfterOTA();
                otaInProgress = false;
                continue;
            }

            performOTA(url);

            // If we get here, performOTA() failed (success would have restarted).
            resumeNormalTasksAfterOTA();
            otaInProgress = false;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  dwin.begin(115200, SERIAL_8N1, 23, 13);

  dwinMutex = xSemaphoreCreateMutex();

  strip.begin();
  strip.clear();
  strip.show();

  mac = getDefaultMacAddress();

  Serial.print("Device MAC: ");
  Serial.println(mac);

  AWS_IOT_PUBLISH_TOPIC = "airbuddi/device/" + mac + "/telemetry";
  AWS_IOT_SUBSCRIBE_TOPIC = "airbuddi/device/" + mac + "/control";
  AWS_IOT_STATUS_TOPIC = "airbuddi/device/" + mac + "/status";

  Serial.print("Device ID: ");
  Serial.println(mac);

  Serial.print("Telemetry topic: ");
  Serial.println(AWS_IOT_PUBLISH_TOPIC);

  Serial.print("Control topic: ");
  Serial.println(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.print("Firmware Version: ");
  Serial.println(FIRMWARE_VERSION);

  pinMode(speed1, OUTPUT);
  pinMode(speed2, OUTPUT);
  pinMode(speed3, OUTPUT);
  pinMode(UV_PROTECTION, OUTPUT);
  pinMode(UPPER_CHAMBER_PIN, OUTPUT);
  pinMode(LOWER_CHAMBER_PIN, OUTPUT);
  pinMode(SOIL_MOISTURE_SENSOR_1, INPUT);
  pinMode(SOIL_MOISTURE_SENSOR_2, INPUT);
  pinMode(Watersprinkler1, OUTPUT);
  pinMode(Watersprinkler2, OUTPUT);
  digitalWrite(speed1, LOW);
  digitalWrite(speed2, LOW);
  digitalWrite(speed3, LOW);
  digitalWrite(UV_PROTECTION, LOW);
  digitalWrite(UPPER_CHAMBER_PIN, LOW);
  digitalWrite(LOWER_CHAMBER_PIN, LOW);
  digitalWrite(Watersprinkler1, LOW);
  digitalWrite(Watersprinkler2, LOW);

  xTaskCreate(AWSTask, "AWS Task", 16384, NULL, 3, &AWSTaskHandle);
  xTaskCreate(displaycontrol, "Display Control", 8192, NULL, 3, &controltaskhandle);
  xTaskCreate(ledlight, "RGBLED", 2048, NULL, 1, &ledlighttaskhandle);
  xTaskCreate(displayTask, "Display Task", 4096, NULL, 2, &displayTaskHandle);
  xTaskCreate(SoilMoisture_1, "SoilMoisture1", 2048, NULL, 1, &SoilMoisture_1_Handle);
  xTaskCreate(SoilMoisture_2, "SoilMoisture2", 2048, NULL, 1, &SoilMoisture_2_Handle);
  xTaskCreate(WifiManagerTask, "WifiManager Task", 6144, NULL, 2, &WifiManagerTaskHandle);
  xTaskCreate(bmeTask, "BME Task", 8192, NULL, 2, &bmeTaskHandle);
  xTaskCreate(hpmaTask, "HPMA Task", 4096, NULL, 2, &hpmaTaskHandle);
  xTaskCreate(otaTask, "OTA Task", OTA_TASK_STACK_SIZE, NULL, OTA_TASK_PRIORITY, &otaTaskHandle);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// =====================================================
// RGB LED TASK
// =====================================================
void ledlight(void *parameter) {
  bool ledsCleared = false;
  while (1) {
    int currentPM25 = PM25;
    if (p != 5) {
      ledsCleared = false;
      uint32_t targetColor;
      if (currentPM25 > 500) {
        targetColor = strip.Color(255, 0, 0);
      } else if (currentPM25 > 100) {
        targetColor = strip.Color(0, 0, 255);
      } else {
        targetColor = strip.Color(0, 255, 0);
      }
      colorWipe(targetColor, 50);
      vTaskDelay(pdMS_TO_TICKS(50));
    } else {
      if (!ledsCleared) {
        strip.clear();
        strip.show();
        pixelCurrent = 0;
        ledsCleared = true;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

// =====================================================
// DISPLAY SENSOR UPDATE TASK
// =====================================================
void displayTask(void *pvParameters) {
  while (1) {
    UPPER_CHAMBER = readMoisturePercent(SOIL_MOISTURE_SENSOR_1, UPPER_CHAMBER);
    LOWER_CHAMBER = readMoisturePercent(SOIL_MOISTURE_SENSOR_2, LOWER_CHAMBER);

    if (UPPER_CHAMBER <= 35 && LOWER_CHAMBER <= 35) {
      moist = 1;
    } else {
      moist = 0;
    }

    dwinSendSensorValue(a, (int32_t)Gasresistance);
    dwinSendSensorValue(b, (int32_t)pressure);
    dwinSendSensorValue(c, (int32_t)temperature);
    dwinSendSensorValue(d, (int32_t)Humidity);
    dwinSendSensorValue(e, (int32_t)IAQ);
    dwinSendSensorValue(f, (int32_t)Co2);
    dwinSendSensorValue(k, (int32_t)(Vocs * 100.0f));
    dwinSendSensorValue(l, (int32_t)PM25);
    dwinSendSensorValue(m, (int32_t)PM10);
    dwinSendSensorValue(n, (int32_t)UPPER_CHAMBER);
    dwinSendSensorValue(o, (int32_t)LOWER_CHAMBER);

    sendMacToDwin();

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// =====================================================
// SOIL MOISTURE TASKS (Non-Blocking 3-Hour Cooldown)
// =====================================================
void SoilMoisture_1(void *pvParameters) {
  TickType_t lastWateredTick = 0;
  bool hasWateredOnce = false;
  const TickType_t cooldownTicks = pdMS_TO_TICKS(10800000UL);

  while (1) {
    TickType_t now = xTaskGetTickCount();
    bool cooldownExpired = (!hasWateredOnce) || ((now - lastWateredTick) >= cooldownTicks);

    if (UPPER_CHAMBER <= soilmoistsetting1 && sl1 == 0 && cooldownExpired) {
      digitalWrite(Watersprinkler1, HIGH);
      vTaskDelay(pdMS_TO_TICKS(5000));
      digitalWrite(Watersprinkler1, LOW);
      lastWateredTick = xTaskGetTickCount();
      hasWateredOnce = true;
    } else {
      digitalWrite(Watersprinkler1, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void SoilMoisture_2(void *pvParameters) {
  TickType_t lastWateredTick = 0;
  bool hasWateredOnce = false;
  const TickType_t cooldownTicks = pdMS_TO_TICKS(10800000UL);

  while (1) {
    TickType_t now = xTaskGetTickCount();
    bool cooldownExpired = (!hasWateredOnce) || ((now - lastWateredTick) >= cooldownTicks);

    if (LOWER_CHAMBER <= soilmoistsetting2 && sl2 == 0 && moist == 0 && cooldownExpired) {
      digitalWrite(Watersprinkler2, HIGH);
      vTaskDelay(pdMS_TO_TICKS(5000));
      digitalWrite(Watersprinkler2, LOW);
      lastWateredTick = xTaskGetTickCount();
      hasWateredOnce = true;
    } else {
      digitalWrite(Watersprinkler2, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// =====================================================
// DWIN FRAME READER (Robust with timeout)
// =====================================================
bool readDwinFrame(uint8_t *buf, size_t maxLen) {
  while (dwin.available() > 0) {
    if (dwin.peek() != 0x5A) {
      dwin.read();
      continue;
    }
    uint32_t t0 = millis();
    while (dwin.available() < 3) {
      if (millis() - t0 > 25) return false;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    buf[0] = dwin.read();
    buf[1] = dwin.read();
    if (buf[1] != 0xA5) {
      continue;
    }
    buf[2] = dwin.read();
    uint8_t payloadLen = buf[2];
    if (payloadLen + 3 > maxLen) {
      for (uint8_t i = 0; i < payloadLen && dwin.available(); i++) dwin.read();
      continue;
    }
    t0 = millis();
    while (dwin.available() < payloadLen) {
      if (millis() - t0 > 35) return false;
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    for (uint8_t i = 0; i < payloadLen; i++) {
      buf[3 + i] = dwin.read();
    }
    return true;
  }
  return false;
}

// =====================================================
// DISPLAY AND CONTROL TASK
// =====================================================
void displaycontrol(void *parameter)
{
    uint8_t Buffer[16];

    while (1)
    {
        int mynumb = PM25;
        memset(Buffer, 0, sizeof(Buffer));

        if (readDwinFrame(Buffer, sizeof(Buffer)))
        {
            Serial.printf(
                "DWIN RX: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                Buffer[0], Buffer[1], Buffer[2],
                Buffer[3], Buffer[4], Buffer[5],
                Buffer[6], Buffer[7], Buffer[8]
            );

            switch (Buffer[5])
            {
                // FAN SPEED (VP = 0x1005)
                case 0X05:
                    switch (Buffer[8])
                    {
                        case 0X03:
                            p = 7;
                            autoMode = false;
                            digitalWrite(speed1, LOW);
                            digitalWrite(speed2, LOW);
                            digitalWrite(speed3, LOW);
                            mine = 2;
                            break;
                        case 0X00:
                            p = 7;
                            autoMode = false;
                            digitalWrite(speed1, HIGH);
                            digitalWrite(speed2, HIGH);
                            digitalWrite(speed3, LOW);
                            mine = 2;
                            break;
                        case 0X01:
                            p = 7;
                            autoMode = false;
                            digitalWrite(speed1, LOW);
                            digitalWrite(speed2, LOW);
                            digitalWrite(speed3, HIGH);
                            mine = 2;
                            break;
                        case 0X02:
                            p = 7;
                            autoMode = false;
                            digitalWrite(speed1, LOW);
                            digitalWrite(speed2, HIGH);
                            digitalWrite(speed3, LOW);
                            mine = 2;
                            break;
                    }
                    break;

                // UV (VP = 0x1006)
                case 0X06:
                    switch (Buffer[8])
                    {
                        case 0X00:
                            p = 7;
                            autoMode = false;
                            digitalWrite(UV_PROTECTION, LOW);
                            mine = 2;
                            break;
                        case 0X01:
                            p = 7;
                            autoMode = false;
                            digitalWrite(UV_PROTECTION, HIGH);
                            mine = 2;
                            break;
                    }
                    break;

                // SLEEP MODE ON (VP = 0x1007)
                case 0X07:
                    if (Buffer[8] == 0X05)
                    {
                        p = 5;
                        autoMode = false;
                        sleepMode = true;
                        digitalWrite(speed1, LOW);
                        digitalWrite(speed2, HIGH);
                        digitalWrite(speed3, LOW);
                        digitalWrite(UPPER_CHAMBER_PIN, HIGH);
                        digitalWrite(LOWER_CHAMBER_PIN, LOW);
                        digitalWrite(UV_PROTECTION, LOW);
                        sl1 = 2;
                        sl2 = 2;
                        mine = 2;
                    }
                    break;

                // SLEEP MODE OFF (VP = 0x1003)
                case 0X03:
                    if (Buffer[8] == 0X07)
                    {
                        p = 7;
                        mine = 2;
                        autoMode = false;
                        sleepMode = false;
                        reset();
                    }
                    break;

                // UPPER CHAMBER (VP = 0x1008)
                case 0X08:
                    if (Buffer[8] == 0X00) {
                        p = 7; autoMode = false;
                        digitalWrite(UPPER_CHAMBER_PIN, LOW);
                        mine = 2;
                    } else if (Buffer[8] == 0X01) {
                        p = 7; autoMode = false;
                        digitalWrite(UPPER_CHAMBER_PIN, HIGH);
                        mine = 2;
                    }
                    break;

                // LOWER CHAMBER (VP = 0x1009)
                case 0X09:
                    if (Buffer[8] == 0X00) {
                        p = 7; autoMode = false;
                        digitalWrite(LOWER_CHAMBER_PIN, LOW);
                        mine = 2;
                    } else if (Buffer[8] == 0X01) {
                        p = 7; autoMode = false;
                        digitalWrite(LOWER_CHAMBER_PIN, HIGH);
                        mine = 2;
                    }
                    break;

                // AUTO / MANUAL (VP = 0x1010)
                case 0X10:
                    switch (Buffer[8])
                    {
                        case 0X00: // MANUAL MODE
                            if (mod == 1) {
                                dwinWriteBytes(K, sizeof(K));
                                dwinWriteBytes(A, sizeof(A));
                                mod = 0;
                            } else if (mod == 2) {
                                dwinWriteBytes(J, sizeof(J));
                                dwinWriteBytes(A, sizeof(A));
                                mod = 0;
                            } else if (mod == 3) {
                                dwinWriteBytes(I, sizeof(I));
                                dwinWriteBytes(B, sizeof(B));
                                mod = 0;
                            } else if (mod == 4) {
                                dwinWriteBytes(W, sizeof(W));
                                dwinWriteBytes(B, sizeof(B));
                                mod = 0;
                            }

                            if (text == 1) {
                                dwinWriteBytes(E, sizeof(E));
                                dwinWriteBytes(G, sizeof(G));
                                text = 0;
                            } else if (text == 2) {
                                dwinWriteBytes(E, sizeof(E));
                                dwinWriteBytes(H, sizeof(H));
                                text = 0;
                            } else if (text == 3) {
                                dwinWriteBytes(F, sizeof(F));
                                dwinWriteBytes(H, sizeof(H));
                                text = 0;
                            }

                            mine = 2;
                            autoMode = false;
                            break;

                        case 0X02: // AUTO MODE
                            p = 7;
                            mine = 1;
                            autoMode = true;
                            sleepMode = false;
                            break;
                    }
                    break;

                // POWER ON (VP = 0x1011)
                case 0X11:
                    if (Buffer[8] == 0X01)
                    {
                        p = 7;
                        mine = 2;
                        autoMode = false;
                        sleepMode = false;
                        dwinWriteBytes(S, sizeof(S));
                        dwinWriteBytes(L, sizeof(L));
                    }
                    break;

                // POWER OFF (VP = 0x1012)
                case 0X12:
                    if (Buffer[8] == 0X05)
                    {
                        mine = 2;
                        p = 5;
                        autoMode = false;
                        sleepMode = false;
                        powerState = false;
                        fanSpeed = 0;
                        uvState = false;
                        upperChamberState = false;
                        lowerChamberState = false;
                        reset();
                    }
                    break;
            }

            // SOIL MOISTURE SETTINGS
            switch (Buffer[4])
            {
                case 0X41:
                    soilmoistsetting1 = Buffer[8];
                    Serial.print("UPPER MOISTURE LEVEL : ");
                    Serial.println(soilmoistsetting1);
                    break;
                case 0X42:
                    soilmoistsetting2 = Buffer[8];
                    Serial.print("LOWER MOISTURE LEVEL : ");
                    Serial.println(soilmoistsetting2);
                    break;
            }
        }

        // AUTO MODE
        if (mine == 1 && p == 7)
        {
            if (mynumb >= 500)
            {
                mod = 1;
                digitalWrite(speed1, LOW);
                digitalWrite(speed2, LOW);
                digitalWrite(speed3, HIGH);
                digitalWrite(UV_PROTECTION, HIGH);
            }
            else if (100 < mynumb && mynumb < 500)
            {
                mod = 2;
                digitalWrite(speed1, HIGH);
                digitalWrite(speed2, HIGH);
                digitalWrite(speed3, LOW);
                digitalWrite(UV_PROTECTION, HIGH);
            }
            else if (40 < mynumb && mynumb <= 100)
            {
                mod = 3;
                digitalWrite(speed1, LOW);
                digitalWrite(speed2, HIGH);
                digitalWrite(speed3, LOW);
                digitalWrite(UV_PROTECTION, LOW);
            }
            else if (mynumb <= 40)
            {
                mod = 4;
                digitalWrite(speed1, LOW);
                digitalWrite(speed2, LOW);
                digitalWrite(speed3, LOW);
                digitalWrite(UV_PROTECTION, LOW);
            }

            // VOC CONTROL
            float currentVocs = Vocs;
            if (currentVocs > 0.75f)
            {
                text = 1;
                digitalWrite(UPPER_CHAMBER_PIN, HIGH);
                digitalWrite(LOWER_CHAMBER_PIN, HIGH);
            }
            else if (0.25f < currentVocs && currentVocs <= 0.75f)
            {
                text = 2;
                digitalWrite(UPPER_CHAMBER_PIN, HIGH);
                digitalWrite(LOWER_CHAMBER_PIN, LOW);
            }
            else if (currentVocs <= 0.25f)
            {
                text = 3;
                digitalWrite(UPPER_CHAMBER_PIN, LOW);
                digitalWrite(LOWER_CHAMBER_PIN, LOW);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// =====================================================
// BME688 TASK (Zero Heap Fragmentation)
// =====================================================
void bmeTask(void *pvParameters) {
  pinMode(LED_BUILTIN, OUTPUT);
  iaqSensor.begin(BME68X_I2C_ADDR_LOW, Wire);
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

  while (1) {
    if (iaqSensor.run()) {
      digitalWrite(LED_BUILTIN, LOW);
      pressure      = iaqSensor.pressure;
      temperature   = iaqSensor.temperature;
      Humidity      = iaqSensor.humidity;
      Gasresistance = iaqSensor.gasResistance;
      IAQ           = iaqSensor.iaq;
      Co2           = iaqSensor.co2Equivalent;
      Vocs          = iaqSensor.breathVocEquivalent;
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      checkIaqSensorStatus();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// =====================================================
// HPMA115S0 TASK (Crash-Proof)
// =====================================================
void hpmaTask(void *pvParameters) {
  HPMA115S0.begin(9600, SERIAL_8N1, RXD2, TXD2);
  vTaskDelay(pdMS_TO_TICKS(500));
  start_autosend();

  while (1) {
    if (receive_measurement()) {
      if (PM10 != 0) {
        Serial.printf("PM 2.5:\t%d ug/m3\nPM 10:\t%d ug/m3\n", PM25, PM10);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// =====================================================
// HELPER FUNCTIONS
// =====================================================
void checkIaqSensorStatus(void) {
  if (iaqSensor.bsecStatus != BSEC_OK) {
    Serial.printf("BSEC status code: %d\n", iaqSensor.bsecStatus);
    if (iaqSensor.bsecStatus < BSEC_OK) {
      errLeds();
    }
  }
  if (iaqSensor.bme68xStatus != BME68X_OK) {
    Serial.printf("BME68X status code: %d\n", iaqSensor.bme68xStatus);
    if (iaqSensor.bme68xStatus < BME68X_OK) {
      errLeds();
      iaqSensor.begin(BME68X_I2C_ADDR_LOW, Wire);
    }
  }
}

void errLeds(void) {
  digitalWrite(LED_BUILTIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(LED_BUILTIN, LOW);
  vTaskDelay(pdMS_TO_TICKS(100));
}

bool start_autosend(void) {
  const uint8_t cmd[] = { 0x68, 0x01, 0x40, 0x57 };
  HPMA115S0.write(cmd, sizeof(cmd));
  HPMA115S0.flush();

  uint32_t startWait = millis();
  while (HPMA115S0.available() < 2) {
    if (millis() - startWait > 1000) {
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  uint8_t read1 = HPMA115S0.read();
  uint8_t read2 = HPMA115S0.read();
  return (read1 == 0xA5 && read2 == 0xA5);
}

bool receive_measurement(void) {
  // Drain backed-up data
  while (HPMA115S0.available() >= 64) {
    HPMA115S0.read();
  }

  uint32_t startWait = millis();
  while (HPMA115S0.available() < 32) {
    if (millis() - startWait > 250) {
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Synchronize to 0x42 0x4D header
  startWait = millis();
  while (HPMA115S0.available() >= 2) {
    if (millis() - startWait > 200) return false;

    uint8_t head0 = HPMA115S0.read();
    if (head0 != 0x42) {
      continue;
    }
    if (HPMA115S0.peek() == 0x4D) {
      uint8_t head1 = HPMA115S0.read();

      uint32_t payloadWait = millis();
      while (HPMA115S0.available() < 30) {
        if (millis() - payloadWait > 150) return false;
        vTaskDelay(pdMS_TO_TICKS(5));
      }

      uint8_t buf[30];
      HPMA115S0.readBytes(buf, 30);

      uint32_t sum = head0 + head1;
      for (int i = 0; i < 28; i++) {
        sum += buf[i];
      }

      uint8_t checkSumL = buf[29];
      if ((sum % 256) != checkSumL) {
        Serial.println("HPMA Checksum fail");
        while (HPMA115S0.available()) HPMA115S0.read();
        return false;
      }

      PM25 = (buf[4] << 8) | buf[5];
      PM10 = (buf[6] << 8) | buf[7];
      return true;
    }
  }
  return false;
}

void colorWipe(uint32_t color, int wait) {
  if (pixelInterval != wait) {
    pixelInterval = wait;
  }
  strip.setPixelColor(pixelCurrent, color);
  strip.show();
  pixelCurrent++;
  if (pixelCurrent >= pixelNumber) {
    pixelCurrent = 0;
  }
}

void reset(void) {
  dwinWriteBytes(H, sizeof(H));
  dwinWriteBytes(B, sizeof(B));
  dwinWriteBytes(W, sizeof(W));
  dwinWriteBytes(F, sizeof(F));

  vTaskDelay(pdMS_TO_TICKS(5));
  digitalWrite(speed1, LOW);
  digitalWrite(speed2, LOW);
  digitalWrite(speed3, LOW);
  digitalWrite(UV_PROTECTION, LOW);
  digitalWrite(UPPER_CHAMBER_PIN, LOW);
  digitalWrite(LOWER_CHAMBER_PIN, LOW);
  sl1 = 0;
  sl2 = 0;
  fanSpeed = 0;
  uvState = false;
  upperChamberState = false;
  lowerChamberState = false;
  sleepMode = false;
}

// =====================================================
// AWS IoT MQTT HANDLERS
// =====================================================
void messageHandler(char* topic, byte* payload, unsigned int length)
{
    String mqttMsg;

    for (unsigned int i = 0; i < length; i++)
    {
        mqttMsg += (char)payload[i];
    }

    mqttMsg.trim();

    Serial.println();
    Serial.println("===== MQTT MESSAGE RECEIVED =====");
    Serial.print("Topic: ");
    Serial.println(topic);
    Serial.print("Payload: ");
    Serial.println(mqttMsg);

    if (String(topic) != AWS_IOT_SUBSCRIBE_TOPIC)
    {
        Serial.println("Unexpected MQTT topic!");
        Serial.println("=================================");
        return;
    }

    JsonDocument doc;

    DeserializationError error = deserializeJson(doc, mqttMsg);

    if (error)
    {
        Serial.print("JSON Error: ");
        Serial.println(error.c_str());
        Serial.println("=================================");
        return;
    }

    String message = doc["message"] | "";
    message.trim();

    Serial.print("Command: ");
    Serial.println(message);

    // =========================================================
    // HTTPS OTA UPDATE
    // =========================================================
    if (message == "firmware_update")
    {
        Serial.println("===== FIRMWARE UPDATE REQUEST =====");

        String firmwareUrl = doc["url"] | "";
        firmwareUrl.trim();

        if (firmwareUrl.length() == 0)
        {
            Serial.println("OTA REJECTED: Missing firmware URL.");
            Serial.println("==================================");
            return;
        }

        if (otaInProgress || otaRequested)
        {
            Serial.println("OTA REJECTED: OTA already active.");
            Serial.println("==================================");
            return;
        }

        String requestedModel = doc["model"] | "";

        if (requestedModel.length() > 0)
        {
            requestedModel.trim();

            if (requestedModel != DEVICE_MODEL)
            {
                Serial.println("OTA REJECTED: Firmware model mismatch.");
                Serial.print("Device model: ");
                Serial.println(DEVICE_MODEL);
                Serial.print("Requested model: ");
                Serial.println(requestedModel);
                Serial.println("==================================");
                return;
            }
        }

        String requestedVersion = doc["version"] | "";

        Serial.println("OTA request accepted.");
        Serial.print("Device model: ");
        Serial.println(DEVICE_MODEL);
        Serial.print("Current firmware: ");
        Serial.println(FIRMWARE_VERSION);

        if (requestedVersion.length() > 0)
        {
            Serial.print("Requested firmware: ");
            Serial.println(requestedVersion);
        }

        Serial.print("Firmware URL length: ");
        Serial.println(firmwareUrl.length());

        otaFirmwareUrl = firmwareUrl;
        otaRequested = true;

        Serial.println("OTA task notified.");
        Serial.println("==================================");

        return;
    }

    // =========================================================
    // NORMAL DEVICE COMMANDS
    // =========================================================
    int command = 0;

    if (message == "power_on")        command = 1;
    else if (message == "power_off")  command = 2;
    else if (message == "lower_on")   command = 3;
    else if (message == "lower_off")  command = 4;
    else if (message == "upper_on")   command = 5;
    else if (message == "upper_off")  command = 6;
    else if (message == "uvc_on")     command = 7;
    else if (message == "uvc_off")    command = 8;
    else if (message == "fan_1")      command = 9;
    else if (message == "fan_2")      command = 10;
    else if (message == "fan_3")      command = 11;
    else if (message == "fan_off")    command = 12;
    else if (message == "sleep_on")   command = 13;
    else if (message == "sleep_off")  command = 14;
    else if (message == "auto_on")    command = 15;
    else if (message == "auto_off")   command = 16;

    switch (command)
    {
        case 1:
            mine = 2;
            autoMode = false;
            p = 7;
            dwinWriteBytes(S, sizeof(S));
            dwinWriteBytes(L, sizeof(L));
            Serial.println("Power ON command received");
            break;

        case 2:
            p = 5;
            mine = 2;
            mod = 0;
            text = 0;
            powerState = false;
            lowerChamberState = false;
            upperChamberState = false;
            autoMode = false;
            sleepMode = false;
            uvState = false;
            fanSpeed = 0;
            reset();
            dwinWriteBytes(R, sizeof(R));
            dwinWriteBytes(M, sizeof(M));
            Serial.println("Power OFF command received");
            break;

        case 3:
            p = 7;
            mine = 2;
            autoMode = false;
            dwinWriteBytes(G, sizeof(G));
            digitalWrite(LOWER_CHAMBER_PIN, HIGH);
            lowerChamberState = true;
            Serial.println("Lower chamber ON command received");
            break;

        case 4:
            p = 7;
            mine = 2;
            autoMode = false;
            dwinWriteBytes(H, sizeof(H));
            digitalWrite(LOWER_CHAMBER_PIN, LOW);
            lowerChamberState = false;
            Serial.println("Lower chamber OFF command received");
            break;

        case 5:
            p = 7;
            mine = 2;
            autoMode = false;
            dwinWriteBytes(E, sizeof(E));
            digitalWrite(UPPER_CHAMBER_PIN, HIGH);
            upperChamberState = true;
            Serial.println("Upper chamber ON command received");
            break;

        case 6:
            p = 7;
            mine = 2;
            autoMode = false;
            dwinWriteBytes(F, sizeof(F));
            digitalWrite(UPPER_CHAMBER_PIN, LOW);
            upperChamberState = false;
            Serial.println("Upper chamber OFF command received");
            break;

        case 7:
            p = 7;
            mine = 2;
            autoMode = false;
            dwinWriteBytes(A, sizeof(A));
            digitalWrite(UV_PROTECTION, HIGH);
            uvState = true;
            Serial.println("UV Protection ON command received");
            break;

        case 8:
            p = 7;
            mine = 2;
            autoMode = false;
            dwinWriteBytes(B, sizeof(B));
            digitalWrite(UV_PROTECTION, LOW);
            uvState = false;
            Serial.println("UV Protection OFF command received");
            break;

        case 9:
            p = 7;
            mine = 2;
            autoMode = false;
            dwinWriteBytes(I, sizeof(I));
            digitalWrite(speed1, LOW);
            digitalWrite(speed2, HIGH);
            digitalWrite(speed3, LOW);
            fanSpeed = 1;
            Serial.println("Fan Speed 1 command received");
            break;

        case 10:
            p = 7;
            mine = 2;
            autoMode = false;
            dwinWriteBytes(J, sizeof(J));
            digitalWrite(speed1, HIGH);
            digitalWrite(speed2, HIGH);
            digitalWrite(speed3, LOW);
            fanSpeed = 2;
            Serial.println("Fan Speed 2 command received");
            break;

        case 11:
            p = 7;
            mine = 2;
            autoMode = false;
            dwinWriteBytes(K, sizeof(K));
            digitalWrite(speed1, LOW);
            digitalWrite(speed2, LOW);
            digitalWrite(speed3, HIGH);
            fanSpeed = 3;
            Serial.println("Fan Speed 3 command received");
            break;

        case 12:
            p = 7;
            mine = 2;
            autoMode = false;
            dwinWriteBytes(W, sizeof(W));
            digitalWrite(speed1, LOW);
            digitalWrite(speed2, LOW);
            digitalWrite(speed3, LOW);
            fanSpeed = 0;
            Serial.println("Fan OFF command received");
            break;

        case 13:
            mine = 2;
            autoMode = false;
            p = 5;
            sleepMode = true;
            dwinWriteBytes(T, sizeof(T));
            dwinWriteBytes(C, sizeof(C));
            digitalWrite(speed1, LOW);
            digitalWrite(speed2, HIGH);
            digitalWrite(speed3, LOW);
            digitalWrite(UPPER_CHAMBER_PIN, HIGH);
            digitalWrite(LOWER_CHAMBER_PIN, LOW);
            digitalWrite(UV_PROTECTION, LOW);
            sl1 = 2;
            sl2 = 2;
            Serial.println("Sleep Mode ON command received");
            break;

        case 14:
            p = 7;
            mine = 2;
            autoMode = false;
            sleepMode = false;
            sl1 = 0;
            sl2 = 0;
            dwinWriteBytes(U, sizeof(U));
            dwinWriteBytes(D, sizeof(D));
            reset();
            Serial.println("Sleep Mode OFF command received");
            break;

        case 15:
            p = 7;
            mine = 1;
            autoMode = true;
            sleepMode = false;
            dwinWriteBytes(X, sizeof(X));
            vTaskDelay(pdMS_TO_TICKS(20));
            dwinWriteBytes(V, sizeof(V));
            Serial.println("Auto Mode ON command received");
            break;

        case 16:
            if (mod == 1) {
                dwinWriteBytes(K, sizeof(K));
                dwinWriteBytes(A, sizeof(A));
                mod = 0;
            } else if (mod == 2) {
                dwinWriteBytes(J, sizeof(J));
                dwinWriteBytes(A, sizeof(A));
                mod = 0;
            } else if (mod == 3) {
                dwinWriteBytes(I, sizeof(I));
                dwinWriteBytes(B, sizeof(B));
                mod = 0;
            } else if (mod == 4) {
                dwinWriteBytes(W, sizeof(W));
                dwinWriteBytes(B, sizeof(B));
                mod = 0;
            }

            if (text == 1) {
                dwinWriteBytes(E, sizeof(E));
                dwinWriteBytes(G, sizeof(G));
                text = 0;
            } else if (text == 2) {
                dwinWriteBytes(E, sizeof(E));
                dwinWriteBytes(H, sizeof(H));
                text = 0;
            } else if (text == 3) {
                dwinWriteBytes(F, sizeof(F));
                dwinWriteBytes(H, sizeof(H));
                text = 0;
            }

            mine = 2;
            autoMode = false;
            delay(200);
            dwinWriteBytes(Y, sizeof(Y));
            Serial.println("Auto Mode OFF command received");
            break;

        default:
            Serial.print("Unknown command received: ");
            Serial.println(message);
            break;
    }

    Serial.println("=================================");
}

const char* mqttStateMessage(int state)
{
    switch (state)
    {
        case MQTT_CONNECTION_TIMEOUT: return "connection timeout";
        case MQTT_CONNECTION_LOST: return "connection lost";
        case MQTT_CONNECT_FAILED: return "TCP/TLS connect failed";
        case MQTT_DISCONNECTED: return "disconnected";
        case MQTT_CONNECTED: return "connected";
        case MQTT_CONNECT_BAD_PROTOCOL: return "bad MQTT protocol";
        case MQTT_CONNECT_BAD_CLIENT_ID: return "bad client ID / thing name";
        case MQTT_CONNECT_UNAVAILABLE: return "broker unavailable";
        case MQTT_CONNECT_BAD_CREDENTIALS: return "bad credentials";
        case MQTT_CONNECT_UNAUTHORIZED: return "unauthorized - check AWS IoT policy/certificate";
        default: return "unknown";
    }
}

bool syncTimeForTLS()
{
    time_t now = time(nullptr);
    if (now > 1700000000)
        return true;

    Serial.println("Syncing time for TLS...");
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");

    for (int i = 0; i < 30; i++)
    {
        now = time(nullptr);
        if (now > 1700000000)
        {
            Serial.print("Time synced: ");
            Serial.println(ctime(&now));
            return true;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    Serial.println("Time sync failed. TLS certificate validation may fail.");
    return false;
}

bool connectAWS()
{
    if (client.connected())
        return true;

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi not connected!");
        return false;
    }

    syncTimeForTLS();
    printAwsNetworkDiagnostics();

    Serial.println("Connecting to AWS IoT...");

    for (int attempt = 1; attempt <= 5 && !client.connected(); attempt++)
    {
        Serial.print("AWS connect attempt ");
        Serial.println(attempt);

        if (client.connect(mac.c_str()))
        {
            Serial.println("AWS IoT Connected!");
            publishStatus();

            if (client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC.c_str()))
            {
                Serial.println("MQTT SUBSCRIBE OK");
                Serial.print("Subscribed topic: ");
                Serial.println(AWS_IOT_SUBSCRIBE_TOPIC);
            }
            else
            {
                Serial.println("MQTT SUBSCRIBE FAILED");
            }
            return true;
        }

        int state = client.state();
        Serial.print("AWS connect failed, state = ");
        Serial.print(state);
        Serial.print(" (");
        Serial.print(mqttStateMessage(state));
        Serial.println(")");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    return false;
}

void publishMessage() {

  JsonDocument doc;
  doc["NAME"] = "Airbuddi Max";
  doc["id"] = mac;
  doc["IAQ"] = IAQ;
  doc["Humidity"] = Humidity;
  doc["PM 2.5"] = PM25;
  doc["PM 10"] = PM10;
  doc["Temperature"] = temperature;
  doc["Pressure"] = pressure;
  doc["C02 Equivalent"] = Co2;
  doc["VOC's"] = Vocs;
  doc["Gas Resistance"] = Gasresistance;

  char jsonBuffer[512];
  size_t len = serializeJson(doc, jsonBuffer, sizeof(jsonBuffer));

  if (len >= sizeof(jsonBuffer)) {
    Serial.println("JSON buffer too small; message not published");
    return;
  }

  bool ok = client.publish(AWS_IOT_PUBLISH_TOPIC.c_str(), jsonBuffer);

  if (ok) {
    Serial.println("Published to AWS IoT OK");
  } else {
    Serial.println("Publish FAILED");
  }
}

bool lastPowerState = false;
bool lastAutoMode = false;
bool lastSleepMode = false;
bool lastUvcState = false;
bool lastUpperChamber = false;
bool lastLowerChamber = false;
int lastFanSpeed = -1;

bool statusInitialized = false;

bool statusChanged()
{
    bool currentPower =
        (p == 7);

    bool currentAutoMode =
        (mine == 1);

    bool currentSleepMode =
        (sl1 == 2 && sl2 == 2);

    bool currentUvc =
        (digitalRead(UV_PROTECTION) == HIGH);

    bool currentUpper =
        (digitalRead(UPPER_CHAMBER_PIN) == HIGH);

    bool currentLower =
        (digitalRead(LOWER_CHAMBER_PIN) == HIGH);

    int currentFanSpeed;

    bool s1 = digitalRead(speed1);
    bool s2 = digitalRead(speed2);
    bool s3 = digitalRead(speed3);

    if (s3 == HIGH)
    {
        currentFanSpeed = 3;
    }
    else if (s1 == HIGH && s2 == HIGH)
    {
        currentFanSpeed = 2;
    }
    else if (s1 == LOW && s2 == HIGH && s3 == LOW)
    {
        currentFanSpeed = 1;
    }
    else
    {
        currentFanSpeed = 0;
    }

    if (!statusInitialized)
    {
        lastPowerState = currentPower;
        lastAutoMode = currentAutoMode;
        lastSleepMode = currentSleepMode;
        lastUvcState = currentUvc;
        lastUpperChamber = currentUpper;
        lastLowerChamber = currentLower;
        lastFanSpeed = currentFanSpeed;

        statusInitialized = true;

        return true;
    }

    bool changed =
        currentPower != lastPowerState ||
        currentAutoMode != lastAutoMode ||
        currentSleepMode != lastSleepMode ||
        currentUvc != lastUvcState ||
        currentUpper != lastUpperChamber ||
        currentLower != lastLowerChamber ||
        currentFanSpeed != lastFanSpeed;

    if (changed)
    {
      sleepStatusChange = (currentSleepMode != lastSleepMode);
        lastPowerState = currentPower;
        lastAutoMode = currentAutoMode;
        lastSleepMode = currentSleepMode;
        lastUvcState = currentUvc;
        lastUpperChamber = currentUpper;
        lastLowerChamber = currentLower;
        lastFanSpeed = currentFanSpeed;

        return true;
    }

    return false;
}

void publishStatus()
{
    JsonDocument doc;

    doc["deviceId"] = mac;
    doc["status"] = "online";
    doc["power"] = (p == 7);
    doc["autoMode"] = (mine == 1);
    doc["sleepMode"] = (sl1 == 2 && sl2 == 2);

      bool s1 = digitalRead(speed1);
      bool s2 = digitalRead(speed2);
      bool s3 = digitalRead(speed3);

      if (s3 == HIGH)
      {
          doc["fanSpeed"] = 3;
      }
      else if (s1 == HIGH && s2 == HIGH)
      {
          doc["fanSpeed"] = 2;
      }
      else if (s1 == LOW && s2 == HIGH && s3 == LOW)
      {
          doc["fanSpeed"] = 1;
      }
      else
      {
          doc["fanSpeed"] = 0;
      }

    doc["uvc"] = (digitalRead(UV_PROTECTION) == HIGH);

    doc["upperChamber"] =
        (digitalRead(UPPER_CHAMBER_PIN) == HIGH);

    doc["lowerChamber"] =
        (digitalRead(LOWER_CHAMBER_PIN) == HIGH);

    char jsonBuffer[512];

    size_t len = serializeJson(
        doc,
        jsonBuffer,
        sizeof(jsonBuffer)
    );

    if (len >= sizeof(jsonBuffer))
    {
        Serial.println("Status JSON too large");
        return;
    }

    if (client.publish(
    AWS_IOT_STATUS_TOPIC.c_str(),
    jsonBuffer
    ))
    {
        Serial.println("===== STATUS PUBLISHED =====");
        Serial.println(jsonBuffer);
        Serial.println("============================");
    }
    else
    {
        Serial.println("STATUS PUBLISH FAILED");
    }

    sleepStatusChange = false;
}

void AWSTask(void *pvParameters)
{
    while (WiFi.status() != WL_CONNECTED)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    client.setServer(AWS_IOT_ENDPOINT, 8883);
    client.setCallback(messageHandler);
    client.setBufferSize(16384);
    client.setKeepAlive(60);
    client.setSocketTimeout(15);

    while (!connectAWS())
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    publishStatus();

    for (int i = 0; i < 20; i++)
    {
        client.loop();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TickType_t lastPublish = xTaskGetTickCount();
    int reconnectDelay = 5000;

    while (true)
    {
      // OTA has priority
      if (otaInProgress || otaRequested)
      {
          vTaskDelay(pdMS_TO_TICKS(100));
          continue;
      }
      // Wait for WiFi before doing any MQTT work
      if (WiFi.status() != WL_CONNECTED)
      {
          Serial.println("WiFi lost. Waiting for reconnect...");
          vTaskDelay(pdMS_TO_TICKS(5000));
          continue;
      }

      client.loop();

        if (!client.connected())
        {
            Serial.println("MQTT Lost. Reconnecting...");

            vTaskDelay(pdMS_TO_TICKS(reconnectDelay));

            if (connectAWS())
            {
                reconnectDelay = 5000;

                for (int i = 0; i < 20; i++)
                {
                    client.loop();
                    vTaskDelay(pdMS_TO_TICKS(100));
                }

                lastPublish = xTaskGetTickCount();
            }
            else
            {
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

void WifiManagerTask(void *pvParameters) {
  WiFi.mode(WIFI_STA);

  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      vTaskDelay(pdMS_TO_TICKS(10000));
      continue;
    }

    Serial.println("WiFi not connected. Starting WiFiManager...");
    WiFiManager wm;
    wm.setConfigPortalTimeout(120);
    bool res = wm.autoConnect("AirBuddi4", "password");
    if (!res) {
      Serial.println("Failed to connect. Retrying in 10s...");
      vTaskDelay(pdMS_TO_TICKS(10000));
    } else {
      Serial.println("WiFi connected!");
    }
  }
}

String getDefaultMacAddress()
{
    uint8_t mac_base[6] = {0};

    if (esp_efuse_mac_get_default(mac_base) != ESP_OK)
    {
        return "";
    }

    String mac = "";

    for (int i = 0; i < 6; i++)
    {
        if (i > 0)
            mac += ":";

        if (mac_base[i] < 0x10)
            mac += "0";

        mac += String(mac_base[i], HEX);
    }

    mac.toUpperCase();

    return mac;
}

void printAwsNetworkDiagnostics()
{
    Serial.print("WiFi IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("WiFi RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");

    IPAddress awsIp;
    if (WiFi.hostByName(AWS_IOT_ENDPOINT, awsIp))
    {
        Serial.print("AWS endpoint resolved to: ");
        Serial.println(awsIp);
    }
    else
    {
        Serial.println("DNS failed for AWS IoT endpoint.");
    }
}

// =====================================================
// HTTPS OTA FUNCTION
// =====================================================
void performOTA(String firmwareUrl)
{
    Serial.println();
    Serial.println("====================================");
    Serial.println("        AIRBUDDI HTTPS OTA");
    Serial.println("====================================");

    Serial.printf("Free heap before OTA: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("Largest free block: %u bytes\n", ESP.getMaxAllocHeap());

    client.disconnect();

    delay(500);

    Serial.printf("Free heap after MQTT disconnect: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("Largest free block: %u bytes\n", ESP.getMaxAllocHeap());
    Serial.printf("Free heap before TLS: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("Largest free block before TLS: %u bytes\n", ESP.getMaxAllocHeap());
    WiFiClientSecure otaClient;

    otaClient.setInsecure();

    HTTPClient http;

    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.setTimeout(30000);

    Serial.println("Connecting to firmware server...");

    if (!http.begin(otaClient, firmwareUrl))
    {
        Serial.println("OTA FAILED: HTTP begin failed");
        return;
    }

    int httpCode = http.GET();

    Serial.printf("HTTP response: %d\n", httpCode);

    if (httpCode != HTTP_CODE_OK)
    {
        Serial.printf("OTA FAILED: HTTP error = %d\n", httpCode);
        http.end();
        return;
    }

    int contentLength = http.getSize();

    Serial.printf("Firmware size: %d bytes\n", contentLength);

    if (contentLength <= 0)
    {
        Serial.println("OTA FAILED: Invalid firmware size");
        http.end();
        return;
    }

    if (!Update.begin(contentLength))
    {
        Serial.printf(
            "OTA FAILED: Update.begin error: %s\n",
            Update.errorString()
        );

        http.end();
        return;
    }

    WiFiClient *stream = http.getStreamPtr();

    uint8_t buffer[1024];

    size_t written = 0;
    unsigned long lastDataTime = millis();

    while (http.connected() && written < (size_t)contentLength)
    {
        size_t available = stream->available();

        if (available)
        {
            size_t readSize = available;

            if (readSize > sizeof(buffer))
                readSize = sizeof(buffer);

            int bytesRead = stream->readBytes(buffer, readSize);

            if (bytesRead > 0)
            {
                size_t bytesWritten = Update.write(buffer, bytesRead);

                if (bytesWritten != (size_t)bytesRead)
                {
                    Serial.println("OTA FAILED: Flash write error");
                    Update.abort();
                    http.end();
                    return;
                }

                written += bytesWritten;
                lastDataTime = millis();

                Serial.printf(
                    "OTA progress: %u / %u bytes (%u%%)\n",
                    (unsigned)written,
                    (unsigned)contentLength,
                    (unsigned)((written * 100) / contentLength)
                );
            }
        }

        if (millis() - lastDataTime > 15000)
        {
            Serial.println("OTA FAILED: Download timeout");
            Update.abort();
            http.end();
            return;
        }

        delay(1);
    }

    if (written != (size_t)contentLength)
    {
        Serial.println("OTA FAILED: Incomplete firmware download");
        Update.abort();
        http.end();
        return;
    }

    if (!Update.end())
    {
        Serial.printf(
            "OTA FAILED: Update.end error: %s\n",
            Update.errorString()
        );

        http.end();
        return;
    }

    if (!Update.isFinished())
    {
        Serial.println("OTA FAILED: Update not finished");
        http.end();
        return;
    }

    Serial.println();
    Serial.println("====================================");
    Serial.println("       OTA UPDATE SUCCESSFUL");
    Serial.println("====================================");

    http.end();

    delay(1000);

    ESP.restart();
}
