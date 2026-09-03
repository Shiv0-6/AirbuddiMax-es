//AirBuddi1
//SPM1
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
#include <ArduinoOTA.h>


#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif


//Aws
#define AWS_IOT_PUBLISH_TOPIC   "AQMG_5"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/control"
#define AWS_IOT_STATUS_TOPIC    "airbuddi/status"

String mac = "";
int connectedFlag = 0;
String getDefaultMacAddress();



WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

bool connectAWS();
bool syncTimeForTLS();
void printAwsNetworkDiagnostics();
const char* mqttStateMessage(int state);
//void messageHandler(char* topic, byte* payload, unsigned int length);
void publishMessage();
void publishStatus();
void AWSTask(void *pvParameters);
void WifiManagerTask(void *pvParameters);

//BME 688
void checkIaqSensorStatus(void);
void errLeds(void);
Bsec iaqSensor;
String output;
void colorWipe(uint32_t color, int wait);
void reset(void);

//VARIABLES FOR HPMA
long lastMsg = 0;
char msg[50];
bool HPMAstatus = false;
int PM25;
int PM10;


int soilmoistsetting1 = 50;
int soilmoistsetting2 = 51;
int sl1 = 0;
int sl2 = 0;
int moist = 0;


//VARIABLES FOR DWIN DISPLAY
unsigned char Buffer[9];  //BUFFER TO STORE THE COMMAND RECEIVED FROM THE DWIN DISPLAY
uint8_t p = 5;            //led state

//VARIABLES FOR BME
float pressure;
float temperature;
float Humidity;
float Gasresistance;
float IAQ;
float Co2;
float Vocs;

//VARIABLES FOR UPPER AND LOWER CHAMBER MOISTURE
float UPPER_CHAMBER;
float LOWER_CHAMBER;

//DEFINIG WHICH SERIAL PORT SHOULD BE USED FOR WHICH DEVICE/SENSOR
HardwareSerial dwin(2);       //DWIN DISPLAY CONNECTED TO TX0 AND RX0 OF ESP32
HardwareSerial HPMA115S0(1);  //HPMA SENSOR CONNECTED TO TX2 AND RX2 OF ESP32

bool start_autosend(void);
bool receive_measurement(void);

//PIN DEFINATIONS
//RGB
#define RGB_PIN 5  //4X4 LED CONNECTED TO PIN D5
#define LED_COUNT 16

#define RXD2 16                    //TX CONNECTED TO PIN RX2 OF ESP32
#define TXD2 17                    //RX CONNECTED TO PIN TX2 OF ESP32
#define UV_PROTECTION 19           // UV LIGHT CONNECTED TO PIN D19 OF ESP32
#define UPPER_CHAMBER_PIN 12       //UPPER CHAMBER PURIFICATION CONNECTED TO PIN D15
#define LOWER_CHAMBER_PIN 15       //UPPER CHAMBER PURIFICATION CONNECTED TO PIN D15
#define SOIL_MOISTURE_SENSOR_1 25  //soil moisture 1 pin
#define SOIL_MOISTURE_SENSOR_2 32  //soil moisture 2 pin
#define Watersprinkler1 26         //water sprinkler 1 pin
#define Watersprinkler2 33         //water sprinkler 2 pin
#define speed1 4                   // FAN SPEED PIN
#define speed2 18                  // FAN SPEED PIN
#define speed3 27                  // FAN SPEED PIN

//VARIABLES FOR AUTOMODE
int mod = 0;
int text = 0;
int mine = 5;


// =====================================================
// DEVICE STATUS / STATE
// =====================================================

bool powerState = false;

int fanSpeed = 0;
// 0 = OFF
// 1 = LOW
// 2 = MEDIUM
// 3 = HIGH

bool uvState = false;

bool upperChamberState = false;
bool lowerChamberState = false;

bool sleepMode = false;
bool autoMode = false;

//VARIABLES FOR RGB LED
int mynumb;
Adafruit_NeoPixel strip(LED_COUNT, RGB_PIN, NEO_GRB + NEO_KHZ800);

//LED MATRIX PARAMETERS
int pixelInterval = 50;
uint16_t pixelCurrent = 0;
uint16_t pixelNumber = LED_COUNT;

// TO SELECT DIFFERENT BUTTONS IN THE DISPLAY WITH BLUETOOTH
unsigned char A[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X06, 0x00, 0X01 };  //(UV ON)
unsigned char B[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X06, 0x00, 0X00 };  //(UV OFF)
unsigned char C[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X07, 0x00, 0X05 };  //(SLEEP ON)
unsigned char D[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X03, 0x00, 0X07 };  //(SLEEP OFF)
unsigned char T[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X00, 0X84, 0X5A, 0X01, 0X00, 0X09 };  //(GO TO PAGE 9 - SLEEP MODE ON SCREEN)
unsigned char U[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X00, 0X84, 0X5A, 0X01, 0X00, 0X0C };  //(GO TO PAGE 12 - SLEEP MODE OFF SCREEN)
unsigned char E[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X08, 0x00, 0X01 };  //(UPPER CHAMBER ON)
unsigned char F[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X08, 0x00, 0X00 };  //(UPPER CHAMBER OFF)
unsigned char G[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X09, 0x00, 0X01 };  //(LOWER CHAMBER ON)
unsigned char H[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X09, 0x00, 0X00 };  //(LOWER CHAMBER OFF)
unsigned char I[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X05, 0x00, 0X02 };  //(FAN LOW)
unsigned char J[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X05, 0x00, 0X00 };  //(FAN MEDIUM)
unsigned char K[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X05, 0x00, 0X01 };  //(FAN HIGH)
unsigned char W[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X05, 0x00, 0X03 };  //(FAN OFF)
unsigned char X[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X10, 0x00, 0X01 };  //AUTO MODE
unsigned char Y[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X10, 0X10, 0x00, 0X00 };  //(AUTO MODE OFF) 
unsigned char L[8] = { 0X5A, 0XA5, 0X05, 0X82, 0X10, 0X11, 0X00, 0X01 };  //(TRIGGER POWER ON BUTTON)
unsigned char M[8] = { 0X5A, 0XA5, 0X05, 0X82, 0X10, 0X12, 0X00, 0X05 };  //(TRIGGER POWER OFF BUTTON)
unsigned char R[10]  = { 0X5A, 0XA5, 0X07, 0X82, 0X00, 0X84, 0X5A, 0X01, 0X00, 0X00 };  //(GO TO PAGE 0 - POWER ON BUTTON SCREEN)
unsigned char S[10] = { 0X5A, 0XA5, 0X07, 0X82, 0X00, 0X84, 0X5A, 0X01, 0X00, 0X0C };  //(GO TO PAGE 12 - POWER ON ACTIVE SCREEN)

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
  0X5A, 0XA5, 0X07, 0X82, 0X23, 0X00, 0X00  //Serial output prefix to publish upper chamber moisture
};
unsigned char o[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X29, 0X00, 0X00  //Serial output prefix to publish lower chamber moisture
};
unsigned char q[7] = {
  0X5A, 0XA5, 0X07, 0X82, 0X31, 0X00, 0X00  //Serial output prefix to publish fan mode
};
//*********************************************************************FUNCTIONS**************************************************************************************************

void bmeTask(void *pvParameters);         //FUNCTION TO HANDLE BME TASK
void hpmaTask(void *pvParameters);        //FUNCTION TO HANDLE HPMA TASK
void displayTask(void *pvParameters);     //FUNCTION TO HANDLE DWIN DISPLAY TASK
void displaycontrol(void *parameter);     //FUNCTION TO HANDLE DISPLAY CONTROL TASK
void ledlight(void *parameter);           //FUNCTION TO CONTROL RGB LED COLOURS
void SoilMoisture_1(void *pvParameters);  //FUNCTION TO HANDLE SOIL MOISTURE 1
void SoilMoisture_2(void *pvParameters);  //FUNCTION TO HANDLE SOIL MOISTURE 2
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX-END-XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

// *************************************************************************-Task handles-**********************************************************************************************
TaskHandle_t bmeTaskHandle;          //TASK HANDLE FOR BME
TaskHandle_t hpmaTaskHandle;         //TASK HANDLE FOR HPMA
TaskHandle_t displayTaskHandle;      //TASK HANDLE FOR DISPLAY
TaskHandle_t controltaskhandle;      //TASK HANDLE FOR DISPLAY CONTROL
TaskHandle_t ledlighttaskhandle;     //TASK HANDLE FOR LED TASK
TaskHandle_t SoilMoisture_1_Handle;  //TASK HANDLE FOR SOIL MOISTURE SENSOR 2
TaskHandle_t SoilMoisture_2_Handle;  //TASK HANDLE FOR SOIL MOISTURE SENSOR 2
TaskHandle_t AWSTaskHandle = NULL;
TaskHandle_t WifiManagerTaskHandle = NULL;
TaskHandle_t OTATaskHandle = NULL;
TaskHandle_t builtinLedTaskHandle = NULL;

volatile bool builtinLedRunning = true;
volatile bool led12Running = false;
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX-END-XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

//*****************************************************************************-SETUP-**************************************************************************************************
void setup() {
  Serial.begin(115200);
  dwin.begin(115200, SERIAL_8N1, 23, 13);  // RX first, then TX

  mac = getDefaultMacAddress();

    Serial.print("Device MAC: ");
    Serial.println(mac);

  //******************************************************************************-PIN DEFINITIONS-*****************************************************************************************
  pinMode(speed1, OUTPUT);                 //FAN PIN SET AS AN OUTPUT
  pinMode(speed2, OUTPUT);                 //FAN PIN SET AS AN OUTPUT
  pinMode(speed3, OUTPUT);                 //FAN PIN SET AS AN OUTPUT
  pinMode(UV_PROTECTION, OUTPUT);          //UV PROTECTION SET AS AN OUTPUT
  pinMode(UPPER_CHAMBER_PIN, OUTPUT);      //UPPER CHAMBER AS AN OUTPUT
  pinMode(LOWER_CHAMBER_PIN, OUTPUT);      //LOWER CHAMBER SET AS AN OUTPUT
  pinMode(SOIL_MOISTURE_SENSOR_1, INPUT);  //SOIL MOISTURE SENSOR 1  PIN SET AS AN INPUT FOR READING
  pinMode(SOIL_MOISTURE_SENSOR_2, INPUT);  //SOIL MOISTURE SENSOR 2  PIN SET AS AN INPUT FOR READING
  pinMode(Watersprinkler1, OUTPUT);        //WATER PUMP CONNECTED TO SOIL MOSTURE SENSOR 1
  pinMode(Watersprinkler2, OUTPUT);        //WATER PUMP CONNECTED TO SOIL MOSTURE SENSOR 2
  digitalWrite(speed1, LOW);               //FAN INITIALLY OFF
  digitalWrite(speed2, LOW);               //FAN INITIALLY OFF
  digitalWrite(speed3, LOW);               //FAN INITIALLY OFF
  digitalWrite(UV_PROTECTION, LOW);        //UV PROTECTION INITIALLY OFF
  digitalWrite(UPPER_CHAMBER_PIN, LOW);    //UPPER_CHAMBER INITIALLY OFF
  digitalWrite(LOWER_CHAMBER_PIN, LOW);    //LOWER_CHAMBER INITIALLY OFF
  digitalWrite(Watersprinkler1, LOW);      //UPPER_CHAMBER INITIALLY OFF
  digitalWrite(Watersprinkler2, LOW);      //LOWER_CHAMBER INITIALLY OFF

  //****************************************************************************-Create tasks-*******************************************************************************************************
  xTaskCreate(AWSTask, "AWS Task", 16384, NULL, 3, &AWSTaskHandle);
  xTaskCreate(displaycontrol, "Display Control", 16384, NULL, 2, &controltaskhandle);
  xTaskCreate(ledlight, "RGBLED", 2048, NULL, 1, &ledlighttaskhandle);                  //TASK CREATED FOR RGB LED LIGHTS
  xTaskCreate(displayTask, "Display Task", 4096, NULL, 1, &displayTaskHandle);          //TASK CREATED FOR DISPLAY(SIR)
  xTaskCreate(SoilMoisture_1, "SoilMoisture1", 4096, NULL, 1, &SoilMoisture_1_Handle);  //TASK CREATED FOR SOIL MOISTURE 1
  xTaskCreate(SoilMoisture_2, "SoilMoisture2", 4096, NULL, 1, &SoilMoisture_2_Handle);  //TASK CREATED FOR SOIL MOISTURE 2 
  xTaskCreate(WifiManagerTask, "WifiManager Task", 4096, NULL, 1, &WifiManagerTaskHandle);  
  xTaskCreate(bmeTask, "BME Task", 8192, NULL, 1, &bmeTaskHandle);                      //TASK CREATED FOR BME
  xTaskCreate(hpmaTask, "HPMA Task", 8192, NULL, 1, &hpmaTaskHandle);                   //TASK CREATED FOR HPMA
  //xTaskCreate(OTATask, "OTA Task", 4096, NULL, 2, &OTATaskHandle);  //TASK CREATED FOR DWIN DISPLAY CONTROL(AVAPSC)
}
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX-END-XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
void loop() {}
//******************************************************************************-RGB LED TASK-***************************************************************************************************
void ledlight(void *parameter) {
  while (1) {
    int lm = 0;
    mynumb = PM25;
    if (p != 5) {  // If fan is ON6
      if (mynumb > 500) {
        for (lm = 0; lm < 16; lm++) {
          colorWipe(strip.Color(255, 0, 0), 50);  // Red
        }
      }

      else if (mynumb > 100) {
        for (lm = 0; lm < 16; lm++) {
          colorWipe(strip.Color(0, 0, 255), 50);  // Blue
        }
      }

      else {
        for (lm = 0; lm < 16; lm++) {
          colorWipe(strip.Color(0, 255, 0), 50);  // Green
        }
      }
    } else {  // If fan is OFF, turn off RGB LED
      colorWipe(strip.Color(0, 0, 0), 50);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  //DELAY OF 1 SECOND BEFORE NEXT EXECUTION OF THE LOOP
  }
}


//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX-END-XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

//******************************************************************************-DISPLAY TASK-*******************************************************************************************************
void displayTask(void *pvParameters) {
  while (1) {
    long g;
    long h;
    long i;
    long j;
    int soilMoistureValue1 = analogRead(SOIL_MOISTURE_SENSOR_1);
    int soilMoisturePercent1 = map(soilMoistureValue1, 4095, 0, 0, 100);
    UPPER_CHAMBER = soilMoisturePercent1;
    int soilMoistureValue2 = analogRead(SOIL_MOISTURE_SENSOR_2);
    int soilMoisturePercent2 = map(soilMoistureValue2, 4095, 0, 0, 100);
    LOWER_CHAMBER = soilMoisturePercent2;
    if (UPPER_CHAMBER <= 35 && LOWER_CHAMBER <= 35) {
      moist = 1;
    }

    else {
      moist = 0;
    }
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

    g = UPPER_CHAMBER;
    i = g >> 8;
    j = g >> 16;
    dwin.write(n, 7);
    dwin.write(j);
    dwin.write(i & 0x0000FF);
    dwin.write(g & 0x0000FF);

    g = LOWER_CHAMBER;
    i = g >> 8;
    j = g >> 16;
    dwin.write(o, 7);
    dwin.write(j);
    dwin.write(i & 0x0000FF);
    dwin.write(g & 0x0000FF);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX-END-XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX



//******************************************************************************-SOIL MOISTURE SENSOR-***************************************************************************************************
// //FUNCTION TO RUN SOIL MOISTURE SENSOR 1
void SoilMoisture_1(void *pvParameters) {
  while (1) {
    int soilMoistureValue1 = analogRead(SOIL_MOISTURE_SENSOR_1);
    int soilMoisturePercent1 = map(soilMoistureValue1, 4095, 0, 0, 100);
    UPPER_CHAMBER = soilMoisturePercent1;
    if (UPPER_CHAMBER <= soilmoistsetting1 && sl1 == 0) {
      digitalWrite(Watersprinkler1, HIGH);
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      digitalWrite(Watersprinkler1, LOW);
      vTaskDelay(10800000 / portTICK_PERIOD_MS);  //10800000(3hours)

    } else {
      digitalWrite(Watersprinkler1, LOW);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);  //DELAY OF 1 SECOND BEFORE NEXT EXECUTION OF THE LOOP
  }
}
//FUNCTION TO RUN SOIL MOISTURE SENSOR 2
void SoilMoisture_2(void *pvParameters) {
  while (1) {
    int soilMoistureValue2 = analogRead(SOIL_MOISTURE_SENSOR_2);
    int soilMoisturePercent2 = map(soilMoistureValue2, 4095, 0, 0, 100);
    LOWER_CHAMBER = soilMoisturePercent2;
    if (LOWER_CHAMBER <= soilmoistsetting2 && sl2 == 0 && moist == 0) {
      digitalWrite(Watersprinkler2, HIGH);
      vTaskDelay(5000 / portTICK_PERIOD_MS);
      digitalWrite(Watersprinkler2, LOW);
      vTaskDelay(10800000 / portTICK_PERIOD_MS);
    } else {
      digitalWrite(Watersprinkler2, LOW);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);  //DELAY OF 1 SECOND BEFORE NEXT EXECUTION OF THE LOOP
  }
}
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX-END-XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX



//*******************************************************************DISPLAY AND RGB LED CONTROL COMMANDS*********************************************************************************************
void displaycontrol(void *parameter) {
  while (1) {
    int lm = 0;     // LED Matrix
    mynumb = PM25;  //VARIABLE TO STORE PM2.5 VALUE
    memset(Buffer, 0, 9);


    if (dwin.available()) {
      for (int i = 0; i <= 8; i++) {
        Buffer[i] = dwin.read();
      }
    }

    switch (Buffer[5]) {

      case 0X05:
        switch (Buffer[8]) {
          case 0X03:  // FOR OFF SPEED (OFF)
            digitalWrite(speed1, LOW);
            digitalWrite(speed2, LOW);
            digitalWrite(speed3, LOW);
            break;

          case 0X00:  //FOR SPEED 3(MEDIUM)
            digitalWrite(speed1, HIGH);
            digitalWrite(speed2, HIGH);
            digitalWrite(speed3, LOW);
            break;

          case 0X01:  //FOR FULL SPEED 4(HIGH)
            digitalWrite(speed1, LOW);
            digitalWrite(speed2, LOW);
            digitalWrite(speed3, HIGH);
            break;


          case 0X02:  //FOR  SPEED 2(LOW)
            digitalWrite(speed1, LOW);
            digitalWrite(speed2, HIGH);
            digitalWrite(speed3, LOW);
            break;
        }
        break;


      case 0X06:
        switch (Buffer[8]) {
          case 0X00:
            digitalWrite(UV_PROTECTION, LOW);
            break;

          case 0X01:
            digitalWrite(UV_PROTECTION, HIGH);
            break;
        }
        break;



      case 0X07:  //SLEEP MODE ON
        switch (Buffer[8]) {
          case 0X05:
            p = 5;
            digitalWrite(speed1, LOW);
            digitalWrite(speed2, HIGH);
            digitalWrite(speed3, LOW);
            digitalWrite(UPPER_CHAMBER_PIN, HIGH);
            digitalWrite(LOWER_CHAMBER_PIN, LOW);
            digitalWrite(UV_PROTECTION, LOW);
            sl1 = 2;
            sl2 = 2;
            break;
        }
        break;


      case 0X03:  //SLEEP MODE OFF
        switch (Buffer[8]) {
          case 0X07:
            p = 7;
            reset();
            break;
        }
        break;



      case 0X08:  //UPPER CHAMBER ON AND OFF
        switch (Buffer[8]) {
          case 0X00:
            digitalWrite(UPPER_CHAMBER_PIN, LOW);
            break;

          case 0X01:
            digitalWrite(UPPER_CHAMBER_PIN, HIGH);
            break;
        }
        break;



      case 0X09:  //LOWER CHAMBER ON AND OFF
        switch (Buffer[8]) {
          case 0X00:
            digitalWrite(LOWER_CHAMBER_PIN, LOW);
            break;

          case 0X01:
            digitalWrite(LOWER_CHAMBER_PIN, HIGH);
            break;
        }
        break;

      case 0X10:  //TURN OFF AUTO MODE ENTER MANUAL MODE                        //AUTO MODE HIGHLIGHTS AND BUTTON MODE
        switch (Buffer[8]) {
          case 0X00:
            if (mod == 1) {
              //TO TURN FAN HIGH
              for (int i = 0; i < sizeof(K); i++) {
                dwin.write(K[i]);
              }

              for (int i = 0; i < sizeof(A); i++) {
                dwin.write(A[i]);
              }
              mod = 0;
            }

            else if (mod == 2) {
              //TO TURN FAN MEDIUM

              for (int i = 0; i < sizeof(J); i++) {
                dwin.write(J[i]);
              }

              for (int i = 0; i < sizeof(A); i++) {
                dwin.write(A[i]);
              }
              mod = 0;

            }

            else if (mod == 3) {
              //TO TURN FAN OFF
              for (int i = 0; i < sizeof(I); i++) {
                dwin.write(I[i]);
              }

              for (int i = 0; i < sizeof(B); i++) {
                dwin.write(B[i]);
              }
              mod = 0;
            }

            else if (mod == 4) {
              //TO TURN FAN OFF
              for (int i = 0; i < sizeof(W); i++) {
                dwin.write(W[i]);
              }

              for (int i = 0; i < sizeof(B); i++) {
                dwin.write(B[i]);
              }
              mod = 0;
            }

            if (text == 1) {
              //TO TURN FAN OFF
              for (int i = 0; i < sizeof(E); i++) {
                dwin.write(E[i]);
              }

              for (int i = 0; i < sizeof(G); i++) {
                dwin.write(G[i]);
              }
              text = 0;
            } else if (text == 2) {
              //TO TURN FAN OFF
              for (int i = 0; i < sizeof(E); i++) {
                dwin.write(E[i]);
              }

              for (int i = 0; i < sizeof(H); i++) {
                dwin.write(H[i]);
              }
              text = 0;

            }

            else if (text == 3) {
              //TO TURN FAN OFF
              for (int i = 0; i < sizeof(F); i++) {
                dwin.write(F[i]);
              }

              for (int i = 0; i < sizeof(H); i++) {
                dwin.write(H[i]);
              }
              text = 0;
            }
            mine = 2;

            break;


          case 0X02:  // TO TURN ON AUTO MODE
            mine = 1;
            break;
        }
      case 0X11:  //POWER ON
        switch (Buffer[8]) {
          case 0X01:
            p = 7;
            //TO TURN FAN OFF
            for (int i = 0; i < sizeof(X); i++) {
              dwin.write(X[i]);
            }
            break;
        }
      case 0X12:  //POWER OFF
        switch (Buffer[8]) {
          case 0X05:
            mine = 2;
            p = 5;
            reset();
            break;
        }
    }

    switch (Buffer[4]) {

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




    if (mine == 1) {
      if (mynumb >= 500) {
        mod = 1;
        digitalWrite(speed1, LOW);
        digitalWrite(speed2, LOW);
        digitalWrite(speed3, HIGH);
        digitalWrite(UV_PROTECTION, HIGH);
      } else if (100 < mynumb && mynumb < 500) {
        mod = 2;
        digitalWrite(speed1, HIGH);
        digitalWrite(speed2, HIGH);
        digitalWrite(speed3, LOW);
        digitalWrite(UV_PROTECTION, HIGH);
      } else if (40 < mynumb && mynumb <= 100) {
        mod = 3;
        digitalWrite(speed1, LOW);
        digitalWrite(speed2, HIGH);
        digitalWrite(speed3, LOW);
        digitalWrite(UV_PROTECTION, LOW);
      } else if (mynumb <= 40) {
        mod = 4;
        digitalWrite(speed1, LOW);
        digitalWrite(speed2, LOW);
        digitalWrite(speed3, LOW);
        digitalWrite(UV_PROTECTION, LOW);
      }

      if (Vocs > 0.75) {
        text = 1;
        digitalWrite(UPPER_CHAMBER_PIN, HIGH);
        digitalWrite(LOWER_CHAMBER_PIN, HIGH);
      } else if (0.25 < Vocs && Vocs <= 0.75) {
        text = 2;
        digitalWrite(UPPER_CHAMBER_PIN, HIGH);
        digitalWrite(LOWER_CHAMBER_PIN, LOW);

      } else if (Vocs <= 0.25) {
        text = 3;
        digitalWrite(UPPER_CHAMBER_PIN, LOW);
        digitalWrite(LOWER_CHAMBER_PIN, LOW);
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

//Helper funtion for the full command array
void sendCommand(const unsigned char *cmd, size_t len) {
  for (size_t i = 0; i < len; i++) {
    dwin.write(cmd[i]);
  }
}


//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX-END-XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

//******************************************************************************-BME FUNCTION-*********************************************************************************************************
// Task functions
void bmeTask(void *pvParameters) {
  pinMode(2, OUTPUT);
  iaqSensor.begin(BME68X_I2C_ADDR_LOW, Wire);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  //Serial.println(output);
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
    if (iaqSensor.run()) {  // If new data is available
      digitalWrite(2, LOW);
      output = String(time_trigger);
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
      //Serial.println(output);
      digitalWrite(2, HIGH);
    } else {
      checkIaqSensorStatus();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX-END-XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

//******************************************************************************-HPMA FUNCTION-******************************************************************************************************
//OK
void hpmaTask(void *pvParameters) {
  HPMA115S0.begin(9600, SERIAL_8N1, RXD2, TXD2);
  while (!HPMA115S0)
    ;
  start_autosend();

  while (1) {


    HPMAstatus = receive_measurement();
    if (!HPMAstatus) {

      Serial.println("Cannot receive data from HPMA115S0!");
      return;
    }
    snprintf(msg, 16, "%D", PM25);

    snprintf(msg, 16, "%D", PM10);
    if (PM10 != 0) {

      Serial.println("PM 2.5:\t" + String(PM25) + " ug/m3");
      Serial.println("PM 10:\t" + String(PM10) + " ug/m3");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX-END-XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
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
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(100);
  digitalWrite(2, LOW);
  delay(100);
}

bool start_autosend(void) {
  // Start auto send
  byte start_autosend[] = { 0x68, 0x01, 0x40, 0x57 };
  HPMA115S0.write(start_autosend, sizeof(start_autosend));
  HPMA115S0.flush();
  delay(500);
  //Then we wait for the response
  while (HPMA115S0.available() < 2)
    ;
  byte read1 = HPMA115S0.read();
  byte read2 = HPMA115S0.read();
  // Test the response
  if ((read1 == 0xA5) && (read2 == 0xA5)) {
    // ACK
    return true;
  } else if ((read1 == 0x96) && (read2 == 0x96)) {
    // NACK
    return false;
  } else {
    return false;
  }
}

//OK
bool receive_measurement(void) {
  unsigned long startTime = millis();
  int startWait = millis();
  while (HPMA115S0.available() < 32)
    if(millis() - startWait > 100) {
      return false; //timeout bail-out
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  byte HEAD0 = HPMA115S0.read();
  byte HEAD1 = HPMA115S0.read();
  while (HEAD0 != 0x42) {
    if (HEAD1 == 0x42) {
      HEAD0 = HEAD1;
      HEAD1 = HPMA115S0.read();
    } else {
      HEAD0 = HPMA115S0.read();
      HEAD1 = HPMA115S0.read();
    }
  }
  if (HEAD0 == 0x42 && HEAD1 == 0x4D) {
    byte LENH = HPMA115S0.read();
    byte LENL = HPMA115S0.read();
    byte Data0H = HPMA115S0.read();
    byte Data0L = HPMA115S0.read();
    byte Data1H = HPMA115S0.read();
    byte Data1L = HPMA115S0.read();
    byte Data2H = HPMA115S0.read();
    byte Data2L = HPMA115S0.read();
    byte Data3H = HPMA115S0.read();
    byte Data3L = HPMA115S0.read();
    byte Data4H = HPMA115S0.read();
    byte Data4L = HPMA115S0.read();
    byte Data5H = HPMA115S0.read();
    byte Data5L = HPMA115S0.read();
    byte Data6H = HPMA115S0.read();
    byte Data6L = HPMA115S0.read();
    byte Data7H = HPMA115S0.read();
    byte Data7L = HPMA115S0.read();
    byte Data8H = HPMA115S0.read();
    byte Data8L = HPMA115S0.read();
    byte Data9H = HPMA115S0.read();
    byte Data9L = HPMA115S0.read();
    byte Data10H = HPMA115S0.read();
    byte Data10L = HPMA115S0.read();
    byte Data11H = HPMA115S0.read();
    byte Data11L = HPMA115S0.read();
    byte Data12H = HPMA115S0.read();
    byte Data12L = HPMA115S0.read();
    byte CheckSumH = HPMA115S0.read();
    byte CheckSumL = HPMA115S0.read();

    if (((HEAD0 + HEAD1 + LENH + LENL + Data0H + Data0L + Data1H + Data1L + Data2H + Data2L + Data3H + Data3L + Data4H + Data4L + Data5H + Data5L + Data6H + Data6L + Data7H + Data7L + Data8H + Data8L + Data9H + Data9L + Data10H + Data10L + Data11H + Data11L + Data12H + Data12L) % 256) != CheckSumL) {
      Serial.println("Checksum fail");
      HPMA115S0.flush();
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      Serial.println("BUFFER FLUSHED");
      return true;
    }
    PM25 = (Data1H * 256) + Data1L;
    PM10 = (Data2H * 256) + Data2L;
    return true;
  }
  return false;
}

void colorWipe(uint32_t color, int wait) {
  if (pixelInterval != wait)
    pixelInterval = wait;                    //  Update delay time
  strip.setPixelColor(pixelCurrent, color);  //  Set pixel's color (in RAM)
  strip.show();                              //  Update strip to match
  pixelCurrent++;                            //  Advance current pixel
  if (pixelCurrent >= pixelNumber)           //  Loop the pattern from the first LED
    pixelCurrent = 0;
}

void reset(void) {

  //TO TURN OFF LOWER CHAMBER
  for (int i = 0; i < sizeof(H); i++) {
    dwin.write(H[i]);
  }

  //TO TURN OFF UV PROTECTION
  for (int i = 0; i < sizeof(B); i++) {
    dwin.write(B[i]);
  }

  //TO TURN OFF FAN
  for (int i = 0; i < sizeof(W); i++) {
    dwin.write(W[i]);
  }

  //TO TURN OFF UPPER CHAMBER
  for (int i = 0; i < sizeof(F); i++) {
    dwin.write(F[i]);
  }
  vTaskDelay(5 / portTICK_PERIOD_MS);
  digitalWrite(speed1, LOW);
  digitalWrite(speed2, LOW);
  digitalWrite(speed3, LOW);
  digitalWrite(UV_PROTECTION, LOW);
  digitalWrite(UPPER_CHAMBER_PIN, LOW);
  digitalWrite(LOWER_CHAMBER_PIN, LOW);
  sl1 = 0;
  sl2 = 0;
}

void messageHandler(char* topic, byte* payload, unsigned int length)
{
    // Convert MQTT payload to String
    String msg;

    for (unsigned int i = 0; i < length; i++)
    {
        msg += (char)payload[i];
    }

    msg.trim();

    // Debug information
    Serial.println();
    Serial.println("===== MQTT MESSAGE RECEIVED =====");
    Serial.print("Topic: ");
    Serial.println(topic);
    Serial.print("Raw Payload: ");
    Serial.println(msg);

    // Check expected topic
    if (String(topic) != AWS_IOT_SUBSCRIBE_TOPIC)
    {
        Serial.println("Unexpected MQTT topic!");
        Serial.println("=================================");
        return;
    }

    // Parse JSON
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, msg);

    if (error)
    {
        Serial.print("JSON Error: ");
        Serial.println(error.c_str());
        Serial.println("=================================");
        return;
    }

    // Extract message
    String message = doc["message"] | "";
    message.trim();

    Serial.print("Command: ");
    Serial.println(message);

    // =====================================================
    // CONVERT STRING COMMAND TO INTEGER
    // =====================================================

    int command = 0;

    if (message == "power_on")       command = 1;
    else if (message == "power_off")       command = 2;
    else if (message == "lower_on")     command = 3;
    else if (message == "lower_off")    command = 4;
    else if (message == "upper_on")     command = 5;
    else if (message == "upper_off")    command = 6;
    else if (message == "uvc_on")     command = 7;
    else if (message == "uvc_off")    command = 8;
    else if (message == "fan_1")   command = 9;
    else if (message == "fan_2")   command = 10;
    else if (message == "fan_3")   command = 11;
    else if (message == "fan_off") command = 12;
    else if (message == "sleep_on")  command = 13;
    else if (message == "sleep_off") command = 14;
    else if (message == "auto_on")   command = 15;
    else if (message == "auto_off")  command = 16;

    // =====================================================
    // SWITCH CASE
    // =====================================================

    switch (command)
    {
        case 1:
            // POWER ON
            p = 7;

            dwin.write(S, 10);
            dwin.write(L, 8);
            Serial.println("Power ON command received");
            Serial.print("p = ");
            Serial.println(p);
            break;


        case 2:
            // POWER OFF
            p = 5;

            dwin.write(R, 10);
            dwin.write(M, 8);
            Serial.println("Power OFF command received");
            Serial.print("p = ");
            Serial.println(p);
            break;


        case 3:
            // LOWER CHAMBER ON
            dwin.write(G, 8);
            digitalWrite(LOWER_CHAMBER_PIN, HIGH);

            Serial.println("Lower chamber ON command received");
            break;


        case 4:
            // LOWER CHAMBER OFF
            dwin.write(H, 8);
            digitalWrite(LOWER_CHAMBER_PIN, LOW);

            Serial.println("Lower chamber OFF command received");
            break;


        case 5:
            // UPPER CHAMBER ON
            dwin.write(E, 8);
            digitalWrite(UPPER_CHAMBER_PIN, HIGH);

            Serial.println("Upper chamber ON command received");
            break;


        case 6:
            // UPPER CHAMBER OFF
            dwin.write(F, 8);
            digitalWrite(UPPER_CHAMBER_PIN, LOW);

            Serial.println("Upper chamber OFF command received");
            break;


        case 7:
            // UV ON
            dwin.write(A, 8);
            digitalWrite(UV_PROTECTION, HIGH);

            Serial.println("UV Protection ON command received");
            break;


        case 8:
            // UV OFF
            dwin.write(B, 8);
            digitalWrite(UV_PROTECTION, LOW);

            Serial.println("UV Protection OFF command received");
            break;


        case 9:
            // FAN SPEED 1
            dwin.write(I, 8);
            digitalWrite(speed1, LOW);
            digitalWrite(speed2, HIGH);
            digitalWrite(speed3, LOW);

            Serial.println("Fan Speed 1 command received");
            break;


        case 10:
            // FAN SPEED 2
            dwin.write(J, 8);
            digitalWrite(speed1, HIGH);
            digitalWrite(speed2, HIGH);
            digitalWrite(speed3, LOW);

            Serial.println("Fan Speed 2 command received");
            break;


        case 11:
            // FAN SPEED 3
            dwin.write(K, 8);
            digitalWrite(speed1, LOW);
            digitalWrite(speed2, LOW);
            digitalWrite(speed3, HIGH);

            Serial.println("Fan Speed 3 command received");
            break;


        case 12:
            // FAN OFF
            dwin.write(W, 8);
            digitalWrite(speed1, LOW);
            digitalWrite(speed2, LOW);
            digitalWrite(speed3, LOW); 

            Serial.println("Fan OFF command received");
            break;


        case 13:
            // SLEEP MODE ON
            p = 5;

            dwin.write(T, 10); 
            dwin.write(C, 8);
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
            // SLEEP MODE OFF
            p = 7;

            dwin.write(U, 10);
            dwin.write(D, 8);
            reset();

            Serial.println("Sleep Mode OFF command received");
            break;
        
        case 15:
    // AUTO MODE ON
    mine = 1;
    dwin.write(X, 8);
    Serial.println("Auto Mode ON command received");
    break;


case 16:
    // AUTO MODE OFF
    if (mod == 1) {
        for (int i = 0; i < sizeof(K); i++) dwin.write(K[i]);
        for (int i = 0; i < sizeof(A); i++) dwin.write(A[i]);
        mod = 0;
    }
    else if (mod == 2) {
        for (int i = 0; i < sizeof(J); i++) dwin.write(J[i]);
        for (int i = 0; i < sizeof(A); i++) dwin.write(A[i]);
        mod = 0;
    }
    else if (mod == 3) {
        for (int i = 0; i < sizeof(I); i++) dwin.write(I[i]);
        for (int i = 0; i < sizeof(B); i++) dwin.write(B[i]);
        mod = 0;
    }
    else if (mod == 4) {
        for (int i = 0; i < sizeof(W); i++) dwin.write(W[i]);
        for (int i = 0; i < sizeof(B); i++) dwin.write(B[i]);
        mod = 0;
    }

    if (text == 1) {
        for (int i = 0; i < sizeof(E); i++) dwin.write(E[i]);
        for (int i = 0; i < sizeof(G); i++) dwin.write(G[i]);
        text = 0;
    }
    else if (text == 2) {
        for (int i = 0; i < sizeof(E); i++) dwin.write(E[i]);
        for (int i = 0; i < sizeof(H); i++) dwin.write(H[i]);
        text = 0;
    }
    else if (text == 3) {
        for (int i = 0; i < sizeof(F); i++) dwin.write(F[i]);
        for (int i = 0; i < sizeof(H); i++) dwin.write(H[i]);
        text = 0;
    }

    mine = 2;
    dwin.write(Y, 8);
    Serial.println("Auto Mode OFF command received");
    break;    

        default:
            // UNKNOWN COMMAND
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

    // TLS certs and MQTT server are configured once in AWSTask init

    Serial.println("Connecting to AWS IoT...");

    for (int attempt = 1; attempt <= 5 && !client.connected(); attempt++)
    {
        Serial.print("AWS connect attempt ");
        Serial.println(attempt);

        if (client.connect(THINGNAME))
        {
            Serial.println("AWS IoT Connected!");
            publishStatus();

            // Subscribe again after every reconnect
            if (client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC))
                Serial.println("AWS IoT subscribe OK");
            else
                Serial.println("AWS IoT subscribe failed");

            return true;
        }

        int state = client.state();
        Serial.print("AWS connect failed, state = ");
        Serial.print(state);
        Serial.print(" (");
        Serial.print(mqttStateMessage(state));
        Serial.println(")");

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return false;
}

void publishMessage() {
  
  JsonDocument doc;
  doc["NAME"] = "MONITOR 3"; 
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

  // Avoid calling client.connected() / client.state() here — those
  // PubSubClient calls trigger internal disconnect detection and can
  // tear down the TCP socket if the broker sent a FIN between packets.
  bool ok = client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);

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

    // First call: establish baseline
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

    // Check for changes
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

    // Device identity
    doc["deviceId"] = mac;

    // Connection status
    doc["status"] = "online";

    // Power
    doc["power"] = (p == 7);

    // Operating modes
    doc["autoMode"] = (mine == 1);
    doc["sleepMode"] = (sl1 == 2 && sl2 == 2);

    // Fan speed
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

    // UV
    doc["uvc"] = (digitalRead(UV_PROTECTION) == HIGH);

    // Chambers
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
        AWS_IOT_STATUS_TOPIC,
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
}

void AWSTask(void *pvParameters)
{
    // Wait for WiFi connection
    while (WiFi.status() != WL_CONNECTED)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // ---- One-time TLS + MQTT configuration ----
    net.setCACert(AWS_CERT_CA);
    net.setCertificate(AWS_CERT_CRT);
    net.setPrivateKey(AWS_CERT_PRIVATE);

    client.setServer(AWS_IOT_ENDPOINT, 8883);
    client.setCallback(messageHandler);
    client.setBufferSize(1024);
    client.setKeepAlive(60);
    client.setSocketTimeout(15);

    // Initial connection
    while (!connectAWS())
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    publishStatus();

    // Let the connection & subscription fully stabilize
    for (int i = 0; i < 20; i++)
    {
        client.loop();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TickType_t lastPublish = xTaskGetTickCount();
    int reconnectDelay = 5000;  // ms, with exponential backoff

    while (true)
    {
        // Process incoming MQTT data FIRST (keepalive, SUBACK, etc.)
        client.loop();

        // Then check connection status
        if (!client.connected())
        {
            Serial.println("MQTT Lost. Reconnecting...");

            vTaskDelay(pdMS_TO_TICKS(reconnectDelay));

            if (connectAWS())
            {
                reconnectDelay = 5000;  // reset backoff

                // Stabilise after reconnection
                for (int i = 0; i < 20; i++)
                {
                    client.loop();
                    vTaskDelay(pdMS_TO_TICKS(100));
                }

                lastPublish = xTaskGetTickCount();
            }
            else
            {
                // Exponential backoff, max 60 s
                reconnectDelay = min(reconnectDelay * 2, 60000);
            }

            continue;  // re-enter loop from the top
        }

        //Check purifier status
        if (statusChanged())
        {
            publishStatus();
        }

        // Publish every 5 seconds
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



String getDefaultMacAddress() {
  unsigned char mac_base[6] = {0};
  if (esp_efuse_mac_get_default(mac_base) == ESP_OK) {
    char buffer[18];  // 6*2 characters for hex + 5 characters for colons + 1 character for null terminator
    sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);
    mac = buffer;
  }
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
