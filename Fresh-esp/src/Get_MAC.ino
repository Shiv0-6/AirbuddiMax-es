#include <WiFi.h>
#include "esp_mac.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    uint8_t factoryMac[6] = {0};

    Serial.println();
    Serial.println("================================");
    Serial.println("       ESP32 MAC INFORMATION");
    Serial.println("================================");

    // Normal Wi-Fi MAC
    Serial.print("WiFi MAC:    ");
    Serial.println(WiFi.macAddress());

    // Factory/base MAC from eFuse
    if (esp_efuse_mac_get_default(factoryMac) == ESP_OK)
    {
        Serial.printf(
            "Factory MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
            factoryMac[0],
            factoryMac[1],
            factoryMac[2],
            factoryMac[3],
            factoryMac[4],
            factoryMac[5]
        );
    }
    else
    {
        Serial.println("Factory MAC: ERROR");
    }

    Serial.println("================================");
}

void loop()
{
}