#include <Arduino.h>
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_system.h"

static void printBytes(const char* label, size_t bytes)
{
    Serial.printf("%-28s : %10u bytes  (%7.2f MB)\n",
                  label,
                  (unsigned int)bytes,
                  bytes / 1024.0 / 1024.0);
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" NiceFlightRadar V2");
    Serial.println(" Hardware Probe V2.0");
    Serial.println("========================================");
    Serial.println();

    esp_chip_info_t chipInfo;
    esp_chip_info(&chipInfo);

    Serial.printf("Chip                       : ESP32-S3\n");
    Serial.printf("CPU cores                  : %d\n", chipInfo.cores);
    Serial.printf("CPU frequency              : %u MHz\n", getCpuFrequencyMhz());
    Serial.printf("Chip revision              : %d\n", chipInfo.revision);

    Serial.println();
    Serial.println("----------- FLASH ----------------------");

    printBytes("Flash size", ESP.getFlashChipSize());

    Serial.println();
    Serial.println("----------- INTERNAL RAM ---------------");

    printBytes("Heap total", ESP.getHeapSize());
    printBytes("Heap free", ESP.getFreeHeap());
    printBytes("Heap minimum free", ESP.getMinFreeHeap());
    printBytes("Largest free block",
               heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    Serial.println();
    Serial.println("----------- PSRAM ----------------------");

    if (psramFound())
    {
        Serial.println("PSRAM                      : FOUND");
        printBytes("PSRAM total", ESP.getPsramSize());
        printBytes("PSRAM free", ESP.getFreePsram());
    }
    else
    {
        Serial.println("PSRAM                      : NOT FOUND");
    }

    Serial.println();
    Serial.println("----------- MEMORY TEST ----------------");

    const size_t testSize = 1024 * 1024;

    uint8_t* test = (uint8_t*)heap_caps_malloc(
        testSize,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    );

    if (test)
    {
        memset(test, 0x55, testSize);

        Serial.println("1 MB PSRAM allocation      : OK");

        heap_caps_free(test);
    }
    else
    {
        Serial.println("1 MB PSRAM allocation      : FAILED");
    }

    Serial.println();
    Serial.println("========================================");
    Serial.println(" Hardware Probe V2.0 READY");
    Serial.println("========================================");
}

void loop()
{
    delay(1000);
}
