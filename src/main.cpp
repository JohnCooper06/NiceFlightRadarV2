#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>

// ============================================================
// NiceFlightRadar V2
// Display Probe V2.1
//
// Hardware:
// ESP32-4848S040
// ESP32-S3 / N16R8
// ST7701 RGB 480x480
// ============================================================

#define TFT_WIDTH   480
#define TFT_HEIGHT  480
#define TFT_BL      38

#define TOUCH_SDA   19
#define TOUCH_SCL   45
#define GT911_ADDR1 0x5D
#define GT911_ADDR2 0x14

// ST7701 configuration bus
Arduino_ESP32SPI *panelBus = new Arduino_ESP32SPI(
    GFX_NOT_DEFINED, // DC
    39,              // CS
    48,              // SCK
    47,              // MOSI
    GFX_NOT_DEFINED  // MISO
);

// RGB parallel panel
Arduino_ESP32RGBPanel *rgbPanel = new Arduino_ESP32RGBPanel(
    18, // DE
    17, // VSYNC
    16, // HSYNC
    21, // PCLK

    // RED
    11, // R0
    12, // R1
    13, // R2
    14, // R3
    0,  // R4

    // GREEN
    8,  // G0
    20, // G1
    3,  // G2
    46, // G3
    9,  // G4
    10, // G5

    // BLUE
    4,  // B0
    5,  // B1
    6,  // B2
    7,  // B3
    15, // B4

    // Horizontal timing
    1,  // HSYNC polarity
    10, // front porch
    8,  // pulse width
    50, // back porch

    // Vertical timing
    1,  // VSYNC polarity
    10, // front porch
    8,  // pulse width
    20  // back porch
);

Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    TFT_WIDTH,
    TFT_HEIGHT,
    rgbPanel,
    0,     // rotation
    true,  // auto flush
    panelBus,
    GFX_NOT_DEFINED,
    st7701_type9_init_operations,
    sizeof(st7701_type9_init_operations)
);


bool gt911Read(uint16_t reg, uint8_t *data, size_t len)
{
    Wire.beginTransmission(GT911_ADDR1);
    Wire.write((reg >> 8) & 0xFF);
    Wire.write(reg & 0xFF);

    if (Wire.endTransmission(false) != 0)
        return false;

    size_t received = Wire.requestFrom((uint8_t)GT911_ADDR1, len);

    if (received != len)
        return false;

    for (size_t i = 0; i < len; i++)
        data[i] = Wire.read();

    return true;
}

bool gt911Present()
{
    Wire.beginTransmission(GT911_ADDR1);

    if (Wire.endTransmission() == 0)
        return true;

    Wire.beginTransmission(GT911_ADDR2);

    if (Wire.endTransmission() == 0)
        return true;

    return false;
}

void scanI2C()
{
    Serial.println("[I2C] scanning...");

    int count = 0;

    for (uint8_t addr = 1; addr < 127; addr++)
    {
        Wire.beginTransmission(addr);

        if (Wire.endTransmission() == 0)
        {
            Serial.printf("[I2C] device found at 0x%02X\n", addr);
            count++;
        }
    }

    Serial.printf("[I2C] scan complete: %d device(s)\n", count);
}

void readTouch()
{
    uint8_t status = 0;

    if (!gt911Read(0x814E, &status, 1))
        return;

    if ((status & 0x80) == 0)
        return;

    uint8_t touches = status & 0x0F;

    if (touches > 0 && touches <= 5)
    {
        uint8_t point[8] = {0};

        if (gt911Read(0x8150, point, sizeof(point)))
        {
            uint16_t x = point[0] | (point[1] << 8);
            uint16_t y = point[2] | (point[3] << 8);

            if (x < 480 && y < 480)
            {
                Serial.printf(
                    "[TOUCH] points=%u x=%u y=%u\n",
                    touches,
                    x,
                    y
                );

                gfx->fillCircle(x, y, 6, RED);
            }
        }
    }

    uint8_t clear = 0;

    Wire.beginTransmission(GT911_ADDR1);
    Wire.write(0x81);
    Wire.write(0x4E);
    Wire.write(clear);
    Wire.endTransmission();
}

void drawTestScreen()
{
    gfx->fillScreen(BLACK);

    gfx->setTextColor(GREEN);
    gfx->setTextSize(4);

    gfx->setCursor(55, 60);
    gfx->println("NiceFlightRadar");

    gfx->setTextColor(WHITE);
    gfx->setTextSize(3);

    gfx->setCursor(110, 125);
    gfx->println("V2 DISPLAY");

    gfx->setCursor(130, 170);
    gfx->println("480 x 480");

    gfx->drawCircle(
        240,
        310,
        110,
        GREEN
    );

    gfx->drawCircle(
        240,
        310,
        70,
        GREEN
    );

    gfx->drawLine(
        130,
        310,
        350,
        310,
        GREEN
    );

    gfx->drawLine(
        240,
        200,
        240,
        420,
        GREEN
    );

    gfx->fillCircle(
        240,
        310,
        5,
        YELLOW
    );

    gfx->setTextColor(GREEN);
    gfx->setTextSize(2);

    gfx->setCursor(165, 445);
    gfx->println("DISPLAY V2.1 OK");
}

void setup()
{
    Serial.begin(115200);
    delay(1500);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" NiceFlightRadar V2");
    Serial.println(" Display Probe V2.1");
    Serial.println(" ESP32-4848S040");
    Serial.println("========================================");

    Serial.printf(
        "[MEM] PSRAM total: %.2f MB\n",
        ESP.getPsramSize() / 1024.0 / 1024.0
    );

    Serial.printf(
        "[MEM] PSRAM free : %.2f MB\n",
        ESP.getFreePsram() / 1024.0 / 1024.0
    );

    // Backlight OFF during initialization
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, LOW);

    Serial.println("[TOUCH] Initializing I2C...");
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
    Wire.setClock(400000);

    scanI2C();

    if (gt911Present())
        Serial.println("[TOUCH] GT911 detected");
    else
        Serial.println("[TOUCH] GT911 NOT detected");

    Serial.println("[LCD] Initializing ST7701...");

    if (!gfx->begin())
    {
        Serial.println("[LCD] ERROR: gfx->begin() failed");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("[LCD] ST7701 initialized");

    gfx->displayOn();

    delay(100);

    // Backlight ON
    digitalWrite(TFT_BL, HIGH);

    Serial.println("[LCD] Backlight ON");

    // Color sequence
    Serial.println("[LCD] RED");
    gfx->fillScreen(RED);
    delay(700);

    Serial.println("[LCD] GREEN");
    gfx->fillScreen(GREEN);
    delay(700);

    Serial.println("[LCD] BLUE");
    gfx->fillScreen(BLUE);
    delay(700);

    Serial.println("[LCD] WHITE");
    gfx->fillScreen(WHITE);
    delay(700);

    Serial.println("[LCD] BLACK");
    gfx->fillScreen(BLACK);
    delay(500);

    drawTestScreen();

    Serial.println();
    Serial.println("========================================");
    Serial.println(" Display Probe V2.1 READY");
    Serial.println("========================================");
}

void loop()
{
    readTouch();
    delay(10);
}
