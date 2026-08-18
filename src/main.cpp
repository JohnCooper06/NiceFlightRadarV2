#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <math.h>

// ============================================================
// NiceFlightRadar V2
// Radar Graphics Probe V2.3
// ESP32-4848S040 / ESP32-S3 / ST7701 / GT911
// ============================================================

#define SCREEN_W 480
#define SCREEN_H 480

#define TFT_BL 38

#define TOUCH_SDA 19
#define TOUCH_SCL 45
#define GT911_ADDR 0x5D

// ------------------------------------------------------------
// ST7701
// ------------------------------------------------------------

Arduino_ESP32SPI *panelBus = new Arduino_ESP32SPI(
    GFX_NOT_DEFINED,
    39,
    48,
    47,
    GFX_NOT_DEFINED
);

Arduino_ESP32RGBPanel *rgbPanel = new Arduino_ESP32RGBPanel(
    18, 17, 16, 21,

    11, 12, 13, 14, 0,
    8, 20, 3, 46, 9, 10,
    4, 5, 6, 7, 15,

    1, 10, 8, 50,
    1, 10, 8, 20
);

Arduino_RGB_Display *display = new Arduino_RGB_Display(
    SCREEN_W,
    SCREEN_H,
    rgbPanel,
    0,
    true,
    panelBus,
    GFX_NOT_DEFINED,
    st7701_type9_init_operations,
    sizeof(st7701_type9_init_operations)
);

// Full-screen RGB565 framebuffer.
// Arduino_Canvas allows the whole radar frame to be composed off-screen
// and transferred to the RGB display in one operation.
Arduino_GFX *gfx = new Arduino_Canvas(
    SCREEN_W,
    SCREEN_H,
    display
);

// ------------------------------------------------------------
// Radar geometry
// ------------------------------------------------------------

static const int RADAR_CX = 240;
static const int RADAR_CY = 245;
static const int RADAR_R  = 215;

// 90° clockwise map orientation.
// Geographic north is displayed to the RIGHT.
static const float MAP_ROTATION_DEG = 90.0f;

// ------------------------------------------------------------
// Colors
// ------------------------------------------------------------

uint16_t COL_BG;
uint16_t COL_GRID;
uint16_t COL_GRID_DIM;
uint16_t COL_SWEEP;
uint16_t COL_TEXT;
uint16_t COL_HOME;
uint16_t COL_ARRIVAL;
uint16_t COL_DEPARTURE;

// ------------------------------------------------------------
// Demo aircraft
// ------------------------------------------------------------

struct DemoAircraft {
    float bearing;
    float radius;
    float heading;
    const char *label;
    bool arrival;
};

DemoAircraft aircraft[] = {
    { 335.0f, 0.72f, 150.0f, "AFR41",  true  },
    {  48.0f, 0.50f, 238.0f, "EZY82",  false },
    { 122.0f, 0.82f, 290.0f, "BAW34",  true  },
    { 205.0f, 0.61f,  35.0f, "SWR5K",  false },
    { 268.0f, 0.35f,  95.0f, "DLH7A",  true  }
};

static const int AIRCRAFT_COUNT =
    sizeof(aircraft) / sizeof(aircraft[0]);

// ------------------------------------------------------------
// Touch
// ------------------------------------------------------------

bool gt911Read(uint16_t reg, uint8_t *data, size_t len)
{
    Wire.beginTransmission(GT911_ADDR);
    Wire.write((reg >> 8) & 0xFF);
    Wire.write(reg & 0xFF);

    if (Wire.endTransmission(false) != 0)
        return false;

    if (Wire.requestFrom((uint8_t)GT911_ADDR, len) != len)
        return false;

    for (size_t i = 0; i < len; i++)
        data[i] = Wire.read();

    return true;
}

void gt911ClearStatus()
{
    Wire.beginTransmission(GT911_ADDR);
    Wire.write(0x81);
    Wire.write(0x4E);
    Wire.write((uint8_t)0);
    Wire.endTransmission();
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

            if (x < SCREEN_W && y < SCREEN_H)
            {
                Serial.printf("[TOUCH] x=%u y=%u\n", x, y);
            }
        }
    }

    gt911ClearStatus();
}

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------

float degToRad(float deg)
{
    return deg * PI / 180.0f;
}

// Geographic bearing -> screen angle.
// 0° geographic = North.
// With +90° map rotation, North points screen-right.
float screenAngleForBearing(float bearing)
{
    return bearing - 90.0f + MAP_ROTATION_DEG;
}

void polarToScreen(
    float bearing,
    float radius,
    int &x,
    int &y
)
{
    float angle = degToRad(screenAngleForBearing(bearing));

    x = RADAR_CX + (int)(cosf(angle) * radius);
    y = RADAR_CY + (int)(sinf(angle) * radius);
}

// ------------------------------------------------------------
// Aircraft drawing
// ------------------------------------------------------------

void drawAircraftSymbol(
    int x,
    int y,
    float heading,
    uint16_t color
)
{
    float a = degToRad(screenAngleForBearing(heading));

    const float size = 10.0f;

    int noseX = x + cosf(a) * size;
    int noseY = y + sinf(a) * size;

    int tailX = x - cosf(a) * 6.0f;
    int tailY = y - sinf(a) * 6.0f;

    float wingA = a + PI / 2.0f;

    int wing1X = x + cosf(wingA) * 5.0f;
    int wing1Y = y + sinf(wingA) * 5.0f;

    int wing2X = x - cosf(wingA) * 5.0f;
    int wing2Y = y - sinf(wingA) * 5.0f;

    gfx->drawLine(tailX, tailY, noseX, noseY, color);
    gfx->drawLine(wing1X, wing1Y, wing2X, wing2Y, color);

    gfx->fillCircle(x, y, 2, color);
}

void drawDemoAircraft()
{
    gfx->setTextSize(1);

    for (int i = 0; i < AIRCRAFT_COUNT; i++)
    {
        DemoAircraft &ac = aircraft[i];

        int x;
        int y;

        polarToScreen(
            ac.bearing,
            ac.radius * RADAR_R,
            x,
            y
        );

        uint16_t color =
            ac.arrival ? COL_ARRIVAL : COL_DEPARTURE;

        drawAircraftSymbol(
            x,
            y,
            ac.heading,
            color
        );

        gfx->setTextColor(color);
        gfx->setCursor(x + 8, y - 9);
        gfx->print(ac.label);
    }
}

// ------------------------------------------------------------
// Radar static graphics
// ------------------------------------------------------------

void drawRadarGrid()
{
    // Outer circle
    gfx->drawCircle(
        RADAR_CX,
        RADAR_CY,
        RADAR_R,
        COL_GRID
    );

    // Range rings
    gfx->drawCircle(
        RADAR_CX,
        RADAR_CY,
        RADAR_R * 3 / 4,
        COL_GRID_DIM
    );

    gfx->drawCircle(
        RADAR_CX,
        RADAR_CY,
        RADAR_R / 2,
        COL_GRID_DIM
    );

    gfx->drawCircle(
        RADAR_CX,
        RADAR_CY,
        RADAR_R / 4,
        COL_GRID_DIM
    );

    // Horizontal / vertical
    gfx->drawLine(
        RADAR_CX - RADAR_R,
        RADAR_CY,
        RADAR_CX + RADAR_R,
        RADAR_CY,
        COL_GRID_DIM
    );

    gfx->drawLine(
        RADAR_CX,
        RADAR_CY - RADAR_R,
        RADAR_CX,
        RADAR_CY + RADAR_R,
        COL_GRID_DIM
    );

    // Diagonals
    const float d = RADAR_R * 0.7071f;

    gfx->drawLine(
        RADAR_CX - d,
        RADAR_CY - d,
        RADAR_CX + d,
        RADAR_CY + d,
        COL_GRID_DIM
    );

    gfx->drawLine(
        RADAR_CX + d,
        RADAR_CY - d,
        RADAR_CX - d,
        RADAR_CY + d,
        COL_GRID_DIM
    );
}

void drawCompass()
{
    gfx->setTextSize(2);
    gfx->setTextColor(COL_TEXT);

    // V1 orientation: north = right side
    gfx->setCursor(452, RADAR_CY - 8);
    gfx->print("N");
}

void drawHeader()
{
    gfx->setTextSize(1);
    gfx->setTextColor(COL_TEXT);

    gfx->setCursor(10, 8);
    gfx->print("NICE FLIGHT RADAR V2");

    gfx->setCursor(390, 8);
    gfx->print("25 NM");
}

void drawHomeAndAirport()
{
    // NCE / radar center
    gfx->fillCircle(
        RADAR_CX,
        RADAR_CY,
        4,
        COL_HOME
    );

    gfx->setTextSize(1);
    gfx->setTextColor(COL_HOME);
    gfx->setCursor(
        RADAR_CX + 7,
        RADAR_CY + 5
    );
    gfx->print("NCE");

    // Temporary HOME demonstration marker.
    // Real geographic calculation comes later.
    int hx;
    int hy;

    polarToScreen(
        47.4f,
        RADAR_R * 0.20f,
        hx,
        hy
    );

    gfx->fillCircle(
        hx,
        hy,
        3,
        COL_TEXT
    );

    gfx->setTextColor(COL_TEXT);
    gfx->setCursor(hx + 5, hy - 4);
    gfx->print("HOME");
}

void drawFooter()
{
    gfx->setTextSize(1);
    gfx->setTextColor(COL_TEXT);

    gfx->setCursor(10, 464);
    gfx->print("RADAR V2.3");

    gfx->setCursor(370, 464);
    gfx->print("5 AC");
}

// ------------------------------------------------------------
// Sweep
// ------------------------------------------------------------

float sweepBearing = 0.0f;

void drawSweep()
{
    float angle =
        degToRad(
            screenAngleForBearing(sweepBearing)
        );

    int x =
        RADAR_CX +
        cosf(angle) * RADAR_R;

    int y =
        RADAR_CY +
        sinf(angle) * RADAR_R;

    gfx->drawLine(
        RADAR_CX,
        RADAR_CY,
        x,
        y,
        COL_SWEEP
    );

    // Cursor on outer radar circumference
    gfx->fillCircle(
        x,
        y,
        4,
        COL_SWEEP
    );
}

// ------------------------------------------------------------
// Full frame
// ------------------------------------------------------------

void drawFrame()
{
    gfx->fillScreen(COL_BG);

    drawRadarGrid();
    drawCompass();
    drawHeader();
    drawHomeAndAirport();
    drawDemoAircraft();
    drawSweep();
    drawFooter();

    // Present the completed framebuffer.
    gfx->flush();
}

// ------------------------------------------------------------
// FPS / memory
// ------------------------------------------------------------

uint32_t fpsTimer = 0;
uint32_t frames = 0;

void printStats()
{
    frames++;

    uint32_t now = millis();

    if (now - fpsTimer >= 1000)
    {
        float fps =
            frames * 1000.0f /
            (now - fpsTimer);

        Serial.printf(
            "[PERF] FPS %.1f | heap %.1f KB | psram %.2f MB\n",
            fps,
            ESP.getFreeHeap() / 1024.0f,
            ESP.getFreePsram() / 1024.0f / 1024.0f
        );

        frames = 0;
        fpsTimer = now;
    }
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(800);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" NiceFlightRadar V2");
    Serial.println(" Radar Graphics Probe V2.3");
    Serial.println(" ESP32-4848S040");
    Serial.println("========================================");

    Serial.printf(
        "[MEM] PSRAM %.2f MB\n",
        ESP.getPsramSize() / 1024.0f / 1024.0f
    );

    // Touch
    Wire.begin(
        TOUCH_SDA,
        TOUCH_SCL
    );

    Wire.setClock(400000);

    // Backlight OFF during initialization
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, LOW);

    Serial.println("[LCD] initializing...");

    if (!gfx->begin())
    {
        Serial.println("[LCD] INIT FAILED");
        while (true)
            delay(1000);
    }

    Serial.println("[LCD] ST7701 + Canvas OK");

    Serial.printf(
        "[FRAMEBUFFER] 480x480 RGB565 = %.1f KB\n",
        (SCREEN_W * SCREEN_H * 2) / 1024.0f
    );

    // Palette
    COL_BG        = gfx->color565(0,   6,   2);
    COL_GRID      = gfx->color565(40,  220, 80);
    COL_GRID_DIM  = gfx->color565(12,  75,  30);
    COL_SWEEP     = gfx->color565(90,  255, 120);
    COL_TEXT      = gfx->color565(220, 255, 220);
    COL_HOME      = gfx->color565(255, 210, 30);
    COL_ARRIVAL   = gfx->color565(255, 80,  60);
    COL_DEPARTURE = gfx->color565(50,  150, 255);

    gfx->fillScreen(COL_BG);

    digitalWrite(TFT_BL, HIGH);

    Serial.println("[LCD] backlight ON");
    Serial.println("[RADAR] V2.3 READY");

    fpsTimer = millis();
}

// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------

void loop()
{
    readTouch();

    drawFrame();

    sweepBearing += 2.5f;

    if (sweepBearing >= 360.0f)
        sweepBearing -= 360.0f;

    printStats();

    delay(15);
}
