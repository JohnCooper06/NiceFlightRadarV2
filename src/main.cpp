#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Arduino_GFX_Library.h>
#include <math.h>

#include "secrets.h"

// ============================================================
// NiceFlightRadar V2.5
// LIVE ADS-B
//
// Hardware:
//   ESP32-4848S040
//   ESP32-S3 N16R8
//   ST7701 480x480
//   GT911
//
// Graphics:
//   Full-screen RGB565 framebuffer
//
// Network:
//   airplanes.live
//   asynchronous FreeRTOS task
// ============================================================

#define SCREEN_W 480
#define SCREEN_H 480

#define TFT_BL 38

#define TOUCH_SDA 19
#define TOUCH_SCL 45
#define GT911_ADDR 0x5D

// ============================================================
// DISPLAY HARDWARE
// ============================================================

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

Arduino_GFX *gfx = new Arduino_Canvas(
    SCREEN_W,
    SCREEN_H,
    display
);

// ============================================================
// RADAR GEOMETRY
// ============================================================

static const int RADAR_CX = 240;
static const int RADAR_CY = 245;
static const int RADAR_R  = 215;

// Keep the orientation currently validated on V2.
static const float MAP_ROTATION_DEG = 90.0f;

// Nice Côte d'Azur / LFMN.
static const double RADAR_LAT = 43.6584;
static const double RADAR_LON = 7.2159;

// Display range.
static const float RADAR_RANGE_NM = 25.0f;

// Network acquisition radius.
// Slightly larger than the visible radar so aircraft approaching
// the edge are already present in the snapshot.
static const int API_RADIUS_NM = 40;

static const double EARTH_RADIUS_NM = 3440.065;

// ============================================================
// COLORS
// ============================================================

uint16_t COL_BG;
uint16_t COL_GRID;
uint16_t COL_GRID_DIM;
uint16_t COL_SWEEP;
uint16_t COL_TEXT;
uint16_t COL_HOME;
uint16_t COL_AIRCRAFT;
uint16_t COL_GROUND;
uint16_t COL_WARNING;

// ============================================================
// AIRCRAFT
// ============================================================

struct Aircraft
{
    char hex[9] = {0};
    char callsign[12] = {0};

    double lat = 0.0;
    double lon = 0.0;

    int altitude = 0;
    bool onGround = false;

    int verticalRate = 0;

    float speed = 0.0f;
    float heading = 0.0f;

    float distanceNm = 0.0f;
    float bearing = 0.0f;

    bool valid = false;
};

static const int MAX_AIRCRAFT = 50;

Aircraft aircraft[MAX_AIRCRAFT];
int aircraftCount = 0;

Aircraft pendingAircraft[MAX_AIRCRAFT];
int pendingAircraftCount = 0;

SemaphoreHandle_t aircraftMutex = nullptr;

volatile bool pendingAircraftReady = false;

TaskHandle_t aircraftNetworkTaskHandle = nullptr;

volatile bool networkRequestActive = false;

unsigned long lastAircraftUpdateMs = 0;

// ============================================================
// GEOGRAPHY
// ============================================================

double degToRad(double deg)
{
    return deg * PI / 180.0;
}

double radToDeg(double rad)
{
    return rad * 180.0 / PI;
}

float geographicDistanceNm(
    double lat1,
    double lon1,
    double lat2,
    double lon2
)
{
    double dLat =
        degToRad(lat2 - lat1);

    double dLon =
        degToRad(lon2 - lon1);

    double p1 =
        degToRad(lat1);

    double p2 =
        degToRad(lat2);

    double a =
        sin(dLat / 2.0) *
        sin(dLat / 2.0) +
        sin(dLon / 2.0) *
        sin(dLon / 2.0) *
        cos(p1) *
        cos(p2);

    double c =
        2.0 *
        atan2(
            sqrt(a),
            sqrt(1.0 - a)
        );

    return
        EARTH_RADIUS_NM * c;
}

float geographicBearing(
    double lat1,
    double lon1,
    double lat2,
    double lon2
)
{
    double p1 =
        degToRad(lat1);

    double p2 =
        degToRad(lat2);

    double dLon =
        degToRad(lon2 - lon1);

    double y =
        sin(dLon) *
        cos(p2);

    double x =
        cos(p1) *
        sin(p2) -
        sin(p1) *
        cos(p2) *
        cos(dLon);

    double bearing =
        radToDeg(
            atan2(y, x)
        );

    if (bearing < 0.0)
        bearing += 360.0;

    return bearing;
}

// Geographic bearing -> display angle.
//
// 0° bearing = geographic North.
// Rotation is then applied to the complete radar map.
float screenAngleForBearing(float bearing)
{
    return
        bearing -
        90.0f +
        MAP_ROTATION_DEG;
}

void polarToScreen(
    float bearing,
    float radius,
    int &x,
    int &y
)
{
    float angle =
        degToRad(
            screenAngleForBearing(
                bearing
            )
        );

    x =
        RADAR_CX +
        roundf(
            cosf(angle) *
            radius
        );

    y =
        RADAR_CY +
        roundf(
            sinf(angle) *
            radius
        );
}

bool aircraftToScreen(
    const Aircraft &ac,
    int &x,
    int &y
)
{
    if (!ac.valid)
        return false;

    if (ac.distanceNm > RADAR_RANGE_NM)
        return false;

    float radius =
        (ac.distanceNm / RADAR_RANGE_NM) *
        RADAR_R;

    polarToScreen(
        ac.bearing,
        radius,
        x,
        y
    );

    return true;
}

// ============================================================
// TOUCH GT911
// ============================================================

bool gt911Read(
    uint16_t reg,
    uint8_t *data,
    size_t len
)
{
    Wire.beginTransmission(
        GT911_ADDR
    );

    Wire.write(
        (reg >> 8) & 0xFF
    );

    Wire.write(
        reg & 0xFF
    );

    if (
        Wire.endTransmission(false)
        != 0
    ) {
        return false;
    }

    if (
        Wire.requestFrom(
            (uint8_t)GT911_ADDR,
            len
        ) != len
    ) {
        return false;
    }

    for (
        size_t i = 0;
        i < len;
        i++
    ) {
        data[i] =
            Wire.read();
    }

    return true;
}

void gt911ClearStatus()
{
    Wire.beginTransmission(
        GT911_ADDR
    );

    Wire.write(0x81);
    Wire.write(0x4E);
    Wire.write((uint8_t)0);

    Wire.endTransmission();
}

void readTouch()
{
    uint8_t status = 0;

    if (
        !gt911Read(
            0x814E,
            &status,
            1
        )
    ) {
        return;
    }

    if (
        (status & 0x80) == 0
    ) {
        return;
    }

    uint8_t touches =
        status & 0x0F;

    if (
        touches > 0 &&
        touches <= 5
    ) {
        uint8_t point[8] = {0};

        if (
            gt911Read(
                0x8150,
                point,
                sizeof(point)
            )
        ) {
            uint16_t x =
                point[0] |
                (point[1] << 8);

            uint16_t y =
                point[2] |
                (point[3] << 8);

            if (
                x < SCREEN_W &&
                y < SCREEN_H
            ) {
                static uint32_t lastTouchLog = 0;

                if (
                    millis() -
                    lastTouchLog >
                    100
                ) {
                    lastTouchLog =
                        millis();

                    Serial.printf(
                        "[TOUCH] x=%u y=%u\n",
                        x,
                        y
                    );
                }
            }
        }
    }

    gt911ClearStatus();
}

// ============================================================
// NETWORK
// ============================================================

void safeCopy(
    char *destination,
    size_t destinationSize,
    const char *source
)
{
    if (
        destinationSize == 0
    ) {
        return;
    }

    if (
        source == nullptr
    ) {
        destination[0] = '\0';
        return;
    }

    strncpy(
        destination,
        source,
        destinationSize - 1
    );

    destination[
        destinationSize - 1
    ] = '\0';
}

bool fetchAircraftNetwork()
{
    if (
        WiFi.status() !=
        WL_CONNECTED
    ) {
        Serial.println(
            "[NET] WiFi disconnected"
        );

        return false;
    }

    networkRequestActive =
        true;

    uint32_t started =
        millis();

    HTTPClient http;

    http.setTimeout(
        8000
    );

    http.setUserAgent(
        "NiceFlightRadarV2/2.5"
    );

    String url =
        "https://opendata.adsb.fi/api/v3/lat/" +
        String(RADAR_LAT, 4) +
        "/lon/" +
        String(RADAR_LON, 4) +
        "/dist/" +
        String(API_RADIUS_NM);

    Serial.println();
    Serial.println(
        "[NET] GET"
    );

    Serial.println(
        url
    );

    if (
        !http.begin(url)
    ) {
        Serial.println(
            "[NET] http.begin failed"
        );

        networkRequestActive =
            false;

        return false;
    }

    int httpCode =
        http.GET();

    uint32_t requestDuration =
        millis() -
        started;

    if (
        httpCode !=
        HTTP_CODE_OK
    ) {
        Serial.printf(
            "[NET] HTTP %d / %lu ms\n",
            httpCode,
            requestDuration
        );

        http.end();

        networkRequestActive =
            false;

        return false;
    }

    String payload =
        http.getString();

    http.end();

    Serial.printf(
        "[NET] payload %u bytes / %lu ms\n",
        (unsigned)payload.length(),
        requestDuration
    );

    // --------------------------------------------------------
    // JSON filter from the stable V1 implementation.
    // --------------------------------------------------------

    JsonDocument filter;

    JsonObject aircraftFilter =
        filter["ac"].add<JsonObject>();

    aircraftFilter["hex"] =
        true;

    aircraftFilter["flight"] =
        true;

    aircraftFilter["lat"] =
        true;

    aircraftFilter["lon"] =
        true;

    aircraftFilter["alt_baro"] =
        true;

    aircraftFilter["baro_rate"] =
        true;

    aircraftFilter["gs"] =
        true;

    aircraftFilter["track"] =
        true;

    JsonDocument doc;

    DeserializationError error =
        deserializeJson(
            doc,
            payload,
            DeserializationOption::Filter(
                filter
            )
        );

    if (error)
    {
        Serial.print(
            "[NET] JSON error: "
        );

        Serial.println(
            error.c_str()
        );

        networkRequestActive =
            false;

        return false;
    }

    JsonArray planes =
        doc["ac"].as<JsonArray>();

    Aircraft temp[
        MAX_AIRCRAFT
    ];

    int tempCount =
        0;

    for (
        JsonObject plane :
        planes
    ) {
        if (
            tempCount >=
            MAX_AIRCRAFT
        ) {
            break;
        }

        if (
            !plane["lat"].is<double>() ||
            !plane["lon"].is<double>()
        ) {
            continue;
        }

        Aircraft &ac =
            temp[tempCount];

        ac =
            Aircraft();

        safeCopy(
            ac.hex,
            sizeof(ac.hex),
            plane["hex"] | ""
        );

        safeCopy(
            ac.callsign,
            sizeof(ac.callsign),
            plane["flight"] | ""
        );

        // Remove trailing spaces often present in ADS-B flight.
        for (
            int i =
                strlen(ac.callsign) - 1;
            i >= 0;
            i--
        ) {
            if (
                ac.callsign[i] == ' '
            ) {
                ac.callsign[i] =
                    '\0';
            }
            else {
                break;
            }
        }

        ac.lat =
            plane["lat"];

        ac.lon =
            plane["lon"];

        ac.onGround =
            false;

        if (
            plane["alt_baro"].is<int>()
        ) {
            ac.altitude =
                plane["alt_baro"];
        }
        else if (
            plane["alt_baro"]
                .is<const char*>()
        ) {
            const char *alt =
                plane["alt_baro"]
                    .as<const char*>();

            if (
                alt != nullptr &&
                strcmp(
                    alt,
                    "ground"
                ) == 0
            ) {
                ac.onGround =
                    true;

                ac.altitude =
                    0;
            }
            else {
                ac.altitude =
                    alt
                    ? atoi(alt)
                    : 0;
            }
        }

        ac.verticalRate =
            plane["baro_rate"] |
            0;

        ac.speed =
            plane["gs"] |
            0.0f;

        ac.heading =
            plane["track"] |
            0.0f;

        ac.distanceNm =
            geographicDistanceNm(
                RADAR_LAT,
                RADAR_LON,
                ac.lat,
                ac.lon
            );

        ac.bearing =
            geographicBearing(
                RADAR_LAT,
                RADAR_LON,
                ac.lat,
                ac.lon
            );

        ac.valid =
            true;

        tempCount++;
    }

    if (
        aircraftMutex == nullptr
    ) {
        networkRequestActive =
            false;

        return false;
    }

    if (
        xSemaphoreTake(
            aircraftMutex,
            portMAX_DELAY
        ) == pdTRUE
    ) {
        pendingAircraftCount =
            tempCount;

        for (
            int i = 0;
            i < tempCount;
            i++
        ) {
            pendingAircraft[i] =
                temp[i];
        }

        pendingAircraftReady =
            true;

        xSemaphoreGive(
            aircraftMutex
        );
    }

    Serial.printf(
        "[NET] %d aircraft ready\n",
        tempCount
    );

    networkRequestActive =
        false;

    return true;
}

void aircraftNetworkTask(
    void *parameter
)
{
    Serial.println(
        "[NET] aircraft task started"
    );

    for (;;)
    {
        fetchAircraftNetwork();

        vTaskDelay(
            pdMS_TO_TICKS(
                5000
            )
        );
    }
}

void startAircraftNetworkTask()
{
    if (
        aircraftNetworkTaskHandle !=
        nullptr
    ) {
        return;
    }

    if (
        aircraftMutex ==
        nullptr
    ) {
        aircraftMutex =
            xSemaphoreCreateMutex();

        if (
            aircraftMutex ==
            nullptr
        ) {
            Serial.println(
                "[NET] mutex creation failed"
            );

            return;
        }
    }

    BaseType_t result =
        xTaskCreatePinnedToCore(
            aircraftNetworkTask,
            "adsb-net",
            16384,
            nullptr,
            1,
            &aircraftNetworkTaskHandle,
            0
        );

    if (
        result !=
        pdPASS
    ) {
        aircraftNetworkTaskHandle =
            nullptr;

        Serial.println(
            "[NET] task creation failed"
        );

        return;
    }
}

bool applyPendingAircraft()
{
    if (
        !pendingAircraftReady ||
        aircraftMutex ==
        nullptr
    ) {
        return false;
    }

    if (
        xSemaphoreTake(
            aircraftMutex,
            0
        ) != pdTRUE
    ) {
        return false;
    }

    if (
        pendingAircraftReady
    ) {
        aircraftCount =
            pendingAircraftCount;

        for (
            int i = 0;
            i < aircraftCount;
            i++
        ) {
            aircraft[i] =
                pendingAircraft[i];
        }

        pendingAircraftReady =
            false;

        lastAircraftUpdateMs =
            millis();

        Serial.printf(
            "[RADAR] snapshot applied: %d aircraft\n",
            aircraftCount
        );
    }

    xSemaphoreGive(
        aircraftMutex
    );

    return true;
}

// ============================================================
// WIFI
// ============================================================

void connectWiFi()
{
    Serial.println();
    Serial.println(
        "[WIFI] connecting..."
    );

    WiFi.mode(
        WIFI_STA
    );

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );

    uint32_t started =
        millis();

    while (
        WiFi.status() !=
        WL_CONNECTED
    ) {
        delay(250);

        Serial.print(".");

        if (
            millis() -
            started >
            20000
        ) {
            Serial.println();
            Serial.println(
                "[WIFI] connection timeout"
            );

            return;
        }
    }

    Serial.println();

    Serial.print(
        "[WIFI] connected: "
    );

    Serial.println(
        WiFi.localIP()
    );

    Serial.printf(
        "[WIFI] RSSI: %d dBm\n",
        WiFi.RSSI()
    );
}

// ============================================================
// AIRCRAFT DRAWING
// ============================================================

void drawAircraftSymbol(
    int x,
    int y,
    float heading,
    uint16_t color
)
{
    float angle =
        degToRad(
            screenAngleForBearing(
                heading
            )
        );

    // Compact radar plot.
    gfx->fillCircle(
        x,
        y,
        3,
        color
    );

    int hx =
        x +
        roundf(
            cosf(angle) *
            10.0f
        );

    int hy =
        y +
        roundf(
            sinf(angle) *
            10.0f
        );

    gfx->drawLine(
        x,
        y,
        hx,
        hy,
        color
    );
}

void drawAircraft()
{
    gfx->setTextSize(1);

    int visibleCount =
        0;

    for (
        int i = 0;
        i < aircraftCount;
        i++
    ) {
        const Aircraft &ac =
            aircraft[i];

        if (
            !ac.valid ||
            ac.onGround
        ) {
            continue;
        }

        int x;
        int y;

        if (
            !aircraftToScreen(
                ac,
                x,
                y
            )
        ) {
            continue;
        }

        visibleCount++;

        drawAircraftSymbol(
            x,
            y,
            ac.heading,
            COL_AIRCRAFT
        );

        String label;

        if (
            strlen(
                ac.callsign
            ) > 0
        ) {
            label =
                ac.callsign;
        }
        else {
            label =
                ac.hex;
        }

        if (
            label.length() >
            9
        ) {
            label =
                label.substring(
                    0,
                    9
                );
        }

        int labelX =
            x + 8;

        int labelY =
            y - 12;

        // Prevent labels leaving right edge.
        if (
            x >
            SCREEN_W - 90
        ) {
            labelX =
                x - 55;
        }

        // Prevent labels leaving top edge.
        if (
            labelY < 28
        ) {
            labelY =
                y + 8;
        }

        gfx->setTextColor(
            COL_TEXT
        );

        gfx->setCursor(
            labelX,
            labelY
        );

        gfx->print(
            label
        );

        if (
            ac.altitude > 0
        ) {
            gfx->setTextColor(
                COL_GRID
            );

            gfx->setCursor(
                labelX,
                labelY + 10
            );

            gfx->printf(
                "%dFT",
                ac.altitude
            );
        }
    }
}

// ============================================================
// STATIC RADAR
// ============================================================

void drawRadarGrid()
{
    gfx->drawCircle(
        RADAR_CX,
        RADAR_CY,
        RADAR_R,
        COL_GRID
    );

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

    const float d =
        RADAR_R * 0.7071f;

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
    int nx;
    int ny;

    polarToScreen(
        0.0f,
        RADAR_R - 12,
        nx,
        ny
    );

    gfx->setTextColor(
        COL_TEXT
    );

    gfx->setTextSize(2);

    gfx->setCursor(
        nx - 6,
        ny - 8
    );

    gfx->print("N");
}

void drawHeader()
{
    gfx->setTextSize(1);

    gfx->setTextColor(
        COL_TEXT
    );

    gfx->setCursor(
        10,
        8
    );

    gfx->print(
        "NICE FLIGHT RADAR V2"
    );

    gfx->setCursor(
        390,
        8
    );

    gfx->printf(
        "%.0f NM",
        RADAR_RANGE_NM
    );
}

void drawAirport()
{
    gfx->fillCircle(
        RADAR_CX,
        RADAR_CY,
        4,
        COL_HOME
    );

    gfx->setTextColor(
        COL_HOME
    );

    gfx->setTextSize(1);

    gfx->setCursor(
        RADAR_CX + 7,
        RADAR_CY + 5
    );

    gfx->print(
        "NCE"
    );
}

void drawFooter()
{
    gfx->setTextSize(1);

    gfx->setTextColor(
        COL_TEXT
    );

    gfx->setCursor(
        10,
        464
    );

    if (
        WiFi.status() ==
        WL_CONNECTED
    ) {
        gfx->print(
            "LIVE ADS-B"
        );
    }
    else {
        gfx->setTextColor(
            COL_WARNING
        );

        gfx->print(
            "WIFI OFF"
        );
    }

    int visible =
        0;

    for (
        int i = 0;
        i < aircraftCount;
        i++
    ) {
        if (
            aircraft[i].valid &&
            !aircraft[i].onGround &&
            aircraft[i].distanceNm <=
                RADAR_RANGE_NM
        ) {
            visible++;
        }
    }

    gfx->setTextColor(
        COL_TEXT
    );

    gfx->setCursor(
        378,
        464
    );

    gfx->printf(
        "%d AC",
        visible
    );
}

// ============================================================
// SWEEP
// ============================================================

float sweepBearing =
    0.0f;

void drawSweep()
{
    // Short fading trail.
    static const int TRAIL_COUNT =
        5;

    for (
        int trail =
            TRAIL_COUNT - 1;
        trail >= 0;
        trail--
    ) {
        float bearing =
            sweepBearing -
            trail * 2.0f;

        while (
            bearing < 0.0f
        ) {
            bearing +=
                360.0f;
        }

        int x;
        int y;

        polarToScreen(
            bearing,
            RADAR_R,
            x,
            y
        );

        uint16_t color =
            trail == 0
            ? COL_SWEEP
            : COL_GRID_DIM;

        gfx->drawLine(
            RADAR_CX,
            RADAR_CY,
            x,
            y,
            color
        );

        if (
            trail == 0
        ) {
            gfx->fillCircle(
                x,
                y,
                4,
                COL_SWEEP
            );
        }
    }
}

// ============================================================
// FRAME
// ============================================================

void drawFrame()
{
    gfx->fillScreen(
        COL_BG
    );

    drawRadarGrid();
    drawCompass();
    drawHeader();
    drawAirport();

    drawAircraft();

    drawSweep();
    drawFooter();

    gfx->flush();
}

// ============================================================
// PERFORMANCE
// ============================================================

uint32_t fpsTimer =
    0;

uint32_t frames =
    0;

void printStats()
{
    frames++;

    uint32_t now =
        millis();

    if (
        now -
        fpsTimer >=
        1000
    ) {
        float fps =
            frames *
            1000.0f /
            (
                now -
                fpsTimer
            );

        Serial.printf(
            "[PERF] FPS %.1f | heap %.1f KB | psram %.2f MB | AC %d | WiFi %d\n",
            fps,
            ESP.getFreeHeap() /
                1024.0f,
            ESP.getFreePsram() /
                1024.0f /
                1024.0f,
            aircraftCount,
            WiFi.RSSI()
        );

        frames =
            0;

        fpsTimer =
            now;
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        115200
    );

    delay(
        800
    );

    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        " NiceFlightRadar V2.5"
    );

    Serial.println(
        " LIVE ADS-B"
    );

    Serial.println(
        " ESP32-4848S040"
    );

    Serial.println(
        "========================================"
    );

    Serial.printf(
        "[MEM] PSRAM %.2f MB\n",
        ESP.getPsramSize() /
            1024.0f /
            1024.0f
    );

    Wire.begin(
        TOUCH_SDA,
        TOUCH_SCL
    );

    Wire.setClock(
        400000
    );

    pinMode(
        TFT_BL,
        OUTPUT
    );

    digitalWrite(
        TFT_BL,
        LOW
    );

    Serial.println(
        "[LCD] initializing..."
    );

    if (
        !gfx->begin()
    ) {
        Serial.println(
            "[LCD] INIT FAILED"
        );

        while (true) {
            delay(
                1000
            );
        }
    }

    Serial.println(
        "[LCD] ST7701 + Canvas OK"
    );

    Serial.printf(
        "[FRAMEBUFFER] %.1f KB\n",
        (
            SCREEN_W *
            SCREEN_H *
            2
        ) /
        1024.0f
    );

    COL_BG =
        gfx->color565(
            0,
            6,
            2
        );

    COL_GRID =
        gfx->color565(
            40,
            220,
            80
        );

    COL_GRID_DIM =
        gfx->color565(
            12,
            75,
            30
        );

    COL_SWEEP =
        gfx->color565(
            90,
            255,
            120
        );

    COL_TEXT =
        gfx->color565(
            220,
            255,
            220
        );

    COL_HOME =
        gfx->color565(
            255,
            210,
            30
        );

    COL_AIRCRAFT =
        gfx->color565(
            80,
            210,
            255
        );

    COL_GROUND =
        gfx->color565(
            255,
            200,
            40
        );

    COL_WARNING =
        gfx->color565(
            255,
            100,
            70
        );

    gfx->fillScreen(
        COL_BG
    );

    gfx->flush();

    digitalWrite(
        TFT_BL,
        HIGH
    );

    Serial.println(
        "[LCD] backlight ON"
    );

    connectWiFi();

    startAircraftNetworkTask();

    Serial.printf(
        "[RADAR] center %.4f %.4f\n",
        RADAR_LAT,
        RADAR_LON
    );

    Serial.printf(
        "[RADAR] display %.1f NM / API %d NM\n",
        RADAR_RANGE_NM,
        API_RADIUS_NM
    );

    Serial.println(
        "[RADAR] V2.5 LIVE READY"
    );

    fpsTimer =
        millis();
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    readTouch();

    applyPendingAircraft();

    drawFrame();

    sweepBearing +=
        2.5f;

    if (
        sweepBearing >=
        360.0f
    ) {
        sweepBearing -=
            360.0f;
    }

    printStats();

    delay(
        15
    );
}
