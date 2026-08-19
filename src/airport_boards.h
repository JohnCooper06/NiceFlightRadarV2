#pragma once

// ============================================================
// NiceFlightRadar V2
// NCE OFFICIAL ARRIVALS / DEPARTURES BOARDS
// Ported from validated V1 parser
// ============================================================

struct AirportFlight
{
    String time;
    String origin;
    String destination;
    String flight;
    String status;
    String terminal;
};

static constexpr int AIRPORT_BOARD_MAX = 20;
static constexpr int AIRPORT_BOARD_VISIBLE = 7;
static constexpr unsigned long AIRPORT_BOARD_FETCH_INTERVAL = 300000UL;

AirportFlight airportArrivals[AIRPORT_BOARD_MAX];
AirportFlight airportDepartures[AIRPORT_BOARD_MAX];

volatile int airportArrivalCount = 0;
volatile int airportDepartureCount = 0;

int airportArrivalPage = 0;
int airportDeparturePage = 0;

SemaphoreHandle_t airportArrivalsMutex = nullptr;
SemaphoreHandle_t airportDeparturesMutex = nullptr;

unsigned long lastAirportBoardsFetch = 0;


// ============================================================
// HTML
// ============================================================

String airportStripHtml(String value)
{
    String result;
    bool insideTag = false;

    for (unsigned int i = 0; i < value.length(); i++)
    {
        if ((i & 0xFF) == 0)
            taskYIELD();

        char c = value[i];

        if (c == '<')
        {
            insideTag = true;
            continue;
        }

        if (c == '>')
        {
            insideTag = false;
            continue;
        }

        if (!insideTag)
            result += c;
    }

    result.replace("&nbsp;", " ");
    result.replace("&amp;", "&");
    result.trim();

    while (result.indexOf("  ") >= 0)
        result.replace("  ", " ");

    return result;
}


bool airportLooksLikeTime(const String &s)
{
    return
        s.length() == 5 &&
        isDigit(s[0]) &&
        isDigit(s[1]) &&
        s[2] == ':' &&
        isDigit(s[3]) &&
        isDigit(s[4]);
}


String airportCompactCode(String value)
{
    value.toUpperCase();
    value.trim();

    if (value.indexOf("PARIS CDG") >= 0) return "CDG";
    if (value.indexOf("PARIS ORLY") >= 0) return "ORY";
    if (value.indexOf("LONDON HEATHROW") >= 0) return "LHR";
    if (value.indexOf("LONDON GATWICK") >= 0) return "LGW";
    if (value.indexOf("AMSTERDAM") >= 0) return "AMS";
    if (value.indexOf("FRANKFURT") >= 0) return "FRA";
    if (value.indexOf("FRANCFORT") >= 0) return "FRA";
    if (value.indexOf("ZURICH") >= 0) return "ZRH";
    if (value.indexOf("ROME") >= 0) return "FCO";
    if (value.indexOf("DUBLIN") >= 0) return "DUB";
    if (value.indexOf("DOHA") >= 0) return "DOH";
    if (value.indexOf("TIRANA") >= 0) return "TIA";
    if (value.indexOf("REYKJAVIK") >= 0) return "KEF";
    if (value.indexOf("DUBAI") >= 0) return "DXB";
    if (value.indexOf("MADRID") >= 0) return "MAD";
    if (value.indexOf("BARCELONA") >= 0) return "BCN";
    if (value.indexOf("LISBON") >= 0) return "LIS";
    if (value.indexOf("MUNICH") >= 0) return "MUC";
    if (value.indexOf("GENEVA") >= 0) return "GVA";
    if (value.indexOf("BRUSSELS") >= 0) return "BRU";
    if (value.indexOf("BRUXELLES") >= 0) return "BRU";
    if (value.indexOf("ATHENS") >= 0) return "ATH";
    if (value.indexOf("ATHENES") >= 0) return "ATH";

    if (value.length() > 8)
        value = value.substring(0, 8);

    return value;
}


String airportTerminal(String text)
{
    text.trim();
    text.toUpperCase();

    if (
        text.indexOf("TERMINAL 1") >= 0 ||
        text == "T1" ||
        text == "T 1" ||
        text == "1"
    )
        return "T1";

    if (
        text.indexOf("TERMINAL 2") >= 0 ||
        text == "T2" ||
        text == "T 2" ||
        text == "2"
    )
        return "T2";

    return "";
}


// ============================================================
// TIME
// ============================================================

int airportClockMinutes(const String &value)
{
    for (unsigned int i = 0; i + 4 < value.length(); i++)
    {
        if (
            isDigit(value[i]) &&
            isDigit(value[i + 1]) &&
            value[i + 2] == ':' &&
            isDigit(value[i + 3]) &&
            isDigit(value[i + 4])
        )
        {
            int hour =
                (value[i] - '0') * 10 +
                (value[i + 1] - '0');

            int minute =
                (value[i + 3] - '0') * 10 +
                (value[i + 4] - '0');

            if (
                hour >= 0 && hour <= 23 &&
                minute >= 0 && minute <= 59
            )
                return hour * 60 + minute;
        }
    }

    return -1;
}


int airportCurrentMinutes()
{
    time_t now = time(nullptr);

    if (now < 100000)
        return -1;

    struct tm localTime;

    if (localtime_r(&now, &localTime) == nullptr)
        return -1;

    return
        localTime.tm_hour * 60 +
        localTime.tm_min;
}


// ============================================================
// ARRIVAL DISPLAY
// ============================================================

String airportArrivalDisplayTime(const AirportFlight &flight)
{
    String upper = flight.status;
    upper.toUpperCase();

    if (
        upper.startsWith("EXPECTED") ||
        upper.startsWith("DELAYED") ||
        upper.startsWith("ARRIVED") ||
        upper.startsWith("LANDED")
    )
    {
        int minutes = airportClockMinutes(flight.status);

        if (minutes >= 0)
        {
            char buffer[6];

            snprintf(
                buffer,
                sizeof(buffer),
                "%02d:%02d",
                minutes / 60,
                minutes % 60
            );

            return String(buffer);
        }
    }

    return flight.time;
}


String airportArrivalStatus(const AirportFlight &flight)
{
    String status = flight.status;
    String upper = status;
    upper.toUpperCase();

    if (upper.startsWith("EXPECTED")) return "EXPECTED";
    if (upper.startsWith("ARRIVED")) return "ARRIVED";
    if (upper.startsWith("LANDED")) return "LANDED";
    if (upper.startsWith("LANDING")) return "LANDING";
    if (upper.startsWith("DELAYED")) return "DELAYED";

    if (
        upper.startsWith("CANCELLED") ||
        upper.startsWith("CANCELED")
    )
        return "CANCELLED";

    if (upper == "SCHEDULED")
        return "ON TIME";

    if (status.length() > 14)
        status = status.substring(0, 14);

    return status;
}


int airportArrivalSortMinutes(const AirportFlight &flight)
{
    int minutes =
        airportClockMinutes(
            airportArrivalDisplayTime(flight)
        );

    return minutes >= 0
        ? minutes
        : 24 * 60;
}


// ============================================================
// DEPARTURE DISPLAY
// ============================================================

bool airportIsDepartureStatus(String text)
{
    text.trim();
    text.toUpperCase();

    return
        text.startsWith("TAKE OFF") ||
        text.startsWith("DEPARTED") ||
        text.startsWith("BOARDING") ||
        text.startsWith("BOARDING CLOSED") ||
        text.startsWith("LAST CALL") ||
        text.startsWith("GATE CLOSED") ||
        text.startsWith("GATE OPEN") ||
        text.startsWith("DELAYED") ||
        text.startsWith("CANCELLED") ||
        text.startsWith("CANCELED") ||
        text.startsWith("CHECK-IN") ||
        text.startsWith("CHECK IN") ||
        text.startsWith("ON TIME") ||
        text.startsWith("EXPECTED");
}


String airportDepartureDisplayTime(const AirportFlight &flight)
{
    String upper = flight.status;
    upper.toUpperCase();

    if (
        upper.startsWith("TAKE OFF") ||
        upper.startsWith("DEPARTED") ||
        upper.startsWith("DELAYED") ||
        upper.startsWith("EXPECTED")
    )
    {
        int minutes = airportClockMinutes(flight.status);

        if (minutes >= 0)
        {
            char buffer[6];

            snprintf(
                buffer,
                sizeof(buffer),
                "%02d:%02d",
                minutes / 60,
                minutes % 60
            );

            return String(buffer);
        }
    }

    return flight.time;
}


String airportDepartureStatus(const AirportFlight &flight)
{
    String status = flight.status;
    String upper = status;

    status.trim();
    upper.toUpperCase();

    if (
        upper.startsWith("TAKE OFF") ||
        upper.startsWith("DEPARTED")
    )
        return "DEPARTED";

    if (upper.startsWith("BOARDING CLOSED")) return "BOARD CLOSED";
    if (upper.startsWith("BOARDING")) return "BOARDING";
    if (upper.startsWith("LAST CALL")) return "LAST CALL";
    if (upper.startsWith("GATE CLOSED")) return "GATE CLOSED";
    if (upper.startsWith("GATE OPEN")) return "GATE OPEN";
    if (upper.startsWith("DELAYED")) return "DELAYED";

    if (
        upper.startsWith("CANCELLED") ||
        upper.startsWith("CANCELED")
    )
        return "CANCELLED";

    if (
        upper.startsWith("CHECK-IN") ||
        upper.startsWith("CHECK IN")
    )
        return "CHECK-IN";

    if (
        upper.startsWith("ON TIME") ||
        upper == "SCHEDULED"
    )
        return "ON TIME";

    if (status.length() > 14)
        status = status.substring(0, 14);

    return status;
}


int airportDepartureSortMinutes(const AirportFlight &flight)
{
    int minutes =
        airportClockMinutes(
            airportDepartureDisplayTime(flight)
        );

    return minutes >= 0
        ? minutes
        : 24 * 60;
}


// ============================================================
// EXPIRATION
// ============================================================

bool airportArrivalExpired(const String &status)
{
    String upper = status;
    upper.toUpperCase();

    if (
        !upper.startsWith("ARRIVED") &&
        !upper.startsWith("LANDED")
    )
        return false;

    int landed = airportClockMinutes(status);
    int now = airportCurrentMinutes();

    if (landed < 0 || now < 0)
        return false;

    int elapsed = now - landed;

    if (elapsed < 0)
        elapsed += 24 * 60;

    if (elapsed > 12 * 60)
        return false;

    return elapsed >= 20;
}


bool airportDepartureExpired(const String &status)
{
    String upper = status;
    upper.toUpperCase();

    if (
        !upper.startsWith("TAKE OFF") &&
        !upper.startsWith("DEPARTED")
    )
        return false;

    int departed = airportClockMinutes(status);
    int now = airportCurrentMinutes();

    if (departed < 0 || now < 0)
        return false;

    int elapsed = now - departed;

    if (elapsed < 0)
        elapsed += 24 * 60;

    if (elapsed > 12 * 60)
        return false;

    return elapsed >= 20;
}


// ============================================================
// FETCH ARRIVALS
// ============================================================

bool fetchAirportArrivals()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("[ARR] WiFi disconnected");
        return false;
    }

    HTTPClient http;

    http.setUserAgent(
        "Mozilla/5.0 NiceFlightRadarV2/2.6"
    );

    http.setTimeout(10000);

    const char *url =
        "https://www.nice.aeroport.fr/en/flights/arrivals";

    Serial.println();
    Serial.println("[ARR] Fetching NCE arrivals");

    if (!http.begin(url))
    {
        Serial.println("[ARR] http.begin failed");
        return false;
    }

    int code = http.GET();

    if (code != HTTP_CODE_OK)
    {
        Serial.printf("[ARR] HTTP %d\n", code);
        http.end();
        return false;
    }

    WiFiClient *stream =
        http.getStreamPtr();

    stream->setTimeout(150);

    const unsigned long started =
        millis();

    constexpr unsigned long MAX_PARSE =
        12000UL;

    AirportFlight temp[AIRPORT_BOARD_MAX];

    int found = 0;

    while (
        http.connected() &&
        found < AIRPORT_BOARD_MAX
    )
    {
        if (millis() - started > MAX_PARSE)
        {
            Serial.println("[ARR] Parsing timeout");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));

        String line =
            stream->readStringUntil('\n');

        vTaskDelay(pdMS_TO_TICKS(1));

        line.trim();

        String clean =
            airportStripHtml(line);

        if (!airportLooksLikeTime(clean))
            continue;

        AirportFlight candidate;

        candidate.time = clean;
        candidate.destination = "NCE";

        String previousText;

        for (int scan = 0; scan < 80; scan++)
        {
            if (millis() - started > MAX_PARSE)
                break;

            if (!http.connected())
                break;

            vTaskDelay(pdMS_TO_TICKS(1));

            String detail =
                stream->readStringUntil('\n');

            vTaskDelay(pdMS_TO_TICKS(1));

            detail.trim();

            String text =
                airportStripHtml(detail);

            if (text.length() == 0)
                continue;

            bool isStatus =
                text.startsWith("Expected") ||
                text.startsWith("Arrived") ||
                text.startsWith("Delayed") ||
                text.startsWith("Cancelled") ||
                text.startsWith("Canceled") ||
                text.startsWith("Landing") ||
                text.startsWith("Landed");

            if (isStatus)
                candidate.status = text;

            if (candidate.terminal.length() == 0)
            {
                String terminal =
                    airportTerminal(text);

                if (terminal.length() > 0)
                    candidate.terminal = terminal;
            }

            if (candidate.flight.length() == 0)
            {
                String token;

                for (
                    unsigned int i = 0;
                    i <= text.length();
                    i++
                )
                {
                    char c =
                        i < text.length()
                            ? text[i]
                            : ' ';

                    if (isalnum(c))
                    {
                        token += c;
                    }
                    else
                    {
                        bool hasDigit = false;
                        bool hasAlpha = false;

                        for (
                            unsigned int j = 0;
                            j < token.length();
                            j++
                        )
                        {
                            hasDigit |= isDigit(token[j]);
                            hasAlpha |= isAlpha(token[j]);
                        }

                        if (
                            hasDigit &&
                            hasAlpha &&
                            token.length() >= 4 &&
                            token.length() <= 8
                        )
                        {
                            candidate.flight = token;

                            int flightPos =
                                text.indexOf(token);

                            if (flightPos > 0)
                            {
                                String rawOrigin =
                                    text.substring(
                                        0,
                                        flightPos
                                    );

                                rawOrigin.trim();

                                if (rawOrigin.length() >= 2)
                                    candidate.origin =
                                        airportCompactCode(
                                            rawOrigin
                                        );
                            }

                            if (
                                candidate.origin.length() == 0 &&
                                previousText.length() >= 2
                            )
                            {
                                candidate.origin =
                                    airportCompactCode(
                                        previousText
                                    );
                            }

                            break;
                        }

                        token = "";
                    }
                }
            }

            bool onlyDigits =
                text.length() > 0;

            for (
                unsigned int i = 0;
                i < text.length();
                i++
            )
            {
                if (!isDigit(text[i]))
                {
                    onlyDigits = false;
                    break;
                }
            }

            if (
                !isStatus &&
                !airportLooksLikeTime(text) &&
                !onlyDigits &&
                text.length() >= 2 &&
                text.length() <= 40 &&
                candidate.flight.length() == 0
            )
            {
                previousText = text;
            }

            if (
                candidate.origin.length() > 0 &&
                candidate.flight.length() > 0 &&
                candidate.status.length() > 0
            )
                break;
        }

        if (
            candidate.origin.length() > 0 &&
            candidate.flight.length() > 0
        )
        {
            if (candidate.status.length() == 0)
                candidate.status = "SCHEDULED";

            if (airportArrivalExpired(candidate.status))
                continue;

            temp[found++] = candidate;
        }
    }

    http.end();

    if (found == 0)
    {
        Serial.println("[ARR] No flights parsed");
        return false;
    }

    for (int i = 0; i < found - 1; i++)
    {
        for (int j = i + 1; j < found; j++)
        {
            if (
                airportArrivalSortMinutes(temp[j]) <
                airportArrivalSortMinutes(temp[i])
            )
            {
                AirportFlight swap = temp[i];
                temp[i] = temp[j];
                temp[j] = swap;
            }
        }
    }

    if (
        airportArrivalsMutex != nullptr &&
        xSemaphoreTake(
            airportArrivalsMutex,
            portMAX_DELAY
        ) == pdTRUE
    )
    {
        for (int i = 0; i < found; i++)
            airportArrivals[i] = temp[i];

        airportArrivalCount = found;

        xSemaphoreGive(
            airportArrivalsMutex
        );
    }

    Serial.printf(
        "[ARR] %d live arrivals parsed\n",
        found
    );

    for (int i = 0; i < found; i++)
    {
        Serial.printf(
            "[ARR] %s %s>NCE %s %s %s\n",
            temp[i].time.c_str(),
            temp[i].origin.c_str(),
            temp[i].flight.c_str(),
            temp[i].terminal.c_str(),
            temp[i].status.c_str()
        );
    }

    return true;
}


// ============================================================
// FETCH DEPARTURES
// ============================================================

bool fetchAirportDepartures()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("[DEP] WiFi disconnected");
        return false;
    }

    HTTPClient http;

    http.setUserAgent(
        "Mozilla/5.0 NiceFlightRadarV2/2.6"
    );

    http.setTimeout(10000);

    const char *url =
        "https://www.nice.aeroport.fr/en/flights/departures";

    Serial.println();
    Serial.println("[DEP] Fetching NCE departures");

    if (!http.begin(url))
    {
        Serial.println("[DEP] http.begin failed");
        return false;
    }

    int code = http.GET();

    if (code != HTTP_CODE_OK)
    {
        Serial.printf("[DEP] HTTP %d\n", code);
        http.end();
        return false;
    }

    WiFiClient *stream =
        http.getStreamPtr();

    stream->setTimeout(150);

    const unsigned long started =
        millis();

    constexpr unsigned long MAX_PARSE =
        12000UL;

    AirportFlight temp[AIRPORT_BOARD_MAX];

    int found = 0;

    while (
        http.connected() &&
        found < AIRPORT_BOARD_MAX
    )
    {
        if (millis() - started > MAX_PARSE)
        {
            Serial.println("[DEP] Parsing timeout");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));

        String line =
            stream->readStringUntil('\n');

        vTaskDelay(pdMS_TO_TICKS(1));

        line.trim();

        String clean =
            airportStripHtml(line);

        if (!airportLooksLikeTime(clean))
            continue;

        AirportFlight candidate;

        candidate.time = clean;
        candidate.origin = "NCE";

        String previousText;

        for (int scan = 0; scan < 80; scan++)
        {
            if (millis() - started > MAX_PARSE)
                break;

            if (!http.connected())
                break;

            vTaskDelay(pdMS_TO_TICKS(1));

            String detail =
                stream->readStringUntil('\n');

            vTaskDelay(pdMS_TO_TICKS(1));

            detail.trim();

            String text =
                airportStripHtml(detail);

            if (text.length() == 0)
                continue;

            bool isStatus =
                airportIsDepartureStatus(text);

            if (isStatus)
                candidate.status = text;

            if (candidate.terminal.length() == 0)
            {
                String terminal =
                    airportTerminal(text);

                if (terminal.length() > 0)
                    candidate.terminal = terminal;
            }

            if (candidate.flight.length() == 0)
            {
                String token;

                for (
                    unsigned int i = 0;
                    i <= text.length();
                    i++
                )
                {
                    char c =
                        i < text.length()
                            ? text[i]
                            : ' ';

                    if (isalnum(c))
                    {
                        token += c;
                    }
                    else
                    {
                        bool hasDigit = false;
                        bool hasAlpha = false;

                        for (
                            unsigned int j = 0;
                            j < token.length();
                            j++
                        )
                        {
                            hasDigit |= isDigit(token[j]);
                            hasAlpha |= isAlpha(token[j]);
                        }

                        if (
                            hasDigit &&
                            hasAlpha &&
                            token.length() >= 4 &&
                            token.length() <= 8
                        )
                        {
                            candidate.flight = token;

                            int flightPos =
                                text.indexOf(token);

                            if (flightPos > 0)
                            {
                                String rawDestination =
                                    text.substring(
                                        0,
                                        flightPos
                                    );

                                rawDestination.trim();

                                if (
                                    rawDestination.length() >= 2
                                )
                                    candidate.destination =
                                        airportCompactCode(
                                            rawDestination
                                        );
                            }

                            if (
                                candidate.destination.length() == 0 &&
                                previousText.length() >= 2
                            )
                            {
                                candidate.destination =
                                    airportCompactCode(
                                        previousText
                                    );
                            }

                            break;
                        }

                        token = "";
                    }
                }
            }

            bool onlyDigits =
                text.length() > 0;

            for (
                unsigned int i = 0;
                i < text.length();
                i++
            )
            {
                if (!isDigit(text[i]))
                {
                    onlyDigits = false;
                    break;
                }
            }

            if (
                !isStatus &&
                !airportLooksLikeTime(text) &&
                !onlyDigits &&
                text.length() >= 2 &&
                text.length() <= 40 &&
                candidate.flight.length() == 0
            )
            {
                previousText = text;
            }

            if (
                candidate.destination.length() > 0 &&
                candidate.flight.length() > 0 &&
                candidate.status.length() > 0
            )
                break;
        }

        if (
            candidate.destination.length() > 0 &&
            candidate.flight.length() > 0
        )
        {
            if (candidate.status.length() == 0)
                candidate.status = "SCHEDULED";

            if (airportDepartureExpired(candidate.status))
                continue;

            temp[found++] = candidate;
        }
    }

    http.end();

    if (found == 0)
    {
        Serial.println("[DEP] No flights parsed");
        return false;
    }

    for (int i = 0; i < found - 1; i++)
    {
        for (int j = i + 1; j < found; j++)
        {
            if (
                airportDepartureSortMinutes(temp[j]) <
                airportDepartureSortMinutes(temp[i])
            )
            {
                AirportFlight swap = temp[i];
                temp[i] = temp[j];
                temp[j] = swap;
            }
        }
    }

    if (
        airportDeparturesMutex != nullptr &&
        xSemaphoreTake(
            airportDeparturesMutex,
            portMAX_DELAY
        ) == pdTRUE
    )
    {
        for (int i = 0; i < found; i++)
            airportDepartures[i] = temp[i];

        airportDepartureCount = found;

        xSemaphoreGive(
            airportDeparturesMutex
        );
    }

    Serial.printf(
        "[DEP] %d live departures parsed\n",
        found
    );

    for (int i = 0; i < found; i++)
    {
        Serial.printf(
            "[DEP] %s NCE>%s %s %s %s\n",
            temp[i].time.c_str(),
            temp[i].destination.c_str(),
            temp[i].flight.c_str(),
            temp[i].terminal.c_str(),
            temp[i].status.c_str()
        );
    }

    return true;
}


// ============================================================
// NETWORK SCHEDULER
// ============================================================

void updateAirportBoardsNetwork()
{
    unsigned long now = millis();

    if (
        lastAirportBoardsFetch != 0 &&
        now - lastAirportBoardsFetch <
            AIRPORT_BOARD_FETCH_INTERVAL
    )
        return;

    lastAirportBoardsFetch = now;

    fetchAirportArrivals();

    vTaskDelay(
        pdMS_TO_TICKS(250)
    );

    fetchAirportDepartures();
}


// ============================================================
// PURGE
// ============================================================

void purgeExpiredAirportBoards()
{
    static unsigned long lastPurge = 0;

    unsigned long now = millis();

    if (now - lastPurge < 5000)
        return;

    lastPurge = now;

    if (
        airportArrivalsMutex != nullptr &&
        xSemaphoreTake(
            airportArrivalsMutex,
            pdMS_TO_TICKS(20)
        ) == pdTRUE
    )
    {
        int write = 0;

        for (
            int i = 0;
            i < airportArrivalCount;
            i++
        )
        {
            if (
                !airportArrivalExpired(
                    airportArrivals[i].status
                )
            )
            {
                if (write != i)
                    airportArrivals[write] =
                        airportArrivals[i];

                write++;
            }
        }

        airportArrivalCount = write;

        xSemaphoreGive(
            airportArrivalsMutex
        );
    }

    if (
        airportDeparturesMutex != nullptr &&
        xSemaphoreTake(
            airportDeparturesMutex,
            pdMS_TO_TICKS(20)
        ) == pdTRUE
    )
    {
        int write = 0;

        for (
            int i = 0;
            i < airportDepartureCount;
            i++
        )
        {
            if (
                !airportDepartureExpired(
                    airportDepartures[i].status
                )
            )
            {
                if (write != i)
                    airportDepartures[write] =
                        airportDepartures[i];

                write++;
            }
        }

        airportDepartureCount = write;

        xSemaphoreGive(
            airportDeparturesMutex
        );
    }
}


// ============================================================
// INIT
// ============================================================

void initAirportBoards()
{
    configTzTime(
        "CET-1CEST,M3.5.0,M10.5.0/3",
        "pool.ntp.org",
        "time.nist.gov"
    );

    Serial.println(
        "[TIME] Europe/Paris configured"
    );

    airportArrivalsMutex =
        xSemaphoreCreateMutex();

    airportDeparturesMutex =
        xSemaphoreCreateMutex();

    Serial.printf(
        "[BOARD] ARR mutex %s | DEP mutex %s\n",
        airportArrivalsMutex ? "OK" : "ERROR",
        airportDeparturesMutex ? "OK" : "ERROR"
    );
}


// ============================================================
// COLORS
// ============================================================

uint16_t airportBoardStatusColor(
    String status
)
{
    status.toUpperCase();

    if (
        status.startsWith("CANCELLED") ||
        status.startsWith("CANCELED")
    )
        return COL_WARNING;

    if (status.startsWith("DELAYED"))
        return COL_WARNING;

    if (
        status.startsWith("EXPECTED") ||
        status.startsWith("LAST CALL") ||
        status.startsWith("GATE CLOSED")
    )
        return COL_HOME;

    if (
        status.startsWith("LANDING") ||
        status.startsWith("BOARDING") ||
        status.startsWith("GATE OPEN") ||
        status.startsWith("CHECK")
    )
        return COL_AIRCRAFT;

    if (
        status.startsWith("ARRIVED") ||
        status.startsWith("LANDED") ||
        status.startsWith("DEPARTED") ||
        status.startsWith("TAKE OFF")
    )
        return gfx->color565(
            150,
            170,
            155
        );

    if (
        status.startsWith("ON TIME") ||
        status == "SCHEDULED"
    )
        return COL_SWEEP;

    return COL_TEXT;
}


// ============================================================
// UI HELPERS
// ============================================================

void drawAirportBoardButton(
    int x,
    int w,
    const char *label,
    bool selected
)
{
    const int y = 442;
    const int h = 30;

    uint16_t fill =
        selected
            ? COL_GRID_DIM
            : COL_BG;

    uint16_t border =
        selected
            ? COL_SWEEP
            : COL_GRID_DIM;

    gfx->fillRect(
        x,
        y,
        w,
        h,
        fill
    );

    gfx->drawRect(
        x,
        y,
        w,
        h,
        border
    );

    gfx->setTextSize(1);

    gfx->setTextColor(
        selected
            ? COL_SWEEP
            : COL_TEXT
    );

    int approxWidth =
        strlen(label) * 6;

    gfx->setCursor(
        x + (w - approxWidth) / 2,
        y + 11
    );

    gfx->print(label);
}


void drawAirportBoardNavigation(
    int selected
)
{
    drawAirportBoardButton(
        4,
        112,
        "RADAR",
        selected == 0
    );

    drawAirportBoardButton(
        124,
        112,
        "ARR",
        selected == 1
    );

    drawAirportBoardButton(
        244,
        112,
        "DEP",
        selected == 2
    );

    drawAirportBoardButton(
        364,
        112,
        "SET",
        selected == 3
    );
}


void drawRadarBoardButtons()
{
    gfx->fillRect(
        6,
        432,
        145,
        30,
        COL_BG
    );

    gfx->drawRect(
        6,
        432,
        68,
        30,
        COL_GRID_DIM
    );

    gfx->drawRect(
        82,
        432,
        68,
        30,
        COL_GRID_DIM
    );

    gfx->setTextSize(1);
    gfx->setTextColor(COL_TEXT);

    gfx->setCursor(
        29,
        443
    );
    gfx->print("ARR");

    gfx->setCursor(
        105,
        443
    );
    gfx->print("DEP");
}


// ============================================================
// GENERIC BOARD
// ============================================================

void drawAirportBoard(
    bool arrivals
)
{
    gfx->fillScreen(
        COL_BG
    );

    // Header
    gfx->fillRect(
        0,
        0,
        SCREEN_W,
        36,
        COL_GRID_DIM
    );

    gfx->setTextColor(COL_TEXT);
    gfx->setTextSize(2);
    gfx->setCursor(115, 11);
    gfx->print("NICE FLIGHT RADAR");

    gfx->setTextSize(2);

    gfx->setTextColor(
        arrivals
            ? COL_SWEEP
            : COL_AIRCRAFT
    );

    gfx->setCursor(
        arrivals ? 146 : 158,
        49
    );

    gfx->print(
        arrivals
            ? "ARRIVALS NCE"
            : "DEPARTURES NCE"
    );

    gfx->drawLine(
        10,
        76,
        470,
        76,
        COL_GRID_DIM
    );

    AirportFlight visible[
        AIRPORT_BOARD_VISIBLE
    ];

    int visibleCount = 0;
    int totalCount = 0;
    int pageCount = 1;
    int currentPage =
        arrivals
            ? airportArrivalPage
            : airportDeparturePage;

    SemaphoreHandle_t mutex =
        arrivals
            ? airportArrivalsMutex
            : airportDeparturesMutex;

    if (
        mutex != nullptr &&
        xSemaphoreTake(
            mutex,
            pdMS_TO_TICKS(20)
        ) == pdTRUE
    )
    {
        totalCount =
            arrivals
                ? airportArrivalCount
                : airportDepartureCount;

        pageCount =
            (
                totalCount +
                AIRPORT_BOARD_VISIBLE - 1
            ) /
            AIRPORT_BOARD_VISIBLE;

        if (pageCount < 1)
            pageCount = 1;

        if (currentPage >= pageCount)
            currentPage = pageCount - 1;

        if (currentPage < 0)
            currentPage = 0;

        if (arrivals)
            airportArrivalPage =
                currentPage;
        else
            airportDeparturePage =
                currentPage;

        int start =
            currentPage *
            AIRPORT_BOARD_VISIBLE;

        for (
            int i = 0;
            i < AIRPORT_BOARD_VISIBLE;
            i++
        )
        {
            int source =
                start + i;

            if (source >= totalCount)
                break;

            visible[visibleCount++] =
                arrivals
                    ? airportArrivals[source]
                    : airportDepartures[source];
        }

        xSemaphoreGive(mutex);
    }

    // Column headers
    gfx->setTextColor(COL_GRID);
    gfx->setTextSize(1);

    gfx->setCursor(10, 84);
    gfx->print("TIME");

    gfx->setCursor(96, 84);
    gfx->print("ROUTE");

    gfx->setCursor(276, 84);
    gfx->print("FLIGHT");

    gfx->setCursor(406, 84);
    gfx->print("TERM");

    if (totalCount == 0)
    {
        gfx->setTextSize(2);
        gfx->setTextColor(COL_HOME);
        gfx->setCursor(160, 220);
        gfx->print("LOADING...");
    }

    const int firstY = 104;
    const int rowHeight = 44;

    for (
        int i = 0;
        i < visibleCount;
        i++
    )
    {
        const AirportFlight &flight =
            visible[i];

        int y =
            firstY +
            i * rowHeight;

        String displayTime =
            arrivals
                ? airportArrivalDisplayTime(
                    flight
                )
                : airportDepartureDisplayTime(
                    flight
                );

        String displayStatus =
            arrivals
                ? airportArrivalStatus(
                    flight
                )
                : airportDepartureStatus(
                    flight
                );

        bool updatedTime =
            displayTime != flight.time;

        gfx->setTextSize(2);

        gfx->setTextColor(
            updatedTime
                ? airportBoardStatusColor(
                    flight.status
                )
                : COL_TEXT
        );

        gfx->setCursor(
            10,
            y
        );

        gfx->print(
            displayTime
        );

        String route;

        if (arrivals)
        {
            route =
                flight.origin +
                " > NCE";
        }
        else
        {
            route =
                "NCE > " +
                flight.destination;
        }

        gfx->setTextColor(COL_TEXT);
        gfx->setCursor(96, y);
        gfx->print(route);

        gfx->setTextColor(COL_AIRCRAFT);
        gfx->setCursor(276, y);
        gfx->print(flight.flight);

        if (flight.terminal.length() > 0)
        {
            gfx->setTextColor(COL_HOME);
            gfx->setCursor(410, y);
            gfx->print(flight.terminal);
        }

        gfx->setTextSize(1);

        gfx->setTextColor(
            airportBoardStatusColor(
                flight.status
            )
        );

        gfx->setCursor(
            96,
            y + 22
        );

        gfx->print(
            displayStatus
        );

        if (updatedTime)
        {
            gfx->setTextColor(COL_GRID_DIM);
            gfx->setCursor(
                12,
                y + 24
            );
            gfx->print(
                flight.time
            );
        }

        gfx->drawLine(
            10,
            y + 37,
            470,
            y + 37,
            COL_GRID_DIM
        );
    }

    // Page indicator + prev/next
    gfx->setTextSize(1);
    gfx->setTextColor(COL_GRID);

    String pageText =
        String(totalCount) +
        " FLIGHTS   PAGE " +
        String(currentPage + 1) +
        "/" +
        String(pageCount);

    gfx->setCursor(
        160,
        419
    );

    gfx->print(
        pageText
    );

    gfx->setTextColor(COL_TEXT);

    gfx->setCursor(18, 419);
    gfx->print("< PREV");

    gfx->setCursor(420, 419);
    gfx->print("NEXT >");

    drawAirportBoardNavigation(
        arrivals ? 1 : 2
    );

    gfx->flush();
}


void drawArrivalsPage()
{
    drawAirportBoard(true);
}


void drawDeparturesPage()
{
    drawAirportBoard(false);
}
