# NiceFlightRadar V2

NiceFlightRadar V2 is a personal, non-commercial flight radar project built around an ESP32-S3.

The goal is to create a small standalone physical radar displaying live aircraft traffic around Nice Côte d'Azur Airport (NCE / LFMN).

## Hardware

- ESP32-S3
- 16 MB Flash
- 8 MB PSRAM
- 480 × 480 IPS display
- ST7701 display controller
- GT911 capacitive touchscreen

## Current features

- 480 × 480 radar display
- Double-buffered rendering using PSRAM
- Geographic aircraft projection
- Animated radar sweep
- Capacitive touchscreen support
- Wi-Fi connectivity
- Live ADS-B integration in development

## Radar area

The project is centered around Nice Côte d'Azur Airport in southern France.

The radar software converts real aircraft latitude/longitude positions into bearing, distance and screen coordinates.

## Project status

NiceFlightRadar V2 is currently under active development.

This is a hobby project only.

It is:

- non-commercial
- not monetized
- not offered as a public flight tracking service
- primarily used by its developer for experimentation and learning

## Data sources

The project experiments with public ADS-B data sources.

API credentials, Wi-Fi credentials and other private configuration are intentionally excluded from this repository.

## License

Personal experimental project. Licensing information may be added later.
