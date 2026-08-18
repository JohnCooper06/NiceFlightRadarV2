# NiceFlightRadar V2

Second generation of the NiceFlightRadar embedded flight tracker.

## Hardware target

- ESP32-S3
- 8 MB PSRAM
- 16 MB Flash
- 480x480 IPS display
- Touchscreen
- Wi-Fi
- Bluetooth

## Goal

Rebuild the original NiceFlightRadar project on a significantly more
capable hardware platform while retaining the features developed in V1.

Planned features include:

- Real-time aircraft radar
- Aircraft arrivals / departures
- Aircraft touch selection
- Multiple radar ranges
- Rotated radar orientation
- Aircraft route information
- Nice Airport integration
- Wind / METAR information
- Weather information
- Radar sweep animation
- Touch UI
- Future graphical improvements enabled by PSRAM

## Development strategy

V2 starts from a clean hardware baseline.

The original V1 code will not be copied wholesale. Features will be
ported incrementally after the ESP32-S3, PSRAM, display and touchscreen
have been validated.

## Current milestone

Hardware Probe V2.0

Validation of:

- ESP32-S3
- 16 MB Flash
- 8 MB PSRAM
- external PSRAM allocation
