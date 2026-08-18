# NiceFlightRadar V2 Hardware

## MCU

ESP32-S3

## Memory target

- Flash: 16 MB
- PSRAM: 8 MB

## Display

- Resolution: 480 x 480
- Type: IPS
- Touchscreen: yes

Exact LCD controller, touch controller and GPIO mapping must be
identified before display integration.

## Hardware validation order

1. ESP32-S3 boot
2. Flash detection
3. PSRAM detection
4. PSRAM allocation test
5. LCD controller
6. Backlight
7. Touch controller
8. Wi-Fi
9. Radar rendering
