# NiceFlightRadar V2 — Hardware Baseline

## Hardware Probe V2.0

Physical hardware validated successfully.

### MCU

- ESP32-S3
- Revision 0.2 detected by esptool
- Dual core
- 240 MHz
- Wi-Fi
- Bluetooth LE

### Flash

Physical flash detected at runtime:

- 16,777,216 bytes
- 16 MB

### PSRAM

Embedded PSRAM detected by esptool:

- 8 MB
- AP_3v3
- Octal PSRAM configuration

Runtime:

- Total: ~8 MB
- Free after boot: ~8 MB
- 1 MB external allocation test: PASS

### Internal memory

Hardware Probe V2.0:

- Heap total: ~397 KB
- Heap free after boot: ~372 KB

PSRAM will be used for large graphical buffers and other
memory-intensive components.

### Display

Target:

- IPS
- 480 x 480 pixels
- Touchscreen

LCD controller, display bus, touch controller and GPIO mapping
still need to be identified and validated.

### Upload

USB serial:

- `/dev/cu.usbserial-110`
- Stable upload speed: 460800 baud

921600 baud was tested but was unstable after the esptool stub
changed baud rate.

## Validation status

- [x] ESP32-S3 boot
- [x] 16 MB Flash detection
- [x] 8 MB PSRAM detection
- [x] PSRAM allocation
- [x] Serial upload
- [ ] LCD controller
- [ ] LCD backlight
- [ ] Touch controller
- [ ] Wi-Fi
- [ ] Radar rendering
