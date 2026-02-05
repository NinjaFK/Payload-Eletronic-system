# UNLV IREC Payload Electronics and Software

### Overview
This repository contains the firmware, wiring references, and documentation for the **University of Nevada, Las Vegas (UNLV)** CubeSat‑class payload being flown in the **Intercollegiate Rocket Engineering Competition (IREC)**.  
The payload performs high‑G data logging, video capture, and fluid‑dynamics experiments during flight.

---

### Subsystem Summary
| Subsystem | Hardware | Description |
|------------|-----------|-------------|
| **Microcontroller / Data Logger** | Teensy 4.1 | Logs sensor data to on‑board SD (3.3 V logic) |
| **Sensors** | ADXL375 ±200 g, LSM6DSO32 IMU, BMP388 Barometer, MPR121 Capacitive Sensor, Flow Sensor | Collect acceleration, pressure, and fluid state data |
| **Camera System** | Raspberry Pi Zero v1.3 + Pi Camera Module 3 Wide | Captures video of tank experiment triggered in‑flight |
| **RF Module** | Adafruit RFM95W (LoRa 433 MHz) | Receives pre‑launch commands and sends telemetry |
| **Lighting / Actuation** | Strip LEDs + 12 V Solenoid Valve (via TPS1H200A Switch) | Illumination and controlled fluid dump |
| **Power** | 12.8 V LiFePO₄ pack → 5 V/3.3 V buck converters | Main regulated power bus |

---



