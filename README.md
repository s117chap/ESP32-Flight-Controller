# Custom ESP32 Flight Controller for RC Aircraft

## Overview
A custom-built flight control system designed for a fixed-wing RC aircraft (F-22). The firmware runs on an ESP32 microcontroller, integrating multi-axis attitude sensing, barometer data, and real-time multi-channel RC signal processing via the i-BUS protocol.

## Hardware Architecture
* **Microcontroller:** ESP32
* **Sensors:** 
  * GY-521 / MPU6500 (6-Axis IMU for Roll/Pitch attitude estimation)
  * BME280 (Barometer for relative altitude tracking)
* **Communication:** FlySky FS-i6X Transmitter with i-BUS Receiver
* **Actuators:** 3x Servos (Left Aileron/Elevon, Right Aileron/Elevon, Steering/Rudder)
* **Display:** SSD1306 OLED (Real-time telemetry output)

## Key Technical Features (C++)
* **i-BUS Protocol Parsing:** Developed a robust serial parser to decode 14-channel high-speed control signals from the FlySky receiver.
* **Attitude Algorithm:** Implemented a complementary filter combining gyroscope and accelerometer data for accurate, real-time Roll and Pitch stabilization.
* **Dynamic Flight Modes:** 
  * `Manual`: Direct passthrough control.
  * `Assist`: Angle-limited stabilization.
  * `Comfort`: Heavy dampening for smooth cruising.
* **Elevon Mixing:** Computed complex mixer algorithms to translate pitch and roll targets into left/right elevon servo angles.
