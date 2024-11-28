# ESP32 Solar Tracking, Irrigation, and Environmental Monitoring System

This project is a comprehensive **solar tracking**, **environmental monitoring**, and **irrigation management system** built using an **ESP32 microcontroller**. It integrates various sensors to monitor key environmental parameters such as temperature, humidity, rain, water flow, voltage, and current. The system also automatically optimizes solar panel alignment using **Light Dependent Resistors (LDRs)** and a **servo motor**, ensuring efficient energy capture. Real-time data is displayed on a **16x2 LCD screen**, providing critical insights for solar panel optimization and resource management.

## Features
-** Wi-Fi Connectivity: Establishes a connection to a Wi-Fi network for internet access.
-** Blynk Integration: Sends sensor data to a Blynk dashboard, allowing remote access to irrigation system metrics.*
- **Solar Tracking**: Dynamically adjusts the solar panel’s position using two LDRs and a servo motor to maximize sunlight exposure.
- 
- **Temperature and Humidity Monitoring**: Measures ambient temperature and humidity levels using a DHT11 sensor.
- **Rain Detection**: Detects rainfall using a rain sensor and adjusts system behavior (e.g., irrigation) accordingly.
- **Water Flow Monitoring**: Tracks water usage in real-time with a water flow sensor, allowing for precise water management.
- **Voltage and Current Measurement**: Continuously monitors the voltage and current from connected circuits using voltage and ACS712 current sensors.
- **LCD Display**: Displays real-time sensor data on a 16x2 I2C LCD, including environmental conditions, power statistics, and water usage.
- **Cumulative Water Usage Tracking**: Logs total water usage over time in liters.

## Hardware Requirements

- **ESP32 Development Board**
- **DHT11 Temperature and Humidity Sensor**
- **2x Light Dependent Resistors (LDRs)**
- **Servo Motor** (for solar panel tracking)
- **Water Flow Sensor**
- **Rain Sensor**
- **Voltage Sensor**
- **ACS712 Current Sensor**
- **16x2 I2C LCD Display**
- **Connecting Wires**
- **Power Supply** (for ESP32 and peripherals)

## Pin Configuration

| Component         | Pin Assignment (ESP32)           |
|-------------------|-----------------------------------|
| DHT11 Sensor      | GPIO 4                            |
| LDR1 (Solar)      | GPIO 34                           |
| LDR2 (Solar)      | GPIO 35                           |
| Servo Motor       | GPIO 13                           |
| LCD SDA           | GPIO 21                           |
| LCD SCL           | GPIO 22                           |
| Water Flow Sensor | GPIO 23                           |
| Rain Sensor       | GPIO 19                           |
| Voltage Sensor    | GPIO 33                           |
| Current Sensor    | GPIO 32                           |

## Software Requirements

- **Arduino IDE** with ESP32 core
- **ESP32Servo Library** (for servo motor control)
- **DHT Library** (for temperature and humidity readings)
- **Wire Library** (for I2C communication with the LCD)

## How It Works

### Solar Tracking
The system uses two LDRs to detect light intensity on either side of the solar panel. The `handleSolarTracking()` function compares the light levels and moves the servo motor to adjust the solar panel’s position for optimal sunlight exposure. The servo will shift left or right depending on which LDR detects more light.

### Environmental Monitoring

- **Temperature and Humidity**: The DHT11 sensor provides real-time temperature and humidity readings, which are displayed on the LCD.
- **Rain Detection**: The rain sensor gives a digital output; if rain is detected, the system can modify its behavior (e.g., irrigation control). An "R" is displayed on the LCD when rain is detected.
- **Voltage and Current Measurement**: The system uses the voltage sensor and ACS712 current sensor to measure electrical parameters from the solar system. These values are displayed in real-time.

### Water Flow Monitoring
The flow sensor tracks the water flow rate in liters per minute. It counts pulses from the water flow sensor and uses them to calculate and display total water usage.

### LCD Display
The LCD displays the following information:
- **Temperature** (in °C)
- **Humidity** (in %)
- **Voltage** (in V)
- **Current** (in A)
- **Total Water Usage** (in liters)
- **Rain Detection** (if rain is detected, "R" is displayed)

### Real-Time Interrupt Handling
The flow sensor uses interrupts to count pulses, allowing the system to track water flow accurately without missing any data.

## Code Overview

### Key Functions

- `handleSolarTracking()`: Adjusts the solar panel's angle based on LDR readings.
- `getTemperature()` and `getHumidity()`: Read temperature and humidity values from the DHT11 sensor.
- `isRainDetected()`: Determines if rain is present based on the rain sensor’s output.
- `readVoltage()` and `readCurrent()`: Convert analog sensor values to voltage and current readings.
- `calculateFlowRate()`: Computes the water flow rate in liters per minute from sensor pulses.
- `updateWaterUsage()`: Logs the cumulative water usage.
- `displayData()`: Displays all relevant sensor data on the LCD.

## Example LCD Output

```
T: 25.0C  H: 60.0%
V: 12.5V  I: 0.30A
W: 8.4L   R
```

- **T**: Temperature in Celsius
- **H**: Humidity in percentage
- **V**: Voltage in Volts
- **I**: Current in Amps
- **W**: Water usage in liters
- **R**: Indicates rain is detected

## Blynk Integration

The system uses Blynk to send real-time data to the cloud, allowing you to remotely monitor key metrics from your smartphone or web dashboard. The system sends the following data to Blynk:

- **Temperature** (Blynk Virtual Pin V0)
- **Humidity** (Blynk Virtual Pin V1)
- **Voltage** (Blynk Virtual Pin V2)
- **Current** (Blynk Virtual Pin V3)
- **Water Usage** (Blynk Virtual Pin V4)
