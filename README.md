# ESP32 Solar Tracking, Irrigation and Environmental Monitoring System
This project is a comprehensive **solar energy tracking** and **environmental monitoring system** built using an **ESP32 microcontroller**. The system integrates multiple sensors to monitor environmental parameters such as temperature, humidity, rain detection, water flow, voltage, and current, while also optimizing solar panel alignment using **light-dependent resistors (LDRs)** and a **servo motor**. It displays real-time data on an **LCD screen** and logs critical values for solar panel optimization and resource management.

## Features

- **Solar Tracking**: Automatically adjusts the position of a solar panel to maximize energy capture using a servo motor and two LDRs.
- **Temperature & Humidity Monitoring**: Continuously measures the environmental temperature and humidity using a DHT11 sensor.
- **Rain Detection**: Detects rain using a rain sensor and adjusts the system behavior accordingly.
- **Water Flow Measurement**: Tracks water usage in real-time using a flow sensor.
- **Voltage & Current Sensing**: Measures voltage and current from connected circuits using a voltage sensor and an ACS712 current sensor.
- **LCD Display**: Real-time display of all sensor data on a 16x2 LCD screen via I2C.
- **Water Usage Tracking**: Tracks cumulative water usage over time.
  
## Hardware Requirements

- **ESP32 Development Board**
- **DHT11 Temperature and Humidity Sensor**
- **LDRs (2x Light Dependent Resistors)**
- **Servo Motor** (for solar tracking)
- **Water Flow Sensor**
- **Rain Sensor**
- **Voltage Sensor** (Analog input)
- **ACS712 Current Sensor**
- **16x2 I2C LCD Display**
- **Connecting Wires**
- **Power Supply** (for ESP32 and other components)
  
## Pin Configuration

| Component         | Pin Assignment    |
|-------------------|-------------------|
| DHT11 Sensor      | GPIO 4            |
| LDR1 (Solar)      | GPIO 34           |
| LDR2 (Solar)      | GPIO 35           |
| Servo Motor       | GPIO 13           |
| LCD SDA           | GPIO 21           |
| LCD SCL           | GPIO 22           |
| Water Flow Sensor | GPIO 23           |
| Rain Sensor       | GPIO 19           |
| Voltage Sensor    | GPIO 33           |
| Current Sensor    | GPIO 32           |

## Software Requirements

- **Arduino IDE** with ESP32 core
- **ESP32Servo Library**: For controlling the servo motor.
- **DHT Library**: For handling DHT11 temperature and humidity sensor.
- **Wire Library**: For I2C communication with the LCD.

## Installation and Setup

1. **Install Arduino IDE**: If you don’t have it installed, download the Arduino IDE from [here](https://www.arduino.cc/en/software).
   
2. **Set Up ESP32 in Arduino IDE**:
   - Open Arduino IDE and go to **File > Preferences**.
   - In the “Additional Boards Manager URLs” field, add the following URL:
     ```
     https://dl.espressif.com/dl/package_esp32_index.json
     ```
   - Go to **Tools > Board > Board Manager**, search for "ESP32" and install the **ESP32 by Espressif Systems** package.
   
3. **Install Required Libraries**:
   - Install the following libraries from **Sketch > Include Library > Manage Libraries**:
     - **ESP32Servo**
     - **DHT sensor library**
     - **LiquidCrystal_I2C**

4. **Connect Components**:
   - Connect all sensors, servo motor, and the LCD display to the ESP32 as per the pin configuration above.
   
5. **Upload the Code**:
   - Open the `main.ino` file in Arduino IDE.
   - Select the correct ESP32 board and the appropriate port under **Tools**.
   - Upload the code to the ESP32 by clicking the **Upload** button.

## How It Works

### Solar Tracking
The system uses two LDRs to monitor light intensity on either side of the solar panel. Based on the light levels, the servo motor adjusts the solar panel's angle for optimal exposure to sunlight. The `handleSolarTracking()` function compares the LDR values and moves the servo motor in small increments to align the panel.

### Environmental Monitoring
- **Temperature and Humidity**: The DHT11 sensor provides temperature and humidity data. These values are displayed on the LCD.
- **Rain Detection**: The rain sensor outputs a digital signal that is processed to determine whether rain is detected. If rain is detected, an "R" is displayed on the LCD.
- **Voltage and Current Measurement**: The voltage and current sensors provide real-time readings, which are displayed on the LCD. The ACS712 current sensor outputs current in Amps, and the voltage sensor outputs values in Volts.
  
### Water Flow Monitoring
A flow sensor measures the water flow through the system. Every pulse from the sensor corresponds to a certain volume of water. The total water usage is calculated and displayed in liters on the LCD.

### LCD Display
The LCD displays all sensor data including:
- Temperature in Celsius
- Humidity in percentage
- Voltage in Volts
- Current in Amps
- Total water usage in liters
- Rain detection status ("R" symbol)

### Interrupts and Real-Time Data
The water flow sensor uses an interrupt to count pulses, ensuring precise water flow measurements without delay in data collection.

## Code Overview

### Main Components

1. **Solar Tracking**:
   - Adjusts the servo motor based on the difference in LDR values to align the solar panel.
   
2. **Environmental Sensors**:
   - Reads and processes data from the DHT11, rain sensor, voltage sensor, and ACS712 current sensor.
   
3. **Water Flow Measurement**:
   - Tracks pulses from the flow sensor, converts it to flow rate (liters per minute), and updates total water consumption.
   
4. **LCD Display**:
   - Displays sensor values on a 16x2 LCD screen, updating every second.

### Core Functions

- `handleSolarTracking()`: Controls the solar panel alignment based on LDR values.
- `getTemperature()` and `getHumidity()`: Read values from the DHT11 sensor.
- `isRainDetected()`: Checks if rain is detected by the rain sensor.
- `readVoltage()` and `readCurrent()`: Convert raw analog readings to actual voltage and current values.
- `calculateFlowRate()`: Computes the water flow rate in liters per minute based on pulse counts.
- `updateWaterUsage()`: Updates the total water usage in liters.
- `displayData()`: Displays real-time sensor data on the LCD.

## Example Output

```
LCD Display:

T: 25.0C  H: 55.0%
V: 12.5V  I: 0.25A
W: 10.5L   R
```

- **T**: Temperature in Celsius.
- **H**: Humidity in percentage.
- **V**: Voltage in Volts.
- **I**: Current in Amps.
- **W**: Water usage in liters.
- **R**: Rain detected.

## Future Improvements

- **Data Logging**: Add functionality to log data to an SD card or send it to a remote server for historical analysis.
- **Mobile App Integration**: Implement Bluetooth or Wi-Fi connectivity to allow remote monitoring via a smartphone app.
- **Power Optimization**: Integrate sleep modes and power-saving techniques for better energy efficiency.

## License

This project is open-source and available under the [MIT License](LICENSE).

## Contributions

Contributions to this project are welcome. Feel free to open issues or submit pull requests on the [GitHub repository](https://github.com/your-repo-link).

---

## Contact

For any inquiries or contributions, feel free to reach out:

- **Email**: your-email@example.com
- **GitHub**: [Your GitHub Profile](https://github.com/your-profile)

