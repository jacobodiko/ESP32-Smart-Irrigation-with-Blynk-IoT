// Libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
// #include <LiquidCrystal_I2C.h>

#include <ESP32Servo.h>
#include <DHT.h>

// Pin Configurations
#define DHT_PIN 4           // DHT11 data pin
#define LDR1_PIN 34         // LDR1 for solar tracking
#define LDR2_PIN 35         // LDR2 for solar tracking
#define SERVO_PIN 13        // Servo motor control
#define LCD_SDA_PIN 21      // LCD SDA
#define LCD_SCL_PIN 22      // LCD SCL
#define FLOW_SENSOR_PIN 23  // Water flow sensor
#define RAIN_SENSOR_PIN 19  // Rain sensor digital output
#define VOLTAGE_PIN 33      // Voltage sensor
#define CURRENT_PIN 32      // ACS712 current sensor

// Object Definitions
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C LCD address 0x27
DHT dht(DHT_PIN, DHT11);             // DHT11 sensor
Servo solarServo;                    // Servo motor for solar tracking

// Variables for Solar Tracking
int servoPosition = 90; // Initial servo position (90°)
int ldrThreshold = 50;  // Threshold for LDR sensitivity

// Variables for Sensors
volatile int flowPulseCount = 0; // Pulse count for flow sensor
float flowRate = 0.0;            // Calculated flow rate
float totalWaterUsed = 0.0;      // Total water usage in liters

// Timing
unsigned long previousMillis = 0;
const unsigned long displayInterval = 1000; // LCD refresh interval (1 second)

// Function to Count Pulses for Flow Sensor
void IRAM_ATTR countFlowPulse() {
    flowPulseCount++;
}

void setup() {
    // Initialize Serial Monitor
    Serial.begin(115200);
    while (!Serial);

    // Initialize LCD
    Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN); // Initialize I2C on ESP32 pins
    lcd.begin(16, 2);                     // Set LCD dimensions
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("System Starting...");
    delay(2000);
    lcd.clear();

    // Initialize DHT11
    dht.begin();

    // Initialize Servo
    solarServo.attach(SERVO_PIN, 500, 2400); // Attach servo with calibrated PWM range
    solarServo.write(servoPosition);

    // Initialize Flow Sensor
    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), countFlowPulse, FALLING);

    // Initialize Rain Sensor
    pinMode(RAIN_SENSOR_PIN, INPUT);

    // Initialize Voltage and Current Sensors
    pinMode(VOLTAGE_PIN, INPUT);
    pinMode(CURRENT_PIN, INPUT);
}

void loop() {
    // Handle Solar Tracking
    handleSolarTracking();

    // Get Sensor Data
    float temperature = getTemperature();
    float humidity = getHumidity();
    bool rainDetected = isRainDetected();
    float voltage = readVoltage();
    float current = readCurrent();
    float flow = calculateFlowRate();
    updateWaterUsage(flow);

    // Display Data on LCD
    displayData(temperature, humidity, voltage, current, flow, rainDetected);

    // Delay to Prevent Overloading the Loop
    delay(100);
}

void handleSolarTracking() {
    int ldr1Value = analogRead(LDR1_PIN);
    int ldr2Value = analogRead(LDR2_PIN);

    if (ldr1Value > ldr2Value + ldrThreshold && servoPosition > 0) {
        servoPosition -= 5; // Move left
    } else if (ldr2Value > ldr1Value + ldrThreshold && servoPosition < 180) {
        servoPosition += 5; // Move right
    }

    solarServo.write(servoPosition);
    delay(15); // Smooth servo movement
}

float getTemperature() {
    float temperature = dht.readTemperature();
    if (isnan(temperature)) {
        Serial.println("Failed to read temperature!");
        return -1; // Error value
    }
    return temperature;
}

float getHumidity() {
    float humidity = dht.readHumidity();
    if (isnan(humidity)) {
        Serial.println("Failed to read humidity!");
        return -1; // Error value
    }
    return humidity;
}

bool isRainDetected() {
    int rainValue = analogRead(RAIN_SENSOR_PIN);
    return rainValue < 500; // Assuming lower value means rain
}

float readVoltage() {
    int rawValue = analogRead(VOLTAGE_PIN);
    float voltage = (rawValue / 4095.0) * 3.3 * 5.0; // Assuming voltage divider scaling
    return voltage;
}

float readCurrent() {
    int rawValue = analogRead(CURRENT_PIN);
    float current = ((rawValue / 4095.0) * 3.3 - 2.5) / 0.185; // ACS712 5A module
    return current;
}

float calculateFlowRate() {
    static unsigned long lastFlowCheck = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastFlowCheck >= 1000) {
        flowRate = (flowPulseCount / 7.5); // Calculate flow rate in liters/minute
        flowPulseCount = 0;
        lastFlowCheck = currentMillis;
    }
    return flowRate;
}

void updateWaterUsage(float flow) {
    totalWaterUsed += flow / 60.0; // Convert from liters/minute to liters/second
}

void displayData(float temperature, float humidity, float voltage, float current, float flow, bool rainDetected) {
    static unsigned long lastDisplay = 0;
    unsigned long currentMillis = millis();

    if (currentMillis - lastDisplay >= displayInterval) {
        lastDisplay = currentMillis;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("T:");
        lcd.print(temperature);
        lcd.print("C H:");
        lcd.print(humidity);
        lcd.setCursor(0, 1);
        lcd.print("V:");
        lcd.print(voltage);
        lcd.print(" I:");
        lcd.print(current);
        lcd.setCursor(8, 1);
        lcd.print("W:");
        lcd.print(totalWaterUsed, 1); // Total water used
        if (rainDetected) {
            lcd.setCursor(15, 1);
            lcd.print("R");
        }
    }
}
