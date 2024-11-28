// Blynk Authentication Token
#define BLYNK_TEMPLATE_ID "TMPL2SlaLwnEY"
#define BLYNK_TEMPLATE_NAME "Smart Irrigation"
#define BLYNK_AUTH_TOKEN "Y0bq9MgoEVqwLapcwo26t4PaidwhJbHy"

// Blynk Virtual Pins for Widgets
#define BLYNK_VPIN_TEMP V0
#define BLYNK_VPIN_HUMIDITY V1
#define BLYNK_VPIN_VOLTAGE V2
#define BLYNK_VPIN_CURRENT V3
#define BLYNK_VPIN_WATER_USAGE V4

bool isBlynkConnected = false;     // Track Blynk connection status

const char *ssid = "slntgns";      // Replace with your Wi-Fi SSID
const char *pass = "123456789";    // Replace with your Wi-Fi password

// Libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <DHT.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Pin Configurations
#define DHT_PIN 4           // DHT11 data pin
#define LDR1_PIN 34         // LDR1 for solar tracking
#define LDR2_PIN 35         // LDR2 for solar tracking
#define SERVO_PIN 13        // Servo motor control
#define LCD_SDA_PIN 21      // LCD SDA
#define LCD_SCL_PIN 22      // LCD SCL
#define FLOW_SENSOR_PIN 23  // Water flow sensor
#define RAIN_SENSOR_DIGITAL_PIN 19  // Rain sensor digital output pin
#define RAIN_SENSOR_ANALOG_PIN 18   // Rain sensor analog output pin
#define VOLTAGE_PIN 33      // Voltage sensor (Solar Panel)
#define CURRENT_PIN 32      // ACS712 current sensor (Solar Panel)

// Object Definitions
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C LCD address 0x27
DHT dht(DHT_PIN, DHT11);             // DHT11 sensor
Servo solarServo;                    // Servo motor for solar tracking

// Variables for Solar Tracking
int servoPosition = 90;  // Initial servo position (90°)
int ldrThreshold = 50;   // Threshold for LDR sensitivity

// Variables for Sensors
volatile int flowPulseCount = 0;  // Pulse count for flow sensor
float flowRate = 0.0;             // Calculated flow rate
float totalWaterUsed = 0.0;       // Total water usage in liters

// Timing
unsigned long previousMillis = 0;
const unsigned long displayInterval = 1000;  // LCD refresh interval (1 second)
unsigned long lastBlynkUpdate = 0;
const unsigned long blynkUpdateInterval = 2000; // Blynk update interval (2 seconds)

// Function to Count Pulses for Flow Sensor
void IRAM_ATTR countFlowPulse() {
  flowPulseCount++;
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize LCD
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);  // Initialize I2C on ESP32 pins
  lcd.begin(16, 2);                      // Set LCD dimensions
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("System Starting...");
  delay(2000);
  lcd.clear();

  // Initialize DHT11
  dht.begin();

  // Initialize Servo
  solarServo.attach(SERVO_PIN, 500, 2400);  // Attach servo with calibrated PWM range
  solarServo.write(servoPosition);

  // Initialize Flow Sensor
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), countFlowPulse, FALLING);

  // Initialize Rain Sensor
  pinMode(RAIN_SENSOR_DIGITAL_PIN, INPUT);  // Digital pin for detecting rain (high or low)
  pinMode(RAIN_SENSOR_ANALOG_PIN, INPUT);   // Analog pin for measuring rain intensity

  // Initialize Voltage and Current Sensors (Solar Panel)
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);

  // Initialize Wi-Fi and Blynk
  connectToWiFi();
  Blynk.config(BLYNK_AUTH_TOKEN);  // Configure Blynk with the auth token
}

void loop() {
  // Maintain Wi-Fi connection
  maintainWiFiConnection();

  // Maintain Blynk connection
  if (WiFi.status() == WL_CONNECTED) {
    if (!isBlynkConnected) {
      isBlynkConnected = Blynk.connect();  // Attempt to connect to Blynk
    } else {
      Blynk.run();  // Process Blynk tasks
    }
  } else {
    isBlynkConnected = false; // Mark Blynk as disconnected
  }

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

  // Send Data to Blynk at intervals
  if (millis() - lastBlynkUpdate >= blynkUpdateInterval) {
    if (isBlynkConnected) {
      sendDataToBlynk(temperature, humidity, voltage, current, flow);
    }
    lastBlynkUpdate = millis();
  }

  // Delay to prevent overloading the loop
  delay(100);
}

// Handle Solar Tracking based on LDR values
void handleSolarTracking() {
  int ldr1Value = analogRead(LDR1_PIN);
  int ldr2Value = analogRead(LDR2_PIN);
  int positionChange = 0;

  if (ldr1Value > ldr2Value + ldrThreshold && servoPosition > 0) {
    positionChange = -5;  // Move left
  } else if (ldr2Value > ldr1Value + ldrThreshold && servoPosition < 180) {
    positionChange = 5;   // Move right
  }

  // Move only if there's a significant change
  if (positionChange != 0) {
    servoPosition += positionChange;
    solarServo.write(servoPosition);
    delay(500);  // Smooth the movement
  }
}

// Read Temperature from DHT11 sensor with retries
float getTemperature() {
  for (int i = 0; i < 3; i++) {  // Retry up to 3 times
    float temperature = dht.readTemperature();
    if (!isnan(temperature)) {
      return temperature;
    }
    delay(500);  // Wait before retrying
  }
  Serial.println("Temperature Read Error");
  return -1;  // Error value
}

// Read Humidity from DHT11 sensor with retries
float getHumidity() {
  for (int i = 0; i < 3; i++) {
    float humidity = dht.readHumidity();
    if (!isnan(humidity)) {
      return humidity;
    }
    delay(500);
  }
  Serial.println("Humidity Read Error");
  return -1;  // Error value
}

// Check for rain based on rain sensor readings using two pins (digital and analog)
bool isRainDetected() {
  int rainDigitalValue = digitalRead(RAIN_SENSOR_DIGITAL_PIN);  // Check digital pin (high/low)
  int rainAnalogValue = analogRead(RAIN_SENSOR_ANALOG_PIN);     // Check analog pin (rain intensity)

  // Logic: If digital pin is LOW (rain detected), and analog value is below a threshold, return true
  if (rainDigitalValue == LOW && rainAnalogValue < 500) {
    return true;  // Rain is detected
  }
  return false;  // No rain detected
}

// Read Voltage from solar panel with smoothing (averaging 10 readings)
float readVoltage() {
  float voltageSum = 0;
  float voltageConversionFactor = (3.3 / 4095.0) * 11.0;  // Voltage divider factor for scaling

  for (int i = 0; i < 10; i++) {
    int rawValue = analogRead(VOLTAGE_PIN);
    voltageSum += rawValue * voltageConversionFactor;  // Convert to actual voltage
    delay(10);
  }
  return voltageSum / 10.0;  // Return averaged value
}

// Read Current from ACS712 sensor (solar panel) with smoothing (averaging 10 readings)
float readCurrent() {
  float currentSum = 0;
  float currentOffset = 2.5;  // Offset for ACS712 (middle point of sensor output)
  float sensitivity = 0.185;  // ACS712 sensitivity (185mV/A for 5A version)

  for (int i = 0; i < 10; i++) {
    int rawValue = analogRead(CURRENT_PIN);
    float voltage = (rawValue / 4095.0) * 3.3;  // Convert raw ADC value to voltage
    currentSum += (voltage - currentOffset) / sensitivity;  // Calculate current
    delay(10);
  }
  return currentSum / 10.0;  // Return averaged value
}

// Calculate Flow Rate based on flow sensor pulses
float calculateFlowRate() {
  static unsigned long lastFlowCheck = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastFlowCheck >= 1000) {
    flowRate = (flowPulseCount / 7.5);  // Flow rate in liters/minute
    flowPulseCount = 0;
    lastFlowCheck = currentMillis;
  }

  return flowRate;
}

// Update total water usage in liters
void updateWaterUsage(float flow) {
  totalWaterUsed += flow / 60.0;  // Convert from liters/minute to liters/second
}

// Display sensor data on the LCD screen
void displayData(float temperature, float humidity, float voltage, float current, float flow, bool rainDetected) {
  static unsigned long lastDisplay = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastDisplay >= displayInterval) {
    lastDisplay = currentMillis;

    lcd.setCursor(0, 0);

    // Display temperature or "!" if failed
    lcd.print("T: ");
    if (temperature != -1) {
      lcd.print(temperature);
      lcd.print("C ");
    } else {
      lcd.print("C!! ");
    }

    // Display humidity or "!" if failed
    lcd.print("H:");
    if (humidity != -1) {
      lcd.print(humidity);
    } else {
      lcd.print("H!!");
    }

    // Display voltage or "!" if failed
    lcd.setCursor(0, 1);
    lcd.print("V: ");
    if (voltage != 0.0) {
      lcd.print(voltage);
    } else {
      lcd.print("V!!");
    }

    // Display current or "!" if failed
    lcd.print(" I: ");
    if (current != 0.0) {
      lcd.print(current);
    } else {
      lcd.print("I!!");
    }

    // Display water usage and rain status
    lcd.setCursor(8, 1);
    lcd.print("W:");
    lcd.print(totalWaterUsed, 1);  // Total water used

    // Indicate rain if detected
    if (rainDetected) {
      lcd.setCursor(15, 1);
      lcd.print("R");
    }
  }
}

// Send data to Blynk Virtual Pins
void sendDataToBlynk(float temperature, float humidity, float voltage, float current, float flow) {
  Blynk.virtualWrite(BLYNK_VPIN_TEMP, temperature);
  Blynk.virtualWrite(BLYNK_VPIN_HUMIDITY, humidity);
  Blynk.virtualWrite(BLYNK_VPIN_VOLTAGE, voltage);
  Blynk.virtualWrite(BLYNK_VPIN_CURRENT, current);
  Blynk.virtualWrite(BLYNK_VPIN_WATER_USAGE, totalWaterUsed);
}

// Ensure Wi-Fi connection
void connectToWiFi() {
  lcd.setCursor(0, 0);
  lcd.print("Connecting to WiFi...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    lcd.print(".");
  }
  lcd.clear();
  lcd.print("WiFi Connected!");
  delay(2000);
  lcd.clear();
}

// Maintain Wi-Fi connection
void maintainWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    lcd.setCursor(0, 0);
    lcd.print("Reconnecting WiFi...");
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      lcd.print(".");
    }
    lcd.clear();
    lcd.print("WiFi Connected!");
    delay(2000);
    lcd.clear();
  }
}
