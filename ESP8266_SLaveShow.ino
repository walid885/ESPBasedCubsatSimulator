//  libraries to be downloaded via the Library managers
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>

// Hardware Configuration
#define SDA_PIN 4  // GPIO4 (D2)
#define SCL_PIN 5  // GPIO5 (D1)
#define LED_STATUS 2  // GPIO2 (D4) - Built-in LED for status

// I2C Addresses
#define LCD_ADDRESS 0x09
#define ESP32_MASTER_ADDRESS 0x08
#define ESP8266_SLAVE_ADDRESS 0x09

// System Configuration
#define TELEMETRY_INTERVAL 1000  // 1 second
#define DISPLAY_UPDATE_INTERVAL 500  // 0.5 seconds
#define HEALTH_CHECK_INTERVAL 5000   // 5 seconds

// LCD Setup (0x2716x2 or 20x4)
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);

// System State Structure
struct SystemTelemetry {
  float voltage;
  float current;
  float temperature;
  int rssi;
  unsigned long uptime;
  bool sunSensorStatus;
  float attitudeX, attitudeY, attitudeZ;
  int servoPosition1, servoPosition2;
  int stepperPosition;
  String missionPhase;
  bool systemHealth;
};

SystemTelemetry telemetry;
unsigned long lastTelemetryUpdate = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastHealthCheck = 0;
int displayPage = 0;
bool i2cDataReceived = false;

// JSON buffer for data exchange
StaticJsonDocument<512> jsonBuffer;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== CubeSat ESP8266 Telemetry System ===");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN, ESP8266_SLAVE_ADDRESS);
  Wire.onReceive(receiveI2CData);
  Wire.onRequest(sendI2CData);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("CubeSat Init...");
  delay(1000);
  
  // Initialize GPIO
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, HIGH);  // LED off (inverted logic)
  
  // Initialize telemetry structure
  initializeTelemetry();
  
  // Display startup message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Online");
  lcd.setCursor(0, 1);
  lcd.print("Waiting for data");
  
  Serial.println("ESP8266 Telemetry System Ready");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Update telemetry data
  if (currentTime - lastTelemetryUpdate >= TELEMETRY_INTERVAL) {
    //updateTelemetry();
    lastTelemetryUpdate = currentTime;
  }
  
  // Update display
  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
   // updateDisplay();
    lastDisplayUpdate = currentTime;
  }
  
  // System health check
  if (currentTime - lastHealthCheck >= HEALTH_CHECK_INTERVAL) {
    //performHealthCheck();
    lastHealthCheck = currentTime;
  }
  
  // Handle serial commands for testing
  handleSerialCommands();
  
  delay(100);  // Small delay for stability
}

void initializeTelemetry() {
  telemetry.voltage = 3.3;
  telemetry.current = 0.0;
  telemetry.temperature = 25.0;
  telemetry.rssi = WiFi.RSSI();
  telemetry.uptime = 0;
  telemetry.sunSensorStatus = false;
  telemetry.attitudeX = 0.0;
  telemetry.attitudeY = 0.0;
  telemetry.attitudeZ = 0.0;
  telemetry.servoPosition1 = 90;
  telemetry.servoPosition2 = 90;
  telemetry.stepperPosition = 0;
  telemetry.missionPhase = "INIT";
  telemetry.systemHealth = true;
}

void updateTelemetry() {
  // Update local telemetry data
  telemetry.uptime = millis() / 1000;
  telemetry.voltage = 3.3 + (random(-10, 10) / 100.0);  // Simulated voltage variation
  telemetry.current = 0.15 + (random(-5, 5) / 1000.0);  // Simulated current
  telemetry.temperature = 25.0 + (random(-20, 20) / 10.0);  // Simulated temperature
  
  // Print telemetry to serial for debugging
  if (Serial.available() == 0) {  // Only if not processing commands
    printTelemetryToSerial();
  }
}

void updateDisplay() {
  lcd.clear();
  
  // Cycle through different display pages
  switch (displayPage % 4) {
    case 0:  // System status page
      lcd.setCursor(0, 0);
      lcd.print("SYS: ");
      lcd.print(telemetry.missionPhase);
      lcd.setCursor(0, 1);
      lcd.print("UP: ");
      lcd.print(telemetry.uptime);
      lcd.print("s");
      break;
      
    case 1:  // Power status page
      lcd.setCursor(0, 0);
      lcd.print("V:");
      lcd.print(telemetry.voltage, 2);
      lcd.print("V I:");
      lcd.print(telemetry.current * 1000, 0);
      lcd.print("mA");
      lcd.setCursor(0, 1);
      lcd.print("T:");
      lcd.print(telemetry.temperature, 1);
      lcd.print("C");
      break;
      
    case 2:  // Attitude page
      lcd.setCursor(0, 0);
      lcd.print("ATT X:");
      lcd.print(telemetry.attitudeX, 1);
      lcd.setCursor(0, 1);
      lcd.print("Y:");
      lcd.print(telemetry.attitudeY, 1);
      lcd.print(" Z:");
      lcd.print(telemetry.attitudeZ, 1);
      break;
      
    case 3:  // Actuator status page
      lcd.setCursor(0, 0);
      lcd.print("SRV:");
      lcd.print(telemetry.servoPosition1);
      lcd.print("/");
      lcd.print(telemetry.servoPosition2);
      lcd.setCursor(0, 1);
      lcd.print("STP:");
      lcd.print(telemetry.stepperPosition);
      lcd.print(" SUN:");
      lcd.print(telemetry.sunSensorStatus ? "Y" : "N");
      break;
  }
  
  displayPage++;
}

void performHealthCheck() {
  // Check system health indicators
  bool healthStatus = true;
  
  // Voltage check
  if (telemetry.voltage < 3.0 || telemetry.voltage > 3.6) {
    healthStatus = false;
  }
  
  // Temperature check
  if (telemetry.temperature < -10 || telemetry.temperature > 60) {
    healthStatus = false;
  }
  
  // Communication check (if no I2C data received in 10 seconds)
  if (millis() - lastTelemetryUpdate > 10000 && !i2cDataReceived) {
    healthStatus = false;
  }
  
  telemetry.systemHealth = healthStatus;
  
  // Update status LED
  digitalWrite(LED_STATUS, healthStatus ? HIGH : LOW);  // Inverted logic
  
  // Send health status via serial
  Serial.print("Health Check: ");
  Serial.println(healthStatus ? "HEALTHY" : "FAULT");
}

// I2C Data Reception from Master ESP32
void receiveI2CData(int numBytes) {
  String receivedData = "";
  
  while (Wire.available()) {
    char c = Wire.read();
    receivedData += c;
  }
  
  // Parse JSON data
  DeserializationError error = deserializeJson(jsonBuffer, receivedData);
  
  if (!error) {
    // Update telemetry with received data
    if (jsonBuffer.containsKey("sunSensor")) {
      telemetry.sunSensorStatus = jsonBuffer["sunSensor"];
    }
    if (jsonBuffer.containsKey("attitude")) {
      telemetry.attitudeX = jsonBuffer["attitude"]["x"];
      telemetry.attitudeY = jsonBuffer["attitude"]["y"];
      telemetry.attitudeZ = jsonBuffer["attitude"]["z"];
    }
    if (jsonBuffer.containsKey("servos")) {
      telemetry.servoPosition1 = jsonBuffer["servos"]["servo1"];
      telemetry.servoPosition2 = jsonBuffer["servos"]["servo2"];
    }
    if (jsonBuffer.containsKey("stepper")) {
      telemetry.stepperPosition = jsonBuffer["stepper"];
    }
    if (jsonBuffer.containsKey("phase")) {
      telemetry.missionPhase = jsonBuffer["phase"].as<String>();
    }
    
    i2cDataReceived = true;
    Serial.println("I2C Data Received: " + receivedData);
  } else {
    Serial.println("JSON Parse Error: " + String(error.c_str()));
  }
}

// I2C Data Transmission to Master ESP32
void sendI2CData() {
  // Prepare telemetry data as JSON
  jsonBuffer.clear();
  jsonBuffer["voltage"] = telemetry.voltage;
  jsonBuffer["current"] = telemetry.current;
  jsonBuffer["temperature"] = telemetry.temperature;
  jsonBuffer["uptime"] = telemetry.uptime;
  jsonBuffer["health"] = telemetry.systemHealth;
  jsonBuffer["rssi"] = telemetry.rssi;
  
  String jsonString;
  serializeJson(jsonBuffer, jsonString);
  
  // Send data over I2C
  Wire.write(jsonString.c_str());
}

void printTelemetryToSerial() {
  Serial.println("\n=== TELEMETRY DATA ===");
  Serial.println("System Phase: " + telemetry.missionPhase);
  Serial.println("Uptime: " + String(telemetry.uptime) + "s");
  Serial.println("Voltage: " + String(telemetry.voltage) + "V");
  Serial.println("Current: " + String(telemetry.current * 1000) + "mA");
  Serial.println("Temperature: " + String(telemetry.temperature) + "C");
  Serial.println("Sun Sensor: " + String(telemetry.sunSensorStatus ? "ACTIVE" : "INACTIVE"));
  Serial.println("Attitude: X=" + String(telemetry.attitudeX) + " Y=" + String(telemetry.attitudeY) + " Z=" + String(telemetry.attitudeZ));
  Serial.println("Servos: " + String(telemetry.servoPosition1) + "/" + String(telemetry.servoPosition2));
  Serial.println("Stepper: " + String(telemetry.stepperPosition));
  Serial.println("Health: " + String(telemetry.systemHealth ? "HEALTHY" : "FAULT"));
  Serial.println("======================");
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "status") {
      printTelemetryToSerial();
    } else if (command == "test") {
      // Test mode - simulate data
      telemetry.sunSensorStatus = !telemetry.sunSensorStatus;
      telemetry.attitudeX = random(-180, 180);
      telemetry.attitudeY = random(-180, 180);
      telemetry.attitudeZ = random(-180, 180);
      telemetry.missionPhase = "TEST";
      Serial.println("Test mode activated - simulating data");
    } else if (command == "reset") {
      initializeTelemetry();
      lcd.clear();
      lcd.print("System Reset");
      Serial.println("System reset");
    } else if (command == "scan") {
      scanI2CDevices();
    } else if (command == "lcd") {
      testLCD();
    } else if (command == "help") {
      Serial.println("Available commands:");
      Serial.println("- status: Print current telemetry");
      Serial.println("- test: Activate test mode");
      Serial.println("- reset: Reset system");
      Serial.println("- scan: Scan I2C devices");
      Serial.println("- lcd: Test LCD directly");
      Serial.println("- help: Show this help");
    }
  }
}

void scanI2CDevices() {
  int deviceCount = 0;
  Serial.println("I2C Device Scan:");
  
  for (int address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    int error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.println("Scan complete.");
  }
}

void testLCD() {
  Serial.println("Testing LCD directly...");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LCD TEST");
  lcd.setCursor(0, 1);
  lcd.print("Line 2 Works!");
  Serial.println("LCD test sent");
}