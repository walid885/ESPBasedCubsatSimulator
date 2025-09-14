/*
 * IR LED Test Program for ESP32
 * Tests 850nm IR LED transmitter and receiver pair
 * 
 * Hardware:
 * - IR LED 850nm 5mm (transmitter) with 100Ω resistor
 * - IR photodiode/phototransistor (receiver) with 10kΩ pull-up
 */

// Pin definitions
#define IR_LED_1 23        // IR transmitter 1 
#define IR_LED_2 13        // IR transmitter 2
#define IR_SENSOR_1 32     // IR receiver 1 (analog)
#define IR_SENSOR_2 33     // IR receiver 2 (analog)
#define STATUS_LED 2       // Built-in LED

// Test parameters
#define IR_THRESHOLD 500   // Analog threshold for IR detection
#define PWM_FREQ 38000     // 38kHz carrier frequency (optional)
#define PWM_CHANNEL_1 0
#define PWM_CHANNEL_2 1
#define PWM_RESOLUTION 8

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== IR LED Test Program ===");
  
  // Initialize pins
  pinMode(IR_LED_1, OUTPUT);
  pinMode(IR_LED_2, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(IR_SENSOR_1, INPUT);
  pinMode(IR_SENSOR_2, INPUT);
  
  // Optional: Setup PWM for modulated IR (more robust detection)
  // Uncomment if you want 38kHz modulated IR
  /*
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(IR_LED_1, PWM_CHANNEL_1);
  ledcAttachPin(IR_LED_2, PWM_CHANNEL_2);
  */
  
  Serial.println("IR Test Ready!");
  Serial.println("Commands:");
  Serial.println("- 'test1' : Test IR pair 1");
  Serial.println("- 'test2' : Test IR pair 2"); 
  Serial.println("- 'both' : Test both pairs");
  Serial.println("- 'continuous' : Continuous monitoring");
  Serial.println("- 'calibrate' : Calibration mode");
}

void loop() {
  handleSerialCommands();
  delay(100);
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "test1") {
      testIRPair(1);
    } else if (command == "test2") {
      testIRPair(2);
    } else if (command == "both") {
      testBothPairs();
    } else if (command == "continuous") {
      continuousMonitoring();
    } else if (command == "calibrate") {
      calibrateIRSensors();
    } else if (command == "help") {
      printHelp();
    } else {
      Serial.println("Unknown command. Type 'help' for available commands.");
    }
  }
}

void testIRPair(int pairNumber) {
  int ledPin = (pairNumber == 1) ? IR_LED_1 : IR_LED_2;
  int sensorPin = (pairNumber == 1) ? IR_SENSOR_1 : IR_SENSOR_2;
  
  Serial.println("\n=== Testing IR Pair " + String(pairNumber) + " ===");
  
  // Test with LED OFF
  digitalWrite(ledPin, LOW);
  delay(100);
  int offReading = analogRead(sensorPin);
  
  // Test with LED ON
  digitalWrite(ledPin, HIGH);
  delay(100);
  int onReading = analogRead(sensorPin);
  
  // Turn LED OFF
  digitalWrite(ledPin, LOW);
  
  // Calculate difference
  int difference = abs(onReading - offReading);
  
  Serial.println("LED OFF reading: " + String(offReading));
  Serial.println("LED ON reading: " + String(onReading));
  Serial.println("Difference: " + String(difference));
  
  if (difference > 100) {
    Serial.println("✓ IR Pair " + String(pairNumber) + " WORKING");
    blinkStatusLED(3, 200); // 3 quick blinks
  } else {
    Serial.println("✗ IR Pair " + String(pairNumber) + " NOT WORKING or weak signal");
    blinkStatusLED(1, 1000); // 1 long blink
  }
  
  Serial.println("========================\n");
}

void testBothPairs() {
  Serial.println("\n=== Testing Both IR Pairs ===");
  testIRPair(1);
  delay(500);
  testIRPair(2);
}

void continuousMonitoring() {
  Serial.println("\n=== Continuous IR Monitoring ===");
  Serial.println("Press any key to stop...");
  
  // Alternate LED patterns for testing
  bool led1State = false;
  bool led2State = false;
  unsigned long lastToggle = 0;
  
  while (!Serial.available()) {
    // Toggle LEDs every 500ms in alternating pattern
    if (millis() - lastToggle > 500) {
      led1State = !led1State;
      led2State = !led2State;
      
      digitalWrite(IR_LED_1, led1State);
      digitalWrite(IR_LED_2, led2State);
      
      lastToggle = millis();
    }
    
    // Read sensors
    int sensor1 = analogRead(IR_SENSOR_1);
    int sensor2 = analogRead(IR_SENSOR_2);
    
    // Print readings
    Serial.print("Sensor1: ");
    Serial.print(sensor1);
    Serial.print(" | Sensor2: ");
    Serial.print(sensor2);
    Serial.print(" | LED1: ");
    Serial.print(led1State ? "ON" : "OFF");
    Serial.print(" | LED2: ");
    Serial.println(led2State ? "ON" : "OFF");
    
    delay(200);
  }
  
  // Turn off LEDs
  digitalWrite(IR_LED_1, LOW);
  digitalWrite(IR_LED_2, LOW);
  
  // Clear serial buffer
  while (Serial.available()) Serial.read();
  
  Serial.println("Monitoring stopped.");
}

void calibrateIRSensors() {
  Serial.println("\n=== IR Sensor Calibration ===");
  Serial.println("Place IR LEDs and sensors in different positions");
  Serial.println("Collecting baseline readings...");
  
  // Collect baseline (no IR)
  digitalWrite(IR_LED_1, LOW);
  digitalWrite(IR_LED_2, LOW);
  delay(500);
  
  int baseline1 = 0, baseline2 = 0;
  for (int i = 0; i < 10; i++) {
    baseline1 += analogRead(IR_SENSOR_1);
    baseline2 += analogRead(IR_SENSOR_2);
    delay(50);
  }
  baseline1 /= 10;
  baseline2 /= 10;
  
  Serial.println("Baseline readings:");
  Serial.println("Sensor 1: " + String(baseline1));
  Serial.println("Sensor 2: " + String(baseline2));
  
  // Test maximum readings
  Serial.println("\nTesting maximum IR signal...");
  digitalWrite(IR_LED_1, HIGH);
  digitalWrite(IR_LED_2, HIGH);
  delay(500);
  
  int max1 = 0, max2 = 0;
  for (int i = 0; i < 10; i++) {
    max1 += analogRead(IR_SENSOR_1);
    max2 += analogRead(IR_SENSOR_2);
    delay(50);
  }
  max1 /= 10;
  max2 /= 10;
  
  digitalWrite(IR_LED_1, LOW);
  digitalWrite(IR_LED_2, LOW);
  
  Serial.println("Maximum readings:");
  Serial.println("Sensor 1: " + String(max1));
  Serial.println("Sensor 2: " + String(max2));
  
  // Calculate suggested thresholds
  int threshold1 = baseline1 + (max1 - baseline1) / 3;
  int threshold2 = baseline2 + (max2 - baseline2) / 3;
  
  Serial.println("\nSuggested thresholds:");
  Serial.println("Sensor 1: " + String(threshold1));
  Serial.println("Sensor 2: " + String(threshold2));
  
  Serial.println("\nCalibration complete!");
}

void blinkStatusLED(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(duration);
    digitalWrite(STATUS_LED, LOW);
    delay(duration);
  }
}

void printHelp() {
  Serial.println("\n=== IR Test Commands ===");
  Serial.println("test1 - Test IR LED/Sensor pair 1");
  Serial.println("test2 - Test IR LED/Sensor pair 2");
  Serial.println("both - Test both pairs sequentially");
  Serial.println("continuous - Real-time monitoring");
  Serial.println("calibrate - Calibration and threshold setup");
  Serial.println("debug - Circuit debugging");
  Serial.println("raw - Raw ADC readings");
  Serial.println("help - Show this help message");
  Serial.println("========================");
}

void debugCircuit() {
  Serial.println("\n=== Circuit Debug Mode ===");
  
  // Test if GPIO pins are working
  Serial.println("Testing GPIO pins...");
  
  // Test IR LED pins
  digitalWrite(IR_LED_1, HIGH);
  delay(500);
  Serial.println("IR LED 1 should be ON now");
  digitalWrite(IR_LED_1, LOW);
  delay(500);
  Serial.println("IR LED 1 should be OFF now");
  
  digitalWrite(IR_LED_2, HIGH);
  delay(500);
  Serial.println("IR LED 2 should be ON now");
  digitalWrite(IR_LED_2, LOW);
  delay(500);
  Serial.println("IR LED 2 should be OFF now");
  
  // Test analog pins with floating inputs
  Serial.println("\nTesting analog pins (floating)...");
  Serial.println("Disconnect IR sensors and try 'raw' command");
  
  Serial.println("Debug complete.");
}

void rawADCTest() {
  Serial.println("\n=== Raw ADC Test ===");
  Serial.println("This tests ADC pins without any circuit");
  Serial.println("Expected readings with floating pins: random values");
  Serial.println("Press any key to stop...");
  
  while (!Serial.available()) {
    int raw1 = analogRead(IR_SENSOR_1);
    int raw2 = analogRead(IR_SENSOR_2);
    
    float voltage1 = (raw1 * 3.3) / 4095.0;
    float voltage2 = (raw2 * 3.3) / 4095.0;
    
    Serial.print("GPIO32: ");
    Serial.print(raw1);
    Serial.print(" (");
    Serial.print(voltage1, 2);
    Serial.print("V) | GPIO33: ");
    Serial.print(raw2);
    Serial.print(" (");
    Serial.print(voltage2, 2);
    Serial.println("V)");
    
    delay(500);
  }
  
  // Clear serial buffer
  while (Serial.available()) Serial.read();
  Serial.println("Raw test stopped.");
}