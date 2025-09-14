/*
 * ESP32 ADCS (Attitude Determination & Control System)
 * CubeSat Solar Sensor Simulation using IR LEDs
 * 
 * Hardware Components:
 * - 2x IR LED 850nm 5mm (solar simulation)
 * - 2x IR Photodiode/Phototransistor (solar sensors)
 * - 2x SG90 Servos (solar panel actuators)
 * - 1x 28BYJ-48 Stepper + ULN2003 (reaction wheel)
 * - ESP32 WROOM module
 */

#include <WiFi.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Stepper.h>
#include <ArduinoJson.h>
#include <math.h>

// ============ PIN DEFINITIONS ============
// IR Solar Simulation System
#define IR_LED_1 23        // Solar simulator LED 1 (+ 100Ω to GND)
#define IR_LED_2 13        // Solar simulator LED 2 (+ 100Ω to GND)
#define IR_SENSOR_1 32     // Solar sensor 1 (3.3V + 10kΩ to sensor to GPIO32)
#define IR_SENSOR_2 33     // Solar sensor 2 (3.3V + 10kΩ to sensor to GPIO33)

// Solar Panel Actuators (SG90 Servos)
#define SERVO_PANEL_X 18   // Solar panel X-axis servo (pitch)
#define SERVO_PANEL_Y 19   // Solar panel Y-axis servo (yaw)

// Reaction Wheel (28BYJ-48 Stepper + ULN2003)
#define STEPPER_IN1 25     // ULN2003 IN1 (Orange wire)
#define STEPPER_IN2 26     // ULN2003 IN2 (Yellow wire)  
#define STEPPER_IN3 27     // ULN2003 IN3 (Pink wire)
#define STEPPER_IN4 14     // ULN2003 IN4 (Blue wire)

// Communication & Status
#define I2C_SDA 21         // I2C Data line
#define I2C_SCL 22         // I2C Clock line
#define STATUS_LED 2       // Built-in LED indicator

// ============ SYSTEM PARAMETERS ============
#define STEPS_PER_REV 2048      // 28BYJ-48 steps per revolution
#define STEPPER_SPEED 10        // RPM for reaction wheel
#define SERVO_MIN_ANGLE 0       // SG90 minimum angle
#define SERVO_MAX_ANGLE 180     // SG90 maximum angle
#define SERVO_CENTER 90         // SG90 center position

// Solar Sensor Parameters
#define SOLAR_THRESHOLD_MIN 200    // Minimum solar detection
#define SOLAR_THRESHOLD_MAX 3000   // Maximum useful reading
#define ECLIPSE_THRESHOLD 100      // Below this = eclipse

// I2C Communication
#define ADCS_I2C_ADDRESS 0x0A
#define MASTER_I2C_ADDRESS 0x08
#define TELEMETRY_I2C_ADDRESS 0x09

// Control System Parameters
#define CONTROL_LOOP_FREQ 50    // Hz
#define PID_KP 1.5              // Proportional gain
#define PID_KI 0.1              // Integral gain  
#define PID_KD 0.3              // Derivative gain
#define MAX_SERVO_STEP 5        // Max degrees per control cycle

// Orbital Simulation
#define ORBITAL_PERIOD_MS 5400000   // 90 minutes
#define ECLIPSE_DURATION_MS 2700000 // 45 minutes eclipse
#define SUN_INTENSITY_CYCLES 10     // Sun intensity variation cycles

// ============ HARDWARE OBJECTS ============
Servo solarPanelX;    // X-axis solar panel servo
Servo solarPanelY;    // Y-axis solar panel servo  
Stepper reactionWheel(STEPS_PER_REV, STEPPER_IN1, STEPPER_IN3, STEPPER_IN2, STEPPER_IN4);

// ============ SYSTEM STATE STRUCTURE ============
struct CubeSatADCS {
  // Solar Sensor Data
  struct {
    int sensor1Raw;           // Raw ADC reading sensor 1
    int sensor2Raw;           // Raw ADC reading sensor 2
    float sensor1Voltage;     // Converted to voltage
    float sensor2Voltage;     // Converted to voltage
    bool sunVisible;          // Sun detection flag
    float sunAzimuth;         // Sun azimuth angle (-90 to +90)
    float sunElevation;       // Sun elevation angle (-90 to +90)
    float sunIntensity;       // Normalized sun intensity (0-1)
  } solar;
  
  // Attitude Data (simulated orbital dynamics)
  struct {
    float roll, pitch, yaw;           // Current attitude (degrees)
    float rollRate, pitchRate, yawRate; // Angular rates (deg/s)
    float targetRoll, targetPitch, targetYaw; // Target attitude
    bool attitudeStable;              // Stability flag
    float attitudeError;              // Total attitude error
  } attitude;
  
  // Solar Panel Control
  struct {
    int panelXAngle;          // Current X panel angle
    int panelYAngle;          // Current Y panel angle  
    int targetXAngle;         // Target X panel angle
    int targetYAngle;         // Target Y panel angle
    float trackingError;      // Solar tracking error
    bool trackingActive;      // Solar tracking status
  } solarPanels;
  
  // Reaction Wheel Control
  struct {
    long position;            // Current stepper position
    int speed;                // Current wheel speed
    bool active;              // Wheel active status
    float momentum;           // Angular momentum storage
  } reactionWheel;
  
  // Mission State
  struct {
    unsigned long missionTime;     // Mission elapsed time (ms)
    unsigned long orbitTime;       // Current orbit time (ms)
    bool inEclipse;               // Eclipse status
    String operatingMode;         // Current operating mode
    float powerGeneration;        // Estimated power generation (W)
    bool systemHealthy;           // Overall system health
  } mission;
  
  // Control System
  struct {
    float pidErrorSumX, pidErrorSumY;    // PID integral terms
    float lastErrorX, lastErrorY;        // PID previous errors
    unsigned long lastControlUpdate;     // Last control loop time
  } control;
};

CubeSatADCS adcs;

// ============ DUAL CORE TASK HANDLES ============
TaskHandle_t sensorTaskHandle;
TaskHandle_t controlTaskHandle;
TaskHandle_t commTaskHandle;

// ============ SETUP FUNCTION ============
void setup() {
  Serial.begin(115200);
  Serial.println("\n======================================");
  Serial.println("    ESP32 CubeSat ADCS System v2.0    ");
  Serial.println("======================================");
  
  // Initialize GPIO pins
  initializeGPIO();
  
  // Initialize I2C communication
  Wire.begin(I2C_SDA, I2C_SCL, ADCS_I2C_ADDRESS);
  Wire.onReceive(receiveI2CCommand);
  Wire.onRequest(sendADCSData);
  Serial.println("✓ I2C Communication initialized");
  
  // Initialize solar panel servos
  solarPanelX.attach(SERVO_PANEL_X);
  solarPanelY.attach(SERVO_PANEL_Y);
  solarPanelX.write(SERVO_CENTER);
  solarPanelY.write(SERVO_CENTER);
  delay(1000); // Allow servos to reach position
  Serial.println("✓ Solar panel servos initialized");
  
  // Initialize reaction wheel stepper
  reactionWheel.setSpeed(STEPPER_SPEED);
  Serial.println("✓ Reaction wheel stepper initialized");
  
  // Initialize system state
  initializeADCS();
  Serial.println("✓ ADCS system state initialized");
  
  // Create FreeRTOS tasks for dual-core operation
  createADCSTasks();
  
  Serial.println("✓ System ready for operation");
  Serial.println("Available commands: status, solar, attitude, eclipse, reset, help");
  Serial.println("======================================\n");
}

void initializeGPIO() {
  // Solar simulation LEDs
  pinMode(IR_LED_1, OUTPUT);
  pinMode(IR_LED_2, OUTPUT);
  digitalWrite(IR_LED_1, LOW);
  digitalWrite(IR_LED_2, LOW);
  
  // Solar sensors (analog inputs)
  pinMode(IR_SENSOR_1, INPUT);
  pinMode(IR_SENSOR_2, INPUT);
  
  // Status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  
  Serial.println("✓ GPIO pins initialized");
}

void initializeADCS() {
  // Initialize solar sensor data
  adcs.solar.sensor1Raw = 0;
  adcs.solar.sensor2Raw = 0;
  adcs.solar.sensor1Voltage = 0.0;
  adcs.solar.sensor2Voltage = 0.0;
  adcs.solar.sunVisible = false;
  adcs.solar.sunAzimuth = 0.0;
  adcs.solar.sunElevation = 0.0;
  adcs.solar.sunIntensity = 0.0;
  
  // Initialize attitude data
  adcs.attitude.roll = 0.0;
  adcs.attitude.pitch = 0.0; 
  adcs.attitude.yaw = 0.0;
  adcs.attitude.rollRate = 0.0;
  adcs.attitude.pitchRate = 0.0;
  adcs.attitude.yawRate = 0.0;
  adcs.attitude.targetRoll = 0.0;
  adcs.attitude.targetPitch = 0.0;
  adcs.attitude.targetYaw = 0.0;
  adcs.attitude.attitudeStable = true;
  adcs.attitude.attitudeError = 0.0;
  
  // Initialize solar panels
  adcs.solarPanels.panelXAngle = SERVO_CENTER;
  adcs.solarPanels.panelYAngle = SERVO_CENTER;
  adcs.solarPanels.targetXAngle = SERVO_CENTER;
  adcs.solarPanels.targetYAngle = SERVO_CENTER;
  adcs.solarPanels.trackingError = 0.0;
  adcs.solarPanels.trackingActive = false;
  
  // Initialize reaction wheel
  adcs.reactionWheel.position = 0;
  adcs.reactionWheel.speed = 0;
  adcs.reactionWheel.active = false;
  adcs.reactionWheel.momentum = 0.0;
  
  // Initialize mission state
  adcs.mission.missionTime = 0;
  adcs.mission.orbitTime = 0;
  adcs.mission.inEclipse = false;
  adcs.mission.operatingMode = "INITIALIZATION";
  adcs.mission.powerGeneration = 0.0;
  adcs.mission.systemHealthy = true;
  
  // Initialize control system
  adcs.control.pidErrorSumX = 0.0;
  adcs.control.pidErrorSumY = 0.0;
  adcs.control.lastErrorX = 0.0;
  adcs.control.lastErrorY = 0.0;
  adcs.control.lastControlUpdate = millis();
}

void createADCSTasks() {
  // Core 0: High-frequency sensor processing
  xTaskCreatePinnedToCore(
    sensorProcessingTask,    // Function name
    "SensorTask",           // Task name
    8192,                   // Stack size (bytes)
    NULL,                   // Parameters
    3,                      // Priority (high)
    &sensorTaskHandle,      // Task handle
    0                       // Core 0
  );
  
  // Core 1: Control algorithms and actuators
  xTaskCreatePinnedToCore(
    controlSystemTask,      // Function name
    "ControlTask",         // Task name
    8192,                  // Stack size (bytes)
    NULL,                  // Parameters
    2,                     // Priority (medium)
    &controlTaskHandle,    // Task handle
    1                      // Core 1
  );
  
  // Core 1: Communication and mission management
  xTaskCreatePinnedToCore(
    communicationTask,     // Function name
    "CommTask",           // Task name
    4096,                 // Stack size (bytes)
    NULL,                 // Parameters
    1,                    // Priority (low)
    &commTaskHandle,      // Task handle
    1                     // Core 1
  );
  
  Serial.println("✓ Dual-core tasks created successfully");
}

// ============ MAIN LOOP ============
void loop() {
  // Main loop handles user interface and diagnostics
  handleSerialCommands();
  updateStatusLED();
  delay(100);
}

// ============ CORE 0: SENSOR PROCESSING TASK ============
void sensorProcessingTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
  
  Serial.println("✓ Sensor processing task started on Core 0");
  
  while (true) {
    // Read solar sensors
    readSolarSensors();
    
    // Calculate sun position
    calculateSunPosition();
    
    // Simulate orbital dynamics
    simulateOrbitalDynamics();
    
    // Update mission time and orbital position
    updateMissionState();
    
    // Maintain precise timing
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ============ CORE 1: CONTROL SYSTEM TASK ============
void controlSystemTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
  
  Serial.println("✓ Control system task started on Core 1");
  
  while (true) {
    // Determine operating mode
    determineOperatingMode();
    
    // Execute control algorithms based on mode
    if (adcs.mission.operatingMode == "SUN_TRACKING") {
      executeSolarTracking();
    } else if (adcs.mission.operatingMode == "ATTITUDE_CONTROL") {
      executeAttitudeControl();
    } else if (adcs.mission.operatingMode == "SAFE_MODE") {
      executeSafeMode();
    }
    
    // Update actuators
    updateActuators();
    
    // Calculate performance metrics
    calculatePerformanceMetrics();
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ============ CORE 1: COMMUNICATION TASK ============
void communicationTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(200); // 5Hz
  
  Serial.println("✓ Communication task started on Core 1");
  
  while (true) {
    // Handle I2C communication with other subsystems
    // Data is automatically sent via interrupt handlers
    
    // Perform system health checks
    performSystemHealthCheck();
    
    // Update solar simulation (eclipse, sun intensity)
    updateSolarSimulation();
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ============ SENSOR PROCESSING FUNCTIONS ============
void readSolarSensors() {
  // Read raw ADC values
  adcs.solar.sensor1Raw = analogRead(IR_SENSOR_1);
  adcs.solar.sensor2Raw = analogRead(IR_SENSOR_2);
  
  // Convert to voltage
  adcs.solar.sensor1Voltage = (adcs.solar.sensor1Raw * 3.3) / 4095.0;
  adcs.solar.sensor2Voltage = (adcs.solar.sensor2Raw * 3.3) / 4095.0;
  
  // Determine sun visibility
  adcs.solar.sunVisible = !adcs.mission.inEclipse && 
                          (adcs.solar.sensor1Raw > SOLAR_THRESHOLD_MIN || 
                           adcs.solar.sensor2Raw > SOLAR_THRESHOLD_MIN);
}

void calculateSunPosition() {
  if (adcs.solar.sunVisible) {
    // Normalize sensor readings (0.0 to 1.0)
    float sensor1Norm = constrain((float)adcs.solar.sensor1Raw / SOLAR_THRESHOLD_MAX, 0.0, 1.0);
    float sensor2Norm = constrain((float)adcs.solar.sensor2Raw / SOLAR_THRESHOLD_MAX, 0.0, 1.0);
    
    // Calculate sun angles based on sensor differential
    // This is a simplified model - real CubeSats use more sophisticated algorithms
    float sensorDiff = sensor1Norm - sensor2Norm;
    
    // Calculate azimuth based on sensor difference
    adcs.solar.sunAzimuth = sensorDiff * 90.0; // -90 to +90 degrees
    
    // Calculate elevation based on average intensity
    float avgIntensity = (sensor1Norm + sensor2Norm) / 2.0;
    adcs.solar.sunElevation = (avgIntensity - 0.5) * 180.0; // -90 to +90 degrees
    
    // Calculate overall sun intensity
    adcs.solar.sunIntensity = avgIntensity;
  } else {
    adcs.solar.sunAzimuth = 0.0;
    adcs.solar.sunElevation = 0.0;
    adcs.solar.sunIntensity = 0.0;
  }
}

void simulateOrbitalDynamics() {
  // Simple orbital attitude simulation
  unsigned long currentTime = millis();
  float orbitPhase = (float)(adcs.mission.orbitTime % ORBITAL_PERIOD_MS) / ORBITAL_PERIOD_MS;
  
  // Simulate natural orbital motion
  adcs.attitude.roll = 10.0 * sin(2 * PI * orbitPhase) + random(-20, 20) / 10.0;
  adcs.attitude.pitch = 15.0 * cos(2 * PI * orbitPhase) + random(-20, 20) / 10.0;  
  adcs.attitude.yaw = 5.0 * sin(4 * PI * orbitPhase) + random(-20, 20) / 10.0;
  
  // Calculate attitude rates (simple differentiation)
  static float lastRoll = 0, lastPitch = 0, lastYaw = 0;
  static unsigned long lastTime = 0;
  
  if (currentTime - lastTime > 0) {
    float dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
    
    adcs.attitude.rollRate = (adcs.attitude.roll - lastRoll) / dt;
    adcs.attitude.pitchRate = (adcs.attitude.pitch - lastPitch) / dt;
    adcs.attitude.yawRate = (adcs.attitude.yaw - lastYaw) / dt;
    
    lastRoll = adcs.attitude.roll;
    lastPitch = adcs.attitude.pitch;
    lastYaw = adcs.attitude.yaw;
    lastTime = currentTime;
  }
  
  // Calculate attitude error
  adcs.attitude.attitudeError = sqrt(pow(adcs.attitude.roll - adcs.attitude.targetRoll, 2) +
                                    pow(adcs.attitude.pitch - adcs.attitude.targetPitch, 2) +
                                    pow(adcs.attitude.yaw - adcs.attitude.targetYaw, 2));
  
  // Determine stability
  adcs.attitude.attitudeStable = (adcs.attitude.attitudeError < 5.0) &&
                                (abs(adcs.attitude.rollRate) < 1.0) &&
                                (abs(adcs.attitude.pitchRate) < 1.0) &&
                                (abs(adcs.attitude.yawRate) < 1.0);
}

void updateMissionState() {
  adcs.mission.missionTime = millis();
  adcs.mission.orbitTime = adcs.mission.missionTime % ORBITAL_PERIOD_MS;
  
  // Determine eclipse status
  adcs.mission.inEclipse = adcs.mission.orbitTime > (ORBITAL_PERIOD_MS - ECLIPSE_DURATION_MS);
}

// ============ CONTROL SYSTEM FUNCTIONS ============
void determineOperatingMode() {
  if (!adcs.mission.systemHealthy) {
    adcs.mission.operatingMode = "SAFE_MODE";
  } else if (adcs.solar.sunVisible && !adcs.mission.inEclipse) {
    adcs.mission.operatingMode = "SUN_TRACKING";
  } else if (adcs.attitude.attitudeError > 10.0) {
    adcs.mission.operatingMode = "ATTITUDE_CONTROL";
  } else {
    adcs.mission.operatingMode = "STANDBY";
  }
}

void executeSolarTracking() {
  adcs.solarPanels.trackingActive = true;
  
  // PID control for solar tracking
  unsigned long now = millis();
  float dt = (now - adcs.control.lastControlUpdate) / 1000.0;
  
  if (dt > 0) {
    // X-axis control (azimuth tracking)
    float errorX = adcs.solar.sunAzimuth;
    adcs.control.pidErrorSumX += errorX * dt;
    float dErrorX = (errorX - adcs.control.lastErrorX) / dt;
    
    float outputX = PID_KP * errorX + PID_KI * adcs.control.pidErrorSumX + PID_KD * dErrorX;
    outputX = constrain(outputX, -MAX_SERVO_STEP, MAX_SERVO_STEP);
    
    // Update target angle
    adcs.solarPanels.targetXAngle = constrain(adcs.solarPanels.panelXAngle + outputX, 
                                             SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    
    // Y-axis control (elevation tracking)
    float errorY = adcs.solar.sunElevation;
    adcs.control.pidErrorSumY += errorY * dt;
    float dErrorY = (errorY - adcs.control.lastErrorY) / dt;
    
    float outputY = PID_KP * errorY + PID_KI * adcs.control.pidErrorSumY + PID_KD * dErrorY;
    outputY = constrain(outputY, -MAX_SERVO_STEP, MAX_SERVO_STEP);
    
    adcs.solarPanels.targetYAngle = constrain(adcs.solarPanels.panelYAngle + outputY,
                                             SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    
    // Calculate tracking error
    adcs.solarPanels.trackingError = sqrt(pow(errorX, 2) + pow(errorY, 2));
    
    // Update control system state
    adcs.control.lastErrorX = errorX;
    adcs.control.lastErrorY = errorY;
    adcs.control.lastControlUpdate = now;
  }
}

void executeAttitudeControl() {
  adcs.reactionWheel.active = true;
  
  // Use reaction wheel for attitude stabilization
  if (adcs.attitude.attitudeError > 5.0) {
    // Simple proportional control for reaction wheel
    int wheelSteps = (int)(adcs.attitude.attitudeError * 10);
    wheelSteps = constrain(wheelSteps, -100, 100);
    
    if (abs(wheelSteps) > 5) {
      reactionWheel.step(wheelSteps);
      adcs.reactionWheel.position += wheelSteps;
      adcs.reactionWheel.momentum += wheelSteps * 0.01; // Simplified momentum calculation
    }
  }
}

void executeSafeMode() {
  // Safe mode: center solar panels and stop reaction wheel
  adcs.solarPanels.targetXAngle = SERVO_CENTER;
  adcs.solarPanels.targetYAngle = SERVO_CENTER;
  adcs.solarPanels.trackingActive = false;
  adcs.reactionWheel.active = false;
  
  // Reset control system
  adcs.control.pidErrorSumX = 0.0;
  adcs.control.pidErrorSumY = 0.0;
}

void updateActuators() {
  // Smooth servo movement to target positions
  if (adcs.solarPanels.panelXAngle != adcs.solarPanels.targetXAngle) {
    if (adcs.solarPanels.panelXAngle < adcs.solarPanels.targetXAngle) {
      adcs.solarPanels.panelXAngle = min(adcs.solarPanels.panelXAngle + 1, adcs.solarPanels.targetXAngle);
    } else {
      adcs.solarPanels.panelXAngle = max(adcs.solarPanels.panelXAngle - 1, adcs.solarPanels.targetXAngle);
    }
    solarPanelX.write(adcs.solarPanels.panelXAngle);
  }
  
  if (adcs.solarPanels.panelYAngle != adcs.solarPanels.targetYAngle) {
    if (adcs.solarPanels.panelYAngle < adcs.solarPanels.targetYAngle) {
      adcs.solarPanels.panelYAngle = min(adcs.solarPanels.panelYAngle + 1, adcs.solarPanels.targetYAngle);
    } else {
      adcs.solarPanels.panelYAngle = max(adcs.solarPanels.panelYAngle - 1, adcs.solarPanels.targetYAngle);
    }
    solarPanelY.write(adcs.solarPanels.panelYAngle);
  }
}

void calculatePerformanceMetrics() {
  // Calculate power generation based on solar tracking efficiency
  if (adcs.solar.sunVisible && adcs.solarPanels.trackingActive) {
    float trackingEfficiency = 1.0 - (adcs.solarPanels.trackingError / 180.0);
    trackingEfficiency = constrain(trackingEfficiency, 0.0, 1.0);
    adcs.mission.powerGeneration = adcs.solar.sunIntensity * trackingEfficiency * 100.0; // Max 100W
  } else {
    adcs.mission.powerGeneration = 0.0;
  }
}

// ============ SOLAR SIMULATION FUNCTIONS ============
void updateSolarSimulation() {
  // Simulate sun intensity variation and eclipse
  if (!adcs.mission.inEclipse) {
    // Calculate sun intensity based on orbital position
    float orbitPhase = (float)adcs.mission.orbitTime / ORBITAL_PERIOD_MS;
    float baseIntensity = (sin(2 * PI * orbitPhase * SUN_INTENSITY_CYCLES) + 1.0) / 2.0;
    
    // Control IR LEDs to simulate sun
    int led1Brightness = (int)(baseIntensity * 255 * (1.0 + sin(orbitPhase * 4 * PI)) / 2.0);
    int led2Brightness = (int)(baseIntensity * 255 * (1.0 + cos(orbitPhase * 4 * PI)) / 2.0);
    
    // Use PWM for variable brightness
    analogWrite(IR_LED_1, led1Brightness);
    analogWrite(IR_LED_2, led2Brightness);
  } else {
    // Eclipse: turn off sun simulation
    digitalWrite(IR_LED_1, LOW);
    digitalWrite(IR_LED_2, LOW);
  }
}

void performSystemHealthCheck() {
  // Check various system parameters
  bool health = true;
  
  // Check sensor readings are within reasonable range
  if (adcs.solar.sensor1Raw > 4090 || adcs.solar.sensor2Raw > 4090) {
    health = false; // Possible sensor disconnection
  }
  
  // Check servo positions are reasonable
  if (adcs.solarPanels.panelXAngle < 0 || adcs.solarPanels.panelXAngle > 180 ||
      adcs.solarPanels.panelYAngle < 0 || adcs.solarPanels.panelYAngle > 180) {
    health = false;
  }
  
  // Check attitude error is not too large
  if (adcs.attitude.attitudeError > 45.0) {
    health = false;
  }
  
  adcs.mission.systemHealthy = health;
}

void updateStatusLED() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  
  unsigned long blinkInterval;
  
  // Different blink patterns for different states
  if (!adcs.mission.systemHealthy) {
    blinkInterval = 100; // Fast blink for fault
  } else if (adcs.mission.operatingMode == "SUN_TRACKING") {
    blinkInterval = 500; // Medium blink for tracking
  } else {
    blinkInterval = 2000; // Slow blink for normal
  }
  
  if (millis() - lastBlink > blinkInterval) {
    ledState = !ledState;
    digitalWrite(STATUS_LED, ledState);
    lastBlink = millis();
  }
}

// ============ I2C COMMUNICATION FUNCTIONS ============
void receiveI2CCommand(int numBytes) {
  String command = "";
  while (Wire.available()) {
    command += (char)Wire.read();
  }
  
  // Process received commands
  if (command == "SAFE_MODE") {
    adcs.mission.operatingMode = "SAFE_MODE";
  } else if (command == "SUN_TRACK") {
    if (adcs.solar.sunVisible) {
      adcs.mission.operatingMode = "SUN_TRACKING";
    }
  } else if (command == "ATTITUDE_HOLD") {
    adcs.mission.operatingMode = "ATTITUDE_CONTROL";
  } else if (command == "RESET") {
    initializeADCS();
  }
  
  Serial.println("I2C Command received: " + command);
}

void sendADCSData() {
  StaticJsonDocument<512> doc;
  
  // Pack telemetry data for transmission
  doc["solar"]["visible"] = adcs.solar.sunVisible;
  doc["solar"]["intensity"] = adcs.solar.sunIntensity;
  doc["solar"]["azimuth"] = adcs.solar.sunAzimuth;
  doc["solar"]["elevation"] = adcs.solar.sunElevation;
  
  doc["attitude"]["roll"] = adcs.attitude.roll;
  doc["attitude"]["pitch"] = adcs.attitude.pitch;
  doc["attitude"]["yaw"] = adcs.attitude.yaw;
  doc["attitude"]["stable"] = adcs.attitude.attitudeStable;
  
  doc["panels"]["x"] = adcs.solarPanels.panelXAngle;
  doc["panels"]["y"] = adcs.solarPanels.panelYAngle;
  doc["panels"]["tracking"] = adcs.solarPanels.trackingActive;
  
  doc["wheel"]["position"] = adcs.reactionWheel.position;
  doc["wheel"]["active"] = adcs.reactionWheel.active;
  
  doc["mission"]["mode"] = adcs.mission.operatingMode;
  doc["mission"]["eclipse"] = adcs.mission.inEclipse;
  doc["mission"]["power"] = adcs.mission.powerGeneration;
  doc["mission"]["healthy"] = adcs.mission.systemHealthy;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  // Send JSON string byte by byte
  for (int i = 0; i < jsonString.length(); i++) {
    Wire.write((uint8_t)jsonString.charAt(i));
  }
}

// ============ SERIAL COMMAND INTERFACE ============
void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "status") {
      printSystemStatus();
    } else if (command == "solar") {
      printSolarData();
    } else if (command == "attitude") {
      printAttitudeData();
    } else if (command == "panels") {
      printSolarPanelData();
    } else if (command == "wheel") {
      printReactionWheelData();
    } else if (command == "eclipse") {
      toggleEclipse();
    } else if (command == "safe") {
      adcs.mission.operatingMode = "SAFE_MODE";
      Serial.println("Entering SAFE MODE");
    } else if (command == "track") {
      if (adcs.solar.sunVisible) {
        adcs.mission.operatingMode = "SUN_TRACKING";
        Serial.println("Entering SUN TRACKING mode");
      } else {
        Serial.println("Cannot track - no sun visible");
      }
    } else if (command == "center") {
      centerSolarPanels();
    } else if (command == "test") {
      runSystemTest();
    } else if (command == "reset") {
      initializeADCS();
      Serial.println("System reset complete");
    } else if (command == "help") {
      printHelp();
    } else {
      Serial.println("Unknown command. Type 'help' for available commands.");
    }
  }
}

void printSystemStatus() {
  Serial.println("\n========== CUBESAT ADCS STATUS ==========");
  Serial.printf("Mission Time: %lu s\n", adcs.mission.missionTime / 1000);
  Serial.printf("Operating Mode: %s\n", adcs.mission.operatingMode.c_str());
  Serial.printf("System Health: %s\n", adcs.mission.systemHealthy ? "HEALTHY" : "FAULT");
  Serial.printf("Eclipse Status: %s\n", adcs.mission.inEclipse ? "IN ECLIPSE" : "SUNLIT");
  Serial.printf("Power Generation: %.1f W\n", adcs.mission.powerGeneration);
  
  Serial.println("\n--- Solar Sensors ---");
  Serial.printf("Sun Visible: %s\n", adcs.solar.sunVisible ? "YES" : "NO");
  Serial.printf("Sun Intensity: %.2f\n", adcs.solar.sunIntensity);
  Serial.printf("Sun Azimuth: %.1f°\n", adcs.solar.sunAzimuth);
  Serial.printf("Sun Elevation: %.1f°\n", adcs.solar.sunElevation);
  Serial.printf("Sensor 1: %d (%.2fV)\n", adcs.solar.sensor1Raw, adcs.solar.sensor1Voltage);
  Serial.printf("Sensor 2: %d (%.2fV)\n", adcs.solar.sensor2Raw, adcs.solar.sensor2Voltage);
  
  Serial.println("\n--- Attitude ---");
  Serial.printf("Roll: %.1f° (Rate: %.2f°/s)\n", adcs.attitude.roll, adcs.attitude.rollRate);
  Serial.printf("Pitch: %.1f° (Rate: %.2f°/s)\n", adcs.attitude.pitch, adcs.attitude.pitchRate);
  Serial.printf("Yaw: %.1f° (Rate: %.2f°/s)\n", adcs.attitude.yaw, adcs.attitude.yawRate);
  Serial.printf("Attitude Error: %.1f°\n", adcs.attitude.attitudeError);
  Serial.printf("Stability: %s\n", adcs.attitude.attitudeStable ? "STABLE" : "UNSTABLE");
  
  Serial.println("\n--- Solar Panels ---");
  Serial.printf("Panel X: %d° (Target: %d°)\n", adcs.solarPanels.panelXAngle, adcs.solarPanels.targetXAngle);
  Serial.printf("Panel Y: %d° (Target: %d°)\n", adcs.solarPanels.panelYAngle, adcs.solarPanels.targetYAngle);
  Serial.printf("Tracking Active: %s\n", adcs.solarPanels.trackingActive ? "YES" : "NO");
  Serial.printf("Tracking Error: %.1f°\n", adcs.solarPanels.trackingError);
  
  Serial.println("\n--- Reaction Wheel ---");
  Serial.printf("Position: %ld steps\n", adcs.reactionWheel.position);
  Serial.printf("Active: %s\n", adcs.reactionWheel.active ? "YES" : "NO");
  Serial.printf("Momentum: %.2f\n", adcs.reactionWheel.momentum);
  
  Serial.println("=========================================\n");
}

void printSolarData() {
  Serial.println("\n--- SOLAR SENSOR DATA ---");
  Serial.printf("Raw ADC 1: %d\n", adcs.solar.sensor1Raw);
  Serial.printf("Raw ADC 2: %d\n", adcs.solar.sensor2Raw);
  Serial.printf("Voltage 1: %.3f V\n", adcs.solar.sensor1Voltage);
  Serial.printf("Voltage 2: %.3f V\n", adcs.solar.sensor2Voltage);
  Serial.printf("Sun Visible: %s\n", adcs.solar.sunVisible ? "TRUE" : "FALSE");
  Serial.printf("Sun Azimuth: %.2f°\n", adcs.solar.sunAzimuth);
  Serial.printf("Sun Elevation: %.2f°\n", adcs.solar.sunElevation);
  Serial.printf("Sun Intensity: %.3f\n", adcs.solar.sunIntensity);
  Serial.println("-------------------------\n");
}

void printAttitudeData() {
  Serial.println("\n--- ATTITUDE DATA ---");
  Serial.printf("Roll: %.2f° (Rate: %.3f°/s)\n", adcs.attitude.roll, adcs.attitude.rollRate);
  Serial.printf("Pitch: %.2f° (Rate: %.3f°/s)\n", adcs.attitude.pitch, adcs.attitude.pitchRate);
  Serial.printf("Yaw: %.2f° (Rate: %.3f°/s)\n", adcs.attitude.yaw, adcs.attitude.yawRate);
  Serial.printf("Target Roll: %.2f°\n", adcs.attitude.targetRoll);
  Serial.printf("Target Pitch: %.2f°\n", adcs.attitude.targetPitch);
  Serial.printf("Target Yaw: %.2f°\n", adcs.attitude.targetYaw);
  Serial.printf("Attitude Error: %.2f°\n", adcs.attitude.attitudeError);
  Serial.printf("Stable: %s\n", adcs.attitude.attitudeStable ? "YES" : "NO");
  Serial.println("--------------------\n");
}

void printSolarPanelData() {
  Serial.println("\n--- SOLAR PANEL DATA ---");
  Serial.printf("Panel X Angle: %d°\n", adcs.solarPanels.panelXAngle);
  Serial.printf("Panel Y Angle: %d°\n", adcs.solarPanels.panelYAngle);
  Serial.printf("Target X Angle: %d°\n", adcs.solarPanels.targetXAngle);
  Serial.printf("Target Y Angle: %d°\n", adcs.solarPanels.targetYAngle);
  Serial.printf("Tracking Active: %s\n", adcs.solarPanels.trackingActive ? "YES" : "NO");
  Serial.printf("Tracking Error: %.2f°\n", adcs.solarPanels.trackingError);
  Serial.println("------------------------\n");
}

void printReactionWheelData() {
  Serial.println("\n--- REACTION WHEEL DATA ---");
  Serial.printf("Position: %ld steps\n", adcs.reactionWheel.position);
  Serial.printf("Speed: %d RPM\n", adcs.reactionWheel.speed);
  Serial.printf("Active: %s\n", adcs.reactionWheel.active ? "YES" : "NO");
  Serial.printf("Momentum: %.3f\n", adcs.reactionWheel.momentum);
  Serial.println("---------------------------\n");
}

void toggleEclipse() {
  adcs.mission.inEclipse = !adcs.mission.inEclipse;
  Serial.printf("Eclipse mode: %s\n", adcs.mission.inEclipse ? "ENABLED" : "DISABLED");
}

void centerSolarPanels() {
  adcs.solarPanels.targetXAngle = SERVO_CENTER;
  adcs.solarPanels.targetYAngle = SERVO_CENTER;
  Serial.println("Solar panels centering...");
}

void runSystemTest() {
  Serial.println("\n=== RUNNING SYSTEM TEST ===");
  
  // Test solar panel movement
  Serial.println("Testing solar panel servos...");
  for (int angle = 45; angle <= 135; angle += 45) {
    adcs.solarPanels.targetXAngle = angle;
    adcs.solarPanels.targetYAngle = angle;
    delay(1000);
    Serial.printf("Panels at %d°\n", angle);
  }
  
  // Center panels
  adcs.solarPanels.targetXAngle = SERVO_CENTER;
  adcs.solarPanels.targetYAngle = SERVO_CENTER;
  delay(1000);
  
  // Test reaction wheel
  Serial.println("Testing reaction wheel...");
  for (int i = 0; i < 5; i++) {
    reactionWheel.step(100);
    adcs.reactionWheel.position += 100;
    delay(500);
    Serial.printf("Wheel position: %ld\n", adcs.reactionWheel.position);
  }
  
  // Test solar simulation
  Serial.println("Testing solar simulation...");
  digitalWrite(IR_LED_1, HIGH);
  digitalWrite(IR_LED_2, HIGH);
  delay(2000);
  Serial.printf("Solar readings: %d, %d\n", analogRead(IR_SENSOR_1), analogRead(IR_SENSOR_2));
  digitalWrite(IR_LED_1, LOW);
  digitalWrite(IR_LED_2, LOW);
  
  Serial.println("System test complete!\n");
}

void printHelp() {
  Serial.println("\n========== AVAILABLE COMMANDS ==========");
  Serial.println("status    - Full system status");
  Serial.println("solar     - Solar sensor data");
  Serial.println("attitude  - Attitude data");
  Serial.println("panels    - Solar panel status");
  Serial.println("wheel     - Reaction wheel status");
  Serial.println("eclipse   - Toggle eclipse mode");
  Serial.println("safe      - Enter safe mode");
  Serial.println("track     - Enter sun tracking mode");
  Serial.println("center    - Center solar panels");
  Serial.println("test      - Run system test");
  Serial.println("reset     - Reset system");
  Serial.println("help      - Show this help");
  Serial.println("=========================================\n");
}