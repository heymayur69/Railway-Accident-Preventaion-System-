/* 
  Railway Accident Prevention System (ESP32)
  -------------------------------------------
  Refactored for reliability, better synchronization, real-time performance, and safe resume logic.
*/

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <IRremote.h>
#include "BluetoothSerial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Pin Definitions
#define BUTTON_PIN 17
#define IR_RECEIVE_PIN 15
#define TRIG_PIN 5
#define ECHO_PIN 18
#define RELAY_PIN 19
#define BUZZER_PIN 4
#define LED_PIN 2

// Timing Constants
static constexpr uint32_t SENSOR_RATE_MS = 25; // Reduced for faster sampling
static constexpr uint32_t BUTTON_DEBOUNCE_MS = 300;
static constexpr uint32_t LIDAR_OBSTACLE_THRESHOLD_MM = 70;
static constexpr uint32_t ULTRASONIC_OBSTACLE_THRESHOLD_CM = 10;
static constexpr uint32_t ULTRASONIC_TIMEOUT_US = 30000UL;

// System States
enum class SystemState : uint8_t { OFF, ON };

// Hazard type definitions using bit fields for simultaneous tracking
typedef enum {
    NO_HAZARD = 0,          // No hazards detected
    OBSTACLE = 1 << 0,      // Obstacle detected by ultrasonic/IR
    TRACK_DEFECT = 1 << 1,  // Track defect detected by LiDAR
    RED_SIGNAL = 1 << 2     // Red signal detected by IR receiver
} HazardType;

// Shared State
static SemaphoreHandle_t stateMutex = nullptr;
static SystemState systemState = SystemState::OFF;
static uint8_t activeHazards = NO_HAZARD;
static SemaphoreHandle_t hazardMutex = nullptr;

// FreeRTOS Handles
static TaskHandle_t sensorTaskHandle = nullptr;
static TaskHandle_t buttonTaskHandle = nullptr;
static TaskHandle_t bluetoothTaskHandle = nullptr;

// Sensor and Comm Objects
static Adafruit_VL53L0X lox;
static BluetoothSerial SerialBT;
static bool lidarSensorAvailable = false;

// Function Declarations
void setup();
void loop();
static void buttonTask(void* parameters);
static void bluetoothTask(void* parameters);
static void setSystemState(SystemState newState);
static void updateTrainState();
static void sensorTask(void* parameters);
static int32_t readLiDAR();
static int32_t readUltrasonic();
static uint8_t readSensors(); // Returns bit field of detected hazards
static void logMessage(const char* message);
static void initSensors();

// Setup Function
void setup() {
    Serial.begin(115200);
    SerialBT.begin("RailwayTrain");

    // Configure pins
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);

    // Set initial state
    digitalWrite(RELAY_PIN, HIGH);  // Train OFF initially
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);

    // Initialize FreeRTOS objects
    stateMutex = xSemaphoreCreateMutex();
    hazardMutex = xSemaphoreCreateMutex();
    if (!stateMutex || !hazardMutex) {
        logMessage("❌ FreeRTOS resource creation failed!");
        while (1) { delay(1000); } // Safety halt
    }

    // Initialize sensors
    initSensors();

    // Create FreeRTOS tasks
    BaseType_t taskCreated = pdFALSE;
    
    taskCreated = xTaskCreatePinnedToCore(
        sensorTask,           // Function
        "SensorTask",         // Name
        4096,                 // Stack size
        nullptr,              // Parameters
        1,                    // Priority
        &sensorTaskHandle,    // Handle
        1);                   // Core
    
    if (taskCreated != pdPASS) {
        logMessage("❌ Failed to create sensor task!");
    }
    
    taskCreated = xTaskCreatePinnedToCore(
        buttonTask,           // Function
        "ButtonTask",         // Name
        2048,                 // Stack size
        nullptr,              // Parameters
        1,                    // Priority
        &buttonTaskHandle,    // Handle
        0);                   // Core
    
    if (taskCreated != pdPASS) {
        logMessage("❌ Failed to create button task!");
    }
    
    taskCreated = xTaskCreatePinnedToCore(
        bluetoothTask,        // Function
        "BluetoothTask",      // Name
        2048,                 // Stack size
        nullptr,              // Parameters
        1,                    // Priority
        &bluetoothTaskHandle, // Handle
        0);                   // Core
    
    if (taskCreated != pdPASS) {
        logMessage("❌ Failed to create bluetooth task!");
    }
        
    logMessage("📲 System Ready. Use button or Bluetooth to toggle ON/OFF.");
}

// Initialize sensors
static void initSensors() {
    // Initialize IR receiver
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    // Initialize VL53L0X sensor with retry
    lidarSensorAvailable = false;
    Wire.begin();
    
    for (int i = 0; i < 3 && !lidarSensorAvailable; i++) {
        lidarSensorAvailable = lox.begin();
        if (!lidarSensorAvailable) {
            delay(100);
        }
    }

    if (!lidarSensorAvailable) {
        logMessage("❌ VL53L0X sensor init failure - continuing without LiDAR");
    } else {
        lox.setMeasurementTimingBudgetMicroSeconds(20000); // Set to 20 ms (20000 μs) for high-speed mode
        logMessage("✅ VL53L0X sensor initialized successfully");
    }
}

// Main Loop - Lightweight since tasks handle the work
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000)); // Just to prevent watchdog timeouts
}

// Button Task
static void buttonTask(void* parameters) {
    TickType_t lastPressTime = 0;
    bool lastState = HIGH;
    
    while (true) {
        bool currentState = digitalRead(BUTTON_PIN);
        TickType_t currentTime = xTaskGetTickCount();
        
        // Check for button press with debounce
        if (lastState == HIGH && currentState == LOW && 
            (currentTime - lastPressTime) > pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
            
            // Check current system state
            SystemState currentSystemState;
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            currentSystemState = systemState;
            xSemaphoreGive(stateMutex);
            
            // Toggle system state
            setSystemState(currentSystemState == SystemState::ON ? SystemState::OFF : SystemState::ON);
            
            // Update last press time
            lastPressTime = currentTime;
        }
        
        lastState = currentState;
        vTaskDelay(pdMS_TO_TICKS(50)); // Small delay to prevent busy-waiting
    }
}

// Bluetooth Task
static void bluetoothTask(void* parameters) {
    char buf[32] = {0};
    
    while (true) {
        if (SerialBT.available()) {
            memset(buf, 0, sizeof(buf));
            size_t len = SerialBT.readBytesUntil('\n', buf, sizeof(buf) - 1);
            buf[len] = '\0';
            
            // Convert to uppercase for case-insensitive comparison
            for (size_t i = 0; i < len; i++) {
                buf[i] = toupper(buf[i]);
            }
            
            // Process command
            SystemState currentState;
            xSemaphoreTake(stateMutex, portMAX_DELAY);
            currentState = systemState;
            xSemaphoreGive(stateMutex);
            
            if (strncmp(buf, "ON", 2) == 0) {
                if (currentState == SystemState::OFF) {
                    setSystemState(SystemState::ON);
                    SerialBT.println("✅ System ON");
                } else {
                    SerialBT.println("ℹ️ System already ON");
                }
            } else if (strncmp(buf, "OFF", 3) == 0) {
                if (currentState == SystemState::ON) {
                    setSystemState(SystemState::OFF);
                    SerialBT.println("❌ System OFF");
                } else {
                    SerialBT.println("ℹ️ System already OFF");
                }
            } else {
                SerialBT.println("⚠️ Invalid command. Use 'ON' or 'OFF'.");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Small delay to prevent busy-waiting
    }
}

// Set System State (thread-safe)
static void setSystemState(SystemState newState) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    systemState = newState;
    xSemaphoreGive(stateMutex);

    if (newState == SystemState::ON) {
        // Read sensors once to initialize activeHazards
        uint8_t initialHazards = readSensors();
        xSemaphoreTake(hazardMutex, portMAX_DELAY);
        activeHazards = initialHazards;
        xSemaphoreGive(hazardMutex);
    }

    updateTrainState(); // Update train state immediately
    
    if (newState == SystemState::ON) {
        logMessage("✅ SYSTEM ON");
    } else {
        logMessage("❌ SYSTEM OFF");
    }
}

// Update Train State based on systemState and activeHazards
static void updateTrainState() {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    SystemState currentState = systemState;
    xSemaphoreGive(stateMutex);

    xSemaphoreTake(hazardMutex, portMAX_DELAY);
    uint8_t currentHazards = activeHazards;
    xSemaphoreGive(hazardMutex);

    if (currentState == SystemState::OFF) {
        digitalWrite(RELAY_PIN, HIGH);  // Stop train
        digitalWrite(BUZZER_PIN, LOW);  // Off
        digitalWrite(LED_PIN, LOW);     // Off
    } else if (currentState == SystemState::ON) {
        if (currentHazards != NO_HAZARD) {
            digitalWrite(RELAY_PIN, HIGH);  // Stop train
            digitalWrite(BUZZER_PIN, HIGH); // On
            digitalWrite(LED_PIN, HIGH);    // On
        } else {
            digitalWrite(RELAY_PIN, LOW);   // Run train
            digitalWrite(BUZZER_PIN, LOW);  // Off
            digitalWrite(LED_PIN, LOW);     // Off
        }
    }
}

// Sensor Task
static void sensorTask(void* parameters) {
    while (true) {
        xSemaphoreTake(stateMutex, portMAX_DELAY);
        SystemState currentState = systemState;
        xSemaphoreGive(stateMutex);

        if (currentState == SystemState::ON) {
            uint8_t currentHazards = readSensors();
            xSemaphoreTake(hazardMutex, portMAX_DELAY);
            activeHazards = currentHazards;
            xSemaphoreGive(hazardMutex);

            updateTrainState();
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_RATE_MS));
    }
}

// Read Sensors and return bit field of detected hazards
static uint8_t readSensors() {
    uint8_t hazards = NO_HAZARD;

    // LiDAR Detection
    if (lidarSensorAvailable) {
        int32_t lidarDist = readLiDAR();
        if (lidarDist > 0) {
            Serial.printf("📏 LiDAR Distance: %d mm\n", lidarDist);
            SerialBT.printf("📏 LiDAR Distance: %d mm\n", lidarDist);
            if (lidarDist > LIDAR_OBSTACLE_THRESHOLD_MM) {
                hazards |= TRACK_DEFECT;
                logMessage("🚧 Track defect detected!");
            }
        }
    }

    // Ultrasonic Detection
    int32_t ultraDist = readUltrasonic();
    if (ultraDist > 0) {
        Serial.printf("📏 Ultrasonic Distance: %d cm\n", ultraDist);
        SerialBT.printf("📏 Ultrasonic Distance: %d cm\n", ultraDist);
        if (ultraDist < ULTRASONIC_OBSTACLE_THRESHOLD_CM) {
            hazards |= OBSTACLE;
            logMessage("⚠️ Obstacle detected!");
        }
    }

    // IR Signal Detection
    if (IrReceiver.decode()) {
        hazards |= RED_SIGNAL;
        logMessage("🔴 Red signal detected!");
        IrReceiver.resume();
    }

    return hazards;
}

// Sensor Reading Functions
static int32_t readLiDAR() {
    if (!lidarSensorAvailable) return -1;
    
    VL53L0X_RangingMeasurementData_t measurement;
    lox.rangingTest(&measurement, false);
    
    // Check for valid measurement
    if (measurement.RangeStatus != 4) {  // 4 = out of range
        return measurement.RangeMilliMeter;
    }
    
    return -1; // Invalid reading
}

static int32_t readUltrasonic() {
    // Send trigger pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Read echo pulse duration with timeout
    long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
    
    // Convert to distance if valid reading
    if (duration > 0) {
        return duration * 0.034 / 2; // Speed of sound conversion to cm
    }
    
    return -1; // Invalid reading
}

// Utility function for consistent logging
static void logMessage(const char* message) {
    Serial.println(message);
    SerialBT.println(message);
}