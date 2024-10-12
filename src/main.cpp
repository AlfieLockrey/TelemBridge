// main.cpp
#include <BluetoothSerial.h>
#include <TinyPICO.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "display_manager.h"
#include "pin_definitions.h"
#include "esp_task_wdt.h"
#include "globals.h"

// Telemetry Radio Serial Setup
HardwareSerial TelemSerial(2); // Use UART2

// Reporting Setup
unsigned long lastTime = 0;
unsigned long timeInterval = REPORT_INTERVAL; // Time interval in milliseconds

// Watchdog Timer reset tracking
unsigned long lastWDTReset = 0;

// Bluetooth variables
BluetoothSerial SerialBT; // Bluetooth Serial object
bool btConnected = false;

// Buffers for data handling
uint8_t clientBuffer[BUFFER_SIZE];
uint8_t serialBuffer[BUFFER_SIZE];

// Rate tracking variables
volatile unsigned long uploadedBytes = 0;
volatile unsigned long downloadedBytes = 0;

// Function declarations
void setupBluetooth();
void handleClient();
void resetWDT();
int calculateUploadRate();
int calculateDownloadRate();

void setup() {
  // Initialize Serial for debugging
  Serial.begin(57600);
  while (!Serial) {
    delay(5); // Wait for USB serial to connect
  }
  Serial.println("USB Serial connected.");

  // Initialize Telemetry Serial
  TelemSerial.begin(57600, SERIAL_8N1, TELEM_RX_PIN, TELEM_TX_PIN);
  Serial.println("Telemetry Serial initialized.");

  // Initialize Display
  setupDisplay();
  Serial.println("Display initialized.");

  // Initialize Watchdog Timer
  if (esp_task_wdt_init(WDT_TIMEOUT, true) != ESP_OK) {
    Serial.println("Failed to initialize Watchdog Timer.");
  } else {
    if (esp_task_wdt_add(NULL) != ESP_OK) {
      Serial.println("Failed to add current task to Watchdog Timer.");
    } else {
      Serial.println("Watchdog Timer initialized.");
    }
  }

  // Setup Bluetooth
  setupBluetooth();

  updateDisplay();
}

void setupBluetooth() {
  SerialBT.begin(BT_SPP_NAME); // Start Bluetooth SPP with specified name
  Serial.println("Bluetooth SPP initialized.");
}

void handleClient() {
    // Check if Bluetooth client is connected
    if (SerialBT.connected()) {
        // Handle incoming data from the client
        while (SerialBT.available()) {
            int bytesRead = SerialBT.read(); // Read a single byte
            if (bytesRead != -1) { // Check if a byte was read
                clientBuffer[uploadedBytes % BUFFER_SIZE] = bytesRead; // Store in buffer
                uploadedBytes++;
                if (uploadedBytes >= BUFFER_SIZE) {
                    // Write the whole buffer to Telemetry Serial
                    TelemSerial.write(clientBuffer, BUFFER_SIZE);
                    uploadedBytes = 0; // Reset count after writing
                }
            }
        }

        // Handle outgoing data to the client
        while (TelemSerial.available()) {
            ssize_t bytesRead = TelemSerial.read(serialBuffer, BUFFER_SIZE);
            if (bytesRead > 0) {
                size_t bytesWritten = SerialBT.write(serialBuffer, bytesRead);
                if (bytesWritten > 0) {
                    downloadedBytes += bytesWritten;
                }
            }
        }
    }
}

void loop() {
  // Reset Watchdog Timer
  resetWDT();

  // Handle Bluetooth client connections
  handleClient();

  // Update Display
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= timeInterval) {
    // Calculate rates and update the display
    uploadRate = calculateUploadRate();
    downloadRate = calculateDownloadRate();
    updateDisplay(); // Update the OLED display (26ms)

    // Update lastTime for the next interval
    lastTime = currentTime;
    uploadedBytes = 0;
    downloadedBytes = 0;
  }
}

// Function to reset the Watchdog Timer
void resetWDT() {
  esp_task_wdt_reset();
}

// Example implementations of rate calculation
int calculateUploadRate() {
  return uploadedBytes / (timeInterval / 1000); // Bps
}

int calculateDownloadRate() {
  return downloadedBytes / (timeInterval / 1000); // Bps
}
