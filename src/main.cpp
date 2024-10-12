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
unsigned long lastHandleTime = 0;
uint8_t clientBuffer[CLIENT_BUFFER_SIZE];
uint8_t serialBuffer[SERIAL_BUFFER_SIZE];

// Rate tracking variables
volatile unsigned long uploadedBytes = 0;
volatile unsigned long downloadedBytes = 0;

// Function declarations
void setupBluetooth();
void handleClient();
void resetWDT();
void clearBuffers();
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

  // Clear buffers on boot
  clearBuffers();

  updateDisplay();
}

void setupBluetooth() {
  SerialBT.begin(BT_SPP_NAME); // Start Bluetooth SPP with specified name
  Serial.println("Bluetooth SPP initialized.");
}

void handleClient() {
  // Handle incoming data from the client in non-blocking chunks
  int availableBytes = SerialBT.available();
  if (availableBytes > 0) {
    // Read only available bytes, but limit to buffer size
    int bytesToRead = min(availableBytes, static_cast<int>(CLIENT_BUFFER_SIZE - uploadedBytes));
    int bytesRead = SerialBT.readBytes(clientBuffer + uploadedBytes, bytesToRead);

    uploadedBytes += bytesRead;
    if (uploadedBytes >= CLIENT_BUFFER_SIZE) {
      // Write the whole buffer to Telemetry Serial
      TelemSerial.write(clientBuffer, CLIENT_BUFFER_SIZE);
      uploadedBytes = 0; // Reset count after writing
    }
  }

  // Handle outgoing data to the client in non-blocking chunks
  availableBytes = TelemSerial.available();
  if (availableBytes > 0) {
    // Read only available bytes, but limit to buffer size
    int bytesToRead = min(availableBytes, SERIAL_BUFFER_SIZE);
    int bytesRead = TelemSerial.readBytes(serialBuffer, bytesToRead);

    if (bytesRead > 0) {
      size_t bytesWritten = SerialBT.write(serialBuffer, bytesRead);
      if (bytesWritten > 0) {
        downloadedBytes += bytesWritten;
      }
    }
  }
}

void loop() {
  // Reset Watchdog Timer
  resetWDT();

  // Handle Bluetooth client connections
  // Check if 30ms have passed since the last call to handleClient
  if (millis() - lastHandleTime >= 30) {
    if (SerialBT.connected()) {
      handleClient();
    }
    lastHandleTime = millis(); // Update the last call time
  }

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

void clearBuffers() {
    memset(clientBuffer, 0, sizeof(clientBuffer));  // Clear client buffer
    memset(serialBuffer, 0, sizeof(serialBuffer));  // Clear serial buffer
    uploadedBytes = 0;
    downloadedBytes = 0;
}