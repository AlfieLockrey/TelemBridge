// main.cpp
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
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

// Wi-Fi variables
WiFiServer server(TCP_PORT);
bool wifiEnabled = false;

// Client State Management
struct ClientState {
  WiFiClient client;
  bool connected;
};
ClientState currentClient = {WiFiClient(), false};

// Buffers for data handling
uint8_t clientBuffer[BUFFER_SIZE];

uint8_t serialBuffer[BUFFER_SIZE];

// Rate tracking variables
volatile unsigned long uploadedBytes = 0;
volatile unsigned long downloadedBytes = 0;

// Function declarations
void startWiFiAP();
void stopWiFiAP();
void handleClient();
void resetWDT();
int calculateUploadRate();
int calculateDownloadRate();

void setup() {
  // Initialize Serial for debugging
  Serial.begin(57600);
  while (!Serial) {
    delay(10); // Wait for USB serial to connect
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

  // Configure AP IP settings (optional)
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(8, 8, 8, 8); // Optional
  IPAddress secondaryDNS(8, 8, 4, 4); // Optional

  if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("AP Config Failed");
  }

  // Start Wi-Fi Access Point
  startWiFiAP();

  // Display setup message
  Serial.println("The device started, now you can connect via Wi-Fi!");
}

void startWiFiAP() {
  const char* ssid = "TelemBridge_ESP32";
  const char* password = "13111999"; // Use a strong password
  if (WiFi.softAP(ssid, password)) {
    server.begin();
    wifiEnabled = true;
    Serial.println("Wi-Fi Access Point started.");
  } else {
    Serial.println("Failed to start Wi-Fi Access Point.");
  }
}

void stopWiFiAP() {
  if (WiFi.softAPdisconnect(true)) {
    wifiEnabled = false;
    Serial.println("Wi-Fi Access Point stopped.");
  } else {
    Serial.println("Failed to stop Wi-Fi Access Point.");
  }
}

void handleClient() {
  if (!currentClient.connected && server.hasClient()) {
    WiFiClient newClient = server.available();
    if (newClient) {
      if (currentClient.client && currentClient.client.connected()) {
        // If a client is already connected, reject the new one
        newClient.stop();
        Serial.println("Rejected new client: already connected.");
      } else {
        currentClient.client = newClient;
        currentClient.connected = true;
        Serial.println("New client connected.");
      }
    }
  }

  if (currentClient.connected) {
    WiFiClient& client = currentClient.client;

    // Handle incoming data from the client
    if (client.available()) {
      ssize_t bytesRead = client.read(clientBuffer, BUFFER_SIZE);
      if (bytesRead > 0) {
        size_t bytesWritten = TelemSerial.write(clientBuffer, bytesRead);
        if (bytesWritten > 0) {
          uploadedBytes += bytesWritten;
        }
      }
    }

    // Handle outgoing data to the client
    if (TelemSerial.available()) {
      ssize_t bytesRead = TelemSerial.read(serialBuffer, BUFFER_SIZE);
      if (bytesRead > 0) {
        size_t bytesWritten = client.write(serialBuffer, bytesRead);
        if (bytesWritten > 0) {
          downloadedBytes += bytesWritten;
        }
      }
    }

    // Check if client is still connected
    if (!client.connected()) {
      client.stop();
      currentClient.connected = false;
      Serial.println("Client disconnected.");
    }
  }
}

void loop() {
  // Reset Watchdog Timer
  resetWDT();

  // Handle Wi-Fi client connections
  if (wifiEnabled) {
    handleClient();
  }

  
  // Update Display
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= timeInterval) {
    uploadRate = calculateUploadRate();
    downloadRate = calculateDownloadRate();
    updateDisplay(); // Update the OLED display

    lastTime = currentTime;
    uploadedBytes = 0;
    downloadedBytes = 0;

    unsigned long elapsedTime = millis() - currentTime; // Calculate elapsed time
    // Print the elapsed time to the Serial Monitor in a single line
    Serial.printf("Display update took: %lu ms\n", elapsedTime);
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
