// main.cpp
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
// #include <BluetoothSerial.h> // Removed if not used
#include <TinyPICO.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "display_manager.h"
#include "pin_definitions.h"
#include "esp_task_wdt.h"
#include "globals.h"

// Initialize the Adafruit SSD1306 display (Defined here)
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// Initialize the TinyPICO library (Defined here)
TinyPICO tp;

// Serial and Wi-Fi parameters
#define SERIAL_BUFFER_SIZE 1024
#define WDT_TIMEOUT 10 // Watchdog Timer timeout in seconds
#define TCP_PORT 5760 // Custom port for your TCP server
#define REPORT_INTERVAL 1000 // in milliseconds
#define USB_TIMEOUT 5000 // 5 seconds

HardwareSerial TelemSerial(2); // Use UART2
unsigned long lastTime = 0;
unsigned long timeInterval = REPORT_INTERVAL; // Time interval in milliseconds
unsigned long uploadBytes = 0;
unsigned long downloadBytes = 0;

// Watchdog Timer reset tracking
unsigned long lastWDTReset = 0;

// Wi-Fi variables
WiFiServer server(TCP_PORT);
bool wifiEnabled = false;

// Global Variables (Defined here)
volatile int uploadRate = 0;
volatile int downloadRate = 0;

// USB Switch-Over Control Flag (Set to false to disable switch-over)
const bool ENABLE_USB_SWITCHOVER = false; // Set to 'false' to keep Wi-Fi active during USB debugging

// Client State Management
struct ClientState {
  WiFiClient client;
  bool connected;
};

ClientState currentClient = {WiFiClient(), false};

// USB Connection Tracking
bool usbConnected = false;
unsigned long usbLastActivity = 0;

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
  WiFi.softAP(ssid, password);
  server.begin();
  wifiEnabled = true;
  Serial.println("Wi-Fi Access Point started.");
}

void stopWiFiAP() {
  WiFi.softAPdisconnect(true);
  wifiEnabled = false;
  Serial.println("Wi-Fi Access Point stopped.");
}

void handleClient() {
  if (!currentClient.connected && server.hasClient()) {
    currentClient.client = server.available();
    currentClient.connected = true;
    Serial.println("New client connected.");
  }

  if (currentClient.connected) {
    WiFiClient client = currentClient.client;

    // Handle incoming data from the client
    while (client.available()) {
      char c = client.read();
      TelemSerial.write(c);
      uploadBytes++;
    }

    // Handle outgoing data to the client
    while (TelemSerial.available()) {
      char c = TelemSerial.read();
      client.write(c);
      downloadBytes++;
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
  // Check for USB connection only if switch-over is enabled
  if (ENABLE_USB_SWITCHOVER && Serial) {
    // If USB is connected, disable Wi-Fi
    if (wifiEnabled) {
      stopWiFiAP();
    }

    // Handle USB Serial data
    while (Serial.available()) {
      char c = Serial.read();
      TelemSerial.write(c);
      uploadBytes++;
      usbConnected = true;
      usbLastActivity = millis();
    }
  } else {
    // Handle Wi-Fi client connections
    if (wifiEnabled) {
      handleClient();
    }

    // If USB switch-over is disabled, still handle USB Serial data without affecting Wi-Fi
    if (Serial && !ENABLE_USB_SWITCHOVER) {
      while (Serial.available()) {
        char c = Serial.read();
        TelemSerial.write(c);
        uploadBytes++;
        // Optionally track USB activity if needed
      }
    }
  }

  // Track USB connection timeout only if switch-over is enabled
  if (ENABLE_USB_SWITCHOVER) {
    if (usbConnected && (millis() - usbLastActivity > USB_TIMEOUT)) {
      usbConnected = false;
      if (!wifiEnabled) {
        startWiFiAP();
      }
    }
  }

  // Calculate and print rates at intervals
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= timeInterval) {
    uploadRate = calculateUploadRate();
    downloadRate = calculateDownloadRate();

    Serial.printf("Upload: %d Bps, Download: %d Bps\n", uploadRate, downloadRate);
    lastTime = currentTime;
    uploadBytes = 0;
    downloadBytes = 0;
    updateDisplay(); // Update the OLED display
  }

  resetWDT();
}

// Function to reset the Watchdog Timer
void resetWDT() {
  esp_task_wdt_reset();
}

// Example implementations of rate calculation
int calculateUploadRate() {
  return uploadBytes / (timeInterval / 1000); // Bps
}

int calculateDownloadRate() {
  return downloadBytes / (timeInterval / 1000); // Bps
}
