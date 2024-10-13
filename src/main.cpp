#include <BluetoothSerial.h>
#include <TinyPICO.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include "display_manager.h"
#include "pin_definitions.h"
#include "esp_task_wdt.h"
#include "globals.h"
#include <mavlink.h> // Include MAVLink library
#include "mavlink_message_types.h" // Include the new header for message types
#include "driver/uart.h"

// Telemetry Radio Serial Setup
HardwareSerial TelemSerial(1); // Use UART2

// Reporting Setup
unsigned long lastTime = 0;
unsigned long timeInterval = UPDATE_INTERVAL; // Time interval in milliseconds

// Watchdog Timer reset tracking
unsigned long lastWDTReset = 0;

// Bluetooth variables
BluetoothSerial SerialBT; // Bluetooth Serial object
bool btConnected = false;

// Buffers for data handling
uint8_t clientBuffer[CLIENT_BUFFER_SIZE];
uint8_t serialBuffer[SERIAL_BUFFER_SIZE];

// Rate tracking variables
volatile unsigned long uploadedBytes = 0;
volatile unsigned long downloadedBytes = 0;

// MAVLink tracking variables
uint8_t lastBTSeq = 0; // Last sequence number received from Bluetooth
uint8_t lastSerialSeq = 0; // Last sequence number received from Telemetry Serial
unsigned long lostBTPackets = 0; // Count lost packets from Bluetooth
unsigned long lostSerialPackets = 0; // Count lost packets from Telemetry Serial

// Additional variables to track total bytes
unsigned long totalBTBytesIn = 0;    // Total bytes received from Bluetooth
unsigned long totalBTBytesOut = 0;   // Total bytes sent to Bluetooth
unsigned long totalSerialBytesIn = 0;   // Total bytes received from Telemetry Serial
unsigned long totalSerialBytesOut = 0;  // Total bytes sent to Telemetry Serial

unsigned long lastReportTime = 0; // Time tracking for reporting data rate
unsigned long reportInterval = REPORT_INTERVAL; // 10 seconds reporting interval

// Function declarations
void setupBluetooth();
void handleDataFlow();
void handleBTtoSerial();
void handleSerialtoBT();
void resetWDT();
void reportLostPackets(const char* source, uint8_t actualSeq, uint8_t expectedSeq, uint8_t msgId);
void reportTotalBytes();
bool detectMavlinkMessage(const uint8_t* buffer, int length, mavlink_message_t& msg, mavlink_status_t& status, int& discardedBytes);
void forwardMavlinkMessageToBT(const mavlink_message_t& msg);
void forwardMavlinkMessageToSerial(const mavlink_message_t& msg);
void clearTelemSerial();

void setup() {
  // Initialize Serial for debugging
  Serial.begin(57600);
  while (!Serial) {
    delay(1000); // Wait for USB serial to connect
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

  //update Display
  updateDisplay();
}

void setupBluetooth() {
  SerialBT.begin(BT_SPP_NAME); // Start Bluetooth SPP with specified name
  Serial.println("Bluetooth SPP initialized.");
}

void loop() {
  // Reset Watchdog Timer
  resetWDT();

  if (SerialBT.hasClient()) {
    if (!btConnected) {
      clearTelemSerial();  // Clear the telemetry buffer before handling
      btConnected = true;
      Serial.println("Client Connected, cleared Serial Buffer");
    }
    handleDataFlow();
  } else {
    btConnected = false;
   
  }

  // Update Display 
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= timeInterval) {
    // Serial.println("Updating Display");
    updateDisplay(); // Update the OLED display
    lastTime = currentTime;
  }

  // Report total bytes
  if (currentTime - lastReportTime >= reportInterval) {
    // reportTotalBytes();
    lastReportTime = currentTime;
  }
}

void Serial2BT() {
  // Handle data from Telemetry Serial to Bluetooth
  int bytesAvailable = TelemSerial.available(); 
  int bytes2read = min(bytesAvailable, SERIAL_BUFFER_SIZE);
  if (bytes2read>0) {
      //int bytes2read = min(TelemSerial.available(), SERIAL_BUFFER_SIZE); // SERIAL_BUFFER_SIZE = 32
      
      if (bytes2read < bytesAvailable) {
        Serial.print("Telem Bytes available > Buffer: ");
        Serial.println(TelemSerial.available());
      }

      int bytesRead = TelemSerial.readBytes(serialBuffer, bytes2read);
/* 
      // Ensure there are non-zero bytes
      bool hasNonZeroBytes = false;
      for (int i = 0; i < bytesRead; i++) {
        if (serialBuffer[i] != 0) {
          hasNonZeroBytes = true;
          break;
        }
      }
 */
      // Only write to Bluetooth if there are non-zero bytes
      //if (hasNonZeroBytes) {
      if (1) {
        unsigned long startTimeWrite = micros();  // Track the write time

        // Attempt to write data to Bluetooth
        int bytesWritten = SerialBT.write(serialBuffer, bytesRead);

        // Check if we successfully wrote the data and how long it took
        unsigned long endTimeWrite = micros();
        unsigned long durationWrite = endTimeWrite - startTimeWrite;

        // If duration is too long, consider introducing pacing here
        if (durationWrite > 10000) {
          Serial.println("Bluetooth write is slow, consider optimizing buffer or connection.");
        }
      } else {
        Serial.println("No non-zero bytes to send from Telemetry.");
      }
    }
}

/* void Serial2BT() {
  // Handle data from Telemetry Serial to Bluetooth
  if (TelemSerial.available()) {
      int bytes2read = min(TelemSerial.available(), SERIAL_BUFFER_SIZE);
      
      if (TelemSerial.available() > bytes2read) {
        Serial.print("Telem Bytes available > Buffer: ");
        Serial.println(TelemSerial.available());
      }

      int bytesRead = TelemSerial.readBytes(serialBuffer, bytes2read);

      // Iterate through the buffer and check for non-zero bytes
      bool hasNonZeroBytes = false;
      for (int i = 0; i < bytesRead; i++) {
        if (serialBuffer[i] != 0) {
          hasNonZeroBytes = true;  // Found non-zero byte
          break;  // No need to check further, exit the loop early
        }
      }

      // Only write to Bluetooth if there are non-zero bytes
      if (hasNonZeroBytes) {
        SerialBT.write(serialBuffer, bytesRead);
      } else {
        Serial.println("No non-zero bytes to send from Telemetry.");
      }
  }
} */

void BT2Serial() {
  // Handle data from Bluetooth to Telemetry Serial
  int bytesAvailable = SerialBT.available();
  if (bytesAvailable) {
    int bytes2read = min(bytesAvailable, CLIENT_BUFFER_SIZE);
    
    if (bytesAvailable > bytes2read) {
      Serial.print("BT Bytes available > Buffer: ");
      Serial.println(bytesAvailable);
    }

    int bytesRead = SerialBT.readBytes(clientBuffer, bytes2read);

    // Iterate through the buffer and check for non-zero bytes
    bool hasNonZeroBytes = false;
    for (int i = 0; i < bytesRead; i++) {
      if (clientBuffer[i] != 0) {
        hasNonZeroBytes = true;  // Found non-zero byte
        break;  // Exit the loop early when non-zero byte is found
      }
    }

    // Only write to Telemetry Serial if there are non-zero bytes
    if (hasNonZeroBytes) {
      TelemSerial.write(clientBuffer, bytesRead);
    } else {
      Serial.println("No non-zero bytes to send from Bluetooth.");
    }
  }
}

void handleDataFlow() {
  // Start timer for Serial2BT
  //unsigned long startTimeSerial2BT = micros();
  Serial2BT();  // Handle data from Telemetry Serial to Bluetooth
  //unsigned long endTimeSerial2BT = micros();

  // Start timer for BT2Serial
  //unsigned long startTimeBT2Serial = micros();
  BT2Serial();  // Handle data from Bluetooth to Telemetry Serial
  //unsigned long endTimeBT2Serial = micros();

  //unsigned long durationSerial2BT = endTimeSerial2BT - startTimeSerial2BT;
  //unsigned long durationBT2Serial = endTimeBT2Serial - startTimeBT2Serial;
  //Serial.println("Serial2BT: " + String(durationSerial2BT) + " us");
  //Serial.println("BT2Serial: " + String(durationBT2Serial) + " us");

  // delay(2);
}

// Function to reset the Watchdog Timer
void resetWDT() {
  esp_task_wdt_reset();
}

// Function to report total bytes transferred
void reportTotalBytes() {
  Serial.print("Total Bytes from Bluetooth: In: ");
  Serial.print(totalBTBytesIn);
  Serial.print(", Out: ");
  Serial.println(totalBTBytesOut);

  Serial.print("Total Bytes from Serial: In: ");
  Serial.print(totalSerialBytesIn);
  Serial.print(", Out: ");
  Serial.println(totalSerialBytesOut);

  // Reset the counters after reporting
  totalBTBytesIn = 0;
  totalBTBytesOut = 0;
  totalSerialBytesIn = 0;
  totalSerialBytesOut = 0;
}

void clearTelemSerial() {
  // Clear the telemetry serial buffer by reading and discarding data
  while (TelemSerial.available()) {
    TelemSerial.read();  // Read and discard each byte
  }
}