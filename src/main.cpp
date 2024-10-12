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
unsigned long reportInterval = 10000; // 10 seconds reporting interval

// Function declarations
void setupBluetooth();
void handleDataFlow();
void handleBTtoSerial();
void handleSerialtoBT();
void resetWDT();
void clearBuffers();
void reportLostPackets(const char* source, uint8_t actualSeq, uint8_t expectedSeq, uint8_t msgId);
void reportTotalBytes();
bool detectMavlinkMessage(const uint8_t* buffer, int length, mavlink_message_t& msg, mavlink_status_t& status, int& discardedBytes);
void forwardMavlinkMessageToBT(const mavlink_message_t& msg);
void forwardMavlinkMessageToSerial(const mavlink_message_t& msg);

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

void handleDataFlow() {
  // If client connected, handle data flow. Else clear the buffers.
  if (SerialBT.connected()) {
    handleBTtoSerial();   // Handle data from Bluetooth to Telemetry Serial
    handleSerialtoBT();   // Handle data from Telemetry Serial to Bluetooth
  }
}

void handleSerialtoBT() {
  int bytesAvailable = TelemSerial.available();  // Check how many bytes are available in the buffer

  while (bytesAvailable > 0) {
    int bytesToRead = min(bytesAvailable, SERIAL_BUFFER_SIZE);  // Read up to buffer size
    int bytesRead = TelemSerial.readBytes(serialBuffer, bytesToRead);  // Read multiple bytes

    totalSerialBytesIn += bytesRead; // Track total bytes received from Serial

    mavlink_message_t msg;
    static mavlink_status_t status;  // Persistent status
    int discardedBytes = 0;

    // Detect MAVLink message and handle discarded bytes
    if (detectMavlinkMessage(serialBuffer, bytesRead, msg, status, discardedBytes)) {
      // Forward valid MAVLink message to Bluetooth
      forwardMavlinkMessageToBT(msg);
      lastSerialSeq = msg.seq;
    }

    // Update the count of downloaded bytes and account for discarded bytes
    downloadedBytes += (bytesRead - discardedBytes);

    // Recheck how many bytes are still available after processing
    bytesAvailable = TelemSerial.available();
  }
}

void handleBTtoSerial() {
  // Handle incoming data from Bluetooth to Telemetry Radio
  int availableBytes = SerialBT.available();
  if (availableBytes > 0) {
    // Read only available bytes, but limit to buffer size
    int bytesToRead = min(availableBytes, static_cast<int>(CLIENT_BUFFER_SIZE));
    int bytesRead = SerialBT.readBytes(clientBuffer, bytesToRead);

    totalBTBytesIn += bytesRead; // Track total bytes received from Bluetooth

    mavlink_message_t msg;
    static mavlink_status_t status;
    int discardedBytes = 0;

    // Detect MAVLink message and handle discarded bytes
    if (detectMavlinkMessage(clientBuffer, bytesRead, msg, status, discardedBytes)) {
      // Forward valid MAVLink message to Telemetry Serial
      forwardMavlinkMessageToSerial(msg);
    }

    // Account for bytes successfully sent to serial
    uploadedBytes += (bytesRead - discardedBytes);
  }
}

void loop() {
  // Reset Watchdog Timer
  resetWDT();

  // Handle serial data flow both ways
  handleDataFlow();

  // Update Display every 5 seconds
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= timeInterval) {
    Serial.println("Updating Display");
    updateDisplay(); // Update the OLED display
    lastTime = currentTime;
  }

  // Report total bytes every 10 seconds
  if (currentTime - lastReportTime >= reportInterval) {
    reportTotalBytes();
    lastReportTime = currentTime;
  }
}

// Function to reset the Watchdog Timer
void resetWDT() {
  esp_task_wdt_reset();
}

void clearBuffers() {
  memset(clientBuffer, 0, sizeof(clientBuffer));  // Clear client buffer
  memset(serialBuffer, 0, sizeof(serialBuffer));  // Clear serial buffer
  uploadedBytes = 0;
  downloadedBytes = 0;
}

// Function to forward a MAVLink message to Bluetooth
void forwardMavlinkMessageToBT(const mavlink_message_t& msg) {
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int len = mavlink_msg_to_send_buffer(buffer, &msg);

  if (len > 0) {
    SerialBT.write(buffer, len);
    totalBTBytesOut += len; // Track total bytes sent to Bluetooth

    const char* msgType = getMessageType(msg.msgid);

    if (msg.seq != (lastBTSeq + 1) % 256) { // Handling wrap-around
      lostBTPackets += (msg.seq - lastBTSeq + 256) % 256 - 1;
      reportLostPackets("Telem", msg.seq, (lastBTSeq + 1) % 256, msg.msgid);
    }
    lastBTSeq = msg.seq;
  } else {
    Serial.println("Failed to convert MAVLink message.");
  }
}

// Function to forward a MAVLink message to Telemetry Serial
void forwardMavlinkMessageToSerial(const mavlink_message_t& msg) {
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int len = mavlink_msg_to_send_buffer(buffer, &msg);

  if (len > 0) {
    TelemSerial.write(buffer, len);
    totalSerialBytesOut += len; // Track total bytes sent to Telemetry Serial
  } else {
    Serial.println("Failed to convert MAVLink message for serial forwarding.");
  }
}

bool detectMavlinkMessage(const uint8_t* buffer, int length, mavlink_message_t& msg, mavlink_status_t& status, int& discardedBytes) {
  discardedBytes = 0;
  for (int i = 0; i < length; ++i) {
    if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
      // A complete MAVLink message has been detected
      return true;
    } else {
      // No complete MAVLink message, this byte might be discarded
      discardedBytes++;
    }
  }
  // No full message detected, all bytes might have been discarded
  return false;
}

// Function to report lost packets
void reportLostPackets(const char* source, uint8_t actualSeq, uint8_t expectedSeq, uint8_t msgId) {
  Serial.print("Lost packet from ");
  Serial.print(source);
  Serial.print(" (");
  Serial.print(lostBTPackets);
  Serial.print(") : Expected Seq: ");
  Serial.print(expectedSeq);
  Serial.print(", Actual Seq: ");
  Serial.print(actualSeq);
  Serial.print(", Message ID: ");
  Serial.print(msgId);
  Serial.print(", MSG Type: ");
  Serial.println(getMessageType(msgId));
  lostBTPackets = 0;
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
