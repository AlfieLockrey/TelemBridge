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

// Function declarations
void setupBluetooth();
void handleDataFlow();
void handleBTtoSerial();
void handleSerialtoBT();
void resetWDT();
void clearBuffers();
void forwardMavlinkMessage(const mavlink_message_t& msg);
void reportLostPackets(const char* source, uint8_t actualSeq, uint8_t expectedSeq, uint8_t msgId);

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
  } else {
    clearBuffers();
    delay(50);
  }
}

void handleBTtoSerial() {
  // Handle incoming data from Bluetooth to Telemetry Radio
  int availableBytes = SerialBT.available();
  if (availableBytes > 0) {
    // Read only available bytes, but limit to buffer size
    int bytesToRead = min(availableBytes, static_cast<int>(CLIENT_BUFFER_SIZE));
    int bytesRead = SerialBT.readBytes(clientBuffer, bytesToRead);
    
    // Forward incoming MAVLink messages to Telemetry Serial
    for (int i = 0; i < bytesRead; i++) {
      TelemSerial.write(clientBuffer[i]);
      uploadedBytes++;
    }
  }
}

void handleSerialtoBT() {
  // Handle outgoing data from Telemetry Radio to Bluetooth
  int bytesAvailable = TelemSerial.available();
  if (bytesAvailable > 0) {
    // Read only available bytes, but limit to buffer size
    int bytesToRead = min(bytesAvailable, SERIAL_BUFFER_SIZE);
    int bytesRead = TelemSerial.readBytes(serialBuffer, bytesToRead);
    
    // Parse and forward valid MAVLink messages
    mavlink_message_t msg;
    mavlink_status_t status;

    for (int i = 0; i < bytesRead; ++i) {
      // Try to parse the MAVLink message
      if (mavlink_parse_char(MAVLINK_COMM_0, serialBuffer[i], &msg, &status)) {
        forwardMavlinkMessage(msg); // Forward recognized MAVLink messages

        lastSerialSeq = msg.seq; // Update the last received sequence number
      }
    }
    downloadedBytes += bytesRead; // Update downloaded byte count
  }
}

void loop() {
  // Reset Watchdog Timer
  resetWDT();

  // Handle serial data flow both ways
  handleDataFlow();

  // Update Display
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= timeInterval) {
    updateDisplay(); // Update the OLED display
    lastTime = currentTime;
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

// Function to forward MAVLink messages to Bluetooth and report the message type
void forwardMavlinkMessage(const mavlink_message_t& msg) {
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  int len = mavlink_msg_to_send_buffer(buffer, &msg);
  
  if (len > 0) {
    SerialBT.write(buffer, len); // Forward to Bluetooth

    // Additional handling based on the message type (if needed)
    const char* msgType = getMessageType(msg.msgid); // Get message type
    //Serial.print("Message Type: ");
    //Serial.println(msgType);
  }

  // Check for lost packets from BT
  if (msg.seq != (lastBTSeq + 1) % 256) { // Handling wrap-around
    lostBTPackets += (msg.seq - lastBTSeq + 256) % 256 - 1; // Count lost packets
    reportLostPackets("Telem", msg.seq, (lastBTSeq + 1) % 256, msg.msgid);
  }
  lastBTSeq = msg.seq; // Update the last received sequence number
}

// Function to report lost packets
void reportLostPackets(const char* source, uint8_t actualSeq, uint8_t expectedSeq, uint8_t msgId) {
  if (strcmp(source, "Telem") == 0 && lostBTPackets > 0) {
    Serial.print("Lost Packets from Telem: ");
    Serial.print(lostBTPackets);
    Serial.print(" (Last Seq: ");
    Serial.print(lastBTSeq);
    Serial.print(", Expec. Seq: ");
    Serial.print(expectedSeq);
    Serial.print(", Act. Seq: ");
    Serial.print(actualSeq);
    Serial.print(", MSG ID: ");
    Serial.print(msgId);
    Serial.print(", MSG Type: ");
    Serial.println(getMessageType(msgId));
    lostBTPackets = 0; // Reset count after reporting
  } else if (strcmp(source, "Serial") == 0 && lostSerialPackets > 0) {
    Serial.print("Lost Packets from Serial: ");
    Serial.print(lostSerialPackets);
    Serial.print(" (Last Serial Seq: ");
    Serial.print(lastSerialSeq);
    Serial.print(", Expected Seq: ");
    Serial.print(expectedSeq);
    Serial.print(", Actual Seq: ");
    Serial.print(actualSeq);
    Serial.print(", Message ID: ");
    Serial.print(msgId);
    Serial.print(", Message Type: ");
    Serial.println(getMessageType(msgId));
    lostSerialPackets = 0; // Reset count after reporting
  }
}
