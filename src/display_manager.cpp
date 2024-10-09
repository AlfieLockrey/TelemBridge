// display_manager.cpp
#include "display_manager.h"
#include <Adafruit_SSD1306.h>
#include "TinyPICO.h"
#include "pin_definitions.h"
#include "globals.h"
#include <Wire.h>

// External Objects and Variables
extern Adafruit_SSD1306 display;
extern TinyPICO tp;
extern volatile int uploadRate;
extern volatile int downloadRate;

// Define constants
const int downShift = 7; // Number of pixels to shift the display down
const int sidePadding = 4; // Number of pixels to pad the display from the sides
const int ICON_SIZE = 15;
const int TEXT_SIZE = 2;
const int TEXT_OFFSET_X = 18; // Adjust as needed
const int TEXT_OFFSET_Y_UPLOAD = 18;
const int TEXT_OFFSET_Y_DOWNLOAD = 36;

// 'Bluetooth-Logo-PNG', 15x15px
const unsigned char Bluetooth_logo [] PROGMEM = {
	0x01, 0x80, 0x01, 0xc0, 0x01, 0xe0, 0x01, 0xb0, 0x0d, 0xb0, 0x07, 0xe0, 0x03, 0xc0, 0x03, 0x80, 
	0x03, 0xc0, 0x07, 0xe0, 0x0d, 0xb0, 0x09, 0xb0, 0x01, 0xe0, 0x01, 0xc0, 0x01, 0x80
};
const unsigned char BattIconNormal [] PROGMEM = {
  0x01, 0x01, 0x07, 0xc1, 0x04, 0x41, 0x04, 0x41, 0x04, 0x41, 0x04, 0x41, 0x04, 0x41, 0x07, 0xc1, 
  0x07, 0xc1, 0x07, 0xc1, 0x07, 0xc1, 0x04, 0x41, 0x07, 0xc1, 0x07, 0xc1, 0x07, 0xc1
};
const unsigned char BattIconCharging [] PROGMEM = {
	0x00, 0x00, 0x00, 0x04, 0x00, 0x18, 0x00, 0x30, 0x00, 0xe0, 0x01, 0xc0, 0x07, 0x80, 0x0f, 0xf0, 
	0x3f, 0xc0, 0x07, 0x80, 0x0e, 0x00, 0x1c, 0x00, 0x30, 0x00, 0x60, 0x00, 0x00, 0x00
};
const unsigned char USBIcon [] PROGMEM = {
	0x00, 0x00, 0x0f, 0xe0, 0x08, 0x20, 0x0e, 0xe0, 0x0e, 0xe0, 0x08, 0x20, 0x1f, 0xf0, 0x10, 0x10, 
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0f, 0xe0, 0x00, 0x00
};
const unsigned char UparrowIcon [] PROGMEM = {
	0x03, 0x80, 0x07, 0xc0, 0x0f, 0xe0, 0x1f, 0xf0, 0x3f, 0xf8, 0x7f, 0xfc, 0xff, 0xfe, 0xf7, 0xde, 
	0x67, 0xcc, 0x07, 0xc0, 0x07, 0xc0, 0x07, 0xc0, 0x07, 0xc0, 0x07, 0xc0, 0x03, 0x80
};
const unsigned char DownarrowIcon [] PROGMEM = {
	0x03, 0x80, 0x07, 0xc0, 0x07, 0xc0, 0x07, 0xc0, 0x07, 0xc0, 0x07, 0xc0, 0x67, 0xcc, 0xf7, 0xde, 
	0xff, 0xfe, 0x7f, 0xfc, 0x3f, 0xf8, 0x1f, 0xf0, 0x0f, 0xe0, 0x07, 0xc0, 0x03, 0x80
};


// Battery Voltage Averager (Circular Buffer)
#define MAX_VOLTAGE_READINGS 50

float voltageReadings[MAX_VOLTAGE_READINGS];
int voltageIndex = 0;
float voltageSum = 0;
int voltageCount = 0;

// Charging Status Averager (Circular Buffer)
#define MAX_CHARGING_STATUS_READINGS 10

bool chargingStatusReadings[MAX_CHARGING_STATUS_READINGS];
int chargingIndex = 0;
int chargingCount = 0;

// Function to get the average battery voltage
float getAverageVoltage() {
  float voltage = tp.GetBatteryVoltage();

  // Subtract the oldest reading from the sum
  voltageSum -= voltageReadings[voltageIndex];

  // Add the new reading to the sum
  voltageReadings[voltageIndex] = voltage;
  voltageSum += voltageReadings[voltageIndex];

  // Move to the next index
  voltageIndex = (voltageIndex + 1) % MAX_VOLTAGE_READINGS;

  // Increment count up to MAX_VOLTAGE_READINGS
  if (voltageCount < MAX_VOLTAGE_READINGS) {
    voltageCount++;
  }

  // Calculate average
  return voltageSum / voltageCount;
}

// Function to get the smoothed charging status
bool getSmoothedChargingStatus() {
  bool isCharging = tp.IsChargingBattery();

  // Subtract the oldest reading from the count
  if (chargingStatusReadings[chargingIndex]) {
    chargingCount--;
  }

  // Add the new reading
  chargingStatusReadings[chargingIndex] = isCharging;
  if (isCharging) {
    chargingCount++;
  }

  // Move to the next index
  chargingIndex = (chargingIndex + 1) % MAX_CHARGING_STATUS_READINGS;

  // Determine if charging based on the majority
  return chargingCount > (MAX_CHARGING_STATUS_READINGS / 2);
}

// Function to display battery and Bluetooth status
void displayBatteryStatus(bool isCharging, bool isUsbPowered, float voltage) {
  // Choose the appropriate icon
  if (isUsbPowered && isCharging) {
    display.drawBitmap(sidePadding, downShift, BattIconCharging, ICON_SIZE, ICON_SIZE, WHITE);
  } else if (isUsbPowered) {
    display.drawBitmap(sidePadding, downShift, USBIcon, ICON_SIZE, ICON_SIZE, WHITE);
  } else {
    display.drawBitmap(sidePadding, downShift, BattIconNormal, ICON_SIZE, ICON_SIZE, WHITE);
  }

  // Display Voltage
  char voltageStr[10];
  dtostrf(voltage, 4, 2, voltageStr);
  display.setTextSize(TEXT_SIZE);
  display.setCursor(sidePadding + ICON_SIZE + 2, downShift);
  display.print(voltageStr);
  display.println("V");

  // Display Bluetooth Status
  display.drawBitmap(96 - sidePadding, downShift, Bluetooth_logo, ICON_SIZE, ICON_SIZE, WHITE);
  display.setCursor(112 - sidePadding, downShift);
  display.println("0"); // Update with actual connected devices
}

// Function to display upload and download rates
void displayNetworkStatus(int uploadRate, int downloadRate) {
  // Upload Status
  display.drawBitmap(sidePadding, 18 + downShift, UparrowIcon, ICON_SIZE, ICON_SIZE, WHITE);
  char uploadStr[10];
  sprintf(uploadStr, "%d", uploadRate);
  display.setTextSize(TEXT_SIZE);
  display.setCursor(TEXT_OFFSET_X + sidePadding, 18 + downShift);
  display.print(uploadStr);

  display.setCursor(85 - sidePadding, 18 + downShift);
  display.println("Bps");

  // Download Status
  display.drawBitmap(sidePadding, 36 + downShift, DownarrowIcon, ICON_SIZE, ICON_SIZE, WHITE);
  char downloadStr[10];
  sprintf(downloadStr, "%d", downloadRate);
  display.setCursor(TEXT_OFFSET_X + sidePadding, 36 + downShift);
  display.print(downloadStr);

  display.setCursor(85 - sidePadding, 36 + downShift);
  display.println("Bps");
}

// Function to update the OLED display
void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(WHITE);

  bool isCharging = getSmoothedChargingStatus();
  bool isUsbPowered = digitalRead(USB_POWER_PIN) == HIGH;

  float batteryVoltage = getAverageVoltage();

  // Display Battery and Bluetooth Status
  displayBatteryStatus(isCharging, isUsbPowered, batteryVoltage);

  // Display Upload and Download Rates
  displayNetworkStatus(uploadRate, downloadRate);

  display.display();
}

// Function to setup the OLED display
void setupDisplay() {
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(USB_POWER_PIN, INPUT_PULLUP); // Configure USB power detection pin

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1); // Infinite loop to halt the program
  }

  display.clearDisplay();
  display.setTextSize(TEXT_SIZE);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Booting...");
  display.display(); // Render the initial message
}
