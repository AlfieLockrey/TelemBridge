// include/globals.h
#ifndef GLOBALS_H
#define GLOBALS_H

#include <Adafruit_SSD1306.h>
#include <TinyPICO.h>

// External Objects
extern Adafruit_SSD1306 display;
extern TinyPICO tp;

// External Variables
extern volatile int uploadRate;
extern volatile int downloadRate;

// Configuration parameters (Performance Based)
#define BT_SPP_NAME "TelemBridge_BT" // Bluetooth SPP name
#define REPORT_INTERVAL 10000 // in milliseconds
#define UPDATE_INTERVAL 10000 // in milliseconds

// Buffer size for read/write operations
/* 
#define SERIAL_BUFFER_SIZE 280 // Max size of a mavlink2 message
#define CLIENT_BUFFER_SIZE 280 // Max size of a mavlink2 message
 */
#define SERIAL_BUFFER_SIZE 120 // BT SPP 120 Byte Limit
#define CLIENT_BUFFER_SIZE 120

// Non-performance related configuration
#define WDT_TIMEOUT 15 // Watchdog Timer timeout in seconds

#endif // GLOBALS_H