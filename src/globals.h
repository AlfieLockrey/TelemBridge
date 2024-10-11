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
#define TCP_PORT 5760
#define REPORT_INTERVAL 10000 // in milliseconds
#define BUFFER_SIZE 4096 // Buffer size for read/write operations

// Non-performance related configuration
#define WDT_TIMEOUT 10 // Watchdog Timer timeout in seconds

#endif // GLOBALS_H
