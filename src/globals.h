// globals.h
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

// USB Switch-Over Control Flag
extern const bool ENABLE_USB_SWITCHOVER;

#endif // GLOBALS_H
