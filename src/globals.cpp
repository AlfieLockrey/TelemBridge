// globals.cpp
#include "globals.h"

// Define the Adafruit SSD1306 display object
Adafruit_SSD1306 display(128, 64, &Wire, -1);

// Define other global objects and variables
TinyPICO tp;

volatile int uploadRate = 0;
volatile int downloadRate = 0;

