#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <senseBoxIO.h>
#include "SenseBoxMCU.h"

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

BMX055 bmx;

// defining screen dimensions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

void setup() {
  // init display
  senseBoxIO.powerI2C(true);
  delay(2000);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  display.display();
  delay(100);
  display.clearDisplay();

  // init BMX055
  bmx.begin();
}

void loop() {
  // clear screen
  display.fillScreen(0);

  // get sensor values
  int XA, YA, ZA; // accelerometer axis
  bmx.getAcceleration(&XA, &YA, &ZA);

  // map sensor values to screen dimensions
  int xMap = map(XA, -1000, 1000, SCREEN_WIDTH, 0);
  int yMap = map(YA, -1000, 1000, 0, SCREEN_HEIGHT);

  // drawing dot
  display.fillCircle(xMap, yMap, 3, 1);
  display.display();
}
