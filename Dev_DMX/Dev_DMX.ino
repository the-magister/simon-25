// Board: M5StackCore2 within M5Stack definitions (NOT esp32 definition)

// https://github.com/m5stack/M5Unified
#include <M5Unified.h>
// https://docs.m5stack.com/en/arduino/m5gfx/m5gfx_appendix
#include <M5GFX.h>
// https://github.com/sparkfun/SparkFunDMX/tree/master
//#include <HardwareSerial.h>
#include <SparkFunDMX.h>
// https://github.com/SofaPirate/Chrono
#include <Chrono.h>
// Handle DMX calls for light and fire
#include "DMX.h"

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// tedious to do this, but prevents a flicker from .clear() taking ~50ms to execute.
M5Canvas spr = M5Canvas(&M5.Display);

// heavy-lifting library attached as .cpp and .h files. grab this.  
DMX dmx;

void setup() {
  // https://docs.m5stack.com/en/arduino/m5gfx/m5gfx_appendix
  // https://github.com/m5stack/M5StickCPlus2
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  Serial.flush();

  spr.createSprite(M5.Display.width(), M5.Display.height());
  spr.setRotation(0);
  spr.setTextColor(WHITE);
  spr.setTextDatum(middle_center);
  spr.setTextFont(&fonts::FreeSans12pt7b);
  spr.setTextSize(1.0);
  spr.clear();

  Serial.println("DMX Test");
  dmx.begin();

  Serial.println("DMX initialized!");
}

void loop() {
  // check for presses and the like.
  M5.update();
  dmx.update();

  // rotate through which tower we're addressing
  static tower towerIndex = I_ALL;
  if (M5.BtnB.wasReleased()) {

    // shut down everything, first.  
    dmx.towerLight(I_ALL, cOff);
    dmx.towerFire(towerIndex, fOff);

    // change which tower we're addressing
    towerIndex = (tower)(((byte)towerIndex + 1) % ((byte)I_ALL + 1));
  }

  // Turn on all fire
  if (M5.BtnA.wasPressed()) dmx.towerFire(towerIndex, fOn);

  // Turn off all fire
  if (M5.BtnA.wasReleased()) dmx.towerFire(towerIndex, fOff);

  // check sensors
  static tower colorIndex = I_RED;
  static float avgIntensity = 0.0;
  static auto data = M5.Imu.getImuData();

  if (M5.Imu.update()) {
    data = M5.Imu.getImuData();

    // color by tilt
    if (data.accel.x < 0 && data.accel.y < 0) colorIndex = I_RED;
    if (data.accel.x > 0 && data.accel.y < 0) colorIndex = I_GRN;
    if (data.accel.x > 0 && data.accel.y > 0) colorIndex = I_YEL;
    if (data.accel.x < 0 && data.accel.y > 0) colorIndex = I_BLU;

    // degree of tilt
    float accZ = constrain(data.accel.z, 0.0, 1.0);
    float newIntensity = mapfloat(accZ, 0.0, 1.0, 255.0, 0.0);

    // smooth the sensor results; exponential smoother.
    const float smooth = 10.0;
    avgIntensity = (avgIntensity * (smooth - 1.0) + newIntensity) / smooth;
  }

  // send dmx
  static Chrono dmxUpdate;

  if (dmxUpdate.hasPassed(25UL, true)) {
    // turn on specific lights
    colorInstruction color = cMap[colorIndex];
    color.master = (byte)avgIntensity;

    dmx.towerLight(towerIndex, color);
  }

  // show sensors
  static Chrono displayUpdate;

  if (displayUpdate.hasPassed(100UL, true)) {

    spr.fillSprite(TFT_BLACK);
    spr.setCursor(0, 0);

    switch (towerIndex) {
      case I_RED:
        spr.setTextColor(RED);
        spr.drawString("Tower: Red", spr.width() / 2, spr.height() * 1 / 5);
        break;
      case I_GRN:
        spr.setTextColor(GREEN);
        spr.drawString("Tower: Green", spr.width() / 2, spr.height() * 1 / 5);
        break;
      case I_YEL:
        spr.setTextColor(YELLOW);
        spr.drawString("Tower: Yellow", spr.width() / 2, spr.height() * 1 / 5);
        break;
      case I_BLU:
        spr.setTextColor(BLUE);
        spr.drawString("Tower: Blue", spr.width() / 2, spr.height() * 1 / 5);
        break;
      case I_ALL:
        spr.setTextColor(WHITE);
        spr.drawString("Tower: ALL", spr.width() / 2, spr.height() * 1 / 5);
        break;
    }

    switch (colorIndex) {
      case I_RED:
        spr.setTextColor(RED);
        spr.drawString("Color: Red", spr.width() / 2, spr.height() * 2 / 5);
        break;
      case I_GRN:
        spr.setTextColor(GREEN);
        spr.drawString("Color: Green", spr.width() / 2, spr.height() * 2 / 5);
        break;
      case I_YEL:
        spr.setTextColor(YELLOW);
        spr.drawString("Color: Yellow", spr.width() / 2, spr.height() * 2 / 5);
        break;
      case I_BLU:
        spr.setTextColor(BLUE);
        spr.drawString("Color: Blue", spr.width() / 2, spr.height() * 2 / 5);
        break;
      case I_ALL:
        spr.setTextColor(WHITE);
        spr.drawString("Color: ALL", spr.width() / 2, spr.height() * 2 / 5);
        break;
    }

    String s = "Level: " + String(avgIntensity, 0);
    spr.setTextColor(WHITE);
    spr.drawString(s, spr.width() / 2, spr.height() * 3 / 5);

    switch (M5.BtnA.isPressed()) {
      case true:
        spr.setTextColor(WHITE);
        spr.drawString("FIRE", spr.width() / 2, spr.height() * 4 / 5);
        break;
    }

    spr.pushSprite(0, 0);
  }
}