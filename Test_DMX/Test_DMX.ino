/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */

// Board: M5StickCPlus2 within M5Stack definitions (NOT esp32 definition)


// https://github.com/m5stack/M5Unified
#include <M5Unified.h>
// https://docs.m5stack.com/en/arduino/m5gfx/m5gfx_appendix
#include <M5GFX.h>
// https://github.com/sparkfun/SparkFunDMX/tree/master
#include <SparkFunDMX.h>
// https://github.com/SofaPirate/Chrono
#include <Chrono.h>

// Create DMX object
SparkFunDMX dmx;
// Create serial port to be used for DMX interface.
HardwareSerial dmxSerial(2);
// Serial pinout
const uint8_t enPin = -1, rxPin = G33, txPin = G32;
// Number of DMX channels, can be up tp 512
const uint16_t numChannels = 100;

// counting things
uint8_t sendCounter = 0;

// relative locations for the colors on the Simon Console
enum color {
  I_RED = 0,  // upper right
  I_GRN,      // upper left
  I_BLU,      // lower right
  I_YEL,      // lower left
  N_COLORS,   // use this to size arrays appropriately

  I_START = N_COLORS,  // 4
  I_RIGHT,
  I_LEFT,
  N_BUTTONS
};

typedef struct {
  byte master;
  byte red;
  byte green;
  byte blue;
  byte white;
} colorDMX;

const colorDMX cOff = { 0, 0, 0, 0, 0 };
const colorDMX cRed = { 255, 255, 0, 0, 0 };
const colorDMX cGreen = { 255, 0, 255, 0, 0 };
const colorDMX cBlue = { 255, 0, 0, 255, 0 };
const colorDMX cYellow = { 255, 255, 100, 0, 0 };
const colorDMX cWhite = { 255, 0, 0, 0, 255 };
// and this serves as an easy way to pull out the right RGB color from the
const colorDMX cMap[N_COLORS] = { cRed, cGreen, cBlue, cYellow };

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// tedious to do this, but prevents a flicker from .clear() taking ~50ms to execute.
M5Canvas spr = M5Canvas(&M5.Display);

void setup() {
  // https://docs.m5stack.com/en/arduino/m5gfx/m5gfx_appendix
  // https://github.com/m5stack/M5StickCPlus2
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  Serial.flush();

  spr.createSprite(M5.Display.width(), M5.Display.height());
  spr.setRotation(2);
  spr.setTextColor(WHITE);
  spr.setTextDatum(middle_center);
  spr.setTextFont(&fonts::FreeSans12pt7b);
  spr.setTextSize(1.0);
  spr.clear();

  Serial.println("DMX Test");

  // Begin DMX serial port
  dmxSerial.begin(DMX_BAUD, DMX_FORMAT, rxPin, txPin);

  // Begin DMX driver
  dmx.begin(dmxSerial, enPin, numChannels);

  // Set communicaiton direction, which can be changed on the fly as needed
  dmx.setComDir(DMX_WRITE_DIR);

  Serial.println("DMX initialized!");

}

void loop() {
  // check for presses and the like.
  M5.update();

  // keep the DMX gear in sync; some drop out without a periodic update.
  static Chrono dmxHeartbeat;
  if (dmxHeartbeat.hasPassed(1000UL, true)) dmx.update();

  // rotate through which tower we're addressing
  static byte towerIndex = 0;
  if (M5.BtnB.wasReleased()) {
    // change which tower we're addressing
    towerIndex = (towerIndex + 1) % N_COLORS;
  }

  // Turn on all fire
  if (M5.BtnA.wasPressed()) {
    // clear the fire
    dmx.writeByte(0, (0 * 10) + 9);
    dmx.writeByte(0, (0 * 10) + 10);
    dmx.writeByte(0, (1 * 10) + 9);
    dmx.writeByte(0, (1 * 10) + 10);
    dmx.writeByte(0, (2 * 10) + 9);
    dmx.writeByte(0, (2 * 10) + 10);
    dmx.writeByte(0, (3 * 10) + 9);
    dmx.writeByte(0, (3 * 10) + 10);

    // write fire to the correct tower
    dmx.writeByte(255, (towerIndex * 10) + 9);
    dmx.writeByte(255, (towerIndex * 10) + 10);

    dmx.update();            // send it
    dmxHeartbeat.restart();  // no need to resend
  }

  // Turn on all fire
  if (M5.BtnA.wasReleased()) {
    dmx.writeByte(0, (towerIndex * 10) + 9);
    dmx.writeByte(0, (towerIndex * 10) + 10);

    dmx.update();            // send it
    dmxHeartbeat.restart();  // no need to resend
  }

  // check sensors
  static color newColor = I_RED, currentColor = I_RED;
  static float avgIntensity = 0.0;
  static auto data = M5.Imu.getImuData();

  if (M5.Imu.update()) {
    data = M5.Imu.getImuData();

    if (data.accel.x > 0 && data.accel.y > 0) newColor = I_RED;
    if (data.accel.x < 0 && data.accel.y > 0) newColor = I_GRN;
    if (data.accel.x < 0 && data.accel.y < 0) newColor = I_YEL;
    if (data.accel.x > 0 && data.accel.y < 0) newColor = I_BLU;

    float accZ = constrain(data.accel.z, 0.0, 1.0);
    float newIntensity = mapfloat(accZ, 0.0, 1.0, 255.0, 0.0);

    // smooth the sensor results; exponential smoother.
    const float smooth = 10;
    avgIntensity = (avgIntensity * (smooth - 1.0) + newIntensity) / smooth;
  }

  // send dmx
  static Chrono dmxUpdate;

  if (dmxUpdate.hasPassed(50UL, true)) {

    // clear the lights
    dmx.writeBytes((uint8_t*)&cOff, sizeof(colorDMX), (0 * 10) + 1);
    dmx.writeBytes((uint8_t*)&cOff, sizeof(colorDMX), (1 * 10) + 1);
    dmx.writeBytes((uint8_t*)&cOff, sizeof(colorDMX), (2 * 10) + 1);
    dmx.writeBytes((uint8_t*)&cOff, sizeof(colorDMX), (3 * 10) + 1);

    // get our color
    colorDMX color = cMap[newColor];
    color.master = (byte)avgIntensity;

    // write this color to the correct tower.
    dmx.writeBytes((uint8_t*)&color, sizeof(colorDMX), (towerIndex * 10) + 1);

    dmx.update();
    dmxHeartbeat.restart();
  }

  // show sensors
  static Chrono displayUpdate;

  if (displayUpdate.hasPassed(100UL, true)) {

//    M5.Display.clear();
    spr.fillSprite(TFT_BLACK);
    spr.setCursor(0,0);

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
    }

    switch (newColor) {
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

    spr.pushSprite(0,0);
  }
}