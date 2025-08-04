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

void setup() {
  // https://docs.m5stack.com/en/arduino/m5gfx/m5gfx_appendix
  // https://github.com/m5stack/M5StickCPlus2
  auto cfg = M5.config();
  M5.begin(cfg);

  M5.Display.setRotation(3);
  M5.Display.setTextColor(BLUE);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextFont(&fonts::FreeSans12pt7b);
  M5.Display.setTextSize(1.3);
  M5.Display.clear();
  M5.Display.drawString("DMX Test", M5.Display.width() / 2, M5.Display.height() / 2);

  Serial.println("SparkFun DMX Example 1 - Output");

  // Begin DMX serial port
  dmxSerial.begin(DMX_BAUD, DMX_FORMAT, rxPin, txPin);

  // Begin DMX driver
  dmx.begin(dmxSerial, enPin, numChannels);

  // Set communicaiton direction, which can be changed on the fly as needed
  dmx.setComDir(DMX_WRITE_DIR);

  Serial.println("DMX initialized!");

  // set masters up.
  dmx.writeByte(255, 1);
  dmx.writeByte(255, 11);
  dmx.update();
}

void loop() {

  // check for presses and the like.
  M5.update();

  // keep the DMX gear in sync; some drop out without a periodic update.
  static Chrono dmxUpdate;
  if (dmxUpdate.hasPassed(1000UL, true)) dmx.update();

  if (M5.BtnB.wasPressed()) {
    // Turn on all fire
    dmx.writeByte(255, 21);
    dmx.writeByte(255, 22);

    dmx.update(); // send it
    dmxUpdate.restart(); // no need to resend

    M5.Display.clear();
    M5.Display.drawString("Fire On", M5.Display.width() / 2, M5.Display.height() / 3);
  }

  if (M5.BtnB.wasReleased()) {
    // Turn on all fire
    dmx.writeByte(0, 21);
    dmx.writeByte(0, 22);

    dmx.update(); // send it
    dmxUpdate.restart(); // no need to resend

    M5.Display.clear();
    M5.Display.drawString("Fire Off", M5.Display.width() / 2, M5.Display.height() / 3);
  }

  if (M5.BtnA.wasPressed()) {
    // Turn on all lamps
    dmx.writeByte(255, 2);
    dmx.writeByte(255, 3);
    dmx.writeByte(255, 4);
    dmx.writeByte(255, 5);

    dmx.writeByte(255, 12);
    dmx.writeByte(255, 13);
    dmx.writeByte(255, 14);
    dmx.writeByte(255, 15);

    dmx.update();
    dmxUpdate.restart();

    M5.Display.clear();
    M5.Display.drawString("Light On", M5.Display.width() / 2, M5.Display.height() * 2 / 3);
  }

  if (M5.BtnA.wasReleased()) {
    // turn off all lamps
    dmx.writeByte(0, 2);
    dmx.writeByte(0, 3);
    dmx.writeByte(0, 4);
    dmx.writeByte(0, 5);

    dmx.writeByte(0, 12);
    dmx.writeByte(0, 13);
    dmx.writeByte(0, 14);
    dmx.writeByte(0, 15);

    dmx.update();
    dmxUpdate.restart();

    M5.Display.clear();
    M5.Display.drawString("Light Off", M5.Display.width() / 2, M5.Display.height() * 2 / 3);
  }
}