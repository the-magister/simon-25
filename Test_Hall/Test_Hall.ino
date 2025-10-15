/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
 * @Hardwares: M5Core + Unit Hall
 * @Platform Version: Arduino M5Stack Board Manager v2.1.3
 * @Dependent Library:
 * M5Stack@^0.4.6: https://github.com/m5stack/M5Stack
 */
#include "M5Unified.h"
#include "M5GFX.h"

#define HALL 33

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(3);
  M5.Display.setTextColor(PURPLE);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextFont(&fonts::FreeSans12pt7b);
  M5.Display.setTextSize(1.3);

  M5.Display.drawString("Hall Switch Test", M5.Display.width() / 2,
                        M5.Display.height() / 2);

  pinMode(HALL, INPUT);  // Set the pins to which the Hall sensor is connected to
                         // the input mode.  将霍尔传感器所连接的引脚设置为输入模式
}

void loop() {
  M5.update();

  static bool lastStatus = !digitalRead(HALL);
  bool status = digitalRead(HALL);

  if (status != lastStatus) {
    M5.Display.clear();
    M5.Speaker.tone(8000, 20);
    if (status) {
      M5.Display.drawString("Hall: 1",
                            M5.Display.width() / 2,
                            M5.Display.height() / 2);
    } else {
      M5.Display.drawString("Hall: 0",
                            M5.Display.width() / 2,
                            M5.Display.height() / 2);
    }
  }

  lastStatus = status;
}