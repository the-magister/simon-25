/**
 * @file button.ino
 * @author SeanKwok (shaoxiang@m5stack.com)
 * @brief M5StickCPlus2 Button Test
 * @version 0.1
 * @date 2023-12-09
 *
 *
 * @Hardwares: M5StickCPlus2
 * @Platform Version: Arduino M5Stack Board Manager v2.0.9
 * @Dependent Library:
 * M5GFX: https://github.com/m5stack/M5GFX
 * M5Unified: https://github.com/m5stack/M5Unified
 * M5StickCPlus2: https://github.com/m5stack/M5StickCPlus2
 */

// https://docs.m5stack.com/en/arduino/m5stickc_plus2/program

#include "M5Unified.h"
#include "M5GFX.h"

void setup() {
    // https://docs.m5stack.com/en/arduino/m5gfx/m5gfx_appendix
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(3);
    M5.Display.setTextColor(PURPLE);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextFont(&fonts::FreeSans12pt7b);
    M5.Display.setTextSize(1.3);
    M5.Display.drawString("Button Test", M5.Display.width() / 2,
                                M5.Display.height() / 2);
}

void loop() {
    M5.update();
    if (M5.BtnA.wasPressed()) {
        M5.Speaker.tone(8000, 20);
        M5.Display.clear();
        M5.Display.drawString("A Btn Pressed",
                                    M5.Display.width() / 2,
                                    M5.Display.height() / 2);
    }
    if (M5.BtnA.wasReleased()) {
        M5.Speaker.tone(8000, 20);
        M5.Display.clear();
        M5.Display.drawString("A Btn Released",
                                    M5.Display.width() / 2,
                                    M5.Display.height() / 2);
    }
    if (M5.BtnB.wasPressed()) {
        M5.Speaker.tone(8000, 20);
        M5.Display.clear();
        M5.Display.drawString("B Btn Pressed",
                                    M5.Display.width() / 2,
                                    M5.Display.height() / 2);
    }
    if (M5.BtnB.wasReleased()) {
        M5.Speaker.tone(8000, 20);
        M5.Display.clear();
        M5.Display.drawString("B Btn Released",
                                    M5.Display.width() / 2,
                                    M5.Display.height() / 2);
    }
}
