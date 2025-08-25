/*
 * SPDX-FileCopyrightText: 2025 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
/*
 * @Hardwares: M5Core + Unit IR
 * @Platform Version: Arduino M5Stack Board Manager v2.1.3
 * @Dependent Library:
 * M5Stack@^0.4.6: https://github.com/m5stack/M5Stack
 */

#include <M5Unified.h>
#include <Chrono.h>

int ir_recv_pin = 33;  // set the input pin.  设置引脚
int ir_send_pin = 32;

void setup()
{
    auto cfg = M5.config();
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);

    M5.Power.begin();

    pinMode(ir_recv_pin, INPUT);
    pinMode(ir_send_pin, OUTPUT);
  
 //   digitalWrite(ir_send_pin, 1);

    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print("Test for IR receiver: ");
}

void loop()
{
    static Chrono pulseIR;
    if( pulseIR.hasPassed(100UL) ) {
      // pulse light.
      digitalWrite(ir_send_pin, 1);
      delay(2);
      byte curr_val = digitalRead(ir_recv_pin);
      digitalWrite(ir_send_pin, 0);
      
      Serial.print(millis());
      Serial.print(" -> ");
      Serial.println(curr_val);

      pulseIR.restart();
    }

}