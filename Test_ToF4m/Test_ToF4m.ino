/**
 * @file ContinuousWithDetails.ino
 * @author SeanKwok (shaoxiang@m5stack.com)
 * @brief M5Unit ToF4M Continuous Read Data With Details Demo.
 * @version 0.1
 * @date 2023-11-23
 *
 *
 * @Hardwares: M5Unit ToF4M
 * @Platform Version: Arduino M5Stack Board Manager v2.0.7
 * @Dependent Library:
 * VL53L1X: https://github.com/pololu/vl53l1x-arduino
 * M5Unified: https://github.com/m5stack/M5Unified
 */

#include "M5Unified.h"
#include <Wire.h>
#include <VL53L1X.h>

#define WIRE_SDA 32
#define WIRE_SCL 33

VL53L1X sensor;

void setup() {
    auto cfg = M5.config();
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);

    // M5.Ex_I2C.begin(21, 22, 400000);
    // M5.Ex_I2C.begin(WIRE_SDA, WIRE_SCL);
    Wire.begin(WIRE_SDA, WIRE_SCL);

    sensor.setBus(&Wire);
    sensor.setTimeout(500);
    if (!sensor.init()) {
        Serial.println("Failed to detect and initialize sensor!");
        while (1);
    }

  uint8_t address = sensor.getAddress();
  Serial.print("I2C address: ");
  Serial.println(address);

    // Use long distance mode and allow up to 50000 us (50 ms) for a
    // measurement. You can change these settings to adjust the performance of
    // the sensor, but the minimum timing budget is 20 ms for short distance
    // mode and 33 ms for medium and long distance modes. See the VL53L1X
    // datasheet for more information on range and timing limits.
//    sensor.setDistanceMode(VL53L1X::Long);
//    sensor.setMeasurementTimingBudget(50000);
    sensor.setDistanceMode(VL53L1X::Short);
    sensor.setMeasurementTimingBudget(20000);

    // Start continuous readings at a rate of one measurement every 50 ms (the
    // inter-measurement period). This period should be at least as long as the
    // timing budget.
    sensor.startContinuous(20);

    Serial.println("Range");
}

void loop() {
    sensor.read();

    Serial.print("Range:");
    Serial.print(sensor.ranging_data.range_mm);
 /*
    Serial.print("\tstatus: ");
    Serial.print(
        VL53L1X::rangeStatusToString(sensor.ranging_data.range_status));
    Serial.print("\tpeak signal: ");
    Serial.print(sensor.ranging_data.peak_signal_count_rate_MCPS);
    Serial.print("\tambient: ");
    Serial.print(sensor.ranging_data.ambient_count_rate_MCPS);
*/
    Serial.println();
}
