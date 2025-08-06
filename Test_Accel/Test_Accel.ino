/**
 * @file imu.ino
 * @author SeanKwok (shaoxiang@m5stack.com)
 * @brief M5StickCPlus2 get IMU data
 * @version 0.1
 * @date 2023-12-19
 *
 *
 * @Hardwares: M5StickCPlus2
 * @Platform Version: Arduino M5Stack Board Manager v2.0.9
 */

/*
 * @Dependent Library:
 * (deprecated) M5StickCPlus2: https://github.com/m5stack/M5StickCPlus2
 * M5Unified: https://github.com/m5stack/M5Unified
 * M5GFX: https://github.com/m5stack/M5GFX
 * Chrono: https://github.com/SofaPirate/Chrono
 */
#include <M5Unified.h>
#include <M5GFX.h>
#include <Chrono.h>

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

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(2);
  M5.Display.setTextColor(BLUE);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setFont(&fonts::FreeSansBold9pt7b);
  M5.Display.setTextSize(1.5);
}

void loop(void) {

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

  // show sensors
  static Chrono dmxUpdate;

  if (dmxUpdate.hasPassed(100UL, true)) {

    M5.Display.setCursor(0, 40);
    M5.Display.clear();   

    Serial.printf("ax:%f  ay:%f  az:%f\r\n", data.accel.x, data.accel.y, data.accel.z);
    Serial.printf("gx:%f  gy:%f  gz:%f\r\n", data.gyro.x, data.gyro.y, data.gyro.z);

    M5.Display.printf(" %0.2f %0.2f\r\n %0.2f\r\n", data.accel.x, data.accel.y, data.accel.z);

    M5.Display.printf(" %0d\r\n", newColor);
    M5.Display.printf(" %d\r\n", (byte)avgIntensity);
  }
}