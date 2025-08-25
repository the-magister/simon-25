/* This example shows how to use continuous mode to take
range measurements with the VL53L0X. It is based on
vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.

The range readings are in units of mm. */


// https://github.com/m5stack/M5Unit-TOF

/*
Audio: G0, G2, G19, G27, G34
	I2C: G21, G22 (address: 0x10 and 0x33)

DMX: G19, G27, G35

ExtPort:
	PORTB: G26, G36
	PORTC: G14, G13
	PORTD: G1 (was G35), G3 (was G34)
	PORTE: not configured (was G19), (was G27)
*/

#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>

// A works
#define PORTA_WIRE_SDA 32
#define PORTA_WIRE_SCL 33

// C works
#define PORTC_WIRE_SDA 14
#define PORTC_WIRE_SCL 13

VL53L0X ToF;
VL53L1X ToF4M;
uint8_t addressToF, addressToF4M;

TwoWire Wire2 = TwoWire(2);
TwoWire Wire3 = TwoWire(3);

void setup()
{
  Serial.begin(115200);

  Wire2.begin(PORTA_WIRE_SDA, PORTA_WIRE_SCL);

  ToF.setBus(&Wire2);
  ToF.setTimeout(100);
  if (!ToF.init()) {
    Serial.println("Failed to detect and initialize ToF!");
    delay(5000);
    ESP.restart();
  }

  addressToF = ToF.getAddress();
  Serial.print("I2C address: ");
  Serial.println(addressToF);

  ToF.setMeasurementTimingBudget(20000);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. ToF.startContinuous(100)).
  ToF.startContinuous(20);

  Wire3.begin(PORTC_WIRE_SDA, PORTC_WIRE_SCL);

  ToF4M.setBus(&Wire3);
  ToF4M.setTimeout(100);
  if (!ToF4M.init()) {
    Serial.println("Failed to detect and initialize ToF4M!");
    delay(5000);
    ESP.restart();
  }

  addressToF4M = ToF4M.getAddress();
  Serial.print("I2C address: ");
  Serial.println(addressToF4M);

  ToF4M.setDistanceMode(VL53L1X::Short);
  ToF4M.setMeasurementTimingBudget(20000);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. ToF.startContinuous(100)).
  ToF4M.startContinuous(20);
}

void loop()
{
  Serial.print("Min:0,Max:2000");

  uint16_t ToFmeas = ToF.readRangeContinuousMillimeters();
  ToFmeas = constrain(ToFmeas, 0, 2000);
  Serial.print(",ToF:");
  Serial.print(ToFmeas);

  uint16_t ToF4Mmeas = ToF4M.read();
  ToF4Mmeas = constrain(ToF4Mmeas, 0, 2000);
  Serial.print(",ToF4M:");
  Serial.print(ToF4Mmeas);

  Serial.println();
}
