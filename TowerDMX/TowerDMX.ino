#include <Arduino.h>
#include <esp_dmx.h>
#include <M5StickPlus2.h>

// https://docs.m5stack.com/en/arduino/m5stickc_plus2/program 
/*
  1. Add to Board Managers in Preferences: https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json
  2. Install esp_dmx Library
  3. Install M5Unit-DMX512

*/

int transmitPin = 17;
int receivePin = 16;
int enablePin = -1;
dmx_port_t dmxPort = 1;

byte data[DMX_PACKET_SIZE];
unsigned long lastUpdate = millis();
bool setupSuccess = false, baudState = false;  
uint8_t sendCounter = 0;


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
