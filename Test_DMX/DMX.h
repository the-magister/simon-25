#ifndef DMX_h
#define DMX_h

#include <Arduino.h>

// https://github.com/sparkfun/SparkFunDMX/tree/master
//#include <HardwareSerial.h>
#include <SparkFunDMX.h>
// https://github.com/SofaPirate/Chrono
#include <Chrono.h>

#include <Streaming.h>

// relative locations for the colors on the Simon Console
enum tower {
  I_RED = 0,  // upper right
  I_GRN,      // upper left
  I_BLU,      // lower right
  I_YEL,      // lower left
  I_ALL,      // use this to size arrays appropriately
};

typedef struct {
  byte master;
  byte red;
  byte green;
  byte blue;
  byte white;
} colorInstruction;

const colorInstruction cOff = { 0, 0, 0, 0, 0 };
const colorInstruction cRed = { 255, 255, 0, 0, 0 };
const colorInstruction cGreen = { 255, 0, 255, 0, 0 };
const colorInstruction cBlue = { 255, 0, 0, 255, 0 };
const colorInstruction cYellow = { 255, 255, 100, 0, 0 };
const colorInstruction cWhite = { 255, 0, 0, 0, 255 };
const colorInstruction cAll = { 255, 255, 255, 255, 255 };
// and this serves as an easy way to pull out the right RGB color from the
const colorInstruction cMap[I_ALL] = { cRed, cGreen, cBlue, cYellow };

typedef struct {
  byte first;
  byte second;
} fireInstruction;

const fireInstruction fOff = { 0, 0 };
const fireInstruction fOn = { 255, 255 };

class DMX {
  public:
    void begin();
    bool update();

    void towerLight(tower Tower, colorInstruction Color);
    void towerFire(tower Tower, fireInstruction Fire);

  private:
    bool needsUpdate;
};

#endif