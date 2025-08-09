#ifndef DMX_h
#define DMX_h

#include <Arduino.h>

#include <HardwareSerial.h>
// https://github.com/sparkfun/SparkFunDMX/tree/master
#include <SparkFunDMX.h>

// https://github.com/SofaPirate/Chrono
#include <Chrono.h>

// C++ style piping for Serial
#include <Streaming.h>

// relative locations for the colors on the Simon Console
enum tower {
  I_RED = 0,  // upper right
  I_GRN,      // upper left
  I_BLU,      // lower right
  I_YEL,      // lower left
  I_ALL,      // use this to size arrays appropriately
};

// sized by the fixture DMX structure
typedef struct {
  byte master;
  byte red;
  byte green;
  byte blue;
  byte white;
} colorInstruction;

// could add more from the DMX repertoire 
const colorInstruction cOff = { 0, 0, 0, 0, 0 };
const colorInstruction cRed = { 255, 255, 0, 0, 0 };
const colorInstruction cGreen = { 255, 0, 255, 0, 0 };
const colorInstruction cBlue = { 255, 0, 0, 255, 0 };
const colorInstruction cYellow = { 255, 255, 100, 0, 0 };
const colorInstruction cWhite = { 255, 0, 0, 0, 255 };
const colorInstruction cAll = { 255, 255, 255, 255, 255 };
// and this serves as an easy way to pull out the right RGB color for each tower
const colorInstruction cMap[I_ALL] = { cRed, cGreen, cBlue, cYellow };

// likewise, the fixture DMX structure
typedef struct {
  byte first;
  byte second;
} fireInstruction;

// unlikely to add more from the DMX repertoire
const fireInstruction fOff = { 0, 0 };
const fireInstruction fOn = { 255, 255 };

class DMX {
  public:
    void begin();
    bool update(); // call periodically

    // execute gestures across the DMX universe
    void towerLight(tower Tower, colorInstruction Color);
    void towerFire(tower Tower, fireInstruction Fire);

  private:
    // flags a change that needs a push; limited by DMX refresh rate.
    bool needsUpdate;
};

#endif