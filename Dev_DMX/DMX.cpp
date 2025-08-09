// Handle DMX calls for light and fire
#include "DMX.h"

// Create DMX object
SparkFunDMX dmxDevice;

// Create serial port to be used for DMX interface.
HardwareSerial dmxSerial(2);

// Serial pinout

// const uint8_t enPin = -1, rxPin = G33, txPin = G32;
const uint8_t enPin = -1;
const uint8_t rxPin = G33;
const uint8_t txPin = G32;
//const uint8_t rxPin = SCL;
//const uint8_t txPin = SDA;

// Number of DMX channels, can be up tp 512
// REVISIT
const uint16_t numChannels = 64;

uint16_t dmxSerialBufferSize = 0;

void DMX::begin() {
  // Begin DMX serial port
  dmxSerial.begin(DMX_BAUD, DMX_FORMAT, rxPin, txPin);
  dmxSerial.setTxBufferSize(512); // dmx universe size.
  dmxSerial.flush();
  // how big is the buffer?
  dmxSerialBufferSize = dmxSerial.availableForWrite();

  // Begin DMX driver
  dmxDevice.begin(dmxSerial, enPin, numChannels);

  // Set communicaiton direction, which can be changed on the fly as needed
  dmxDevice.setComDir(DMX_WRITE_DIR);

  // and plan an update
  this->needsUpdate = true;
}

bool DMX::update() {
  // current outbound buffer size
  uint16_t sendingBufferSize = dmxSerialBufferSize - dmxSerial.availableForWrite();
  // if we're currently sending, bail out.
  if( sendingBufferSize > 0 ) return(false);

  // some devices will timeout without a DMX stream.
  static Chrono dmxSendInterval;
  if ( this->needsUpdate || dmxSendInterval.hasPassed(1000UL) ) {
    // no need to resend
    this->needsUpdate = false;
    dmxSendInterval.restart();

    // send it 
    return( dmxDevice.update() );
  }

}

void DMX::towerLight(tower Tower, colorInstruction Color) {
  // all towers?
  if (Tower == I_ALL) {
    this->towerLight(I_RED, Color);
    this->towerLight(I_GRN, Color); 
    this->towerLight(I_BLU, Color);
    this->towerLight(I_YEL, Color);
    return;
  }

  // otherwise, locate the tower
  byte addressStart = ((byte)Tower * 10) + 1;

  // write this color to the correct tower.
  dmxDevice.writeBytes((uint8_t*)&Color, sizeof(colorInstruction), addressStart);

  // schedule a send
  this->needsUpdate = true;

  Serial << "T" << (byte)Tower << " A" << addressStart << " -> M" << Color.master << " R" << Color.red << " G" << Color.green << " B" << Color.blue << endl;
}

void DMX::towerFire(tower Tower, fireInstruction Fire) {
  // all towers?
  if (Tower == I_ALL) {
    this->towerFire(I_RED, Fire);
    this->towerFire(I_GRN, Fire); 
    this->towerFire(I_BLU, Fire); 
    this->towerFire(I_YEL, Fire);
    return;
  }

  // otherwise, locate the tower
  byte addressStart = ((byte)Tower * 10) + 9;

  // write this color to the correct tower.
  dmxDevice.writeBytes((uint8_t*)&Fire, sizeof(fireInstruction), addressStart);

  // schedule a send
  this->needsUpdate = true;

  Serial << "T" << (byte)Tower << " A"<< addressStart << " -> 1 " << Fire.first << " 2 " << Fire.second << endl;
}
