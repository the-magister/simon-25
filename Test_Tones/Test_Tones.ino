
#include <M5Unified.hpp>  // v0.2.7
#include <M5GFX.h>        // v0.2.9
#include <Streaming.h>
#include <Chrono.h>  // v1.2.0

#define I_RED 0
#define I_GREEN 1
#define I_BLUE 2
#define I_YELLOW 3
#define I_ALL 4
#define N_COLORS 4
#define I_NONE 99

#define SPEAKER_VOLUME 255
const uint16_t Tones[N_COLORS] = { 330, 392, 196, 262 };

void setup() {
  // put your setup code here, to run once:
  // M5 init (Serial @115200)
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  Serial << "=== Simon Setup ===" << endl;

  M5.Speaker.begin();
  M5.Speaker.setVolume(SPEAKER_VOLUME);
  for (byte i = I_RED; i < N_COLORS; i++) {
    M5.Speaker.setChannelVolume(i, 0);
    M5.Speaker.tone(Tones[i], UINT32_MAX, i, false);
  }
  Serial << "Speaker OK" << endl;

}

void loop() {
  // put your main code here, to run repeatedly:
  M5.update();

  static byte currIndex = I_NONE;

  if (M5.BtnA.wasReleased()) {
    if( currIndex < N_COLORS ) M5.Speaker.setChannelVolume(currIndex, 0);
    Serial << "Old Index: " << currIndex;

    currIndex++;
    if( currIndex >= I_NONE ) currIndex = I_RED;
    if( currIndex >= N_COLORS ) currIndex = I_NONE;

    if( currIndex < N_COLORS ) M5.Speaker.setChannelVolume(currIndex, 255);

    Serial << ". New Index: " << currIndex << endl;
  }

}
