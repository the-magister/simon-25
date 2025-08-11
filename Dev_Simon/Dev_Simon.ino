/**
 * Simon Memory Game for M5Stack Core2—Enhanced
 *
 * Libraries (versions in comments):
 *   M5Unified v0.2.7       // M5Unified.hpp
 *   M5GFX     v0.2.9       // M5GFX.h
  *   SparkFunDMX v2.0.1     // SparkFunDMX.h
 *   Chrono    v1.2.0       // Chrono.h
 *   PNGdec    v1.1.4       // PNGdec.h
 *
 *   - Four colored quadrants with unique tones
 *   - Progressive speed-up as sequence grows
 *   - Debounced touches (press-and-release)
 *   - Safe DMX / watchdog service during all waits
 *   - LittleFS/PNG logo cached in sprite
 */

// probably want a ESP32-S3-WROOM-1U with 16 Mb of memory so we can store more songs.

#include <SD.h>
#include <M5Unified.hpp>  // v0.2.7
#include <M5GFX.h>        // v0.2.9
#include <SparkFunDMX.h>  // v2.0.1
#include <Chrono.h>       // v1.2.0
#include <vector>
#include <Arduino.h>
#include <Streaming.h>

// Display geometry
static constexpr int SCREEN_W = 320;
static constexpr int SCREEN_H = 240;
static constexpr int BORDER = 5;  // gap between buttons
static constexpr int RADIUS = 30;  // corner radius

// Pre-calc half sizes
static constexpr int HALF_W = SCREEN_W / 2;
static constexpr int HALF_H = SCREEN_H / 2;
static constexpr int BORDER2 = BORDER / 2;

// Canvas & logo sprite
M5Canvas canvas(&M5.Display);
static uint8_t *logoBuf = nullptr;
static size_t logoLen = 0;

// Sequence storage
static const int MAX_SEQ = 64;
uint8_t sequence[MAX_SEQ];
int sequenceLen = 0, userIndex = 0;

// Game states
enum GameState { GAME_IDLE,
                 GAME_SHOW,
                 GAME_INPUT,
                 TEST } gameState = GAME_IDLE;

// DMX
// --- DMX Configuration ------------------------------------------------------
#define DMX_RX_PIN G33
#define DMX_TX_PIN G32
#define DMX_EN_PIN -1
#define DMX_CHANNELS 64
#define DMX_BAUD 250000
#define DMX_FORMAT SERIAL_8N2
#define DMX_BUF_SIZE 256
HardwareSerial dmxSerial(2);
SparkFunDMX dmx;
Chrono dmxChrono;

// indexing for colors, towers, DMX
#define I_RED 0
#define I_GREEN 1
#define I_BLUE 2
#define I_YELLOW 3
#define I_ALL 4
#define N_COLORS 4
#define I_NONE 99

// how loud?
#define SPEAKER_VOLUME 255
static constexpr const gpio_num_t SDCARD_CSPIN = GPIO_NUM_4;
uint8_t *wavBuf = nullptr;
size_t wavLen = 0;

// Button struct & array
struct Btn {
  int x, y, w, h;
  uint16_t color;
};
static Btn btns[4];

// Forward declarations
void drawBoard(byte index);
void playSD(String fname, uint32_t repeat);
void playTone(byte index);
void playFire(byte index, byte level);
void playColor(byte index, byte level);
void fanfare();
void showSequence();
void handleUserInput(bool withFire);
void startNewRound();
void safeDMXUpdate();
void safeDelay(uint32_t ms);

void setup() {
  // M5 init (Serial @115200)
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  Serial.println("=== Simon Setup ===");

  // random() is improved in ESP32
  useRealRandomGenerator(true);

  if (!SD.begin(SDCARD_CSPIN, SPI, 25000000)) {  // TFCARD_CS is typically defined in M5Stack libraries
    Serial.println("SD Card initialization failed!");
  }
  Serial.println("SD Card initialized.");

  File f;

  // Load logo from SD
  f = SD.open("/Simon10Logo.png", "r");
  if (f) {
    logoLen = f.size();
    logoBuf = (uint8_t *)malloc(logoLen);
    f.read(logoBuf, logoLen);
    f.close();
    Serial.printf("[Setup] Logo %u bytes\n", logoLen);
  } else Serial.println("[Error] Logo open failed");

  // define once
  btns[I_RED] = { HALF_W + BORDER2, 0, HALF_W - BORDER2, HALF_H - BORDER2, RED };                   // UR
  btns[I_GREEN] = { 0, 0, HALF_W - BORDER2, HALF_H - BORDER2, GREEN };                              // UL
  btns[I_BLUE] = { HALF_W + BORDER2, HALF_H + BORDER2, HALF_W - BORDER2, HALF_H - BORDER2, BLUE };  // LR
  btns[I_YELLOW] = { 0, HALF_H + BORDER2, HALF_W - BORDER2, HALF_H - BORDER2, YELLOW };             // LL
  canvas.createSprite(SCREEN_W, SCREEN_H);
  drawBoard(I_NONE);
  gameState = GAME_IDLE;

  M5.Speaker.begin();
  M5.Speaker.setVolume(SPEAKER_VOLUME);
  Serial.println("Speaker OK");

  // DMX stuff
  dmxSerial.setTxBufferSize(DMX_BUF_SIZE);
  dmxSerial.begin(DMX_BAUD, DMX_FORMAT, DMX_RX_PIN, DMX_TX_PIN);
  dmx.begin(dmxSerial, DMX_EN_PIN, DMX_CHANNELS);
  dmx.setComDir(DMX_WRITE_DIR);
  // pre-set the right colors for each tower
  dmx.writeByte(255, 0 + 2);   // red
  dmx.writeByte(255, 10 + 3);  // green
  dmx.writeByte(255, 20 + 4);  // blue
  dmx.writeByte(255, 30 + 2);  // yellow
  dmx.writeByte(100, 30 + 3);  // yellow
  // master for each tower can now be used to on/off the colors.
  safeDMXUpdate();
  Serial.println("DMX OK");

  Serial.println("Setup complete");
}

void loop() {
  M5.update();
  safeDMXUpdate();

  if (M5.BtnA.wasReleased()) {
    sequenceLen = 0;
    gameState = GAME_IDLE;
  }
  if (M5.BtnB.wasReleased()) gameState = TEST;

  switch (gameState) {
    case GAME_IDLE:
      if (M5.Touch.getCount() > 0) {
        while (M5.Touch.getCount() > 0) {
          M5.update();
          safeDMXUpdate();
        }
        startNewRound();
      }
      break;
    case GAME_SHOW:
      showSequence();
      gameState = GAME_INPUT;
      userIndex = 0;
      break;
    case GAME_INPUT:
      handleUserInput(false);
      break;
    case TEST:
      handleUserInput(true);
      break;
  }
}

void playSD(String fname, uint32_t repeat) {

  File wavFile = SD.open(fname, "r");
  if (!wavFile) {
    Serial << "Cannot open file: " << fname << endl;
    return;
  }

  // free up memory from last song.
  M5.Speaker.stop();
  free(wavBuf);

  // load the music
  if( repeat == 1 ) wavLen = wavFile.size();
  else wavLen = 44100UL; // cheaty-pants.  Since we'll repeat this, then get one second of a 44.1 kHz file.

  size_t heap_size = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  Serial << "Allocating " << wavLen << " of available " << heap_size << " bytes for " << fname << endl;

  wavBuf = (uint8_t *)malloc(wavLen);
  wavFile.read(wavBuf, wavLen);
  wavFile.close();

  // Play the WAV file
  M5.Speaker.playWav(wavBuf, wavLen, repeat, -1, true);
}

void playTone(byte index) {
  if (index == I_NONE) {
    M5.Speaker.stop();
    return;
  }

  String fname;
  switch (index) {
    // this creates a lot of lag, but I'm intentionally stress-testing running music on the same device.  
    // almost certainly want to pre-load these into memory.
    case I_RED: playSD("/tone_red.wav", 99); break;
    case I_GREEN: playSD("/tone_green.wav", 99); break;
    case I_BLUE: playSD("/tone_blue.wav", 99); break;
    case I_YELLOW: playSD("/tone_yellow.wav", 99); break;
    default:
      Serial << "Can't load a tone file.  Exiting." << endl;
      return;
      break;
  }
}

void playFire(byte index, byte level) {
  if (index == I_ALL) {
    playFire(I_RED, level);
    playFire(I_GREEN, level);
    playFire(I_BLUE, level);
    playFire(I_YELLOW, level);
    return;
  }
  byte channel = (index * 10) + 9;  // solenoid
  dmx.writeByte(level, channel);
}

void playColor(byte index, byte level) {
  if (index == I_ALL) {
    playColor(I_RED, level);
    playColor(I_GREEN, level);
    playColor(I_BLUE, level);
    playColor(I_YELLOW, level);
    return;
  }
  byte channel = (index * 10) + 1;  // master
  dmx.writeByte(level, channel);
}

void drawBoard(byte index) {
  // clear
  canvas.fillSprite(0);

  // logo
  int w = canvas.width(), h = canvas.height();
  int hw = w / 2, hh = h / 2;
  int cx = hw, cy = hh, cr = 55;
  canvas.fillCircle(cx, cy, cr, TFT_BLACK);
  canvas.drawPng(logoBuf, logoLen, cx - cr, cy - cr);

  for (byte i = I_RED; i < N_COLORS; i++) {
    if (i == index) {
      // pressed = filled
      playColor(i, 255);
      canvas.fillRoundRect(btns[i].x, btns[i].y, btns[i].w, btns[i].h, RADIUS, btns[i].color);
    } else {
      // unpressed = open
      playColor(i, 0);
      canvas.drawRoundRect(btns[i].x, btns[i].y, btns[i].w, btns[i].h, RADIUS, btns[i].color);
    }
  }

  // ship it
  canvas.pushSprite(0, 0);
}

void showSequence() {
  // speed-up factor: shorter pause as seq grows
  uint32_t pauseUnlit = max(200, 300 - sequenceLen * 5);
  safeDelay(pauseUnlit);
  for (int i = 0; i < sequenceLen; i++) {
    byte c = sequence[i];

    playTone(c);
    drawBoard(c);

    // pause
    safeDelay(200);

    drawBoard(I_NONE);
    playTone(I_NONE);

    // pause
    safeDelay(pauseUnlit);
  }
}

void handleUserInput(bool withFire) {
  // wait for press
  if (M5.Touch.getCount() == 0) return;
  auto dt = M5.Touch.getDetail(0);
  int tx = dt.x, ty = dt.y;

  // find which
  for (byte i = I_RED; i < N_COLORS; i++) {
    auto &b = btns[i];
    if (tx >= b.x && tx <= b.x + b.w && ty >= b.y && ty <= b.y + b.h) {
      // update the board and make a tone
      playTone(i);
      drawBoard(i);
      if (withFire) playFire(i, 255);

      // debounce: wait release
      while (M5.Touch.getCount() > 0) {
        M5.update();
        safeDMXUpdate();
      }

      // clean up
      playTone(I_NONE);
      drawBoard(I_NONE);
      playFire(I_ALL, 0);

      if (gameState == TEST) break;

      if (i == sequence[userIndex]) {
        userIndex++;
        if (userIndex >= sequenceLen) {
          // success
          if (sequenceLen >= MAX_SEQ) fanfare();
          else startNewRound();
        }
      } else {
        // mistake
        fanfare();
      }
      break;
    }
  }
}

void startNewRound() {
  if (sequenceLen < MAX_SEQ) {
    sequence[sequenceLen++] = random(0, N_COLORS);
    gameState = GAME_SHOW;
  } else {
    fanfare();
  }
}

void fanfare() {
  M5.Speaker.stop();

  Serial << "Fanfare with sequence length=" << sequenceLen << endl;

  File wavFile;
  if (sequenceLen < 4) playSD("/Fail 1.wav", 1);
  else playSD("/Win 1.wav", 1);

  // reset for next game
  sequenceLen = 0;
  gameState = GAME_IDLE;

  // do some lights and fire while playing
  while (M5.Speaker.isPlaying()) {
    // send some lights
    playColor(I_ALL, 0);
    playFire(I_ALL, 0);

    byte index = random(I_RED, I_ALL + 1);

    playColor(index, 255);
    playFire(index, 255);

    uint32_t dur = random(100UL, 500UL);
    safeDelay(dur);
  }

  playFire(I_ALL, 0);
  playColor(I_ALL, 0);
}

void safeDMXUpdate() {
  if (dmxSerial.availableForWrite() == DMX_BUF_SIZE) {
    dmx.update();
  }
}

void safeDelay(uint32_t ms) {
  Chrono cd(true);
  while (!cd.hasPassed(ms)) {
    M5.update();  // feeds M5 watchdog, services touch
    safeDMXUpdate();
    yield();  // ESP RTOS
  }
}
