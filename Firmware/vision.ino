// ============================================================================
// MODULOVE & SPCTRL / WGD - MODULAR VISION — Eurorack Oscilloscope / Tuner / Vibe
// ============================================================================
// Hardware: Arduino Nano, SSD1306 128x32 OLED, rotary encoder w/ push button
// Analog input: A0 (0–5V via internal 1.1V reference + voltage divider)
// ============================================================================
// Dependencies:
//   - Adafruit_SSD1306, Adafruit_GFX
//   - EncoderButton (https://github.com/Stutchbury/EncoderButton)
// ============================================================================

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <math.h>
#include <EncoderButton.h>

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------------------------------------------------------------------
// Pins — fixed by VISION PCB layout
// Pin 2 = INT0 (encoder A), Pin 4 = encoder B (polled), Pin 5 = button
// ---------------------------------------------------------------------------
#define ENCODER_PIN_A  2
#define ENCODER_PIN_B  4
#define BUTTON_PIN     5
#define ANALOG_PIN     0      // Use raw analog channel number (A0), matching original firmware

// ---------------------------------------------------------------------------
// EncoderButton instance
// ---------------------------------------------------------------------------
EncoderButton eb(ENCODER_PIN_A, ENCODER_PIN_B, BUTTON_PIN);

// ---------------------------------------------------------------------------
// Sampling buffer
// ---------------------------------------------------------------------------
#define REC_LENGTH 200
int waveBuff[REC_LENGTH];

// ---------------------------------------------------------------------------
// Operating modes
// ---------------------------------------------------------------------------
enum Mode : uint8_t {
  MODE_SCOPE = 0,
  MODE_TUNE  = 1,
  MODE_COUNT
};

enum ScopeParam : uint8_t {
  SPARAM_TIMEBASE = 0,
  SPARAM_VRANGE,
  SPARAM_TRIGGER,
  SPARAM_OFFSET,
  SPARAM_COUNT
};

enum TuneParam : uint8_t {
  TPARAM_DISPLAY = 0,
  TPARAM_REF_PITCH,
  TPARAM_AVG_SPEED,
  TPARAM_COUNT
};

enum TuneDisplay : uint8_t {
  TDISP_HZ   = 0,
  TDISP_NOTE = 1,
  TDISP_COUNT
};

// ---------------------------------------------------------------------------
// Time base — stored as separate PROGMEM arrays to avoid struct padding issues
// ---------------------------------------------------------------------------
// Labels (6 chars + null each)
const char tb_label_0[]  PROGMEM = "  10us";
const char tb_label_1[]  PROGMEM = "  50us";
const char tb_label_2[]  PROGMEM = " 100us";
const char tb_label_3[]  PROGMEM = " 200us";
const char tb_label_4[]  PROGMEM = " 500us";
const char tb_label_5[]  PROGMEM = "   1ms";
const char tb_label_6[]  PROGMEM = "   2ms";
const char tb_label_7[]  PROGMEM = "   5ms";
const char tb_label_8[]  PROGMEM = "  10ms";
const char tb_label_9[]  PROGMEM = " 100ms";
const char tb_label_10[] PROGMEM = " 500ms";
const char tb_label_11[] PROGMEM = "    1s";
const char tb_label_12[] PROGMEM = "   10s";

const char* const tb_labels[] PROGMEM = {
  tb_label_0, tb_label_1, tb_label_2, tb_label_3,
  tb_label_4, tb_label_5, tb_label_6, tb_label_7,
  tb_label_8, tb_label_9, tb_label_10, tb_label_11, tb_label_12
};

// Inter-sample delay (microseconds); 0 = fastest / no extra delay
const uint16_t tb_delayUs[] PROGMEM = {
  0, 5, 18, 4, 8, 18, 48, 138, 288, 0, 0, 0, 0
};

// ADC prescaler bits — always use 0x07 (prescaler 128) for reliable readings
// with the INTERNAL 1.1V reference. Faster prescalers (0x04) cause inaccurate
// readings on Nano and are not used in the original working firmware.
const uint8_t tb_prescaler[] PROGMEM = {
  0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
  0x07, 0x07, 0x07, 0x07, 0x07
};

// For slow time bases (index 9–12): use millis-based pacing
// 0 = use delayMicroseconds, >0 = millis delay per sample
const uint16_t tb_delayMs[] PROGMEM = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 5, 50
};
// Indices 0–8 use delayMicroseconds; 9 uses neither (just ADC time);
// 10–12 use millis delay

#define NUM_TIMEBASES 13

// Helper: is this time base millis-paced?
bool tb_isMillis(uint8_t idx) {
  return (idx >= 9);
}

// ---------------------------------------------------------------------------
// Voltage range — simple PROGMEM arrays
// ---------------------------------------------------------------------------
const char vr_label_0[] PROGMEM = "0.5V";
const char vr_label_1[] PROGMEM = "  1V";
const char vr_label_2[] PROGMEM = "  2V";
const char vr_label_3[] PROGMEM = "  3V";
const char vr_label_4[] PROGMEM = "  5V";

const char* const vr_labels[] PROGMEM = {
  vr_label_0, vr_label_1, vr_label_2, vr_label_3, vr_label_4
};

const uint16_t vr_fullScale_mV[] PROGMEM = {
  500, 1000, 2000, 3000, 5000
};

#define NUM_VRANGES 5

// ---------------------------------------------------------------------------
// Note names for tuner
// ---------------------------------------------------------------------------
const char noteNames[12][3] PROGMEM = {
  "C ", "C#", "D ", "D#", "E ", "F ",
  "F#", "G ", "G#", "A ", "A#", "B "
};

// ---------------------------------------------------------------------------
// State variables
// ---------------------------------------------------------------------------
uint8_t currentMode = MODE_SCOPE;

// Scope
uint8_t  timeBaseIdx   = 8;    // default 10 ms
uint8_t  vRangeIdx     = 4;    // default 5V
int16_t  trigThreshold = 512;  // 0–1023 ADC
int8_t   displayOffset = 0;    // vertical pixel offset (–16 … +16)
uint8_t  scopeParam    = SPARAM_TIMEBASE;

// Tuner
uint8_t  tuneParam     = TPARAM_DISPLAY;
uint8_t  tuneDisplay   = TDISP_NOTE;
uint16_t refPitchX10   = 4400;  // 440.0 Hz in tenths
uint8_t  avgSpeed      = 2;     // 1=fast, 2=med, 3=slow
float    measuredFreq  = 0.0;
float    smoothedFreq  = 0.0;

// Hold
bool hold = false;

// UI
unsigned long menuTimeout = 0;
bool          menuVisible = true;

// Screensaver
unsigned long lastActivity = 0;
const unsigned long SCREENSAVER_TIMEOUT = 30000;
#define SIGNAL_NOISE_FLOOR 15

// Starfield — use int16_t instead of float to save RAM
#define NUM_STARS 16
struct Star {
  int16_t x, y, z;  // 6 bytes vs 12 for floats → saves 96 bytes
};
Star stars[NUM_STARS]; // 96 bytes total

// EEPROM addresses
#define EE_TIMEBASE   0
#define EE_VRANGE     1
#define EE_TRIG_H     2
#define EE_TRIG_L     3
#define EE_OFFSET     4
#define EE_MODE       5
#define EE_TUNEDISP   6
#define EE_REFPITCH_H 7
#define EE_REFPITCH_L 8
#define EE_AVGSPEED   9

unsigned long lastSaveTime = 0;
bool          needsSave    = false;

// Data analysis
int dataMin, dataMax, dataAve;

// Temp buffer for reading PROGMEM strings
char labelBuf[8];

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void readWave();
void dataAnalyze();
void plotScope();
void drawTuner();
void drawScreensaver();
void drawScopeMenu();
void drawTunerMenu();
void loadSettings();
void saveSettings();
void initStarfield();
void startScreen();
float measureFrequency();
void onEncoder(EncoderButton &eb);
void onClicked(EncoderButton &eb);
void onLongPress(EncoderButton &eb);

// ---------------------------------------------------------------------------
// Helper: mark user activity
// ---------------------------------------------------------------------------
void markActivity() {
  lastActivity = millis();
  menuTimeout  = millis();
  menuVisible  = true;
}

// ---------------------------------------------------------------------------
// Helper: read a timebase label into labelBuf
// ---------------------------------------------------------------------------
void readTbLabel(uint8_t idx) {
  strcpy_P(labelBuf, (const char*)pgm_read_word(&tb_labels[idx]));
}

// Helper: read a vrange label into labelBuf
void readVrLabel(uint8_t idx) {
  strcpy_P(labelBuf, (const char*)pgm_read_word(&vr_labels[idx]));
}

// ============================================================================
// ENCODER CALLBACK
// ============================================================================
void onEncoder(EncoderButton &eb) {
  int delta = eb.increment();
  markActivity();

  if (currentMode == MODE_SCOPE) {
    switch (scopeParam) {
      case SPARAM_TIMEBASE:
        timeBaseIdx = constrain((int8_t)timeBaseIdx + delta, 0, NUM_TIMEBASES - 1);
        break;
      case SPARAM_VRANGE:
        vRangeIdx = constrain((int8_t)vRangeIdx + delta, 0, NUM_VRANGES - 1);
        break;
      case SPARAM_TRIGGER:
        trigThreshold = constrain(trigThreshold + delta * 16, 0, 1023);
        break;
      case SPARAM_OFFSET:
        displayOffset = constrain(displayOffset + delta, -16, 16);
        break;
    }
  } else {
    switch (tuneParam) {
      case TPARAM_DISPLAY:
        tuneDisplay = (tuneDisplay + delta + TDISP_COUNT) % TDISP_COUNT;
        break;
      case TPARAM_REF_PITCH:
        refPitchX10 = constrain((int16_t)refPitchX10 + delta * 10, 4000, 4600);
        break;
      case TPARAM_AVG_SPEED:
        avgSpeed = constrain((int8_t)avgSpeed + delta, 1, 3);
        break;
    }
  }
  needsSave = true;
}

// ============================================================================
// CLICK CALLBACK — cycles parameters, wraps to next mode
// ============================================================================
void onClicked(EncoderButton &eb) {
  markActivity();

  if (currentMode == MODE_SCOPE) {
    scopeParam = (scopeParam + 1) % (SPARAM_COUNT + 1);
    if (scopeParam == SPARAM_COUNT) {
      currentMode = MODE_TUNE;
      scopeParam = SPARAM_TIMEBASE;
      smoothedFreq = 0;
    }
  } else {
    tuneParam = (tuneParam + 1) % (TPARAM_COUNT + 1);
    if (tuneParam == TPARAM_COUNT) {
      currentMode = MODE_SCOPE;
      tuneParam = TPARAM_DISPLAY;
    }
  }
  needsSave = true;
}

// ============================================================================
// LONG PRESS CALLBACK — toggles hold
// ============================================================================
void onLongPress(EncoderButton &eb) {
  markActivity();
  hold = !hold;
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  pinMode(13, OUTPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for (;;);
  }

  analogReference(INTERNAL);
  loadSettings();
  initStarfield();

  eb.setEncoderHandler(onEncoder);
  eb.setClickHandler(onClicked);
  eb.setLongPressHandler(onLongPress);

  startScreen();
  lastActivity = millis();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  eb.update();

  // Screensaver — only when idle AND no signal
  if (millis() - lastActivity > SCREENSAVER_TIMEOUT && !hold) {
    readWave();
    dataAnalyze();
    if ((dataMax - dataMin) < SIGNAL_NOISE_FLOOR) {
      drawScreensaver();
      display.display();
      return;
    }
  }

  if (hold) {
    display.fillRect(SCREEN_WIDTH - 12, 0, 12, 9, BLACK);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(SCREEN_WIDTH - 11, 0);
    display.print(F("H"));
    display.display();
    return;
  }

  switch (currentMode) {
    case MODE_SCOPE:
      readWave();
      dataAnalyze();
      display.clearDisplay();
      plotScope();
      if (menuVisible) drawScopeMenu();
      display.display();
      break;

    case MODE_TUNE: {
      // Use a fixed time base for tuning: 5ms (idx 7) captures ~4–20 cycles
      // for typical eurorack audio range. Scope time base is preserved.
      uint8_t savedTb = timeBaseIdx;
      timeBaseIdx = 7; // 5ms

      readWave();
      dataAnalyze();
      measuredFreq = measureFrequency();

      timeBaseIdx = savedTb; // restore user's scope time base

      // Update smoothed frequency with outlier rejection
      if (measuredFreq < 0.5) {
        // No valid reading — slowly decay to show "---"
        smoothedFreq *= 0.95;
        if (smoothedFreq < 1.0) smoothedFreq = 0;
      } else if (smoothedFreq < 1.0) {
        // First valid reading — seed directly
        smoothedFreq = measuredFreq;
      } else {
        // Outlier rejection: ignore readings that jump more than ±30%
        float ratio = measuredFreq / smoothedFreq;
        if (ratio > 0.7 && ratio < 1.4) {
          float alpha;
          switch (avgSpeed) {
            case 1:  alpha = 0.4;  break;  // fast
            case 2:  alpha = 0.2;  break;  // medium
            default: alpha = 0.08; break;  // slow
          }
          smoothedFreq = alpha * measuredFreq + (1.0 - alpha) * smoothedFreq;
        }
        // else: outlier, silently discard
      }

      display.clearDisplay();
      drawTuner();
      if (menuVisible) drawTunerMenu();
      display.display();
      break;
    }
  }

  // Menu auto-hide
  if (menuVisible && millis() - menuTimeout > 3000) {
    menuVisible = false;
  }

  // Periodic EEPROM save
  if (needsSave && millis() - lastSaveTime > 5000) {
    saveSettings();
    needsSave = false;
    lastSaveTime = millis();
  }
}

// ============================================================================
// READ WAVEFORM
// ============================================================================
void readWave() {
  // Pin 12 controls the input attenuation circuit on the VISION PCB.
  // Setting it to INPUT (high-Z) selects the 5V direct path.
  pinMode(12, INPUT);

  // Always use prescaler 128 (0x07) — matches original working firmware.
  ADCSRA = (ADCSRA & 0xF8) | 0x07;

  uint16_t delayUs = pgm_read_word(&tb_delayUs[timeBaseIdx]);
  uint16_t delayMs = pgm_read_word(&tb_delayMs[timeBaseIdx]);
  bool     useMs = tb_isMillis(timeBaseIdx);

  if (useMs) {
    for (int i = 0; i < REC_LENGTH; i++) {
      waveBuff[i] = analogRead(ANALOG_PIN);
      if (delayMs > 0) delay(delayMs);
    }
  } else {
    for (int i = 0; i < REC_LENGTH; i++) {
      waveBuff[i] = analogRead(ANALOG_PIN);
      if (delayUs > 0) delayMicroseconds(delayUs);
    }
  }
}

// ============================================================================
// DATA ANALYSIS
// ============================================================================
void dataAnalyze() {
  int yMin = 1023, yMax = 0;
  long ySum = 0;
  for (int i = 0; i < REC_LENGTH; i++) {
    int v = waveBuff[i];
    ySum += v;
    if (v > yMax) yMax = v;
    if (v < yMin) yMin = v;
  }
  dataAve = ySum / REC_LENGTH;
  dataMax = yMax;
  dataMin = yMin;
}

// ============================================================================
// FIND TRIGGER — rising edge crossing
// ============================================================================
int findTrigger() {
  for (int i = 1; i < REC_LENGTH - SCREEN_WIDTH - 2; i++) {
    if (waveBuff[i - 1] < trigThreshold && waveBuff[i] >= trigThreshold) {
      return i;
    }
  }
  return 0;
}

// ============================================================================
// PLOT SCOPE — waveform only when menu hidden, trigger line when menu shown
// ============================================================================
void plotScope() {
  uint16_t fsMillivolts = pgm_read_word(&vr_fullScale_mV[vRangeIdx]);
  int rangeMax = (long)fsMillivolts * 1023L / 5000L;
  int rangeMin = 0;

  int trigStart = findTrigger();

  // Only show trigger line when menu is visible (it's a settings indicator)
  if (menuVisible) {
    int trigY = map(trigThreshold, rangeMin, rangeMax, SCREEN_HEIGHT - 1, 0) + displayOffset;
    trigY = constrain(trigY, 0, SCREEN_HEIGHT - 1);
    for (int x = 0; x < SCREEN_WIDTH; x += 4) {
      display.drawPixel(x, trigY, WHITE);
    }
  }

  // Waveform — full screen width, full height
  for (int x = 0; x < SCREEN_WIDTH - 1; x++) {
    int idx = trigStart + x;
    if (idx + 1 >= REC_LENGTH) break;
    int y1 = map(waveBuff[idx],     rangeMin, rangeMax, SCREEN_HEIGHT - 1, 0) + displayOffset;
    int y2 = map(waveBuff[idx + 1], rangeMin, rangeMax, SCREEN_HEIGHT - 1, 0) + displayOffset;
    y1 = constrain(y1, 0, SCREEN_HEIGHT - 1);
    y2 = constrain(y2, 0, SCREEN_HEIGHT - 1);
    display.drawLine(x, y1, x + 1, y2, WHITE);
  }
}

// ============================================================================
// SCOPE MENU OVERLAY
// ============================================================================
void drawScopeMenu() {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  switch (scopeParam) {
    case SPARAM_TIMEBASE:
      display.print(F(">T:"));
      readTbLabel(timeBaseIdx);
      display.print(labelBuf);
      break;
    case SPARAM_VRANGE:
      display.print(F(">V:"));
      readVrLabel(vRangeIdx);
      display.print(labelBuf);
      break;
    case SPARAM_TRIGGER: {
      display.print(F(">Trg:"));
      float trigV = (float)trigThreshold * 5.0 / 1023.0;
      display.print(trigV, 1);
      display.print(F("V"));
      break;
    }
    case SPARAM_OFFSET:
      display.print(F(">Ofs:"));
      display.print(displayOffset);
      break;
  }

  //display.setCursor(0, SCREEN_HEIGHT - 8);
  //display.print(F("SCOPE"));
}

// ============================================================================
// MEASURE FREQUENCY — zero-crossing with hysteresis
// Uses Schmitt-trigger style crossing detection to reject noise.
// ============================================================================
float measureFrequency() {
  // Minimum peak-to-peak amplitude to attempt measurement (reject noise)
  int amplitude = dataMax - dataMin;
  if (amplitude < 30) return 0.0;  // ~150 mV minimum signal

  int thresh = dataAve;
  // Hysteresis band: 10% of amplitude on each side of threshold
  int hysteresis = amplitude / 10;
  if (hysteresis < 3) hysteresis = 3;
  int threshHigh = thresh + hysteresis;
  int threshLow  = thresh - hysteresis;

  // Find rising-edge crossings with hysteresis
  // We don't store all crossings — just track first, last, and count
  int firstCross = -1;
  int lastCross = -1;
  int nCross = 0;
  bool armed = false;  // must go below threshLow before next rising cross

  for (int i = 1; i < REC_LENGTH; i++) {
    if (waveBuff[i] < threshLow) {
      armed = true;
    }
    if (armed && waveBuff[i - 1] < thresh && waveBuff[i] >= thresh) {
      if (firstCross < 0) firstCross = i;
      lastCross = i;
      nCross++;
      armed = false;
    }
  }

  if (nCross < 2) return 0.0;

  float totalSamples = (float)(lastCross - firstCross);
  float periods = (float)(nCross - 1);

  uint16_t delayUs  = pgm_read_word(&tb_delayUs[timeBaseIdx]);
  uint8_t  prescaler = pgm_read_byte(&tb_prescaler[timeBaseIdx]);
  uint16_t delayMs  = pgm_read_word(&tb_delayMs[timeBaseIdx]);

  float sampleInterval_us;
  if (tb_isMillis(timeBaseIdx)) {
    sampleInterval_us = 112.0 + (float)delayMs * 1000.0;
  } else {
    sampleInterval_us = 112.0 + (float)delayUs;  // all prescalers are 0x07 = ~112µs ADC
  }

  float periodSamples = totalSamples / periods;
  float periodSeconds = periodSamples * sampleInterval_us / 1000000.0;

  if (periodSeconds <= 0) return 0.0;
  return 1.0 / periodSeconds;
}

// ============================================================================
// TUNER DISPLAY
// Two layouts:
//   Menu visible  → compact, leaves room for menu text at bottom
//   Menu hidden   → enlarged, fills the whole 128×32 screen
// ============================================================================
void drawTuner() {
  float freq = smoothedFreq;
  float refA4 = (float)refPitchX10 / 10.0;
  display.setTextColor(WHITE);

  // --- No signal ---
  if (freq < 0.5) {
    display.setTextSize(2);
    display.setCursor(40, 8);
    display.print(F("---"));
    return;
  }

  if (tuneDisplay == TDISP_HZ) {
    // ---- Hz display ----
    if (!menuVisible) {
      // FULL SCREEN: large Hz, centered
      // Size 3 = 18×24 pixels per char, fits ~7 chars across 128px
      display.setTextSize(3);
      display.setCursor(0, 4);
      if (freq < 100.0)       display.print(freq, 1);
      else if (freq < 1000.0) display.print(freq, 0);
      else                     display.print(freq, 0);
      // "Hz" label in size 1, tucked to the right
      display.setTextSize(1);
      display.setCursor(display.getCursorX() + 2, 16);
      display.print(F("Hz"));
    } else {
      // COMPACT: size 2, leave bottom row for menu
      display.setTextSize(2);
      display.setCursor(0, 0);
      if (freq < 100.0)       display.print(freq, 2);
      else if (freq < 1000.0) display.print(freq, 1);
      else                     display.print(freq, 0);
      display.setTextSize(1);
      display.setCursor(display.getCursorX() + 2, 8);
      display.print(F("Hz"));
    }
  } else {
    // ---- Note name + cents display ----
    float semitones = 12.0 * (log(freq / refA4) / log(2.0));
    int nearestSemi = (int)round(semitones);
    float centsOff = (semitones - nearestSemi) * 100.0;

    int noteIdx = ((nearestSemi % 12) + 12 + 9) % 12;
    int octave = 4 + (nearestSemi + 9) / 12;
    if ((nearestSemi + 9) < 0 && ((nearestSemi + 9) % 12 != 0)) octave--;

    char noteBuf[3];
    memcpy_P(noteBuf, noteNames[noteIdx], 3);

    if (!menuVisible) {
      // FULL SCREEN layout:
      // Large note name (size 3) on the left
      // Octave number (size 2) right after
      // Cents + Hz on the right side
      // Cents bar across the bottom

      // Note name — size 3 (18×24 per char)
      display.setTextSize(3);
      display.setCursor(0, 0);
      display.print(noteBuf[0]);
      if (noteBuf[1] == '#') {
        display.setTextSize(2);
        display.print('#');
      }
      // Octave — size 2 right after note
      display.setTextSize(2);
      int ofsX = display.getCursorX();
      display.setCursor(ofsX, 8); // align baseline with note
      display.print(octave);

      // Cents value — right side, size 2
      display.setTextSize(2);
      display.setCursor(80, 0);
      if (centsOff >= 0) display.print('+');
      display.print((int)centsOff);

      // Hz — right side, size 1
      display.setTextSize(1);
      display.setCursor(80, 18);
      display.print(freq, 1);
      display.print(F("Hz"));

      // Cents bar — full width along bottom
      int barY = 30;
      int barCenter = SCREEN_WIDTH / 2;
      // Scale ticks
      display.drawFastVLine(barCenter, barY - 3, 4, WHITE);
      display.drawFastVLine(barCenter - 32, barY - 2, 3, WHITE);
      display.drawFastVLine(barCenter + 32, barY - 2, 3, WHITE);
      display.drawFastVLine(barCenter - 63, barY - 2, 3, WHITE);
      display.drawFastVLine(barCenter + 63, barY - 2, 3, WHITE);
      // Indicator — maps ±50 cents to ±63 pixels
      int indicatorX = barCenter + (int)(constrain(centsOff, -50, 50) * 63.0 / 50.0);
      display.fillRect(indicatorX - 2, barY - 4, 5, 5, WHITE);
    } else {
      // COMPACT layout: leave bottom 8px for menu
      display.setTextSize(2);
      display.setCursor(0, 0);
      display.print(noteBuf[0]);
      if (noteBuf[1] == '#') display.print('#');
      display.setTextSize(1);
      display.print(octave);

      // Cents bar — middle area
      int barCenter = SCREEN_WIDTH / 2;
      int barY = 20;
      display.drawFastVLine(barCenter, barY - 2, 5, WHITE);
      display.drawFastVLine(barCenter - 25, barY - 1, 3, WHITE);
      display.drawFastVLine(barCenter + 25, barY - 1, 3, WHITE);
      display.drawFastVLine(barCenter - 50, barY - 1, 3, WHITE);
      display.drawFastVLine(barCenter + 50, barY - 1, 3, WHITE);
      int indicatorX = barCenter + (int)constrain(centsOff, -50, 50);
      display.fillRect(indicatorX - 2, barY - 3, 5, 7, WHITE);

      // Cents + Hz — top right
      display.setTextSize(1);
      display.setCursor(80, 0);
      if (centsOff >= 0) display.print('+');
      display.print((int)centsOff);
      display.print('c');
      display.setCursor(80, 10);
      display.print(freq, 1);
      display.print(F("Hz"));
    }
  }
}

// ============================================================================
// TUNER MENU
// ============================================================================
void drawTunerMenu() {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, SCREEN_HEIGHT - 8);

  switch (tuneParam) {
    case TPARAM_DISPLAY:
      display.print(F(">"));
      display.print(tuneDisplay == TDISP_HZ ? F("Hz") : F("Note"));
      break;
    case TPARAM_REF_PITCH:
      display.print(F(">A4="));
      display.print(refPitchX10 / 10);
      display.print('.');
      display.print(refPitchX10 % 10);
      break;
    case TPARAM_AVG_SPEED:
      display.print(F(">Avg:"));
      display.print(avgSpeed == 1 ? F("Fast") : (avgSpeed == 2 ? F("Med") : F("Slow")));
      break;
  }

  display.setCursor(SCREEN_WIDTH - 30, SCREEN_HEIGHT - 8);
  display.print(F("TUNE"));
}

// ============================================================================
// STARFIELD SCREENSAVER — integer math, reduced star count
// ============================================================================
void initStarfield() {
  for (int i = 0; i < NUM_STARS; i++) {
    stars[i].x = random(-200, 200);
    stars[i].y = random(-100, 100);
    stars[i].z = random(10, 200);
  }
}

void drawScreensaver() {
  display.clearDisplay();

  for (int i = 0; i < NUM_STARS; i++) {
    stars[i].z -= 4;
    if (stars[i].z <= 1) {
      stars[i].x = random(-200, 200);
      stars[i].y = random(-100, 100);
      stars[i].z = 200;
    }

    // Integer projection — no floats needed
    int sx = (int)((long)stars[i].x * 64 / stars[i].z) + SCREEN_WIDTH / 2;
    int sy = (int)((long)stars[i].y * 16 / stars[i].z) + SCREEN_HEIGHT / 2;

    if (sx >= 0 && sx < SCREEN_WIDTH && sy >= 0 && sy < SCREEN_HEIGHT) {
      if (stars[i].z < 50) {
        display.fillRect(sx, sy, 2, 2, WHITE);
      } else {
        display.drawPixel(sx, sy, WHITE);
      }
    }
  }

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(44, SCREEN_HEIGHT - 8);
  display.print(F("VISION"));
}

// ============================================================================
// EEPROM
// ============================================================================
void loadSettings() {
  timeBaseIdx = EEPROM.read(EE_TIMEBASE);
  if (timeBaseIdx >= NUM_TIMEBASES) timeBaseIdx = 8;

  vRangeIdx = EEPROM.read(EE_VRANGE);
  if (vRangeIdx >= NUM_VRANGES) vRangeIdx = 4;

  trigThreshold = (EEPROM.read(EE_TRIG_H) << 8) | EEPROM.read(EE_TRIG_L);
  if (trigThreshold > 1023) trigThreshold = 512;

  int8_t ofs = (int8_t)EEPROM.read(EE_OFFSET);
  if (ofs < -16 || ofs > 16) ofs = 0;
  displayOffset = ofs;

  currentMode = EEPROM.read(EE_MODE);
  if (currentMode >= MODE_COUNT) currentMode = MODE_SCOPE;

  tuneDisplay = EEPROM.read(EE_TUNEDISP);
  if (tuneDisplay >= TDISP_COUNT) tuneDisplay = TDISP_NOTE;

  refPitchX10 = (EEPROM.read(EE_REFPITCH_H) << 8) | EEPROM.read(EE_REFPITCH_L);
  if (refPitchX10 < 4000 || refPitchX10 > 4600) refPitchX10 = 4400;

  avgSpeed = EEPROM.read(EE_AVGSPEED);
  if (avgSpeed < 1 || avgSpeed > 3) avgSpeed = 2;
}

void saveSettings() {
  EEPROM.update(EE_TIMEBASE, timeBaseIdx);
  EEPROM.update(EE_VRANGE, vRangeIdx);
  EEPROM.update(EE_TRIG_H, trigThreshold >> 8);
  EEPROM.update(EE_TRIG_L, trigThreshold & 0xFF);
  EEPROM.update(EE_OFFSET, (uint8_t)displayOffset);
  EEPROM.update(EE_MODE, currentMode);
  EEPROM.update(EE_TUNEDISP, tuneDisplay);
  EEPROM.update(EE_REFPITCH_H, refPitchX10 >> 8);
  EEPROM.update(EE_REFPITCH_L, refPitchX10 & 0xFF);
  EEPROM.update(EE_AVGSPEED, avgSpeed);
}

// ============================================================================
// START SCREEN
// ============================================================================
void startScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(16, 0);
  display.println(F("MODULOVE"));
  display.setTextSize(1);
  display.setCursor(40, 20);
  display.print(F("VISION v2"));
  display.display();
  delay(1500);

  // Quick starfield burst
  for (int frame = 0; frame < 30; frame++) {
    drawScreensaver();
    display.display();
  }

  display.clearDisplay();
  display.display();
}
