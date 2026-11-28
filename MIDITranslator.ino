/*
 * ============================================================================
 *  Casio LK-160 <-> Teensy 4.1 MIDI Bridge & Live OLED MIDI Analyzer
 *  (Dodeka + Synthesia + Dubler 2 + Melody Assist + Octave Shift + Range Alert)
 * ============================================================================
 *
 *  SERIAL COMMANDS (Tools -> USB Type -> Serial + MIDI):
 *    Open Arduino Serial Monitor and send a single character:
 *      - '?' or 'v' : Print firmware version, build timestamp, active Dodeka
 *                     range, switch states, OLED status, and any latched
 *                     out-of-range note alert.
 *      - 'O'        : Toggle Dodeka window between C3-B5 (default) and C4-B6.
 *      - '+' / '-'  : Shift Dodeka window up or down by 1 octave (C2..C5 base).
 *      - 'C'        : Clear the latched Range-Exceeded indicator (!HI / !LO).
 *      - 'L'        : Toggle OLED analyzer between Layout A (Fixed Slots)
 *                     and Layout B (4-Line Scrolling Log).
 *
 *  OPTIONAL 1.3" SH1106 I2C OLED DISPLAY:
 *    - Set ENABLE_OLED to 0 to compile without any display libraries.
 *    - Set ENABLE_OLED to 1 after installing "Adafruit SH110X" (+ "Adafruit GFX").
 *    - Wiring (check silkscreen labels on your OLED PCB):
 *        * VCC -> Teensy 3.3V   (do NOT use 5V)
 *        * GND -> Teensy GND
 *        * SDA -> Teensy Pin 18
 *        * SCK -> Teensy Pin 19
 *    - Auto-detects screen at boot; runs normally with zero overhead if absent.
 *
 *  HARDWARE SWITCHES (All use INPUT_PULLUP: OPEN = HIGH, GROUNDED = LOW):
 *    - Pin 2 (Keyboard Mode) : OPEN = 61-key Piano | GND = 36-key Dodeka
 *    - Pin 3 (Dubler Gate)   : OPEN = Ch 1 blocked | GND = Ch 1 allowed (+ CC #104)
 *    - Pin 4 (Hand Filter)   : OPEN = Both Hands   | GND = Right Hand Only
 *    - Pin 5 (Melody Assist) : OPEN = OFF          | GND = ON
 *    - Pin 6 (Harmony Sound) : (Active only when Pin 5 is GND)
 *                              OPEN = Silent Assist (highest LED only, vel = 1)
 *                              GND  = Sound Assist  (all LEDs, full velocity)
 * ============================================================================
 */

// ----------------------------------------------------------------------------
// 0. FIRMWARE VERSION, OLED ENABLE, & ANALYZER LAYOUT OPTIONS
// ----------------------------------------------------------------------------
#define FW_NAME     "Casio LK-160 Bridge"
#define FW_VERSION  "v2.3"

// Change to 1 after installing "Adafruit SH110X" and "Adafruit GFX Library"
#define ENABLE_OLED 0

// OLED Analyzer Layout Options:
//   OLED_LAYOUT_SLOTS (1) = Layout A: Fixed rows (IN / FLT / OUT / AUT)
//   OLED_LAYOUT_LOG   (2) = Layout B: 4-line chronological scrolling event log
#define OLED_LAYOUT_SLOTS   1
#define OLED_LAYOUT_LOG     2
#define DEFAULT_OLED_LAYOUT OLED_LAYOUT_SLOTS

#include <USBHost_t36.h>
#include <stdio.h>
#include <string.h>

#if ENABLE_OLED
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SH110X.h>
#endif

// ----------------------------------------------------------------------------
// 1. USB HOST OBJECTS (Teensy talking to the Casio LK-160)
// ----------------------------------------------------------------------------
USBHost myusb;
USBHub hub1(myusb);
MIDIDevice casio(myusb);

// ----------------------------------------------------------------------------
// 2. HARDWARE PINS & MIDI CHANNEL CONFIGURATION
// ----------------------------------------------------------------------------
const int modeSwitchPin   = 2;        // OPEN = Piano,        GND = Dodeka
const int dublerPin       = 3;        // OPEN = Dubler Off,   GND = Dubler On
const int handFilterPin   = 4;        // OPEN = Both Hands,   GND = Right Hand Only
const int melodyAssistPin = 5;        // OPEN = Assist Off,   GND = Assist On
const int harmonySoundPin = 6;        // OPEN = Silent (v=1), GND = Sound (full vel)

const byte CASIO_LIGHT_CHANNEL = 3;   // Casio LK-160 key-light Navigate Channel
const byte DUBLER_CHANNEL      = 1;   // Expected MIDI channel from Dubler 2
const byte DUBLER_CC           = 104; // MIDI CC sent to Mac when Pin 3 toggles
const byte SILENT_VELOCITY     = 1;   // Lowest valid Note On (inaudible in Synthesia)

// ----------------------------------------------------------------------------
// 3. KEYBOARD RANGE, DODEKA MAPPING, & BLINK TIMING
// ----------------------------------------------------------------------------
const int PHYSICAL_START_NOTE = 36;   // Leftmost key on 61-key Casio LK-160 (C2)
const int PHYSICAL_END_NOTE   = 96;   // Rightmost key on 61-key Casio LK-160 (C7)
const int NUM_WHITE_KEYS      = 36;   // 36 white keys (C2..C7) -> 3 chromatic octaves

const int DEFAULT_DODEKA_START = 48;  // C3 (window: C3..B5)
const int ALT_DODEKA_START     = 60;  // C4 (window: C4..B6)
const int MIN_DODEKA_START     = 36;  // C2 (lowest allowed shift via '-')
const int MAX_DODEKA_START     = 72;  // C5 (highest allowed shift via '+')

int currentDodekaStart = DEFAULT_DODEKA_START;

const unsigned long RETRIGGER_GAP_MS = 60; // Dark blink gap (ms) for repeated notes
const int whiteKeyGaps[] = {2, 2, 1, 2, 2, 2, 1};

int dodekaMap[128];                   // physicalCasioPitch -> musicalPitch (-1 = ignore)
int reverseDodekaMap[128];            // musicalPitch -> physicalCasioPitch (-1 = out of range)

const char* NOTE_NAMES[12] = {
  "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

// ----------------------------------------------------------------------------
// 4. LATCHED RANGE-EXCEEDED TRACKER (Cleared via Serial 'C' or Octave Change)
// ----------------------------------------------------------------------------
bool oorLow       = false;            // True if a playable note fell below range
bool oorHigh      = false;            // True if a playable note exceeded range
int  oorMinPitch  = 128;              // Lowest out-of-range pitch captured
int  oorMaxPitch  = -1;               // Highest out-of-range pitch captured

// ----------------------------------------------------------------------------
// 5. PER-KEY CASIO LED STATE (indexed by physical Casio MIDI note 0..127)
// ----------------------------------------------------------------------------
bool keyLit[128];
unsigned long keyOffTime[128];
bool pendingOn[128];
unsigned long pendingOnTime[128];
byte pendingVelocity[128];

// ----------------------------------------------------------------------------
// 6. MELODY ASSIST TRACKER (indexed by musical MIDI pitch 0..127)
// ----------------------------------------------------------------------------
bool synthesiaActive[128];
byte synthesiaVelocity[128];
bool autoSatisfied[128];
bool autoNoteOnSent[128];

// ----------------------------------------------------------------------------
// 7. SWITCH STATE & DEBOUNCING
// ----------------------------------------------------------------------------
bool lastModeState         = HIGH;
bool lastHandFilterState   = HIGH;
bool lastMelodyAssistState = HIGH;
bool lastHarmonySoundState = HIGH;

bool lastDublerReading        = HIGH;
bool dublerActive             = false;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_MS = 30;

// ----------------------------------------------------------------------------
// 8. OPTIONAL SH1106 OLED & MIDI ANALYZER STATE
// ----------------------------------------------------------------------------
byte currentOledLayout = DEFAULT_OLED_LAYOUT;

#if ENABLE_OLED
  Adafruit_SH1106G display(128, 64, &Wire, -1);
  bool oledPresent              = false;
  byte oledAddr                 = 0x3C;
  bool splashActive             = false;
  unsigned long splashEndMs     = 0;
  bool oledDirty                = false;
  unsigned long lastOledDrawMs  = 0;

  const unsigned long SPLASH_DURATION_MS  = 2000;
  const unsigned long OLED_MIN_REFRESH_MS = 80;

  // Layout A: Fixed Slot text buffers
  char slotIn[24]  = "IN  ---";
  char slotFlt[24] = "FLT ---";
  char slotOut[24] = "OUT ---";
  char slotAut[24] = "AUT ---";

  // Layout B: 4-Line chronological scrolling log
  char logLines[4][24] = { "", "", "", "" };
#endif

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void clearAllLightsAndState();

// ============================================================================
// HELPER FUNCTIONS: MODE, RANGE, & NOTE FORMATTING
// ============================================================================

bool isPianoMode()        { return digitalRead(modeSwitchPin)   == HIGH; }
bool isBothHandsAllowed() { return digitalRead(handFilterPin)   == HIGH; }
bool isMelodyAssistOn()   { return digitalRead(melodyAssistPin) == LOW;  }
bool isHarmonySoundOn()   { return digitalRead(harmonySoundPin) == LOW;  }
bool isSilentAssist()     { return isMelodyAssistOn() && !isHarmonySoundOn(); }

bool isLeftHand(byte channel)  { return (channel >= 2 && channel <= 6)  || (channel == 12); }
bool isRightHand(byte channel) { return (channel >= 7 && channel <= 11) || (channel == 13); }

const char* getAssistModeLabel() {
  if (!isMelodyAssistOn()) return "OFF";
  return isHarmonySoundOn() ? "SOUND" : "SILENT";
}

const char* getAssistShortLabel() {
  if (!isMelodyAssistOn()) return "OFF";
  return isHarmonySoundOn() ? "SND" : "SIL";
}

// Formats a MIDI pitch (0..127) as "C3", "G#4", etc.
void formatNoteName(byte pitch, char* out, size_t outSize) {
  int octave = (pitch / 12) - 1;
  snprintf(out, outSize, "%s%d", NOTE_NAMES[pitch % 12], octave);
}

int getMinPlayablePitch() {
  return isPianoMode() ? PHYSICAL_START_NOTE : currentDodekaStart;
}

int getMaxPlayablePitch() {
  return isPianoMode() ? PHYSICAL_END_NOTE : (currentDodekaStart + NUM_WHITE_KEYS - 1);
}

// Maps musical pitch to physical Casio key (-1 if outside physical range)
int musicalToCasioPitch(int musicalPitch) {
  if (musicalPitch < 0 || musicalPitch >= 128) return -1;
  if (isPianoMode()) {
    if (musicalPitch < PHYSICAL_START_NOTE || musicalPitch > PHYSICAL_END_NOTE) return -1;
    return musicalPitch;
  }
  return reverseDodekaMap[musicalPitch];
}

int getHighestActiveSynthesiaNote() {
  for (int p = 127; p >= 0; p--) {
    if (synthesiaActive[p]) return p;
  }
  return -1;
}

// Rebuilds the bidirectional 36-key Dodeka lookup tables for currentDodekaStart
void rebuildDodekaMap() {
  for (int i = 0; i < 128; i++) {
    dodekaMap[i]        = -1;
    reverseDodekaMap[i] = -1;
  }

  int currentCasioNote = PHYSICAL_START_NOTE;
  int dodekaNote       = currentDodekaStart;
  for (int i = 0; i < NUM_WHITE_KEYS; i++) {
    if (currentCasioNote < 128 && dodekaNote < 128) {
      dodekaMap[currentCasioNote]  = dodekaNote;
      reverseDodekaMap[dodekaNote] = currentCasioNote;
    }
    dodekaNote++;
    currentCasioNote += whiteKeyGaps[i % 7];
  }
}

void clearRangeAlert() {
  oorLow      = false;
  oorHigh     = false;
  oorMinPitch = 128;
  oorMaxPitch = -1;
#if ENABLE_OLED
  splashActive = false;
  oledDirty    = true;
#endif
}

// Latches low/high range-exceeded flags when a playable note falls outside range
void latchOutOfRange(byte pitch) {
  if (pitch < getMinPlayablePitch()) {
    oorLow = true;
    if (pitch < oorMinPitch) oorMinPitch = pitch;
  } else if (pitch > getMaxPlayablePitch()) {
    oorHigh = true;
    if (pitch > oorMaxPitch) oorMaxPitch = pitch;
  }
#if ENABLE_OLED
  oledDirty = true;
#endif
}

// ============================================================================
// HELPER FUNCTIONS: MIDI ANALYZER RECORDING & OLED RENDERING
// ============================================================================

#if ENABLE_OLED
void pushLogLine(const char* line) {
  for (int i = 0; i < 3; i++) {
    strncpy(logLines[i], logLines[i + 1], sizeof(logLines[i]) - 1);
    logLines[i][sizeof(logLines[i]) - 1] = '\0';
  }
  strncpy(logLines[3], line, sizeof(logLines[3]) - 1);
  logLines[3][sizeof(logLines[3]) - 1] = '\0';
}
#endif

void recordMidiEvent(const char* dirTag, byte channel, byte pitch, const char* detail) {
#if ENABLE_OLED
  if (!oledPresent) return;

  char noteStr[8];
  formatNoteName(pitch, noteStr, sizeof(noteStr));

  char line[24];
  snprintf(line, sizeof(line), "%-3s C%02d %s(%d) %s", dirTag, channel, noteStr, pitch, detail);

  if (strcmp(dirTag, "IN") == 0) {
    strncpy(slotIn, line, sizeof(slotIn) - 1);
    slotIn[sizeof(slotIn) - 1] = '\0';
  } else if (strcmp(dirTag, "FLT") == 0) {
    strncpy(slotFlt, line, sizeof(slotFlt) - 1);
    slotFlt[sizeof(slotFlt) - 1] = '\0';
  } else if (strcmp(dirTag, "OUT") == 0) {
    strncpy(slotOut, line, sizeof(slotOut) - 1);
    slotOut[sizeof(slotOut) - 1] = '\0';
  } else if (strcmp(dirTag, "AUT") == 0) {
    strncpy(slotAut, line, sizeof(slotAut) - 1);
    slotAut[sizeof(slotAut) - 1] = '\0';
  }

  pushLogLine(line);
  oledDirty = true;
#else
  (void)dirTag; (void)channel; (void)pitch; (void)detail;
#endif
}

void drawOledScreen() {
#if ENABLE_OLED
  if (!oledPresent) return;

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setTextWrap(false);

  // --- Top Compact Status Bar ---
  // Left/Center: Mode + Base Note + Hands + Assist + Dubler (e.g. "DDK C3 BH SIL D0")
  char startStr[6];
  if (isPianoMode()) {
    snprintf(startStr, sizeof(startStr), "61");
  } else {
    formatNoteName((byte)currentDodekaStart, startStr, sizeof(startStr));
  }

  char header[20];
  snprintf(header, sizeof(header), "%s %s %s %s D%d",
           isPianoMode() ? "PIA" : "DDK",
           startStr,
           isBothHandsAllowed() ? "BH" : "RH",
           getAssistShortLabel(),
           dublerActive ? 1 : 0);

  display.setCursor(0, 1);
  display.print(header);

  // Top-Right Corner: Inverted Range Alert (!HI / !LO / !HL) OR Layout (L:A / L:B)
  if (oorLow || oorHigh) {
    const char* badge = (oorLow && oorHigh) ? "!HL" : (oorHigh ? "!HI" : "!LO");
    display.fillRect(104, 0, 24, 9, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK);
    display.setCursor(107, 1);
    display.print(badge);
    display.setTextColor(SH110X_WHITE);
  } else {
    display.setCursor(109, 1);
    display.print((currentOledLayout == OLED_LAYOUT_SLOTS) ? "L:A" : "L:B");
  }

  display.drawFastHLine(0, 11, 128, SH110X_WHITE);

  // --- Analyzer Body (4 lines) ---
  if (currentOledLayout == OLED_LAYOUT_SLOTS) {
    display.setCursor(0, 15); display.print(slotIn);
    display.setCursor(0, 28); display.print(slotFlt);
    display.setCursor(0, 41); display.print(slotOut);
    display.setCursor(0, 54); display.print(slotAut);
  } else {
    display.setCursor(0, 15); display.print(logLines[0]);
    display.setCursor(0, 28); display.print(logLines[1]);
    display.setCursor(0, 41); display.print(logLines[2]);
    display.setCursor(0, 54); display.print(logLines[3]);
  }

  display.display();
  oledDirty = false;
  lastOledDrawMs = millis();
#endif
}

void initOled() {
#if ENABLE_OLED
  Wire.begin();
  Wire.setClock(400000);

  const byte candidateAddrs[2] = {0x3C, 0x3D};
  for (int i = 0; i < 2; i++) {
    Wire.beginTransmission(candidateAddrs[i]);
    if (Wire.endTransmission() == 0) {
      oledAddr = candidateAddrs[i];
      oledPresent = true;
      break;
    }
  }

  if (!oledPresent) return;

  if (!display.begin(oledAddr, true)) {
    oledPresent = false;
    return;
  }

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(2);
  display.setCursor(10, 4);
  display.println("CASIO");
  display.setTextSize(1);
  display.setCursor(10, 24);
  display.print("Bridge ");
  display.println(FW_VERSION);
  display.setCursor(10, 38);
  display.println(__DATE__);
  display.setCursor(10, 48);
  display.println(__TIME__);
  display.display();

  splashActive = true;
  splashEndMs  = millis() + SPLASH_DURATION_MS;
#endif
}

void serviceOled() {
#if ENABLE_OLED
  if (!oledPresent) return;
  unsigned long now = millis();

  if (splashActive) {
    if ((long)(now - splashEndMs) >= 0) {
      splashActive = false;
      oledDirty = true;
    } else {
      return;
    }
  }

  if (oledDirty && (now - lastOledDrawMs) >= OLED_MIN_REFRESH_MS) {
    drawOledScreen();
  }
#endif
}

// ============================================================================
// HELPER FUNCTIONS: SERIAL COMMANDS ('?', 'O', '+', '-', 'C', 'L')
// ============================================================================

void printActiveRange() {
  char lowStr[8], highStr[8];
  formatNoteName((byte)getMinPlayablePitch(), lowStr, sizeof(lowStr));
  formatNoteName((byte)getMaxPlayablePitch(), highStr, sizeof(highStr));
  Serial.print(lowStr);
  Serial.print("-");
  Serial.print(highStr);
  Serial.print(" (MIDI ");
  Serial.print(getMinPlayablePitch());
  Serial.print("-");
  Serial.print(getMaxPlayablePitch());
  Serial.print(")");
}

void printFirmwareInfo() {
  Serial.println();
  Serial.println("========================================");
  Serial.print("Firmware : ");
  Serial.print(FW_NAME);
  Serial.print(" ");
  Serial.println(FW_VERSION);
  Serial.print("Built    : ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.println(__TIME__);

  Serial.print("OLED     : ");
#if ENABLE_OLED
  if (oledPresent) {
    Serial.print("Detected at 0x");
    Serial.print(oledAddr, HEX);
    Serial.print(" | Layout ");
    Serial.println(currentOledLayout == OLED_LAYOUT_SLOTS
                     ? "A (Fixed Slots)"
                     : "B (Scrolling Log)");
  } else {
    Serial.println("Enabled in code, but no screen detected");
  }
#else
  Serial.println("Disabled (set ENABLE_OLED 1 when installed)");
#endif

  Serial.print("Switches : ");
  Serial.print(isPianoMode() ? "Piano" : "Dodeka");
  Serial.print(" | Hands: ");
  Serial.print(isBothHandsAllowed() ? "Both" : "RH Only");
  Serial.print(" | Assist: ");
  Serial.print(getAssistModeLabel());
  Serial.print(" | Dubler: ");
  Serial.println(dublerActive ? "ON" : "OFF");

  Serial.print("Range    : ");
  printActiveRange();
  Serial.println();

  Serial.print("OOR Alert: ");
  if (!oorLow && !oorHigh) {
    Serial.println("OK (no notes exceeded range)");
  } else {
    char noteStr[8];
    if (oorLow) {
      formatNoteName((byte)oorMinPitch, noteStr, sizeof(noteStr));
      Serial.print("LOW (min ");
      Serial.print(noteStr);
      Serial.print("/");
      Serial.print(oorMinPitch);
      Serial.print(") ");
    }
    if (oorHigh) {
      formatNoteName((byte)oorMaxPitch, noteStr, sizeof(noteStr));
      Serial.print("HIGH (max ");
      Serial.print(noteStr);
      Serial.print("/");
      Serial.print(oorMaxPitch);
      Serial.print(") ");
    }
    Serial.println("-- send 'C' to clear");
  }

  Serial.println("Commands : '?'=Status | 'O'=C3/C4 | '+'/'-'=Octave | 'C'=Clear OOR | 'L'=Layout");
  Serial.println("========================================");
}

void applyDodekaStart(int newStart) {
  if (newStart < MIN_DODEKA_START) newStart = MIN_DODEKA_START;
  if (newStart > MAX_DODEKA_START) newStart = MAX_DODEKA_START;

  currentDodekaStart = newStart;
  clearAllLightsAndState();
  rebuildDodekaMap();
  clearRangeAlert();

  Serial.print("Dodeka range set to: ");
  printActiveRange();
  Serial.println();
}

void checkSerialQuery() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '?' || c == 'v' || c == 'V') {
      printFirmwareInfo();
    } else if (c == 'o' || c == 'O') {
      int nextStart = (currentDodekaStart == DEFAULT_DODEKA_START)
                        ? ALT_DODEKA_START
                        : DEFAULT_DODEKA_START;
      applyDodekaStart(nextStart);
    } else if (c == '+' || c == '=') {
      applyDodekaStart(currentDodekaStart + 12);
    } else if (c == '-' || c == '_') {
      applyDodekaStart(currentDodekaStart - 12);
    } else if (c == 'c' || c == 'C') {
      clearRangeAlert();
      Serial.println("Range-exceeded alert cleared.");
    } else if (c == 'l' || c == 'L') {
      currentOledLayout = (currentOledLayout == OLED_LAYOUT_SLOTS)
                            ? OLED_LAYOUT_LOG
                            : OLED_LAYOUT_SLOTS;
      Serial.print("OLED Layout switched to: ");
      Serial.println(currentOledLayout == OLED_LAYOUT_SLOTS
                       ? "A (Fixed Slots)"
                       : "B (4-Line Scrolling Log)");
#if ENABLE_OLED
      splashActive = false;
      oledDirty = true;
#endif
    }
  }
}

// ============================================================================
// HELPER FUNCTIONS: CASIO KEY-LIGHT CONTROL (WITH 60 ms BLINK)
// ============================================================================

void lightPhysicalKey(int casioPitch, byte velocity) {
  if (casioPitch < 0 || casioPitch >= 128) return;
  unsigned long now = millis();

  if (keyLit[casioPitch]) {
    casio.sendNoteOff(casioPitch, 0, CASIO_LIGHT_CHANNEL);
    keyLit[casioPitch] = false;
    keyOffTime[casioPitch] = now;
    pendingOn[casioPitch] = true;
    pendingOnTime[casioPitch] = now + RETRIGGER_GAP_MS;
    pendingVelocity[casioPitch] = velocity;
  } else if (keyOffTime[casioPitch] != 0 && (now - keyOffTime[casioPitch]) < RETRIGGER_GAP_MS) {
    pendingOn[casioPitch] = true;
    pendingOnTime[casioPitch] = keyOffTime[casioPitch] + RETRIGGER_GAP_MS;
    pendingVelocity[casioPitch] = velocity;
  } else {
    pendingOn[casioPitch] = false;
    casio.sendNoteOn(casioPitch, velocity, CASIO_LIGHT_CHANNEL);
    keyLit[casioPitch] = true;
  }
}

void extinguishPhysicalKey(int casioPitch) {
  if (casioPitch < 0 || casioPitch >= 128) return;
  if (!keyLit[casioPitch] && !pendingOn[casioPitch]) return;

  pendingOn[casioPitch] = false;
  keyLit[casioPitch] = false;
  keyOffTime[casioPitch] = millis();
  casio.sendNoteOff(casioPitch, 0, CASIO_LIGHT_CHANNEL);
}

void clearAllLightsAndState() {
  unsigned long now = millis();
  for (int i = 0; i < 128; i++) {
    if (keyLit[i] || pendingOn[i]) {
      pendingOn[i] = false;
      keyLit[i] = false;
      keyOffTime[i] = now;
      casio.sendNoteOff(i, 0, CASIO_LIGHT_CHANNEL);
    }
    if (autoNoteOnSent[i]) {
      usbMIDI.sendNoteOff(i, 0, 1);
      autoNoteOnSent[i] = false;
    }
    synthesiaActive[i] = false;
    synthesiaVelocity[i] = 0;
    autoSatisfied[i] = false;
  }
  casio.sendControlChange(123, 0, CASIO_LIGHT_CHANNEL);
}

void checkControlPins() {
  bool modeNow   = digitalRead(modeSwitchPin);
  bool handNow   = digitalRead(handFilterPin);
  bool assistNow = digitalRead(melodyAssistPin);
  bool soundNow  = digitalRead(harmonySoundPin);
  bool stateChanged = false;

  if (modeNow != lastModeState || handNow != lastHandFilterState ||
      assistNow != lastMelodyAssistState || soundNow != lastHarmonySoundState) {
    if (modeNow != lastModeState) {
      clearRangeAlert(); // Switching Piano <-> Dodeka changes playable range
    }
    lastModeState         = modeNow;
    lastHandFilterState   = handNow;
    lastMelodyAssistState = assistNow;
    lastHarmonySoundState = soundNow;
    clearAllLightsAndState();
    stateChanged = true;
  }

  bool reading = digitalRead(dublerPin);
  if (reading != lastDublerReading) {
    lastDebounceTime = millis();
    lastDublerReading = reading;
  }
  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    bool isGrounded = (reading == LOW);
    if (isGrounded != dublerActive) {
      dublerActive = isGrounded;
      clearAllLightsAndState();
      usbMIDI.sendControlChange(DUBLER_CC, dublerActive ? 127 : 0, 1);
      stateChanged = true;
    }
  }

  if (stateChanged) {
#if ENABLE_OLED
    splashActive = false;
    oledDirty = true;
#endif
  }
}

void processPendingLights() {
  unsigned long now = millis();
  for (int i = 0; i < 128; i++) {
    if (pendingOn[i] && (long)(now - pendingOnTime[i]) >= 0) {
      pendingOn[i] = false;
      casio.sendNoteOn(i, pendingVelocity[i], CASIO_LIGHT_CHANNEL);
      keyLit[i] = true;
    }
  }
}

// ============================================================================
// OUTGOING MIDI CALLBACKS: Casio LK-160 Keys -> Mac (Synthesia / DAW)
// ============================================================================

void casioToPcNoteOn(byte channel, byte pitch, byte velocity) {
  if (velocity == 0) {
    casioToPcNoteOff(channel, pitch, 0);
    return;
  }

  int musicalPitch = isPianoMode() ? pitch : dodekaMap[pitch];
  if (musicalPitch == -1) return;

  usbMIDI.sendNoteOn(musicalPitch, velocity, channel);

  char velBuf[8];
  snprintf(velBuf, sizeof(velBuf), "v%d", velocity);
  recordMidiEvent("OUT", channel, (byte)musicalPitch, velBuf);

  if (isMelodyAssistOn()) {
    int highest = getHighestActiveSynthesiaNote();
    if (highest != -1 && musicalPitch == highest) {
      byte autoVel = isHarmonySoundOn() ? velocity : SILENT_VELOCITY;
      char autoVelBuf[8];
      snprintf(autoVelBuf, sizeof(autoVelBuf), "v%d", autoVel);

      for (int p = 0; p < highest; p++) {
        if (synthesiaActive[p] && !autoSatisfied[p]) {
          usbMIDI.sendNoteOn(p, autoVel, channel);
          autoSatisfied[p]  = true;
          autoNoteOnSent[p] = true;
          recordMidiEvent("AUT", channel, (byte)p, autoVelBuf);
        }
      }
    }
  }
}

void casioToPcNoteOff(byte channel, byte pitch, byte velocity) {
  int musicalPitch = isPianoMode() ? pitch : dodekaMap[pitch];
  if (musicalPitch == -1) return;

  usbMIDI.sendNoteOff(musicalPitch, 0, channel);
}

// ============================================================================
// INCOMING MIDI CALLBACKS: Mac (Synthesia / Dubler 2) -> Casio Key Lights
// ============================================================================

void pcToCasioNoteOn(byte channel, byte pitch, byte velocity) {
  if (velocity == 0) {
    pcToCasioNoteOff(channel, pitch, 0);
    return;
  }

  char velBuf[8];
  snprintf(velBuf, sizeof(velBuf), "v%d", velocity);

  // Case 1: Dubler 2 (Channel 1) bypasses Melody Assist chord tracking
  if (dublerActive && channel == DUBLER_CHANNEL) {
    int casioKey = musicalToCasioPitch(pitch);
    if (casioKey != -1) {
      lightPhysicalKey(casioKey, velocity);
      recordMidiEvent("IN", channel, pitch, velBuf);
    } else {
      latchOutOfRange(pitch);
      recordMidiEvent("FLT", channel, pitch, "OOR");
    }
    return;
  }

  // Case 2: Synthesia Channel Gatekeeper (controlled by Pin 4)
  bool allowSynthesia = isRightHand(channel) || (isBothHandsAllowed() && isLeftHand(channel));
  if (!allowSynthesia) {
    recordMidiEvent("FLT", channel, pitch, "BLK");
    return;
  }

  synthesiaActive[pitch]   = true;
  synthesiaVelocity[pitch] = velocity;
  autoSatisfied[pitch]     = false;

  if (isSilentAssist()) {
    int highest = getHighestActiveSynthesiaNote();
    if (pitch == highest) {
      for (int p = 0; p < highest; p++) {
        if (synthesiaActive[p]) {
          extinguishPhysicalKey(musicalToCasioPitch(p));
        }
      }
      int casioKey = musicalToCasioPitch(pitch);
      if (casioKey != -1) {
        lightPhysicalKey(casioKey, velocity);
        recordMidiEvent("IN", channel, pitch, velBuf);
      } else {
        latchOutOfRange(pitch);
        recordMidiEvent("FLT", channel, pitch, "OOR");
      }
    } else {
      // Lower note kept dark in Silent Assist (auto-pressed silently; never triggers OOR)
      recordMidiEvent("IN", channel, pitch, "HID");
    }
  } else {
    int casioKey = musicalToCasioPitch(pitch);
    if (casioKey != -1) {
      lightPhysicalKey(casioKey, velocity);
      recordMidiEvent("IN", channel, pitch, velBuf);
    } else {
      latchOutOfRange(pitch);
      recordMidiEvent("FLT", channel, pitch, "OOR");
    }
  }
}

void pcToCasioNoteOff(byte channel, byte pitch, byte velocity) {
  if (autoNoteOnSent[pitch]) {
    usbMIDI.sendNoteOff(pitch, 0, 1);
    autoNoteOnSent[pitch] = false;
  }

  synthesiaActive[pitch]   = false;
  synthesiaVelocity[pitch] = 0;
  autoSatisfied[pitch]     = false;

  extinguishPhysicalKey(musicalToCasioPitch(pitch));

  if (isSilentAssist()) {
    int newHighest = getHighestActiveSynthesiaNote();
    if (newHighest != -1) {
      int casioKey = musicalToCasioPitch(newHighest);
      if (casioKey != -1 && !keyLit[casioKey] && !pendingOn[casioKey]) {
        lightPhysicalKey(casioKey, synthesiaVelocity[newHighest]);
      }
    }
  }
}

void pcToCasioControlChange(byte channel, byte control, byte value) {
  if (control == 120 || control == 123) {
    // Clear active lights & chord state, but keep latched OOR alert visible
    clearAllLightsAndState();
  }
}

// ============================================================================
// ARDUINO SETUP & MAIN LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);

  pinMode(modeSwitchPin,   INPUT_PULLUP);
  pinMode(dublerPin,       INPUT_PULLUP);
  pinMode(handFilterPin,   INPUT_PULLUP);
  pinMode(melodyAssistPin, INPUT_PULLUP);
  pinMode(harmonySoundPin, INPUT_PULLUP);

  lastModeState         = digitalRead(modeSwitchPin);
  lastHandFilterState   = digitalRead(handFilterPin);
  lastMelodyAssistState = digitalRead(melodyAssistPin);
  lastHarmonySoundState = digitalRead(harmonySoundPin);
  lastDublerReading     = digitalRead(dublerPin);
  dublerActive          = (lastDublerReading == LOW);

  for (int i = 0; i < 128; i++) {
    keyLit[i]            = false;
    keyOffTime[i]        = 0;
    pendingOn[i]         = false;
    pendingOnTime[i]     = 0;
    pendingVelocity[i]   = 0;
    synthesiaActive[i]   = false;
    synthesiaVelocity[i] = 0;
    autoSatisfied[i]     = false;
    autoNoteOnSent[i]    = false;
  }

  rebuildDodekaMap();
  initOled();

  myusb.begin();

  casio.setHandleNoteOn(casioToPcNoteOn);
  casio.setHandleNoteOff(casioToPcNoteOff);

  usbMIDI.setHandleNoteOn(pcToCasioNoteOn);
  usbMIDI.setHandleNoteOff(pcToCasioNoteOff);
  usbMIDI.setHandleControlChange(pcToCasioControlChange);
}

void loop() {
  myusb.Task();                    // Service Teensy USB Host hardware
  while (casio.read()) {}          // Drain all queued MIDI messages from Casio
  while (usbMIDI.read()) {}        // Drain all queued MIDI messages from Mac
  checkControlPins();              // Poll Pins 2, 3, 4, 5, and 6
  processPendingLights();          // Fire any completed 60 ms repeated-note blinks
  serviceOled();                   // Rate-limited OLED splash & MIDI analyzer refresh
  checkSerialQuery();              // Handle '?', 'O', '+', '-', 'C', and 'L'
}