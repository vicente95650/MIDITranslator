#include <USBHost_t36.h>

USBHost myusb;
USBHub hub1(myusb);
MIDIDevice casio(myusb);

const int modeSwitchPin = 2;          // Pin 2 -> GND = Dodeka mode
const int dublerPin = 3;              // Pin 3 -> GND = Dubler active
const byte CASIO_LIGHT_CHANNEL = 3;   // Casio LK-160 key-light Navigate Channel
const byte DUBLER_CC = 104;           // MIDI CC sent on Pin 3 state change

const int PHYSICAL_START_NOTE = 36;   // Leftmost physical white key on 61-key Casio (C2)
const int DODEKA_START_NOTE = 48;     // Output pitch for that leftmost key in Dodeka mode (C3)
const int NUM_WHITE_KEYS = 36;        // All 36 white keys (C2-C7 -> 3 full octaves: C3-B5)

// Minimum dark gap (ms) before relighting the exact same key
const unsigned long RETRIGGER_GAP_MS = 60;

const int whiteKeyGaps[] = {2, 2, 1, 2, 2, 2, 1};

int dodekaMap[128];
int reverseDodekaMap[128];

// Per-key LED state for repeated-note blink & stuck-light prevention
bool keyLit[128];
unsigned long keyOffTime[128];
bool pendingOn[128];
unsigned long pendingOnTime[128];
byte pendingVelocity[128];

// Mode switch & Dubler pin state
bool lastModeState = HIGH;
bool lastDublerReading = HIGH;
bool dublerActive = false;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_MS = 30;

// Returns true only for Synthesia's Right-Hand channels (1-indexed in Teensy usbMIDI)
bool isRightHand(byte channel) {
  return (channel >= 7 && channel <= 11) || (channel == 13);
}

void clearAllLights() {
  unsigned long now = millis();
  for (int i = 0; i < 128; i++) {
    if (keyLit[i] || pendingOn[i]) {
      pendingOn[i] = false;
      keyLit[i] = false;
      keyOffTime[i] = now;
      casio.sendNoteOff(i, 0, CASIO_LIGHT_CHANNEL);
    }
  }
  casio.sendControlChange(123, 0, CASIO_LIGHT_CHANNEL);
}

void setup() {
  pinMode(modeSwitchPin, INPUT_PULLUP);
  pinMode(dublerPin, INPUT_PULLUP);
  lastModeState = digitalRead(modeSwitchPin);

  for (int i = 0; i < 128; i++) {
    dodekaMap[i] = -1;
    reverseDodekaMap[i] = -1;
    keyLit[i] = false;
    keyOffTime[i] = 0;
    pendingOn[i] = false;
    pendingOnTime[i] = 0;
    pendingVelocity[i] = 0;
  }

  int currentCasioNote = PHYSICAL_START_NOTE;
  int dodekaNote = DODEKA_START_NOTE;

  // Map all 36 physical white keys (C2..C7) to 36 chromatic Dodeka notes (C3..B5)
  for (int i = 0; i < NUM_WHITE_KEYS; i++) {
    if (currentCasioNote < 128 && dodekaNote < 128) {
      dodekaMap[currentCasioNote] = dodekaNote;
      reverseDodekaMap[dodekaNote] = currentCasioNote;
    }
    dodekaNote++;
    currentCasioNote += whiteKeyGaps[i % 7];
  }

  myusb.begin();

  // Outgoing: Casio keys -> Mac (Synthesia / DAW)
  casio.setHandleNoteOn(casioToPcNoteOn);
  casio.setHandleNoteOff(casioToPcNoteOff);

  // Incoming: Mac (Synthesia / Dubler) -> Casio key lights
  usbMIDI.setHandleNoteOn(pcToCasioNoteOn);
  usbMIDI.setHandleNoteOff(pcToCasioNoteOff);
  usbMIDI.setHandleControlChange(pcToCasioControlChange);
}

void checkControlPins() {
  bool modeNow = digitalRead(modeSwitchPin);
  if (modeNow != lastModeState) {
    lastModeState = modeNow;
    clearAllLights();
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
      clearAllLights();
      usbMIDI.sendControlChange(DUBLER_CC, dublerActive ? 127 : 0, 1);
    }
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

void casioToPcNoteOn(byte channel, byte pitch, byte velocity) {
  if (velocity == 0) {
    casioToPcNoteOff(channel, pitch, 0);
    return;
  }

  if (digitalRead(modeSwitchPin) == HIGH) {
    usbMIDI.sendNoteOn(pitch, velocity, channel); // Mode 1: Standard Piano
  } else {
    int translatedNote = dodekaMap[pitch];
    if (translatedNote != -1) {
      usbMIDI.sendNoteOn(translatedNote, velocity, channel); // Mode 2: Dodeka
    }
  }
}

void casioToPcNoteOff(byte channel, byte pitch, byte velocity) {
  if (digitalRead(modeSwitchPin) == HIGH) {
    usbMIDI.sendNoteOff(pitch, 0, channel);
  } else {
    int translatedNote = dodekaMap[pitch];
    if (translatedNote != -1) {
      usbMIDI.sendNoteOff(translatedNote, 0, channel);
    }
  }
}

void pcToCasioNoteOn(byte channel, byte pitch, byte velocity) {
  if (velocity == 0) {
    pcToCasioNoteOff(channel, pitch, 0);
    return;
  }

  // When Dubler is idle, block left-hand/unassigned Synthesia channels
  if (!dublerActive && !isRightHand(channel)) return;

  int casioPitch = (digitalRead(modeSwitchPin) == HIGH) ? pitch : reverseDodekaMap[pitch];
  if (casioPitch < 0 || casioPitch >= 128) return;

  unsigned long now = millis();

  if (keyLit[casioPitch]) {
    // Key is still lit: force it off and schedule a crisp re-light
    casio.sendNoteOff(casioPitch, 0, CASIO_LIGHT_CHANNEL);
    keyLit[casioPitch] = false;
    keyOffTime[casioPitch] = now;
    pendingOn[casioPitch] = true;
    pendingOnTime[casioPitch] = now + RETRIGGER_GAP_MS;
    pendingVelocity[casioPitch] = velocity;
  } else if (keyOffTime[casioPitch] != 0 && (now - keyOffTime[casioPitch]) < RETRIGGER_GAP_MS) {
    // Key just turned off moments ago: wait out the rest of the blink gap
    pendingOn[casioPitch] = true;
    pendingOnTime[casioPitch] = keyOffTime[casioPitch] + RETRIGGER_GAP_MS;
    pendingVelocity[casioPitch] = velocity;
  } else {
    // Normal note: light immediately
    pendingOn[casioPitch] = false;
    casio.sendNoteOn(casioPitch, velocity, CASIO_LIGHT_CHANNEL);
    keyLit[casioPitch] = true;
  }
}

void pcToCasioNoteOff(byte channel, byte pitch, byte velocity) {
  int casioPitch = (digitalRead(modeSwitchPin) == HIGH) ? pitch : reverseDodekaMap[pitch];
  if (casioPitch < 0 || casioPitch >= 128) return;

  // Always honor Note Off for any lit or pending Casio key, regardless of channel
  if (!keyLit[casioPitch] && !pendingOn[casioPitch]) return;

  pendingOn[casioPitch] = false;
  keyLit[casioPitch] = false;
  keyOffTime[casioPitch] = millis();
  casio.sendNoteOff(casioPitch, 0, CASIO_LIGHT_CHANNEL);
}

void pcToCasioControlChange(byte channel, byte control, byte value) {
  // Handle All Sound Off (120) and All Notes Off (123) on any channel
  if (control == 120 || control == 123) {
    clearAllLights();
  }
}

void loop() {
  myusb.Task();
  while (casio.read()) {}
  while (usbMIDI.read()) {}
  checkControlPins();
  processPendingLights();
}