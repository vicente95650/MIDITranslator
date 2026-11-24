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

const int whiteKeyGaps[] = {2, 2, 1, 2, 2, 2, 1};

int dodekaMap[128];
int reverseDodekaMap[128];

// Edge-detection & debounce state for Pin 3
bool lastDublerReading = HIGH;
bool dublerActive = false;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_MS = 30;

// Returns true only for Synthesia's Right-Hand channels (1-indexed in Teensy usbMIDI)
bool isRightHand(byte channel) {
  return (channel >= 7 && channel <= 11) || (channel == 13);
}

void setup() {
  pinMode(modeSwitchPin, INPUT_PULLUP);
  pinMode(dublerPin, INPUT_PULLUP);

  for (int i = 0; i < 128; i++) {
    dodekaMap[i] = -1;
    reverseDodekaMap[i] = -1;
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
}

void checkDublerPin() {
  bool reading = digitalRead(dublerPin);

  if (reading != lastDublerReading) {
    lastDebounceTime = millis();
    lastDublerReading = reading;
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    bool isGrounded = (reading == LOW);
    if (isGrounded != dublerActive) {
      dublerActive = isGrounded;
      usbMIDI.sendControlChange(DUBLER_CC, dublerActive ? 127 : 0, 1);
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

  if (digitalRead(modeSwitchPin) == HIGH) {
    casio.sendNoteOn(pitch, velocity, CASIO_LIGHT_CHANNEL);
  } else {
    int casioPitch = reverseDodekaMap[pitch];
    if (casioPitch != -1) {
      casio.sendNoteOn(casioPitch, velocity, CASIO_LIGHT_CHANNEL);
    }
  }
}

void pcToCasioNoteOff(byte channel, byte pitch, byte velocity) {
  if (!dublerActive && !isRightHand(channel)) return;

  if (digitalRead(modeSwitchPin) == HIGH) {
    casio.sendNoteOff(pitch, 0, CASIO_LIGHT_CHANNEL);
  } else {
    int casioPitch = reverseDodekaMap[pitch];
    if (casioPitch != -1) {
      casio.sendNoteOff(casioPitch, 0, CASIO_LIGHT_CHANNEL);
    }
  }
}

void loop() {
  myusb.Task();
  casio.read();
  usbMIDI.read();
  checkDublerPin();
}