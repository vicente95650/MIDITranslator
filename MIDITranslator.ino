#include <USBHost_t36.h>

USBHost myusb;
USBHub hub1(myusb);
MIDIDevice casioLower(myusb);
MIDIDevice casioUpper(myusb);

const int modeSwitchPin = 2;          // Pin 2 -> GND = Dodeka mode
const int dublerPin = 3;              // Pin 3 -> GND = Dubler active
const byte CASIO_LIGHT_CHANNEL = 3;   // Casio LK-160 key-light Navigate Channel
const byte DUBLER_CC = 104;           // MIDI CC sent to Dubler

int startingMidiNote = 48;            // C3 (change to 36 if you start at C2)
int whiteKeyGaps[] = {2, 2, 1, 2, 2, 2, 1};

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

  int currentCasioNote = startingMidiNote;
  int dodekaNote = startingMidiNote;

  // Map 72 consecutive white keys
  for (int i = 0; i < 72; i++) {
    if (currentCasioNote < 128 && dodekaNote < 128) {
      dodekaMap[currentCasioNote] = dodekaNote;
      reverseDodekaMap[dodekaNote] = currentCasioNote;
    }
    dodekaNote++;
    currentCasioNote += whiteKeyGaps[i % 7];
  }

  myusb.begin();

  // Outgoing: Casio keys -> Mac (Synthesia / DAW)
  casioLower.setHandleNoteOn(casioToPcNoteOn);
  casioLower.setHandleNoteOff(casioToPcNoteOff);
  casioUpper.setHandleNoteOn(casioToPcNoteOn);
  casioUpper.setHandleNoteOff(casioToPcNoteOff);

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
    usbMIDI.sendNoteOn(pitch, velocity, channel); // Mode 1: Piano
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
    casioLower.sendNoteOn(pitch, velocity, CASIO_LIGHT_CHANNEL);
    casioUpper.sendNoteOn(pitch, velocity, CASIO_LIGHT_CHANNEL);
  } else {
    int casioPitch = reverseDodekaMap[pitch];
    if (casioPitch != -1) {
      casioLower.sendNoteOn(casioPitch, velocity, CASIO_LIGHT_CHANNEL);
      casioUpper.sendNoteOn(casioPitch, velocity, CASIO_LIGHT_CHANNEL);
    }
  }
}

void pcToCasioNoteOff(byte channel, byte pitch, byte velocity) {
  if (!dublerActive && !isRightHand(channel)) return;

  if (digitalRead(modeSwitchPin) == HIGH) {
    casioLower.sendNoteOff(pitch, 0, CASIO_LIGHT_CHANNEL);
    casioUpper.sendNoteOff(pitch, 0, CASIO_LIGHT_CHANNEL);
  } else {
    int casioPitch = reverseDodekaMap[pitch];
    if (casioPitch != -1) {
      casioLower.sendNoteOff(casioPitch, 0, CASIO_LIGHT_CHANNEL);
      casioUpper.sendNoteOff(casioPitch, 0, CASIO_LIGHT_CHANNEL);
    }
  }
}

void loop() {
  myusb.Task();
  casioLower.read();
  casioUpper.read();
  usbMIDI.read();
  checkDublerPin();
}