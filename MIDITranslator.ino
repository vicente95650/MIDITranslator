/*
 * ============================================================================
 *  Casio LK-160 <-> Teensy 4.1 MIDI Bridge (Dodeka + Synthesia + Dubler 2)
 * ============================================================================
 *
 *  PURPOSE:
 *    Sits between a 61-key Casio LK-160 keyboard (on Teensy's USB Host port)
 *    and a Mac running Synthesia and/or Dubler 2 (on Teensy's USB Device port).
 *    Translates notes bidirectionally between a standard 61-key layout and a
 *    36-key isomorphic Dodeka layout, filters incoming key-light MIDI channels,
 *    blinks repeated notes so they are visually distinct, and prevents stuck LEDs.
 *
 *  HARDWARE & WIRING ASSUMPTIONS:
 *    - Board: Teensy 4.1 (Arduino IDE -> Tools -> USB Type -> MIDI).
 *    - USB Host Port (5-pin header): Connected to Casio LK-160 USB-B port.
 *    - USB Device Port (Micro-USB):  Connected to Mac.
 *    - Pin 2 (Mode Switch):  Wired to GND via switch/jumper (uses internal pull-up).
 *        * OPEN (HIGH)     = Standard 61-key Piano mode (1:1 pass-through).
 *        * GROUNDED (LOW)  = 36-key Dodeka mode (white keys only -> C3..B5).
 *    - Pin 3 (Dubler Gate):  Wired to GND via switch/pedal/jumper (internal pull-up).
 *        * OPEN (HIGH)     = Dubler 2 channel blocked; sends CC #104 = 0 on Ch 1.
 *        * GROUNDED (LOW)  = Dubler 2 channel allowed; sends CC #104 = 127 on Ch 1.
 *
 *  CASIO LK-160 ASSUMPTIONS:
 *    - 61 keys spanning physical MIDI 36 (C2) to MIDI 96 (C7) = 36 white keys.
 *    - Key-lighting hardware listens on Navigate Channel 3 (CASIO_LIGHT_CHANNEL).
 *    - Power on while holding [TONE] + [Power] to disable Auto Power Off so
 *      Local Control stays off during long practice sessions.
 *
 *  SYNTHESIA ASSUMPTIONS (Settings -> Music Devices -> Teensy MIDI Output):
 *    - "Key Lights" is set to "Finger-based channel".
 *    - "My Notes", "Background Instruments", "Percussion", and "Metronome" are
 *      all turned OFF for Teensy MIDI (audio plays through the Mac's synth).
 *    - In "Finger-based channel" mode, Synthesia outputs 1-indexed MIDI channels:
 *        * Ch 1      : Unassigned / unknown hand
 *        * Ch 2 - 6  : Left Hand (fingers 1-5)   -> BLOCKED on Casio
 *        * Ch 7 - 11 : Right Hand (fingers 1-5)  -> ALLOWED on Casio
 *        * Ch 12     : Left Hand (unknown finger)-> BLOCKED on Casio
 *        * Ch 13     : Right Hand (unknown finger)-> ALLOWED on Casio
 *
 *  DUBLER 2 ASSUMPTIONS:
 *    - Dubler 2 sends whistle/voice pitch MIDI notes on Channel 1 (DUBLER_CHANNEL).
 *    - Because Pin 3 specifically opens ONLY DUBLER_CHANNEL (Ch 1), Synthesia's
 *      Left Hand channels (Ch 2-6 and Ch 12) remain blocked even when Pin 3 is grounded.
 *    - Note: If you ever change Dubler's output channel, update DUBLER_CHANNEL below.
 * ============================================================================
 */

#include <USBHost_t36.h>

// ----------------------------------------------------------------------------
// 1. USB HOST OBJECTS (Teensy talking to the Casio LK-160)
// ----------------------------------------------------------------------------
USBHost myusb;
USBHub hub1(myusb);       // Supports USB hubs if ever connected between Teensy & Casio
MIDIDevice casio(myusb);  // Represents the Casio LK-160 on the USB Host port

// ----------------------------------------------------------------------------
// 2. HARDWARE PINS & MIDI CHANNEL CONFIGURATION
// ----------------------------------------------------------------------------
const int modeSwitchPin = 2;          // Pin 2 -> GND selects Dodeka mode
const int dublerPin = 3;              // Pin 3 -> GND enables Dubler 2 lighting

const byte CASIO_LIGHT_CHANNEL = 3;   // Casio LK-160 key-light Navigate Channel (1-16)
const byte DUBLER_CHANNEL = 1;        // Expected MIDI channel from Dubler 2 (1-16)
const byte DUBLER_CC = 104;           // MIDI CC sent to Mac when Pin 3 toggles (Ch 1)

// ----------------------------------------------------------------------------
// 3. DODEKA MAPPING & BLINK TIMING CONFIGURATION
// ----------------------------------------------------------------------------
const int PHYSICAL_START_NOTE = 36;   // Leftmost physical white key on Casio (C2)
const int DODEKA_START_NOTE = 48;     // Musical pitch output by that key in Dodeka (C3)
const int NUM_WHITE_KEYS = 36;        // All 36 white keys (C2..C7) -> 3 octaves (C3..B5)

// Minimum dark gap (in milliseconds) before relighting the exact same key.
// Makes back-to-back repeated notes visibly blink off instead of staying solid.
const unsigned long RETRIGGER_GAP_MS = 60;

// Semitone step from one white key to the next across a standard piano octave:
// C->D (+2), D->E (+2), E->F (+1), F->G (+2), G->A (+2), A->B (+2), B->C (+1)
const int whiteKeyGaps[] = {2, 2, 1, 2, 2, 2, 1};

// Lookup tables built once in setup() for instant O(1) pitch translation:
//   dodekaMap[physicalCasioPitch]  -> musical Dodeka pitch (or -1 if black key / unmapped)
//   reverseDodekaMap[musicalPitch] -> physical Casio white key to light (or -1 if out of range)
int dodekaMap[128];
int reverseDodekaMap[128];

// ----------------------------------------------------------------------------
// 4. PER-KEY LED STATE (for repeated-note blink & stuck-light prevention)
//    Indexed by physical Casio MIDI note number (0..127).
// ----------------------------------------------------------------------------
bool keyLit[128];                     // True if we believe this Casio LED is currently ON
unsigned long keyOffTime[128];        // Timestamp (ms) when this Casio LED was last turned OFF
bool pendingOn[128];                  // True if a delayed relight is queued for this key
unsigned long pendingOnTime[128];     // Timestamp (ms) when the delayed relight should fire
byte pendingVelocity[128];            // Saved velocity for the delayed relight

// ----------------------------------------------------------------------------
// 5. SWITCH STATE & DEBOUNCING
// ----------------------------------------------------------------------------
bool lastModeState = HIGH;            // Last read state of Pin 2 (Piano vs Dodeka)
bool lastDublerReading = HIGH;        // Raw last read state of Pin 3 (for debouncing)
bool dublerActive = false;            // Debounced state: true when Pin 3 is grounded
unsigned long lastDebounceTime = 0;   // Last time Pin 3 changed raw state
const unsigned long DEBOUNCE_MS = 30; // Debounce window (ms) to ignore switch chatter

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

// Returns true if the incoming 1-indexed MIDI channel is one of Synthesia's
// Right-Hand key-lighting channels (Ch 7-11 for fingered notes, Ch 13 for unfingered).
bool isRightHand(byte channel) {
  return (channel >= 7 && channel <= 11) || (channel == 13);
}

// Immediately extinguishes all lit Casio keys and cancels any queued blink timers.
// Called on MIDI All Notes Off (CC 123), All Sound Off (CC 120), or switch changes.
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
  // Also send MIDI CC #123 (All Notes Off) directly to the Casio as a safety net
  casio.sendControlChange(123, 0, CASIO_LIGHT_CHANNEL);
}

// Polls Pin 2 (Piano/Dodeka mode) and Pin 3 (Dubler gate).
// Clears all lights on any state change so no LEDs get stranded across mode switches.
void checkControlPins() {
  // --- Pin 2: Piano vs. Dodeka Mode ---
  bool modeNow = digitalRead(modeSwitchPin);
  if (modeNow != lastModeState) {
    lastModeState = modeNow;
    clearAllLights();
  }

  // --- Pin 3: Dubler Gate (debounced) ---
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
      // Notify Mac software (e.g. DAW / Dubler) of the gate change via CC #104 on Ch 1
      usbMIDI.sendControlChange(DUBLER_CC, dublerActive ? 127 : 0, 1);
    }
  }
}

// Non-blocking scheduler: checks if any key waiting in its 60 ms blink gap
// is ready to be lit, and sends its Note On to the Casio.
void processPendingLights() {
  unsigned long now = millis();
  for (int i = 0; i < 128; i++) {
    // Signed subtraction safely handles millis() rollover after ~49 days
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
  // MIDI standard treats Note On with velocity 0 as a Note Off
  if (velocity == 0) {
    casioToPcNoteOff(channel, pitch, 0);
    return;
  }

  if (digitalRead(modeSwitchPin) == HIGH) {
    // Mode 1: Standard Piano -> pass physical pitch straight through
    usbMIDI.sendNoteOn(pitch, velocity, channel);
  } else {
    // Mode 2: Dodeka -> translate white key pitch; silently ignore black keys (-1)
    int translatedNote = dodekaMap[pitch];
    if (translatedNote != -1) {
      usbMIDI.sendNoteOn(translatedNote, velocity, channel);
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

// ============================================================================
// INCOMING MIDI CALLBACKS: Mac (Synthesia / Dubler 2) -> Casio Key Lights
// ============================================================================

void pcToCasioNoteOn(byte channel, byte pitch, byte velocity) {
  // Treat Note On with velocity 0 as Note Off
  if (velocity == 0) {
    pcToCasioNoteOff(channel, pitch, 0);
    return;
  }

  // Channel Gatekeeper:
  //   - Always allow Synthesia's Right-Hand channels (Ch 7-11, Ch 13).
  //   - Allow Dubler's channel (Ch 1) ONLY when Pin 3 is grounded (dublerActive).
  //   - Block Synthesia's Left-Hand channels (Ch 2-6, Ch 12) at all times.
  bool allowChannel = isRightHand(channel) || (dublerActive && channel == DUBLER_CHANNEL);
  if (!allowChannel) return;

  // Map incoming musical pitch to the target physical key on the Casio
  int casioPitch = (digitalRead(modeSwitchPin) == HIGH) ? pitch : reverseDodekaMap[pitch];
  if (casioPitch < 0 || casioPitch >= 128) return; // Ignore notes outside C3..B5 in Dodeka mode

  unsigned long now = millis();

  if (keyLit[casioPitch]) {
    // Case A: Key is ALREADY lit (overlapping or zero-gap repeated note).
    // Force it off now and schedule it to relight after RETRIGGER_GAP_MS.
    casio.sendNoteOff(casioPitch, 0, CASIO_LIGHT_CHANNEL);
    keyLit[casioPitch] = false;
    keyOffTime[casioPitch] = now;
    pendingOn[casioPitch] = true;
    pendingOnTime[casioPitch] = now + RETRIGGER_GAP_MS;
    pendingVelocity[casioPitch] = velocity;
  } else if (keyOffTime[casioPitch] != 0 && (now - keyOffTime[casioPitch]) < RETRIGGER_GAP_MS) {
    // Case B: Key was JUST turned off less than RETRIGGER_GAP_MS ago.
    // Hold it dark for the remainder of the blink window, then relight.
    pendingOn[casioPitch] = true;
    pendingOnTime[casioPitch] = keyOffTime[casioPitch] + RETRIGGER_GAP_MS;
    pendingVelocity[casioPitch] = velocity;
  } else {
    // Case C: Normal note (key has been dark long enough) -> light immediately.
    pendingOn[casioPitch] = false;
    casio.sendNoteOn(casioPitch, velocity, CASIO_LIGHT_CHANNEL);
    keyLit[casioPitch] = true;
  }
}

void pcToCasioNoteOff(byte channel, byte pitch, byte velocity) {
  int casioPitch = (digitalRead(modeSwitchPin) == HIGH) ? pitch : reverseDodekaMap[pitch];
  if (casioPitch < 0 || casioPitch >= 128) return;

  // Channel-agnostic Note Off:
  // If this physical key is currently lit (or queued to blink), turn it off
  // regardless of which MIDI channel carried the Note Off. This prevents stuck
  // LEDs if Synthesia sends Note Off on a different channel than Note On, while
  // still ignoring Note Offs for unlit Left-Hand keys.
  if (!keyLit[casioPitch] && !pendingOn[casioPitch]) return;

  pendingOn[casioPitch] = false;
  keyLit[casioPitch] = false;
  keyOffTime[casioPitch] = millis();
  casio.sendNoteOff(casioPitch, 0, CASIO_LIGHT_CHANNEL);
}

void pcToCasioControlChange(byte channel, byte control, byte value) {
  // Synthesia sends CC #120 (All Sound Off) or CC #123 (All Notes Off) when
  // pausing, stopping, looping, or finishing a song. Clear all lights on any channel.
  if (control == 120 || control == 123) {
    clearAllLights();
  }
}

// ============================================================================
// ARDUINO SETUP & MAIN LOOP
// ============================================================================

void setup() {
  // Enable internal pull-up resistors (pins read HIGH when open, LOW when grounded)
  pinMode(modeSwitchPin, INPUT_PULLUP);
  pinMode(dublerPin, INPUT_PULLUP);
  lastModeState = digitalRead(modeSwitchPin);

  // Initialize lookup tables and LED tracking arrays to a clean default state
  for (int i = 0; i < 128; i++) {
    dodekaMap[i] = -1;
    reverseDodekaMap[i] = -1;
    keyLit[i] = false;
    keyOffTime[i] = 0;
    pendingOn[i] = false;
    pendingOnTime[i] = 0;
    pendingVelocity[i] = 0;
  }

  // Populate the bidirectional 36-key Dodeka map:
  // Steps through every white key from physical C2 (36) to C7 (96) and assigns
  // consecutive chromatic notes from C3 (48) to B5 (83).
  int currentCasioNote = PHYSICAL_START_NOTE;
  int dodekaNote = DODEKA_START_NOTE;

  for (int i = 0; i < NUM_WHITE_KEYS; i++) {
    if (currentCasioNote < 128 && dodekaNote < 128) {
      dodekaMap[currentCasioNote] = dodekaNote;
      reverseDodekaMap[dodekaNote] = currentCasioNote;
    }
    dodekaNote++;
    currentCasioNote += whiteKeyGaps[i % 7];
  }

  // Start USB Host stack for the Casio LK-160
  myusb.begin();

  // Register outgoing callbacks (Casio -> Mac)
  casio.setHandleNoteOn(casioToPcNoteOn);
  casio.setHandleNoteOff(casioToPcNoteOff);

  // Register incoming callbacks (Mac -> Casio)
  usbMIDI.setHandleNoteOn(pcToCasioNoteOn);
  usbMIDI.setHandleNoteOff(pcToCasioNoteOff);
  usbMIDI.setHandleControlChange(pcToCasioControlChange);
}

void loop() {
  myusb.Task();                    // Service Teensy USB Host hardware
  while (casio.read()) {}          // Drain all queued MIDI messages from Casio
  while (usbMIDI.read()) {}        // Drain all queued MIDI messages from Mac
  checkControlPins();              // Poll Pin 2 (Mode) and Pin 3 (Dubler Gate)
  processPendingLights();          // Fire any completed 60 ms repeated-note blinks
}