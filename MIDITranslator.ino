// MIDI Translator handles two Casio keyboards
#include <USBHost_t36.h>

USBHost myusb;
USBHub hub1(myusb);
// Instantiate both Casios 
MIDIDevice casioLower(myusb); 
MIDIDevice casioUpper(myusb);

const int modeSwitchPin = 2; 

int startingMidiNote = 48; // C3
int whiteKeyGaps[] = {2, 2, 1, 2, 2, 2, 1};

int dodekaMap[128];        
int reverseDodekaMap[128]; 

void setup() {
  pinMode(3, INPUT_PULLUP); //Dubler enable switch
  pinMode(modeSwitchPin, INPUT_PULLUP);
  
  // Initialize arrays to -1 (ignore unused notes)
  for(int i = 0; i < 128; i++) {
    dodekaMap[i] = -1;
    reverseDodekaMap[i] = -1;
  }
  
  int currentCasioNote = startingMidiNote;
  int dodekaNote = startingMidiNote; 
  
  // Map 72 consecutive white keys
  for(int i = 0; i < 72; i++) { 
     if (currentCasioNote < 128 && dodekaNote < 128) {
       dodekaMap[currentCasioNote] = dodekaNote;
       reverseDodekaMap[dodekaNote] = currentCasioNote; 
     }
     dodekaNote++; // Chromatic step
     currentCasioNote += whiteKeyGaps[i % 7]; // Skip black keys
  }

  myusb.begin();
  
  // Attach handlers for hardware inputs (Casios to PC)
  casioLower.setHandleNoteOn(casioToPcNoteOn);
  casioLower.setHandleNoteOff(casioToPcNoteOff);
  casioUpper.setHandleNoteOn(casioToPcNoteOn);
  casioUpper.setHandleNoteOff(casioToPcNoteOff);

  // Attach handlers for software inputs (PC to Casios)
  usbMIDI.setHandleNoteOn(pcToCasioNoteOn);
  usbMIDI.setHandleNoteOff(pcToCasioNoteOff);
}

void casioToPcNoteOn(byte channel, byte pitch, byte velocity) {
  if (velocity == 0) { casioToPcNoteOff(channel, pitch, 0); return; }
  
  if (digitalRead(modeSwitchPin) == HIGH) {
    usbMIDI.sendNoteOn(pitch, velocity, channel); // Mode 1: Piano
  } else {
    int translatedNote = dodekaMap[pitch];
    if (translatedNote != -1) usbMIDI.sendNoteOn(translatedNote, velocity, channel); // Mode 2: Dodeka
  }
}

void casioToPcNoteOff(byte channel, byte pitch, byte velocity) {
  if (digitalRead(modeSwitchPin) == HIGH) {
    usbMIDI.sendNoteOff(pitch, 0, channel);
  } else {
    int translatedNote = dodekaMap[pitch];
    if (translatedNote != -1) usbMIDI.sendNoteOff(translatedNote, 0, channel);
  }
}

void pcToCasioNoteOn(byte channel, byte pitch, byte velocity) {
  if (velocity == 0) { pcToCasioNoteOff(channel, pitch, 0); return; }

  if (digitalRead(modeSwitchPin) == HIGH) { 
    // Broadcast piano passthrough to both
    casioLower.sendNoteOn(pitch, velocity, channel);
    casioUpper.sendNoteOn(pitch, velocity, channel);
  } else {                                  
    // Broadcast translated Dodeka light command to both
    int casioPitch = reverseDodekaMap[pitch];
    if (casioPitch != -1) {
      casioLower.sendNoteOn(casioPitch, velocity, channel);
      casioUpper.sendNoteOn(casioPitch, velocity, channel);
    }
  }
}

void pcToCasioNoteOff(byte channel, byte pitch, byte velocity) {
  if (digitalRead(modeSwitchPin) == HIGH) { 
    casioLower.sendNoteOff(pitch, 0, channel);
    casioUpper.sendNoteOff(pitch, 0, channel);
  } else {                                  
    int casioPitch = reverseDodekaMap[pitch];
    if (casioPitch != -1) {
      casioLower.sendNoteOff(casioPitch, 0, channel);
      casioUpper.sendNoteOff(casioPitch, 0, channel);
    }
  }
}

void loop() {
  myusb.Task();
  casioLower.read();
  casioUpper.read();
  usbMIDI.read(); 
  // Example inside loop() to handle Dubler switch. 
if (digitalRead(3) == LOW) {
    usbMIDI.sendControlChange(104, 127, 1); // Send CC 104 on Channel 1
    delay(200); // Debounce
}
}