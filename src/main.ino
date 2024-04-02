#include <Arduino.h>

// add "~/Library/Arduino15/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7/avr/include/avr/iom328p.h" for the uno / mega to help with the missing definitions

#include <AltSoftSerial.h>
AltSoftSerial midiSerial; // 2 is RX, 3 is TX

/*  
 * ADC work
 */
 
// #define SERIAL_PLOT_MODE
// #define DEBUG

#define MIN_THRESHOLD 30 //discard readings below this adc value
#define MIN_NOTE_THRESHOLD 60 //minimum time allowed between notes on the same channel
#define MIN_BUTTON_HOLD_TIME 1000
#define BUTTON_HOLD_PROGRAM_TIME 2000

#define ANALOGPINS 6
byte adcPin[ANALOGPINS] = {0,1,2,3,4,5};  // This relies on A0 being 0, etc, which on the UNO is true. Mapped manually for clarity.

#define CHANNELS 4
volatile uint32_t maxResults[CHANNELS];

#define DISCARD 1
byte discardCounter = 0;

#ifdef SERIAL_PLOT_MODE
volatile int countedSamples = 0;
volatile int missedSamples = 0;
#endif

byte currentPin = 0;
int adcValue[CHANNELS];

volatile boolean inLoop;

/*
 * MIDI setup
 */

// #define SEND_NOTE_OFF_VELOCITY
bool hiHatPedalMakesOpeningSounds = false;
bool noteProgrammingMode = false;
byte noteToProgram = 0;

#define KICK_INPUT_PIN 5
#define HIT_HAT_INPUT_PIN 6
#define B_BUTTON 2
#define RIGHT_BUTTON 2
#define Y_BUTTON 3
#define UP_BUTTON 3
#define X_BUTTON 4
#define LEFT_BUTTON 4
#define A_BUTTON 7
#define DOWN_BUTTON 7

#define DIGITAL_INPUTS 6
bool buttonState[DIGITAL_INPUTS];
byte digitalPins[DIGITAL_INPUTS] = {KICK_INPUT_PIN, HIT_HAT_INPUT_PIN,RIGHT_BUTTON,UP_BUTTON,LEFT_BUTTON,DOWN_BUTTON};
unsigned long buttonPressedDuration[DIGITAL_INPUTS];

#define HIGH_Q 27 // D#0
#define SLAP 28 // E0
#define SCRATCH_PUSH 29 // F0
#define SCRATCH_PULL 30 // F#0
#define STICKS 31 // G0
#define SQUARE_CLICK 32 // G#0
#define METRONOME_CLICK 33 // A0
#define METRONOME_BELL 34 // A#0
#define KICK_DRUM_2 35 // B0
#define KICK_DRUM_1 36 // C1
#define SIDE_STICK 37 // C#1
#define SNARE_DRUM_1 38 // D1
#define HAND_CLAP 39 // D#1
#define SNARE_DRUM_2 40 // E1
#define LOW_TOM_2 41 // F1
#define CLOSED_HI_HAT 42 // F#1
#define LOW_TOM_1 43 // G1
#define PEDAL_HI_HAT 44 // G#1
#define MID_TOM_2 45 // A1
#define OPEN_HI_HAT 46 // A#1
#define MID_TOM_1 47 // B1
#define HIGH_TOM_2 48 // C2
#define CRASH_CYMBAL 49 // C#2
#define HIGH_TOM_1 50 // D2
#define RIDE_CYMBAL 51 // D#2
#define CHINESE_CYMBAL 52 // E2
#define RIDE_BELL 53 // F2
#define TAMBOURINE 54 // F#2
#define SPLASH_CYMBAL 55 // G2
#define COWBELL 56 // G#2
#define CRASH_CYMBAL_2 57 // A2
#define VIBRA_SLAP 58 // A#2
#define RIDE_CYMBAL_2 59 // B2
#define HIGH_BONGO 60 // C3
#define LOW_BONGO 61 // C#3
#define MUTE_HI_CONGA 62 // D3
#define OPEN_HI_CONGA 63 // D#3
#define LOW_CONGA 64 // E3
#define HIGH_TIMBALE 65 // F3
#define LOW_TIMBALE 66 // F#3
#define HIGH_AGOGO 67 // G3
#define LOW_AGOGO 68 // G#3
#define CABASA 69 // A3
#define MARACAS 70 // A#3
#define SHORT_HI_WHISTLE 71 // B3
#define LONG_LO_WHISTLE 72 // C4
#define SHORT_GUIRO 73 // C#4
#define LONG_GUIRO 74 // D4
#define CLAVES 75 // D#4
#define HIGH_WOODBLOCK 76 // E4
#define LOW_WOODBLOCK 77 // F4
#define MUTE_CUICA 78 // F#4
#define OPEN_CUICA 79 // G4
#define MUTE_TRIANGLE 80 // G#4
#define OPEN_TRIANGLE 81 // A4
#define SHAKER 82 // A#4
#define JINGLE_BELL 83 // B4
#define BELLTREE 84 // C5
#define CASTANETS 85 // C#5
#define MUTE_SURDO 86 // D5
#define OPEN_SURDO 87 // D#5

byte noteMap[4] = {OPEN_HI_HAT,SNARE_DRUM_1,LOW_TOM_2,CRASH_CYMBAL}; 

//MIDI defines
#define NOTE_ON_CMD 0x90
#define NOTE_OFF_CMD 0x80
#define DRUM_CHANNEL 0x09
#define PIANO_CHANNEL 0x00
#define MAX_MIDI_VELOCITY 127
#define NOMINAL_MIDI_VELOCITY 64

unsigned long timeOfLastNote[CHANNELS];
bool channelArmed[CHANNELS];
int lastNoteVelocity[CHANNELS];

#define SEND_BYTES(x) for (byte l : x) { midiSerial.write(l);}

// MIDI baud rate
#define MIDI_SERIAL_RATE 31250
#define SERIAL_RATE 38400

void setup ()
  {
  Serial.println("Startup");
  
  Serial.begin (SERIAL_RATE);
  midiSerial.begin(MIDI_SERIAL_RATE);
  
  for (int i=0;i<CHANNELS;i++) {
    maxResults[i]=0;
    timeOfLastNote[i]=0;
  }

  for (int i=0;i<DIGITAL_INPUTS;i++) {
    pinMode(digitalPins[i], INPUT_PULLUP);
    buttonState[i] = HIGH;
    buttonPressedDuration[i] = 0x00;
  }

  pinMode(LED_BUILTIN, OUTPUT);

  // FYI the Roland Mt-80s Device ID is 16 - 0x10
  // GS Mode Reset - Seems to be unnecessary so removed for now
  // byte sysexReset[] =  { 0xF0,0x41,0x16,0x42,0x12,0x40,0x00,0x7F,0x00,0x41,0xF7 };
  // SEND_BYTES(sysexReset)

  // Use SysEx to Assign Mode to "2" on Channel 10, which allows notes to overlap each other
  // Otherwise, a new note with a lower velocity will chop the existing note's sustain to the new volume
  byte assignMode[] =  { 0xF0,0x41,0x10,0x42,0x12,0x40,0x10,0x14,0x01,0x1a,0xF7 };
  SEND_BYTES(assignMode)
  delay(500);

  // This sets the drum kit.
  // byte setKit[] = {0xC9,0x10};
  // SEND_BYTES(setKit)

  /* working kits
  0x10 = power kit aka Phil Collins
  0x18 = electronic 
  0x19 = moar electronic 
  0x28 = brush
  /*


  /* ADC Setup
  * all the hard work for setting the registers and finding the fastest way to access the adc data was done by this fellow:
  * http://yaab-arduino.blogspot.com/2015/02/fast-sampling-from-analog-input.html
  */

  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX |= (adcPin[0] & 0x07);    // set A0 analog input pin
  ADMUX |= (1 << REFS0);  // set reference voltage
  // ADMUX |= (1 << ADLAR);  // left align ADC value to 8 bits from ADCH register

  // sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
  // for Arduino Uno ADC clock is 16 MHz and a conversion takes 13 clock cycles
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5 KHz
//  ADCSRA |= (1 << ADPS2);                     // 16 prescaler for 76.9 KHz
//  ADCSRA |= (1 << ADPS1) | (1 << ADPS0);    // 8 prescaler for 153.8 KHz

  ADCSRA |= (1 << ADATE); // enable auto trigger
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  ADCSRA |= (1 << ADSC);  // start ADC measurements

  // end code from http://yaab-arduino.blogspot.com/2015/02/fast-sampling-from-analog-input.html

} 

// ADC complete ISR
ISR (ADC_vect) {
  if (discardCounter == DISCARD) {
    if (inLoop) {
      #ifdef SERIAL_PLOT_MODE
        missedSamples++;
      #endif
      return;
    }
    
    discardCounter = 0;
    #ifdef SERIAL_PLOT_MODE
      countedSamples++; 
    #endif

    if (ADC > maxResults[currentPin])
      maxResults[currentPin] = ADC;
    
    currentPin++;
    currentPin = currentPin%CHANNELS;
  
    ADMUX = bit (REFS0) | (adcPin[currentPin] & 7);
  }
  else {
    discardCounter++;
    return;
  }
}


void loop () {
  // this pulls the LED down and off, its turned on during a midiWrite event at least 2ms ago
  digitalWrite(LED_BUILTIN, LOW);

  digitalPinScan();

  // the inLoop boolean prevents the ADC interrupt code from colliding with this loop while the work is done below. keep anything processor intensive OUT of this section of the loop, like Serial output or heavy math.
  // on an UNO, this loop work occasionally causes ONE sample to be discarded, which is only 1/50,000 of a seconds' worth of data. That's great.
  inLoop = true;

  for (int i=0;i<CHANNELS;i++) {
    adcValue[i] = maxResults[i];
    maxResults[i] = 0;
  }

  inLoop = false;

  // This is where the real code needs to go to DO something with this.
  unsigned long now = millis();
  for (int i = 0; i<CHANNELS; i++) {
    if (adcValue[i] > MIN_THRESHOLD) {
      long timeSinceLastNote = now - timeOfLastNote[i];
      byte velocity = adcValue[i] / 8;
      if (velocity > lastNoteVelocity[i]) {
        channelArmed[i] = true;
      }
      else if (channelArmed[i]) {
        if (timeSinceLastNote > MIN_NOTE_THRESHOLD) {
          handleAnalogEvent(noteMap[i],lastNoteVelocity[i]);
          
          timeOfLastNote[i] = now;
        }
        channelArmed[i] = false;
      }
      lastNoteVelocity[i] = velocity;
      
      
      #ifdef DEBUG
      Serial.print("Channel: ");
      Serial.print(i);
      Serial.print(" Velocity: ");
      Serial.print(velocity);
      Serial.print(" Now: ");
      Serial.print(now);
      Serial.print(" timeOfLastNote: ");
      Serial.print(timeOfLastNote[i]);
      Serial.print(" timeSinceLastNote: ");
      Serial.println(timeSinceLastNote);
      #endif

    }
  }

  #ifdef SERIAL_PLOT_MODE
  bool constantlyDraw = true;
  for (int i=0;i<CHANNELS;i++) {
    if (adcValue[i] > MIN_THRESHOLD) {
      constantlyDraw = true;
    }
  }
  if (constantlyDraw) {
    Serial.print ("s:");
    Serial.print ("1023");
    Serial.print (",m:");
    Serial.print (missedSamples);
    Serial.print (",c:"); 
    Serial.print (countedSamples);
    for (int i=0;i<CHANNELS;i++) {
      Serial.print (",P");
      Serial.print (i);
      Serial.print (":");
      Serial.print (adcValue[i]);
    }
    Serial.println ();
  }
    countedSamples = missedSamples = 0;
  #endif
  
  /*  This delay ensures enough time to prevent aliasing. 
  *  For example:
  *  If countedSamples == 50000 per second, and CHANNELS == 6, then you'll get 8333 samples per second.
  *  Half are discarded by the DISCARD system to allow the analog pin to settle.
  *  Half again can be considered discarded by being a negative voltage (unless you either full bridge recitfy (BOOOM) or bias the ac signal, which you should.
  *  In my test, my UNO with 6 channels, i could sample down to 80Hz without missing the peak with a delay of 10, thus getting a stable max value on a sine wave with an acceptable latency.
  *  Less channels and higher minimum freq means you can lower this delay.
  *  
  *  Note that at HIGHER piezo freqencies, the sampling algo will break down if they match the sample rate or are multiple of it, due to the sample peak being 'skipped over'.
  *  This can be addressed by sampling one channel at a much higher freq for CHANNEL # of cycles ones, but that can cause slightly less accuracy to get missed at lower freqs occasionally too.
  *  It's more logic in the interrupt though, which is bad.
  *  
  *  The short of it is: tune the system to your expected piezo frequency. This will NOT do higher AND lower freqs perfectly without compromise.
  */

  // on my UNO, 3ms can reliably detect the max value of single peak 100hz sine wave.
  delay(2);
}

void digitalPinScan() {
  for (int i=0;i<DIGITAL_INPUTS;i++) { // because i hate when other people write this kind of code, i'll put comments for future me
    if (digitalRead(digitalPins[i]) == !buttonState[i]) { // if the pin state doesn't match the stored state
      handleDigitalInput(digitalPins[i], buttonState[i]); // then send the opposite (ie the same as pin state without reading it again)
      buttonState[i] = !buttonState[i]; // and flip it
    }
    if (buttonState[i] == LOW && i > 1) { // if the button is pressed and its not the first two pins
      if (buttonPressedDuration[i]
          && (millis() - buttonPressedDuration[i]) > MIN_BUTTON_HOLD_TIME
          && (millis() - buttonPressedDuration[i]) < (MIN_BUTTON_HOLD_TIME + 10)) {
        playCurrentDrumSound(digitalPins[i]);
        delay(11);
      }
      else if (buttonPressedDuration[i]
          && (millis() - buttonPressedDuration[i]) > BUTTON_HOLD_PROGRAM_TIME) {
        noteProgrammingMode = true;
        buttonPressedDuration[i] = 0;
        programNotify(HIGH);
      }
    }
  }
}

void handleDigitalInput(byte pin, bool pressed) {
  Serial.print("handleDigitalInput ");
  Serial.println(pin);
  Serial.print("pressed ");
  Serial.println(pressed);
  // pedals
  if (pin == KICK_INPUT_PIN && pressed) {
    noteFireLinearVelocity(KICK_DRUM_1,80, DRUM_CHANNEL);
  }
  else if (pin == HIT_HAT_INPUT_PIN && pressed) {
    noteFireLinearVelocity(PEDAL_HI_HAT,80, DRUM_CHANNEL);
  }
  else if (hiHatPedalMakesOpeningSounds && pin == HIT_HAT_INPUT_PIN && !pressed) {
    noteFireLinearVelocity(OPEN_HI_HAT,80, DRUM_CHANNEL);
  }

  if (noteProgrammingMode) {
    if (pressed) {
      if (pin == UP_BUTTON) {
        noteMap[noteToProgram]++;
        midiNoteOn(noteMap[noteToProgram],64,DRUM_CHANNEL);
      }
      else if (pin == DOWN_BUTTON) {
        noteMap[noteToProgram]--;
        midiNoteOn(noteMap[noteToProgram],64,DRUM_CHANNEL);
      }
      else if (pin == LEFT_BUTTON) {
        noteProgrammingMode = false;
        programNotify(LOW);
      }
    }
    return;
  }

  //buttons
  for (int i = 0; i < DIGITAL_INPUTS; i++)
  {
    if (digitalPins[i] == pin) {
      if (pressed) {
        Serial.print("Pin Press at ");
        Serial.println(millis());
        buttonPressedDuration[i] = millis();
      }
      else {
        Serial.print("Pin held for ");
        Serial.println(millis()-buttonPressedDuration[i]);
        // buttonPressedDuration[i] = 0x00;
      }
    }
  }
}

#ifdef DEBUG
void noteDebug(byte note, byte velocity)
{
  Serial.print("Note: ");
  Serial.print(note);
  Serial.print(" Velocity: ");
  Serial.print(velocity);
  Serial.println();
}
#endif

#define VELOCITY_FACTOR 1.5
// a VELOCITY_FACTOR greater than one removes dynamic range and sets a higher low limit
// less than one burns your house down and increases dynamic range. you don't want this.
// 1 makes the range limits 0-127
// 2 makes the range limits 63-127
// 3 makes the range limits 85-127

byte correctVelocityCurve(byte velocity) {
  return (velocity / VELOCITY_FACTOR) + (MAX_MIDI_VELOCITY - (MAX_MIDI_VELOCITY / VELOCITY_FACTOR));
}

void handleAnalogEvent(byte note, byte velocity) {
  if (note == OPEN_HI_HAT && buttonState[1] == LOW) {
      note = CLOSED_HI_HAT;
  }
  noteFireLinearVelocity(note, velocity, DRUM_CHANNEL);
}

void noteFire(byte note, byte velocity, byte channel)
{
  noteFireLinearVelocity(note, correctVelocityCurve(velocity), channel);
}

void noteFireLinearVelocity(byte note, byte velocity, byte channel)
{
  midiNoteOn(note, velocity, channel);
  midiNoteOff(note, velocity, channel);
  #ifdef DEBUG
  noteDebug(note, velocity);
  #endif
}

void midiNoteOn(byte note, byte midiVelocity, byte channel)
{
  digitalWrite(LED_BUILTIN, HIGH);
  midiSerial.write(NOTE_ON_CMD + channel);
  midiSerial.write(note);
  midiSerial.write(midiVelocity);
}

void midiNoteOff(byte note, byte midiVelocity, byte channel)
{
  midiSerial.write(NOTE_OFF_CMD + channel);
  midiSerial.write(note);
  #ifdef SEND_NOTE_OFF_VELOCITY 
  midiSerial.write(midiVelocity); // unread on the MT-80s
  #endif
}

void programNotify(bool up) {
  if (up) {
    higherNote();
    lowerNote();
  }
  else {
    lowerNote();
    higherNote();
  }
}

void higherNote() {
  midiNoteOn(78,64,PIANO_CHANNEL);
  delay(200);
  midiNoteOff(78,64,PIANO_CHANNEL);
}

void lowerNote() {
  midiNoteOn(83,64,PIANO_CHANNEL);
  delay(200);
  midiNoteOff(83,64,PIANO_CHANNEL);
}

void playCurrentDrumSound(byte pin) {
  if (pin == B_BUTTON) {
    noteToProgram = 0;
  }
  else if (pin == Y_BUTTON) {
    noteToProgram = 1;
  }
  else if (pin == X_BUTTON) {
    noteToProgram = 2;
  }
  else if (pin == A_BUTTON) {
    noteToProgram = 3;
  }
  midiNoteOn(noteMap[noteToProgram],64,DRUM_CHANNEL);
}