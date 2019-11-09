// 09 Nov 2019   00:51 UTC
// rotary-simple-a/src/rotary-sketch.cpp
// nefeppha    plugaro     fredo

#include <Arduino.h>
#define MAX_POS 37

// inspired/borrowed from:
// https://github.com/mathertel/RotaryEncoder

volatile int sigA  = 0;
volatile int sigB  = 0;
volatile int sigSw = 0;

volatile uint8_t tick_recent, pbswitch_recent, neoPixel2_recent, lcd_clear_recent = 0; // FALSE

const uint8_t encoderKnobPinB = A1;
const uint8_t encoderKnobPinA = A2;
const uint8_t        pbSwitch = A3;
const uint8_t       backlight = A4;
const uint8_t         buzzPin = A5; // piezo
const uint8_t          ledPin = 13; // onboard D13 LED pin

const uint8_t LATCHSTATE = 3;
int8_t oldState = 3;

const int8_t KNOBDIR[] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
  -1,  0,  0,  1,
  0,  1, -1,  0
};

volatile int positionInternal = 0;
volatile int positionExternal = 0;
volatile int oldPositionExternal = 0;

volatile int hysteresis_upward = 0 ;

int altinFeet = -1;      // TRUE -- altitude preference is in FEET
uint32_t timer = millis();

//  - - - - ------------------- ROTARY ENCODER ------------------

/* ISR - momentary pushbutton switch */
void sw_isr() { // ref: const uint8_t   pbSwitch = A3;
  // pbSwitch
  noInterrupts();
  detachInterrupt(pbSwitch);
  delayMicroseconds(2200); // debounce
  sigSw = digitalRead(pbSwitch);
  pbswitch_recent = -1; // TRUE
  attachInterrupt(pbSwitch,        sw_isr,  FALLING);
  interrupts();
}

/* ISR - rotary shaft encoder - front panel control knob */
void tick_isr() {
  // local variables now global: sigA sigB to shorten ISR
  noInterrupts();
  detachInterrupt(encoderKnobPinA);
  detachInterrupt(encoderKnobPinB);
  delayMicroseconds(2200); // debounce
  sigA = digitalRead(encoderKnobPinA);
  sigB = digitalRead(encoderKnobPinB);

  tick_recent = -1; // TRUE -- if FALSE here, the thing stalls out nicely

  attachInterrupt(encoderKnobPinA, tick_isr, CHANGE);
  attachInterrupt(encoderKnobPinB, tick_isr, CHANGE);
  interrupts();
}

void setup_rotEnc(void) {
  pinMode(encoderKnobPinA, INPUT_PULLUP);
  pinMode(encoderKnobPinB, INPUT_PULLUP);
  pinMode(pbSwitch,        INPUT_PULLUP);

  digitalWrite(encoderKnobPinA, HIGH); // raise the pin to Vcc
  digitalWrite(encoderKnobPinB, HIGH);
  digitalWrite(pbSwitch,        HIGH);

  attachInterrupt(encoderKnobPinA, tick_isr, CHANGE); // RISING
  attachInterrupt(encoderKnobPinB, tick_isr, CHANGE); // RISING
  attachInterrupt(pbSwitch,        sw_isr,  FALLING);
}

// my morse code oriented stuff, piezo buzzer:

int bzFreqP =  180;
int bzFreqQ =  140;

void dah(void) { // low freq buzzer
  int braapDuration = 1100;

  noInterrupts();
  detachInterrupt(encoderKnobPinA);
  detachInterrupt(encoderKnobPinB);

  for (int i = 0; i < braapDuration; i++) {
    digitalWrite(buzzPin, 1);
    delayMicroseconds(bzFreqP);
    digitalWrite(buzzPin, 0);
    delayMicroseconds(bzFreqQ);
  }
  attachInterrupt(encoderKnobPinA, tick_isr, CHANGE);
  attachInterrupt(encoderKnobPinB, tick_isr, CHANGE);
  interrupts();
}

void dit(void) { // low freq buzzer
  // int buzzPin = A5;
  int braapDuration = 400 ;

  noInterrupts();
  detachInterrupt(encoderKnobPinA);
  detachInterrupt(encoderKnobPinB);

  for (int i = 0; i < braapDuration; i++) {
    digitalWrite(buzzPin, 1);
    delayMicroseconds(bzFreqP);
    digitalWrite(buzzPin, 0);
    delayMicroseconds(bzFreqQ);
  }
  attachInterrupt(encoderKnobPinA, tick_isr, CHANGE);  // CHANGE
  attachInterrupt(encoderKnobPinB, tick_isr, CHANGE);  // CHANGE
  interrupts();
}

void m_ies(void) { // inter-element-space
  delayMicroseconds(153000);
}

void dits(void) {
  dit(); m_ies();
  m_ies();
  m_ies();
}

void m_A(void) {
  dit(); m_ies();
  dah(); m_ies();
}

void m_B(void) {
  dah(); m_ies();
  dit(); m_ies();
  dit(); m_ies();
  dit();
}

void m_C(void) {
  dah(); m_ies();
  dit(); m_ies();
  dah(); m_ies();
  dit();
}

void m_D(void) {
  dah(); m_ies();
  dit(); m_ies();
  dit();
}

void m_E(void) {
  dit();
}

void m_I(void) {
  dit(); m_ies();
  dit();
}

void m_F(void) {
  dit(); m_ies();
  dit(); m_ies();
  dah(); m_ies();
  dit(); m_ies();
}

void m_G(void) {
  dah(); m_ies();
  dah(); m_ies();
  dit();
}

void alertNovC(void) {
  m_C(); // morse 'C'
}

#define ALERT alertNovC();

void m_ics() {
  m_ies(); m_ies(); m_ies();
  m_ies(); m_ies(); m_ies();
}

void m_Q(void) {
  dah(); m_ies();
  dah(); m_ies();
  dit(); m_ies();
  dah();
}

#define DALERT m_Q();

void m_ids() {
  m_ics();
  m_ics();
  m_ics();
}

void morse(void) {
  dit(); m_ies();
  dit(); m_ies();
  dit(); m_ies();
  dit(); m_ies();
}

void morse_rare(void) {
  dit(); m_ies();
  dit(); m_ies();
  dah(); m_ies();
  dah(); m_ies();
  dit(); m_ies();
  dit();
}

void setup_neoPixel(void) { }
void setup_LCD(void) { }
void neoPixel(void) { }

void setup(void) {
  delayMicroseconds((100));
  setup_neoPixel();

  pinMode(buzzPin, OUTPUT);
  pinMode(backlight, OUTPUT);
  pinMode(ledPin, OUTPUT);

  setup_rotEnc();
  setup_LCD();

  delay(1000);
  neoPixel();
  morse();
}

void lcd_location_lat(void) {
}

void lcd_location_long(void) {
}

// super irrelevant code block omitted, just the function header is given:

void lcd_location_lat_OLD(void) {
}

// special note: itoa is used .. include an example of its use:

//    - - - - -   TODO


void tty_location(void) {
  // Serial.print(", ");
}


// count of birds (space vehicles for GPS constellation):

void lcd_birds(void) {
  // glcd.drawstring(7, 6, "birds: "); // was " birds" leading space
}




// - - - - - - - -   ROTARY again - - - -


// purpose is to change units of altitude for GPS data from feet to meters and back:

void lcd_altitude_rotary(void) {
}

void lcd_altitude_standalone(void) {
}

void lcd_rot_multi_alts(void) {
}

// that was probably about interrupting writing to the LCD without messing it up due
// to rotary encoder woes

// looks like pure rotary encoder stuff:

void reset_positions(void) {
  positionInternal = 0;
  positionExternal = 0;
  oldPositionExternal = 0;
}

void decrement_positions(void) {
  positionInternal-- ;
  positionExternal-- ;
  oldPositionExternal-- ;
}

void increment_positions(void) {
  positionInternal++ ;
  positionExternal++ ;
  oldPositionExternal++ ;
}

void flash_gordon(void) { // LED show - multi blink

  digitalWrite(backlight, LOW); // turn it off, brother
  delayMicroseconds((80 * 1500));

  for (int i=4; i>0; i--) {
      digitalWrite(backlight, HIGH); // turn it off, brother
      delayMicroseconds((80 * 1500));
      digitalWrite(backlight, LOW); // turn it off, brother
      delayMicroseconds((80 * 1500));
  }

  digitalWrite(backlight, HIGH); // turn it off, brother
  delayMicroseconds((80 * 1500));

  // extra half stanza for positive logic:
  digitalWrite(backlight, LOW); // turn it off, brother
  delayMicroseconds((80 * 1500));

}

int morseGoFlg = -1;

void lcd_rot_multi_3_to_9_alts(void) {
  if (positionExternal ==  8) {
    morseGoFlg = -1;
  }
  if (positionExternal ==  9) {
    if (morseGoFlg) {
      noInterrupts();
      m_A();
      morseGoFlg = 0;
      interrupts();
    }
  }
  if (positionExternal == 14) {
    morseGoFlg = -1;
  }
  if (positionExternal == 15) {
    if (morseGoFlg) {
      m_B();
      morseGoFlg = 0;
    }
  }
  if (positionExternal == 20) {
    morseGoFlg = -1;
  }
  if (positionExternal == 21) {
    if (morseGoFlg) {
      m_C();
      morseGoFlg = 0;
    }
  }
  if (positionExternal == 26) {
    morseGoFlg = -1;
  }
  if (positionExternal == 27) {
    if (morseGoFlg) {
      m_D();
      morseGoFlg = 0;
    }
  }
  if (positionExternal == 32) {
    morseGoFlg = -1;
  }
  if (positionExternal == 33) {
    if (morseGoFlg) {
      m_F(); m_ics();
      m_F(); m_ics();
      m_B(); m_ics();
      m_B(); m_ics();
      delay(1000);
      for (int i = 8 + 1; i > 0; i--) {
        dits();
      } morseGoFlg = 0;
      // reset_positions();
      flash_gordon();
    }
#ifdef EXTRA_NVIC_RESET
    NVIC_SystemReset();
#endif // #ifdef EXTRA_NVIC_RESET
  }
  if (positionExternal == 34) {
    morseGoFlg = -1; // to dial backwards
  }
  if (positionExternal >= MAX_POS) {
    increment_positions(); // no real sense of how this overshoots .. or doesn't
    increment_positions();
    decrement_positions();
    decrement_positions();
    decrement_positions();
    flash_gordon();
  }

  if (positionExternal <= -3) { // was -4
    reset_positions();
    flash_gordon();
  }
}

int locked = 0; // FALSE
int lock_tick = 0; // count

void proc_sw(void) {
  uint32_t milsc = 0;
  lock_tick = lock_tick + 1;
  if (lock_tick >= 3) {
    lock_tick = 0; // reset
    if (locked == 0) {
      locked = -1; // raise TRUE
    } else {
      locked = 0; // raise FALSE - was TRUE
    }
  }
  if (locked) {
    // glcd.drawstring(1, 5, "    LOCK IS: LOCKED.  ");
  } else {
    // glcd.drawstring(1, 5, "    LOCK IS: UNLOCKED.");
  }
  delayMicroseconds((1000 * 1000));
  milsc = millis();
  for (uint32_t index = 0; index < 4000000; index++) {

    if (millis() < (milsc + 50000)) {
    } else {
      index = 3999997;
    }
  }
}

void proc_tick(void) {
  int8_t thisState = sigA | (sigB << 1);
  if (oldState != thisState) {
    positionInternal += KNOBDIR[thisState | (oldState << 2)];
    hysteresis_upward += KNOBDIR[thisState | (oldState << 2)];

    if (thisState == LATCHSTATE)
      positionExternal = positionInternal >> 2;
    oldState = thisState;
  }
}

void loop_for_rotEnc(void) {
  if (tick_recent) {
    if (locked == 0) {
      // ignore tick when locked == -1
      proc_tick(); // if there was an interrupt recently, process the data in the globals sigA sigB
      lcd_rot_multi_3_to_9_alts();
    }
    tick_recent = 0; // reset tick_recent -- set the trap for a new acquisition
  }

  if (oldPositionExternal != positionExternal) {
    // lcd_rot_alt_alts(); // set the LCD altitude preference
  }

  // if(oldPositionExternal < positionExternal) { // hysteresis
  // hysteresis_upward = -1 ; // TRUE
  // } else {
  // hysteresis_upward =  0 ; // FALSE
  // }

  oldPositionExternal = positionExternal;
  lcd_rot_multi_alts(); // not single-shot; do every loop
}

void loop(void) {
  if (tick_recent) {
    loop_for_rotEnc();
  }
  if (timer > millis()) timer = millis();
  if (millis() - timer > 1700) {
    timer = millis(); // reset the timer
  }
}
