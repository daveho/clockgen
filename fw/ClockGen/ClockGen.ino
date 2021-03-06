#include <stdint.h>
#include <avr/pgmspace.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Bounce2.h>

// display refresh interval (about 8 times per second)
#define REFRESH_MS 125

// port D bitmask for clock divisor selection pins (PD2..PD4)
#define CDIV_SEL_MASK 0x1C

// pins for -FASTEN and -CTCLR signals
#define FASTEN_PIN 5
#define CTCLR_PIN  6

// SLOWCLK pin
#define SLOWCLK_PIN 7

// RST and -RST pin
#define RST_PIN  14
#define NRST_PIN 15 // "N" stands for "Negated", i.e., active low

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

// Button event types
enum ButtonEvent {
  PRESS,
  RELEASE,
  NO_CHANGE,
};

// Mode types
enum ModeType {
  MANUAL,
  SLOW,
  FAST,
};

// Mode struct
struct Mode {
  uint8_t type;
  uint8_t speed;
};

// Mode names (as strings in program memory)
// The fast modes assume a 4 MHz oscillator
const char modename_manual[] PROGMEM = "Manual";
const char modename_slow0[] PROGMEM = "1 Hz";
const char modename_slow1[] PROGMEM = "2 Hz";
const char modename_slow2[] PROGMEM = "4 Hz";
const char modename_slow3[] PROGMEM = "8 Hz";
const char modename_slow4[] PROGMEM = "16 Hz";
const char modename_slow5[] PROGMEM = "32 Hz";
const char modename_slow6[] PROGMEM = "64 Hz";
const char modename_slow7[] PROGMEM = "128 Hz";
const char modename_fast0[] PROGMEM = "15.62 KHz";
const char modename_fast1[] PROGMEM = "31.25 KHz";
const char modename_fast2[] PROGMEM = "62.5 KHz";
const char modename_fast3[] PROGMEM = "125 KHz";
const char modename_fast4[] PROGMEM = "250 KHz";
const char modename_fast5[] PROGMEM = "500 KHz";
const char modename_fast6[] PROGMEM = "1 MHz";
const char modename_fast7[] PROGMEM = "2 MHz";

// Mode name table (see https://www.arduino.cc/en/Reference/PROGMEM)
const char* const s_modenames[] PROGMEM = {
  modename_manual,
  modename_slow0,
  modename_slow1,
  modename_slow2,
  modename_slow3,
  modename_slow4,
  modename_slow5,
  modename_slow6,
  modename_slow7,
  modename_fast0,
  modename_fast1,
  modename_fast2,
  modename_fast3,
  modename_fast4,
  modename_fast5,
  modename_fast6,
  modename_fast7,
};

// Modes
const Mode s_modes[] PROGMEM = {
  { MANUAL, 0 },
  { SLOW, 7 }, // slowest slow mode
  { SLOW, 6 },
  { SLOW, 5 },
  { SLOW, 4 },
  { SLOW, 3 },
  { SLOW, 2 },
  { SLOW, 1 },
  { SLOW, 0 }, // fastest slow mode
  { FAST, 7 }, // slowest fast mode
  { FAST, 6 },
  { FAST, 5 },
  { FAST, 4 },
  { FAST, 3 },
  { FAST, 2 },
  { FAST, 1 },
  { FAST, 0 }, // fastest fast mode
};
const uint8_t NUM_MODES = (uint8_t) (sizeof(s_modes)/sizeof(Mode));

// State data
uint8_t s_enabled;
uint8_t s_modeNum;
uint8_t s_nextModeNum;
Mode s_curMode;
uint8_t s_slowCount; // used to generate the slow clock frequencies

// Button debouncing
Bounce s_btns[5];
uint8_t s_buttons = 0xFF; // bits corresponding to button readings, initially not pressed

// Timestamp (for periodically updating the display)
unsigned long s_ts;

void setup() {
  Serial.begin(9600);

  // Initialize current mode: the timer1 interrupt handler
  // will use this data to determine whether to generate the
  // slow clock
  memcpy_P(&s_curMode, &s_modes[s_modeNum], sizeof(Mode));

  // Initialize buttons
  for (uint8_t i = 0; i < 5; i++) {
    uint8_t pin = 12 - i;
    pinMode(pin, INPUT_PULLUP);
    s_btns[i].attach(pin);
    s_btns[i].interval(5);
  }

  // Configure port D for clock divisor selection outputs
  DDRD |= CDIV_SEL_MASK;
  
  PORTD = (PORTD & ~CDIV_SEL_MASK) | (7 << 2); // select slowest fastclock divisor

  // Enable outputs for -CTCLR, -FASTEN signals
  pinMode(FASTEN_PIN, OUTPUT);
  pinMode(CTCLR_PIN, OUTPUT);
  digitalWrite(FASTEN_PIN, HIGH); // initially not asserted
  digitalWrite(CTCLR_PIN, HIGH);  // initially not asserted

  // Enable SLOWCLK output
  pinMode(SLOWCLK_PIN, OUTPUT);

  // Enable RST and -RST outputs
  pinMode(RST_PIN, OUTPUT);
  pinMode(NRST_PIN, OUTPUT);

  // Initially, reset is not asserted
  digitalWrite(RST_PIN, LOW);
  digitalWrite(NRST_PIN, HIGH);

  // Reset fast clock counter and output flip flop
  digitalWrite(CTCLR_PIN, LOW);
  delay(1);
  digitalWrite(CTCLR_PIN, HIGH);

  // For testing slow clock timer interrupt
  pinMode(LED_BUILTIN, OUTPUT);

  // Start from known-good default outputs
  setDefaultOutputs();

  // Configure timer1 for slowclock generation
  // See: http://www.hobbytronics.co.uk/arduino-timer-interrupts
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  // We'll configure timer1 to overflow 256 times per second
  TCNT1 = 3036;             // preload timer 65536-16MHz/256Hz
  TCCR1B |= (1 << CS10);    // no prescaler
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();

  // Initialize OLED display
  // Note that the Adafruit display uses address 0x3D, but the
  // cheap ebay displays seem to use 0x3C
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.clearDisplay();

  s_ts = millis();
}

// Timer1 interrupt handler: generates slowclock output
ISR(TIMER1_OVF_vect) {
  if (s_curMode.type != SLOW || !s_enabled) {
    return;
  }
  uint8_t poll = (s_slowCount & (1 << s_curMode.speed)) ? LOW : HIGH;
  digitalWrite(LED_BUILTIN, poll);
  digitalWrite(SLOWCLK_PIN, poll);
  s_slowCount++;
}

uint8_t checkButton(uint8_t last, uint8_t current, uint8_t bit) {
  uint8_t lastBit = last & (1 << bit);
  uint8_t curBit = current & (1 << bit);
  if (lastBit == curBit) {
    return NO_CHANGE;
  } else if (curBit) {
    return RELEASE;
  } else {
    return PRESS;
  }
}

void loop() {
  // Debounce buttons
  for (uint8_t i = 0; i < 5; i++) {
    s_btns[i].update();
  }

  // Determine button events
  uint8_t current = 0xFF;
  for (uint8_t i = 0; i < 5; i++) {
    current = (current << 1) | (s_btns[4-i].read() ? 1 : 0);
  }
  uint8_t evt1 = checkButton(s_buttons, current, 0);
  uint8_t evt2 = checkButton(s_buttons, current, 1);
  uint8_t evt3 = checkButton(s_buttons, current, 2);
  uint8_t evt4 = checkButton(s_buttons, current, 3);
  uint8_t evt5 = checkButton(s_buttons, current, 4);
  handleButton1(evt1);
  handleButton2(evt2);
  handleButton3(evt3);
  handleButton4(evt4);
  handleButton5(evt5);
  s_buttons = current;

  unsigned long now = millis();
  if (!s_ts || (now > s_ts && (now - s_ts) >= REFRESH_MS)) {
    updateDisplay();
    s_ts = now;
  }
}

void setDefaultOutputs() {
  // Disable fast clock
  digitalWrite(FASTEN_PIN, HIGH);

  // Set slow clock to default (low, producing high clock output when inverted)
  digitalWrite(SLOWCLK_PIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
}

void onModeChange() {
  noInterrupts();
  
  s_modeNum = s_nextModeNum;

  // Copy Mode from progmem
  memcpy_P(&s_curMode, &s_modes[s_modeNum], sizeof(Mode));

  // Start from known-good default outputs
  setDefaultOutputs();

  if (s_enabled) {
    // Enable new mode
    if (s_curMode.type == FAST) {
      // update clock divisor
      PORTD = (PORTD & ~CDIV_SEL_MASK) | (s_curMode.speed << 2);
      // reset fastclock counter
      digitalWrite(CTCLR_PIN, LOW);
      delayMicroseconds(10);
      digitalWrite(CTCLR_PIN, HIGH);
      // reenable fast clock
      digitalWrite(FASTEN_PIN, LOW);
    } else if (s_curMode.type == SLOW) {
      // reset slow counter
      s_slowCount = 0;
    } else {
      // Manual mode: set slowclock output to be consistent with toggle input
      digitalWrite(SLOWCLK_PIN, s_btns[3].read() ? LOW : HIGH);
    }
  }

  interrupts();
}

// Left button (select previous mode)
void handleButton1(uint8_t evt) {
  if (evt != PRESS) {
    return;
  }
  if (s_modeNum == 0) {
    s_nextModeNum = NUM_MODES - 1;
  } else {
    s_nextModeNum = s_modeNum - 1;
  }
  onModeChange();
}

// Right button (select next mode)
void handleButton2(uint8_t evt) {
  if (evt != PRESS) {
    return;
  }
  if (s_modeNum == NUM_MODES - 1) {
    s_nextModeNum = 0;
  } else {
    s_nextModeNum = s_modeNum + 1;
  }
  onModeChange();
}

// Enable/disable button
void handleButton3(uint8_t evt) {
  if (evt != PRESS) {
    return;
  }
  s_enabled ^= 1;
  if (s_enabled) {
    // Enable the currently-selected mode
    onModeChange();
  } else {
    // Reset to default outputs
    setDefaultOutputs();
  }
}

// Manual slowclock toggle button (only allowed in manual mode)
void handleButton4(uint8_t evt) {
  if (!s_enabled || s_curMode.type != MANUAL || evt == NO_CHANGE) {
    return;
  }
  // we're in manual mode, so drive SLOWCLK directly from
  // the button input (recall that SLOWCLK is inverted by
  // the NOR gate that combines the slow and fast clock signals)
  if (evt == PRESS) {
    digitalWrite(SLOWCLK_PIN, HIGH); // will make clock output go low
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    digitalWrite(SLOWCLK_PIN, LOW);  // will make clock output go high
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

// Reset button (pressed=reset asserted)
void handleButton5(uint8_t evt) {
  if (evt == NO_CHANGE) {
    return;
  }
  if (evt == PRESS) {
    // assert reset signals
    digitalWrite(RST_PIN, HIGH);
    digitalWrite(NRST_PIN, LOW);
  } else { // release
    // deassert reset signals
    digitalWrite(RST_PIN, LOW);
    digitalWrite(NRST_PIN, HIGH);
  }
}

#ifdef DEBUG_DISPLAY_UPDATE
uint8_t s_ticks;
#endif

#define EN_DIS_X 0
#define CLK_X     48
#define CLK_IND_X (CLK_X+24)
#define RST_X     96
#define RST_IND_X (RST_X+24)

#define TEXT_Y    4
#define RECT_Y    0
#define IND_Y     7

void updateDisplay() {
  display.clearDisplay();

  display.setTextSize(1);

  if (s_enabled) {
    display.setTextColor(BLACK, WHITE);
    display.fillRoundRect(EN_DIS_X, RECT_Y, 29, 15, 4, WHITE);
    display.setCursor(9, TEXT_Y);
    display.print("EN");
  } else {
    display.setTextColor(BLACK, WHITE);
    display.drawRoundRect(EN_DIS_X, RECT_Y, 29, 15, 4, WHITE);
    display.setTextColor(WHITE);
    display.setCursor(6, TEXT_Y);
    display.print("DIS");
  }

  display.setTextColor(WHITE);
  display.setCursor(CLK_X, TEXT_Y);
  display.print("CLK");
  display.drawCircle(CLK_IND_X, IND_Y, 5, WHITE);
  // The clock inidicator is only meaningful in manual mode
  if (s_curMode.type == MANUAL) {
    // keep in mind that the SLOWCLK output is inverted
    // by the NOR gate that combines the slow and fast
    // clock signals
    if (digitalRead(SLOWCLK_PIN) == LOW) {
      display.fillCircle(CLK_IND_X, IND_Y, 2, WHITE);
    }
  }

  display.setTextColor(WHITE);
  display.setCursor(RST_X, TEXT_Y);
  display.print("RST");
  display.drawCircle(RST_IND_X, IND_Y, 5, WHITE);
  if (digitalRead(RST_PIN) == HIGH) {
    // reset asserted
    display.fillCircle(RST_IND_X, IND_Y, 2, WHITE);
  }
  
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(16,36);
  char buffer[12];
  strcpy_P(buffer, (char*)pgm_read_word(&(s_modenames[s_modeNum])));
  display.print(buffer);

#ifdef DEBUG_DISPLAY_UPDATE
  display.setTextSize(1);
  display.setCursor(120, 56);
  display.print((int) (s_ticks & 0xF), HEX);
  s_ticks++;
#endif
  
  display.display();
}

