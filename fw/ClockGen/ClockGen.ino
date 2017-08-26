#include <stdint.h>
#include <avr/pgmspace.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Bounce2.h>

// display refresh interval (about 8 times per second)
#define REFRESH_MS 125

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
const char modename_manual[] PROGMEM = "Manual";
const char modename_slow0[] PROGMEM = ".5 Hz";
const char modename_slow1[] PROGMEM = "1 Hz";
const char modename_slow2[] PROGMEM = "2 Hz";
const char modename_slow3[] PROGMEM = "4 Hz";
const char modename_slow4[] PROGMEM = "8 Hz";
const char modename_slow5[] PROGMEM = "16 Hz";
const char modename_slow6[] PROGMEM = "32 Hz";
const char modename_slow7[] PROGMEM = "64 Hz";
const char modename_fast0[] PROGMEM = "14.4 KHz";
const char modename_fast1[] PROGMEM = "28.8 KHz";
const char modename_fast2[] PROGMEM = "57.6 KHz";
const char modename_fast3[] PROGMEM = "115 KHz";
const char modename_fast4[] PROGMEM = "230 KHz";
const char modename_fast5[] PROGMEM = "461 KHz";
const char modename_fast6[] PROGMEM = "921 KHz";
const char modename_fast7[] PROGMEM = "1.84 MHz";

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
  { SLOW, 0 },
  { SLOW, 1 },
  { SLOW, 2 },
  { SLOW, 3 },
  { SLOW, 4 },
  { SLOW, 5 },
  { SLOW, 6 },
  { SLOW, 7 },
  { FAST, 0 },
  { FAST, 1 },
  { FAST, 2 },
  { FAST, 3 },
  { FAST, 4 },
  { FAST, 5 },
  { FAST, 6 },
  { FAST, 7 },
};
const uint8_t NUM_MODES = (uint8_t) (sizeof(s_modes)/sizeof(Mode));

// State data
uint8_t s_enabled;
uint8_t s_mode;

// Button debouncing
Bounce s_btn1 = Bounce(); // pin 12
Bounce s_btn2 = Bounce(); // pin 11
uint8_t s_buttons = 0xFF; // bits corresponding to button readings, initially not pressed

// Timestamp (for periodically updating the display)
unsigned long s_ts;

void setup() {
  Serial.begin(9600);

  // Initialize buttons
  pinMode(12, INPUT_PULLUP);
  s_btn1.attach(12);
  s_btn1.interval(5);
  pinMode(11, INPUT_PULLUP);
  s_btn2.attach(11);
  s_btn2.interval(5);

  // Initialize OLED display
  // Note that the Adafruit display uses address 0x3D, but the
  // cheap ebay displays seem to use 0x3C
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.clearDisplay();

  // Debugging
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  s_ts = millis();
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
  s_btn1.update();
  s_btn2.update();

  // Debugging button input
  digitalWrite(2, s_btn1.read() ? HIGH : LOW);
  digitalWrite(3, s_btn2.read() ? HIGH : LOW);

  // Determine button events
  uint8_t current = 0xFF;
  current = (current << 1) | (s_btn1.read() ? 1 : 0);
  current = (current << 1) | (s_btn2.read() ? 1 : 0);
  uint8_t evt1 = checkButton(s_buttons, current, 0);
  uint8_t evt2 = checkButton(s_buttons, current, 1);
  handleButton1(evt1);
  handleButton2(evt2);
  s_buttons = current;

  unsigned long now = millis();
  if (!s_ts || (now > s_ts && (now - s_ts) >= REFRESH_MS)) {
    updateDisplay();
    s_ts = now;
  }
}

void handleButton1(uint8_t evt) {
  if (evt != PRESS) {
    return;
  }
  if (s_mode == 0) {
    s_mode = NUM_MODES - 1;
  } else {
    s_mode--;
  }
}

void handleButton2(uint8_t evt) {
  if (evt != PRESS) {
    return;
  }
  if (s_mode == NUM_MODES - 1) {
    s_mode = 0;
  } else {
    s_mode++;
  }
}

uint8_t s_ticks;

void updateDisplay() {
  display.clearDisplay();

  display.setTextSize(1);

  if (s_enabled) {
    display.setTextColor(BLACK, WHITE);
    display.drawRoundRect(4, 6, 30, 14, 4, WHITE);
    display.setCursor(14, 10);
    display.println("EN");
  } else {
    display.setTextColor(BLACK, WHITE);
    display.drawRoundRect(4, 6, 29, 15, 4, WHITE);
    display.setTextColor(WHITE);
    display.setCursor(10, 10);
    display.println("DIS");
  }
  
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(16,32);
  char buffer[12];
  strcpy_P(buffer, (char*)pgm_read_word(&(s_modenames[s_mode])));
  display.println(buffer);

  display.setTextSize(1);
  display.setCursor(120, 56);
  display.println((int) (s_ticks & 0xF), HEX);
  s_ticks++;
  
  display.display();
}

