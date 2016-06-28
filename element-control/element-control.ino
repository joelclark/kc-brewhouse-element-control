/*
 * New Forest Brewhouse Control
 * Joel Clark <joel@angrybits.com>
 * MIT License, Copyright 2016
 * 
 * This firmware provides the following features:
 * 
 *    -- Element selector / lockout
 * 
 * 
 * 
 * Rotary encoder logic lifted from the code found here:
 *   https://learn.adafruit.com/pro-trinket-rotary-encoder/example-rotary-encoder-volume-control
 *   
 * Many thanks to Mike Barela at Adafruit for releasing this stuff under a permissive MIT license.
 * 
 * 
 */

// ---------------------------------------------------------------------------------- lifted code

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

static uint8_t enc_prev_pos   = 0;
static uint8_t enc_flags      = 0;
static char    sw_was_pressed = 0;

LiquidCrystal_I2C lcd(0x27,20,4);

#define PIN_ENCODER_A      3
#define PIN_ENCODER_B      5
#define TRINKET_PINx       PIND
#define PIN_ENCODER_SWITCH 4

// ---------------------------------------------------------------------------------- declarations

/*
 * The panel will use a SparkFun RGB rotary encoder for a UI,
 * and there will be a simple color scheme to provide feedback.
 * 
 * -- When all elements are off, it will be steady green.
 * -- When any elements are on, it will be steady red.
 * -- When in choosing mode, it will flash between either red or green and yellow
 *    -- Red vs green depends on if any elements were firing when the
 *       user went into choosing mode.
 * -- If choosing mode times out, it will return to the appropriate steady color
 * -- If a choice is confirmed, it will flash a few times before going steady
 */
#define RED_PIN 2
#define GREEN_PIN 13
#define LED_ON LOW
#define LED_OFF HIGH

/*
 * The ulimate goal here is to choose which 2 out of 5 heating 
 * elements to run.  This is necessary because we'll be running
 * a 50 amp circuit with nearly 100 amps worth of elements in 
 * the system.  This pins will go to a relay board and open 
 * and close the low voltage line running from the PID to the SSR.
 */
#define OFF_PIN -1
#define BOIL1_PIN 8
#define BOIL2_PIN 9
#define HLT1_PIN 10
#define HLT2_PIN 11
#define RIMS_PIN 12
#define ELEMENT_OFF LOW
#define ELEMENT_ON HIGH

static char element_configuration_labels[7][20];
static int element_configuration_pins[7][2] = {
  {   OFF_PIN,   OFF_PIN },
  {  RIMS_PIN,   OFF_PIN },
  { BOIL1_PIN, BOIL2_PIN },
  { BOIL1_PIN,  HLT1_PIN },
  { BOIL1_PIN,  RIMS_PIN },
  {  RIMS_PIN,  HLT1_PIN },
  {  HLT1_PIN,  HLT2_PIN }
};

static int element_pins[5] = { BOIL1_PIN, BOIL2_PIN, HLT1_PIN, HLT2_PIN, RIMS_PIN };

/*
 * These variables will house the state for managing the configuration 
 */
 
#define UI_MODE_RUNNING 0
#define UI_MODE_CHOOSING 1
#define UI_MODE_TIMEOUT 4000
#define UI_MODE_TOGGLE_TIME 200

static int proposed_element_configuration = 0;
static int actual_element_configuration = proposed_element_configuration;
static int last_requested_proposed_configuration = -1;
static int last_requested_actual_configuration = -1;
static int ui_mode = UI_MODE_RUNNING;
static bool ui_mode_indicator = false;
static unsigned long ui_last_choosing_mode_started_at = 0;
static unsigned long ui_last_choosing_indicator_toggled_at = 0;

// ---------------------------------------------------------------------------------- setup

void setup()
{
  // set element pins to output and off
  for (int i = 0; i < 5; i++) {
    pinMode(element_pins[i], OUTPUT);
    digitalWrite(element_pins[i], ELEMENT_OFF);
  }

  // allocate strings for the element mode labels
  strcpy((char*)&element_configuration_labels[0], "  OFF / OFF  ");
  strcpy((char*)&element_configuration_labels[1], " RIMS / OFF  ");
  strcpy((char*)&element_configuration_labels[2], "BOIL1 / BOIL2");
  strcpy((char*)&element_configuration_labels[3], "BOIL1 / HLT1 ");
  strcpy((char*)&element_configuration_labels[4], "BOIL1 / RIMS ");
  strcpy((char*)&element_configuration_labels[5], " RIMS / HLT1 ");
  strcpy((char*)&element_configuration_labels[6], " HLT1 / HLT2 ");
  
  // ready the rotary encoder pins
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);

  // for the led indicator
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  led_off();

  // ready the encoder push button switch (active high, uses pull-down)
  pinMode(PIN_ENCODER_SWITCH, INPUT);

  // get an initial reading on the encoder pins
  if (digitalRead(PIN_ENCODER_A) == LOW) {
    enc_prev_pos |= (1 << 0);
  }
  if (digitalRead(PIN_ENCODER_B) == LOW) {
    enc_prev_pos |= (1 << 1);
  }

  lcd.init();
  lcd.clear();
  lcd.backlight();
  begin_running(true);
}
 
// ---------------------------------------------------------------------------------- loop

void loop()
{ 
  if (ui_mode == UI_MODE_CHOOSING) {
    if (millis() - ui_last_choosing_mode_started_at > UI_MODE_TIMEOUT) {
      proposed_element_configuration = actual_element_configuration;
      begin_running(false);
    } else {
      if (millis() - ui_last_choosing_indicator_toggled_at > UI_MODE_TOGGLE_TIME) {
        toggle_choosing_indicator();
      }
    }
  }
  
  int8_t enc_action = 0; // 1 or -1 if moved, sign is direction
 
  // note: for better performance, the code will use
  // direct port access techniques
  // http://www.arduino.cc/en/Reference/PortManipulation
  uint8_t enc_cur_pos = 0;
  // read in the encoder state first
  if (bit_is_clear(TRINKET_PINx, PIN_ENCODER_A)) {
    enc_cur_pos |= (1 << 0);
  }
  if (bit_is_clear(TRINKET_PINx, PIN_ENCODER_B)) {
    enc_cur_pos |= (1 << 1);
  }
 
  // if any rotation at all
  if (enc_cur_pos != enc_prev_pos)
  {
    if (enc_prev_pos == 0x00)
    {
      // this is the first edge
      if (enc_cur_pos == 0x01) {
        enc_flags |= (1 << 0);
      }
      else if (enc_cur_pos == 0x02) {
        enc_flags |= (1 << 1);
      }
    }
 
    if (enc_cur_pos == 0x03)
    {
      // this is when the encoder is in the middle of a "step"
      enc_flags |= (1 << 4);
    }
    else if (enc_cur_pos == 0x00)
    {
      // this is the final edge
      if (enc_prev_pos == 0x02) {
        enc_flags |= (1 << 2);
      }
      else if (enc_prev_pos == 0x01) {
        enc_flags |= (1 << 3);
      }
 
      // check the first and last edge
      // or maybe one edge is missing, if missing then require the middle state
      // this will reject bounces and false movements
      if (bit_is_set(enc_flags, 0) && (bit_is_set(enc_flags, 2) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 2) && (bit_is_set(enc_flags, 0) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 1) && (bit_is_set(enc_flags, 3) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
      else if (bit_is_set(enc_flags, 3) && (bit_is_set(enc_flags, 1) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
 
      enc_flags = 0; // reset for next time
    }
  }
 
  enc_prev_pos = enc_cur_pos;
 
  if (enc_action > 0) {
    if (proposed_element_configuration < 6) {
      proposed_element_configuration++;
      begin_choosing();
    }
  }
  else if (enc_action < 0) {
    if (proposed_element_configuration > 0) {
      proposed_element_configuration--;  
      begin_choosing();
    }
  }
  
  if (digitalRead(PIN_ENCODER_SWITCH) == 1) 
  {
    if (sw_was_pressed != 1) // only on initial press, so the keystroke is not repeated while the button is held down
    {
      delay(5); // debounce delay
      if (ui_mode == UI_MODE_CHOOSING) {
        bool is_changing = actual_element_configuration != proposed_element_configuration;
        actual_element_configuration = proposed_element_configuration;
        begin_running(is_changing);
        delay(50);
      }
    }
  }
  else
  {
    if (sw_was_pressed != 0) {
      delay(5); // debounce delay
      sw_was_pressed = 0;
    }
  }
}

// ---------------------------------------------------------------------------------- support

void toggle_choosing_indicator() {
  ui_last_choosing_indicator_toggled_at = millis();
  ui_mode_indicator = !ui_mode_indicator;
  if (ui_mode_indicator) {
    led_yellow();
    set_indicator("?");
  } else {
    led_running();
    set_indicator(" ");
  }
}
  
void sync_proposed_configuration_lcd() {
  if (last_requested_proposed_configuration != proposed_element_configuration) {
    last_requested_proposed_configuration = proposed_element_configuration;
    lcd.setCursor(0,0);
    lcd.print(element_configuration_labels[proposed_element_configuration]);
  }
}


void sync_actual_configration_to_relay_board() {
  all_off();
  for (int element_offset = 0; element_offset < 2; element_offset++) {
    fire_relay(element_offset);
  }
}

void fire_relay(int element_offset) {

  char pin_label[3];
  int pin = element_configuration_pins[actual_element_configuration][element_offset];
  
  lcd.setCursor(0,element_offset+2);
  lcd.print("Pin: ");
  
  if (pin > 0) {
    digitalWrite(pin, ELEMENT_ON);
    sprintf((char*)&pin_label, "%2d", pin);
    lcd.print(pin_label);
  } else {
    lcd.print("None"); 
  }
  
  lcd.print("   ");
}

void all_off() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(element_pins[i], ELEMENT_OFF);
  }
}

void begin_running(bool is_changing) {
  ui_mode = UI_MODE_RUNNING;
  last_requested_proposed_configuration = -1;
  sync_proposed_configuration_lcd();
  set_indicator(" ");
  sync_actual_configration_to_relay_board();
  if (is_changing) {
    bool state = false;
    for (int i = 0; i < 5; i++) {
      led_running();
      delay(50);
      led_off();
      delay(50);
    }
  }
  led_running();
}

void begin_choosing() {
  all_off();
  ui_mode = UI_MODE_CHOOSING;
  ui_last_choosing_mode_started_at = millis();
  sync_proposed_configuration_lcd();
}

void set_indicator(const char *value) {
  lcd.setCursor(19,0);
  lcd.print(value);
}

void led_running() {
  if (actual_element_configuration == 0) {
    led_green();
  } else {
    led_red();
  }
}

void led_red() {
  digitalWrite(GREEN_PIN, LED_OFF);
  digitalWrite(RED_PIN, LED_ON);
}

void led_green() {
  digitalWrite(GREEN_PIN, LED_ON);
  digitalWrite(RED_PIN, LED_OFF);
}

void led_yellow() {
  digitalWrite(GREEN_PIN, LED_ON);
  digitalWrite(RED_PIN, LED_ON);
}

void led_off() {
  digitalWrite(GREEN_PIN, LED_OFF);
  digitalWrite(RED_PIN, LED_OFF);
}

