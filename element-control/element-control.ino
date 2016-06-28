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

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define PIN_ENCODER_A      3
#define PIN_ENCODER_B      5
#define TRINKET_PINx       PIND
#define PIN_ENCODER_SWITCH 4

#define RED_PIN 2
#define GREEN_PIN 13
#define LED_ON LOW
#define LED_OFF HIGH

#define OFF_PIN -1
#define BOIL1_PIN 8
#define BOIL2_PIN 9
#define HLT1_PIN 10
#define HLT2_PIN 11
#define RIMS_PIN 12
#define ELEMENT_OFF 1
#define ELEMENT_ON 0
 
static uint8_t enc_prev_pos   = 0;
static uint8_t enc_flags      = 0;
static char    sw_was_pressed = 0;

LiquidCrystal_I2C lcd(0x27,20,4);

static char element_configuration_labels[7][20];
static int element_configuration_pins[7][2] = {
  {   OFF_PIN,   OFF_PIN },
  {   OFF_PIN,  RIMS_PIN },
  { BOIL1_PIN, BOIL2_PIN },
  { BOIL1_PIN,  HLT1_PIN },
  { BOIL1_PIN,  RIMS_PIN },
  {  RIMS_PIN,  HLT1_PIN },
  {  HLT1_PIN,  HLT2_PIN }
};

static int proposed_element_configuration = 0;
static int actual_element_configuration = proposed_element_configuration;
static int last_requested_proposed_configuration = -1;
static int last_requested_actual_configuration = -1;

static int element_pins[5] = { BOIL1_PIN, BOIL2_PIN, HLT1_PIN, HLT2_PIN, RIMS_PIN };

#define UI_MODE_RUNNING 0
#define UI_MODE_CHOOSING 1

static int ui_mode = UI_MODE_RUNNING;
static bool ui_mode_indicator = false;
static unsigned long ui_last_choosing_mode_started_at = 0;
static unsigned long ui_last_choosing_indicator_toggled_at = 0;

void setup()
{
  // set element pins to output and off
  for (int i = 0; i < 5; i++) {
    pinMode(element_pins[i], OUTPUT);
    digitalWrite(element_pins[i], ELEMENT_OFF);
  }

  // allocate strings for the element mode labels
  strcpy((char*)&element_configuration_labels[0], "  OFF / OFF  ");
  strcpy((char*)&element_configuration_labels[1], "  OFF / RIMS ");
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
  lcd.backlight();
  begin_running();
}

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
 
void loop()
{ 
  if (ui_mode == UI_MODE_CHOOSING) {
    if (millis() - ui_last_choosing_mode_started_at > 4000) {
      proposed_element_configuration = actual_element_configuration;
      begin_running();
    } else {
      if (millis() - ui_last_choosing_indicator_toggled_at > 200) {
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

  char output[21];
 
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
        actual_element_configuration = proposed_element_configuration;
        begin_running();
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

  
void sync_proposed_configuration_lcd() {
  if (last_requested_proposed_configuration != proposed_element_configuration) {
    last_requested_proposed_configuration = proposed_element_configuration;
    lcd.setCursor(0,0);
    lcd.print(element_configuration_labels[proposed_element_configuration]);
  }
}

void all_off() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(element_pins[i], ELEMENT_OFF);
  }
}

void begin_running() {
  ui_mode = UI_MODE_RUNNING;
  led_running();
  last_requested_proposed_configuration = -1;
  sync_proposed_configuration_lcd();
  set_indicator(" ");
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

