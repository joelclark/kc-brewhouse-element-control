

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
#include <Adafruit_NeoPixel.h>

static uint8_t enc_prev_pos   = 0;
static uint8_t enc_flags      = 0;
static char    sw_was_pressed = 0;

LiquidCrystal_I2C lcd(0x27,20,4);

#define PIN_ENCODER_A         3
#define PIN_ENCODER_B         5
#define TRINKET_PINx          PIND
#define PIN_ENCODER_SWITCH    4     // active high, use 4.7K pull-down

// ---------------------------------------------------------------------------------- RGB led management

#define NEO_PIN               6
#define NEO_NUMPIXELS         8     // indicator, 5 x element, 2 x pump
#define NEO_PIXEL_BRIGHTNESS  15    // 0-255
#define NEO_MAIN_LED_OFFSET   0

//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NEO_NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ400);

// pre-calculate colors
static uint32_t neo_color_off     = pixels.Color(0, 0, 0);
static uint32_t neo_color_red     = pixels.Color(NEO_PIXEL_BRIGHTNESS, 0, 0);
static uint32_t neo_color_yellow  = pixels.Color(NEO_PIXEL_BRIGHTNESS, NEO_PIXEL_BRIGHTNESS, 0);
static uint32_t neo_color_green   = pixels.Color(0, NEO_PIXEL_BRIGHTNESS, 0);
static uint32_t neo_color_blue    = pixels.Color(0, 0, NEO_PIXEL_BRIGHTNESS);

// ---------------------------------------------------------------------------------- declarations


/*
 * The ulimate goal here is to choose which 2 out of 5 heating 
 * elements to run.  This is necessary because we'll be running
 * a 50 amp circuit with nearly 100 amps worth of elements in 
 * the system.  This pins will go to a relay board and open 
 * and close the low voltage line running from the PID to the SSR.
 */
#define OFF_PIN                       -1
#define BOIL1_PIN                     8
#define BOIL2_PIN                     9
#define HLT1_PIN                      10
#define HLT2_PIN                      11
#define RIMS_PIN                      12
#define MAX_ELEMENT_PIN               RIMS_PIN
#define MAX_ELEMENT_LABEL_SIZE        6

#define ELEMENT_COUNT                 5
#define OFF_LED_OFFSET                -1
#define BOIL1_LED_OFFSET              1
#define BOIL2_LED_OFFSET              2
#define HLT1_LED_OFFSET               3
#define HLT2_LED_OFFSET               4
#define RIMS_LED_OFFSET               5

#define ELEMENT_OFF                   LOW
#define ELEMENT_ON                    HIGH

#define ELEMENT_CONFIGURATION_COUNT   7

#define UI_MODE_RUNNING               0
#define UI_MODE_CHOOSING              1
#define UI_MODE_TIMEOUT               8000 //ms
#define UI_MODE_TOGGLE_TIME           250  //ms

static char element_configuration_labels[ELEMENT_CONFIGURATION_COUNT][20];

static int element_configuration_pins[ELEMENT_CONFIGURATION_COUNT][2] = {
  {   OFF_PIN,   OFF_PIN },
  {  RIMS_PIN,   OFF_PIN },
  { BOIL1_PIN, BOIL2_PIN },
  { BOIL1_PIN,  HLT1_PIN },
  { BOIL1_PIN,  RIMS_PIN },
  {  RIMS_PIN,  HLT1_PIN },
  {  HLT1_PIN,  HLT2_PIN }
};

static int element_configuration_led_offsets[ELEMENT_CONFIGURATION_COUNT][2] = {
  {   OFF_LED_OFFSET,   OFF_LED_OFFSET },
  {  RIMS_LED_OFFSET,   OFF_LED_OFFSET },
  { BOIL1_LED_OFFSET, BOIL2_LED_OFFSET },
  { BOIL1_LED_OFFSET,  HLT1_LED_OFFSET },
  { BOIL1_LED_OFFSET,  RIMS_LED_OFFSET },
  {  RIMS_LED_OFFSET,  HLT1_LED_OFFSET },
  {  HLT1_LED_OFFSET,  HLT2_LED_OFFSET }
};

static int element_pins[ELEMENT_COUNT] = { BOIL1_PIN, BOIL2_PIN, HLT1_PIN, HLT2_PIN, RIMS_PIN };
static int element_leds[ELEMENT_COUNT] = { BOIL1_LED_OFFSET, BOIL2_LED_OFFSET, HLT1_LED_OFFSET, HLT2_LED_OFFSET, RIMS_LED_OFFSET };

static char * element_names_by_pin[MAX_ELEMENT_PIN + 1];

/*
 * These variables will house the state for managing the configuration 
 */

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
  for (int i = 0; i < ELEMENT_COUNT; i++) {
    pinMode(element_pins[i], OUTPUT);
    digitalWrite(element_pins[i], ELEMENT_OFF);
  }
  
  // allocate strings for the element mode labels
  strcpy((char *) &element_configuration_labels[0], "  OFF / OFF  ");
  strcpy((char *) &element_configuration_labels[1], " RIMS / OFF  ");
  strcpy((char *) &element_configuration_labels[2], "BOIL1 / BOIL2");
  strcpy((char *) &element_configuration_labels[3], "BOIL1 / HLT1 ");
  strcpy((char *) &element_configuration_labels[4], "BOIL1 / RIMS ");
  strcpy((char *) &element_configuration_labels[5], " RIMS / HLT1 ");
  strcpy((char *) &element_configuration_labels[6], " HLT1 / HLT2 ");

  element_names_by_pin[BOIL1_PIN] = (char *) malloc(MAX_ELEMENT_LABEL_SIZE * sizeof(char));
  element_names_by_pin[BOIL2_PIN] = (char *) malloc(MAX_ELEMENT_LABEL_SIZE * sizeof(char));
  element_names_by_pin[HLT1_PIN]  = (char *) malloc(MAX_ELEMENT_LABEL_SIZE * sizeof(char));
  element_names_by_pin[HLT2_PIN]  = (char *) malloc(MAX_ELEMENT_LABEL_SIZE * sizeof(char));
  element_names_by_pin[RIMS_PIN]  = (char *) malloc(MAX_ELEMENT_LABEL_SIZE * sizeof(char));
  
  strcpy(element_names_by_pin[BOIL1_PIN], "BOIL1");
  strcpy(element_names_by_pin[BOIL2_PIN], "BOIL2");
  strcpy(element_names_by_pin[HLT1_PIN], "HLT1");
  strcpy(element_names_by_pin[HLT2_PIN], "HLT2");
  strcpy(element_names_by_pin[RIMS_PIN], "RIMS");
  
  // ready the rotary encoder pins
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);

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
  pixels.begin();
  
  begin_running(true);
}
 
// ---------------------------------------------------------------------------------- loop

void loop()
{ 
  /*
   * If in choosing mode, we need to check for timeout (the user must press the confirm button)
   * and if reached, restore the UI to running mode with no changes.  If timeout not yet reached,
   * the flashing action of the LEDs needs to be maintained.
   * 
   * If in running mode, no need to do anything special.
   */
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

  /*
   * Most of the code here came from the example at:
   *    https://learn.adafruit.com/pro-trinket-rotary-encoder/example-rotary-encoder-volume-control
   * 
   * The notable exceptions are that: 
   *    -- enc_action is consulted to see if the rotary encoder was turned
   *    -- the encoder push switch is consulted to see if a new setting was confirmed
   */
  
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
    // turned to the right
    if (proposed_element_configuration < 6) {
      proposed_element_configuration++;
      begin_choosing();
    }
  }
  else if (enc_action < 0) {
    // turned to the left
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

        /*
         * The user has confirmed an entry by pressing the encoder's knob.  If the confirmed value is different
         * then change the value and indicate the change in the UI.
         */
        
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
    neo_main_led_blue();
    set_indicator("?");
    neo_elements_proposed();
  } else {
    set_indicator(" ");
    neo_elements_off();
    neo_main_led_off();
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
  neo_elements_running();
}

void fire_relay(int element_offset) {

  static char pin_label[21];
  int pin = element_configuration_pins[actual_element_configuration][element_offset];
  
  lcd.setCursor(0,element_offset+2);
  lcd.print("Pin: ");
  
  if (pin > 0) {
    digitalWrite(pin, ELEMENT_ON);
    sprintf((char*)&pin_label, "%2d (%s)  ", pin, element_names_by_pin[pin]);
    lcd.print(pin_label);
  } else {
    lcd.print("None        "); 
  }
}

void all_off() {
  for (int i = 0; i < ELEMENT_COUNT; i++) {
    digitalWrite(element_pins[i], ELEMENT_OFF);
  }
  neo_elements_off();
}

void begin_running(bool is_changing) {
  
  ui_mode = UI_MODE_RUNNING;
  last_requested_proposed_configuration = -1;
  
  sync_proposed_configuration_lcd();
  set_indicator(" ");
  sync_actual_configration_to_relay_board();
  
  if (is_changing) {
    for (int i = 0; i < 5; i++) {
      main_led_running();
      delay(50);
      neo_main_led_off();
      delay(50);
    }
  }
  
  main_led_running();
}

void begin_choosing() {
  all_off();
  ui_last_choosing_indicator_toggled_at = 0;
  ui_mode_indicator = false;
  ui_mode = UI_MODE_CHOOSING;
  ui_last_choosing_mode_started_at = millis();
  sync_proposed_configuration_lcd();
}

void set_indicator(const char *value) {
  lcd.setCursor(19,0);
  lcd.print(value);
}

void neo_led_apply_deferred(int offset, uint32_t color) {
  if (offset >= 0) {
    pixels.setPixelColor(offset, color);
  }
}

void neo_led_apply_all_elements_deferred(uint32_t color) {
  for (int i = 0; i < ELEMENT_COUNT; i++) {
    neo_led_apply_deferred(element_leds[i], color);
  }
}

void neo_show() {
  pixels.show();
}

void neo_led_apply(int offset, uint32_t color) {
  neo_led_apply_deferred(offset, color);
  neo_show();
}

void main_led_running() {
  if (actual_element_configuration == 0) {
    neo_main_led_green();
  } else {
    neo_main_led_red();
  }
}

void neo_elements_off() {
  neo_led_apply_all_elements_deferred(neo_color_off);
  neo_show();
}

void neo_elements_running() {
  neo_elements_apply_configuration(actual_element_configuration, neo_color_green, neo_color_red, neo_color_green);
}

void neo_elements_proposed() {
  neo_elements_apply_configuration(proposed_element_configuration, neo_color_off, neo_color_red, neo_color_green);
}

void neo_elements_apply_configuration(int configuration, uint32_t off_color, uint32_t on_color, uint32_t config_zero_color) {
  
  neo_led_apply_all_elements_deferred(configuration == 0 ? config_zero_color : off_color);
  
  if (configuration > 0) {
    neo_led_apply_deferred(element_configuration_led_offsets[configuration][0], on_color);
    neo_led_apply_deferred(element_configuration_led_offsets[configuration][1], on_color);
  }
  
  neo_show();
}

void neo_main_led_apply(uint32_t color) {
  neo_led_apply(NEO_MAIN_LED_OFFSET, color);
}

void neo_main_led_blue() {
  neo_main_led_apply(neo_color_blue);
}

void neo_main_led_red() {
  neo_main_led_apply(neo_color_red);
}

void neo_main_led_green() {
  neo_main_led_apply(neo_color_green);
}

void neo_main_led_yellow() {
  neo_main_led_apply(neo_color_yellow);
}

void neo_main_led_off() {
  neo_main_led_apply(neo_color_off);
}
