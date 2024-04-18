/**
 * @file main.cpp
 * @author mathewos samson (matty b)
 * @brief main code for ks6e dash
 * @version 0.1
 * @date 2022-12-29
 *
 * @copyright Copyright (c) 2022
 *
 */
// lib includes
#include <Arduino.h>
#include <WS2812Serial.h>
#include "FlexCAN_T4.h"
#include <Wire.h>
#include <SPI.h>
#include <Metro.h>
#include <bitset>
#include "Adafruit_LEDBackpack.h"
// our includes
#include "FlexCAN_util.hpp"
#include <KS6eDashGPIO.hpp>
#include <neopixel_defs.hpp>
#include <MCU_status.hpp>
#include <inverter.hpp>
#include <device_status.hpp>

#define DEBUG false
// timers
Metro update_pixels_timer = Metro(10, 1);
Metro send_buttons_timer = Metro(100, 1);
Metro update_sevensegment_timer = Metro(100, 1);
Metro update_fault_leds = Metro(100, 1);
Metro send_dash_status_timer = Metro(1000, 1);
/**
 * @brief variables, objects, etc. for neopixel handling
 *
 */
const int numled = NUMBER_OF_PIXELS;
byte drawingMemory[numled * 3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled * 12]; // 12 bytes per LED
WS2812Serial leds(numled, displayMemory, drawingMemory, NEOPIXELDIN, WS2812_GRB);
Adafruit_7segment seven_segment = Adafruit_7segment();

// variables for SoC, VCU state, SDC error flags
MC_voltage_information mc_voltage_info;
MC_fault_codes mc_fault_codes;
MCU_status vcu_status;
MC_command_message mc_command_message;
int tempdisplay_;
int tempdisplayvoltage_;
uint8_t state_of_charge = 0;
uint8_t vcu_last_torque = 0;
uint16_t vcu_glv_sense = 0;
uint8_t lc_state;
uint8_t lc_type;
unsigned long vcu_lc_countdown;
unsigned long vcu_lc_delay;

bool dash_init();
bool display_enabled;
void gpio_init();
uint8_t getButtons();
void updateSOCNeopixels(int soc);
void updateStatusNeopixels(MCU_status MCU_status);
void test_socpixels();
void set_single_segment_indicator(uint8_t number_to_display);

static CAN_message_t fw_hash_msg;
device_status_t dash_status_t;
const uint8_t fault_led_duty = 0x05;
static uint8_t torque_mode_uint;
void setup()
{
  init_can();
  gpio_init();
  display_enabled = dash_init();
  vcu_status.set_state(MCU_STATE::STARTUP);
  state_of_charge = 0;

  fw_hash_msg.id = ID_DASH_FW_VERSION;
  fw_hash_msg.len = 8;
  dash_status_t.on_time_seconds = (millis() / 1000);
  memcpy(fw_hash_msg.buf, &dash_status_t, sizeof(dash_status_t));
  WriteCAN(fw_hash_msg);
#if DEBUG
  Serial.println("Startup complete");
#endif
}

void loop()
{
  update_can();

  if (update_pixels_timer.check())
  {
    updateSOCNeopixels(state_of_charge);
    updateStatusNeopixels(vcu_status);
    leds.show();
  }
  if (send_buttons_timer.check())
  {
    CAN_message_t button_status_msg;
    button_status_msg.id = ID_DASH_BUTTONS;
    button_status_msg.len = 1;
    button_status_msg.buf[0] = getButtons();
#if DEBUG
    Serial.println("Sent off buttons");
    Serial.printf("Time: %d\n", millis());
#endif
    WriteCAN(button_status_msg);
  }

  if (send_dash_status_timer.check())
  {
    digitalToggle(LED_BUILTIN);
    dash_status_t.on_time_seconds = (millis() / 1000);
    memcpy(fw_hash_msg.buf, &dash_status_t, sizeof(dash_status_t));
#if DEBUG
    Serial.printf("dash status hash: %x time: %d dirty: %d main: %d\n", dash_status_t.firmware_version, dash_status_t.on_time_seconds, dash_status_t.project_is_dirty, dash_status_t.project_on_main_or_master);
#endif
    WriteCAN(fw_hash_msg);
  }
  if (update_sevensegment_timer.check())
  {
    set_single_segment_indicator(vcu_status.get_torque_mode());
    if (display_enabled | seven_segment.begin())
    {
      seven_segment.begin();
      seven_segment.clear();
      if (tempdisplay_ >= 1)
      {
        seven_segment.print(vcu_status.get_max_torque(), DEC);
        tempdisplay_--; // this is so dumb
      }
      else if (tempdisplayvoltage_ >= 1)
      {
        float glv_v = static_cast<float>(vcu_glv_sense) * (3.3 / 1024) * 8.116578257423327;
        seven_segment.print(glv_v, DEC);
        tempdisplayvoltage_--;
      }
      else if (vcu_status.get_launch_ctrl_active())
      {
        seven_segment.writeDigitAscii(0, 'L');
        seven_segment.writeDigitAscii(1, 'C');
        seven_segment.writeDigitNum(3, lc_type);
      }
      else
      {
        seven_segment.print(mc_voltage_info.get_dc_bus_voltage(), DEC);
      }
      seven_segment.writeDisplay();
    }
  }
  if (update_fault_leds.check())
  {
    // NEED THE ! there so the leds work, pull low instead of high. confirmed working 3/28/23, in VCU false = light ON, true = light OFF
    analogWrite(AMS_LED, fault_led_duty * !(vcu_status.get_bms_ok_high()));

    analogWrite(BSPD_LED, fault_led_duty * !(vcu_status.get_bspd_ok_high()));

    analogWrite(IMD_LED, fault_led_duty * !(vcu_status.get_imd_ok_high()));
    // For the inverter fault, we just OR all the fault fields, since fault code > 0 == bad == turn light on
    analogWrite(INVERTER_LED, fault_led_duty * (mc_fault_codes.get_post_fault_hi() || mc_fault_codes.get_post_fault_lo() || mc_fault_codes.get_run_fault_hi() || mc_fault_codes.get_run_fault_lo()));

    analogWrite(MISCLED3, fault_led_duty * (vcu_status.get_accel_implausability()));
    analogWrite(MISCLED1, fault_led_duty * (vcu_status.get_accel_brake_implausability()));
    analogWrite(MISCLED2, fault_led_duty * (vcu_status.get_brake_implausibility()));
#if DEBUG
    Serial.printf("This is the BMS OK HIGH boolean: %d\n", vcu_status.get_bms_ok_high());
    Serial.printf("This is the BSPD OK HIGH boolean: %d\n", vcu_status.get_bspd_ok_high());
    Serial.printf("This is the IMD OK HIGH boolean: %d\n", vcu_status.get_imd_ok_high());
    Serial.printf("These are the Inverter Fault Codes: Post_fault_hi: %d Post_fault_lo: %d Run_fault_hi: %d Run_fault_lo: %d\n", mc_fault_codes.get_post_fault_hi(), mc_fault_codes.get_post_fault_lo(), mc_fault_codes.get_run_fault_hi(), mc_fault_codes.get_run_fault_lo());
    Serial.printf("Accel implaus: %d Accel&Brake Implaus: %d Brake Implaus: %d\n", vcu_status.get_accel_implausability(), vcu_status.get_accel_brake_implausability(), vcu_status.get_brake_implausibility());
#endif
  }
}

void colorWipe(int color, int wait_us)
{
  for (int i = 0; i < leds.numPixels(); i++)
  {
    leds.setPixel(i, color);
    leds.show();
    delayMicroseconds(wait_us);
  }
}
bool dash_init()
{
  leds.begin();
  leds.setBrightness(BRIGHTNESS);
  delay(10);
  int microsec = 200000 / leds.numPixels();
  colorWipe(RED, microsec);
  colorWipe(ORANGE, microsec);
  colorWipe(YELLOW, microsec);
  colorWipe(GREEN, microsec);
  colorWipe(BLUE, microsec);
  colorWipe(0x8F00FF, microsec);
  colorWipe(PINK, microsec);
  colorWipe(0xFFFFFF, microsec);
  colorWipe(GREEN, microsec);
  for (int i = 0; i < leds.numPixels(); i++)
  {
    leds.setPixel(i, BLACK);
  }
  leds.show();
  if (!seven_segment.begin())
  {
    return false;
#if DEBUG
    Serial.println("L dash");
#endif
  }
  seven_segment.setBrightness(15);
  seven_segment.print("yeet");
  seven_segment.writeDisplay();
  delay(500);
  vcu_status.set_pedal_states(0xff); // Set pedal states to fault state so lights are on if SNA
  return true;
};

/**
 * @brief
 *
 */
void gpio_init()
{
  // initial button pins as inputs
  for (int i = 0; i < 6; i++)
  {
    pinMode(dashButtons[i], INPUT_PULLUP);
  }
  // initialize LED driver pins as outputs
  for (int i = 0; i < 5; i++)
  {
    pinMode(seven_seg_gpios[i], OUTPUT);
  }
  for (int i = 0; i < 3; i++)
  {
    pinMode(misc_led_gpios[i], OUTPUT);
    analogWrite(misc_led_gpios[i], fault_led_duty);
    digitalWrite(misc_led_gpios[i], LOW);
  }
  for (int i = 0; i < 4; i++)
  {
    pinMode(fault_led_gpios[i], OUTPUT);
    analogWrite(fault_led_gpios[i], fault_led_duty);
    digitalWrite(fault_led_gpios[i], LOW);
    // init builtin led
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
  }
#if DEBUG
  Serial.println("GPIOs initialized");
#endif
}
/**
 * @brief Get the Buttons object
 *
 * @return uint8_t
 */
uint8_t getButtons()
{
  uint8_t buttonStatuses = 0;
  for (int i = 0; i < 6; i++)
  {
    // the ! is because buttons are active low
    buttonStatuses |= (!(digitalRead(dashButtons[i])) << i);
  }
#if DEBUG
  Serial.print("Button Stats: ");
  Serial.println(buttonStatuses, BIN);
#endif
  return buttonStatuses;
}
/**
 * @brief
 *
 * @param soc
 */
void updateSOCNeopixels(int soc)
{
  if (!vcu_status.get_launch_ctrl_active())
  {
#if DEBUG
    Serial.printf("State of charge value in neopixel update method: %d\n", soc);
#endif
    if (soc > 100)
    {
      soc = 100;
    }
    else if (soc < 0)
    {
      soc = 0;
    }
    float soc_f = soc;
    soc_f /= 100;
    int num_leds_enabled = PIXELS_FOR_SOC * soc_f;
    int num_leds_leftover = PIXELS_FOR_SOC - num_leds_enabled;
    for (int i = 0; i < num_leds_enabled; i++)
    {
      leds.setPixel(i, GREEN);
    }
    for (int i = (PIXELS_FOR_SOC - 1); i >= num_leds_enabled; i--)
    {
      leds.setPixel(i, 0x0a'00'00);
    }
    int soc_mod = soc % 10;
    if (num_leds_enabled < PIXELS_FOR_SOC && soc_mod > 0)
    {
      float soc_percent_mod = soc_mod;
      soc_percent_mod /= 10;
      uint8_t soc_green = 128 * soc_percent_mod;
      uint8_t soc_red = 255 - soc_green;
      // first byte is red, second is green, third is blue
      uint32_t soc_percentage_color = (soc_red << 16) | (soc_green << 8);
      leds.setPixel(num_leds_enabled, soc_percentage_color);
    }
#if DEBUG
    Serial.printf("Set %d to %d ON, set %d to %d OFF\n", 1, num_leds_enabled, num_leds_enabled + 1, PIXELS_FOR_SOC);
#endif
  }
  else
  {
    int color = 0;
    switch (lc_state)
    {
    case 0:
    {
      color = CYAN;
      break;
    }
    case 1:
    {
      color = YELLOW;
      break;
    }
    case 2:
    {

      color = DEEP_PINK;
      break;
    }
    case 3:
    {
      vcu_lc_countdown = 0;
      color = ORANGE;
      break;
    }
    }
    if (lc_state != 1)
    {
      for (int i = 0; i < (PIXELS_FOR_SOC); i++)
      {
        leds.setPixel(i, color);
      }
    }
    else
    {
      float vcu_lc_countdown_f = vcu_lc_countdown;
      vcu_lc_countdown_f /= vcu_lc_delay;
      vcu_lc_countdown_f = 1.0 - vcu_lc_countdown_f;
      int num_leds_enabled = PIXELS_FOR_SOC * vcu_lc_countdown_f;
      int num_leds_leftover = PIXELS_FOR_SOC - num_leds_enabled;
      for (int i = 0; i < num_leds_enabled; i++)
      {
        leds.setPixel(i, YELLOW);
      }
      for (int i = (PIXELS_FOR_SOC - 1); i >= num_leds_enabled; i--)
      {
        leds.setPixel(i, 0x0a'00'00);
      }
    }
  }
}
/**
 * @brief
 *
 * @param mcu_status
 */
void updateStatusNeopixels(MCU_status mcu_status)
{
  uint32_t status_color = BLACK;
  switch (mcu_status.get_state())
  {
  case MCU_STATE::STARTUP:
  {
    status_color = BLUE;
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
  {
    status_color = GREEN;
    break;
  }
  case MCU_STATE::TRACTIVE_SYSTEM_ACTIVE:
  {
    status_color = RED;
    break;
  }
  case MCU_STATE::ENABLING_INVERTER:
  {
    status_color = YELLOW;
    break;
  }
  case MCU_STATE::WAITING_READY_TO_DRIVE_SOUND:
  {
    status_color = ORANGE;
    break;
  }
  case MCU_STATE::READY_TO_DRIVE:
  {
    status_color = PINK;
    break;
  }
  }
  if (mcu_status.get_launch_ctrl_active())
  {
    leds.setPixel(16, WHITE);
  }
  else
  {
    leds.setPixel(16, BLACK);
  }
  for (int i = PIXELS_FOR_SOC; i < NUMBER_OF_PIXELS - 3; i++)
  {
    leds.setPixel(i, status_color);
  }
}
/**
 * @brief
 *
 */
void test_socpixels()
{
  Serial.println("Starting count up");
  for (int i = 0; i <= 100; i++)
  {
    updateSOCNeopixels(i);
    leds.show();
    Serial.println(i);
    delay(100);
  }
  Serial.println("Starting count down");
  for (int i = 100; i >= 0; i--)
  {
    updateSOCNeopixels(i);
    leds.show();
    Serial.println(i);
    delay(100);
  }
}

void set_single_segment_indicator(uint8_t number_to_display)
{
  digitalWrite(LATCH_1, LOW);
  delayMicroseconds(200);
  bool led_a_high = number_to_display & 0b0001;
  bool led_b_high = number_to_display & 0b0010;
  bool led_c_high = number_to_display & 0b0100;
  bool led_d_high = number_to_display & 0b1000;
#if DEBUG
  Serial.printf("SEVEN SEGMENT INPUT: %d A: %d B: %d C: %d D: %d\n", number_to_display, led_a_high, led_b_high, led_c_high, led_d_high);
#endif
  digitalWrite(LED_A, led_a_high);
  digitalWrite(LED_B, led_b_high);
  digitalWrite(LED_C, led_c_high);
  digitalWrite(LED_D, led_d_high);
  delayMicroseconds(200);
  // there may need to be a delay here, but dont want to block
  digitalWrite(LATCH_1, HIGH);
  delayMicroseconds(200);
  digitalWrite(LATCH_1, LOW);
}