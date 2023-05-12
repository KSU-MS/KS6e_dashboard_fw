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
//lib includes
#include <Arduino.h>
#include <WS2812Serial.h>
#include "FlexCAN_T4.h"
#include <Wire.h>
#include <SPI.h>
#include <Metro.h>
#include <bitset>
#include "Adafruit_LEDBackpack.h"
//our includes
#include "FlexCAN_util.hpp"
#include <KS6eDashGPIO.hpp>
#include <neopixel_defs.hpp>
#include <MCU_status.hpp>
#include <inverter.hpp>
 
#define DEBUG true
#define RELIABLE true // Define this TRUE if the code has been successfully tested & pushed to MAIN branch

//timers
Metro update_pixels_timer = Metro(100,1);
Metro send_buttons_timer = Metro(100,1);
Metro update_sevensegment_timer = Metro(100,1);
Metro update_fault_leds = Metro(100,1);
/**
 * @brief variables, objects, etc. for neopixel handling
 * 
 */
const int numled = NUMBER_OF_PIXELS;
byte drawingMemory[numled * 3];         //  3 bytes per LED
DMAMEM byte displayMemory[numled * 12]; // 12 bytes per LED
WS2812Serial leds(numled, displayMemory, drawingMemory, NEOPIXELDIN, WS2812_GRB);
Adafruit_7segment seven_segment = Adafruit_7segment();

//variables for SoC, VCU state, SDC error flags
MC_voltage_information mc_voltage_info;
MC_fault_codes mc_fault_codes;
MCU_status vcu_status;
int tempdisplay_;
uint8_t state_of_charge = 0;
uint8_t vcu_last_torque = 0;
uint8_t sdc_error_flags = 0; //TODO define bit arrangement for fault flags
void dash_init();
void gpio_init();
uint8_t getButtons();
void updateSOCNeopixels(int soc);
void updateStatusNeopixels(MCU_status MCU_status);
void test_socpixels();

void setup() {
  init_can();
  gpio_init(); 
  dash_init();
}

void loop() {
  update_can();

  if(update_pixels_timer.check()){
    updateSOCNeopixels(state_of_charge);
    updateStatusNeopixels(vcu_status);
    leds.show();
  }
  // if(send_buttons_timer.check()){
  //   uint8_t buf[]={getButtons(),0,0,0,0,0,0,0};
  //   load_can(ID_DASH_BUTTONS,false,buf);
  // }
    if(send_buttons_timer.check()){
    uint8_t buf[]={getButtons(),0,0,0,0,0,0,0}; // TODO reduce the size of this? since it returns 6 rn and that would be more than 8
    load_can(ID_DASH_BUTTONS,false,buf);
  }

  if(update_sevensegment_timer.check()){
    if(tempdisplay_>=1){
      seven_segment.begin();
      seven_segment.clear();
      seven_segment.print(vcu_status.get_max_torque(), DEC);
      seven_segment.writeDisplay();
      tempdisplay_--; //this is so dumb
    }else{
      seven_segment.begin();
      seven_segment.clear();
      seven_segment.print(mc_voltage_info.get_dc_bus_voltage(), DEC);
      seven_segment.writeDisplay();
    }
  }
  if(update_fault_leds.check()){
    digitalWrite(AMS_LED,!(vcu_status.get_bms_ok_high())); // NEED THE ! there so the leds work, pull low instead of high. confirmed working 3/28/23, in VCU false = light ON, true = light OFF
    Serial.printf("This is the BMS OK HIGH boolean: %d\n",vcu_status.get_bms_ok_high());

    digitalWrite(BSPD_LED,!(vcu_status.get_bspd_ok_high() && RELIABLE)); // May highjack this for now and make it the testing code light
    Serial.printf("This is the BSPD OK HIGH boolean: %d\n",vcu_status.get_bspd_ok_high());

    digitalWrite(IMD_LED,!(vcu_status.get_imd_ok_high()));
    Serial.printf("This is the IMD OK HIGH boolean: %d\n",vcu_status.get_imd_ok_high());

    digitalWrite(INVERTER_LED,(mc_fault_codes.get_post_fault_hi() || mc_fault_codes.get_post_fault_lo() || mc_fault_codes.get_run_fault_hi() || mc_fault_codes.get_run_fault_lo()));
    Serial.printf("This is the INVERTER OK HIGH boolean: %d\n",vcu_status.get_imd_ok_high());
  }


  //TODO remove for commissioning
  //test_socpixels();
}


void colorWipe(int color, int wait_us) {
  for (int i=0; i < leds.numPixels(); i++) {
    leds.setPixel(i, color);
    leds.show();
    delayMicroseconds(wait_us);
  }
}
void dash_init(){
  leds.begin();
  leds.setBrightness(BRIGHTNESS);
  delay(10);
  int microsec = 200000 / leds.numPixels();
    colorWipe(RED, microsec);
    colorWipe(ORANGE, microsec);
    colorWipe(YELLOW, microsec);
    colorWipe(GREEN, microsec);
    colorWipe(BLUE, microsec);
    colorWipe(0x8F00FF,microsec);
    colorWipe(PINK, microsec);
    colorWipe(0xFFFFFF, microsec);
    colorWipe(GREEN, microsec);
  for (int i = 0; i < leds.numPixels(); i++)
  {
    leds.setPixel(i, BLACK);
  }
  leds.show();
  if(!seven_segment.begin()){
    Serial.println("L dash");
  }
  seven_segment.setBrightness(15);
  seven_segment.print("COPE");
  seven_segment.writeDisplay();
};



/**
 * @brief 
 * 
 */
void gpio_init(){
  //initial button pins as inputs
  for (int i=0; i<6;i++){
    pinMode(dashButtons[i],INPUT_PULLUP);
  }
  //initialize LED driver pins as outputs
  for (int i=0; i<5;i++){
    pinMode(seven_seg_gpios[i],OUTPUT);
  }
  for (int i=0; i<3;i++){
    pinMode(misc_led_gpios[i],OUTPUT);
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
uint8_t getButtons(){
  uint8_t buttonStatuses=0;
  for (int i=0; i<6;i++){
    buttonStatuses |= (digitalRead(dashButtons[i]) << i);
  }
  #if DEBUG
  Serial.print("Button Stats: ");
  Serial.println(buttonStatuses,BIN);
  #endif
  return buttonStatuses;
}
/**
 * @brief 
 * 
 * @param soc 
 */
void updateSOCNeopixels(int soc){
  if(soc > 100){soc=100;}else if(soc<0){soc=0;}
  float soc_f = soc;
  soc_f /= 100;
  int num_leds_enabled = PIXELS_FOR_SOC * soc_f;
  int num_leds_leftover = PIXELS_FOR_SOC-num_leds_enabled;
  for (int i=0;i<num_leds_enabled;i++){
    leds.setPixel(i,GREEN);
  }
  for (int i=(PIXELS_FOR_SOC-1); i>=num_leds_enabled;i--){
    leds.setPixel(i,0x0f'00'00);
  }
  int soc_mod = soc % 10;
  if(num_leds_enabled<PIXELS_FOR_SOC && soc_mod > 0){
    float soc_percent_mod = soc_mod;
    soc_percent_mod /= 10;
    uint8_t soc_green = 128*soc_percent_mod;
    uint8_t soc_red = 255-soc_green;
    // first byte is red, second is green, third is blue
    uint32_t soc_percentage_color = (soc_red << 16) | (soc_green << 8);
    leds.setPixel(num_leds_enabled,soc_percentage_color);
  }
  // #if DEBUG
  // Serial.printf("Set %d to %d ON, set %d to %d OFF\n",1,num_leds_enabled,num_leds_enabled+1,PIXELS_FOR_SOC);
  // #endif
}
/**
 * @brief 
 * 
 * @param mcu_status 
 */
void updateStatusNeopixels(MCU_status mcu_status){
  uint32_t status_color=BLACK;
  switch (mcu_status.get_state()){
    case MCU_STATE::STARTUP:
    {
      status_color=WHITE;
      break;
    }
    case MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE:
    {
      status_color=GREEN;
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
      status_color=ORANGE;
      break;
    }
    case MCU_STATE::READY_TO_DRIVE:
    {
      status_color=PINK;
      break;
    }
  }
  for(int i=PIXELS_FOR_SOC;i<NUMBER_OF_PIXELS;i++){
    leds.setPixel(i,status_color);
  }
}
/**
 * @brief 
 * 
 */
void test_socpixels(){
  Serial.println("Starting count up");
  for(int i=0; i<=100; i++){
    updateSOCNeopixels(i);
    leds.show();
    Serial.println(i);
    delay(100);
 }
  Serial.println("Starting count down");
  for(int i=100; i>=0; i--){
    updateSOCNeopixels(i);
    leds.show();
    Serial.println(i);
    delay(100);
  }
}