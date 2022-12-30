/// @file    Blink.ino
/// @brief   Blink the first LED of an LED strip
/// @example Blink.ino

#include <FastLED.h>
#include <stdio.h>
// How many leds in your strip?
#define NUM_LEDS 10
#define PIXELS_FOR_SOC 10
// For led chips like WS2812, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
// Clock pin only needed for SPI based chipsets when not using hardware SPI
#define DATA_PIN 4
//#define CLOCK_PIN 13

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() { 
  Serial.begin(9600);
  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);  // GRB ordering is typical

}

void loop() { 
  // Turn the LED on, then pause
  if(Serial.available()){
//    delay(10);
//    Serial.println("paused");
//    while(!Serial.available()){
//    }
    String soc_string = Serial.readString();
    int soc = soc_string.toInt();
    updateSOCNeopixels(soc);
    Serial.println(soc);
  }
 for(int i=0; i<=100; i++){
     Serial.println("Starting count up");
   updateSOCNeopixels(i);
   Serial.println(i);
   delay(100);
 }
  Serial.println("Starting count down");
   for(int i=100; i>=0; i--){
    updateSOCNeopixels(i);
    Serial.println(i);
    delay(100);
  }
  // while(1){}
}

void updateSOCNeopixels(int soc){
  if(soc > 100){soc=100;}else if(soc<0){soc=0;}
  //Serial.print("SOC: ");
  //Serial.println(soc);
  float soc_f = soc;
  soc_f /= 100;
  //Serial.print("SOC Float: ");
  //Serial.println(soc_f);
  int num_leds_enabled = PIXELS_FOR_SOC * soc_f;
    //print("LEDS enabled: ");
    //Serial.println(num_leds_enabled);
  int num_leds_leftover = PIXELS_FOR_SOC-num_leds_enabled;
  for (int i=0;i<num_leds_enabled;i++){
    //Serial.print("Set This LED On: ");
    //Serial.println(i);
    leds[i]=CRGB::Green;
//    FastLED.show();
  }
  for (int i=(PIXELS_FOR_SOC-1); i>=num_leds_enabled;i--){
    //Serial.print("Set This LED Off: ");
    //Serial.println(i);
    leds[i]=CRGB::Black;
  }
  int soc_mod = (int)(soc % 10);
  // Serial.print("Soc Mod: ");
  // Serial.println(soc_mod);
  if((num_leds_enabled<PIXELS_FOR_SOC) && (soc_mod > 0)){
    float soc_percent_mod = soc_mod;
    soc_percent_mod /= 10;
    // Serial.println("we here");
    uint8_t soc_green = 128*soc_percent_mod;
    uint8_t soc_red = 255-soc_green;
    // Serial.println(soc_green);
    // Serial.println(soc_percent_mod);
    // Serial.println(soc_red);
    leds[num_leds_enabled]=CRGB(soc_red, soc_green, 0);
    //leds[num_leds_enabled]=CRGB::Red;   
  }
  FastLED.show();
}
