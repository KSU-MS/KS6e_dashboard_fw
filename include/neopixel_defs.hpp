#ifndef NEOPIXEL_DEFS_HPP
#define NEOPIXEL_DEFS_HPP
#define NUMBER_OF_PIXELS 17 //THere are seventeen total neopixels on the board
#define PIXELS_FOR_SOC 11 //There are eleven neopixels on the top row intended for SOC
#define PIXELS_FOR_VCU_STATE 1
#define PIXELS_FOR_LC_ACTIVE 1 // this is dumb ash
const int LC_ACTIVE_PIXEL = 16;
const int TC_ACTIVE_PIXEL = 15;
bool ledsdirection=true; //true is up, false is down
// neo-pixel specific
// color values are 0-255 (8 bits)
// first byte is red, second is green, third is blue
#define CYAN 0x00FFFF
#define RED    0xFF0000
#define GREEN  0x00FF00
#define BLUE   0x0000FF
#define YELLOW 0xFFFF00
#define PINK   0x10EF0078
#define DEEP_PINK 0xFF1493
#define ORANGE 0xE05800
#define WHITE  0xFFFFFF
#define BLACK  0x0
#define BRIGHTNESS 32

#endif