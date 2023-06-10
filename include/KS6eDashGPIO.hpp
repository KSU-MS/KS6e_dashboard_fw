#ifndef KS6EDASHGPIOS_HPP
#define KS6EDASHGPIOS_HPP
// GPIO defines
//indicator lights
#define INVERTER_LED 0
#define BSPD_LED 2
#define AMS_LED 1
#define IMD_LED 3
const int fault_led_gpios[]={INVERTER_LED,BSPD_LED,AMS_LED,IMD_LED};
//for the single digit seven segment
#define LED_A 4
#define LED_B 5
#define LED_C 6
#define LED_D 6
#define LATCH_1 9
const int seven_seg_gpios[]={LED_A,LED_B,LED_C,LED_D,LATCH_1};

//neopixel
#define NEOPIXELDIN 8
//misc
#define MISCLED3 10
#define MISCLED2 12
#define MISCLED1 11
const int misc_led_gpios[]={MISCLED1,MISCLED2,MISCLED3};
//buttons
#define BUTTON1 21
#define BUTTON2 20
#define BUTTON3 17
#define BUTTON4 16
#define BUTTON5 15
#define BUTTON6 14
const int dashButtons[6] = {BUTTON1, BUTTON2, BUTTON3, BUTTON4, BUTTON5, BUTTON6};

#endif