

#include <Arduino.h>
#include "FlexCAN_util.hpp"





int main(void)
{


    CAN_message_t msg;


    msg.id = 0x100;
    msg.len = 8;
    // msg.flags.extended = 0;
    // msg.flags.remote   = 0;
    // msg.flags.overrun  = 0;
    // msg.flags.reserved = 0;
    msg.buf[0] = 10;
    msg.buf[1] = 20;
    msg.buf[2] = 0;
    msg.buf[3] = 100;
    msg.buf[4] = 128;
    msg.buf[5] = 64;
    msg.buf[6] = 32;
    msg.buf[7] = 16;



    pinMode(25, INPUT);

    init_can();



    while(true)
    {

        msg.buf[0]++;

        WriteCAN(msg);


       
        delay(20);

    }

}