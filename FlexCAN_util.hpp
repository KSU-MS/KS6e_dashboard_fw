

#ifndef FLEXCAN_UTIL_HPP
#define FLEXCAN_UTIL_HPP


#include <FlexCAN_T4.h>
#include "KS6eCAN.hpp"


// global wrapper around flexcan_t4 because it is a shit driver that should feel bad

void init_can();
int WriteCAN(CAN_message_t &msg);
int ReadCAN(CAN_message_t &msg);

//int ReadCAN(CAN_message_t &msg);


#endif