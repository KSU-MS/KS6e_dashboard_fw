#ifndef FLEXCAN_IS_SHIT_HPP
#define FLEXCAN_IS_SHIT_HPP
#include "FlexCAN_T4.h"
#include "KS6eCAN.hpp"

// global wrapper around flexcan_t4 because it is a shit driver that should feel bad
int WriteCAN(CAN_message_t &msg);
int ReadCAN(CAN_message_t &msg);
int load_can(uint32_t id, bool extended, uint8_t buf[8]);
void init_can();

#endif