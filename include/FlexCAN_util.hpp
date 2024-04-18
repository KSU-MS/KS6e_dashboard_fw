#ifndef FLEXCAN_IS_SHIT_HPP
#define FLEXCAN_IS_SHIT_HPP
#include "FlexCAN_T4.h"
#include "KS6eCAN.hpp"
#include <ksu_ev_can.h>
// global wrapper around flexcan_t4 because it is a shit driver that should feel bad
int WriteCAN(CAN_message_t &msg);
int ReadCAN(CAN_message_t &msg);
void init_can();
void update_can();
int unpack_flexcan_message(can_obj_ksu_ev_can_h_t *o, CAN_message_t &msg);
#endif