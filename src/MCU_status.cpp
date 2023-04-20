
#include "MCU_status.hpp"
#include "FlexCAN_util.hpp"





uint8_t *MCU_status::getBusVoltage()
{
    CAN_message_t VCU_msg;

    ReadCAN(VCU_msg);

    if (VCU_msg.id == ID_DASH_BUSVOLT)
    {
        memcpy(VCU_msg.buf, this->BusVolt_ByteEachDigit, VCU_msg.len);
        return this->BusVolt_ByteEachDigit;
    }
    else return 0;
}