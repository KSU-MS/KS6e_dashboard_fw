
#include "MCU_status.hpp"
#include "FlexCAN_util.hpp"


void MCU_status::updateBusVoltage()
{
    CAN_message_t VCU_msg;

    ReadCAN(VCU_msg);

    if (VCU_msg.id == ID_DASH_BUSVOLT)
    {
        memcpy(this->BusVolt_ByteEachDigit, VCU_msg.buf, VCU_msg.len);
    }
}


uint8_t *MCU_status::getBusVoltage()
{
    return this->BusVolt_ByteEachDigit;
}

