
#include "FlexCAN_util.hpp"
#include "KS6eCAN.hpp"



FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Inverter_CAN_;



void init_can()
{
    // inverter can must send & receive, 6rx MB and 2tx MB
    Inverter_CAN_.begin();
    Inverter_CAN_.setBaudRate(1000000);

    Inverter_CAN_.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);

    for (int i = 0; i < NUM_RX_MAILBOXES; i++)
    {

        Inverter_CAN_.setMB((FLEXCAN_MAILBOX)i, RX, STD);

    }

    for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++)
    {

        Inverter_CAN_.setMB((FLEXCAN_MAILBOX)i, TX, STD);

    }

    Inverter_CAN_.mailboxStatus();

}


int ReadCAN(CAN_message_t &msg)
{

    return Inverter_CAN_.read(msg);

}


int WriteCAN(CAN_message_t &msg)
{

    return Inverter_CAN_.write(msg);

}