#include "FlexCAN_util.hpp"
#include <MCU_status.hpp>
extern MCU_status vcu_status;
extern uint8_t state_of_charge;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Inverter_CAN_;


void init_can(){
//inverter can must send & receive, 6rx MB and 2tx MB
Inverter_CAN_.begin();
Inverter_CAN_.setBaudRate(500000);
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
Inverter_CAN_.write(msg);
return Inverter_CAN_.write(msg);
}
/**
 * @brief 
 * 
 * @param id 
 * @param extended 
 * @param buf 
 * @return int 
 */
int load_can(uint32_t id, bool extended, uint8_t buf[]){
    CAN_message_t tx_msg;
    tx_msg.id = id;
    tx_msg.flags.extended = extended;
    memcpy(&tx_msg.buf[0],&buf,sizeof(buf));
    return WriteCAN(tx_msg);
}
/**
 * @brief 
 * 
 */
void update_can(){
    CAN_message_t rx_msg;
    if(ReadCAN(rx_msg)){
        switch (rx_msg.id)
        {
        case (ID_BMS_SOC):
        {
            state_of_charge = rx_msg.buf[0];
            break;
        }
        case (ID_VCU_STATUS):
        {
            vcu_status.load(rx_msg.buf);
            break;
        }
        default:
            break;
        }

    }
}