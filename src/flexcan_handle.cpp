#include "FlexCAN_util.hpp"
#include <MCU_status.hpp>
#include <inverter.hpp>
extern MC_voltage_information mc_voltage_info;
extern MC_fault_codes mc_fault_codes;
extern MCU_status vcu_status;
extern uint8_t state_of_charge;
extern uint8_t vcu_last_torque;
extern int tempdisplay_;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Inverter_CAN_;


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
    Serial.println("hello"); //todo
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
            if(vcu_last_torque!= vcu_status.get_max_torque()){
                vcu_last_torque=vcu_status.get_max_torque();
                tempdisplay_ = 10;
            }
            break;
        }
        case (ID_MC_VOLTAGE_INFORMATION):
        {
            mc_voltage_info.load(rx_msg.buf);
            break;
        }
        case (ID_MC_FAULT_CODES):
        {
            mc_fault_codes.load(rx_msg.buf);
            break;
        }
        default:
            break;
        }

    }
}