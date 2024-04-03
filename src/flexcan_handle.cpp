#include "FlexCAN_util.hpp"
#include <MCU_status.hpp>
#include <inverter.hpp>
extern MC_voltage_information mc_voltage_info;
extern MC_fault_codes mc_fault_codes;
extern MCU_status vcu_status;
extern uint8_t state_of_charge;
extern uint8_t vcu_last_torque;
extern uint16_t vcu_glv_sense;
extern int tempdisplay_;
extern int tempdisplayvoltage_;
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Inverter_CAN_;
elapsedMillis voltageViewTimer = 0;
void init_can()
{
    // inverter can must send & receive, 6rx MB and 2tx MB
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

/**
 * @brief
 *
 */
void update_can()
{
    CAN_message_t rx_msg;
    if (ReadCAN(rx_msg))
    {
        switch (rx_msg.id)
        {
        case (ID_BMS_SOC):
        {
            //the BMS multiples the SOC value by two when it sends it, have to divide by two when received
            state_of_charge = rx_msg.buf[0] / 2;
            break;
        }
        case (ID_VCU_STATUS):
        {
            vcu_status.load(rx_msg.buf);
            if (vcu_last_torque != vcu_status.get_max_torque())
            {
                vcu_last_torque = vcu_status.get_max_torque();
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
        case (ID_VCU_BOARD_READINGS_ONE):
        {
            memcpy(&vcu_glv_sense,&rx_msg.buf[2],sizeof(vcu_glv_sense));
            Serial.printf("Loaded VCU voltage reading: %d",vcu_glv_sense);
            if (voltageViewTimer >= 5000 && vcu_status.get_state() == MCU_STATE::TRACTIVE_SYSTEM_NOT_ACTIVE)
            {
                voltageViewTimer=0;
                tempdisplayvoltage_ = 10;
            }
            break;
        }
        default:
            break;
        }
    }
}