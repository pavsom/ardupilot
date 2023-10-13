#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_CAN2CAN

    #include <dronecan_msgs.h>
    #define IFACE_ALL ((1U<<(HAL_NUM_CAN_IFACES))-1U)

    void AP_Periph_FW::can_send_ServoStatus(void)
    {
    // send a separate status packet for each servo
        for (uint8_t i = 0; i < NUM_SERVOS; i++) {
            if (!servos[i].pending) continue;
            servos[i].pending = false;
            struct uavcan_equipment_actuator_Status pkt;
            memset(&pkt, 0, sizeof(pkt));
            uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];

            // make up some synthetic status data
            pkt.actuator_id = i + (NUM_MOTORS + 1);
            pkt.position = SRV_Channels::get_output_scaled(servos[i].function);
            pkt.force = 3.5 * SRV_Channels::get_output_scaled(servos[i].function);
            pkt.speed = 0; // m/s or rad/s
            pkt.power_rating_pct = i;

            uint32_t len = uavcan_equipment_actuator_Status_encode(&pkt, buffer, !canfdout());

            // we need a static variable for the transfer ID. This is
            // incremeneted on each transfer, allowing for detection of packet
            // loss

            static uint8_t transfer_id;  // Note that the transfer ID variable MUST BE STATIC (or heap-allocated)!

            canardBroadcast(&dronecan.canard,
                            UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
                            UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
                            &transfer_id,
                            CANARD_TRANSFER_PRIORITY_LOW,
                            buffer,
                            len
#if CANARD_MULTI_IFACE
                            ,IFACE_ALL
#endif
            );
        }
    }

    void AP_Periph_FW::handle_ArrayCommand(CanardInstance* canard_instance, CanardRxTransfer* transfer)
    {
        struct uavcan_equipment_actuator_ArrayCommand cmd;
        if (uavcan_equipment_actuator_ArrayCommand_decode(transfer, &cmd)) {
            return;
        }
        uint64_t tnow = AP_HAL::micros64();
        for (uint8_t i=0; i < cmd.commands.len; i++) {
            if ((cmd.commands.data[i].actuator_id <= NUM_MOTORS) || 
            (cmd.commands.data[i].actuator_id > (NUM_MOTORS + NUM_SERVOS))) {
                // not for us
                continue;
            }
            uint8_t id = cmd.commands.data[i].actuator_id - (NUM_MOTORS + 1);
            switch (cmd.commands.data[i].command_type) {
            case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS:
                SRV_Channels::set_output_scaled(servos[id].function, cmd.commands.data[i].command_value);
                break;
            case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM:
                // map PWM to -1 to 1, assuming 1500 trim. If the servo has natural PWM
                // support then we should use it directly instead
                SRV_Channels::set_output_scaled(servos[id].function, cmd.commands.data[i].command_value);
                break;
            }
            servos[id].last_update_us = tnow;
            servos[id].pending = true;
        }
        can_send_ServoStatus();
    }

#endif