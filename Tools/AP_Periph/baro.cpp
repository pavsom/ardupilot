#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_BARO

/*
  barometer support
 */

#include <dronecan_msgs.h>

/*
  update CAN baro
 */
void AP_Periph_FW::can_baro_update(void)
{
    if (!periph.g.baro_enable) {
        return;
    }
    uint32_t now = AP_HAL::millis();
    static uint32_t last_update_ms;
    if (g.baro_max_rate > 0 &&
        now - last_update_ms < uint32_t(1000/g.baro_max_rate)) {
        // limit to max rate
        return;
    }
    last_update_ms = now;
    baro.update();
    uint8_t baroInstances = baro.num_instances();
    for (uint8_t i = 0; i < baroInstances; i++){
        can_baro_send(i);
    }
}

void AP_Periph_FW::can_baro_send(uint8_t id)
{
    if (baro_last_sample_ms[id] == baro.get_last_update(id)) {
        return;
    }

    baro_last_sample_ms[id] = baro.get_last_update(id);
    if (!baro.healthy(id)) {
        // don't send any data
        return;
    }
    const float press = baro.get_pressure(id);
    const float temp = baro.get_temperature(id);

    {
        uavcan_equipment_air_data_StaticPressure pkt {};
        pkt.static_pressure = press;
        pkt.static_pressure_variance = 0; // should we make this a parameter?

        uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_MAX_SIZE] {};
        uint16_t total_size = uavcan_equipment_air_data_StaticPressure_encode(&pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_SIGNATURE,
                        UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }

    {
        uavcan_equipment_air_data_StaticTemperature pkt {};
        pkt.static_temperature = C_TO_KELVIN(temp);
        pkt.static_temperature_variance = 0; // should we make this a parameter?

        uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_MAX_SIZE] {};
        uint16_t total_size = uavcan_equipment_air_data_StaticTemperature_encode(&pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_SIGNATURE,
                        UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
}

#endif // HAL_PERIPH_ENABLE_BARO
