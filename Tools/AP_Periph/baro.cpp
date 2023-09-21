#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_BARO

#ifndef AP_DRONECAN_SNOWSTORM_SUPPORT
#define AP_DRONECAN_SNOWSTORM_SUPPORT 0
#endif

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

#if !AP_DRONECAN_SNOWSTORM_SUPPORT
    uint8_t baroInstances = baro.num_instances();
    for (uint8_t i = 0; i < baroInstances; i++){
        can_baro_send(i);
    }
#else
    static uint8_t sendPressureCounter = 0;
    if ((sendPressureCounter++)%120){
        can_baro_send_altitudes();
    }else{
        can_baro_send_pressures();
    }
    
#endif 
}

#if AP_DRONECAN_SNOWSTORM_SUPPORT
void AP_Periph_FW::can_baro_send_pressures(){
    uint8_t dataLength = 0;
    com_snowstorm_sensors_Pressure msg {};
    uint8_t baroInstances = baro.num_instances();
    for (uint8_t i = 0; i < baroInstances; i++){
        if (!can_baro_data_good(i)) continue;
        msg.pressures.data[dataLength].sensor_id = i;
        msg.pressures.data[dataLength].current_pressure = static_cast<uint32_t>(baro.get_pressure(i));
        msg.pressures.data[dataLength].current_pressure = static_cast<uint32_t>(baro.get_ground_pressure(i));
        dataLength++;
    }
    if (!dataLength) return;
    msg.pressures.len = dataLength;
    {
        uint8_t buffer[COM_SNOWSTORM_SENSORS_PRESSURE_MAX_SIZE] {};
        uint16_t total_size = com_snowstorm_sensors_Pressure_encode(&msg, buffer, !periph.canfdout());
        canard_broadcast(COM_SNOWSTORM_SENSORS_PRESSURE_SIGNATURE,
                            COM_SNOWSTORM_SENSORS_PRESSURE_ID,
                            CANARD_TRANSFER_PRIORITY_LOW,
                            &buffer[0],
                            total_size);
    }
}
void AP_Periph_FW::can_baro_send_altitudes(){
    uint8_t dataLength = 0;
    com_snowstorm_sensors_Altitude msg {};
    uint8_t baroInstances = baro.num_instances();
    for (uint8_t i = 0; i < baroInstances; i++){
        if (!can_baro_data_good(i)) continue;
        msg.altitudes.data[dataLength].sensor_id = i;
        msg.altitudes.data[dataLength].altitude = static_cast<uint32_t>(baro.get_altitude(i));
        dataLength++;
    }
    if (!dataLength) return;
    msg.altitudes.len = dataLength;
    {
        uint8_t buffer[COM_SNOWSTORM_SENSORS_ALTITUDE_MAX_SIZE] {};
        uint16_t total_size = com_snowstorm_sensors_Altitude_encode(&msg, buffer, !periph.canfdout());
        canard_broadcast(COM_SNOWSTORM_SENSORS_ALTITUDE_SIGNATURE,
                            COM_SNOWSTORM_SENSORS_ALTITUDE_ID,
                            CANARD_TRANSFER_PRIORITY_LOW,
                            &buffer[0],
                            total_size);
    }
}
#endif

void AP_Periph_FW::can_baro_send(uint8_t id)
{
    if (!can_baro_data_good(id)) return;
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


bool AP_Periph_FW::can_baro_data_good(uint8_t id){
    if (baro_last_sample_ms[id] == baro.get_last_update(id)) {
        return false;
    }
    baro_last_sample_ms[id] = baro.get_last_update(id);
    if (!baro.healthy(id)) {
        // don't send any data
        return false;
    }
    return true;

}

#endif // HAL_PERIPH_ENABLE_BARO
