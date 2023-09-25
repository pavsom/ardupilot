#include "AP_Periph.h"

#ifdef AP_INERTIALSENSOR_ENABLED

#include <dronecan_msgs.h>

void AP_Periph_FW::can_imu_update(void)
{
    if (!periph.g.imu_enable) {
        return;
    }
    uint32_t now = AP_HAL::millis();
    static uint32_t last_update_ms;
    if ((g.imu_max_rate > 0 &&
        now - last_update_ms < uint32_t(1000/g.imu_max_rate)) || !g.imu_max_rate) {
        // limit to max rate
        return;
    }
    last_update_ms = now;
    imu.update();

    uint8_t imuInstances = INS_MAX_INSTANCES;
    for (uint8_t i = 0; i < imuInstances; i++){
        can_imu_send();
    }
}

void AP_Periph_FW::can_imu_send()
{
    if (!can_imu_data_good()) return;
    const Vector3f accel = imu.get_accel();

    {
        uavcan_equipment_camera_gimbal_Status pkt {};
        pkt.gimbal_id = 0;
        pkt.mode.command_mode = 2;
        pkt.camera_orientation_in_body_frame_xyzw[0] = accel.x;
        pkt.camera_orientation_in_body_frame_xyzw[1] = accel.y;
        pkt.camera_orientation_in_body_frame_xyzw[2] = accel.z;
        pkt.camera_orientation_in_body_frame_xyzw[3] = 0;

        uint8_t buffer[UAVCAN_EQUIPMENT_CAMERA_GIMBAL_STATUS_MAX_SIZE] {};
        uint16_t total_size = uavcan_equipment_camera_gimbal_Status_encode(&pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_CAMERA_GIMBAL_STATUS_SIGNATURE,
                        UAVCAN_EQUIPMENT_CAMERA_GIMBAL_STATUS_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
}

bool AP_Periph_FW::can_imu_data_good(){
    if (imu_last_sample_usec == imu.get_last_update_usec()) {
        return false;
    }
    imu_last_sample_usec = imu.get_last_update_usec();
    if (!imu.healthy()) {
        // don't send any data
        return false;
    }
    return true;

}

#endif