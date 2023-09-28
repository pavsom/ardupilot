#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_DRONECAN_ENABLED

#include <AP_DroneCAN/AP_DroneCAN.h>
#if AP_TEST_DRONECAN_DRIVERS
#include <SITL/SITL.h>
#endif

class AP_Baro_DroneCAN : public AP_Baro_Backend {
public:
    AP_Baro_DroneCAN(AP_Baro &baro);

    void update() override;

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);
    static AP_Baro_DroneCAN* get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, bool create_new);
    static AP_Baro_Backend* probe(AP_Baro &baro);

    static void handle_pressure(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_StaticPressure &msg);
    static void handle_temperature(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_StaticTemperature &msg);
#if AP_DRONECAN_SNOWSTORM_SUPPORT
    static void handle_pressure_short(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_snowstorm_Pressure &msg);
#endif    
#if AP_TEST_DRONECAN_DRIVERS
    void update_healthy_flag(uint8_t instance) override { _frontend.sensors[instance].healthy = !AP::sitl()->baro[instance].disable; };
#endif
private:

    static void _update_and_wrap_accumulator(float *accum, float val, uint8_t *count, const uint8_t max_count);

    uint8_t _instance;

    bool new_pressure;
    float _pressure;
    float _temperature;
    uint8_t  _pressure_count;
    HAL_Semaphore _sem_baro;

    AP_DroneCAN* _ap_dronecan;
    uint8_t _node_id;

    // Module Detection Registry
    static struct DetectedModules {
        AP_DroneCAN* ap_dronecan;
        uint8_t node_id;
        AP_Baro_DroneCAN* driver;
    } _detected_modules[BARO_MAX_DRIVERS];

#if AP_DRONECAN_SNOWSTORM_SUPPORT
    uint32_t lastMessage;
    // send text prefix string to reduce flash cost
    static const char* send_text_prefix;
    int8_t receiveRate = 0;
    int8_t detectedModulesInstance = -1;
    // DroneCAN parameter handling methods
    FUNCTOR_DECLARE(param_int_cb, bool, AP_DroneCAN*, const uint8_t, const char*, int32_t &);
    FUNCTOR_DECLARE(param_string_cb, bool, AP_DroneCAN*, const uint8_t, const char*, AP_DroneCAN::string &);
    FUNCTOR_DECLARE(param_float_cb, bool, AP_DroneCAN*, const uint8_t, const char*, float &);
    FUNCTOR_DECLARE(param_save_cb, void, AP_DroneCAN*, const uint8_t, bool);

    bool handle_param_get_set_response_int(AP_DroneCAN* ap_dronecan, const uint8_t node_id, const char* name, int32_t &value);
    bool handle_param_get_set_response_string(AP_DroneCAN* ap_dronecan, const uint8_t node_id, const char* name, AP_DroneCAN::string &value);
    bool handle_param_get_set_response_float(AP_DroneCAN* ap_dronecan, const uint8_t node_id, const char* name, float &value);
    void handle_param_save_response(AP_DroneCAN* ap_dronecan, const uint8_t node_id, bool success);

    // helper function to get and set parameters
    bool set_param_int32(const char* param_name, int32_t param_value);
    bool set_param_string(const char* param_name, const AP_DroneCAN::string& param_value);
    bool get_param_string(const char* param_name);
    uint32_t last_send_getset_param_ms;             // system time that a get or set parameter message was sent
#endif
    static HAL_Semaphore _sem_registry;
};

#endif  // AP_BARO_DRONECAN_ENABLED
