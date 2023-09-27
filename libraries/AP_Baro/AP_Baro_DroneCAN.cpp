#include "AP_Baro_DroneCAN.h"

#if AP_BARO_DRONECAN_ENABLED

#include <AP_CANManager/AP_CANManager.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_Baro_SITL.h"
#include <AP_Vehicle/AP_Vehicle_Type.h>

#if AP_DRONECAN_SNOWSTORM_SUPPORT   
#include <GCS_MAVLink/GCS.h>
#endif
extern const AP_HAL::HAL& hal;

#define LOG_TAG "Baro"

AP_Baro_DroneCAN::DetectedModules AP_Baro_DroneCAN::_detected_modules[];
HAL_Semaphore AP_Baro_DroneCAN::_sem_registry;

/*
  constructor - registers instance at top Baro driver
 */
AP_Baro_DroneCAN::AP_Baro_DroneCAN(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
#if AP_DRONECAN_SNOWSTORM_SUPPORT   
    param_int_cb = FUNCTOR_BIND_MEMBER(&AP_Baro_DroneCAN::handle_param_get_set_response_int, bool, AP_DroneCAN*, const uint8_t, const char*, int32_t &);
    param_float_cb = FUNCTOR_BIND_MEMBER(&AP_Baro_DroneCAN::handle_param_get_set_response_float, bool, AP_DroneCAN*, const uint8_t, const char*, float &);
    param_string_cb = FUNCTOR_BIND_MEMBER(&AP_Baro_DroneCAN::handle_param_get_set_response_string, bool, AP_DroneCAN*, const uint8_t, const char*, AP_DroneCAN::string &);
    param_save_cb = FUNCTOR_BIND_MEMBER(&AP_Baro_DroneCAN::handle_param_save_response, void, AP_DroneCAN*, const uint8_t, bool);
#endif
}

void AP_Baro_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }
    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_pressure, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("pressure_sub");
    }
#if AP_DRONECAN_SNOWSTORM_SUPPORT    
    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_pressure_short, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("pressure_short_sub");
    }
#endif    
    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_temperature, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("temperature_sub");
    }
}

AP_Baro_Backend* AP_Baro_DroneCAN::probe(AP_Baro &baro)
{
    WITH_SEMAPHORE(_sem_registry);

    AP_Baro_DroneCAN* backend = nullptr;
    for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
        if (_detected_modules[i].driver == nullptr && _detected_modules[i].ap_dronecan != nullptr) {
            backend = new AP_Baro_DroneCAN(baro);
            if (backend == nullptr) {
                AP::can().log_text(AP_CANManager::LOG_ERROR,
                            LOG_TAG,
                            "Failed register DroneCAN Baro Node %d on Bus %d\n",
                            _detected_modules[i].node_id,
                            _detected_modules[i].ap_dronecan->get_driver_index());
            } else {
                _detected_modules[i].driver = backend;
                backend->_pressure = 0;
                backend->_pressure_count = 0;
                backend->_ap_dronecan = _detected_modules[i].ap_dronecan;
                backend->_node_id = _detected_modules[i].node_id;

                backend->_instance = backend->_frontend.register_sensor();
                backend->set_bus_id(backend->_instance, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                                                    _detected_modules[i].ap_dronecan->get_driver_index(),
                                                                                    backend->_node_id, 0));

#if AP_DRONECAN_SNOWSTORM_SUPPORT                                                                                    
                printf("Registered DroneCAN Baro Node %d on Bus %d\n\r",
                            _detected_modules[i].node_id,
                            _detected_modules[i].ap_dronecan->get_driver_index());
#endif
                AP::can().log_text(AP_CANManager::LOG_INFO,
                            LOG_TAG,
                            "Registered DroneCAN Baro Node %d on Bus %d\n",
                            _detected_modules[i].node_id,
                            _detected_modules[i].ap_dronecan->get_driver_index());
            }
            break;
        }
    }
    return backend;
}

AP_Baro_DroneCAN* AP_Baro_DroneCAN::get_dronecan_backend(AP_DroneCAN* ap_dronecan, uint8_t node_id, bool create_new)
{
    if (ap_dronecan == nullptr) {
        return nullptr;
    }
    for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_dronecan == ap_dronecan && 
            _detected_modules[i].node_id == node_id) {
            return _detected_modules[i].driver;
        }
    }
    
    if (create_new) {
        bool already_detected = false;
        //Check if there's an empty spot for possible registeration
        for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
            if (_detected_modules[i].ap_dronecan == ap_dronecan && _detected_modules[i].node_id == node_id) {
                //Already Detected
                already_detected = true;
                break;
            }
        }
        if (!already_detected) {
            uint8_t i1 = 0;
            for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
                if (_detected_modules[i].ap_dronecan == nullptr) {
                    _detected_modules[i].ap_dronecan = ap_dronecan;
                    _detected_modules[i].node_id = node_id;
                    i1 = i;
                    break;
                }
            }
            printf("module %d node %d ap_dronecan 0x%08x created\n\r",i1,_detected_modules[i1].node_id,_detected_modules[i1].ap_dronecan);
        }
    }

    return nullptr;
}


void AP_Baro_DroneCAN::_update_and_wrap_accumulator(float *accum, float val, uint8_t *count, const uint8_t max_count)
{
    *accum += val;
    *count += 1;
    if (*count == max_count) {
        *count = max_count / 2;
        *accum = *accum / 2;
    }
}

void AP_Baro_DroneCAN::handle_pressure(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_StaticPressure &msg)
{
    AP_Baro_DroneCAN* driver;
    {
        WITH_SEMAPHORE(_sem_registry);
        driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id, true);
        if (driver == nullptr) {
            return;
        }
    }
    {
        WITH_SEMAPHORE(driver->_sem_baro);
        _update_and_wrap_accumulator(&driver->_pressure, msg.static_pressure, &driver->_pressure_count, 32);
        driver->new_pressure = true;
    }
}
#if AP_DRONECAN_SNOWSTORM_SUPPORT
const char* AP_Baro_DroneCAN::send_text_prefix = "BAROCAN:";
void AP_Baro_DroneCAN::handle_pressure_short(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const com_snowstorm_Pressure &msg)
{
    AP_Baro_DroneCAN* driver;
    {
        WITH_SEMAPHORE(_sem_registry);
        driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id, true);
        if (driver == nullptr) {
            return;
        }
    }
    {
        WITH_SEMAPHORE(driver->_sem_baro);
        _update_and_wrap_accumulator(&driver->_pressure, msg.pressures.data[0].pressure, &driver->_pressure_count, 32);
        driver->new_pressure = true;
        driver->_frontend.sensors[driver->_instance].type = static_cast<AP_Baro::baro_type_t>(msg.pressures.data[0].baro_type);
    }
}

// handle param get/set response
bool AP_Baro_DroneCAN::handle_param_get_set_response_int(AP_DroneCAN* ap_dronecan, uint8_t node_id, const char* name, int32_t &value)
{
    if (strcmp(name, "BARO_MAX_RATE") == 0) {
        receiveRate = value;
        return false;
    }
    // unhandled parameter get or set
    gcs().send_text(MAV_SEVERITY_INFO, "%s get/set %s res:%ld", send_text_prefix, name, (long int)value);
    return false;
}

// handle param get/set response
bool AP_Baro_DroneCAN::handle_param_get_set_response_string(AP_DroneCAN* ap_dronecan, uint8_t node_id, const char* name, AP_DroneCAN::string &value)
{
    if (strcmp(name, "test1") == 0) {
        return false;
    }

    // unhandled parameter get or set
    gcs().send_text(MAV_SEVERITY_INFO, "%s get/set string %s res:%s", send_text_prefix, name, (const char*)value.data);
    return false;
}
bool AP_Baro_DroneCAN::handle_param_get_set_response_float(AP_DroneCAN* ap_dronecan, uint8_t node_id, const char* name, float &value)
{
    gcs().send_text(MAV_SEVERITY_INFO, "%s get/set %s res:%ld", send_text_prefix, name, (long int)value);
    return false;
}
void AP_Baro_DroneCAN::handle_param_save_response(AP_DroneCAN* ap_dronecan, const uint8_t node_id, bool success)
{
    // display failure to save parameter
    
}

// helper function to set integer parameters
bool AP_Baro_DroneCAN::set_param_int32(const char* param_name, int32_t param_value)
{
    //printf("ap_droncean%d 0x%08x\n\r",detectedModulesInstance,_detected_modules[detectedModulesInstance].ap_dronecan);
    if (_detected_modules[detectedModulesInstance].ap_dronecan == nullptr) {
        return false;
    }

    if (_detected_modules[detectedModulesInstance].ap_dronecan->set_parameter_on_node(_detected_modules[detectedModulesInstance].node_id, param_name, param_value, &param_int_cb)) {
        last_send_getset_param_ms = AP_HAL::millis();
        return true;
    }
    return false;
}

bool AP_Baro_DroneCAN::set_param_string(const char* param_name, const AP_DroneCAN::string& param_value)
{
    if (_detected_modules[detectedModulesInstance].ap_dronecan == nullptr) {
        return false;
    }

    if (_detected_modules[detectedModulesInstance].ap_dronecan->set_parameter_on_node(_detected_modules[detectedModulesInstance].node_id, param_name, param_value, &param_string_cb)) {
        last_send_getset_param_ms = AP_HAL::millis();
        return true;
    }
    return false;
}

// helper function to get string parameters
bool AP_Baro_DroneCAN::get_param_string(const char* param_name)
{
    if (_detected_modules[detectedModulesInstance].ap_dronecan == nullptr) {
        return false;
    }

    if (_detected_modules[detectedModulesInstance].ap_dronecan->get_parameter_on_node(_detected_modules[detectedModulesInstance].node_id, param_name, &param_string_cb)) {
        last_send_getset_param_ms = AP_HAL::millis();
        return true;
    }
    return false;
}
#endif
void AP_Baro_DroneCAN::handle_temperature(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_air_data_StaticTemperature &msg)
{
    AP_Baro_DroneCAN* driver;
    {
        WITH_SEMAPHORE(_sem_registry);
        driver = get_dronecan_backend(ap_dronecan, transfer.source_node_id, false);
        if (driver == nullptr) {
            return;
        }
    }
    {
        WITH_SEMAPHORE(driver->_sem_baro);
        driver->_temperature = KELVIN_TO_C(msg.static_temperature);
    }
}

// Read the sensor
void AP_Baro_DroneCAN::update(void)
{
    float pressure = 0;
#if AP_DRONECAN_SNOWSTORM_SUPPORT
    if (detectedModulesInstance < 0){
        for (uint8_t i = 0; i < BARO_MAX_DRIVERS; i++) {
            if (_node_id == _detected_modules[i].node_id){
                detectedModulesInstance = i;
                if (!_frontend.sensors[_instance].calibrated)_frontend.sensors[_instance].calibrated = true;
            }
        }
    }
    uint32_t now = AP_HAL::millis();
    if (now - last_send_getset_param_ms > 1000 &&
        now - _frontend.sensors[_instance].last_update_ms < 1000){
        if (_frontend.get_primary() == _instance){
            if (receiveRate != 20){
                set_param_int32("BARO_MAX_RATE",20);
                if (now - lastMessage > 1000){
                    lastMessage = now;
                    gcs().send_text(MAV_SEVERITY_INFO, "%s%d set rate %d ", send_text_prefix, _instance, 20);
                }
            }
        }else{
            if (receiveRate != 2){
                set_param_int32("BARO_MAX_RATE",2);
                if (now - lastMessage > 1000){
                    lastMessage = now;
                    gcs().send_text(MAV_SEVERITY_INFO, "%s%d set rate %d ", send_text_prefix, _instance, 2);
                }
            }
        }
    }
#endif
    WITH_SEMAPHORE(_sem_baro);
    if (new_pressure) {
        if (_pressure_count != 0) {
            pressure = _pressure / _pressure_count;
            _pressure_count = 0;
            _pressure = 0;
        }
        _copy_to_frontend(_instance, pressure, _temperature);


        _frontend.set_external_temperature(_temperature);
        new_pressure = false;
    }
}

#endif // AP_BARO_DRONECAN_ENABLED
