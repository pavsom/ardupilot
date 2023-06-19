#include "Sub.h"

// return barometric altitude in centimeters
void Sub::read_barometer()
{
    uint32_t tnow = AP_HAL::millis();
    barometer.update();
    // If we are reading a positive altitude, the sensor needs calibration
    // Even a few meters above the water we should have no significant depth reading
    if (!rangefinder_alt_ok()){
        if (!rangefinder_state.enabled || !depthTerrain){
            depthTerrain = !g.depth_bottom? g.depth_max : MAX(g.depth_bottom, g.depth_max);
        }
    }else{
        float rangeFinderDistance = rangefinder_state.alt_cm_filt.get();
        depthTerrain = barometer.get_altitude()*100.0f - rangeFinderDistance; 
    }
    if(barometer.get_altitude() > 0 &&
    (tnow -  barometerCalibrationTime) > 1000) {
        barometerCalibrationTime = tnow;
        barometer.update_calibration();
        gcs().send_text(MAV_SEVERITY_INFO, "alt =%f, baroOF =%f, eas2 =%f, exTmp =%f, filt =%d, gP  =%f, gT =%f, time =%ld, prs =%f, prCo =%f, prSea =%f, tmp =%f",
        barometer.get_altitude(),
        barometer.get_baro_drift_offset(),
        barometer.get_EAS2TAS(),
        barometer.get_external_temperature(),
        barometer.get_filter_range(),
        barometer.get_ground_pressure(),
        barometer.get_ground_temperature(),
        barometer.get_last_update(),
        barometer.get_pressure(),
        barometer.get_pressure_correction(),
        barometer.get_sealevel_pressure(barometer.get_pressure()),
        barometer.get_temperature());
        
    }

    if (ap.depth_sensor_present) {
        sensor_health.depth = barometer.healthy(depth_sensor_idx);
    }
}

void Sub::init_rangefinder()
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.init(ROTATION_PITCH_270);
    rangefinder_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
    rangefinder_state.enabled = rangefinder.has_orientation(ROTATION_PITCH_270);
#endif
}

// return rangefinder altitude in centimeters
void Sub::read_rangefinder()
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

    rangefinder_state.alt_healthy = ((rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good) && (rangefinder.range_valid_count_orient(ROTATION_PITCH_270) >= RANGEFINDER_HEALTH_MAX));

    int16_t temp_alt = rangefinder.distance_cm_orient(ROTATION_PITCH_270);
    printf("rngRAW = %d ",temp_alt);
#if RANGEFINDER_TILT_CORRECTION == ENABLED
    // correct alt for angle of the rangefinder
    temp_alt = (float)temp_alt * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
#endif

    rangefinder_state.alt_cm = temp_alt;
    printf("rngTilt = %d ",rangefinder_state.alt_cm);
    // filter rangefinder for use by AC_WPNav
    uint32_t now = AP_HAL::millis();

    if (rangefinder_state.alt_healthy) {
        if (now - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS) {
            // reset filter if we haven't used it within the last second
            rangefinder_state.alt_cm_filt.reset(rangefinder_state.alt_cm);
        } else {
            rangefinder_state.alt_cm_filt.apply(rangefinder_state.alt_cm, 0.05f);
        }
        rangefinder_state.last_healthy_ms = now;
    }
    printf("rngFilter = %f \n\r",rangefinder_state.alt_cm_filt.get());
    // send rangefinder altitude and health to waypoint navigation library
#ifdef sub42
    wp_nav.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
    circle_nav.set_rangefinder_alt(rangefinder_state.enabled && wp_nav.rangefinder_used(), rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
#else    
    const float terrain_offset_cm = inertial_nav.get_position_z_up_cm() - rangefinder_state.alt_cm_filt.get();
    wp_nav.set_rangefinder_terrain_offset(rangefinder_state.enabled, rangefinder_state.alt_healthy, terrain_offset_cm);
    circle_nav.set_rangefinder_terrain_offset(rangefinder_state.enabled && wp_nav.rangefinder_used(), rangefinder_state.alt_healthy, terrain_offset_cm);
#endif

#else
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;
#endif
}

// return true if rangefinder_alt can be used
bool Sub::rangefinder_alt_ok() const
{
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
}

#ifdef sub42
/*
  update RPM sensors
 */
#if RPM_ENABLED == ENABLED
void Sub::rpm_update(void)
{
    rpm_sensor.update();
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        if (should_log(MASK_LOG_RCIN)) {
            logger.Write_RPM(rpm_sensor);
        }
    }
}
#endif

void Sub::accel_cal_update()
{
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    // check if new trim values, and set them
    float trim_roll, trim_pitch;
    if (ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }
}

/*
  ask airspeed sensor for a new value, duplicated from plane
 */
void Sub::read_airspeed()
{
    g2.airspeed.update(should_log(MASK_LOG_IMU));
}

#endif