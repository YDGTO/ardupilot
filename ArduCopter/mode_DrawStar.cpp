#include "Copter.h"
#include "Parameters.h"
#if MODE_DRAWSTAR_ENABLED == ENABLED

/*
 * Init and run calls for guided flight mode
 */

// static Vector3p guided_pos_target_cm;       // position target (used by posvel controller only)
// bool guided_pos_terrain_alt_ys;                // true if guided_pos_target_cm.z is an alt above terrain
// static Vector3f guided_vel_target_cms;      // velocity target (used by pos_vel_accel controller and vel_accel controller)
// static Vector3f guided_accel_target_cmss;   // acceleration target (used by pos_vel_accel controller vel_accel controller and accel controller)
// static uint32_t update_time_ms;             // system time of last target update to pos_vel_accel, vel_accel or accel controller

// struct {
//     uint32_t update_time_ms;
//     Quaternion attitude_quat;
//     Vector3f ang_vel;
//     float yaw_rate_cds;
//     float climb_rate_cms;   // climb rate in cms.  Used if use_thrust is false
//     float thrust;           // thrust from -1 to 1.  Used if use_thrust is true
//     bool use_yaw_rate;
//     bool use_thrust;
// } static guided_angle_state;

// init - initialise guided controller
bool ModeDrawStar::init(bool ignore_checks)
{
    gcs().send_text(MAV_SEVERITY_INFO, "-----------------------------------------------");
    // start in velaccel control mode
    path_num_ys = 0;  // 航点号清零，从而切到其他模式再切回来后，可以飞出一个新的五角星航线
    generate_path();
    pos_control_start();
    gcs().send_text(MAV_SEVERITY_INFO, "==============================================");
    return true;
}

void ModeDrawStar::generate_path()
{
    float radius_cm = g2.star_radius_cm;

    wp_nav->get_wp_stopping_point(path_ys[0]);

    path_ys[1] = path_ys[0] + Vector3f(1.0f, 0, 0) * radius_cm;
    path_ys[2] = path_ys[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    path_ys[3] = path_ys[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
    path_ys[4] = path_ys[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
    path_ys[5] = path_ys[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
    path_ys[6] = path_ys[1];

}

// initialise guided mode's position controller
void ModeDrawStar::pos_control_start()
{
    // initialise position controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    wp_nav->set_wp_destination(path_ys[0], false);

    auto_yaw.set_mode_to_default(false);

    gcs().send_text(MAV_SEVERITY_CRITICAL, "pos_control_start: %d\r\n", path_num_ys);
}

void ModeDrawStar::run()
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "Current altitude: %d", path_num_ys);
    if (path_num_ys < 6) {  // 五角星航线尚未走完
        if (wp_nav->reached_wp_destination()) {  // 到达某个端点    //wap_nav 航点导航点
            path_num_ys++;
            wp_nav->set_wp_destination(path_ys[path_num_ys], false);  // 将下一个航点位置设置为导航控制模块的目标位置
            gcs().send_text(MAV_SEVERITY_INFO, "now go into loiter mode");
        }
    } else if ((path_num_ys == 6) && wp_nav->reached_wp_destination()) {  // 五角星航线运行完成，自动进入Loiter模式
        gcs().send_text(MAV_SEVERITY_INFO, "Draw star finished, now go into loiter mode");
        copter.set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);  // 切换到loiter模式
    }
    pos_control_run();
}

// return guided mode timeout in milliseconds. Only used for velocity, acceleration, angle control, and angular rates
uint32_t ModeDrawStar::get_timeout_ms() const
{
    return MAX(copter.g2.guided_timeout, 0.1) * 1000;
}

/*
void ModeDrawStar::pos_control_run()
{
   
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // calculate terrain adjustments
    float terr_offset = 0.0f;
    if (guided_pos_terrain_alt_ys && !wp_nav->get_terrain_offset(terr_offset)) {
        // failure to set destination can only be because of missing terraFpos_control->input_pos_xyz(guided_pos_target_cm, terr_offset, pos_offset_z_buffer);in data
        copter.failsafe_terrain_on_event();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // send position and velocity targets to position controller
    guided_accel_target_cmss.zero();
    guided_vel_target_cms.zero();
 
    // stop rotating if no updates received within timeout_ms
    if (millis() - update_time_ms > get_timeout_ms()) {
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    float pos_offset_z_buffer = 0.0; // Vertical buffer size in m
    if (guided_pos_terrain_alt_ys) {
        pos_offset_z_buffer = MIN(copter.wp_nav->get_terrain_margin() * 100.0, 0.5 * fabsF(guided_pos_target_cm.z));
    }
    pos_control->input_pos_xyz(guided_pos_target_cm, terr_offset, pos_offset_z_buffer);

    // run position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();
    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}
*/

void ModeDrawStar::pos_control_run()
{
    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio && use_pilot_yaw()) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());
    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();
    // call attitude controller
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

#endif
