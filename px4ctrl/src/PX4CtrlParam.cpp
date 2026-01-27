#include "px4ctrl/PX4CtrlParam.h"

Parameter_t::Parameter_t()
{
}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
{
	read_essential_param(nh, "px4ctrl/drone_id", drone_id);
	read_essential_param(nh, "DroneNum", drone_num);
    read_essential_param(nh, "CableLength", l);
    read_essential_param(nh, "MassQ", mass);
    read_essential_param(nh, "MassL", massL);
    read_essential_param(nh, "DroneNum", drone_num);
    read_essential_param(nh, "GravAcc", gra);
	read_essential_param(nh, "InifinAngT", inifinAngT);
	read_essential_param(nh, "Z_attach", z_attach);
	read_essential_param(nh, "FUpperLimit", FUpperLimit);
	read_essential_param(nh, "FLowerLimit", FLowerLimit);

	read_essential_param(nh, "RPMLimit", rpm_Limit);

    read_essential_param(nh, "gain/KF", gain.KF);
	read_essential_param(nh, "gain/Kp0", gain.Kp0);
	read_essential_param(nh, "gain/Kp1", gain.Kp1);
	read_essential_param(nh, "gain/Kp2", gain.Kp2);
	read_essential_param(nh, "gain/Kv0", gain.Kv0);
	read_essential_param(nh, "gain/Kv1", gain.Kv1);
	read_essential_param(nh, "gain/Kv2", gain.Kv2);
	read_essential_param(nh, "gain/Kvi0", gain.Kvi0);
	read_essential_param(nh, "gain/Kvi1", gain.Kvi1);
	read_essential_param(nh, "gain/Kvi2", gain.Kvi2);
	read_essential_param(nh, "gain/Kq0", gain.Kq0);
	read_essential_param(nh, "gain/Kq1", gain.Kq1);
	read_essential_param(nh, "gain/Kq2", gain.Kq2);
	read_essential_param(nh, "gain/KAngR", gain.KAngR);
	read_essential_param(nh, "gain/KAngP", gain.KAngP);
	read_essential_param(nh, "gain/KAngY", gain.KAngY);

	read_essential_param(nh, "gain/KRateRoll", gain.KRateRoll);
	read_essential_param(nh, "gain/KRatePitch", gain.KRatePitch);
	read_essential_param(nh, "gain/KRateYaw", gain.KRateYaw);
	read_essential_param(nh, "gain/KiRateYaw", gain.KiRateYaw);
	read_essential_param(nh, "gain/KiMax", gain.KiMax);
	


	read_essential_param(nh, "rotor_drag/x", rt_drag.x);
	read_essential_param(nh, "rotor_drag/y", rt_drag.y);
	read_essential_param(nh, "rotor_drag/z", rt_drag.z);
	read_essential_param(nh, "rotor_drag/k_thrust_horz", rt_drag.k_thrust_horz);

	read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
	read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
	read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
	read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
	read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);

	read_essential_param(nh, "pose_solver", pose_solver);
	read_essential_param(nh, "wheelbase", wheelbase);
	read_essential_param(nh, "inertia/x", inertia[0]);
	read_essential_param(nh, "inertia/y", inertia[1]);
	read_essential_param(nh, "inertia/z", inertia[2]);
	read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);

	read_essential_param(nh, "ctrl_mode", ctrl_mode);
	read_essential_param(nh, "task_mode", task_mode);
	read_essential_param(nh, "use_bodyrate_ctrl", use_bodyrate_ctrl);
	
	read_essential_param(nh, "max_manual_vel", max_manual_vel);
	read_essential_param(nh, "max_angle", max_angle);
	read_essential_param(nh, "low_voltage", low_voltage);

	read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
	read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
	read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
	read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

	read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
    read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
    read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
	read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land.height);
	read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land.speed);

	read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
	read_essential_param(nh, "thrust_model/K1", thr_map.K1);
	read_essential_param(nh, "thrust_model/K2", thr_map.K2);
	read_essential_param(nh, "thrust_model/K3", thr_map.K3);
	read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
	read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);
	
	read_essential_param(nh, "thrust_model/thrust_k", thr_map.thrust_k);
	read_essential_param(nh, "thrust_model/torque_k", thr_map.torque_k);
	read_essential_param(nh, "thrust_model/torque_z_comp", thr_map.torque_z_comp);
	read_essential_param(nh, "thrust_model/rpm_sp", thr_map.rpm_sp);
	read_essential_param(nh, "thrust_model/rpm_kp", thr_map.rpm_kp);
	read_essential_param(nh, "thrust_model/acc_sp", thr_map.acc_sp);
	read_essential_param(nh, "thrust_model/win_size", thr_map.win_size);


	read_essential_param(nh, "thrust_model/bat_inter_R", thr_map.bat_inter_R);
	read_essential_param(nh, "thrust_model/pwm_to_rpm2_b2", thr_map.pwm_to_rpm2_b2);
	read_essential_param(nh, "thrust_model/pwm_to_rpm2_b1", thr_map.pwm_to_rpm2_b1);
	read_essential_param(nh, "thrust_model/pwm_to_rpm2_b0", thr_map.pwm_to_rpm2_b0);

	read_essential_param(nh, "thrust_model/thrust_to_rpm2_p2", thr_map.thrust_to_rpm2_p2);
	read_essential_param(nh, "thrust_model/thrust_to_rpm2_p1", thr_map.thrust_to_rpm2_p1);
	read_essential_param(nh, "thrust_model/thrust_to_rpm2_p0", thr_map.thrust_to_rpm2_p0);

	read_essential_param(nh, "thrust_model/thrust_to_rpm2_l2", thr_map.thrust_to_rpm2_l2);
	read_essential_param(nh, "thrust_model/thrust_to_rpm2_l1", thr_map.thrust_to_rpm2_l1);
	read_essential_param(nh, "thrust_model/thrust_to_rpm2_l0", thr_map.thrust_to_rpm2_l0);

	read_essential_param(nh, "thrust_model/model_switch_thrust", thr_map.model_switch_thrust);
	read_essential_param(nh, "thrust_model/model_switch_rpm", thr_map.model_switch_rpm);

	read_essential_param(nh, "thrust_model/pwm_to_rpm2_vol_pow", thr_map.pwm_to_rpm2_vol_pow);
	read_essential_param(nh, "thrust_model/mot_pwm_min", thr_map.mot_pwm_min);
	read_essential_param(nh, "thrust_model/mot_pwm_max", thr_map.mot_pwm_max);
	read_essential_param(nh, "thrust_model/mot_spin_min", thr_map.mot_spin_min);
	read_essential_param(nh, "thrust_model/mot_spin_max", thr_map.mot_spin_max);
	read_essential_param(nh, "thrust_model/pwm_output_min", thr_map.pwm_output_min);
	read_essential_param(nh, "thrust_model/pwm_output_max", thr_map.pwm_output_max);

	max_angle /= (180.0 / M_PI);
    scale = gra * massL / (drone_num * cos(inifinAngT[0] * M_PI));
	std::cout << "22222222\t" << gra << "\t" << massL << "\t" << drone_num << "\t" << inifinAngT[0] << "\t"  << scale << std::endl;
	Eigen::Vector2d cst, csp;
	Eigen::Vector4d gs;
	cst = getcs(inifinAngT[0] * M_PI);
	csp = getcs(inifinAngT[1] * M_PI + 2 * (drone_id - 1) * M_PI / drone_num);
	hover_q = getQ(cst, csp, gs);
	hover_F = scale;
	
	thr_map.mix.row(0) << 1, 1, 1, 1;
		thr_map.mix.row(0) *= (thr_map.thrust_k * 1e8);
	thr_map.mix.row(1) << -1, 1, 1, -1;
		thr_map.mix.row(1) *= sqrt(2) / 4.0 * wheelbase * thr_map.thrust_k * 1.0 * 1e8;
		// thr_map.mix.row(1) *= sqrt(2) / 4.0 * wheelbase * thr_map.thrust_k * 1.116 / 1.3950;
	thr_map.mix.row(2) << -1, 1, -1, 1;
		thr_map.mix.row(2) *= sqrt(2) / 4.0 * wheelbase * thr_map.thrust_k * 1.0 * 1e8;
		// thr_map.mix.row(2) *= sqrt(2) / 4.0 * wheelbase * thr_map.thrust_k * 0.856 / 1.4267;
	thr_map.mix.row(3) << -1, -1, 1, 1;
		thr_map.mix.row(3) *= thr_map.torque_k * 1e8;
	thr_map.mix_inv = thr_map.mix.inverse();

	thr_map.limit_mix_0.row(0) <<  1,  1, -1;
	thr_map.limit_mix_0.row(0) *= sqrt(2) / 4.0 * wheelbase * thr_map.thrust_k * 1.0 * 1e8;
	thr_map.limit_mix_0.row(1) <<  1, -1,  1;
	thr_map.limit_mix_0.row(1) *= sqrt(2) / 4.0 * wheelbase * thr_map.thrust_k * 1.0 * 1e8;
	thr_map.limit_mix_0.row(2) << -1,  1,  1;
	thr_map.limit_mix_0.row(2) *= thr_map.torque_k * 1e8;
	thr_map.limit_mix_0_inv = thr_map.limit_mix_0.inverse();

	thr_map.limit_mix_1.row(0) << -1,  1, -1;
	thr_map.limit_mix_1.row(0) *= sqrt(2) / 4.0 * wheelbase * thr_map.thrust_k * 1.0 * 1e8;
	thr_map.limit_mix_1.row(1) << -1, -1,  1;
	thr_map.limit_mix_1.row(1) *= sqrt(2) / 4.0 * wheelbase * thr_map.thrust_k * 1.0 * 1e8;
	thr_map.limit_mix_1.row(2) << -1,  1,  1;
	thr_map.limit_mix_1.row(2) *= thr_map.torque_k * 1e8;
	thr_map.limit_mix_1_inv = thr_map.limit_mix_1.inverse();

	thr_map.limit_mix_2.row(0) << -1,  1, -1;
	thr_map.limit_mix_2.row(0) *= sqrt(2) / 4.0 * wheelbase * thr_map.thrust_k * 1.0 * 1e8;
	thr_map.limit_mix_2.row(1) << -1,  1,  1;
	thr_map.limit_mix_2.row(1) *= sqrt(2) / 4.0 * wheelbase * thr_map.thrust_k * 1.0 * 1e8;
	thr_map.limit_mix_2.row(2) << -1, -1,  1;
	thr_map.limit_mix_2.row(2) *= thr_map.torque_k * 1e8;
	thr_map.limit_mix_2_inv = thr_map.limit_mix_2.inverse();

	thr_map.limit_mix_3.row(0) << -1,  1,  1;
	thr_map.limit_mix_3.row(0) *= sqrt(2) / 4.0 * wheelbase * thr_map.thrust_k * 1.0 * 1e8;
	thr_map.limit_mix_3.row(1) << -1,  1, -1;
	thr_map.limit_mix_3.row(1) *= sqrt(2) / 4.0 * wheelbase * thr_map.thrust_k * 1.0 * 1e8;
	thr_map.limit_mix_3.row(2) << -1, -1,  1;
	thr_map.limit_mix_3.row(2) *= thr_map.torque_k * 1e8;
	thr_map.limit_mix_3_inv = thr_map.limit_mix_3.inverse();

	if ( takeoff_land.enable_auto_arm && !takeoff_land.enable )
	{
		takeoff_land.enable_auto_arm = false;
		ROS_ERROR("\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
	}
	if ( takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable) )
	{
		takeoff_land.no_RC = false;
		ROS_ERROR("\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
	}

	if ( thr_map.print_val )
	{
		ROS_WARN("You should disable \"print_value\" if you are in regular usage.");
	}
};

// void Parameter_t::config_full_thrust(double hov)
// {
// 	full_thrust = mass * gra / hov;
// };
