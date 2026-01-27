#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <funs.hpp>

class Parameter_t
{
public:
	struct Gain
	{
		double KF;
		double Kp0, Kp1, Kp2;
		double Kv0, Kv1, Kv2;
		double Ka0, Ka1, Ka2;
		double Kvi0, Kvi1, Kvi2;
		double Kvd0, Kvd1, Kvd2;
		double Kq0, Kq1, Kq2;
		double KAngR, KAngP, KAngY;
		double KRateRoll, KRatePitch, KRateYaw, KiRateYaw, KiMax;
	};

	struct RotorDrag
	{
		double x, y, z;
		double k_thrust_horz;
	};

	struct MsgTimeout
	{
		double odom;
		double rc;
		double cmd;
		double imu;
		double bat;
	};

	struct ThrustMapping
	{
		bool print_val;
		double K1;
		double K2;
		double K3;
		bool accurate_thrust_model;
		double hover_percentage;
		bool noisy_imu;


		double thrust_k;
		double torque_k;
		double torque_z_comp;
		double rpm_sp;
		double rpm_kp;
		double acc_sp;
		int win_size;

		double bat_inter_R;
		double pwm_to_rpm2_b2;
		double pwm_to_rpm2_b1;
		double pwm_to_rpm2_b0;
		double pwm_to_rpm2_vol_pow;
		

		double thrust_to_rpm2_p2;
		double thrust_to_rpm2_p1;
		double thrust_to_rpm2_p0;

		double thrust_to_rpm2_l2;
		double thrust_to_rpm2_l1;
		double thrust_to_rpm2_l0;

		double model_switch_thrust;
		double model_switch_rpm;



		Eigen::Matrix<double, 4, 4> mix, mix_inv;
		Eigen::Matrix<double, 3, 3> limit_mix_0, limit_mix_0_inv;
		Eigen::Matrix<double, 3, 3> limit_mix_1, limit_mix_1_inv;
		Eigen::Matrix<double, 3, 3> limit_mix_2, limit_mix_2_inv;
		Eigen::Matrix<double, 3, 3> limit_mix_3, limit_mix_3_inv;

		double mot_pwm_min;
		double mot_pwm_max;
		double mot_spin_min;
		double mot_spin_max;
		double pwm_output_min;
		double pwm_output_max;
	};

	struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
	};

	struct AutoTakeoffLand
	{
		bool enable;
		bool enable_auto_arm;
		bool no_RC;
		double height;
		double speed;
	};

	Gain gain;
	RotorDrag rt_drag;
	MsgTimeout msg_timeout;
	RCReverse rc_reverse;
	ThrustMapping thr_map;
	AutoTakeoffLand takeoff_land;


	int pose_solver;
	double mass;
	double massL;
	double wheelbase;
	double gra;
	double inertia[3];
	double max_angle;
	double ctrl_freq_max;
	double max_manual_vel;
	double low_voltage;
	double l;
	double scale;
	double z_attach;
	double FUpperLimit;
	double FLowerLimit;


	int ctrl_mode;
	int task_mode;
	int drone_id;
	int drone_num;
	bool use_bodyrate_ctrl;
	std::vector<double> inifinAngT;
	Eigen::Vector3d hover_q;
	double hover_F;
	double rpm_Limit;
	// bool print_dbg;

	Parameter_t();
	void config_from_ros_handle(const ros::NodeHandle &nh);
	void config_full_thrust(double hov);

private:
	template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};
};

#endif