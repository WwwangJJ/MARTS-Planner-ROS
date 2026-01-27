/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <quadrotor_msgs/geDebug.h>
#include <queue>

#include "input.h"
#include <Eigen/Dense>

struct Desired_State_t
{
	Eigen::Vector2d cst, csp;
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Vector3d s;
	Eigen::Vector3d q;
	Eigen::Vector3d dq;
	Eigen::Vector3d d2q;
	Eigen::Vector3d d3q;
	Eigen::Vector3d d4q;
	Eigen::Vector3d angT;
	Eigen::Vector3d dAngT;
	Eigen::Vector3d d2AngT;
	Eigen::Vector3d d3AngT;
	Eigen::Vector3d d4AngT;
	Eigen::Vector4d gs;
	Eigen::Quaterniond quat;
	double F;
	double dF;
	double d2F;
	double yaw;
	double yaw_rate;
	double yaw_acc;
	int flag;

	Desired_State_t() {};

	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  s(Eigen::Vector3d::Zero()),
		  q(Eigen::Vector3d::Zero()),
		  dq(Eigen::Vector3d::Zero()),
		  d2q(Eigen::Vector3d::Zero()),
		  d3q(Eigen::Vector3d::Zero()),
		  d4q(Eigen::Vector3d::Zero()),
		  angT(Eigen::Vector3d::Zero()),
		  dAngT(Eigen::Vector3d::Zero()),
		  d2AngT(Eigen::Vector3d::Zero()),
		  d3AngT(Eigen::Vector3d::Zero()),
		  d4AngT(Eigen::Vector3d::Zero()),
		  quat(odom.q),
		  F(0.0),
		  dF(0.0),
		  d2F(0.0),
		  flag(0),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0),
		  yaw_acc(0) {};
};

struct Controller_Output_t
{
	Eigen::Quaterniond q;	   // ref Orientation of the body frame with respect to the world frame
	Eigen::Vector3d bodyrates; // ref Body rates in body frame[rad/s]
	Eigen::Vector3d bodyaccs;  // ref Body acc in body frame[rad/s2]

	Eigen::Vector3d torque_des_body; // des torque in body frame[Nm]
	double thrust;					 // thrust force [N]

	// Collective mass normalized thrust
	double motor_thrust[4]; // [0~1]
};

class Controller
{
public:
	Parameter_t &param;
	double KF;
	Eigen::Vector3d Kp;
	Eigen::Vector3d Kv;
	Eigen::Vector3d Ka;
	Eigen::Vector3d Kvi;
	Eigen::Vector3d Kvd;
	Eigen::Vector3d Kq;
	Eigen::Vector3d KAng;
	Eigen::Vector3d int_e_v;
	Eigen::Vector3d Gravity;
	std::queue<std::pair<ros::Time, double>> timed_thrust;
	ros::Time last_ctrl_timestamp_{ros::Time(0)};
	Eigen::Vector3d last_bodyrate_{Eigen::Vector3d(0, 0, 0)};

	quadrotor_msgs::Px4ctrlDebug debug; // debug
	quadrotor_msgs::geDebug gedebug;

	// Thrust-accel mapping params
	double thr_scale_compensate;
	const double rho2 = 0.998; // do not change
	double thr2acc;
	double P;

	Controller(Parameter_t &);

	/* Algorithm0 from  Zhepei Wang*/
	quadrotor_msgs::Px4ctrlDebug update_alg0(
		const Desired_State_t &des,
		const Odom_Data_t &odom,
		const Imu_Data_t &imu,
		Controller_Output_t &u,
		double voltage);

	Eigen::Vector3d computeLimitedTotalAccFromThrustForce(
		const Eigen::Vector3d &thrustforce,
		const double &mass) const;

	bool flatnessWithDrag(const Eigen::Vector3d &vel,
						  const Eigen::Vector3d &acc,
						  const Eigen::Vector3d &jer,
						  const double &psi,
						  const double &dpsi,
						  double &thr,
						  Eigen::Vector4d &quat,
						  Eigen::Vector3d &omg,
						  const double &mass,
						  const double &grav,
						  const double &dh,
						  const double &dv,
						  const double &cp,
						  const double &veps) const;

	bool flatnessTransport(const Eigen::Vector3d &acc,
						   const Eigen::Vector3d &jer,
						   const Eigen::Vector3d &snp,
						   const Eigen::Vector3d &indi,
						   const Eigen::Vector3d &q,
						   const Eigen::Vector3d &dq,
						   const Eigen::Vector3d &d2q,
						   const double &F,
						   const double &dF,
						   const double &d2F,
						   const double &psi,
						   const double &dpsi,
						   const double &ddpsi,
						   double &thr,
						   Eigen::Quaterniond &quat,
						   Eigen::Vector3d &omg,
						   Eigen::Vector3d &domg,
						   const double &massQ,
						   const double &grav) const;

	bool flatnessTransportNew(const Eigen::Vector3d &acc,
							  const Eigen::Vector3d &jer,
							  const Eigen::Vector3d &snp,
							  const Eigen::Vector3d &indi,
							  const Eigen::Vector3d &q,
							  const Eigen::Vector3d &dq,
							  const Eigen::Vector3d &d2q,
							  const double &F,
							  const double &dF,
							  const double &d2F,
							  const double &psi,
							  const double &dpsi,
							  const double &ddpsi,
							  double &thr,
							  Eigen::Vector3d &torque,
							  Eigen::Quaterniond &quat,
							  Eigen::Vector3d &omg,
							  Eigen::Vector3d &domg,
							  const Eigen::Vector3d &J,
							  const double &massQ,
							  const double &grav,
							  const double &za) const;

	void minimumSingularityFlat(const double mass,
								const double grav,
								const Eigen::Vector3d &vel,
								const Eigen::Vector3d &acc,
								const Eigen::Vector3d &jer,
								const Eigen::Vector3d &snp,
								const Eigen::Vector3d &indi,
								const double &yaw,
								const double &yawd,
								const double &yawd2,
								const Eigen::Quaterniond &att_est,
								Eigen::Quaterniond &att,
								Eigen::Vector3d &omg,
								Eigen::Vector3d &domg,
								double &thrust) const;

	void minimumSingularityFlatNew(const double mass,
								   const double grav,
								   const double za,
								   const Eigen::Vector3d &J,
								   const Eigen::Vector3d &vel,
								   const Eigen::Vector3d &acc,
								   const Eigen::Vector3d &jer,
								   const Eigen::Vector3d &snp,
								   const Eigen::Vector3d &indi,
								   const double &yaw,
								   const double &yawd,
								   const double &yawd2,
								   const Eigen::Quaterniond &att_est,
								   Eigen::Quaterniond &att,
								   Eigen::Vector3d &omg,
								   Eigen::Vector3d &domg,
								   Eigen::Vector3d &torque,
								   double &thrust) const;

	void minimumSingularityTransportFlat(const double mass,
										 const double grav,
										 const Eigen::Vector3d &vel,
										 const Eigen::Vector3d &acc,
										 const Eigen::Vector3d &jer,
										 const Eigen::Vector3d &snp,
										 const Eigen::Vector3d &indi,
										 const Eigen::Vector3d &q,
										 const Eigen::Vector3d &dq,
										 const Eigen::Vector3d &d2q,
										 const double &F,
										 const double &dF,
										 const double &d2F,
										 const double &yaw,
										 const double &yawd,
										 const double &yawd2,
										 const Eigen::Quaterniond &att_est,
										 Eigen::Quaterniond &att,
										 Eigen::Vector3d &omg,
										 Eigen::Vector3d &domg,
										 double &thrust) const;

	void minimumSingularityTransportFlatNew(const double mass,
											const double grav,
											const double za,
											const Eigen::Vector3d &J,
											const Eigen::Vector3d &vel,
											const Eigen::Vector3d &acc,
											const Eigen::Vector3d &jer,
											const Eigen::Vector3d &snp,
											const Eigen::Vector3d &indi,
											const Eigen::Vector3d &q,
											const Eigen::Vector3d &dq,
											const Eigen::Vector3d &d2q,
											const double &F,
											const double &dF,
											const double &d2F,
											const double &yaw,
											const double &yawd,
											const double &yawd2,
											const Eigen::Quaterniond &att_est,
											Eigen::Quaterniond &att,
											Eigen::Vector3d &omg,
											Eigen::Vector3d &domg,
											Eigen::Vector3d &torque,
											double &thrust) const;

	void minimumSingularityFlatWithDrag(const double mass,
										const double grav,
										const Eigen::Vector3d &vel,
										const Eigen::Vector3d &acc,
										const Eigen::Vector3d &jer,
										const double &yaw,
										const double &yawd,
										const Eigen::Quaterniond &att_est,
										Eigen::Quaterniond &att,
										Eigen::Vector3d &omg,
										double &thrust) const;

	/* Algorithm1 from  Zhepei Wang*/
	quadrotor_msgs::Px4ctrlDebug update_alg1(
		const Desired_State_t &des,
		const Odom_Data_t &odom,
		const Imu_Data_t &imu,
		Controller_Output_t &u,
		double voltage);

	void normalizeWithGrad(
		const Eigen::Vector3d &x,
		const Eigen::Vector3d &xd,
		Eigen::Vector3d &xNor,
		Eigen::Vector3d &xNord) const;

	void computeFlatInput(
		const Eigen::Vector3d &thr_acc,
		const Eigen::Vector3d &jer,
		const double &yaw,
		const double &yawd,
		const Eigen::Quaterniond &att_est,
		Eigen::Quaterniond &att,
		Eigen::Vector3d &omg) const;

	/*Algorithm from the rotor-drag paper*/
	void update_alg2(
		const Desired_State_t &des,
		const Odom_Data_t &odom,
		const Imu_Data_t &imu,
		Controller_Output_t &u,
		double voltage);

	quadrotor_msgs::Px4ctrlDebug update_alg3(
		Desired_State_t &des,
		const Odom_Data_t &odom,
		const RPM_Data_t &rpm_data,
		const Imu_Data_t &imu,
		Controller_Output_t &u,
		const bool &is_hover_mode,
		const bool &is_command_mode);

	quadrotor_msgs::Px4ctrlDebug update_alg4(
		Desired_State_t &des,
		const Odom_Data_t &odom,
		const RPM_Data_t &rpm_data,
		const Imu_Data_t &imu,
		Controller_Output_t &u,
		const bool &is_hover_mode,
		const bool &is_command_mode,
		double voltage);

	double rpm_to_thrust(double rpm);
	double thrust_to_rpm(double thrust);
	double limit(double val, double max, double min);
	double getK(double thrust);
	double getK2(double thrust);
	void rpm_ctrl_allthrust(Eigen::Vector4d &des_rpm, const double rpm[], double thrust_chs[], const bool &is_hover_mode, quadrotor_msgs::Px4ctrlDebug &debug);

	void computeAeroCompensatedReferenceInputs(
		const Desired_State_t &des,
		const Odom_Data_t &odom, const Parameter_t &param,
		Controller_Output_t *u, Eigen::Vector3d *drag_acc) const;

	Eigen::Vector4d calculateLimitedRPM(const Eigen::Vector4d &cmd_rpm, const double &rpm_Limit, const Eigen::Vector4d &thrust_and_torque);

	Eigen::Quaterniond computeDesiredAttitude(
		const Eigen::Vector3d &des_acc, const double reference_heading,
		const Eigen::Quaterniond &est_q) const;

	Eigen::Vector3d computeRobustBodyXAxis(
		const Eigen::Vector3d &x_B_prototype, const Eigen::Vector3d &x_C,
		const Eigen::Vector3d &y_C,
		const Eigen::Quaterniond &est_q) const;

	/* Shared functions*/
	Eigen::Vector3d computePIDErrorAcc(
		const Odom_Data_t &odom,
		const Desired_State_t &des,
		const Parameter_t &param);

	Eigen::Vector3d computeIndiPIDErrorAcc(
		const Odom_Data_t &odom,
		const Imu_Data_t &imu,
		const Desired_State_t &des,
		const Parameter_t &param,
		const bool &is_command_mode,
		const bool &is_hover_mode);

	Eigen::Vector3d computeTransportIndiPIDErrorAcc(
		const Odom_Data_t &odom,
		const Imu_Data_t &imu,
		const Desired_State_t &des,
		const Parameter_t &param,
		const Eigen::Vector3d &indiq,
		const double &indiF);

	Eigen::Vector3d computeLimitedTotalAcc(
		const Eigen::Vector3d &PIDErrorAcc,
		const Eigen::Vector3d &ref_acc,
		const Eigen::Vector3d &drag_acc = Eigen::Vector3d::Zero()) const;

	Eigen::Vector3d computeLimitedAngularAcc(
		const Eigen::Vector3d candirate_bodyrate);

	double computeDesiredCollectiveThrustSignal(
		const Eigen::Quaterniond &est_q,
		const Eigen::Vector3d &est_v,
		const Eigen::Vector3d &des_acc,
		const Parameter_t &param,
		double voltage);

	double AccurateThrustAccMapping(
		const double des_acc_z,
		double voltage,
		const Parameter_t &param) const;

	Eigen::Vector3d computeFeedBackControlBodyrates(
		const Eigen::Quaterniond &des_q,
		const Eigen::Quaterniond &est_q,
		const Parameter_t &param);

	Eigen::Vector3d computeFeedBackControlBodyaccs(
		const Eigen::Quaterniond &des_q,
		const Eigen::Quaterniond &est_q,
		const Eigen::Vector3d &des_bodyrate,
		const Eigen::Vector3d &est_bodyrate,
		const Parameter_t &param,
		const bool &is_hover_mode,
		quadrotor_msgs::Px4ctrlDebug &debug);

	bool estimateThrustModel(
		const Eigen::Vector3d &est_a,
		const double voltage,
		const Eigen::Vector3d &est_v,
		const Parameter_t &param);

	bool almostZero(const double value) const;

	bool almostZeroThrust(const double thrust_value) const;

	void resetThrustMapping(void);

	Eigen::Vector3d quat_to_rpy(const Eigen::Quaterniond &q);
	Eigen::Vector3d getBodyControlTorque(const double rpm[]);
	double getBodyThrust(const double rpm[]);

private:
	static constexpr double kMinNormalizedCollectiveAcc_ = 3;
	static constexpr double kAlmostZeroValueThreshold_ = 0.001;
	static constexpr double kAlmostZeroThrustThreshold_ = 0.01;
	static constexpr double kMaxBodyratesFeedback_ = 4;
	static constexpr double kMaxAngularAcc_ = 60;
};

#endif
