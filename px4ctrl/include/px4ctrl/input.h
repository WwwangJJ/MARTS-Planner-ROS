#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/PositionCommandNew.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/ESCTelemetry.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Int32MultiArray.h> 
#include <geometry_msgs/TwistStamped.h>

#include <uav_utils/utils.h>
#include "PX4CtrlParam.h"

#include "px4ctrl/filters.hpp"
#include "px4ctrl/derivative_filter.hpp"

#include "px4ctrl/apm_channel_convert.hpp"



class RC_Data_t
{
public:
  double mode;
  double gear;
  double reboot_cmd;
  double last_mode;
  double last_gear;
  double last_reboot_cmd;
  bool have_init_last_mode{false};
  bool have_init_last_gear{false};
  bool have_init_last_reboot_cmd{false};
  double ch[4];

  mavros_msgs::RCIn msg;
  ros::Time rcv_stamp;
  double thrust_ch;

  bool is_command_mode;
  bool enter_command_mode;
  bool is_hover_mode;
  bool enter_hover_mode;
  bool toggle_reboot;

  static constexpr double GEAR_SHIFT_VALUE = 0.75;
  static constexpr double API_MODE_THRESHOLD_VALUE = 0.75;
  static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;
  static constexpr double DEAD_ZONE = 0.25;

  RC_Data_t();
  void check_validity();
  bool check_centered();
  void feed(mavros_msgs::RCInConstPtr pMsg);
  bool is_received(const ros::Time &now_time);
};

class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Quaterniond q;
  Eigen::Vector3d w;

  nav_msgs::Odometry msg;
  ros::Time rcv_stamp;
  bool recv_new_msg;

  Odom_Data_t();
  void feed(nav_msgs::OdometryConstPtr pMsg);
};


class Imu_Data_t
{
public:
  Eigen::Quaterniond q;
  Eigen::Vector3d w;
  Eigen::Vector3d wf;
  Eigen::Vector3d a;
  Eigen::Vector3d af;
  Eigen::Vector3d dw;

  sensor_msgs::Imu msg;
  ros::Time rcv_stamp;

  Imu_Data_t();
  void feed(sensor_msgs::ImuConstPtr pMsg);
};

class State_Data_t
{
public:
  mavros_msgs::State current_state;
  mavros_msgs::State state_before_offboard;

  State_Data_t();
  void feed(mavros_msgs::StateConstPtr pMsg);
};

class ExtendedState_Data_t
{
public:
  mavros_msgs::ExtendedState current_extended_state;

  ExtendedState_Data_t();
  void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
};

class Command_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p, v, a, j, s;
  double yaw;
  double yaw_rate;
  double yaw_acc;

  quadrotor_msgs::PositionCommand msg;
  ros::Time rcv_stamp;

  Command_Data_t();
  void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};

class Command_transport_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p, v, a, j, s, pl, vl, al, jl, sl;
  Eigen::Vector3d angT, dAngT, d2AngT, d3AngT, d4AngT;
  double yaw;
  double yaw_rate;
  double yaw_acc;
  int flag;

  quadrotor_msgs::PositionCommandNew msg;
  ros::Time rcv_stamp;

  Command_transport_Data_t();
  void feed(quadrotor_msgs::PositionCommandNewConstPtr pMsg);
};

class Battery_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double volt{0.0};
  double percentage{0.0};

  sensor_msgs::BatteryState msg;
  ros::Time rcv_stamp;

  Battery_Data_t();
  void feed(sensor_msgs::BatteryStateConstPtr pMsg);
};

class Takeoff_Land_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  bool triggered{false};
  uint8_t takeoff_land_cmd; // see TakeoffLand.msg for its defination

  quadrotor_msgs::TakeoffLand msg;
  ros::Time rcv_stamp;

  Takeoff_Land_Data_t();
  void feed(quadrotor_msgs::TakeoffLandConstPtr pMsg);
};

class RPM_Data_t
{
public:

  double rpm[4] = {0};
  double rpm_fil[4] = {0};
  double sum_rpm2 = 0;
  double sum_rpm2_fil = 0;
  double x_sum_rpm2 = 0;
  double y_sum_rpm2 = 0;
  double z_sum_rpm2 = 0;
  double x_sum_rpm2_fil = 0;
  double y_sum_rpm2_fil = 0;
  double z_sum_rpm2_fil = 0;
  mavros_msgs::ESCTelemetry msg;
  ros::Time rcv_stamp;

  // double thrust_N;
  // double thrust_acc;

  RPM_Data_t();
  void feed(mavros_msgs::ESCTelemetryConstPtr pMsg);
};

class ExtForce_Data_t
{
public:

  Eigen::Vector3d ext_force;
  Eigen::Vector3d ext_torque;

  geometry_msgs::TwistStamped msg;
  ros::Time rcv_stamp;

  ExtForce_Data_t();
  void feed(geometry_msgs::TwistStampedConstPtr pMsg);

};

#endif