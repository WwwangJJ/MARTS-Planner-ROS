#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <gcopter/trajectory.hpp>
#include <gcopter/funs.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <gcopter/flatness.hpp>

using namespace Eigen;
using namespace gcopter;
using namespace flatness;

ros::Publisher cmd_pub, stop_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
std_msgs::Bool cmdb;

bool receive_traj_ = false;
bool hasTrigger = false;
boost::shared_ptr<Trajectory<5>> traj_;
double traj_duration_;
ros::Time start_time_;
Eigen::Vector3d last_pos_;
Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero()), snp(Eigen::Vector3d::Zero());

// yaw control
Eigen::Vector3d last_yaws_, yaws_;

void polyTrajCallback(quadrotor_msgs::PolyTrajPtr msg)
{
  if ((int)msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * ((int)msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<typename Piece<5>::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new Trajectory<5>(dura, cMats));
  traj_duration_ = traj_->getTotalDuration();
  cout << "uav has receive traj!"<< endl;
  receive_traj_ = true;
}

Eigen::Vector3d calculate_yaw()
{ 
  Eigen::Vector3d yaws;
  yaws.setZero();
  return yaws;
}

void publish_quad_cmd( )
{

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);
  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);
  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);
  cmd.jerk.x = jer(0);
  cmd.jerk.y = jer(1);
  cmd.jerk.z = jer(2);
  cmd.snap.x = snp(0);
  cmd.snap.y = snp(1);
  cmd.snap.z = snp(2);
  
  cmd.yaw = yaws_(0);
  cmd.yaw_dot = yaws_(1);
  cmd.yaw_acc = yaws_(2);
  cmd_pub.publish(cmd);

  // last_pos_ = p;
}


void triggerCallback(std_msgs::EmptyPtr msg)
{
  if ((!receive_traj_))
    return;

  hasTrigger = true;
  ROS_WARN("hasTrigger !");
  start_time_ = ros::Time::now();
}

void cmdCallback(const ros::TimerEvent &e)
{
  if (!hasTrigger)
  {
    return;
  }
  // cout << "ABCD"<< endl;
  ros::Time time_now = ros::Time::now();

  double t_cur = (time_now - start_time_).toSec();

  std::pair<double, double> yaw_yawdot(0, 0);

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);
    snp = traj_->getSnp(t_cur);
    
    /*** calculate yaw ***/
    yaws_ = calculate_yaw();
    last_yaws_ = yaws_;
    // last_pos_ = pos;

    // publish
    publish_quad_cmd();
  }
  else if (t_cur > traj_duration_)
  {
    cmdb.data = true;
    stop_cmd_pub.publish(cmdb);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_load_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber trigger_sub = nh.subscribe("planning/trigger", 10, triggerCallback);

  cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/pos_cmd", 50);
  stop_cmd_pub = nh.advertise<std_msgs::Bool>("planning/stop", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.001), cmdCallback);

  last_yaws_.setZero();

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}