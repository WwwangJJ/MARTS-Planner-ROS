#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <gcopter/trajectory.hpp>
#include <gcopter/funs.hpp>
#include <quadrotor_msgs/PositionCommandNew.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <gcopter/flatness.hpp>

using namespace Eigen;
using namespace gcopter;
using namespace flatness;

ros::Publisher cmd_pub, stop_cmd_pub;

quadrotor_msgs::PositionCommandNew cmd;
std_msgs::Bool cmdb;

double replan_duration;
double traj_duration_now;
bool trajIsChanged = false;

bool receive_traj_load_ = false;
bool receive_traj_cable_ = false;
bool hasTriggerFlight = false;
bool hasTriggerTakeOff = false;
bool hasTriggerSwitchControl = false;

boost::shared_ptr<Trajectory<7>> traj_load_;
boost::shared_ptr<Trajectory<7>> traj_cable_;

boost::shared_ptr<Trajectory<7>> traj_load_now;
boost::shared_ptr<Trajectory<7>> traj_cable_now;

double traj_duration_, transient_time_;
double len, initHeight, startHeight, initYaw, takeOffVel;
ros::Time start_time_, start_takeoff_time_;
int drone_id_, drone_num_;
int flag;
Eigen::Vector3d last_pos_;
Eigen::Vector3d posL(Eigen::Vector3d::Zero()), velL(Eigen::Vector3d::Zero()), accL(Eigen::Vector3d::Zero()), jerL(Eigen::Vector3d::Zero()), snpL(Eigen::Vector3d::Zero()), posQ(Eigen::Vector3d::Zero()), velQ(Eigen::Vector3d::Zero()), accQ(Eigen::Vector3d::Zero()), jerQ(Eigen::Vector3d::Zero()), snpQ(Eigen::Vector3d::Zero()), angT(Eigen::Vector3d::Zero()), dAngT(Eigen::Vector3d::Zero()), d2AngT(Eigen::Vector3d::Zero()), d3AngT(Eigen::Vector3d::Zero()), d4AngT(Eigen::Vector3d::Zero());

// yaw control
Eigen::Vector3d last_yaws_, yaws_, init_pos;
std::vector<double> inifinAngT, initLoadPos;
// double kk = 0.0;

void polyTrajCallback(quadrotor_msgs::PolyTrajPtr msg)
{
  if (((int)msg->type == 0 && (int)msg->drone_id != 0) || ((int)msg->type == 1 && (int)msg->drone_id != drone_id_))
  {
    ROS_ERROR("[traj_server] received drone_id != drone_id ");
    return;
  }
  if ((int)msg->order != 7)
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
  std::vector<typename Piece<7>::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i8 = i * 8;
    cMats[i].row(0) << msg->coef_x[i8 + 0], msg->coef_x[i8 + 1], msg->coef_x[i8 + 2],
        msg->coef_x[i8 + 3], msg->coef_x[i8 + 4], msg->coef_x[i8 + 5], msg->coef_x[i8 + 6], msg->coef_x[i8 + 7];
    cMats[i].row(1) << msg->coef_y[i8 + 0], msg->coef_y[i8 + 1], msg->coef_y[i8 + 2],
        msg->coef_y[i8 + 3], msg->coef_y[i8 + 4], msg->coef_y[i8 + 5], msg->coef_y[i8 + 6], msg->coef_y[i8 + 7];
    cMats[i].row(2) << msg->coef_z[i8 + 0], msg->coef_z[i8 + 1], msg->coef_z[i8 + 2],
        msg->coef_z[i8 + 3], msg->coef_z[i8 + 4], msg->coef_z[i8 + 5], msg->coef_z[i8 + 6], msg->coef_z[i8 + 7];

    dura[i] = msg->duration[i];
  }

  if ((int)msg->type == 0)
  {
    traj_load_.reset(new Trajectory<7>(dura, cMats));
    traj_duration_ = traj_load_->getTotalDuration();
    cout << "uav" << drone_id_ << "\t have receive load traj!"<< endl;
    receive_traj_load_ = true;
  }
  else if ((int)msg->type == 1)
  {
    traj_cable_.reset(new Trajectory<7>(dura, cMats));
    cout << "uav" << drone_id_ << "\t have receive cable traj!"<< endl;
    receive_traj_cable_ = true;
  }

  replan_duration = msg->replan_duration;
  trajIsChanged = false;

  ROS_INFO("[Traj Server Output] replan_duration: %f", replan_duration);

  if(receive_traj_load_ && receive_traj_cable_)
  {
    hasTriggerFlight = true;
    hasTriggerSwitchControl = false;
    hasTriggerTakeOff = false;
  }

}

Eigen::Vector3d calculate_yaw()
{ 
  Eigen::Vector3d yaws;
  yaws.setZero();
  return yaws;
}

void publish_quad_cmd(const int &flag)
{

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommandNew::TRAJECTORY_STATUS_READY;
  cmd.flag = flag;
  cmd.position.x = posL(0);
  cmd.position.y = posL(1);
  cmd.position.z = posL(2);
  cmd.velocity.x = velL(0);
  cmd.velocity.y = velL(1);
  cmd.velocity.z = velL(2);
  cmd.acceleration.x = accL(0);
  cmd.acceleration.y = accL(1);
  cmd.acceleration.z = accL(2);
  cmd.jerk.x = jerL(0);
  cmd.jerk.y = jerL(1);
  cmd.jerk.z = jerL(2);
  cmd.snap.x = snpL(0);
  cmd.snap.y = snpL(1);
  cmd.snap.z = snpL(2);
  cmd.angT.x = angT(0);
  cmd.angT.y = angT(1);
  cmd.angT.z = angT(2);
  cmd.dAngT.x = dAngT(0);
  cmd.dAngT.y = dAngT(1);
  cmd.dAngT.z = dAngT(2);
  cmd.d2AngT.x = d2AngT(0);
  cmd.d2AngT.y = d2AngT(1);
  cmd.d2AngT.z = d2AngT(2);
  cmd.d3AngT.x = d3AngT(0);
  cmd.d3AngT.y = d3AngT(1);
  cmd.d3AngT.z = d3AngT(2);
  cmd.d4AngT.x = d4AngT(0);
  cmd.d4AngT.y = d4AngT(1);
  cmd.d4AngT.z = d4AngT(2);
  
  cmd.yaw = yaws_(0);
  cmd.yaw_dot = yaws_(1);
  cmd.yaw_acc = yaws_(2);
  cmd_pub.publish(cmd);
   
  // last_pos_ = p;
}

void triggerCallback(std_msgs::EmptyPtr msg)
{
  if ((!receive_traj_load_) || (!receive_traj_cable_))
    return;

  hasTriggerFlight = true;
  hasTriggerSwitchControl = false;
  hasTriggerTakeOff = false;
  ROS_WARN("hasTriggerFlight !");
  start_time_ = ros::Time::now();
}

void triggerSwitchControlCallback(std_msgs::EmptyPtr msg)
{
  // if ((!receive_traj_load_) || (!receive_traj_cable_))
  //   return;

  hasTriggerSwitchControl = true;
  hasTriggerTakeOff = false;
  ROS_WARN("hasTriggerSwitchControl !");
  start_time_ = ros::Time::now();
}

void triggerTakeOffCallback(std_msgs::EmptyPtr msg)
{

  hasTriggerTakeOff = true;
  ROS_WARN("hasTakeOffTrigger !");
  start_takeoff_time_ = ros::Time::now();
}

void cmdCallback(const ros::TimerEvent &e)
{
  if (!hasTriggerFlight && !hasTriggerTakeOff && !hasTriggerSwitchControl)
  {
    return;
  }
  // cout << "ABCD"<< endl;
  ros::Time time_now = ros::Time::now();

  if (hasTriggerTakeOff)
  {
    double t_cur = (time_now - start_takeoff_time_).toSec();
    posL.x() = init_pos.x();
    posL.y() = init_pos.y();
    velL.x() = 0.0;
    velL.y() = 0.0;
    if (t_cur <= transient_time_)
    {  
      posL.z() = init_pos.z();
      velL.z() = 0.0;
    }
    else if (t_cur <= transient_time_ + (startHeight - init_pos.z()) / takeOffVel)
    {
      posL.z() = min(init_pos.z() + takeOffVel * (t_cur - transient_time_), startHeight);
      velL.z() = takeOffVel;
    }
    else
    {
      posL.z() = startHeight;
      velL.z() = 0.0;
    }

    accL = Eigen::Vector3d::Zero();
    jerL = Eigen::Vector3d::Zero();
    snpL = Eigen::Vector3d::Zero();
    angT = Eigen::Vector3d::Zero();
    dAngT = Eigen::Vector3d::Zero();
    d2AngT = Eigen::Vector3d::Zero();
    d3AngT = Eigen::Vector3d::Zero();
    d4AngT = Eigen::Vector3d::Zero();
    yaws_ = calculate_yaw();
    flag = 0;
    publish_quad_cmd(flag);
    // cout << "aaaa" << endl;

  }
  else if (hasTriggerSwitchControl)
  {
    posL.x() = init_pos.x();
    posL.y() = init_pos.y();
    posL.z() = startHeight;
    velL = Eigen::Vector3d::Zero();
    accL = Eigen::Vector3d::Zero();
    jerL = Eigen::Vector3d::Zero();
    snpL = Eigen::Vector3d::Zero();
    angT = Eigen::Vector3d::Zero();
    dAngT = Eigen::Vector3d::Zero();
    d2AngT = Eigen::Vector3d::Zero();
    d3AngT = Eigen::Vector3d::Zero();
    d4AngT = Eigen::Vector3d::Zero();
    yaws_ = calculate_yaw();
    flag = 1;
    publish_quad_cmd(flag);
  }
  else if (hasTriggerFlight)
  {
    // double t_cur = (time_now - start_time_).toSec();
    double t_cur;
    t_cur = (time_now - start_time_).toSec();
    std::pair<double, double> yaw_yawdot(0, 0);

    if(replan_duration == 0)
    {
      if(!trajIsChanged)
      {
        traj_load_now = traj_load_;
        traj_cable_now = traj_cable_;
        traj_duration_now = traj_duration_;
        trajIsChanged = true;
        start_time_ = ros::Time::now();
      }

      time_now = ros::Time::now();
      t_cur = (time_now - start_time_).toSec();

      ROS_INFO("[replan_duration 0 Output] t_cur: %f", t_cur);

      if (t_cur < traj_duration_now && t_cur >= 0.0)
      {
        posL = traj_load_now->getPos(t_cur);
        velL = traj_load_now->getVel(t_cur);
        accL = traj_load_now->getAcc(t_cur);
        jerL = traj_load_now->getJer(t_cur);
        snpL = traj_load_now->getSnp(t_cur);

        angT = traj_cable_now->getPos(t_cur);
        dAngT = traj_cable_now->getVel(t_cur);
        d2AngT = traj_cable_now->getAcc(t_cur);
        d3AngT = traj_cable_now->getJer(t_cur);
        d4AngT = traj_cable_now->getSnp(t_cur);

        /*** calculate yaw ***/
        yaws_ = calculate_yaw();
        last_yaws_ = yaws_;
        // last_pos_ = pos;

        // publish
        flag = 2;
        publish_quad_cmd(flag);
      }
      else if (t_cur > traj_duration_now)
      {
        // cmdb.data = true;
        // stop_cmd_pub.publish(cmdb);
        posL = traj_load_now->getPos(traj_duration_now);
        velL = Eigen::Vector3d::Zero();
        accL = Eigen::Vector3d::Zero();
        jerL = Eigen::Vector3d::Zero();
        snpL = Eigen::Vector3d::Zero();

        angT = traj_cable_now->getPos(traj_duration_now);
        dAngT = Eigen::Vector3d::Zero();
        d2AngT = Eigen::Vector3d::Zero();
        d3AngT = Eigen::Vector3d::Zero();
        d4AngT = Eigen::Vector3d::Zero();

        /*** calculate yaw ***/
        yaws_ = calculate_yaw();
        last_yaws_ = yaws_;
        // last_pos_ = pos;

        // publish
        flag = 2;
        publish_quad_cmd(flag);
      }

    }
    else
    {
      if(t_cur >= replan_duration && !trajIsChanged)
      {
        traj_load_now = traj_load_;
        traj_cable_now = traj_cable_;
        traj_duration_now = traj_duration_;
        trajIsChanged = true;
        start_time_ = ros::Time::now();
        time_now = ros::Time::now();
        t_cur = (time_now - start_time_).toSec();
        ROS_INFO("[replan has changed!!!!!] t_cur: %f", t_cur);
      }
      time_now = ros::Time::now();
      t_cur = (time_now - start_time_).toSec();

      ROS_INFO("[replan_duration not0 Output] t_cur: %f", t_cur);

      if (t_cur < traj_duration_now && t_cur >= 0.0)
      {
        posL = traj_load_now->getPos(t_cur);
        velL = traj_load_now->getVel(t_cur);
        accL = traj_load_now->getAcc(t_cur);
        jerL = traj_load_now->getJer(t_cur);
        snpL = traj_load_now->getSnp(t_cur);

        angT = traj_cable_now->getPos(t_cur);
        dAngT = traj_cable_now->getVel(t_cur);
        d2AngT = traj_cable_now->getAcc(t_cur);
        d3AngT = traj_cable_now->getJer(t_cur);
        d4AngT = traj_cable_now->getSnp(t_cur);

        /*** calculate yaw ***/
        yaws_ = calculate_yaw();
        last_yaws_ = yaws_;
        // last_pos_ = pos;

        // publish
        flag = 2;
        publish_quad_cmd(flag);
      }
      else if (t_cur > traj_duration_now)
      {
        // cmdb.data = true;
        // stop_cmd_pub.publish(cmdb);
        posL = traj_load_now->getPos(traj_duration_now);
        velL = Eigen::Vector3d::Zero();
        accL = Eigen::Vector3d::Zero();
        jerL = Eigen::Vector3d::Zero();
        snpL = Eigen::Vector3d::Zero();

        angT = traj_cable_now->getPos(traj_duration_now);
        dAngT = Eigen::Vector3d::Zero();
        d2AngT = Eigen::Vector3d::Zero();
        d3AngT = Eigen::Vector3d::Zero();
        d4AngT = Eigen::Vector3d::Zero();

        /*** calculate yaw ***/
        yaws_ = calculate_yaw();
        last_yaws_ = yaws_;
        // last_pos_ = pos;

        // publish
        flag = 2;
        publish_quad_cmd(flag);
      }
    }


    // if (t_cur < traj_duration_ && t_cur >= 0.0)
    // {
    //   posL = traj_load_->getPos(t_cur);
    //   velL = traj_load_->getVel(t_cur);
    //   accL = traj_load_->getAcc(t_cur);
    //   jerL = traj_load_->getJer(t_cur);
    //   snpL = traj_load_->getSnp(t_cur);

    //   angT = traj_cable_->getPos(t_cur);
    //   dAngT = traj_cable_->getVel(t_cur);
    //   d2AngT = traj_cable_->getAcc(t_cur);
    //   d3AngT = traj_cable_->getJer(t_cur);
    //   d4AngT = traj_cable_->getSnp(t_cur);

    //   /*** calculate yaw ***/
    //   yaws_ = calculate_yaw();
    //   last_yaws_ = yaws_;
    //   // last_pos_ = pos;

    //   // publish
    //   flag = 2;
    //   publish_quad_cmd(flag);
    // }
    // else if (t_cur > traj_duration_)
    // {
    //   // cmdb.data = true;
    //   // stop_cmd_pub.publish(cmdb);
    //   posL = traj_load_->getPos(traj_duration_);
    //   velL = Eigen::Vector3d::Zero();
    //   accL = Eigen::Vector3d::Zero();
    //   jerL = Eigen::Vector3d::Zero();
    //   snpL = Eigen::Vector3d::Zero();

    //   angT = traj_cable_->getPos(traj_duration_);
    //   dAngT = Eigen::Vector3d::Zero();
    //   d2AngT = Eigen::Vector3d::Zero();
    //   d3AngT = Eigen::Vector3d::Zero();
    //   d4AngT = Eigen::Vector3d::Zero();

    //   /*** calculate yaw ***/
    //   yaws_ = calculate_yaw();
    //   last_yaws_ = yaws_;
    //   // last_pos_ = pos;

    //   // publish
    //   flag = 2;
    //   publish_quad_cmd(flag);
    // }
    // cout << "bbbb" << endl;
  }


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_load_sub = nh.subscribe("planning/trajectoryLoad", 10, polyTrajCallback);
  ros::Subscriber poly_traj_cable_sub = nh.subscribe("planning/trajectoryCable", 10, polyTrajCallback);
  ros::Subscriber trigger_sub = nh.subscribe("planning/triggerFlight", 10, triggerCallback);
  ros::Subscriber trigger_takeoff_sub = nh.subscribe("planning/triggerTakeOff", 10, triggerTakeOffCallback);
  ros::Subscriber trigger_switch_control_sub = nh.subscribe("planning/triggerSwitchControl", 10, triggerSwitchControlCallback);

  cmd_pub = nh.advertise<quadrotor_msgs::PositionCommandNew>("/pos_cmd", 50);
  stop_cmd_pub = nh.advertise<std_msgs::Bool>("planning/stop", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.003), cmdCallback);
  nh.param("traj_server/drone_id", drone_id_, 0);
  nh.param("CableLength", len, 1.0);
  nh.param("DroneNum", drone_num_, 3);
  nh.getParam("InifinAngT", inifinAngT);
  nh.getParam("InitLoadPos", initLoadPos);
  nh.getParam("initHeight", initHeight);
  nh.getParam("initYaw", initYaw);
  nh.getParam("takeOffVel", takeOffVel);
  nh.getParam("transientTime", transient_time_);
  // cout << "111\t" << drone_num_  << "\t" << len << endl;
  // cout << "222\t" <<  inifinAngT[0] << "\t" << inifinAngT[1] << "\t" << inifinAngT[2] << endl;

  init_pos.x() = initLoadPos[0] + len * sin(inifinAngT[0] * M_PI) * cos(2 * (drone_id_ - 1) * M_PI / drone_num_ + inifinAngT[1] * M_PI);
  init_pos.y() = initLoadPos[1] + len * sin(inifinAngT[0] * M_PI) * sin(2 * (drone_id_ - 1) * M_PI / drone_num_ + inifinAngT[1] * M_PI);

  // init_pos.x() = len * sin(inifinAngT[0] * M_PI) * cos(M_PI);
  // init_pos.y() = len * sin(inifinAngT[0] * M_PI) * sin(M_PI);
  // init_pos.x() = 4.615;
  // init_pos.y() = 0.719;
  init_pos.z() = initHeight;
  // cout << init_pos.z() << endl;
  startHeight = initLoadPos[2] + len * cos(inifinAngT[0] * M_PI);
  // startHeight = 1.45;
  // cout << "111\t" << takeOffVel << endl;
  // cout << "222\t" << takeOffVel << endl;
  last_yaws_.setZero();

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}