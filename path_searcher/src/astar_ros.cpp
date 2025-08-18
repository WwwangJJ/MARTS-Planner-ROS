#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <ctime>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "quadrotor_msgs/PolyTraj.h"

#include "Astar.h"

using namespace std;
using namespace Eigen;

//分辨率、分辨率倒数、未知？？？
double _resolution, _inv_resolution, _cloud_margin;

//世界坐标系(单位m)下，整个点云地图的长、宽、高
double _x_size, _y_size, _z_size;

//世界坐标系(单位m)下，搜索范围内的最大与最小的x、y、z
double _search_x_max, _search_x_min, _search_y_max, _search_y_min, _search_z_max, _search_z_min;

//栅格坐标系(与世界坐标系相差分辨率)下，整个点云地图的长、宽、高
int _max_x_id, _max_y_id, _max_z_id;

//起点坐标 世界坐标系(单位m)
Vector3d _start_pt, _target_pt, target_pt_Exp_Tmp;
bool _has_start_pt  = false;
bool _is_replan     = false;
int replan_num = 1;

//地图三轴最小和最大尺寸 世界坐标系(单位m)
Vector3d _map_lower, _map_upper;

//搜索三轴最小和最大尺寸 世界坐标系(单位m)
Vector3d _search_lower, _search_upper;

std::string _distance;
int _wayPointSplitNum;
double _tar_ang;
double _weight_a,_weight_b,_weight_angle,_length,_max_angle,_min_angle;
int _angle_resulution;


ros::Subscriber _map_sub;               //点云地图的接收者
ros::Subscriber _pts_sub;               //终点坐标的接收者
ros::Subscriber _trans_goal_sub;
ros::Subscriber _polyTrajSub;

ros::Publisher  _grid_path_vis_pub;     //发布Astar找到的路径
ros::Publisher  _visited_nodes_vis_pub; //发布OpenList/CloseList的方格
ros::Publisher  _grid_map_vis_pub;      //发布点云地图
ros::Publisher  _start_point_vis_pub;
ros::Publisher  _target_point_vis_pub;
ros::Publisher  _transport_vis_pub;
ros::Publisher  _line_strip_pub;
ros::Publisher  _replay_pt_pub;

double rvizPointID = 0;

//Astar算法的对象指针
AstarPathFinder * _astar_path_finder     = NULL;

//标志位，确保先有地图再有终点坐标
bool _has_map   = false;

// 是否采用实验模式
bool _isExp;
bool _isReplanMode;
bool _isReplanDemo1;
bool _isReplanDemo2;
bool _isNarrowSlitMode;
double _time_ahead;

ros::Time time_replan;
double time_all;

bool getTmpGoal = false;

int piece_nums;
quadrotor_msgs::PolyTraj trajmsg;

void pathFinding(const Vector3d start_pt, const Vector3d target_pt);

void clearMarkers()
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    _transport_vis_pub.publish(marker_array);
}

void visRvizStart(Vector3d Point)
{
    visualization_msgs::Marker point_vis;
    point_vis.header.frame_id = "odom";
    point_vis.header.stamp = ros::Time::now();

    point_vis.ns = "astar_node/start_point";
    point_vis.id = rvizPointID;

    point_vis.type = visualization_msgs::Marker::SPHERE;
    point_vis.action = visualization_msgs::Marker::ADD;

    point_vis.pose.position.x = Point(0);
    point_vis.pose.position.y = Point(1);
    point_vis.pose.position.z = Point(2);

    point_vis.pose.orientation.x = 0.0;
    point_vis.pose.orientation.y = -sqrt(2)/2;
    point_vis.pose.orientation.z = 0.0;
    point_vis.pose.orientation.w = sqrt(2)/2;

    point_vis.color.a = 1.0;
    point_vis.color.r = 1.0;
    point_vis.color.g = 1.0;
    point_vis.color.b = 1.0;

    point_vis.scale.x = 0.2;
    point_vis.scale.y = 0.2;
    point_vis.scale.z = 0.2;

    _start_point_vis_pub.publish(point_vis);
}

void visRvizTarget(Vector3d Point)
{
    visualization_msgs::Marker point_vis;
    point_vis.header.frame_id = "odom";
    point_vis.header.stamp = ros::Time::now();

    point_vis.ns = "astar_node/target_point";
    point_vis.id = rvizPointID;

    point_vis.type = visualization_msgs::Marker::SPHERE;
    point_vis.action = visualization_msgs::Marker::ADD;

    point_vis.pose.position.x = Point(0);
    point_vis.pose.position.y = Point(1);
    point_vis.pose.position.z = Point(2);

    point_vis.pose.orientation.x = 0.0;
    point_vis.pose.orientation.y = -sqrt(2)/2;
    point_vis.pose.orientation.z = 0.0;
    point_vis.pose.orientation.w = sqrt(2)/2;

    point_vis.color.a = 1.0;
    point_vis.color.r = 1.0;
    point_vis.color.g = 0.0;
    point_vis.color.b = 0.0;

    point_vis.scale.x = 0.2;
    point_vis.scale.y = 0.2;
    point_vis.scale.z = 0.2;

    _target_point_vis_pub.publish(point_vis);
}

void transGoalCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    double goal_x, goal_y, goal_z;
    goal_x = msg->pose.pose.position.x;
    goal_y = msg->pose.pose.position.y;
    goal_z = msg->pose.pose.position.z;

    target_pt_Exp_Tmp << goal_x, goal_y, goal_z;

    visRvizTarget(target_pt_Exp_Tmp);

    getTmpGoal = true;
}
// Eigen::Vector3d
Eigen::Vector3d getReplanStartPt(quadrotor_msgs::PolyTraj trajmsg, double duration)
{
    int i;
    int piece_nums = trajmsg.duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<Eigen::Matrix<double, 3, 8>> cMats(piece_nums);
    for (i = 0; i < piece_nums; ++i)
    {
        int i8 = i * 8;
        cMats[i].row(0) << trajmsg.coef_x[i8 + 0], trajmsg.coef_x[i8 + 1], trajmsg.coef_x[i8 + 2],
            trajmsg.coef_x[i8 + 3], trajmsg.coef_x[i8 + 4], trajmsg.coef_x[i8 + 5], trajmsg.coef_x[i8 + 6], trajmsg.coef_x[i8 + 7];
        cMats[i].row(1) << trajmsg.coef_y[i8 + 0], trajmsg.coef_y[i8 + 1], trajmsg.coef_y[i8 + 2],
            trajmsg.coef_y[i8 + 3], trajmsg.coef_y[i8 + 4], trajmsg.coef_y[i8 + 5], trajmsg.coef_y[i8 + 6], trajmsg.coef_y[i8 + 7];
        cMats[i].row(2) << trajmsg.coef_z[i8 + 0], trajmsg.coef_z[i8 + 1], trajmsg.coef_z[i8 + 2],
            trajmsg.coef_z[i8 + 3], trajmsg.coef_z[i8 + 4], trajmsg.coef_z[i8 + 5], trajmsg.coef_z[i8 + 6], trajmsg.coef_z[i8 + 7];
        dura[i] = trajmsg.duration[i];
        if(duration <= dura[i])
        {
            double t = duration;
            Eigen::VectorXd time;
            time.resize(8);
            time << pow(t,7), pow(t,6), pow(t,5), pow(t,4), pow(t,3), pow(t,2), pow(t,1), pow(t,0);
            Eigen::Vector3d Start_Point = cMats[i] * time;
            std::cout << "!!!!!!!!!!!!!!!!!!" << Start_Point << "!!!!!!!!!!!!!!!!!!" << std::endl;
            return Start_Point;
        }
        else
        {
            duration -= dura[i];
        }
    }
    if(i >= piece_nums)
    {
        double t = dura[piece_nums-1];
        Eigen::VectorXd time;
        time.resize(8);
        time << pow(t,7), pow(t,6), pow(t,5), pow(t,4), pow(t,3), pow(t,2), pow(t,1), pow(t,0);
        Eigen::Vector3d Start_Point = cMats[piece_nums-1] * time;
        std::cout << "!!!!!!!!!!!!!!!!!!" << Start_Point << "!!!!!!!!!!!!!!!!!!" << std::endl;
        return Start_Point;
    }


}

//终点坐标的回调函数
void rcvWaypointsCallback(const nav_msgs::Path & wp)
{   
    //安全性检查 
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;

    if(_isExp == false && !_isReplanMode)
    {
        if(!_has_start_pt)
        {
            // Set by yourself
            _start_pt   <<      wp.poses[0].pose.position.x,
                                wp.poses[0].pose.position.y,
                                wp.poses[0].pose.position.z;

            _has_start_pt = true;
            visRvizStart(_start_pt);
            ROS_INFO("[node] receive the planning start point");
        }
        else
        {
            _target_pt  <<      wp.poses[0].pose.position.x,
                                wp.poses[0].pose.position.y,
                                wp.poses[0].pose.position.z;
            
            visRvizTarget(_target_pt);
            ROS_INFO("[node] receive the planning target point");
            _has_start_pt = false;
            ROS_INFO("[node] Starting Astar Path Finding!");
            pathFinding(_start_pt, _target_pt);
        }    

    }
    else if(_isReplanMode)
    {
        // _has_start_pt = true;
        if(!_has_start_pt)
        {
            // _start_pt   <<      wp.poses[0].pose.position.x,
            //                     wp.poses[0].pose.position.y,
            //                     wp.poses[0].pose.position.z;
            _start_pt   <<      wp.poses[0].pose.position.x,
                                wp.poses[0].pose.position.y,
                                0.5;
            _has_start_pt = true;
            visRvizStart(_start_pt);
            ROS_INFO("[node] receive the planning start point");
        }
        else
        {
            if(!_is_replan)
            {
                // _target_pt  <<      wp.poses[0].pose.position.x,
                //                     wp.poses[0].pose.position.y,
                //                     wp.poses[0].pose.position.z;
                _target_pt  <<      wp.poses[0].pose.position.x,
                                    wp.poses[0].pose.position.y,
                                    0.5;


                visRvizTarget(_target_pt);
                _is_replan = true;
            }
            else
            {
                double start_time = trajmsg.traj_start_time;
                time_replan = ros::Time::now();
                ROS_INFO("111111111111, time_replan, %f", time_replan.toSec());
                double duration = time_replan.toSec() - start_time + _time_ahead;
                ROS_INFO("111111111111, duration, %f", duration);
                
                _start_pt = getReplanStartPt(trajmsg, duration);
                // getReplanStartPt(trajmsg, duration);

                // _start_pt << _target_pt;

                visRvizStart(_start_pt);
                // _target_pt  <<      wp.poses[0].pose.position.x,
                //                     wp.poses[0].pose.position.y,
                //                     wp.poses[0].pose.position.z;
                _target_pt  <<      wp.poses[0].pose.position.x,
                                    wp.poses[0].pose.position.y,
                                    0.8;
                visRvizTarget(_target_pt);
            }
            ROS_INFO("[node] receive the planning target point");
            ROS_INFO("[node] Starting Astar Path Finding!");
            pathFinding(_start_pt, _target_pt);
        }
    }
    else
    {
        if(_has_start_pt == false)
        {
            _start_pt << -7.5, 0.0, 0.8;
            visRvizStart(_start_pt);
            _has_start_pt = true;
            ROS_INFO("[node] receive the planning start point");
        }
        else
        {
            _start_pt << _target_pt;
            visRvizStart(_start_pt);
            ROS_INFO("[node] receive the planning start point");
        }
        if(!getTmpGoal)
        {
            target_pt_Exp_Tmp << _start_pt;
            target_pt_Exp_Tmp[0] += 8.0;
            target_pt_Exp_Tmp[1] += 1.5;
        }
        _target_pt << target_pt_Exp_Tmp;
        ROS_INFO("[node] receive the planning target point");
        visRvizTarget(_target_pt);
        ROS_INFO("[node] Starting Astar Path Finding!");
        pathFinding(_start_pt, _target_pt);
    }

}

//点云地图的回调函数
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    if(!_isExp)
    {
        if(_has_map ) return;
    }
    else
    {
        _astar_path_finder->clearObs();
    }


    //PCL点云格式
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    //ROS点云格式
    sensor_msgs::PointCloud2 map_vis;
    //将ROS格式转成PCL格式
    pcl::fromROSMsg(pointcloud_map, cloud);
    
    if( (int)cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt;   //世界坐标系的三维点
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        

        // 通过世界坐标系的三维点云坐标 设置 一维点云数据格式中的障碍物
        _astar_path_finder->setObs(pt.x, pt.y, pt.z);
        
        // 仅仅为了显示
        Vector3d cor_round = _astar_path_finder->coordRounding(Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;
    
    //PCL格式-->ROS格式
    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "/odom";
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
}

void polyTrajCallback(const quadrotor_msgs::PolyTrajPtr &msg)
{
    ROS_INFO("GET!!!!!!!!!!!");
    trajmsg = *msg;
    for(int i = 0;i < trajmsg.duration.size();i++)
    {
        time_all += trajmsg.duration[i];
    }
    cout << time_all << "iiiiiiiiiiii" << endl;
    
}

void visTransport(Vector4d Point)
{
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "odom";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "astar_node/transport";
    node_vis.type = visualization_msgs::Marker::MESH_RESOURCE;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.mesh_resource = "package://grid_path_searcher/pcd/transport.dae";
    node_vis.id = 0;

    node_vis.pose.position.x = Point(0);
    node_vis.pose.position.y = Point(1);
    node_vis.pose.position.z = Point(2);

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = 0.105*sin(Point(3)/180*M_PI)/sin(_max_angle*M_PI);
    node_vis.scale.y = 0.105*sin(Point(3)/180*M_PI)/sin(_max_angle*M_PI);
    node_vis.scale.z = 0.105*cos(Point(3)/180*M_PI)/cos(_max_angle*M_PI);

    _transport_vis_pub.publish(node_vis);
}

//发布Astar路径
void visGridPath( vector<Vector4d> nodes)
{   
    clearMarkers();
    cout << "_wayPointSplitNum:" << _wayPointSplitNum << endl;
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "odom";
    node_vis.header.stamp = ros::Time::now();

    node_vis.ns = "astar_node/astar_path";

    node_vis.type = visualization_msgs::Marker::LINE_STRIP;
    // node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

   
    node_vis.color.a = 1.0;
    node_vis.color.r = 97.0 / 255.0;
    node_vis.color.g = 100.0 / 255.0;
    node_vis.color.b = 159.0 / 255.0;
    
    node_vis.scale.x = 0.2;
    node_vis.scale.y = 0.2;
    node_vis.scale.z = 0.2;
    // node_vis.scale.x = _resolution;
    // node_vis.scale.y = _resolution;
    // node_vis.scale.z = _resolution;

    visualization_msgs::MarkerArray transport_vis_array;
    visualization_msgs::Marker transport_vis;

    geometry_msgs::Point pt;
    pt.x = _start_pt(0);
    pt.y = _start_pt(1);
    pt.z = _start_pt(2);
    cout<<"Start Position: \n"<<pt<<endl;
    node_vis.points.push_back(pt);

    Eigen::Matrix3Xd wayPoints;
    cout<<"size: "<<nodes.size()<<endl;
    wayPoints.resize(3,max(int(nodes.size())/_wayPointSplitNum+1,2));
    cout<<"wayPoint Num: "<<max(int(nodes.size())/_wayPointSplitNum+1,2)<<endl;
    wayPoints.col(0)(0) = _start_pt(0);
    wayPoints.col(0)(1) = _start_pt(1);
    wayPoints.col(0)(2) = _start_pt(2);

    transport_vis.header.frame_id = "odom";
    transport_vis.header.stamp = time_replan;
    transport_vis.type = visualization_msgs::Marker::MESH_RESOURCE;
    transport_vis.action = visualization_msgs::Marker::ADD;
    transport_vis.mesh_resource = "package://grid_path_searcher/pcd/transport.dae";
    transport_vis.ns = "start_pt";
    transport_vis.id = 0;

    transport_vis.pose.position.x = _start_pt(0);
    transport_vis.pose.position.y = _start_pt(1);
    transport_vis.pose.position.z = _start_pt(2);

    transport_vis.pose.orientation.x = 0.0;
    transport_vis.pose.orientation.y = 0.0;
    transport_vis.pose.orientation.z = - sqrt(2)/2;
    transport_vis.pose.orientation.w = sqrt(2)/2;

    transport_vis.color.a = 0.6;
    transport_vis.color.r = 236.0 / 255.0;
    transport_vis.color.g = 43.0 / 255.0;
    transport_vis.color.b = 36.0 / 255.0;

    transport_vis.scale.x = pow(1.6,1/3) * 2 * 0.105;
    transport_vis.scale.y = pow(1.6,1/3) * 2 * 0.105;
    transport_vis.scale.z = pow(1.6,1/3) * 1.6 * 0.105;

    transport_vis_array.markers.push_back(transport_vis);

    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector4d point = nodes[i];
        pt.x = point(0);
        pt.y = point(1);
        pt.z = point(2);

        if((i+1)%_wayPointSplitNum == 0 && abs(i+1 - int(nodes.size())) > (_wayPointSplitNum-1) )
        {
            wayPoints.col(2+i/_wayPointSplitNum)(0) = point(0);
            wayPoints.col(2+i/_wayPointSplitNum)(1) = point(1);
            wayPoints.col(2+i/_wayPointSplitNum)(2) = point(2);

            transport_vis.header.frame_id = "odom";
            transport_vis.header.stamp = time_replan;
            transport_vis.type = visualization_msgs::Marker::MESH_RESOURCE;
            transport_vis.action = visualization_msgs::Marker::ADD;
            transport_vis.mesh_resource = "package://grid_path_searcher/pcd/transport.dae";
            transport_vis.ns = "way_pts";
            transport_vis.id = i+1;

            transport_vis.pose.position.x = point(0);
            transport_vis.pose.position.y = point(1);
            transport_vis.pose.position.z = point(2);

            transport_vis.pose.orientation.x = 0.0;
            transport_vis.pose.orientation.y = 0.0;
            transport_vis.pose.orientation.z = - sqrt(2)/2;
            transport_vis.pose.orientation.w = sqrt(2)/2;

            transport_vis.color.a = 0.6;
            transport_vis.color.r = 236.0 / 255.0;
            transport_vis.color.g = 43.0 / 255.0;
            transport_vis.color.b = 36.0 / 255.0;

            if((i+1)/_wayPointSplitNum == 2 || (i+1)/_wayPointSplitNum == 3)
                point(3) = 33;
            if((i+1)/_wayPointSplitNum == 1 || (i+1)/_wayPointSplitNum == 4)
                point(3) = 45;    

            transport_vis.scale.x = pow(1.6,1/3) * 2 * 0.105*sin(point(3)/180*M_PI)/sin(_max_angle*M_PI);
            transport_vis.scale.y = pow(1.6,1/3) * 2 * 0.105*sin(point(3)/180*M_PI)/sin(_max_angle*M_PI);
            transport_vis.scale.z = pow(1.6,1/3) * 1.6 * 0.105*cos(point(3)/180*M_PI)/cos(_max_angle*M_PI);
            ROS_INFO("POINT3: %f", point(3));

            transport_vis_array.markers.push_back(transport_vis);
        }

        
        // cout<<"Position: \n"<<pt<<"Angle:"<<point(3)<<endl;
        node_vis.points.push_back(pt);
    }

    pt.x = _target_pt(0);
    pt.y = _target_pt(1);
    pt.z = _target_pt(2);

    wayPoints.col(1)(0) = _target_pt(0);
    wayPoints.col(1)(1) = _target_pt(1);
    wayPoints.col(1)(2) = _target_pt(2);

    cout<<"Target Position: \n"<<pt<<endl;
    node_vis.points.push_back(pt);

    transport_vis.header.frame_id = "odom";
    transport_vis.header.stamp = time_replan;
    transport_vis.type = visualization_msgs::Marker::MESH_RESOURCE;
    transport_vis.action = visualization_msgs::Marker::ADD;
    transport_vis.mesh_resource = "package://grid_path_searcher/pcd/transport.dae";
    transport_vis.ns = "goal_pt";
    transport_vis.id = int(nodes.size());

    transport_vis.pose.position.x = _target_pt(0);
    transport_vis.pose.position.y = _target_pt(1);
    transport_vis.pose.position.z = _target_pt(2);

    transport_vis.pose.orientation.x = 0.0;
    transport_vis.pose.orientation.y = 0.0;
    transport_vis.pose.orientation.z = - sqrt(2)/2;
    transport_vis.pose.orientation.w = sqrt(2)/2;

    transport_vis.color.a = 0.6;
    transport_vis.color.r = 236.0 / 255.0;
    transport_vis.color.g = 43.0 / 255.0;
    transport_vis.color.b = 36.0 / 255.0;

    transport_vis.scale.x = pow(1.6,1/3) * 2 * 0.105;
    transport_vis.scale.y = pow(1.6,1/3) * 2 * 0.105;
    transport_vis.scale.z = pow(1.6,1/3) * 1.6 * 0.105;

    // transport_vis_array.header.stamp = time_replan;
    transport_vis_array.markers.push_back(transport_vis);

    for(int i = 0;i < int(nodes.size())/_wayPointSplitNum+1;i++)
    {
        cout<<wayPoints.col(i)(0)<<"\t"<<wayPoints.col(i)(1)<<"\t"<<wayPoints.col(i)(2)<<endl;
    }

    _grid_path_vis_pub.publish(node_vis);
    _transport_vis_pub.publish(transport_vis_array);

    // _waypoints_pub.publish(wayPoints);
}

//发布OpenList/CloseList的方格
void visVisitedNode( vector<Vector3d> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "odom";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub.publish(node_vis);
}

void visSearchSize(Eigen::Vector3d _search_lower, Eigen::Vector3d _search_upper)
{   
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "odom";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "Search_Size";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.id = 0;
    line_strip.scale.x = 0.05;
    line_strip.color.r = 0.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;
    line_strip.color.a = 0.7;
    geometry_msgs::Point p;
    p.x = _search_lower(0);
    p.y = _search_lower(1);
    p.z = 0.0;
    p.z = _search_upper(2);
    line_strip.points.push_back(p);
    p.x = _search_lower(0);
    p.y = _search_upper(1);
    p.z = 0.0;
    p.z = _search_upper(2);
    line_strip.points.push_back(p);
    p.x = _search_upper(0);
    p.y = _search_upper(1);
    p.z = 0.0;
    p.z = _search_upper(2);
    line_strip.points.push_back(p);
    p.x = _search_upper(0);
    p.y = _search_lower(1);
    p.z = 0.0;
    p.z = _search_upper(2);
    line_strip.points.push_back(p);
    p.x = _search_lower(0);
    p.y = _search_lower(1);
    p.z = 0.0;
    p.z = _search_upper(2);
    line_strip.points.push_back(p);
    _line_strip_pub.publish(line_strip);
}


//路径查找，主要调用Astar的部分接口函数
void pathFinding(const Vector3d start_pt, const Vector3d target_pt)
{
    //调用A星寻路算法
    ros::Time begin = ros::Time::now();
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);
    ros::Time end = ros::Time::now();
    double duration = (end.toSec() - begin.toSec()) * 1000;
    ROS_INFO("[Astar Node] Astar Path Finding Finished, Duration: %lf ms",duration);

    //通过A星算法得到路径点集和close集合
    auto grid_path     = _astar_path_finder->getPath();
    auto visited_nodes = _astar_path_finder->getVisitedNodes();

    //可视化路径和close集合
    visGridPath(grid_path);
    visVisitedNode(visited_nodes);

    //重置Astar算法，方便下次掉用
    _astar_path_finder->resetUsedGrids();

}

//主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "astar_demo");
    ros::NodeHandle nh("~");

    _map_sub                      = nh.subscribe("map",         1,  rcvPointCloudCallBack,  ros::TransportHints().tcpNoDelay());
    _pts_sub                      = nh.subscribe("waypoints",   1,  rcvWaypointsCallback,   ros::TransportHints().tcpNoDelay());
    _trans_goal_sub               = nh.subscribe("/vrpn_client_node/Trans_goal/pose",  40, transGoalCallback,    ros::TransportHints().tcpNoDelay());
    _polyTrajSub                  = nh.subscribe("/load_planning/trajLoad", 100, polyTrajCallback, ros::TransportHints().tcpNoDelay());


    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("point_map", 1);
    _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("astar_path", 1);
    _visited_nodes_vis_pub        = nh.advertise<visualization_msgs::Marker>("visited_nodes",1);
    _start_point_vis_pub          = nh.advertise<visualization_msgs::Marker>("start_point",1);
    _target_point_vis_pub         = nh.advertise<visualization_msgs::Marker>("target_point",1);
    _transport_vis_pub            = nh.advertise<visualization_msgs::MarkerArray>("transport_vis",1000);
    _line_strip_pub               = nh.advertise<visualization_msgs::Marker>("search_size", 10);

    _replay_pt_pub                = nh.advertise<geometry_msgs::PoseStamped>("replay_pt", 10);

    // _replan_time_pub              = nh.advertise<visualization_msgs::Marker>("search_size", 10);

    // _waypoints_pub                = nh.advertise<Eigen::Matrix3Xd>("my_waypoints",1000);

    nh.param("map/cloud_margin",  _cloud_margin,        0.0);
    nh.param("map/resolution",    _resolution,          0.2);
    nh.param("map/x_size",        _x_size,              50.0);
    nh.param("map/y_size",        _y_size,              50.0);
    nh.param("map/z_size",        _z_size,              5.0);

    nh.param("map/isExp",         _isExp,                   false);
    nh.param("map/isReplanMode",  _isReplanMode,            false);
    nh.param("map/isReplanDemo1",  _isReplanDemo1,          false);
    nh.param("map/isReplanDemo2",  _isReplanDemo2,          false);
    nh.param("map/isNarrowSlitMode",  _isNarrowSlitMode,    false);

    nh.param("map/time_ahead",    _time_ahead,          0.240);

    nh.param("search/x_max",      _search_x_max,              10.0);
    nh.param("search/x_min",      _search_x_min,              -10.0);
    nh.param("search/y_max",      _search_y_max,              10.0);
    nh.param("search/y_min",      _search_y_min,              -10.0);
    nh.param("search/z_max",      _search_z_max,              5.0);
    nh.param("search/z_min",      _search_z_min,              0.0);
    
    nh.param("heuristic/distance",          _distance,          string("euclidean"));
    nh.param("weight/a",                    _weight_a,          1.0);
    nh.param("weight/b",                    _weight_b,          1.0);
    nh.param("weight/length",               _length,            1.2);
    nh.param("weight/angle",                _weight_angle,      1.0);
    nh.param("weight/max_angle",            _max_angle,         1.0/3.0);
    nh.param("weight/min_angle",            _min_angle,         1.0/6.0);
    nh.param("weight/angle_resulution",     _angle_resulution,  10);

    nh.param("wayPts/wayPointSplitNum",     _wayPointSplitNum,  3);

    nh.param("wayPts/targetAngle", _tar_ang, 0.0);

    //地图三轴最小和最大尺寸 世界坐标系(单位m)
    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;

    //搜索三轴最小和最大尺寸 世界坐标系(单位m)
    _search_lower << _search_x_min, _search_y_min, _search_z_min;
    _search_upper << _search_x_max, _search_y_max, _search_z_max;  
    
    //分辨率倒数
    _inv_resolution = 1.0 / _resolution;
    
    //栅格坐标系(与世界坐标系相差分辨率)下，整个点云地图的宽、长、高
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);
    
    //构建astar指针对象
    _astar_path_finder  = new AstarPathFinder(_distance,_weight_a,_weight_b,_weight_angle,_length,_max_angle,_min_angle,_angle_resulution);
    
    //初始化点云地图
    _astar_path_finder  -> initGridMap(_resolution, _map_lower, _map_upper, _search_lower, _search_upper, _max_x_id, _max_y_id, _max_z_id);

    ros::Rate rate(1000);
    bool status = ros::ok();

    while(status) 
    {
        visSearchSize(_search_lower, _search_upper);

        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }

    //删除astar指针对象
    delete _astar_path_finder;
    return 0;
}

