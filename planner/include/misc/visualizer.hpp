#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "gcopter/trajectory.hpp"

#include "misc/tinycolormap.hpp"
#include <std_msgs/ColorRGBA.h>

#include <iostream>
#include <memory>
#include <chrono>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
using namespace std;


// Visualizer for the planner
class Visualizer
{
private:
    // config contains the scale for some markers
    ros::NodeHandle nh;

    // These are publishers for path, waypoints on the trajectory,
    // the entire trajectory, the mesh of free-space polytopes,
    // the edge of free-space polytopes, and spheres for safety radius
    ros::Publisher routePub;
    ros::Publisher wayPointsPub;
    ros::Publisher trajectoryPub;
    ros::Publisher meshPub;
    ros::Publisher edgePub;
    ros::Publisher spherePub;
    ros::Publisher listsPub;
    ros::Publisher forceArrowPub;
    ros::Publisher droneListPub;
    ros::Publisher arrowsPub;
    ros::Publisher selectedPointPub;

    visualization_msgs::MarkerArray sphere_array;
    visualization_msgs::MarkerArray line_strip_array;
    visualization_msgs::MarkerArray force_list_array;
    visualization_msgs::MarkerArray drone_list_array;
    // std::vector<visualization_msgs::Marker> force_list;

public:
    ros::Publisher speedPub;
    ros::Publisher thrPub;
    ros::Publisher tiltPub;
    ros::Publisher bdrPub;

public:
    Visualizer(ros::NodeHandle &nh_)
        : nh(nh_)
    {
        routePub = nh.advertise<visualization_msgs::Marker>("/visualizer/route", 10);
        wayPointsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/waypoints", 10);
        trajectoryPub = nh.advertise<visualization_msgs::Marker>("/visualizer/trajectory", 10);
        meshPub = nh.advertise<visualization_msgs::Marker>("/visualizer/mesh", 1000);
        edgePub = nh.advertise<visualization_msgs::Marker>("/visualizer/edge", 1000);   
        spherePub = nh.advertise<visualization_msgs::Marker>("/visualizer/spheres", 1000);

        listsPub = nh.advertise<visualization_msgs::MarkerArray>("/visualizer/lists_array", 10000);
        
        forceArrowPub = nh.advertise<visualization_msgs::MarkerArray>("/visualizer/forceArrow", 10000);
        droneListPub = nh.advertise<visualization_msgs::MarkerArray>("/visualizer/droneListArrow", 10000);


        arrowsPub = nh.advertise<visualization_msgs::Marker>("/visualizer/arrows", 2); 
        selectedPointPub = nh.advertise<visualization_msgs::Marker>("/visualizer/selectedPoint", 1000); 
        speedPub = nh.advertise<std_msgs::Float64>("/visualizer/speed", 1000);
        thrPub = nh.advertise<std_msgs::Float64>("/visualizer/total_thrust", 1000);
        tiltPub = nh.advertise<std_msgs::Float64>("/visualizer/tilt_angle", 1000);
        bdrPub = nh.advertise<std_msgs::Float64>("/visualizer/body_rate", 1000);
        
    }

    // Visualize the trajectory and its front-end path
    void displayMarkerList(const std::vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,tinycolormap::ColormapType color_Load, bool show_sphere = false, bool show_line = true)
    {
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "odom";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id + 1000 + ros::Time::now().toSec();
        line_strip.id = id + rand() * 100 + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = color(0);
        sphere.color.g = line_strip.color.g = color(1);
        sphere.color.b = line_strip.color.b = color(2);
        sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
        sphere.scale.x = scale;
        sphere.scale.y = scale;
        sphere.scale.z = scale;
        line_strip.scale.x = scale / 2;
        geometry_msgs::Point pt;
        for (int i = 0; i < int(list.size()); i++)
        {
            pt.x = list[i](0);
            pt.y = list[i](1);
            // pt.z = list[i](2) + 0.1;
            pt.z = list[i](2);
            if (show_sphere)
            {
                sphere.points.push_back(pt);
            }
            if (show_line)
            {
                double value = (double(i) / double(list.size()));
                // if(value >= 0.225 && value <= 0.70)
                //     line_strip.points.push_back(pt);
                if(color(0)+color(1)+color(2)+color(3) == 0)
                {
                    double value = (double(i) / double(list.size()));
                    while(value > 1.0)
                    {
                        value -= 1.0;
                    }
                    const tinycolormap::Color color = tinycolormap::GetColor(value, color_Load);
                    std_msgs::ColorRGBA line_color;
                    line_color.r = color.r();
                    line_color.g = color.g();
                    line_color.b = color.b();
                    line_strip.points.push_back(pt);
                    line_strip.colors.push_back(line_color);
                }
            }
        }
        
        if (show_sphere)
        {
            sphere_array.markers.push_back(sphere);
            listsPub.publish(sphere_array);
            ros::Duration(0.001).sleep();
        }
        if (show_line)
        {
            line_strip_array.markers.push_back(line_strip);
            listsPub.publish(line_strip_array);
            ros::Duration(0.001).sleep();
        }
    }

    void displayForceArrow(const std::vector<Eigen::Vector3d> &posQList, const std::vector<Eigen::Vector3d> &forceList,
                           tinycolormap::ColormapType color_type, int id, Eigen::Vector4d color_S, double color_a)
    {
        std::vector<visualization_msgs::Marker> force_list;
        force_list.resize(int(posQList.size()));


        // 设置箭头的起点和终点

        for (int i = 0; i < int(posQList.size()); i++)
        {
            force_list[i].header.frame_id = "odom";
            force_list[i].header.stamp = ros::Time::now();
            force_list[i].ns = "force" + to_string(id);
            force_list[i].id = id;

            force_list[i].type = visualization_msgs::Marker::LINE_LIST;
            force_list[i].action = visualization_msgs::Marker::ADD;

            force_list[i].scale.x = 0.03;  // 箭头粗细

            geometry_msgs::Point pt;
            pt.x = posQList[i](0);
            pt.y = posQList[i](1);
            pt.z = posQList[i](2);

            force_list[i].points.push_back(pt);  // 起点

            pt.x = forceList[i](0);
            pt.y = forceList[i](1);
            pt.z = forceList[i](2);

            force_list[i].points.push_back(pt);  // 终点

            double value = ((double(i)/ double(posQList.size())));
            while(value > 1.0)
            {
                value -= 1.0;
            }
            const tinycolormap::Color color = tinycolormap::GetColor(value, color_type);
            const double cr = color.r() * 255.0;
            const double cg = color.g() * 255.0;
            const double cb = color.b() * 255.0;

            force_list[i].color.r = cr / 255.0; 
            force_list[i].color.g = cg / 255.0;
            force_list[i].color.b = cb / 255.0;
            

            force_list[i].color.a = color_a;  // 不透明
            if(i % 6 == 0)
                force_list_array.markers.push_back(force_list[i]);
        }
        forceArrowPub.publish(force_list_array);
        ros::Duration(0.001).sleep();
    }

    void displayDroneArray(const std::vector<Eigen::Vector3d> &posQList, const std::vector<Eigen::Vector4d> &quat, tinycolormap::ColormapType color_type, int id, Eigen::Vector4d color_S, double color_a)
    {
        std::vector<visualization_msgs::Marker> drone_list;
        drone_list.resize(int(posQList.size()));
        for (int i = 0; i < int(posQList.size()); i++)
        {
            drone_list[i].header.frame_id = "odom";
            drone_list[i].header.stamp = ros::Time::now();
            drone_list[i].ns = "mesh" + to_string(id);
            drone_list[i].id = id + rand() * 1000 + 1000 + i;
            drone_list[i].type = visualization_msgs::Marker::MESH_RESOURCE;
            drone_list[i].action = visualization_msgs::Marker::ADD;

            drone_list[i].pose.position.x = posQList[i](0);
            drone_list[i].pose.position.y = posQList[i](1);
            drone_list[i].pose.position.z = posQList[i](2);
            double x = quat[i](1);
            double y = quat[i](2);
            double z = quat[i](3);
            double w = quat[i](0);
            double sinr_cosp = +2.0 * (w * x + y * z);
            double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
            double roll = atan2(sinr_cosp, cosr_cosp);
            double sinp = +2.0 * (w * y - z * x);
            double pitch;
            if (fabs(sinp) >= 1)
                pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
            else
                pitch = asin(sinp);
            double siny_cosp = +2.0 * (w * z + x * y);
            double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
            double yaw = atan2(siny_cosp, cosy_cosp);

            drone_list[i].pose.orientation.w = quat[i](0);
            drone_list[i].pose.orientation.x = quat[i](1);
            drone_list[i].pose.orientation.y = quat[i](2);
            drone_list[i].pose.orientation.z = quat[i](3);
            drone_list[i].scale.x = 0.5;
            drone_list[i].scale.y = 0.5;
            drone_list[i].scale.z = 0.5;
            if(roll < 0)
                roll = -roll;
            if(pitch < 0)
                pitch = -pitch;

            double value = ((double(i)/ double(posQList.size())));
            while(value > 1.0)
            {
                value -= 1.0;
            }

            const tinycolormap::Color color = tinycolormap::GetColor(value, color_type);
            const double color_r = color.r() * 255.0;
            const double color_g = color.g() * 255.0;
            const double color_b = color.b() * 255.0;
            drone_list[i].color.a = color_a;

            drone_list[i].color.r = color_r / 255.0; 
            drone_list[i].color.g = color_g / 255.0;
            drone_list[i].color.b = color_b / 255.0;
            drone_list[i].mesh_resource = std::string("package://odom_visualization/meshes/fake_drone.dae");
            if(i % 2 == 0)
                drone_list_array.markers.push_back(drone_list[i]);
        }
        droneListPub.publish(drone_list_array);
        ros::Duration(0.001).sleep();

    }
    

    inline void displayVectorArrow(const vector<Eigen::Vector3d> &list, Eigen::Vector4d color, const Eigen::Vector3d xyz, double scale, int id)
    {
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "odom";
        arrow.header.stamp = ros::Time::now();
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;

        arrow.color.r = color(0);
        arrow.color.g = color(1);
        arrow.color.b = color(2);
        arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
        arrow.scale.x = xyz(0) * scale;
        arrow.scale.y = xyz(1) * scale;
        arrow.scale.z = xyz(2) * scale;
        arrow.pose.orientation.x = 0.0;
        arrow.pose.orientation.y = 0.0;
        arrow.pose.orientation.z = 0.0;
        arrow.pose.orientation.w = 1.0;

        geometry_msgs::Point start, end;
        for (int i = 0; i < int(list.size() / 2); i++)
        {
            start.x = list[2 * i](0);
            start.y = list[2 * i](1);
            start.z = list[2 * i](2);
            end.x = list[2 * i + 1](0);
            end.y = list[2 * i + 1](1);
            end.z = list[2 * i + 1](2);
            arrow.points.clear();
            arrow.points.push_back(start);
            arrow.points.push_back(end);
            arrow.id = i + id;

            arrowsPub.publish(arrow);
            ros::Duration(0.001).sleep();
        }
        return ;
    }

    inline void displaySelectedPos(Eigen::Vector3d pos, Eigen::Vector4d color, const double scale, int id)
    {
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "odom";
        sphere.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.id = id;

        sphere.pose.orientation.w = 1.0;
        sphere.color.r = color(0);
        sphere.color.g = color(1);
        sphere.color.b = color(2);
        sphere.color.a = color(3);
        sphere.scale.x = scale;
        sphere.scale.y = scale;
        sphere.scale.z = scale;
        sphere.pose.position.x = pos(0);
        sphere.pose.position.y = pos(1);
        sphere.pose.position.z = pos(2);

        selectedPointPub.publish(sphere);
        ros::Duration(0.001).sleep();
    }

    // Visualize all spheres with centers sphs and the same radius
    inline void visualizeSphere(const Eigen::Vector3d &center,
                                const double &radius)
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 0.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 1.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETE;

        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        spherePub.publish(sphereDeleter);
        spherePub.publish(sphereMarkers);
    }

    inline void visualizeStartGoal(const Eigen::Vector3d &center,
                                   const double &radius,
                                   const int sg)
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = sg;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "StartGoal";
        sphereMarkers.color.r = 1.00;
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 0.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = radius * 2.0;
        sphereMarkers.scale.y = radius * 2.0;
        sphereMarkers.scale.z = radius * 2.0;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETEALL;

        geometry_msgs::Point point;
        point.x = center(0);
        point.y = center(1);
        point.z = center(2);
        sphereMarkers.points.push_back(point);

        if (sg == 0)
        {
            spherePub.publish(sphereDeleter);
            ros::Duration(1.0e-9).sleep();
            sphereMarkers.header.stamp = ros::Time::now();
        }
        spherePub.publish(sphereMarkers);
    }
};

#endif