#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"
#include "gcopter/display.hpp"
#include "gcopter/maneuver.hpp"
#include "plan_env/grid_map.h"
#include "quadrotor_msgs/PolyTraj.h"

#include "misc/tinycolormap.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

struct Config
{
    std::string mapTopic;
    std::string targetTopic;
    std::string targetwayPtsTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxHeightQ;
    double minHeightL;
    double maxVelMagL;
    double maxVelMagQ;
    double maxAccMagL;
    double maxBdrMag;
    double maxTiltAngle;
    double maxTheta;
    double maxPsi;
    double minTheta;
    double minPsi;
    double maxT;
    double minT;
    double minThrust;
    double maxThrust;
    double minDistance;
    double clearence;
    double cableLength;
    double massL;
    double massQ;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    double weightE;
    double disBetweenWaypoints;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    int cableSampleNums;
    double relCostTol;

    int pieceNum;
    int droneNum;
    double percent;
    bool isWayPointsRemovable;
    bool showAll;
    double time_ahead;
    std::vector<double> inifinAngT;
    std::vector<double> initLoadPos;
    std::vector<double> endLoadPos;
    std::vector<double> continuityDescend;
    std::vector<double> boundaryConditionSlack;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("TargetwayPtsTopic", targetwayPtsTopic);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
        nh_priv.getParam("TimeoutRRT", timeoutRRT);
        nh_priv.getParam("Percent", percent);
        nh_priv.getParam("MaxHeightQ", maxHeightQ);
        nh_priv.getParam("MinHeightL", minHeightL);
        nh_priv.getParam("MaxVelMagL", maxVelMagL);
        nh_priv.getParam("MaxVelMagQ", maxVelMagQ);
        nh_priv.getParam("MaxAccMagL", maxAccMagL);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        nh_priv.getParam("MinTheta", minTheta);
        nh_priv.getParam("MaxTheta", maxTheta);
        nh_priv.getParam("MinPsi", minPsi);
        nh_priv.getParam("MaxPsi", maxPsi);
        nh_priv.getParam("MinT", minT);
        nh_priv.getParam("MaxT", maxT);
        nh_priv.getParam("MinThrust", minThrust);
        nh_priv.getParam("MaxThrust", maxThrust);
        nh_priv.getParam("MinDistance", minDistance);
        nh_priv.getParam("Clearence", clearence);
        nh_priv.getParam("CableLength", cableLength);
        nh_priv.getParam("MassL", massL);
        nh_priv.getParam("MassQ", massQ);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("HorizDrag", horizDrag);
        nh_priv.getParam("VertDrag", vertDrag);
        nh_priv.getParam("ParasDrag", parasDrag);
        nh_priv.getParam("SpeedEps", speedEps);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("WeightE", weightE);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("IsWayPointsRemovable", isWayPointsRemovable);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("CableSampleNums", cableSampleNums);
        nh_priv.getParam("RelCostTol", relCostTol);
        nh_priv.getParam("PieceNum", pieceNum);
        nh_priv.getParam("DroneNum", droneNum);
        nh_priv.getParam("InifinAngT", inifinAngT);
        nh_priv.getParam("InitLoadPos", initLoadPos);
        nh_priv.getParam("EndLoadPos", endLoadPos);
        nh_priv.getParam("DisBetweenWaypoints", disBetweenWaypoints);
        nh_priv.getParam("ContinuityDescend", continuityDescend);
        nh_priv.getParam("BoundaryConditionSlack", boundaryConditionSlack);
        nh_priv.getParam("ShowAll", showAll);
        nh_priv.getParam("time_ahead", time_ahead);
        // cout << "aaaaaaaaaaaaaaaaaaaaa \t " <<  isWayPointsRemovable << endl;

    }
};

class GlobalPlanner
{
private:
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber targetSub;
    ros::Subscriber wayPtsSub;
    std::vector<ros::Publisher>  odom_pubs;
    std::vector<ros::Publisher>  line_pubs;
    std::vector<double> traj_all;
    ros::Publisher odom_list_pub;
    std::vector<ros::Publisher>  force_arrow_pubs;
    std::vector<ros::Publisher>  force_arrow_show_pubs;

    std::vector<int> odom_flag;
    std::vector<double> odom_x_before;
    std::vector<double> odom_y_before;
    std::vector<double> odom_z_before;

    

    std::vector<visualization_msgs::MarkerArray> force_list_array;
    std::vector<visualization_msgs::MarkerArray> force_show_list_array;

    bool mapInitialized;
    bool isSuccessOpted = false;
    bool isReplan = false;
    bool replanDone = false;
    bool replan_Done = false;
    int replan_time = 0;
    voxel_map::VoxelMap voxelMap;
    Visualizer visualizer;
    Display display;
    std::vector<Eigen::Vector3d> startGoal;
    Eigen::Matrix3Xd wayPts, wayVels, wayAccs;
    Eigen::VectorXi velNums, accNums;

    std::vector<Trajectory<7>> optTrajs;
    std::vector<Trajectory<7>> initTrajs;

    double trajStamp;
    double replan_duration;
    double whole_traj_time;

    double scale;   
    int isshowtime[7];
    int isforcetime[4];
    std::vector<nav_msgs::Odometry>  odom_show;
    GridMap::Ptr grid_map_;
    std::vector<ros::Publisher> polyTrajPub;
    std::vector<quadrotor_msgs::PolyTraj> trajMsgs;

public:
    GlobalPlanner(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          mapInitialized(false),
          visualizer(nh)
    {
        trajMsgs.resize(config.droneNum + 1);
        polyTrajPub.resize(config.droneNum + 1);
        polyTrajPub[0] = nh.advertise<quadrotor_msgs::PolyTraj>("load_planning/trajLoad", 10);
        force_list_array.resize(config.droneNum);
        force_show_list_array.resize(config.droneNum);
        traj_all.resize(config.droneNum + 1);

        odom_show.resize(config.droneNum + 1);

        odom_flag.resize(config.droneNum + 1);
        for(int flag = 0;flag <= config.droneNum; flag++)
        {
            odom_flag[flag] = 0;
            traj_all[flag] = 0.0;
        }
        odom_x_before.resize(config.droneNum + 1);
        odom_y_before.resize(config.droneNum + 1);
        odom_z_before.resize(config.droneNum + 1);

        for (int i = 0; i < config.droneNum; i++)
        {
            polyTrajPub[i + 1] = nh.advertise<quadrotor_msgs::PolyTraj>("drone_" + to_string(i + 1) + "_planning/trajCable", 10);
        }
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        grid_map_.reset(new GridMap);

        grid_map_->initMap();

        targetSub = nh.subscribe(config.targetTopic, 1, &GlobalPlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
        wayPtsSub = nh.subscribe(config.targetwayPtsTopic, 1, &GlobalPlanner::wayPointsCallBack, this,
                                 ros::TransportHints().tcpNoDelay());
    }

    inline void plan()
    {
        if (startGoal.size() == 2)
        {
            std::vector<Eigen::Vector3d> route;
            if (true)
            {

                Eigen::Matrix<double, 3, 4> iniStateL, finStateL;
                Eigen::Matrix<double, 3, 4> inifinStateQ;
                Eigen::Vector3d angT;
                angT(0) = config.inifinAngT[0] * M_PI; 
                angT(1) = config.inifinAngT[1] * M_PI;
                angT(2) = config.inifinAngT[2];  
                iniStateL << startGoal.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                finStateL << startGoal.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                inifinStateQ  << angT, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                Eigen::Matrix<double, 3, 4> initStateQ_1;
                Eigen::Matrix<double, 3, 4> initStateQ_2;
                Eigen::Matrix<double, 3, 4> initStateQ_3;
                initStateQ_1 << Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                initStateQ_2 << Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
                initStateQ_3 << Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

                if(isReplan)
                {
                    ROS_INFO("22222222222222222221111111111111111111111");
                    Eigen::Vector3d omg, dAngT, d2AngT, d3AngT, q, dq, d2q, d3q;
                    Eigen::Vector3d posL, velL, accL, jerL, posQ, velQ, accQ, jerQ;

                    posL = optTrajs[0].getPos(replan_duration);
                    velL = optTrajs[0].getVel(replan_duration);
                    accL = optTrajs[0].getAcc(replan_duration);
                    jerL = optTrajs[0].getJer(replan_duration);
                    cout << replan_duration << endl;
                    cout << posL << endl;

                    iniStateL << posL, velL, accL, jerL;

                    angT   = optTrajs[0 + 1].getPos(replan_duration);
                    dAngT  = optTrajs[0 + 1].getVel(replan_duration);
                    d2AngT = optTrajs[0 + 1].getAcc(replan_duration);
                    d3AngT = optTrajs[0 + 1].getJer(replan_duration);

                    initStateQ_1 << angT, dAngT, d2AngT, d3AngT;

                    angT   = optTrajs[1 + 1].getPos(replan_duration);
                    dAngT  = optTrajs[1 + 1].getVel(replan_duration);
                    d2AngT = optTrajs[1 + 1].getAcc(replan_duration);
                    d3AngT = optTrajs[1 + 1].getJer(replan_duration);

                    initStateQ_2 << angT, dAngT, d2AngT, d3AngT;

                    angT   = optTrajs[2 + 1].getPos(replan_duration);
                    dAngT  = optTrajs[2 + 1].getVel(replan_duration);
                    d2AngT = optTrajs[2 + 1].getAcc(replan_duration);
                    d3AngT = optTrajs[2 + 1].getJer(replan_duration);

                    initStateQ_3 << angT, dAngT, d2AngT, d3AngT;

                    replanDone = true;
                    replan_time++;
                }

                gcopter::GCOPTER_PolytopeSFC gcopter;

                static Eigen::VectorXd magnitudeBounds(12);
                static Eigen::VectorXd angTBounds(6);
                static Eigen::VectorXd penaltyWeights(7);
                static Eigen::VectorXd physicalParams(8);
                static Eigen::Vector3d continuityDescend;
                static Eigen::Vector3d boundaryConditionSlack;
                static bool getParam = false;
                if(!getParam)
                {
                    ROS_INFO("i am here !!!!!!!!!!!!!!!!!!!!!");
                    ros::Time start_time1 = ros::Time::now();
                    continuityDescend(0) = config.continuityDescend[0];
                    continuityDescend(1) = config.continuityDescend[1];
                    continuityDescend(2) = config.continuityDescend[2];
                    boundaryConditionSlack(0) = config.boundaryConditionSlack[0];
                    boundaryConditionSlack(1) = config.boundaryConditionSlack[1];
                    boundaryConditionSlack(2) = config.boundaryConditionSlack[2];
                    magnitudeBounds(0) = config.maxVelMagQ;
                    magnitudeBounds(1) = config.maxBdrMag;
                    magnitudeBounds(2) = config.maxTiltAngle;
                    magnitudeBounds(3) = config.minThrust;
                    magnitudeBounds(4) = config.maxThrust;
                    magnitudeBounds(5) = config.maxVelMagL;
                    magnitudeBounds(6) = config.maxAccMagL;
                    magnitudeBounds(7) = config.minDistance;
                    magnitudeBounds(8) = config.disBetweenWaypoints;
                    magnitudeBounds(9) = config.clearence;
                    magnitudeBounds(10) = config.minHeightL;
                    magnitudeBounds(11) = config.maxHeightQ;

                    angTBounds(0) = config.maxTheta * M_PI;
                    angTBounds(1) = config.minTheta * M_PI;
                    angTBounds(2) = M_PI / config.droneNum;
                    angTBounds(3) = -M_PI / config.droneNum;
                    // angTBounds(2) = M_PI * 1.0;
                    // angTBounds(3) = -M_PI * 1.0;
                    angTBounds(4) = config.maxT;
                    angTBounds(5) = config.minT;
                    penaltyWeights(0) = (config.chiVec)[0];
                    penaltyWeights(1) = (config.chiVec)[1];
                    penaltyWeights(2) = (config.chiVec)[2];
                    penaltyWeights(3) = (config.chiVec)[3];
                    penaltyWeights(4) = (config.chiVec)[4];
                    penaltyWeights(5) = (config.chiVec)[5];
                    penaltyWeights(6) = (config.chiVec)[6];
                    physicalParams(0) = config.massQ;
                    physicalParams(1) = config.gravAcc;
                    physicalParams(2) = config.horizDrag;
                    physicalParams(3) = config.vertDrag;
                    physicalParams(4) = config.parasDrag;
                    physicalParams(5) = config.speedEps;
                    physicalParams(6) = config.cableLength;
                    physicalParams(7) = config.massL;

                    scale = physicalParams(1) * physicalParams(7) / (config.droneNum * cos(config.inifinAngT[0] * M_PI));
                    ros::Time end_time1 = ros::Time::now();
                    ros::Duration duration1 = end_time1 - start_time1;
                    ROS_INFO("Get Param Time Cost: %f", duration1.toSec());

                    getParam = true;
                }
                
                optTrajs.clear();
                optTrajs.resize(config.droneNum + 1);
                initTrajs.clear();
                initTrajs.resize(config.droneNum + 1);
                cout << "AAAAAAAAAAAAAAAAAAA\t"  << config.droneNum << endl;
                display.reset(physicalParams, angTBounds, magnitudeBounds, config.droneNum, scale, config.integralIntervs);

                if (!gcopter.setup(config.weightT,
                                   config.weightE,
                                   iniStateL, finStateL,
                                   inifinStateQ,
                                   initStateQ_1,
                                   initStateQ_2,
                                   initStateQ_3,
                                   isReplan,
                                   wayPts,
                                   config.pieceNum,
                                   config.droneNum,
                                   INFINITY,
                                   config.smoothingEps,
                                   config.isWayPointsRemovable,
                                   scale,
                                   config.integralIntervs,
                                   config.cableSampleNums,
                                   magnitudeBounds,
                                   angTBounds,
                                   penaltyWeights,
                                   physicalParams,
                                   continuityDescend,
                                   boundaryConditionSlack,
                                   grid_map_))
                    return;
                {
                }
                ros::Time start_time = ros::Time::now();
                if (std::isinf(gcopter.optimize(optTrajs, initTrajs, config.relCostTol)))
                {
                    return;
                }
                ros::Time end_time = ros::Time::now();
                ros::Duration duration = end_time - start_time;
                ROS_INFO("Elapsed time: %f seconds", duration.toSec());

                isSuccessOpted = true;

                if (initTrajs[0].getPieceNum() > 0 && optTrajs[0].getPieceNum() > 0)
                {
                    int collisionNum = checkCollision(optTrajs);
                    cout << "ratio\t" << optTrajs[0].getTotalDuration() / initTrajs[0].getTotalDuration() << endl;
                    cout << "init time\t" << initTrajs[0].getTotalDuration() << "init time\t" << optTrajs[0].getTotalDuration() << endl;
                    cout << "Check Collision Nums\t"  << collisionNum << endl;
                    
                    if(isReplan)
                    {
                        display.displayTrajs(visualizer, optTrajs, config.percent, Eigen::Vector4d(0.0, 0.0, 0.0, 0.0 * 255)/255, 0.20, 0, tinycolormap::ColormapType::Heat);
                        display.displayDrone(visualizer, initTrajs, optTrajs, tinycolormap::ColormapType::Heat, 1.0);
                    }
                    else
                    {
                        display.displayTrajs(visualizer, optTrajs, config.percent, Eigen::Vector4d(0.0, 0.0, 0.0, 0.0 * 255)/255, 0.20, 0, tinycolormap::ColormapType::Gray);
                        display.displayDrone(visualizer, initTrajs, optTrajs, tinycolormap::ColormapType::Gray, 0.3);
                    }
                    // display.displayTrajs(visualizer, optTrajs, config.percent, Eigen::Vector4d(229.0, 183.0, 81.0, 1.0 * 255)/255, 0.2, 0, tinycolormap::ColormapType::Heat);

                    if(config.showAll)
                    {
                        // display.displayTrajs(visualizer, optTrajs, config.percent, Eigen::Vector4d(0.0, 1.0, 0.0, 1.0), 0.1, 1);
                        display.displayDetails(visualizer, initTrajs, optTrajs, gcopter.getVelQs(), gcopter.getOmgQs(), gcopter.getAttQs(), gcopter.getTQs());
                        display.displayAllows(visualizer, optTrajs, config.percent);
                        display.displayBounds(visualizer, optTrajs, config.percent);
                    }

                    trajStamp = ros::Time::now().toSec();

                    whole_traj_time = optTrajs[0].getTotalDuration();

                    polyTraj2ROSMsg(trajMsgs);

                    if(collisionNum == 0)
                    {
                        ROS_INFO("No Collision! Traj is Published!");
                        for (int i = 0; i < config.droneNum + 1; i++)
                        {
                            polyTrajPub[i].publish(trajMsgs[i]);
                        }
                    }
                    else
                    {
                        ROS_WARN("Traj has Collisions! Pub is Stopped!");
                    }

                    // open when is replan mode
                    isReplan = true;

                    // visualizer.tests();
                }
            }
        }
    }

    inline int checkCollision(std::vector<Trajectory<7>> &optTrajs)
    {
        int pieceNum = optTrajs[0].getPieceNum();
        int collisionNum = 0;
        int pieceCheckNum = 2 * config.integralIntervs;
        int cableCheckNum = 2 * config.cableSampleNums;
        Eigen::VectorXd times = optTrajs[0].getDurations();
        Eigen::Vector3d pos, posQ, angT, qs;
        Eigen::Vector2d cst, csp;
        Eigen::Vector4d gs;
        double step, t = 0.0, s;
        double sampleInterval, sampleLen;
        double dist;
        for (int i = 0; i < pieceNum; i++)
        {
            step = times(i) / pieceCheckNum;
            for (int j = 0; j < pieceCheckNum; j++)
            {
                s = t + j * step;
                pos = optTrajs[0].getPos(s);
                grid_map_->evaluateEDT(pos, dist);
                if (dist < 0)
                {
                    collisionNum += 1;
                }
                for (int k = 0; k < config.droneNum; k++)
                {
                    angT = optTrajs[k + 1].getPos(s);
                    angT(1) += 2 * k * M_PI / config.droneNum;
                    cst = getcs(angT(0));
                    csp = getcs(angT(1));
                    qs = getQ(cst, csp, gs);
                }

                for (int k = 0; k < config.droneNum; k++)
                {
                    for (int sn = 0; sn < cableCheckNum; sn++)
                    {
                        sampleInterval = config.cableLength / cableCheckNum;
                        sampleLen = (sn + 1) * sampleInterval;
                        posQ = pos + sampleLen * qs;
                        grid_map_->evaluateEDT(posQ, dist);
                        if (dist < 0)
                        {
                            collisionNum += 1;
                        }
                    }
                }
            }
            t += times(i);
        }
        return collisionNum;
    }

    inline void wayPointsCallBack(const visualization_msgs::MarkerArrayConstPtr & msg)
    {
        int size = msg->markers.size();
        cout<<"size: "<<size<<endl;
        Eigen::Matrix3Xd &pts = wayPts;
        int wayPtsNum = 0;
        startGoal.resize(0);
        pts.resize(3,max(size-2,1));

        if(!isReplan)
        {
            for (const auto& marker : msg->markers)
            {
                // ROS_INFO("!!!!!!!!!!!!!!!!!!! marker.header.stamp, %f", marker.header.stamp.toSec());
                replan_duration = marker.header.stamp.toSec() - trajStamp;
                // replan_duration = 0;
                if(marker.action == visualization_msgs::Marker::ADD)
                {
                    // 访问 Marker 的各个字段
                    double point_x = marker.pose.position.x;
                    double point_y = marker.pose.position.y;
                    double point_z = marker.pose.position.z;

                    cout<<"namespace: "<<marker.ns<<endl;

                    if(marker.ns == "start_pt")
                    {
                        const Eigen::Vector3d start(point_x, point_y, point_z);
                        visualizer.visualizeStartGoal(start, 0.1, startGoal.size());
                        startGoal.emplace_back(start);
                    }
                    else if(marker.ns == "goal_pt")
                    {
                        const Eigen::Vector3d goal(point_x, point_y, point_z);
                        visualizer.visualizeStartGoal(goal, 0.1, startGoal.size());
                        startGoal.emplace_back(goal);
                    }
                    else if(marker.ns == "way_pts")
                    {
                        pts.col(wayPtsNum)(0) = point_x;
                        pts.col(wayPtsNum)(1) = point_y;
                        pts.col(wayPtsNum)(2) = point_z;
                        wayPtsNum++;
                    }

                    ROS_INFO("Marker position: (%.2f, %.2f, %.2f)", 
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
                }
            }
            cout<<setiosflags(ios::fixed)<<setprecision(15)<<setiosflags(ios::left);
            cout<<"Time to plan !"<<endl;
            plan();
            return;
        }
        else
        {
            for (const auto& marker : msg->markers)
            {
                ROS_INFO("11111111111111111, %f",marker.header.stamp.toSec());
                replan_duration = marker.header.stamp.toSec() - trajStamp + config.time_ahead;
                // replan_duration = whole_traj_time * 0.40;
                if(replan_duration >= whole_traj_time)
                    replan_duration = whole_traj_time;
                if(marker.action == visualization_msgs::Marker::ADD)
                {
                    // 访问 Marker 的各个字段
                    double point_x = marker.pose.position.x;
                    double point_y = marker.pose.position.y;
                    double point_z = marker.pose.position.z;

                    cout<<"namespace: "<<marker.ns<<endl;

                    if(marker.ns == "start_pt")
                    {
                        const Eigen::Vector3d start(point_x, point_y, point_z);
                        visualizer.visualizeStartGoal(start, 0.1, startGoal.size());
                        startGoal.emplace_back(start);
                    }
                    else if(marker.ns == "goal_pt")
                    {
                        const Eigen::Vector3d goal(point_x, point_y, point_z);
                        visualizer.visualizeStartGoal(goal, 0.1, startGoal.size());
                        startGoal.emplace_back(goal);
                    }
                    else if(marker.ns == "way_pts")
                    {
                        pts.col(wayPtsNum)(0) = point_x;
                        pts.col(wayPtsNum)(1) = point_y;
                        pts.col(wayPtsNum)(2) = point_z;
                        wayPtsNum++;
                    }

                    ROS_INFO("Marker position: (%.2f, %.2f, %.2f)", 
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
                }
            }
            cout<<setiosflags(ios::fixed)<<setprecision(15)<<setiosflags(ios::left);
            cout<<"Time to plan !"<<endl;
            plan();
            return;
        }
    }


    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        // cout <<"BBBBB" << endl;
        // if (mapInitialized)
        // {
            // cout <<"CCCCCC" << endl;
            // if (startGoal.size() >= 2)
            // {
            //     startGoal.clear();
            // }
            const Eigen::Vector3d start(config.initLoadPos[0], config.initLoadPos[1], config.initLoadPos[2]);
            const Eigen::Vector3d goal(config.endLoadPos[0], config.endLoadPos[1], config.endLoadPos[2]);
            // const Eigen::Vector3d goal(config.initLoadPos[0], config.initLoadPos[1], config.initLoadPos[2]);
            // if (grid_map_->getOccupancy(start) == 0 && grid_map_->getOccupancy(goal) == 0)
            // {
                visualizer.visualizeStartGoal(start, 0.1, startGoal.size());
                startGoal.emplace_back(start);
                visualizer.visualizeStartGoal(goal, 0.1, startGoal.size());
                startGoal.emplace_back(goal);
            // }
            // else
            // {
            //     ROS_WARN("Infeasible Position Selected !!!\n");
            // }

            setInitLoadWayPts();
            cout<<setiosflags(ios::fixed)<<setprecision(7)<<setiosflags(ios::left);
            plan();
        // }
        return;
    }

    inline void process()
    {
        Eigen::VectorXd physicalParams(8);
        physicalParams(0) = config.massQ;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;
        physicalParams(6) = config.cableLength;
        physicalParams(7) = config.massL;

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(6));
        if (isSuccessOpted)
        {
            // cout <<"nnnnnnnnnnn" << endl;
            odom_pubs.resize(config.droneNum + 1);
            line_pubs.resize(config.droneNum);
            force_arrow_pubs.resize(config.droneNum);
            force_arrow_show_pubs.resize(config.droneNum);
            odom_list_pub = nh.advertise<visualization_msgs::Marker>("/drone_odom_list", 10);

            for (int k = 0; k < config.droneNum + 1; k++)
            {
                odom_pubs[k] = nh.advertise<nav_msgs::Odometry>("drone_" + to_string(k) + "_odom", 1);
                if(k!=0)
                {
                    line_pubs[k-1] = nh.advertise<visualization_msgs::Marker>("/drone_" + to_string(k) + "_line", 10);
                    force_arrow_pubs[k-1] = nh.advertise<visualization_msgs::MarkerArray>("/drone_" + to_string(k) + "_force", 1000);
                    force_arrow_show_pubs[k-1] = nh.advertise<visualization_msgs::MarkerArray>("/drone_" + to_string(k) + "_force_show", 100);
                }

            }
            ros::Time now_time = ros::Time::now();
            const double delta = now_time.toSec() - trajStamp;

            
            if (delta > 0.0 && delta <= optTrajs[0].getTotalDuration())
            {
                double thr;
                double len = config.cableLength;
                Eigen::Vector4d quat, g;
                Eigen::Vector3d omg, angT, dAngT, d2AngT, d3AngT, q, dq, d2q, d3q, force;
                Eigen::Vector3d posL, velL, accL, jerL, posQ, velQ, accQ, jerQ;
                Eigen::Vector2d cst, csp;
                std::vector<nav_msgs::Odometry>  odoms;
                visualization_msgs::Marker odom_list;
                std::vector<visualization_msgs::Marker> line_list;
                std::vector<visualization_msgs::Marker> force_list;
                std::vector<visualization_msgs::Marker> force_show_list;
                posL = optTrajs[0].getPos(delta);
                velL = optTrajs[0].getVel(delta);
                accL = optTrajs[0].getAcc(delta);
                jerL = optTrajs[0].getJer(delta);
                odoms.resize(config.droneNum + 1);

                line_list.resize(config.droneNum);
                force_list.resize(config.droneNum);
                force_show_list.resize(config.droneNum);
                odoms[0].header.stamp = now_time; 
                odoms[0].pose.pose.position.x = posL(0);
                odoms[0].pose.pose.position.y = posL(1);
                odoms[0].pose.pose.position.z = posL(2);
                odoms[0].pose.pose.orientation.w = 1.0;
                odoms[0].pose.pose.orientation.x = 0.0;
                odoms[0].pose.pose.orientation.y = 0.0;
                odoms[0].pose.pose.orientation.z = 0.0;
                odoms[0].twist.twist.linear.x = velL(0);
                odoms[0].twist.twist.linear.y = velL(1);
                odoms[0].twist.twist.linear.z = velL(2);
                odoms[0].twist.twist.angular.x = 0.0;
                odoms[0].twist.twist.angular.y = 0.0;
                odoms[0].twist.twist.angular.z = 0.0;
                for (int k = 0; k < config.droneNum; k++)
                {
                    angT = optTrajs[k + 1].getPos(delta);
                    angT(1) += 2 * k * M_PI / config.droneNum;
                    dAngT = optTrajs[k + 1].getVel(delta);
                    d2AngT = optTrajs[k + 1].getAcc(delta);
                    d3AngT = optTrajs[k + 1].getJer(delta);
                    cst = getcs(angT(0));
                    csp = getcs(angT(1));
                    q = getQ(cst, csp, g);
                    dq = getdQ(dAngT.head(2), cst);
                    d2q = getd2Q(dAngT.head(2), d2AngT.head(2), cst);
                    d3q = getd3Q(dAngT.head(2), d2AngT.head(2), d3AngT.head(2), cst);
                    posQ = posL + len * q;
                    velQ = velL + len * dq;
                    accQ = accL + len * d2q;
                    jerQ = jerL + len * d3q;

                    force = scale * angT[2] * q;

                    flatmap.forward(accQ, jerQ, q, dq, scale * angT(2), scale * dAngT(2), 0.0, 0.0, thr, quat, omg);
                    odoms[k + 1].pose.pose.position.x = posQ(0);
                    odoms[k + 1].pose.pose.position.y = posQ(1);
                    odoms[k + 1].pose.pose.position.z = posQ(2);
                    odoms[k + 1].pose.pose.orientation.w = quat(0);
                    odoms[k + 1].pose.pose.orientation.x = quat(1);
                    odoms[k + 1].pose.pose.orientation.y = quat(2);
                    odoms[k + 1].pose.pose.orientation.z = quat(3);
                    odoms[k + 1].twist.twist.linear.x = velQ(0);
                    odoms[k + 1].twist.twist.linear.y = velQ(1);
                    odoms[k + 1].twist.twist.linear.z = velQ(2);
                    odoms[k + 1].twist.twist.angular.x = omg(0);
                    odoms[k + 1].twist.twist.angular.y = omg(1);
                    odoms[k + 1].twist.twist.angular.z = omg(2);

                    line_list[k].header.frame_id = "odom";
                    line_list[k].header.stamp = ros::Time::now();
                    line_list[k].ns = "lines" + to_string(k);
                    line_list[k].action = visualization_msgs::Marker::ADD;
                    line_list[k].pose.orientation.w = 1.0;
                    line_list[k].id = k;
                    line_list[k].type = visualization_msgs::Marker::LINE_LIST;
                    line_list[k].scale.x = 0.02;
                    line_list[k].color.r = 0;
                    line_list[k].color.g = 0;
                    line_list[k].color.b = 1.0;
                    line_list[k].color.a = 0.5;
                    geometry_msgs::Point p;
                    p.x = posL(0);
                    p.y = posL(1);
                    p.z = posL(2);
                    line_list[k].points.push_back(p);
                    p.x = posQ(0);
                    p.y = posQ(1);
                    p.z = posQ(2);
                    line_list[k].points.push_back(p);
                    line_pubs[k].publish(line_list[k]);

                    force_list[k].header.frame_id = "odom";
                    force_list[k].header.stamp = ros::Time::now();
                    force_list[k].ns = "force" + to_string(k);
                    force_list[k].id = k + delta*1000000;
                    force_list[k].type = visualization_msgs::Marker::ARROW;
                    force_list[k].action = visualization_msgs::Marker::ADD;
                    // 设置箭头的起点和终点
                    
                    p.x = posQ(0);
                    p.y = posQ(1);
                    p.z = posQ(2);                  
                    force_list[k].points.push_back(p);  // 起点
                    double f_ratio = 0.75;
                    p.x = posQ(0) - 0.4*(force(0) / force.norm()) - force(0) * f_ratio;
                    p.y = posQ(1) - 0.4*(force(1) / force.norm()) - force(1) * f_ratio;
                    p.z = posQ(2) - 0.4*(force(2) / force.norm()) - force(2) * f_ratio;
                    force_list[k].points.push_back(p);  // 终点
                    force_list[k].scale.x = 0.100;  // 箭头粗细
                    force_list[k].scale.y = 0.150;  // 箭头宽度
                    force_list[k].scale.z = 0.27 * 1.75;  // 箭头高度


                    double value = ((delta / optTrajs[0].getTotalDuration()) * 2.7);
                    while(value > 1.0)
                    {
                        value -= 1.0;
                    }
                    const tinycolormap::Color color = tinycolormap::GetColor(value, tinycolormap::ColormapType::Turbo);
                    const double cr = color.r() * 255.0;
                    const double cg = color.g() * 255.0;
                    const double cb = color.b() * 255.0;

                    force_list[0].color.r = cr / 255.0;  // 红色
                    force_list[0].color.g = cg / 255.0;
                    force_list[0].color.b = cb / 255.0;

                    force_list[1].color.r = cr / 255.0;  // 红色
                    force_list[1].color.g = cg / 255.0;
                    force_list[1].color.b = cb / 255.0;

                    force_list[2].color.r = cr / 255.0;  // 红色
                    force_list[2].color.g = cg / 255.0;
                    force_list[2].color.b = cb / 255.0;

                    force_list[k].color.a = 1.0;  // 不透明

                }   
                for (int k = 0; k < config.droneNum + 1; k++)
                {
                    odom_pubs[k].publish(odoms[k]);
                  
                }

                
                odom_list.header.frame_id = "odom";
                odom_list.header.stamp = ros::Time::now();
                odom_list.ns = "odom_list" + to_string(delta);
                odom_list.action = visualization_msgs::Marker::ADD;
                odom_list.pose.orientation.w = 1.0;
                odom_list.id = delta;
                odom_list.type = visualization_msgs::Marker::SPHERE;
                odom_list.scale.x = 0.02;
                odom_list.color.r = 0;
                odom_list.color.g = 0;
                odom_list.color.b = 1.0;
                odom_list.color.a = 1.0;
                odom_list.pose.position.x = posL(0);
                odom_list.pose.position.y = posL(1);
                odom_list.pose.position.z = posL(2);

                odom_list.pose.orientation.w = 1.0;
                odom_list.pose.orientation.x = 0.0;
                odom_list.pose.orientation.y = 0.0;
                odom_list.pose.orientation.z = 0.0;
                odom_list.scale.x = 0.5;
                odom_list.scale.y = 0.5;
                odom_list.scale.z = 0.5;

                // odom_list_pub.publish(odom_list);
            }
        }
    }

    inline void setInitLoadWayPts()
    {
        // J
        // traj_J(config.pieceNum, wayPts, wayVels, wayAccs, velNums, accNums);
        // traj_circle(config.pieceNum, wayPts, wayVels, wayAccs, velNums, accNums);
        // traj_small_circle(config.pieceNum, wayPts, wayVels, wayAccs, velNums, accNums);  
        // traj_high_speed(config.pieceNum, wayPts, wayVels, wayAccs, velNums, accNums);
        // traj_narrow_gap(config.pieceNum, wayPts, wayVels, wayAccs, velNums, accNums);

        traj_avoiding_obstacle(config.pieceNum, wayPts, wayVels, wayAccs, velNums, accNums);
            
    }

    void polyTraj2ROSMsg(std::vector<quadrotor_msgs::PolyTraj> &msgs)
    {
        for (int k = 0; k < config.droneNum + 1; k++)
        {
            auto data = &optTrajs[k];

            msgs[k].traj_start_time = trajStamp;
            msgs[k].replan_duration = replan_duration;

            msgs[k].drone_id = k;
            msgs[k].order = 7; // todo, only support order = 5 now.
            if (k == 0)
            {
                msgs[k].type = 0;
            }
            else
            {
                msgs[k].type = 1;
            }

            Eigen::VectorXd durs = data->getDurations();
            int piece_num = data->getPieceNum();
            msgs[k].duration.resize(piece_num);
            msgs[k].coef_x.resize(8 * piece_num);
            msgs[k].coef_y.resize(8 * piece_num);
            msgs[k].coef_z.resize(8 * piece_num);
            for (int i = 0; i < piece_num; ++i)
            {
                msgs[k].duration[i] = durs(i);

                Piece<7>::CoefficientMat cMat = data->getPiece(i).getCoeffMat();
                int i8 = i * 8;
                for (int j = 0; j < 8; j++)
                {
                    msgs[k].coef_x[i8 + j] = cMat(0, j);
                    msgs[k].coef_y[i8 + j] = cMat(1, j);
                    msgs[k].coef_z[i8 + j] = cMat(2, j);
                }
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh_;

    GlobalPlanner global_planner(Config(ros::NodeHandle("~")), nh_);


    ros::Rate lr(1000);
    while (ros::ok())
    {
        global_planner.process();

        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
