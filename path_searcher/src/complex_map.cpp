#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <random>
#include <stdlib.h>

using namespace std;

ros::Publisher    _all_map_pub;

bool changableMapUsed = false;
bool isBlock0Got = false;
bool isBlock1Got = false;
bool isBlock2Got = false;

ros::Subscriber   _block_0_sub;
ros::Subscriber   _block_1_sub;
ros::Subscriber   _block_2_sub;

bool _has_map  = false;
double _sense_rate;
double cloud_resolution;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> global_map_pcl_cloud;

pcl::PointCloud<pcl::PointXYZ> cloud;
sensor_msgs::PointCloud2 output;

int map_id;

// map in file
std::string pcdPath;

// map of narrow slit
double slit_width;

// map of single wall
double wall_lenth, wall_width, wall_hight;

// map of random map
double random_num;

// map of rectFrame map
double x_l,y_l,z_l,x_h,y_h,z_h,x_m,y_m,z_m,len,height,theta,gap_size;

void geneWall_111(double ori_x , double ori_y ,double ori_z, double length, double width, double height)
{
    pcl::PointXYZ  s_point;

    for( double t_z = ori_z  ; t_z <= ori_z  + height ; t_z += 0.05 )
    {
        for( double t_y = ori_y; t_y <= ori_y + width ; t_y += 0.05 )
        {
            for( double t_x = ori_x; t_x <= ori_x + length; t_x += 0.05 )
            {
                s_point.x = t_x + (rand() % 10) / 250.0 ;
                s_point.y = t_y + (rand() % 10) / 250.0 ;
                s_point.z = t_z + (rand() % 10) / 800.0 ;

                cloud.push_back(s_point);
            }
        }
    }
}

void geneWall(double ori_x , double ori_y ,double ori_z, double length, double width, double height)
{
    pcl::PointXYZ  s_point;

    for( double t_z = ori_z  ; t_z <= ori_z  + height ; t_z += cloud_resolution )
    {
        for( double t_y = ori_y; t_y <= ori_y + width ; t_y += cloud_resolution )
        {
            for( double t_x = ori_x; t_x <= ori_x + length; t_x += cloud_resolution )
            {
                s_point.x = t_x + (rand() % 10) / 250.0 ;
                s_point.y = t_y + (rand() % 10) / 250.0 ;
                s_point.z = t_z + (rand() % 10) / 800.0 ;

                global_map_pcl_cloud.push_back(s_point);
            }
        }
    }
}

void geneCenterSizeWall(double center_x, double center_y, double center_z, double length, double width, double height)
{
  double boost = 1;
  geneWall((center_x - length/2)*boost, (center_y - width/2)*boost, (center_z - height/2)*boost, length*boost, width*boost, height*boost);
}

void geneCircleWall(double ori_x , double ori_y ,double ori_z, double radius, double height)
{
  pcl::PointXYZ  s_point;
  for( double t_z = ori_z ; t_z < ori_z  + height ; t_z += cloud_resolution )
  {
    for( double angle = 0; angle <= 360.0; angle += 10 )
    {
      for(double r = 0; r <= radius; r += cloud_resolution )
      {
        s_point.x = ori_x + r * cos(angle/180.0 * M_PI);
        s_point.y = ori_y + r * sin(angle/180.0 * M_PI);
        s_point.z = t_z;

        global_map_pcl_cloud.push_back(s_point);
      }
    }
  }
}

void FileMapGenerate()
{    
    pcl::io::loadPCDFile(pcdPath,global_map_pcl_cloud);
    // geneWall(-10.0, -5.5, 0, 20.0, 0.2, 0.8);
    // geneWall(-10.0, -5.3, 0, 0.2, 11.0, 0.8);
    // geneWall(-10.0, 5.5, 0, 20.0, 0.2, 0.8);
    // geneWall(10.0, -5.5, 0, 0.2, 11.0, 0.8);
}

void NarrowSlitGenerate()
{
    geneWall(-5.5, -2.5, 0, 11.0, 0.2, 0.8);
    geneWall(-5.5, -2.5, 0, 0.2, 5.0, 0.8);
    geneWall(-5.5, 2.5, 0, 11.0, 0.2, 0.8);
    geneWall(5.5, -2.5, 0, 0.2, 5.0, 0.8);

    geneWall(-0.75, slit_width/2, 0, 1.5, 1.0, 4.0);
    geneWall(-0.75, -1.0 - slit_width/2, 0, 1.5, 1.0, 4.0);
}

void SingleWallGenerate()
{
    geneWall(-10.0, -5.5, 0, 20.0, 0.2, 0.8);
    geneWall(-10.0, -5.3, 0, 0.2, 11.0, 0.8);
    geneWall(-10.0, 5.5, 0, 20.0, 0.2, 0.8);
    geneWall(10.0, -5.5, 0, 0.2, 11.0, 0.8);


    geneWall(-5.0-wall_lenth/2, -2.4-wall_width/2, 0, wall_lenth, wall_width, wall_hight);
    geneWall(6.5-wall_lenth/2, 2.4-wall_width/2, 0, wall_lenth, wall_width, wall_hight);

    // geneWall(-5.0-wall_lenth/2, -2.0-wall_width/2, 0, wall_width, wall_width, wall_hight);
    // geneWall(5.5-wall_lenth/2, 2.5-wall_width/2, 0, wall_width, wall_width, wall_hight);

    // demo ImgMake 2
    geneWall(-0.5 + 1.0, -1.0, 0, 1.4, 1.5, 2.0);
    geneWall(-1.0 + 1.0,  0.0, 0, 1.4, 1.5, 2.0);

    // geneWall(-0.5 + 1.2, -1.0, 0, 1.2, 1.5, 2.0);
    // geneWall(-1.0 + 1.2,  0.0, 0, 1.2, 1.5, 2.0);

    // demo ori
    // geneWall(-0.5, -1.0, 0, 1.5, 1.5, 2.0);
    // geneWall(-1.0,  0.0, 0, 1.5, 1.5, 2.0);


    // demo ImgMake 1
    // geneWall(1.9 + 1.0, -1.0, 0, 1.4, 1.5, 2.0);
    // geneWall(1.4 + 1.0,  0.0, 0, 1.4, 1.5, 2.0);

    // geneWall(4.3 + 1.0, -1.0, 0, 1.4, 1.5, 2.0);
    // geneWall(3.8 + 1.0,  0.0, 0, 1.4, 1.5, 2.0);


    // geneWall( 0.0, -1.2, 0, 0.6, 1.2, 2.0);
    // geneWall(-0.6,  0.0, 0, 0.6, 1.2, 2.0);

    // geneWall(-0.25, -0.75, 0, 1.0, 1.0, 2.0);
    // geneWall(-0.75,  0.0, 0, 1.0, 1.0, 2.0);
}

void RandomMapGenerate()
{
    double x,y,z,len,wid,hig;
    srand(25);
    for(int i = 0 ; i < random_num; i++)
    {
        x = double(rand()%60 - 30);
        y = double(rand()%30 - 15);
        // z = double(rand()%50  - 25) / 10.0;
        z = 0;
        len = double(rand()%3)/10 +  0.4;
        wid = double(rand()%3)/10 +  0.4;
        // len = 0.4;
        // wid = 0.4;
        hig = 2.4;

        geneWall(x,y,z,len,wid,hig);
    }

    // geneWall(-6.3,0.5,0,0.4,0.4,2.4);
    // geneWall(-6.5,-0.3,0,0.4,0.4,2.1);
    // geneWall(-5.5,-1.9,0,0.4,0.4,2.6);
    // geneWall(-6.5,-3.5,0,0.4,0.4,2.6);
    // geneWall(-6.8,-6.0,0,0.6,0.6,2.2);
    // geneWall(-5.5,3.8,0,0.4,0.4,2.5);
    // geneWall(-5.7,5.1,0,0.4,0.4,2.6);
    // geneWall(-7.1 + 0.5,6.8,0,0.4,0.4,2.8);
    // geneWall(-5.7 + 0.5,7.2,0,0.4,0.4,2.6);
    // geneWall(-6.2 + 0.5,8.2,0,0.4,0.4,2.8);
    // // geneWall(-8.5,-1.5,0,0.4,0.4,2.3);
    // geneWall(-8.4,2.2,0,0.4,0.4,2.3);
    // geneWall(-9.5,3.3,0,0.4,0.4,2.4);
    // geneWall(-10.5,5.5,0,0.4,0.4,2.1);
    // geneWall(-11.5,3.6,0,0.4,0.4,2.7);
    // geneWall(-12.5,3.2,0,0.4,0.4,2.4);
    // geneWall(-11.5,6.0,0,0.4,0.4,2.7);
    // geneWall(-11.0,-1.8,0,0.4,0.4,2.9);
    // geneWall(-10.6,-3.0,0,0.4,0.4,2.3);
    // geneWall(-12.5,-3.3,0,0.4,0.4,2.2);

    // geneWall(-10.0,0.3,0,0.4,0.4,2.4);
    // geneWall(-9.0,0.7,0,0.4,0.4,2.4);

    // geneWall(-9.1,-1.0,0,0.4,0.4,2.4);
    // geneWall(-11.5,0,0,0.4,0.4,2.6);
    // geneWall(-11.2,1.8,0,0.4,0.4,2.8);
    // geneWall(-5.8,2.0,0,0.4,0.4,2.8);
    // geneWall(-5.0,-3.8,0,0.6,0.6,2.2);

    // geneWall(-6.6,-2.0,0,0.4,0.4,2.2);

    // geneWall(-4.8,-0.8,0,0.4,0.4,2.2);

    // geneWall(-5.7 + 2.5,5.1 - 5.5,0,0.4,0.4,2.6);
    // geneWall(-7.1 + 2.5,6.8 - 5.5,0,0.4,0.4,2.8);
    // geneWall(-5.7 + 2.5,7.2 - 5.5,0,0.4,0.4,2.6);
    // geneWall(-6.2 + 2.5,8.2 - 5.5,0,0.4,0.4,2.8);

    // geneWall(-5.7 + 2.5,5.1 - 1.2,0,0.4,0.4,2.6);
    // geneWall(-7.1 + 2.5,6.8 - 1.2,0,0.4,0.4,2.8);
    // geneWall(-5.7 + 2.5,7.2 - 1.2,0,0.4,0.4,2.6);
    // geneWall(-6.2 + 2.5,8.2 - 1.2,0,0.4,0.4,2.8);

    //  geneWall(0, 0.5, 0, 1.8, 0.5, 1.0);
    //  geneWall(0, -1.0, 0, 1.8, 0.5, 1.0);
    //  geneWall(-0.5, 0.5, 0, 0.5, 2.0, 1.0);
    //  geneWall(-2.5, -1.0, 0, 0.6, 1.0, 1.0);
    //  geneWall(-4.5, 0.0, 0, 2.6, 0.3, 1.0);
    //  geneWall(-4.5, -0.1, 0, 2.6, 0.2, 20);
    //  geneWall(-4.5, -0.1, -10, 0.8, 0.2, 120);
    //  geneWall(-4.5, 2.4, 0, 4.5, 0.5, 1.0);
    //  geneWall(-2.0, -1.0, 0, 2.0, 0.5, 1.0);
    // //  geneWall(1.5, -1.0, 0, 1.0, 0.2, 1.0);
    // //  geneWall(1.6, 0.6, 0, 1.0, 0.4, 1.0);
    // geneCircleWall(0, 0, 0, 2.0, 4.0);

}

// void benchmarkMapGenerate()
// {
//   // geneCircleWall(2.0, 2.3, 0, 0.5, 3.0);
//   // geneCircleWall(5.2, 2.0, 0, 0.5, 3.0);
//   // geneCircleWall(4.1, 6.5, 0, 0.8, 3.0);
//   // geneCircleWall(7.0, 8.5, 0, 0.6, 3.0);
//   // geneCircleWall(2.0, 15.2, 0, 0.8, 3.0);
//   // geneCircleWall(7.2, 13.7, 0, 0.6, 3.0);
//   // geneCircleWall(12.6, 4.1, 0, 0.6, 3.0);
//   // geneCircleWall(17.2, 2.6, 0, 0.70, 3.0);
//   // geneCircleWall(13.2, 8.6, 0, 0.6, 3.0);
//   // geneCircleWall(11.5, 11.5, 0, 0.50, 3.0);
//   // geneCircleWall(11.0, 13.5, 0, 0.70, 3.0);
//   // geneCenterSizeWall(0.2, 4.9, 1.5, 1.4, 1.4, 3.0);
//   // geneCenterSizeWall(1.9, 8.2, 1.5, 1.8, 1.0, 3.0);
//   // geneCenterSizeWall(2.4, 11.7, 1.5, 1.7, 1.7, 3.0);
//   // geneCenterSizeWall(4.4, 13.3, 1.5, 1.2, 2.7, 3.0); //15
//   // geneCenterSizeWall(3.1, 4.5, 1.5, 1.2, 1.2, 3.0);
//   // geneCenterSizeWall(6.5, 4.1, 1.5, 1.2, 1.2, 3.0);
//   // geneCenterSizeWall(9.5, 1.0, 1.5, 1.0, 2.0, 3.0);
//   // geneCenterSizeWall(9.3, 5.2, 1.5, 1.2, 1.2, 3.0);
//   // geneCenterSizeWall(9.3, 11.5, 1.5, 1.5, 1.5, 3.0);
//   // geneCenterSizeWall(11.6, 1.9, 1.5, 1.2, 1.2, 3.0);
//   // geneCenterSizeWall(7.1, 15.9, 1.5, 1.5, 1.5, 3.0);
//   // geneCenterSizeWall(11.2, 7.5, 1.5, 1.2, 2.5, 3.0);
//   // geneCenterSizeWall(14.4, 5.6, 1.5, 1.2, 1.2, 3.0);
//   // geneCenterSizeWall(15.1, -0.2, 1.5, 1.5, 1.5, 3.0);
//   // geneCenterSizeWall(14.9, 9.6, 1.5, 1.2, 1.2, 3.0); //26
//   // geneCenterSizeWall(17.3, 5.2, 1.5, 1.5, 1.5, 3.0);
//   // geneCircleWall(-3.2, 3.2, 0, 1.0, 3.0);
//   // geneCircleWall(-2.1, 7.1, 0, 0.6, 3.0);
//   // geneCircleWall(-3.0, 10.6, 0, 0.6, 3.0);
//   // geneCircleWall(-0.7, 11.3, 0, 0.6, 3.0);
//   // geneCircleWall(-2.6, 14.3, 0, 0.6, 3.0); //32
//   // geneCenterSizeWall(-1.2, 9.1, 1.5, 1.2, 1.2, 3.0);
//   // geneCenterSizeWall(-0.8, 13.3, 1.5, 1.2, 1.2, 3.0);
//   // geneCenterSizeWall(-2.2, 16.5, 1.5, 2.2, 1.2, 3.0);
//   // geneCenterSizeWall(-3.8, 8.2, 1.5, 1.4, 1.4, 3.0); //36
//   // geneCenterSizeWall(6.4, -1.8, 1.5, 2.2, 1.2, 3.0);

//   srand(10);
//   // (rand() % (4 - 0 + 1) + 0)

//   //     * * * * * 
//   //   * * * * * * * 
//   // * * * * * * * * * 
//   // * * * * * * * * * 
//   // * * * *   * * * * 
//   // * * * * * * * * * 
//   // * * * * * * * * * 
//   //   * * * * * * *
//   //     * * * * *  
//   for(int i = 0; i <= 8; i++)
//     for(int j = 0 ;j <= 8; j++)
//       {
//         if(i==8&&j==8)
//           continue;
//         if(i==8&&j==7)
//           continue;
//         if(i==7&&j==8)
//           continue;
//         if(i==8&&j==0)
//           continue;
//         if(i==8&&j==1)
//           continue;
//         if(i==7&&j==0)
//           continue;
//         if(i==0&&j==0)
//           continue;
//         if(i==0&&j==1)
//           continue;
//         if(i==1&&j==0)
//           continue;
//         if(i==1&&j==8)
//           continue;
//         if(i==0&&j==8)
//           continue;
//         if(i==0&&j==7)
//           continue;
//         if(i==4&&j==4)
//           continue;
//         if(rand()%2 == 0)
//         {
//           double box_x = i*5.4 - 22 + (rand()%3 - 1);
//           double box_y = j*5.4 - 22 + (rand()%3 - 1);
//           double box_z = 3.0;
//           double box_length = 0.8 + (rand()%3)*0.2;
//           double box_width = 0.8 + (rand()%3)*0.2;
//           double box_height = 6.0;
//           // geneCenterSizeWall(i*4.5 - 19 + (rand()%3 - 1), j*4.5 - 19 + (rand()%3 - 1), 1.5, 0.8 + (rand()%3 - 1)*0.2, 0.8 + (rand()%3 - 1)*0.2, 3.0);
//           geneCenterSizeWall(box_x,box_y,box_z,box_length,box_width,box_height);
//           cout<<"- type: box\n"<<"  center: ["<<box_x<<", "<<box_y<<", "<<box_z<<"]\n"<<"  size: ["<<box_length<<", "<<box_width<<", "<<box_height<<"]"<<endl;
//         }
//         else
//         { 
//           double clc_x = i*5.4 - 22 + (rand()%3 - 1);
//           double clc_y = j*5.4 - 22 + (rand()%3 - 1);
//           double clc_z = 0.0;
//           double clc_radius = 0.5+(rand()%3 - 1)*0.1;
//           double clc_height = 6.0;
//           // if(i==2&&j==5)
//           // {
//           //   clc_x-=0.4;
//           //   clc_y+=1.6;
//           // }
//           // geneCircleWall(i*4.5 - 18 + (rand()%3 - 1), j*4.5 - 18 + (rand()%3 - 1), 0, 0.5+(rand()%3 - 1)*0.1, 3.0);
//           geneCircleWall(clc_x,clc_y,clc_z,clc_radius,clc_height);
//           cout<<"- type: cylinder\n"<<"  center: ["<<clc_x<<", "<<clc_y<<", "<<clc_z+3.0<<"]\n"<<"  radius: "<<clc_radius<<"\n"<<"  height: 6.0"<<endl;
//         }

//       }
  


// }

void benchmarkMapGenerate()
{
  // geneCircleWall(2.0, 2.3, 0, 0.5, 3.0);
  // geneCircleWall(5.2, 2.0, 0, 0.5, 3.0);
  // geneCircleWall(4.1, 6.5, 0, 0.8, 3.0);
  // geneCircleWall(7.0, 8.5, 0, 0.6, 3.0);
  // geneCircleWall(2.0, 15.2, 0, 0.8, 3.0);
  // geneCircleWall(7.2, 13.7, 0, 0.6, 3.0);
  // geneCircleWall(12.6, 4.1, 0, 0.6, 3.0);
  // geneCircleWall(17.2, 2.6, 0, 0.70, 3.0);
  // geneCircleWall(13.2, 8.6, 0, 0.6, 3.0);
  // geneCircleWall(11.5, 11.5, 0, 0.50, 3.0);
  // geneCircleWall(11.0, 13.5, 0, 0.70, 3.0);
  // geneCenterSizeWall(0.2, 4.9, 1.5, 1.4, 1.4, 3.0);
  // geneCenterSizeWall(1.9, 8.2, 1.5, 1.8, 1.0, 3.0);
  // geneCenterSizeWall(2.4, 11.7, 1.5, 1.7, 1.7, 3.0);
  // geneCenterSizeWall(4.4, 13.3, 1.5, 1.2, 2.7, 3.0); //15
  // geneCenterSizeWall(3.1, 4.5, 1.5, 1.2, 1.2, 3.0);
  // geneCenterSizeWall(6.5, 4.1, 1.5, 1.2, 1.2, 3.0);
  // geneCenterSizeWall(9.5, 1.0, 1.5, 1.0, 2.0, 3.0);
  // geneCenterSizeWall(9.3, 5.2, 1.5, 1.2, 1.2, 3.0);
  // geneCenterSizeWall(9.3, 11.5, 1.5, 1.5, 1.5, 3.0);
  // geneCenterSizeWall(11.6, 1.9, 1.5, 1.2, 1.2, 3.0);
  // geneCenterSizeWall(7.1, 15.9, 1.5, 1.5, 1.5, 3.0);
  // geneCenterSizeWall(11.2, 7.5, 1.5, 1.2, 2.5, 3.0);
  // geneCenterSizeWall(14.4, 5.6, 1.5, 1.2, 1.2, 3.0);
  // geneCenterSizeWall(15.1, -0.2, 1.5, 1.5, 1.5, 3.0);
  // geneCenterSizeWall(14.9, 9.6, 1.5, 1.2, 1.2, 3.0); //26
  // geneCenterSizeWall(17.3, 5.2, 1.5, 1.5, 1.5, 3.0);
  // geneCircleWall(-3.2, 3.2, 0, 1.0, 3.0);
  // geneCircleWall(-2.1, 7.1, 0, 0.6, 3.0);
  // geneCircleWall(-3.0, 10.6, 0, 0.6, 3.0);
  // geneCircleWall(-0.7, 11.3, 0, 0.6, 3.0);
  // geneCircleWall(-2.6, 14.3, 0, 0.6, 3.0); //32
  // geneCenterSizeWall(-1.2, 9.1, 1.5, 1.2, 1.2, 3.0);
  // geneCenterSizeWall(-0.8, 13.3, 1.5, 1.2, 1.2, 3.0);
  // geneCenterSizeWall(-2.2, 16.5, 1.5, 2.2, 1.2, 3.0);
  // geneCenterSizeWall(-3.8, 8.2, 1.5, 1.4, 1.4, 3.0); //36
  // geneCenterSizeWall(6.4, -1.8, 1.5, 2.2, 1.2, 3.0);

  srand(10);
  // (rand() % (4 - 0 + 1) + 0)


  // for(int i = 0; i <= 8; i++)
  //   for(int j = 0 ;j <= 4; j++)
  //     {
  //       // if(i==8&&j==8)
  //       //   continue;
  //       // if(i==8&&j==7)
  //       //   continue;
  //       // if(i==7&&j==8)
  //       //   continue;
  //       // if(i==8&&j==0)
  //       //   continue;
  //       // if(i==8&&j==1)
  //       //   continue;
  //       // if(i==7&&j==0)
  //       //   continue;
  //       // if(i==0&&j==0)
  //       //   continue;
  //       // if(i==0&&j==1)
  //       //   continue;
  //       // if(i==1&&j==0)
  //       //   continue;
  //       // if(i==1&&j==8)
  //       //   continue;
  //       // if(i==0&&j==8)
  //       //   continue;
  //       // if(i==0&&j==7)
  //       //   continue;
  //       // if(i==4&&j==4)
  //       //   continue;

        
  //       if(rand()%2 == 0)
  //       {
  //         double box_x = i*3.8 - 15.6 +  (rand()%3 - 1)*0.55;
  //         double box_y = j*3.8 - 7.8 + (rand()%3 - 1)*0.55;
  //         double box_z = 3.0;
  //         double box_length = 0.9 + (rand()%3)*0.2;
  //         double box_width = 0.9 + (rand()%3)*0.2;
  //         double box_height = 6.0;
  //         // geneCenterSizeWall(i*4.5 - 19 + (rand()%3 - 1), j*4.5 - 19 + (rand()%3 - 1), 1.5, 0.8 + (rand()%3 - 1)*0.2, 0.8 + (rand()%3 - 1)*0.2, 3.0);
  //         geneCenterSizeWall(box_x,box_y,box_z,box_length,box_width,box_height);
  //         cout<<"  - type: box\n"<<"    center: ["<<box_x<<", "<<box_y<<", "<<box_z<<"]\n"<<"    size: ["<<box_length<<", "<<box_width<<", "<<box_height<<"]"<<endl;
  //       }
  //       else
  //       { 
  //         double clc_x = i*3.8 - 15.6 + (rand()%3 - 1)*0.55;
  //         double clc_y = j*3.8 - 7.8 + (rand()%3 - 1)*0.55;
  //         double clc_z = 0.0;
  //         double clc_radius = 0.6 + (rand()%3 - 1)*0.1;
  //         double clc_height = 6.0;
  //         // if(i==2&&j==5)
  //         // {
  //         //   clc_x-=0.4;
  //         //   clc_y+=1.6;
  //         // }
  //         // geneCircleWall(i*4.5 - 18 + (rand()%3 - 1), j*4.5 - 18 + (rand()%3 - 1), 0, 0.5+(rand()%3 - 1)*0.1, 3.0);
  //         geneCircleWall(clc_x,clc_y,clc_z,clc_radius,clc_height);
  //         cout<<"  - type: cylinder\n"<<"    center: ["<<clc_x<<", "<<clc_y<<", "<<clc_z+3.0<<"]\n"<<"    radius: "<<clc_radius<<"\n"<<"    height: 6.0"<<endl;
  //       }

  //     }

  //     * * * * * 
  //   * * * * * * * 
  // * * * * * * * * * 
  // * * * * * * * * * 
  // * * * *   * * * * 
  // * * * * * * * * * 
  // * * * * * * * * * 
  //   * * * * * * *
  //     * * * * *  
  // dense
  // ofstream p1;
  // p1.open("/home/wangjunjie/map_box.csv",ios::out|ios::trunc);
  // ofstream p2;
  // p2.open("/home/wangjunjie/map_clc.csv",ios::out|ios::trunc);
  // for(int i = 0; i <= 8; i++)
  //   for(int j = 0 ;j <= 8; j++)
  //     {
  //       if(i==8&&j==8)
  //         continue;
  //       if(i==8&&j==7)
  //         continue;
  //       if(i==7&&j==8)
  //         continue;
  //       if(i==8&&j==0)
  //         continue;
  //       if(i==8&&j==1)
  //         continue;
  //       if(i==7&&j==0)
  //         continue;
  //       if(i==0&&j==0)
  //         continue;
  //       if(i==0&&j==1)
  //         continue;
  //       if(i==1&&j==0)
  //         continue;
  //       if(i==1&&j==8)
  //         continue;
  //       if(i==0&&j==8)
  //         continue;
  //       if(i==0&&j==7)
  //         continue;
  //       if(i==4&&j==4)
  //         continue;

        
  //       if(rand()%2 == 0)
  //       {
  //         double box_x = i*3.8 - 15.6 + (rand()%3 - 1)*0.4;
  //         double box_y = j*3.8 - 15.6 + (rand()%3 - 1)*0.4;
  //         double box_z = 3.0;
  //         double box_length = 0.6 + (rand()%3)*0.2;
  //         double box_width = 0.6 + (rand()%3)*0.2;
  //         double box_height = 6.0;
  //         // geneCenterSizeWall(i*4.5 - 19 + (rand()%3 - 1), j*4.5 - 19 + (rand()%3 - 1), 1.5, 0.8 + (rand()%3 - 1)*0.2, 0.8 + (rand()%3 - 1)*0.2, 3.0);
  //         geneCenterSizeWall(box_x,box_y,box_z,box_length,box_width,box_height);
  //         cout<<"  - type: box\n"<<"    center: ["<<box_x<<", "<<box_y<<", "<<box_z<<"]\n"<<"    size: ["<<box_length<<", "<<box_width<<", "<<box_height<<"]"<<endl;
  //         p1 << box_x << "," << box_y << "," << box_z << "," << box_length << "," << box_width << "," << box_height << endl;
  //       }
  //       else
  //       { 
  //         double clc_x = i*3.8 - 15.6 + (rand()%3 - 1)*0.4;
  //         double clc_y = j*3.8 - 15.6 + (rand()%3 - 1)*0.4;
  //         double clc_z = 0.0;
  //         double clc_radius = 0.6 + (rand()%3 - 1)*0.1;
  //         double clc_height = 6.0;
  //         // if(i==2&&j==5)
  //         // {
  //         //   clc_x-=0.4;
  //         //   clc_y+=1.6;
  //         // }
  //         // geneCircleWall(i*4.5 - 18 + (rand()%3 - 1), j*4.5 - 18 + (rand()%3 - 1), 0, 0.5+(rand()%3 - 1)*0.1, 3.0);
  //         geneCircleWall(clc_x,clc_y,clc_z,clc_radius,clc_height);
  //         cout<<"  - type: cylinder\n"<<"    center: ["<<clc_x<<", "<<clc_y<<", "<<clc_z+3.0<<"]\n"<<"    radius: "<<clc_radius<<"\n"<<"    height: 6.0"<<endl;
  //         p2 << clc_x << "," << clc_y << "," << clc_z << clc_radius << "," << clc_height << endl;
  //       }

  //     }


  // thin
  // ofstream p1;
  // p1.open("/home/wangjunjie/map_box.csv",ios::out|ios::trunc);
  // ofstream p2;
  // p2.open("/home/wangjunjie/map_clc.csv",ios::out|ios::trunc);
  // for(int i = 0; i <= 6; i++)
  //   for(int j = 0 ;j <= 6; j++)
  //     {
  //       if(i==6&&j==6)
  //         continue;
  //       if(i==6&&j==5)
  //         continue;
  //       if(i==5&&j==6)
  //         continue;
  //       if(i==6&&j==0)
  //         continue;
  //       if(i==6&&j==1)
  //         continue;
  //       if(i==5&&j==0)
  //         continue;
  //       if(i==0&&j==0)
  //         continue;
  //       if(i==0&&j==1)
  //         continue;
  //       if(i==1&&j==0)
  //         continue;
  //       if(i==1&&j==6)
  //         continue;
  //       if(i==0&&j==6)
  //         continue;
  //       if(i==0&&j==5)
  //         continue;
  //       if(i==3&&j==3)
  //         continue;

        
  //       if(rand()%2 == 0)
  //       {
  //         double box_x = i*5.2 - 15.6 + (rand()%5 - 2)*0.4;
  //         double box_y = j*5.2 - 15.6 + (rand()%5 - 2)*0.4;
  //         double box_z = 3.0;
  //         double box_length = 1.0 + (rand()%3 - 1)*0.2;
  //         double box_width = 1.0 + (rand()%3 - 1)*0.2;
  //         double box_height = 6.0;
  //         // geneCenterSizeWall(i*4.5 - 19 + (rand()%3 - 1), j*4.5 - 19 + (rand()%3 - 1), 1.5, 0.8 + (rand()%3 - 1)*0.2, 0.8 + (rand()%3 - 1)*0.2, 3.0);
  //         geneCenterSizeWall(box_x,box_y,box_z,box_length,box_width,box_height);
  //         cout<<"  - type: box\n"<<"    center: ["<<box_x<<", "<<box_y<<", "<<box_z<<"]\n"<<"    size: ["<<box_length<<", "<<box_width<<", "<<box_height<<"]"<<endl;
  //         p1 << box_x << "," << box_y << "," << box_z << "," << box_length << "," << box_width << "," << box_height << endl;
  //       }
  //       else
  //       { 
  //         double clc_x = i*5.2 - 15.6 + (rand()%5 - 2)*0.4;
  //         double clc_y = j*5.2 - 15.6 + (rand()%5 - 2)*0.4;
  //         double clc_z = 0.0;
  //         double clc_radius = 0.6 + (rand()%3 - 1)*0.1;
  //         double clc_height = 6.0;
  //         // if(i==2&&j==5)
  //         // {
  //         //   clc_x-=0.4;
  //         //   clc_y+=1.6;
  //         // }
  //         // geneCircleWall(i*4.5 - 18 + (rand()%3 - 1), j*4.5 - 18 + (rand()%3 - 1), 0, 0.5+(rand()%3 - 1)*0.1, 3.0);
  //         geneCircleWall(clc_x,clc_y,clc_z,clc_radius,clc_height);
  //         cout<<"  - type: cylinder\n"<<"    center: ["<<clc_x<<", "<<clc_y<<", "<<clc_z+3.0<<"]\n"<<"    radius: "<<clc_radius<<"\n"<<"    height: 6.0"<<endl;
  //         p2 << clc_x << "," << clc_y << "," << clc_z << clc_radius << "," << clc_height << endl;
  //       }

  //     }


  // mid
  srand(19);
  ofstream p1;
  p1.open("/home/wangjunjie/map_box.csv",ios::out|ios::trunc);
  ofstream p2;
  p2.open("/home/wangjunjie/map_clc.csv",ios::out|ios::trunc);
  for(int i = 0; i <= 7; i++)
    for(int j = 0 ;j <= 7; j++)
      {
        if(i==7&&j==7)
          continue;
        if(i==7&&j==6)
          continue;
        if(i==6&&j==7)
          continue;
        if(i==7&&j==0)
          continue;
        if(i==7&&j==1)
          continue;
        if(i==6&&j==0)
          continue;
        if(i==0&&j==0)
          continue;
        if(i==0&&j==1)
          continue;
        if(i==1&&j==0)
          continue;
        if(i==1&&j==7)
          continue;
        if(i==0&&j==7)
          continue;
        if(i==0&&j==6)
          continue;

        
        if(rand()%2 == 0)
        {
          double box_x = i*3.9 - 13.6 + (rand()%3 - 1)*0.4;
          double box_y = j*3.9 - 13.6 + (rand()%3 - 1)*0.4;
          double box_z = 3.0;
          double box_length = 0.8 + (rand()%3 - 1)*0.2;
          double box_width = 0.8 + (rand()%3 - 1)*0.2;
          double box_height = 6.0;
          // geneCenterSizeWall(i*4.5 - 19 + (rand()%3 - 1), j*4.5 - 19 + (rand()%3 - 1), 1.5, 0.8 + (rand()%3 - 1)*0.2, 0.8 + (rand()%3 - 1)*0.2, 3.0);
          if(i == 4 && j == 4)
          {
            box_x += 0.5;
            box_y += 0.5;
            geneCenterSizeWall(box_x,box_y,box_z,box_length,box_width,box_height);
          }
          if(i == 3 && j == 4)
          {
            // box_x -= 0.5;
            // box_y += 0.5;
            geneCenterSizeWall(box_x,box_y,box_z,box_length,box_width,box_height);
          }
          if(i == 3 && j == 3)
          {
            box_x -= 0.5;
            box_y -= 1.0;
            geneCenterSizeWall(box_x,box_y,box_z,box_length,box_width,box_height);
          }
          if(i == 4 && j == 3)
          {
            box_x += 1.0;
            // box_y += 1.5;
            geneCenterSizeWall(box_x,box_y,box_z,box_length,box_width,box_height);
          }
          else
            geneCenterSizeWall(box_x,box_y,box_z,box_length,box_width,box_height);
          cout<<"  - type: box\n"<<"    center: ["<<box_x<<", "<<box_y<<", "<<box_z<<"]\n"<<"    size: ["<<box_length<<", "<<box_width<<", "<<box_height<<"]"<<endl;
          p1 << box_x << "," << box_y << "," << box_z << "," << box_length << "," << box_width << "," << box_height << endl;
        }
        else
        { 
          double clc_x = i*3.9 - 13.6 + (rand()%3 - 1)*0.4;
          double clc_y = j*3.9 - 13.6 + (rand()%3 - 1)*0.4;
          double clc_z = 0.0;
          double clc_radius = 0.4 + (rand()%3 - 1)*0.1;
          double clc_height = 6.0;
          // if(i==2&&j==5)
          // {
          //   clc_x-=0.4;
          //   clc_y+=1.6;
          // }
          // geneCircleWall(i*4.5 - 18 + (rand()%3 - 1), j*4.5 - 18 + (rand()%3 - 1), 0, 0.5+(rand()%3 - 1)*0.1, 3.0);
          if(i == 4 && j == 3)
          {
            clc_x += 1.0;
            // box_y += 1.5;
            geneCircleWall(clc_x,clc_y,clc_z,clc_radius,clc_height);
          }
          else
            geneCircleWall(clc_x,clc_y,clc_z,clc_radius,clc_height);
          cout<<"  - type: cylinder\n"<<"    center: ["<<clc_x<<", "<<clc_y<<", "<<clc_z+3.0<<"]\n"<<"    radius: "<<clc_radius<<"\n"<<"    height: 6.0"<<endl;
          p2 << clc_x << "," << clc_y << "," << clc_z << clc_radius << "," << clc_height << endl;
        }

      }
  


}


int is_inFrame(double x1, double y1,
               double x2, double y2,
               double len,  double height, double theta)
{
  double alpha = atan2(height,len);
  double phi = atan2(y2-y1,x2-x1);
  if(y2-y1 <= 0)
  {
    phi += 2 * M_PI;
  }
  phi = phi + theta;
  if(phi >= 2 * M_PI)
    phi -= 2 * M_PI;
  double l = sqrt(pow((x1-x2),2) + pow((y1-y2),2));
  if((phi>=alpha && phi <=(M_PI-alpha)) || (phi>=(alpha+M_PI) && phi<=(2*M_PI-alpha)))
  {
    if(l <= abs(height/(2*sin(phi))))
      return 1;
    else
      return 0;
  }
  else
  {
    if(l <= abs(len/(2*cos(phi))))
      return 1;
    else
      return 0;
  }
  // else if(phi>=0 && phi<=alpha)
  // {
  //   if(l <= len/2*(cos(phi)))
  //     return 1;
  //   else
  //     return 0;
  // }
  // else if(phi)
}

void rectFrameGenerate()
{
  if(gap_size == 0)
  {
    int x_num, y_num, z_num;
    double scale = 1.0 / cloud_resolution;
    x_num = ceil(x_h * scale);
    y_num = ceil(y_h * scale);
    z_num = ceil(z_h * scale);

    pcl::PointXYZ pt;
    for (int i=0; i<x_num; i++)
      for (int j=0; j<y_num; j++)
        for (int k=0; k<z_num; k++){
          if(y_m == 0)
          {
            if(is_inFrame(x_m,z_m,(double(i)/double(x_num))*x_h,(double(k)/double(z_num))*z_h,len,height,theta))
              continue;
          }
          else if(x_m == 0)
          {
            if(is_inFrame(y_m,z_m,(double(j)/double(y_num))*y_h,(double(k)/double(z_num))*z_h,len,height,theta))
              continue;
          }
          pt.x = x_l + i * cloud_resolution;
          pt.y = y_l + j * cloud_resolution;
          pt.z = z_l + k * cloud_resolution;
          global_map_pcl_cloud.push_back(pt);
        }
  }
  else
  {
    int x_num, y_num, z_num;
    double scale = 1.0 / cloud_resolution;
    x_num = ceil(x_h * scale);
    y_num = ceil((y_h-gap_size)/2 * scale);
    z_num = ceil(z_h * scale);
    pcl::PointXYZ pt;
    for (int i=0; i<x_num; i++)
      for (int j=0; j<y_num; j++)
        for (int k=0; k<z_num; k++){
      pt.x = x_l + i * cloud_resolution;
      pt.y = y_l + j * cloud_resolution;
      pt.z = z_l + k * cloud_resolution;
      global_map_pcl_cloud.push_back(pt);
    }

    x_num = ceil(x_h * scale);
    y_num = ceil((y_h-gap_size)/2 * scale);
    z_num = ceil(z_h * scale);
    for (int i=0; i<x_num; i++)
      for (int j=0; j<y_num; j++)
        for (int k=0; k<z_num; k++){
      pt.x = x_l + i * cloud_resolution;
      pt.y = y_l+(y_h+gap_size)/2 + j * cloud_resolution;
      pt.z = z_l + k * cloud_resolution;
      global_map_pcl_cloud.push_back(pt);
    }
  }
}

void benchmark_forest()
{
  geneCenterSizeWall(7.3,-2.0,3.0,0.4,0.5,6.0);
  geneCenterSizeWall(9.0,-0.0,3.0,0.6,0.4,6.0);
  geneCenterSizeWall(-12.4,-1.8,3.0,0.5,0.4,6.0);
  geneCenterSizeWall(5.1,-0.0,3.0,0.4,0.5,6.0);
  geneCenterSizeWall(2.4,-1.4,3.0,0.6,0.6,6.0);
  geneCenterSizeWall(-11.2,0.7,3.0,0.4,0.4,6.0);
  geneCenterSizeWall(3.7,2.2,3.0,0.6,0.6,6.0);
  geneCenterSizeWall(-1.0,2.6,3.0,0.4,0.4,6.0);
  geneCenterSizeWall(12.5,0.7,3.0,0.5,0.4,6.0);
  geneCenterSizeWall(0.7,-2.3,3.0,0.6,0.5,6.0);
  geneCenterSizeWall(-2.3,-2.4,3.0,0.5,0.6,6.0);
  geneCenterSizeWall(-7.8,-1.2,3.0,0.4,0.5,6.0);
  geneCenterSizeWall(-9.4,2.4,3.0,0.4,0.4,6.0);
  geneCenterSizeWall(14.7,-0.2,3.0,0.6,0.6,6.0);
  geneCenterSizeWall(-2.9,2.5,3.0,0.5,0.4,6.0);
  geneCenterSizeWall(3.0,0.4,3.0,0.4,0.6,6.0);
  geneCenterSizeWall(3.9,-2.6,3.0,0.5,0.5,6.0);
  geneCenterSizeWall(5.8,1.6,3.0,0.6,0.6,6.0);
  geneCenterSizeWall(-14.3,-0.2,3.0,0.6,0.5,6.0);
  geneCenterSizeWall(-14.4,1.4,3.0,0.6,0.6,6.0);
  geneCenterSizeWall(7.9,1.7,3.0,0.4,0.6,6.0);
  geneCenterSizeWall(-6.3,-2.7,3.0,0.5,0.4,6.0);
  geneCenterSizeWall(-9.8,-1.6,3.0,0.4,0.5,6.0);
  geneCenterSizeWall(5.6,-2.5,3.0,0.5,0.6,6.0);
  geneCenterSizeWall(12.7,-2.7,3.0,0.6,0.5,6.0);
  geneCenterSizeWall(-1.0,-0.2,3.0,0.4,0.6,6.0);
  geneCenterSizeWall(10.1,1.3,3.0,0.5,0.6,6.0);
  geneCenterSizeWall(0.7,-0.7,3.0,0.6,0.6,6.0);
  geneCenterSizeWall(9.9,-2.6,3.0,0.6,0.4,6.0);
  geneCenterSizeWall(-4.4,-1.7,3.0,0.5,0.4,6.0);
  geneCenterSizeWall(-7.0,1.4,3.0,0.6,0.6,6.0);
  geneCenterSizeWall(3.9,-1.0,3.0,0.6,0.5,6.0);
  geneCenterSizeWall(-13.3,2.7,3.0,0.6,0.6,6.0);
  geneCenterSizeWall(-11.3,2.3,3.0,0.5,0.4,6.0);
  geneCenterSizeWall(-4.5,2.3,3.0,0.6,0.6,6.0);
  geneCenterSizeWall(-2.4,1.0,3.0,0.6,0.4,6.0);
  geneCenterSizeWall(14.5,-2.0,3.0,0.6,0.5,6.0);
  geneCenterSizeWall(11.8,2.2,3.0,0.4,0.6,6.0);
  geneCenterSizeWall(-5.3,0.7,3.0,0.5,0.6,6.0);
  geneCenterSizeWall(1.3,2.6,3.0,0.5,0.6,6.0);
}

void changableMapGenerate()
{
    changableMapUsed = true;    
    // geneWall(0,0,0,1,1,1);
}

void TrajImgMap()
{
  geneWall(-0.5, -1.0, 0, 1.5, 1.5, 2.0);
  geneWall(-0.5, -1.0, 0, 1.5, 1.5, 2.0);
  geneWall(-0.5, -1.0, 0, 1.5, 1.5, 2.0);
}

void map2Gene(){}
void map3Gene(){}
void map4Gene(){}
void map5Gene(){}
void map6Gene(){}
void map7Gene(){}
void map8Gene(){}
void map9Gene(){}

void GenerateMap(int id)
{
    if(id == 0){FileMapGenerate();}
    if(id == 1){NarrowSlitGenerate();}
    if(id == 2){SingleWallGenerate();}
    if(id == 3){RandomMapGenerate();}
    if(id == 4){rectFrameGenerate();}
    if(id == 5){benchmark_forest();}
    if(id == 6){changableMapGenerate();}
    if(id == 7){TrajImgMap();}
    if(id == 8){benchmarkMapGenerate();}
    if(id == 9){map9Gene();}

    _has_map = true;
    pcl::toROSMsg(global_map_pcl_cloud, globalMap_pcd);
    globalMap_pcd.header.frame_id = "odom";
    ROS_INFO("global map published! ");
}

void block_0_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    double px, py, pz;
    px = msg->pose.pose.position.x;
    py = msg->pose.pose.position.y;
    pz = msg->pose.pose.position.z;

    double length = 1.0;
    double width  = 1.5;
    double height = 2.0;

    if(changableMapUsed)
    {
      if(!isBlock0Got)
      {
        ROS_INFO("iamhereeeeeeeeeeeeeeeeeeeeee");
        geneWall(px, py, pz-height, length, width, height);
        isBlock0Got = true;
        pcl::toROSMsg(global_map_pcl_cloud, globalMap_pcd);
        globalMap_pcd.header.frame_id = "odom";
      }
    }
}

void block_1_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    double px, py, pz;
    px = msg->pose.pose.position.x;
    py = msg->pose.pose.position.y;
    pz = msg->pose.pose.position.z;


    double length = 1.0;
    double width  = 1.5;
    double height = 2.0;

    if(changableMapUsed)
    {
      if(!isBlock1Got)
      {
        geneWall(px, py, pz-height, length, width, height);
        isBlock1Got = true;
        pcl::toROSMsg(global_map_pcl_cloud, globalMap_pcd);
        globalMap_pcd.header.frame_id = "odom";
      }
    }
}

void block_2_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    double px, py, pz;
    px = msg->pose.pose.position.x;
    py = msg->pose.pose.position.y;
    pz = msg->pose.pose.position.z;

    double length = 1.0;
    double width  = 1.5;
    double height = 2.0;

    if(changableMapUsed)
    {
      if(!isBlock2Got)
      {
        geneWall(px, py, pz-height, length, width, height);
        isBlock2Got = true;
        pcl::toROSMsg(global_map_pcl_cloud, globalMap_pcd);
        globalMap_pcd.header.frame_id = "odom";
      }
    }
}

void pubSensedPoints()
{     
   if( !_has_map ) return;

   _all_map_pub.publish(globalMap_pcd);
}

void pubChangableMap()
{
  static ros::Time st = ros::Time::now();
  ros::Time nt = ros::Time::now();
  if((nt-st).toSec() >= 5)
  {
    global_map_pcl_cloud.clear();
    isBlock0Got = false;
    isBlock1Got = false;
    isBlock2Got = false;
    _has_map = true;
    pcl::toROSMsg(global_map_pcl_cloud, globalMap_pcd);
    globalMap_pcd.header.frame_id = "odom";
    ROS_INFO("global map published! ");
    st = nt;
  }
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "complex_map");
    ros::NodeHandle n( "~" );

    _all_map_pub    = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);
    _block_0_sub    = n.subscribe("/vrpn_client_node/block_0/pose", 40, block_0_callback, ros::TransportHints().tcpNoDelay());
    _block_1_sub    = n.subscribe("/vrpn_client_node/block_1/pose", 40, block_1_callback, ros::TransportHints().tcpNoDelay());
    _block_2_sub    = n.subscribe("/vrpn_client_node/block_2/pose", 40, block_2_callback, ros::TransportHints().tcpNoDelay());

    n.param("cloud_resolution", cloud_resolution, 0.2);
    n.param("sensing/rate", _sense_rate, 1.0);

    n.param("map_id", map_id, 0);

    // map in file
    n.param("pcd_path",   pcdPath,    string("/home/tgj/PathFinding/Two/catkin_ws/src/grid_path_searcher/pcd/map.pcd"));

    // map of narrow slit
    n.param("slit_width", slit_width, 2.0);

    // map of single wall
    n.param("wall_lenth", wall_lenth, 3.0);
    n.param("wall_width", wall_width, 3.0);
    n.param("wall_hight", wall_hight, 6.0);

    // random map
    n.param("random_num", random_num, 20.0);

    // rectFrame map
    n.param("x_l",          x_l,        20.0);
    n.param("y_l",          y_l,        20.0);
    n.param("z_l",          z_l,        20.0);
    n.param("x_h",          x_h,        20.0);
    n.param("y_h",          y_h,        20.0);
    n.param("z_h",          z_h,        20.0);
    n.param("x_m",          x_m,        20.0);
    n.param("y_m",          y_m,        20.0);
    n.param("z_m",          z_m,        20.0);
    n.param("len",          len,        20.0);
    n.param("height",       height,     20.0);
    n.param("theta",        theta,      20.0);
    n.param("gap_size",     gap_size,   20.0);
    theta = theta / 180 * M_PI;

    GenerateMap(map_id);

    // ros::Publisher pcl_pub = n.advertise<sensor_msgs::PointCloud2> ("pcl_output",1);
    
    // std::string file_path;
    // n.param<std::string>("file_path", file_path, "/home/wangjunjie/map/map_init_3.pcd");
    // pcl::io::loadPCDFile(file_path,cloud);//通过launch文件修改路径即可

    // geneWall_111(-10.0, -5.5, 0, 20.0, 0.2, 1.0);
    // geneWall_111(-10.0, -5.3, 0, 0.2, 11.0, 1.0);
    // geneWall_111(-10.0, 5.5, 0, 20.0, 0.2, 1.0);
    // geneWall_111(10.0, -5.5, 0, 0.2, 11.0, 1.0);
 
    // pcl::toROSMsg(cloud,output);
    // output.header.frame_id="odom";



    ros::Rate loop_rate(_sense_rate);
    while (ros::ok())
    {
        if(map_id == 6)
        {
          pubChangableMap();
        }
        // pcl_pub.publish(output);
        pubSensedPoints();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
