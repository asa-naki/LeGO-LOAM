#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_


#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cloud_msgs/msg/cloud_info.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include "pcl/filters/impl/filter.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_datatypes.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

#define PI 3.14159265

using namespace std;

template < typename T >
double ROS_TIME(T msg)
{
  return msg->header.stamp.sec +
         msg->header.stamp.nanosec * 1e-9;
}


typedef pcl::PointXYZI  PointType;

class ParamServer : public rclcpp::Node
{
public:
  string pointCloudTopic;
  string imuTopic;
  string odomTopic;

  // Lidar Sensor Configuration
  int N_SCAN;
  int Horizon_SCAN;
  float ang_res_x;
  float ang_res_y;
  bool useCloudRing;
  float ang_bottom;
  int groundScanInd;
  float sensorMountAngle;
  int segmentValidPointNum;
  int segmentValidLineNum;
  float surfThreshold;
  float edgeThreshold;
  int nearestFeatureSearchSqDist;
  float scanPeriod;
  // int imuQueLength;

  ParamServer(std::string node_name, const rclcpp::NodeOptions & options)
    : Node(node_name, options)
  {
    declare_parameter("pointCloudTopic", "points_raw");
    get_parameter("pointCloudTopic", pointCloudTopic);
    declare_parameter("imuTopic", "imu_correct");
    get_parameter("imuTopic", imuTopic);
    declare_parameter("odomTopic", "preintegrated_odom");
    get_parameter("odomTopic", odomTopic);

    declare_parameter("N_SCAN", 16);
    get_parameter("N_SCAN", N_SCAN);
    declare_parameter("Horizon_SCAN", 1800);
    get_parameter("Horizon_SCAN", Horizon_SCAN);
    declare_parameter("ang_res_x",0.2);
    get_parameter("ang_res_x",ang_res_x);
    declare_parameter("ang_res_y",2.0);
    get_parameter("ang_res_y",ang_res_y);
    declare_parameter("useCloudRing",true);
    get_parameter("useCloudRing",useCloudRing);
    declare_parameter("ang_bottom",15.0+0.1);
    get_parameter("ang_bottom",ang_bottom);
    declare_parameter("groundScanInd",7);
    get_parameter("groundScanInd",groundScanInd);
    declare_parameter("sensorMountAngle",0.0);
    get_parameter("sensorMountAngle",sensorMountAngle);
    declare_parameter("segmentValidPointNum",5);
    get_parameter("segmentValidPointNum",segmentValidPointNum);
    declare_parameter("segmentValidLineNum",3);
    get_parameter("segmentValidLineNum",segmentValidLineNum);
    declare_parameter("nearestFeatureSearchSqDist",25);
    get_parameter("nearestFeatureSearchSqDist",nearestFeatureSearchSqDist);
    declare_parameter("edgeThreshold",5.0);
    get_parameter("edgeThreshold",edgeThreshold);
    declare_parameter("surfThreshold",0.1);
    get_parameter("surfThreshold",surfThreshold);
    declare_parameter("scanPeriod",0.1);
    get_parameter("scanPeriod",scanPeriod);
    // declare_parameter("imuQueLength",200);
    // get_parameter("imuQueLength",imuQueLength);

};
/*
extern const string pointCloudTopic = "/velodyne_points";
// extern const string imuTopic = "/imu/data";

// Save pcd
extern const string fileDirectory = "/home/asahi/";

// Using velodyne cloud "ring" channel for image projection (other lidar may have different name for this channel, change "PointXYZIR" below)
extern const bool useCloudRing = true; // if true, ang_res_y and ang_bottom are not used

// VLP-16
extern const int N_SCAN = 16;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 2.0;
extern const float ang_bottom = 15.0+0.1;
extern const int groundScanInd = 7;

// HDL-32E
// extern const int N_SCAN = 32;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 41.33/float(N_SCAN-1);
// extern const float ang_bottom = 30.67;
// extern const int groundScanInd = 20;

// VLS-128
// extern const int N_SCAN = 128;
// extern const int Horizon_SCAN = 1800;
// extern const float ang_res_x = 0.2;
// extern const float ang_res_y = 0.3;
// extern const float ang_bottom = 25.0;
// extern const int groundScanInd = 10;

// Ouster users may need to uncomment line 159 in imageProjection.cpp
// Usage of Ouster imu data is not fully supported yet (LeGO-LOAM needs 9-DOF IMU), please just publish point cloud data
// Ouster OS1-16
// extern const int N_SCAN = 16;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 7;

// Ouster OS1-64
// extern const int N_SCAN = 64;
// extern const int Horizon_SCAN = 1024;
// extern const float ang_res_x = 360.0/float(Horizon_SCAN);
// extern const float ang_res_y = 33.2/float(N_SCAN-1);
// extern const float ang_bottom = 16.6+0.1;
// extern const int groundScanInd = 15;

extern const bool loopClosureEnableFlag = false;
extern const double mappingProcessInterval = 0.3;

extern const float scanPeriod = 0.1;
extern const int systemDelay = 0;
extern const int imuQueLength = 200;

extern const float sensorMinimumRange = 1.0;
extern const float sensorMountAngle = 0.0;
extern const float segmentTheta = 60.0/180.0*M_PI; // decrese this value may improve accuracy
extern const int segmentValidPointNum = 5;
extern const int segmentValidLineNum = 3;
extern const float segmentAlphaX = ang_res_x / 180.0 * M_PI;
extern const float segmentAlphaY = ang_res_y / 180.0 * M_PI;


extern const int edgeFeatureNum = 2;
extern const int surfFeatureNum = 4;
extern const int sectionsTotal = 6;
extern const float edgeThreshold = 0.1;
extern const float surfThreshold = 0.1;
extern const float nearestFeatureSearchSqDist = 25;


// Mapping Params
extern const float surroundingKeyframeSearchRadius = 50.0; // key frame that is within n meters from current pose will be considerd for scan-to-map optimization (when loop closure disabled)
extern const int   surroundingKeyframeSearchNum = 50; // submap size (when loop closure enabled)
// history key frames (history submap for loop closure)
extern const float historyKeyframeSearchRadius = 7.0; // key frame that is within n meters from current pose will be considerd for loop closure
extern const int   historyKeyframeSearchNum = 25; // 2n+1 number of hostory key frames will be fused into a submap for loop closure
extern const float historyKeyframeFitnessScore = 0.3; // the smaller the better alignment

extern const float globalMapVisualizationSearchRadius = 500.0; // key frames with in n meters will be visualized

typedef PointXYZIRPYT  PointTypePose;
*/
};
#endif
