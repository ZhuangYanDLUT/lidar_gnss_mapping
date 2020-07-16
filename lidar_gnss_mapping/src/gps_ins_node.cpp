#include <cstdio>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "gnss_process.hpp"
#include "lidar_gnss_mapping/odom_and_status.h"
#include "lidar_gnss_mapping/gpgga_msg.h"

#ifndef __pi__
#define __pi__
const double PI = 3.141592654;
#endif

typedef geometry_msgs::Vector3 ROSVec3d;
typedef geometry_msgs::Point ROSPoi3d;

bool newGps(false);
double gpsInTime;
double lat, lon, alt;
double last_lat, last_lon, last_alt;
int status, satNum;

bool newImu(false);
bool firstImu(true);
double imuInTime;
ROSVec3d imuInAcc;
ROSVec3d imuInAngle;
geometry_msgs::Quaternion orien;

const int imuQueLen(1000);
double imuTime[imuQueLen] = {0.0};
ROSVec3d imuRPY[imuQueLen];
ROSVec3d imuAcc[imuQueLen];
ROSVec3d imuVelo[imuQueLen];
ROSPoi3d imuShift[imuQueLen];
ROSVec3d imuRPYVel[imuQueLen];
int imuNew(0);

const int gpsQueLen(100);
double gpsTime[gpsQueLen] = {0.0};
ROSPoi3d gpsShift[gpsQueLen];
int gpsNew(0);

const int gpsAftQueLen(200);
double gpsAftTime[gpsAftQueLen];
ROSPoi3d gpsAftShift[gpsAftQueLen];
ROSVec3d gpsAftVelo[gpsAftQueLen];
int gpsAftNew(0);

ROSPoi3d firstPoint;
ROSVec3d firstRPY;
bool isFirst(true);
bool haveNoGpsData(true);
double timeDiff(0.0);
double sinLat, cosLat, sinLon, cosLon; 

ROSVec3d newestStableRpy;
ROSPoi3d newestStablePoi;

ros::Publisher *pubOdom(NULL), *pubOdom1(NULL);
ros::Publisher *pubOdomAndStatus(NULL);
ros::Publisher *pubPoint(NULL);
// float resetloam[6] = {0};
bool new_delta_theta(false);
geometry_msgs::Quaternion geoQuat;
// void loamDeltaTheta(const nav_msgs::Odometry odom){
//   new_delta_theta = true;
//   resetloam[0] = odom.pose.pose.orientation.x;   //roll
//   resetloam[1] = odom.pose.pose.orientation.y;   //pitch
//   resetloam[2] = odom.pose.pose.orientation.z;   //yaw

//   resetloam[3] = odom.pose.pose.position.x;    //x
//   resetloam[4] = odom.pose.pose.position.y;    //y
//   resetloam[5] = odom.pose.pose.position.z;    //z
// }
void GPSCallback(const lidar_gnss_mapping::gpgga_msg::ConstPtr &gpsIn) {
  gpsInTime = gpsIn->header.stamp.toSec();
  lat = gpsIn->latitude; // Unit: degree
  lon = gpsIn->longitude;//
  alt = gpsIn->altitude; // Unit: metre
  status = gpsIn->GPSstatus;
  satNum = gpsIn->satelliteNum;
  newGps = true;
}
void IMUCallback(const sensor_msgs::Imu::ConstPtr &imuIn) {
  imuInTime = imuIn->header.stamp.toSec();
  orien = imuIn->orientation;
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw); //rad

  double accX = (imuIn->linear_acceleration.x + 9.81 * sin(pitch));
  double accY = (imuIn->linear_acceleration.y - 9.81 * sin(roll) * cos(pitch));
  double accZ = (imuIn->linear_acceleration.z - 9.81 * cos(roll) * cos(pitch));

  imuNew = (imuNew + 1) % imuQueLen;
  imuTime[imuNew] = imuInTime;
  imuRPY[imuNew].x = roll;
  imuRPY[imuNew].y = pitch;
  imuRPY[imuNew].z = yaw;
  imuRPYVel[imuNew].x = imuIn->angular_velocity.x;
  imuRPYVel[imuNew].y = imuIn->angular_velocity.y;
  imuRPYVel[imuNew].z = imuIn->angular_velocity.z;

  // printf("%f %f %f\n", imuRPY[imuNew].x, imuRPY[imuNew].y, imuRPY[imuNew].z);
  imuAcc[imuNew].x = accX;
  imuAcc[imuNew].y = accY;
  imuAcc[imuNew].z = accZ;
  newImu = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_ins_node");
  ros::NodeHandle nh;
  ros::Subscriber subGPS =
      nh.subscribe<lidar_gnss_mapping::gpgga_msg>("/gnss/data", 1000, GPSCallback);
  ros::Subscriber subIMU =
      nh.subscribe<sensor_msgs::Imu>("/imu/data", 1000, IMUCallback);
  // ros::Subscriber subDeltaTheta = nh.subscribe<nav_msgs::Odometry>("/delta_theta",100,loamDeltaTheta);
  pubOdom = new ros::Publisher(
      nh.advertise<nav_msgs::Odometry>("/gps_ins_odom", 1000));
  pubOdomAndStatus = new ros::Publisher(
      nh.advertise<lidar_gnss_mapping::odom_and_status>("/odom_and_status", 1000));

  ros::Rate rate(300);
  while (ros::ok()) {
    ros::spinOnce();
    if (newGps && newImu) { 
      newImu = false;
      newGps = false;
      //init
      if (isFirst) {
        if (status !=4 && status !=5) { // No GPS signal
          nav_msgs::Odometry odom;
          odom.header.stamp = ros::Time().fromSec(gpsAftTime[gpsAftNew]);
          odom.header.frame_id = "/camera_init";
          odom.pose.pose.position.x = 0.0;
          odom.pose.pose.position.y = 0.0;
          odom.pose.pose.position.z = 0.0;
          odom.pose.pose.orientation = orien;
          pubOdom->publish(odom);

          lidar_gnss_mapping::odom_and_status odom_status;
          odom_status.header.stamp = ros::Time().fromSec(imuTime[imuNew]);
          odom_status.header.frame_id = "/camera_init";
          odom_status.odom.header.stamp = ros::Time().fromSec(imuTime[imuNew]);
          odom_status.odom.header.frame_id = "/camera_init";
          odom_status.odom.pose.pose.position.x = 0.0;
          odom_status.odom.pose.pose.position.y = 0.0;
          odom_status.odom.pose.pose.position.z = 0.0;
          odom_status.odom.pose.pose.orientation = orien;
          odom_status.status = 0;
          odom_status.sat_num = satNum;
          pubOdomAndStatus->publish(odom_status);
        } else {
          //use the pose obtained from the latest GNSS and IMU to initialize
          std::cout<<"GNSS init "<<std::endl;
          double Xd, Yd, Zd; // ENU
          sinLat = sin(lat * PI / 180);
          cosLat = cos(lat * PI / 180);
          sinLon = sin(lon * PI / 180);
          cosLon = cos(lon * PI / 180);
          BLH2ENU(lat, lon, alt, Xd, Yd, Zd, sinLat, cosLat, sinLon, cosLon);
          firstPoint.x = Xd;
          firstPoint.y = Yd;
          firstPoint.z = Zd;
          firstRPY.x = imuRPY[imuNew].x;
          firstRPY.y = imuRPY[imuNew].y;
          firstRPY.z = imuRPY[imuNew].z;
          gpsTime[gpsNew] = gpsInTime; 
          gpsShift[gpsNew].x = 0.0;    
          gpsShift[gpsNew].y = 0.0;
          gpsShift[gpsNew].z = 0.0;
          newestStablePoi = gpsShift[gpsNew];
          isFirst = false;
          last_lat = lat;
          last_lon = lon;
          last_alt = alt;
        }
        continue;
      }
      if(status== 4 || status == 5){
        if(lat!=0 && lon!=0){
          last_lat = lat;
          last_lon = lon;
          last_alt = alt;
          double Xe, Ye, Ze; // ENU
          BLH2ENU(lat, lon, alt, Xe, Ye, Ze, sinLat, cosLat, sinLon, cosLon);
          gpsNew = (gpsNew + 1) % gpsQueLen;
          gpsTime[gpsNew] = gpsInTime;
          gpsShift[gpsNew].x = Xe;
          gpsShift[gpsNew].y = Ye;
          gpsShift[gpsNew].z = Ze;

          gpsShift[gpsNew].z =(1 - fabs(2 * atan(gpsShift[gpsNew].z)) / PI) * gpsShift[gpsNew].z;

          ROSPoi3d inPosition, outPosition;
          //kalman filter
          inPosition = gpsShift[gpsNew];
          KalmanFun(imuAcc[imuNew], inPosition, 0.1, outPosition, status);
          gpsAftNew = (gpsAftNew + 1) % gpsAftQueLen;
          gpsAftTime[gpsAftNew] = gpsTime[gpsNew];
          gpsAftShift[gpsAftNew] = outPosition;

          int gpsAftBack = (gpsAftNew - 1 + gpsAftQueLen) % gpsAftQueLen;
          double timeDiff = gpsAftTime[gpsAftNew] - gpsAftTime[gpsAftBack];
          if (timeDiff < 0.3) {
            gpsAftVelo[gpsAftNew].x =
                (gpsAftShift[gpsAftNew].x - gpsAftShift[gpsAftBack].x) / timeDiff;
            gpsAftVelo[gpsAftNew].y =
                (gpsAftShift[gpsAftNew].y - gpsAftShift[gpsAftBack].y) / timeDiff;
            gpsAftVelo[gpsAftNew].z =
                (gpsAftShift[gpsAftNew].z - gpsAftShift[gpsAftBack].z) / timeDiff;
          }
          // Reset imu navi parameter
          imuShift[imuNew] = gpsAftShift[gpsAftNew];
          imuVelo[imuNew] = gpsAftVelo[gpsAftNew];

          static nav_msgs::Odometry odom;
          odom.header.stamp = ros::Time().fromSec(gpsAftTime[gpsAftNew]);
          odom.header.frame_id = "/camera_init";
          odom.pose.pose.position = gpsAftShift[gpsAftNew];
          odom.pose.pose.orientation = orien;
          pubOdom->publish(odom);

          static lidar_gnss_mapping::odom_and_status odom_status;
          odom_status.header.stamp = ros::Time().fromSec(imuTime[imuNew]);
          odom_status.header.frame_id = "/camera_init";
          odom_status.odom.header.stamp = ros::Time().fromSec(imuTime[imuNew]);
          odom_status.odom.header.frame_id = "/camera_init";
          odom_status.odom.pose.pose.position = gpsAftShift[gpsAftNew];
          odom_status.odom.pose.pose.orientation = orien;
          odom_status.status = status;
          odom_status.sat_num = satNum;
          pubOdomAndStatus->publish(odom_status);
          if (haveNoGpsData)
            haveNoGpsData = false;
        }else{
          continue;
        }
      }else{
        static nav_msgs::Odometry odom;
          odom.header.stamp = ros::Time().fromSec(gpsAftTime[gpsAftNew]);
          odom.header.frame_id = "/camera_init";
          odom.pose.pose.position = gpsAftShift[gpsAftNew];
          odom.pose.pose.orientation = orien;
          pubOdom->publish(odom);

          static lidar_gnss_mapping::odom_and_status odom_status;
          odom_status.header.stamp = ros::Time().fromSec(imuTime[imuNew]);
          odom_status.header.frame_id = "/camera_init";
          odom_status.odom.header.stamp = ros::Time().fromSec(imuTime[imuNew]);
          odom_status.odom.header.frame_id = "/camera_init";
          odom_status.odom.pose.pose.position = gpsAftShift[gpsAftNew];
          odom_status.odom.pose.pose.orientation = orien;
          odom_status.status = status;
          odom_status.sat_num = satNum;
          pubOdomAndStatus->publish(odom_status);
      }
    }
    //Interpolate GNSS data using imu data
    if (!newGps && newImu) { 
      if (haveNoGpsData)
        continue;
      newImu = false;
      double roll = imuRPY[imuNew].x;
      double pitch = imuRPY[imuNew].y;
      double yaw = imuRPY[imuNew].z;
      double accX = imuAcc[imuNew].x;
      double accY = imuAcc[imuNew].y;
      double accZ = imuAcc[imuNew].z;
      //Convert acceleration to world coordinate system(ENU)
      double x1 = accX;
      double y1 = cos(roll) * accY - sin(roll) * accZ;
      double z1 = sin(roll) * accY + cos(roll) * accZ;

      double x2 = cos(pitch) * x1 + sin(pitch) * z1;
      double y2 = y1;
      double z2 = -sin(pitch) * x1 + cos(pitch) * z1;

      accX = cos(yaw) * x2 - sin(yaw) * y2;
      accY = sin(yaw) * x2 + cos(yaw) * y2;
      accZ = z2;

      int imuBack = (imuNew - 1 + imuQueLen) % imuQueLen;
      double timeDiff = imuTime[imuNew] - imuTime[imuBack];
      if (timeDiff < 0.02) { // nearly==0.01sec

        imuShift[imuNew].x = imuShift[imuBack].x +
                             imuVelo[imuBack].x * timeDiff +
                             accX * timeDiff * timeDiff / 2;
        imuShift[imuNew].y = imuShift[imuBack].y +
                             imuVelo[imuBack].y * timeDiff +
                             accY * timeDiff * timeDiff / 2;
        imuShift[imuNew].z = imuShift[imuBack].z +
                             imuVelo[imuBack].z * timeDiff +
                             accZ * timeDiff * timeDiff / 2;
        imuVelo[imuNew].x = imuVelo[imuBack].x + accX * timeDiff;
        imuVelo[imuNew].y = imuVelo[imuBack].y + accY * timeDiff;
        imuVelo[imuNew].z = imuVelo[imuBack].z + accZ * timeDiff;
      }

      static nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time().fromSec(imuTime[imuNew]);
      odom.header.frame_id = "/camera_init";
      odom.pose.pose.position = imuShift[imuNew];
      odom.pose.pose.orientation = orien;
      pubOdom->publish(odom);

      static lidar_gnss_mapping::odom_and_status odom_status;
      odom_status.header.stamp = ros::Time().fromSec(imuTime[imuNew]);
      odom_status.header.frame_id = "/camera_init";
      odom_status.odom.header.stamp = ros::Time().fromSec(imuTime[imuNew]);
      odom_status.odom.header.frame_id = "/camera_init";
      odom_status.odom.pose.pose.position = imuShift[imuNew];
      odom_status.odom.pose.pose.orientation = orien;
      odom_status.status = status;
      odom_status.sat_num = satNum;
      pubOdomAndStatus->publish(odom_status);
    }
    rate.sleep();
  }
  delete pubOdom;
  delete pubOdomAndStatus;
  return 0;
}