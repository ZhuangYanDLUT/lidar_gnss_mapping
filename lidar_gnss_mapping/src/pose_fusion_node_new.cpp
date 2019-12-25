#include <cmath>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <tf/transform_datatypes.h>

#include "gil_slam_test/odom_and_ctrl.h"
#include "gil_slam_test/odom_and_status.h"
#include "math_process.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

///////////////////////

#include <pcl_ros/point_cloud.h>

#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <iostream>
///////////////////////

typedef geometry_msgs::Point ROSPoi3d;
typedef geometry_msgs::Vector3 ROSVec3d;
#ifndef __pi__
#define __pi__
const double PI = 3.141592654;
#endif
/*
reset_flag_from_pose_fusion
 0 里程计信息由GPS发出
 1 里程计信息从LOAM切换至GPS时GPS提供的里程计信息
 2 里程计信息由LOAM发出
 3 里程计信息从LOAM切换至GPS时LOAM提供的里程计信息
*/
/////////////////////////////////////////////////////////////////////改能完成发送最后odom位姿
/**********gps/ins变量***********/
nav_msgs::Odometry curGpsInsOdom;
double gitime;
ROSPoi3d giposi;
ROSVec3d girpy;
int gistatus;
int gisatnum;
/**********gps/ins变量***********/
bool newGpsIns(false);
const int gpsInsQueLen(1000);
double gpsInsTime[gpsInsQueLen];
ROSPoi3d gpsInsPosi[gpsInsQueLen];
ROSVec3d gpsInsRPY[gpsInsQueLen];
int gpsInsStatus[gpsInsQueLen];
int gpsInsSatNum[gpsInsQueLen];
int gpsInsNew(0);

/**********融合后的位姿***********/
const int fusedQueLen(200);
const int lidarQueLen(200);
double fusedTime[fusedQueLen];
ROSPoi3d fusedPosi[fusedQueLen];
ROSVec3d fusedRPY[fusedQueLen];
int fusedNew(0);

/*********************************************改*************************************************/
int flag(0);                           //延迟改变由loam切换到gps的标志位，让标志位出现在第一个gps位姿里
ROSVec3d tmp_Posi[fusedQueLen];
ROSPoi3d tmp_fusedPosi;
int gps_odom_count(0);
int flag_gps_odom(1);
int count_mapping(0);
int mapping_pose_finish_flag(0);
float xa, ya, za;
bool isfristtimeofodom(true);
double starttimeifodom(0);
ROSPoi3d deltapositionofodom,positionofodom,lidarMappingPosi[lidarQueLen];
ROSVec3d deltaRPYofodom,RPYofodom,lidarMappingRPY[lidarQueLen];
/********mappingOdom变量*********/
double lidarTime[lidarQueLen];
int lastLidarNewMapping(0);
int lidarMappingNew(0);
bool isfristin(true);
int issendlastodompose(0);
double timeofodom(0);
nav_msgs::Odometry curlidarMapping;
bool newlidarmapping(false);
int lidarNewMappingReplaceOdom(0);
ROSPoi3d lidarMappingPosiReplaceOdom[lidarQueLen];
ROSVec3d lidarMappingRPYReplaceOdom[lidarQueLen];
double lidarMappingTimeReplaceOdom[lidarQueLen];
int GPSstablecounter(0);

const int  startTimeWhileGpsSendMapping(10);	//有GPS时同时跑LOAM的开始时刻
const int  endTimeWhileGpsSendMapping(20);		//有GPS时同时跑LOAM的开始时刻
const int  correctAngleofGpscounter(200);
const int  correctAngleOfMappingcounter(100);
//ROSVec3d startpositionX;
/*****************************/
pcl::PointCloud<pcl::PointXYZINormal>::Ptr GpsInsPoseToangle(new pcl::PointCloud<pcl::PointXYZINormal>());
pcl::PointCloud<pcl::PointXYZINormal>::Ptr OdomPoseToangle(new pcl::PointCloud<pcl::PointXYZINormal>());
/*********************************************改*************************************************/
/**********其他变量***********/
ros::Publisher *pub_pose(NULL);
ros::Publisher *pub_odompose(NULL);

double theta;
void GPSINSCallback(const gil_slam_test::odom_and_status msgIn) {
  gitime = msgIn.header.stamp.toSec();
  giposi = msgIn.odom.pose.pose.position;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(msgIn.odom.pose.pose.orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(girpy.x, girpy.y, girpy.z);
  /* 消除两轨迹夹角 */

  ROSPoi3d tm_shift;
  //theta =(-0.304)*PI ; //即为-54.70度 loam轨迹和gps轨迹的夹角-PI / 4 - 0.18
  //theta =(2.6278)*PI;	//7-22xiaoyuan
  //theta = (-0.5)*PI;		//1900
  //theta=(-0.5)*PI;
  //theta = (0.5833)*PI;
  //theta = (0.4442)*PI;//hou
  //theta = (-0.5)*PI;
  //theta = (0.1024)*PI; //7-17zhulou  bag包使用的角度值
  //theta = (-0.1862)*PI;  //7-17zhulou  -s 566
  // theta = (-0.1054)*PI;  //7-17xiaoyuan  bag包使用角度值
  theta = 0;
  
  tm_shift.x = cos(theta) * giposi.x + sin(theta) * giposi.y;
  tm_shift.y = -sin(theta) * giposi.x + cos(theta) * giposi.y;
  tm_shift.z = giposi.z;
  
  giposi.x = tm_shift.y;//坐标轴转换到和loam相同
  giposi.y = tm_shift.z;
  giposi.z = tm_shift.x;
  /* 消除两轨迹夹角结束 */

  

  ROSVec3d tm_rpy;
  tm_rpy.x = girpy.y;
  tm_rpy.y = girpy.z ; // 调整gps/ins姿态角，主要调整yaw角，使和loam重合
  tm_rpy.z = girpy.x;
  girpy = tm_rpy;

  gistatus = msgIn.status;
  gisatnum = msgIn.sat_num;
  newGpsIns = true;

}
int isfristrimeinodom(0);
int reset_flag_from_lidar_odom(0);
bool begin_loam_odom(true);
void loamBeginCallback(const nav_msgs::Odometry odomMappingIn){
	if(odomMappingIn.pose.pose.position.x == 1111){
		begin_loam_odom = true;
	}
}

//***********************************加接收mapping计算到的位姿*****************************//
void lidarMappingOdomCallback(const nav_msgs::Odometry odomMappingIn)
{
	//curlidarOdom.header = odomMappingIn.header;
	newlidarmapping = true;
	//*************************接受loam的mapping的位姿.作用：用于接下来的角度的计算*****************************//
	curlidarMapping = odomMappingIn;
	double rolll, pitchh, yaww;
	geometry_msgs::Quaternion geoQuat1 = odomMappingIn.pose.pose.orientation;
	tf::Matrix3x3(tf::Quaternion(geoQuat1.z, -geoQuat1.x, -geoQuat1.y, geoQuat1.w)).getRPY(rolll, pitchh, yaww);
	lidarMappingNew = (lidarMappingNew + 1) % lidarQueLen;
	lidarMappingRPY[lidarMappingNew].x = -pitchh;
	lidarMappingRPY[lidarMappingNew].y = -yaww;
	lidarMappingRPY[lidarMappingNew].z = rolll;
	lidarMappingPosi[lidarMappingNew] = odomMappingIn.pose.pose.position;
	//*************************用于代替odom使用mapping，作用：代替odom计算的位姿，以后gps没信号的时候不用odom而是用mapping的位姿*******************//
	
	lidarNewMappingReplaceOdom = (lidarNewMappingReplaceOdom + 1) % lidarQueLen;
	lidarMappingRPYReplaceOdom[lidarNewMappingReplaceOdom].x = -pitchh;
	lidarMappingRPYReplaceOdom[lidarNewMappingReplaceOdom].y = -yaww;
	lidarMappingRPYReplaceOdom[lidarNewMappingReplaceOdom].z = rolll;
  	lidarMappingPosiReplaceOdom[lidarNewMappingReplaceOdom] = odomMappingIn.pose.pose.position;
  	lidarMappingTimeReplaceOdom[lidarNewMappingReplaceOdom] = odomMappingIn.header.stamp.toSec();


  	//printf("M:%.2f %.2f %.2f\n",odomMappingIn.pose.pose.position.x,odomMappingIn.pose.pose.position.y,odomMappingIn.pose.pose.position.z);
  
	//*************************进行发出有gps一段时间的loam位姿估计*****************************//
	timeofodom = odomMappingIn.header.stamp.toSec();

	//printf("time=stamp=of=mapping=pose = :%d\n",timeofodom);
	
  //ROS_INFO("exit mapping callback");       
}
int main(int argc, char **argv) {
	ros::init(argc, argv, "pose_fusion_node");
	ros::NodeHandle nh;
	ros::Subscriber subGpsInsOdomAndStatus = nh.subscribe<gil_slam_test::odom_and_status>("/odom_and_status", 500,GPSINSCallback);
	//ros::Subscriber subLidarOdom = nh.subscribe<gil_slam_test::odom_and_ctrl>("/laser_odom_to_init", 50, lidarOdomCallback);
	ros::Subscriber subLidarMappingOdom = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 50, lidarMappingOdomCallback);//aft_mapped_to_init
	ros::Subscriber subLoamBegin = nh.subscribe<nav_msgs::Odometry>("/begin_loam",5,loamBeginCallback);
	pub_odompose = new ros::Publisher(nh.advertise<gil_slam_test::odom_and_ctrl>("/while_gps_use_odom_to_init", 50));
	pub_pose = new ros::Publisher(nh.advertise<gil_slam_test::odom_and_ctrl>("/aft_fused_to_init", 50));
	ros::Publisher *pub_pose2 = new ros::Publisher(nh.advertise<nav_msgs::Odometry>("/rviz_aft_fused_to_init", 50));

	OdomPoseToangle->resize(500);
	GpsInsPoseToangle->resize(500);
	bool isFirst(true);
	const bool gps(false);
	const bool lidar(true);
	int backCount(-1);
	ROSPoi3d gpsDelPosi, lidarDelPosi, deltaPosi;
	ROSVec3d gpsDelRPY, lidarDelRPY, deltaRPY;
	int lastGpsInsNew;
	int lastLidarNew;
	int lastLidarMapping;
	int lastFusedNew;
	bool lastIs(gps);
	bool curIs;
	int fused_time(0.0);
	int reset_flag_from_pose_fusion(0);
	ros::Rate rate(200);
	while (ros::ok()) {
		ros::spinOnce();
		// if (newGpsIns && (newGpsIns || newlidarmapping) )//这意思是必须有gps的时候才能进，有mapping的时候必须和gps一起进  (newlidarOdom && newGpsIns) //newlidarOdom || newlidarmapping) && newGpsIns
		if(newGpsIns && newlidarmapping)
		{    
			newGpsIns = false;
			newlidarmapping = false;
			reset_flag_from_pose_fusion = 0;
			if (isFirst) //原来是if(first)
			{
				//如果是第一个点，需要初始化
				fusedTime[fusedNew] = lidarMappingTimeReplaceOdom[lidarNewMappingReplaceOdom];
				fusedPosi[fusedNew] = lidarMappingPosiReplaceOdom[lidarNewMappingReplaceOdom];
				fusedRPY[fusedNew] = lidarMappingRPYReplaceOdom[lidarNewMappingReplaceOdom];
				curIs = lidar;
				gpsInsTime[gpsInsNew] = gitime;
				gpsInsPosi[gpsInsNew] = giposi;
				gpsInsRPY[gpsInsNew] = girpy;
				gpsInsStatus[gpsInsNew] = gistatus;
				gpsInsSatNum[gpsInsNew] = gisatnum;
				//发布第一个点
				printf("using lidar to init: %i\n", curIs);
				nav_msgs::Odometry newOdom;
				newOdom.header = curlidarMapping.header;
				newOdom.child_frame_id = curlidarMapping.child_frame_id;
				newOdom.pose.pose.position = fusedPosi[fusedNew];
				geometry_msgs::Quaternion geoQuat;
				geoQuat = tf::createQuaternionMsgFromRollPitchYaw(
				fusedRPY[fusedNew].z, -fusedRPY[fusedNew].x, -fusedRPY[fusedNew].y);
				newOdom.pose.pose.orientation.x = -geoQuat.y;
				newOdom.pose.pose.orientation.y = -geoQuat.z;
				newOdom.pose.pose.orientation.z = geoQuat.x;
				newOdom.pose.pose.orientation.w = geoQuat.w;
				newOdom.twist.twist.angular.x = curIs;
				gil_slam_test::odom_and_ctrl oc_msg;
				oc_msg.odom = newOdom;
				oc_msg.ctrl_flag = reset_flag_from_pose_fusion;
				pub_pose->publish(oc_msg);

				isFirst = false;
				printf("初始化成功\n");
				continue;
			}

			//保存和lidar odom时间戳对应的gps/ins信息
			gpsInsNew = (gpsInsNew + 1) % gpsInsQueLen;
			gpsInsTime[gpsInsNew] = gitime;
			gpsInsPosi[gpsInsNew] = giposi;
			gpsInsRPY[gpsInsNew] = girpy;
			gpsInsStatus[gpsInsNew] = gistatus;
			gpsInsSatNum[gpsInsNew] = gisatnum;

			//if((gpsInsStatus[gpsInsNew] == 4 ||gpsInsStatus[gpsInsNew] == 5)&&(timeofodom < 100 ||(int)(timeofodom/150)%2 == 0))// && gpsInsSatNum[gpsInsNew] >= 11)
			//if((gpsInsStatus[gpsInsNew] == 4 ||gpsInsStatus[gpsInsNew] == 5) && (timeofodom<130 || (timeofodom > 380 &&(timeofodom<430)||(timeofodom >640))))// && gpsInsSatNum[gpsInsNew] >= 11)
			if(gpsInsStatus[gpsInsNew] == 4 ||gpsInsStatus[gpsInsNew] == 5)//若GPS信号准确
			{ 	

				GPSstablecounter = 0;
				if (lastIs == gps){////上一帧是gps
					begin_loam_odom = false;
					curIs = gps;
					printf("using gps: %i\n", curIs);
					fusedNew = (fusedNew + 1) % fusedQueLen;
					fusedPosi[fusedNew] = gpsInsPosi[gpsInsNew];
					fusedRPY[fusedNew] = gpsInsRPY[gpsInsNew];
					fusedTime[fusedNew] = gpsInsTime[gpsInsNew];

					//send gps msg	 
					nav_msgs::Odometry newOdom;
					newOdom.header = curlidarMapping.header;
					newOdom.child_frame_id = curlidarMapping.child_frame_id;
					newOdom.pose.pose.position = fusedPosi[fusedNew];
					geometry_msgs::Quaternion geoQuat;
					geoQuat = tf::createQuaternionMsgFromRollPitchYaw(gpsInsRPY[gpsInsNew].z, -gpsInsRPY[gpsInsNew].x,-gpsInsRPY[gpsInsNew].y);
					newOdom.pose.pose.orientation.x = -geoQuat.y;
					newOdom.pose.pose.orientation.y = -geoQuat.z;
					newOdom.pose.pose.orientation.z = geoQuat.x;
					newOdom.pose.pose.orientation.w = geoQuat.w;
					newOdom.twist.twist.angular.x = curIs; // 0 for gps
					gil_slam_test::odom_and_ctrl oc_msg1;
					oc_msg1.odom = newOdom;
					oc_msg1.ctrl_flag = 0;
					pub_pose->publish(oc_msg1);
					pub_pose2->publish(newOdom);
					lastIs = gps;//更新lastIs变量，表明上一帧是什么
					continue;
				} 
				else {//若上一帧不是GPS
					if(backCount<= 60){
						printf("using lidar \n");
						backCount++;
						lastIs = lidar;
						reset_flag_from_pose_fusion = 2;	//yxd add
						curIs = lidar;
						lastLidarMapping = (lidarNewMappingReplaceOdom - 1 + lidarQueLen) % lidarQueLen;
						deltaPosi.x = (lidarMappingPosiReplaceOdom[lidarNewMappingReplaceOdom].x - lidarMappingPosiReplaceOdom[lastLidarMapping].x)*0.94;
						deltaPosi.y = (lidarMappingPosiReplaceOdom[lidarNewMappingReplaceOdom].y - lidarMappingPosiReplaceOdom[lastLidarMapping].y)*0.94;
						deltaPosi.z = (lidarMappingPosiReplaceOdom[lidarNewMappingReplaceOdom].z - lidarMappingPosiReplaceOdom[lastLidarMapping].z)*0.94;
						deltaRPY.x = lidarMappingRPYReplaceOdom[lidarNewMappingReplaceOdom].x - lidarMappingRPYReplaceOdom[lastLidarMapping].x;
						deltaRPY.y = lidarMappingRPYReplaceOdom[lidarNewMappingReplaceOdom].y - lidarMappingRPYReplaceOdom[lastLidarMapping].y;
						deltaRPY.z = lidarMappingRPYReplaceOdom[lidarNewMappingReplaceOdom].z - lidarMappingRPYReplaceOdom[lastLidarMapping].z;
						fused_time = lidarTime[lidarMappingNew];
						
						////////////////////////
						lastFusedNew = fusedNew;
						fusedNew = (fusedNew + 1) % fusedQueLen;
						fusedPosi[fusedNew].x = fusedPosi[lastFusedNew].x + deltaPosi.x;
						fusedPosi[fusedNew].y = fusedPosi[lastFusedNew].y + deltaPosi.y;
						fusedPosi[fusedNew].z = fusedPosi[lastFusedNew].z + deltaPosi.z;
						fusedRPY[fusedNew].x = fusedRPY[lastFusedNew].x + deltaRPY.x;
						fusedRPY[fusedNew].y = fusedRPY[lastFusedNew].y + deltaRPY.y;
						fusedRPY[fusedNew].z = fusedRPY[lastFusedNew].z + deltaRPY.z;
						fusedTime[fusedNew] = fused_time;

						//信息发布
						nav_msgs::Odometry newOdom;
						newOdom.header = curlidarMapping.header;
						newOdom.child_frame_id = curlidarMapping.child_frame_id;
						newOdom.pose.pose.position = fusedPosi[fusedNew];
						//切换到gps时，使用gps绝对位置替换融合累计的位置

						geometry_msgs::Quaternion geoQuat;
						geoQuat = tf::createQuaternionMsgFromRollPitchYaw(fusedRPY[fusedNew].z, -fusedRPY[fusedNew].x, -fusedRPY[fusedNew].y);
						newOdom.pose.pose.orientation.x = -geoQuat.y;
						newOdom.pose.pose.orientation.y = -geoQuat.z;
						newOdom.pose.pose.orientation.z = geoQuat.x;
						newOdom.pose.pose.orientation.w = geoQuat.w;
						newOdom.twist.twist.angular.x = curIs; // 0 for gps
						gil_slam_test::odom_and_ctrl oc_msg1;
						oc_msg1.odom = newOdom;
						oc_msg1.ctrl_flag = reset_flag_from_pose_fusion;

						pub_pose->publish(oc_msg1);
						pub_pose2->publish(newOdom);
						continue;
					}
					else if (backCount > 60){//延时切换	 
						printf("\n\n\n切换\n\n\n");
						lastIs = gps;
						reset_flag_from_pose_fusion = 1;
						backCount = -1;
						curIs = lidar;

						nav_msgs::Odometry newOdom;
						newOdom.header = curlidarMapping.header;
						newOdom.child_frame_id = curlidarMapping.child_frame_id;

						lastLidarMapping = (lidarNewMappingReplaceOdom - 1 + lidarQueLen) % lidarQueLen;
						deltaPosi.x = lidarMappingPosiReplaceOdom[lidarNewMappingReplaceOdom].x - lidarMappingPosiReplaceOdom[lastLidarMapping].x;
						deltaPosi.y = lidarMappingPosiReplaceOdom[lidarNewMappingReplaceOdom].y - lidarMappingPosiReplaceOdom[lastLidarMapping].y;
						deltaPosi.z = lidarMappingPosiReplaceOdom[lidarNewMappingReplaceOdom].z - lidarMappingPosiReplaceOdom[lastLidarMapping].z;
						deltaRPY.x = lidarMappingRPYReplaceOdom[lidarNewMappingReplaceOdom].x - lidarMappingRPYReplaceOdom[lastLidarMapping].x;
						deltaRPY.y = lidarMappingRPYReplaceOdom[lidarNewMappingReplaceOdom].y - lidarMappingRPYReplaceOdom[lastLidarMapping].y;
						deltaRPY.z = lidarMappingRPYReplaceOdom[lidarNewMappingReplaceOdom].z - lidarMappingRPYReplaceOdom[lastLidarMapping].z;
						fused_time = lidarTime[lidarMappingNew];
						//loam
						lastFusedNew = fusedNew;
						fusedNew = (fusedNew + 1) % fusedQueLen;
						fusedPosi[fusedNew].x = fusedPosi[lastFusedNew].x + deltaPosi.x;
						fusedPosi[fusedNew].y = fusedPosi[lastFusedNew].y + deltaPosi.y;
						fusedPosi[fusedNew].z = fusedPosi[lastFusedNew].z + deltaPosi.z;
						fusedRPY[fusedNew].x = fusedRPY[lastFusedNew].x + deltaRPY.x;
						fusedRPY[fusedNew].y = fusedRPY[lastFusedNew].y + deltaRPY.y;
						fusedRPY[fusedNew].z = fusedRPY[lastFusedNew].z + deltaRPY.z;
						fusedTime[fusedNew] = fused_time;
						
						newOdom.pose.pose.position = fusedPosi[fusedNew];
						geometry_msgs::Quaternion geoQuat;
						geoQuat = tf::createQuaternionMsgFromRollPitchYaw(
							fusedRPY[fusedNew].z, -fusedRPY[fusedNew].x,
							-fusedRPY[fusedNew].y);
						newOdom.pose.pose.orientation.x = -geoQuat.y;
						newOdom.pose.pose.orientation.y = -geoQuat.z;
						newOdom.pose.pose.orientation.z = geoQuat.x;
						newOdom.pose.pose.orientation.w = geoQuat.w;
						newOdom.twist.twist.angular.x = curIs; // 0 for gps
						gil_slam_test::odom_and_ctrl oc_msg1;
						oc_msg1.odom = newOdom;
						oc_msg1.ctrl_flag = 3;
						pub_pose->publish(oc_msg1);
						pub_pose2->publish(newOdom);
						//切换时刻GPS信息
						lastFusedNew = fusedNew;
						fusedNew = (fusedNew + 1) % fusedQueLen;
						fusedPosi[fusedNew] = gpsInsPosi[gpsInsNew];
						fusedRPY[fusedNew] = gpsInsRPY[gpsInsNew];
						fusedTime[fusedNew] = gpsInsTime[gpsInsNew];

						newOdom.pose.pose.position = fusedPosi[fusedNew];
						//geometry_msgs::Quaternion geoQuat;
						geoQuat = tf::createQuaternionMsgFromRollPitchYaw(
							fusedRPY[fusedNew].z, -fusedRPY[fusedNew].x,
							-fusedRPY[fusedNew].y);
						newOdom.pose.pose.orientation.x = -geoQuat.y;
						newOdom.pose.pose.orientation.y = -geoQuat.z;
						newOdom.pose.pose.orientation.z = geoQuat.x;
						newOdom.pose.pose.orientation.w = geoQuat.w;
						newOdom.twist.twist.angular.x = curIs; // 0 for gps
						//gil_slam_test::odom_and_ctrl oc_msg1;
						oc_msg1.odom = newOdom;
						oc_msg1.ctrl_flag = 1;
						pub_pose->publish(oc_msg1);
						pub_pose2->publish(newOdom);
						continue;
					}

				}
			}
			else if(!begin_loam_odom && backCount == -1){
				gil_slam_test::odom_and_ctrl oc_msg1;
				oc_msg1.ctrl_flag = 4;
				pub_pose->publish(oc_msg1);
				std::cout<<"1111111111111111111111111111111"<<std::endl;
				continue;
			}
			else if(begin_loam_odom){//GPS信号不好使用Mapping
				curIs = lidar;
				lastIs = lidar;
				printf("using lidar, no gps: %i\n", curIs);
				if (backCount == -1) 
				{	
					//gil_slam_test::odom_and_ctrl oc_msg1;
					backCount = 0;
					continue;
				}
				lastLidarMapping = (lidarNewMappingReplaceOdom - 1 + lidarQueLen) % lidarQueLen;
				deltaPosi.x = (lidarMappingPosiReplaceOdom[lidarNewMappingReplaceOdom].x - lidarMappingPosiReplaceOdom[lastLidarMapping].x)*0.95;
				deltaPosi.y = (lidarMappingPosiReplaceOdom[lidarNewMappingReplaceOdom].y - lidarMappingPosiReplaceOdom[lastLidarMapping].y)*0.95;
				deltaPosi.z = (lidarMappingPosiReplaceOdom[lidarNewMappingReplaceOdom].z - lidarMappingPosiReplaceOdom[lastLidarMapping].z)*0.95;

				deltaRPY.x = lidarMappingRPYReplaceOdom[lidarNewMappingReplaceOdom].x - lidarMappingRPYReplaceOdom[lastLidarMapping].x;
				deltaRPY.y = lidarMappingRPYReplaceOdom[lidarNewMappingReplaceOdom].y - lidarMappingRPYReplaceOdom[lastLidarMapping].y;
				deltaRPY.z = lidarMappingRPYReplaceOdom[lidarNewMappingReplaceOdom].z - lidarMappingRPYReplaceOdom[lastLidarMapping].z;

				fused_time = lidarTime[lidarMappingNew];
			//	lastIs = lidar;
				reset_flag_from_pose_fusion = 2;
				//////////////////////////
				lastFusedNew = fusedNew;
				fusedNew = (fusedNew + 1) % fusedQueLen;
				fusedPosi[fusedNew].x = fusedPosi[lastFusedNew].x + deltaPosi.x;
				fusedPosi[fusedNew].y = fusedPosi[lastFusedNew].y + deltaPosi.y;
				fusedPosi[fusedNew].z = fusedPosi[lastFusedNew].z + deltaPosi.z;
				fusedRPY[fusedNew].x = fusedRPY[lastFusedNew].x + deltaRPY.x;
				fusedRPY[fusedNew].y = fusedRPY[lastFusedNew].y + deltaRPY.y;
				fusedRPY[fusedNew].z = fusedRPY[lastFusedNew].z + deltaRPY.z;
				fusedTime[fusedNew] = fused_time;

				//信息发布
				nav_msgs::Odometry newOdom;
				newOdom.header = curlidarMapping.header;
				newOdom.child_frame_id = curlidarMapping.child_frame_id;
				newOdom.pose.pose.position = fusedPosi[fusedNew];
				//切换到gps时，使用gps绝对位置替换融合累计的位置

				geometry_msgs::Quaternion geoQuat;
				geoQuat = tf::createQuaternionMsgFromRollPitchYaw(fusedRPY[fusedNew].z, -fusedRPY[fusedNew].x, -fusedRPY[fusedNew].y);
				newOdom.pose.pose.orientation.x = -geoQuat.y;
				newOdom.pose.pose.orientation.y = -geoQuat.z;
				newOdom.pose.pose.orientation.z = geoQuat.x;
				newOdom.pose.pose.orientation.w = geoQuat.w;
				newOdom.twist.twist.angular.x = curIs; // 0 for gps
				gil_slam_test::odom_and_ctrl oc_msg1;
				oc_msg1.odom = newOdom;
				oc_msg1.ctrl_flag = reset_flag_from_pose_fusion;

				pub_pose->publish(oc_msg1);
				pub_pose2->publish(newOdom);
			}
		}// if (newlidarOdom && newGpsIns)结束
		rate.sleep();
	}//while 结束
	delete pub_pose;
	return 0;
}
