#include <fstream>
#include <string>
#include <iomanip>
#include <thread>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <tf/transform_datatypes.h>

#include "rtk_slam/odom_and_ctrl.h"
#include "rtk_slam/odom_and_status.h"
#include "math_process.hpp"

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pointType.h"

pcl::PointCloud<PointType>::Ptr laserCloudFullRes;

std::string str_head = "/home/yxd/outdoor_ws/src/rtk_slam/launch/data/10/";
std::string str_end = "2.txt";
bool first_recv(true);
bool first_loam_recv(true);
bool first_optimize_recv(true);
bool write_pointcloud_to_file_XYZI(string str,pcl::PointCloud<PointType>::Ptr cloudIn){
	std::ofstream of(str,std::ios::app);
	if(!of.is_open()){
		std::cout<<"open file "<<str<<" error!"<<std::endl;
		return false;
	}
	for(int i=0;i<cloudIn->points.size();++i){
		PointType *pointTmp;
		pointTmp = &cloudIn->points[i];
		of<<pointTmp->x<<" "<<pointTmp->y<<" "<<pointTmp->z<<" "<<pointTmp->intensity<<std::endl;
	}
	of.close();
	return true;
}
bool write_pointcloud_to_file_XYZ(string str,pcl::PointCloud<PointType>::Ptr cloudIn){
	std::ofstream of(str,std::ios::app);
	if(!of.is_open()){
		std::cout<<"open file "<<str<<" error!"<<std::endl;
		return false;
	}
	for(int i=0;i<cloudIn->points.size();++i){
		PointType *pointTmp;
		pointTmp = &cloudIn->points[i];
		of<<pointTmp->x<<" "<<pointTmp->y<<" "<<pointTmp->z<<std::endl;
	}
	of.close();
	return true;
}

void keyPoseHandler(const sensor_msgs::PointCloud2ConstPtr & keyPoseIn){
	if(first_recv){
		first_recv = false;
		laserCloudFullRes.reset(new pcl::PointCloud<PointType>());
		pcl::fromROSMsg(*keyPoseIn,*laserCloudFullRes);	//从点云数据类型转换为PointType
		std::string str = str_head + "KeyPose_XYZ_"+str_end;
		std::cout<<'\n'<<"write key pose to "<<str<<std::endl;
		write_pointcloud_to_file_XYZ(str,laserCloudFullRes);
		std::cout<<"finish wirte key"<<std::endl;
	}
}
void loamPoseHandler(const sensor_msgs::PointCloud2ConstPtr & loamPoseIn){
	if(first_loam_recv){
		first_loam_recv = false;
		laserCloudFullRes.reset(new pcl::PointCloud<PointType>());
		pcl::fromROSMsg(*loamPoseIn,*laserCloudFullRes);	//从点云数据类型转换为PointType
		std::string str = str_head + "LoamPose_XYZ_"+str_end;
		std::cout<<'\n'<<"write loam pose to "<<str<<std::endl;
		write_pointcloud_to_file_XYZ(str,laserCloudFullRes);
		std::cout<<"finish write loam"<<std::endl;
	}
}
void optimizePoseHandler(const sensor_msgs::PointCloud2ConstPtr & optimizePoseIn){
	if(first_optimize_recv){
		first_optimize_recv = false;
		laserCloudFullRes.reset(new pcl::PointCloud<PointType>());
		pcl::fromROSMsg(*optimizePoseIn,*laserCloudFullRes);	//从点云数据类型转换为PointType
		std::string str = str_head + "Optimize_Pose_XYZ_"+str_end;
		std::cout<<'\n'<<"write optimize pose to "<<str<<std::endl;
		write_pointcloud_to_file_XYZ(str,laserCloudFullRes);
		std::cout<<"finish write optimize"<<std::endl;
	}
}
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2){
	//timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

	laserCloudFullRes.reset(new pcl::PointCloud<PointType>());

	pcl::fromROSMsg(*laserCloudFullRes2,*laserCloudFullRes);	//从点云数据类型转换为PointType
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*laserCloudFullRes,*laserCloudFullRes,indices);//去除点云中的无效点
	std::string str = str_head + "pointcoud"+str_end;
	write_pointcloud_to_file_XYZ(str,laserCloudFullRes);
}
int main(int argc,char**argv){
	ros::init(argc,argv,"write_file");
	ros::NodeHandle nh;
	ros::Subscriber subLaserCloudFullRes =
			nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround_rtk", 2, laserCloudFullResHandler);
			//nh.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);
	ros::Subscriber subKeyPose =
			nh.subscribe<sensor_msgs::PointCloud2>("/key_pose_origin", 2, keyPoseHandler);

	ros::Subscriber subLoamPose =
			nh.subscribe<sensor_msgs::PointCloud2>("/loam_pose", 2, loamPoseHandler);
	
	ros::Subscriber subOptimizePoses = nh.subscribe<sensor_msgs::PointCloud2>("/optimize_pose",2,optimizePoseHandler);
	ros::Rate rate(10);
	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
}