
#include <fstream>
#include <string>
#include <iomanip>
#include <thread>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <tf/transform_datatypes.h>

#include "lidar_gnss_mapping/odom_and_ctrl.h"
#include "lidar_gnss_mapping/odom_and_status.h"
#include "math_process.hpp"

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>

#include <pcl/visualization/pcl_visualizer.h>
#include "loopOptimize.h"

using namespace gtsam;
/*
	laser_gps.intensity = 0 GNSS data available
	laser_gps.intensity = 1 GNSS-Constrained LiDAR-Mapping switch to LiDAR-only (GNSS data)
	laser_gps.intensity = 2 LiDAR only modle
	laser_gps.intensity = 3 GNSS-Constrained LiDAR-Mapping switch to LiDAR-only (LiDAR-only  data)
*/

pcl::PointCloud<PointTypePose>::Ptr odomKeyPoses6D;				//(x y z pitch yaw roll)
pcl::PointCloud<PointTypePose>::Ptr odomBeforOptimizePoses6D;	
pcl::PointCloud<PointTypePose>::Ptr odomAfterOptimizePoses6D;	
pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;				
pcl::PointCloud<PointType>::Ptr optimizePose3D;					
pcl::PointCloud<PointType>::Ptr loamPose3D;						
PointTypePose laser_gps_test;

std::deque<pcl::PointCloud<PointType>::Ptr>recentLidarCloud;	
std::deque<double>timeLaser;									
std::deque<PointTypePose>recentOdom;		
std::deque<double>timeOdometry;									
pcl::PointCloud<PointType>::Ptr laserCloudFullRes;
pcl::PointCloud<PointType>::Ptr globalMapKeyFrames;
pcl::PointCloud<PointType>::Ptr pubGlobalMap;

ros::Publisher pubLaserCloudSurround_rtk;
ros::Publisher pubLoamDeltaTheta;


bool newdata(false);
bool begindata(false);
int reset_flag_from_fusion = 0;
int begin_current_id = 0;			//the first index  when switching from GNSS modle to lidar only
int end_loam_index = 0;				//the last index in lidar only modle
double timeLaserOdometry = 0;
double timeLaserCloudFullRes = 0;
float position_s = 0.9;
float angle_s = 0.0;

vector<double>update_theta;		
vector<double>begin_time;		
vector<double>end_time;			
double final_theta=0.0;
double latest_theta = 0.0;
double test_angle = 0;

int charges = 0;
int count_index = 0;
PointType end_curretn;

loamGtsam LG;
double delta_theta = 0;
int begin_index=3,end_index = 0;//坐标系对齐第一个index

bool write_pointcloud_to_file(string str,pcl::PointCloud<PointType>::Ptr cloudIn){
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
pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn){

    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;
    PointType pointTo;

    int cloudSize = cloudIn->points.size();
    cloudOut->resize(cloudSize);
    for (int i = 0; i < cloudSize; ++i){

        pointFrom = &cloudIn->points[i];
        float x1 = cos(transformIn->yaw) * pointFrom->x - sin(transformIn->yaw) * pointFrom->y;
        float y1 = sin(transformIn->yaw) * pointFrom->x + cos(transformIn->yaw)* pointFrom->y;
        float z1 = pointFrom->z;

        float x2 = x1;
        float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
        float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll)* z1;

        pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 + transformIn->x;
        pointTo.y = y2 + transformIn->y;
        pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 + transformIn->z;
        pointTo.intensity = pointFrom->intensity;

        cloudOut->points[i] = pointTo;
    }

    return cloudOut;
}
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2){
	timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();
	laserCloudFullRes.reset(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr laserCloudTmp;
	laserCloudTmp.reset(new pcl::PointCloud<PointType>());

	pcl::fromROSMsg(*laserCloudFullRes2,*laserCloudFullRes);	
	for(auto i = 0;i<laserCloudFullRes->points.size();i++){
		PointType tmp= laserCloudFullRes->points[i];
		if(sqrt(tmp.x*tmp.x + tmp.y*tmp.y + tmp.z*tmp.z)<2.5){
			continue;
		}else{
			laserCloudTmp->push_back(tmp);
		}
	}
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*laserCloudTmp,*laserCloudTmp,indices);
	recentLidarCloud.push_back(laserCloudTmp);
	timeLaser.push_back(timeLaserCloudFullRes);	//timestamp
}

void transe(float q0, float q1, float q2, float q3,Eigen::Matrix3d &m){

	m(0,0) = 1-2*q2*q2 - 2*q3*q3;
    m(0,1) = 2*q1*q2 + 2*q0*q3;
    m(0,2) = 2*q1*q3 - 2*q0*q2;
    //
    m(1,0) = 2*q1*q2 - 2*q0*q3;
    m(1,1) = 1-2*q1*q1 - 2*q3*q3;
    m(1,2) = 2*q2*q3 + 2*q0*q1;
    //
    m(2,0) = 2*q1*q3 + 2*q0*q2;
    m(2,1) = 2*q2*q3 - 2*q0*q1;
    m(2,2) = 1 - 2*q1*q1 - 2*q2*q2;
}
void registerLidarGpsCallback(
	const lidar_gnss_mapping::odom_and_ctrl::ConstPtr &laserOdometry){
	newdata = true;
	timeLaserOdometry = laserOdometry->odom.header.stamp.toSec();	
	timeOdometry.push_back(timeLaserOdometry);
	reset_flag_from_fusion = laserOdometry->ctrl_flag;
	laser_gps_test.x = laserOdometry->odom.pose.pose.position.x;
	laser_gps_test.y = laserOdometry->odom.pose.pose.position.y;
	laser_gps_test.z = laserOdometry->odom.pose.pose.position.z;
	laser_gps_test.time = laserOdometry->odom.header.stamp.toSec();
	double roll, pitch, yaw;
 	geometry_msgs::Quaternion geoQuat = laserOdometry->odom.pose.pose.orientation;
  	tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w))
      .getRPY(roll, pitch, yaw);
    laser_gps_test.roll = -pitch;
    laser_gps_test.pitch = -yaw;
    laser_gps_test.yaw = roll;
    laser_gps_test.intensity = laserOdometry->ctrl_flag;
    laser_gps_test.index = odomKeyPoses6D->points.size();
    odomKeyPoses6D->push_back(laser_gps_test);		

    PointType thisPose3D;
    thisPose3D.x = laser_gps_test.x;
    thisPose3D.y = laser_gps_test.y;
    thisPose3D.z = laser_gps_test.z;
    thisPose3D.intensity = laser_gps_test.index;	
    cloudKeyPoses3D->push_back(thisPose3D);		
    PointType KeyPoint;
    
    int laser_index = 0;
    int odom_index = 0;
    double theta = 0;

    if(laserOdometry->update == true){
    	final_theta = 0;	

		update_theta.push_back(laserOdometry->theta);		
		begin_time.push_back(cloudKeyPoses3D->points.size());

    }
    else if(final_theta == 0){
    		final_theta = laserOdometry->theta;
    		begin_time.push_back(cloudKeyPoses3D->points.size());

    		end_index = begin_time[begin_time.size()-1];
    		int theta_index = 0;
    		pubGlobalMap.reset(new pcl::PointCloud<PointType>());

    		for(auto i = end_index-update_theta.size();i<end_index;++i){
				if(odomKeyPoses6D->points[i].intensity !=1){
					odomKeyPoses6D->points[i].pitch -= (final_theta - update_theta[theta_index]);
					theta_index++;
					for(auto k=0;k<timeLaser.size();++k){
						if(fabs(timeLaser[k] - timeOdometry[i])<0.05){
							globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
							*globalMapKeyFrames = *transformPointCloud(recentLidarCloud[k],&odomKeyPoses6D->points[i]);
							*pubGlobalMap += *globalMapKeyFrames;
							break;
						}
					}
				}
			}
			sensor_msgs::PointCloud2 cloudMsgTemp;
			globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
			pcl::toROSMsg(*pubGlobalMap, cloudMsgTemp);
			cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaser[timeLaser.size()-1]);
			cloudMsgTemp.header.frame_id = "/camera_init";
			pubLaserCloudSurround_rtk.publish(cloudMsgTemp);
    		begin_time.clear();	
			update_theta.clear();
    		count_index = end_index;
	
    }
    if(odomKeyPoses6D->points[laser_gps_test.index].intensity == 0 && final_theta!=0 && odomKeyPoses6D->points.size()>end_index+1){
    	for(auto i=0;i<timeLaser.size();++i){
    		if(fabs(timeLaserOdometry - timeLaser[i])<0.05){
    			laser_index = i;

    			pcl::PointCloud<PointType>::Ptr pointCloudTmp = recentLidarCloud[i];
    			globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());

    			*globalMapKeyFrames = *transformPointCloud(pointCloudTmp,&odomKeyPoses6D->points[odomKeyPoses6D->points.size()-1]);
    			
    			sensor_msgs::PointCloud2 cloudMsgTemp;
    			pcl::toROSMsg(*globalMapKeyFrames,cloudMsgTemp);
    			cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaser[i]);
		        cloudMsgTemp.header.frame_id = "/camera_init";
		        pubLaserCloudSurround_rtk.publish(cloudMsgTemp);  
		        break;
    		}
    	}
    	globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
    }

	laser_index = 0;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(laser_gps_test.index > 0 && odomKeyPoses6D->points[laser_gps_test.index].intensity == 2
		&& begindata == false){
    	begin_current_id = laser_gps_test.index;
		begindata = true;
    }
    if(laser_gps_test.index > 0 && (laser_gps_test.intensity == 2 ||
    	laser_gps_test.intensity == 3)){
    	odomBeforOptimizePoses6D->push_back(laser_gps_test);
    }
    if(laser_gps_test.index >0 && laser_gps_test.intensity == 1){
    	int end_index = odomBeforOptimizePoses6D->points.size();
		for(int i = 0;i<odomBeforOptimizePoses6D->points.size();++i){
			odomAfterOptimizePoses6D->push_back(odomBeforOptimizePoses6D->points[i]);
		}

		double delta_x_sum = (laser_gps_test.x - odomAfterOptimizePoses6D->points[end_index-1].x) * position_s;
		double delta_y_sum = (laser_gps_test.y - odomAfterOptimizePoses6D->points[end_index-1].y) * position_s;
		double delta_z_sum = (laser_gps_test.z - odomAfterOptimizePoses6D->points[end_index-1].z) * position_s;

		double rang_distance_all = 0.0;		
		for(int i = 1;i<odomAfterOptimizePoses6D->points.size(); ++i){
			rang_distance_all += sqrt((odomAfterOptimizePoses6D->points[i].x - odomAfterOptimizePoses6D->points[i-1].x)*
				(odomAfterOptimizePoses6D->points[i].x - odomAfterOptimizePoses6D->points[i-1].x) + 
				(odomAfterOptimizePoses6D->points[i].y - odomAfterOptimizePoses6D->points[i-1].y)*
				(odomAfterOptimizePoses6D->points[i].y - odomAfterOptimizePoses6D->points[i-1].y) + 
				(odomAfterOptimizePoses6D->points[i].z - odomAfterOptimizePoses6D->points[i-1].z)*
				(odomAfterOptimizePoses6D->points[i].z - odomAfterOptimizePoses6D->points[i-1].z));
		}
		double current_l = 0.0;
		std::vector<double> v_z;
		std::vector<double> v_x;
		std::vector<double> v_y;
		for(int i = 1;i<odomAfterOptimizePoses6D->points.size(); ++i){
			current_l += sqrt((odomBeforOptimizePoses6D->points[i].x - odomBeforOptimizePoses6D->points[i-1].x)*
				(odomBeforOptimizePoses6D->points[i].x - odomBeforOptimizePoses6D->points[i-1].x) + 
				(odomBeforOptimizePoses6D->points[i].y - odomBeforOptimizePoses6D->points[i-1].y)*
				(odomBeforOptimizePoses6D->points[i].y - odomBeforOptimizePoses6D->points[i-1].y) + 
				(odomBeforOptimizePoses6D->points[i].z - odomBeforOptimizePoses6D->points[i-1].z)*
				(odomBeforOptimizePoses6D->points[i].z - odomBeforOptimizePoses6D->points[i-1].z));
			odomAfterOptimizePoses6D->points[i].x += delta_x_sum*(current_l/rang_distance_all);
			odomAfterOptimizePoses6D->points[i].y += delta_y_sum*(current_l/rang_distance_all);
			odomAfterOptimizePoses6D->points[i].z += delta_z_sum*(current_l/rang_distance_all);
		}
		pcl::PointCloud<pcl::PointXYZI>::Ptr origin_pose(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr recent_pose(new pcl::PointCloud<pcl::PointXYZI>());

		origin_pose.reset(new pcl::PointCloud<pcl::PointXYZI>());
		recent_pose.reset(new pcl::PointCloud<pcl::PointXYZI>());
		double tmp_origin_x = odomBeforOptimizePoses6D->points[0].x;
		double tmp_origin_y = odomBeforOptimizePoses6D->points[0].y;
		double tmp_origin_z = odomBeforOptimizePoses6D->points[0].z;
		double delta_pitch = 0;
		double dist = 0;
		std::vector<double>v_dist;
		Eigen::Affine3f correctionCameraFrame;
		for(int i = 0;i<odomAfterOptimizePoses6D->points.size();++i){
			pcl::PointXYZI tmp_origin,tmp_recent;
			tmp_origin.x = odomBeforOptimizePoses6D->points[i].x - tmp_origin_x;
			tmp_origin.y = 0;
			tmp_origin.z = odomBeforOptimizePoses6D->points[i].z - tmp_origin_z;

			tmp_recent.x = odomAfterOptimizePoses6D->points[i].x - tmp_origin_x;
			tmp_recent.y = 0;
			tmp_recent.z = odomAfterOptimizePoses6D->points[i].z - tmp_origin_z;
			origin_pose->push_back(tmp_origin);
			recent_pose->push_back(tmp_recent);
			dist = sqrt((tmp_origin.x-tmp_recent.x)*(tmp_origin.x-tmp_recent.x)+
						(tmp_origin.y-tmp_recent.y)*(tmp_origin.y-tmp_recent.y)+
						(tmp_origin.z-tmp_recent.z)*(tmp_origin.z-tmp_recent.z));
			v_dist.push_back(dist);
		}
        //icp
        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setMaxCorrespondenceDistance(dist*0.8);
        icp.setMaximumIterations(500);
        icp.setTransformationEpsilon(1e-10);
        icp.setEuclideanFitnessEpsilon(0.01);    
        icp.setRANSACIterations(0);
        icp.setInputSource(origin_pose);
        icp.setInputTarget(recent_pose);
        pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result_icp(new pcl::PointCloud<pcl::PointXYZI>());
        icp.align(*unused_result_icp);//ICP

        float x,y,z,roll,pitch,yaw;
        if(icp.hasConverged() == true){
            correctionCameraFrame = icp.getFinalTransformation();
            pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw);
            test_angle = pitch;
        }else{
        	std::cout<<"ICP ERROR "<<std::endl;
        }
		end_loam_index = cloudKeyPoses3D->points.size() ;

		if(yaw != 0 || pitch != 0 || roll != 0){
			double xx = laser_gps_test.x - odomAfterOptimizePoses6D->points[end_index-1].x;
			double yy = laser_gps_test.y - odomAfterOptimizePoses6D->points[end_index-1].y;
			double zz = laser_gps_test.z - odomAfterOptimizePoses6D->points[end_index-1].z;

			double distance_all = 0.0;
			for(int i = 1;i<odomAfterOptimizePoses6D->points.size(); ++i){
				distance_all += sqrt((odomAfterOptimizePoses6D->points[i].x - odomAfterOptimizePoses6D->points[i-1].x)*
					(odomAfterOptimizePoses6D->points[i].x - odomAfterOptimizePoses6D->points[i-1].x) + 
					(odomAfterOptimizePoses6D->points[i].y - odomAfterOptimizePoses6D->points[i-1].y)*
					(odomAfterOptimizePoses6D->points[i].y - odomAfterOptimizePoses6D->points[i-1].y) + 
					(odomAfterOptimizePoses6D->points[i].z - odomAfterOptimizePoses6D->points[i-1].z)*
					(odomAfterOptimizePoses6D->points[i].z - odomAfterOptimizePoses6D->points[i-1].z));
			}
			double test_detal_distance;
			double detal_distance_all = 0.0;
			for(int i = 1;i<odomAfterOptimizePoses6D->points.size(); ++i){
				test_detal_distance = sqrt((odomAfterOptimizePoses6D->points[i].x - odomAfterOptimizePoses6D->points[i-1].x)*
					(odomAfterOptimizePoses6D->points[i].x - odomAfterOptimizePoses6D->points[i-1].x) + 
					(odomAfterOptimizePoses6D->points[i].y - odomAfterOptimizePoses6D->points[i-1].y)*
					(odomAfterOptimizePoses6D->points[i].y - odomAfterOptimizePoses6D->points[i-1].y) + 
					(odomAfterOptimizePoses6D->points[i].z - odomAfterOptimizePoses6D->points[i-1].z)*
					(odomAfterOptimizePoses6D->points[i].z - odomAfterOptimizePoses6D->points[i-1].z));

				detal_distance_all += test_detal_distance;
				double yaw_tmp = yaw ;     
	      		double pitch_tmp = pitch ;     
	        	double roll_tmp = roll ;     
	        	float postw,posti,postj,postk;
				postw = cos(roll_tmp/2.0)*cos(pitch_tmp/2.0)*cos(yaw_tmp/2.0) + sin(roll_tmp/2.0)*sin(pitch_tmp/2.0)*sin(yaw_tmp/2.0);
				posti = sin(roll_tmp/2.0)*cos(pitch_tmp/2.0)*cos(yaw_tmp/2.0) - cos(roll_tmp/2.0)*sin(pitch_tmp/2.0)*sin(yaw_tmp/2.0);
				postj = cos(roll_tmp/2.0)*sin(pitch_tmp/2.0)*cos(yaw_tmp/2.0) + sin(roll_tmp/2.0)*cos(pitch_tmp/2.0)*sin(yaw_tmp/2.0);
				postk = cos(roll_tmp/2.0)*cos(pitch_tmp/2.0)*sin(yaw_tmp/2.0) - sin(roll_tmp/2.0)*sin(pitch_tmp/2.0)*cos(yaw_tmp/2.0);
				Eigen::Matrix3d Matrix_err(3,3);		
				Eigen::Matrix3d error_inv(3,3);

				transe(postw,posti,postj,postk,Matrix_err);
				error_inv = Matrix_err.inverse();
				Eigen::Matrix3d Matrix_err_xyz(3,3);
				Matrix_err_xyz(0,0)=odomAfterOptimizePoses6D->points[i].x - odomAfterOptimizePoses6D->points[0].x;
	    		Matrix_err_xyz(0,1)=odomAfterOptimizePoses6D->points[i].y - odomAfterOptimizePoses6D->points[0].y;
	    		Matrix_err_xyz(0,2)=odomAfterOptimizePoses6D->points[i].z - odomAfterOptimizePoses6D->points[0].z;

				Eigen::Matrix3d Matrix_xyz(3,3);

				Matrix_xyz = error_inv * Matrix_err_xyz;
			    odomAfterOptimizePoses6D->points[i].yaw += yaw_tmp;
			    odomAfterOptimizePoses6D->points[i].pitch += pitch_tmp;
			    odomAfterOptimizePoses6D->points[i].roll += roll_tmp;
				
			}
    	}
		for(int i=0;i<odomAfterOptimizePoses6D->points.size(); ++i){
			LG.odometry.x = odomAfterOptimizePoses6D->points[i].x;
			LG.odometry.y = odomAfterOptimizePoses6D->points[i].y;
			LG.odometry.z = odomAfterOptimizePoses6D->points[i].z;
			LG.odometry.time = odomAfterOptimizePoses6D->points[i].time;
			LG.odometry.roll = odomAfterOptimizePoses6D->points[i].roll;
			LG.odometry.pitch = odomAfterOptimizePoses6D->points[i].pitch;
			LG.odometry.yaw = odomAfterOptimizePoses6D->points[i].yaw;
			LG.odometry.intensity = odomAfterOptimizePoses6D->points[i].intensity;
			LG.recv_data();
			LG.path_optimize();
		}
		LG.odometry.x = laser_gps_test.x;
		LG.odometry.y = laser_gps_test.y;
		LG.odometry.z = laser_gps_test.z;
		LG.odometry.time = laser_gps_test.time;

		LG.odometry.roll = odomAfterOptimizePoses6D->points[odomAfterOptimizePoses6D->points.size()-1].roll;
		LG.odometry.pitch = odomAfterOptimizePoses6D->points[odomAfterOptimizePoses6D->points.size()-1].pitch;
		LG.odometry.yaw = odomAfterOptimizePoses6D->points[odomAfterOptimizePoses6D->points.size()-1].yaw;
		LG.odometry.intensity = laser_gps_test.intensity;
		LG.recv_data();
		LG.path_optimize();
		int tmp_time_index = 0;
		if(LG.save_result()){
			int j=0;
			for(int i=0;i<LG.currentOdomKeyPoses6D->points.size();++i){
				j = i+begin_current_id;
				PointType thisPose;
				thisPose.x = LG.currentOdomKeyPoses6D->points[i].x;
				thisPose.y = LG.currentOdomKeyPoses6D->points[i].y;
				thisPose.z = LG.currentOdomKeyPoses6D->points[i].z;

				optimizePose3D->push_back(thisPose);		
				thisPose.x = cloudKeyPoses3D->points[j].x;
				thisPose.y = cloudKeyPoses3D->points[j].y;
				thisPose.z = cloudKeyPoses3D->points[j].z;
				loamPose3D->push_back(thisPose);		
				cloudKeyPoses3D->points[j].x = LG.currentOdomKeyPoses6D->points[i].x;
				cloudKeyPoses3D->points[j].y = LG.currentOdomKeyPoses6D->points[i].y;
				cloudKeyPoses3D->points[j].z = LG.currentOdomKeyPoses6D->points[i].z;
				odomKeyPoses6D->points[j].x = LG.currentOdomKeyPoses6D->points[i].x;
				odomKeyPoses6D->points[j].y = LG.currentOdomKeyPoses6D->points[i].y;
				odomKeyPoses6D->points[j].z = LG.currentOdomKeyPoses6D->points[i].z;
				odomKeyPoses6D->points[j].roll = LG.currentOdomKeyPoses6D->points[i].roll;
				odomKeyPoses6D->points[j].pitch = LG.currentOdomKeyPoses6D->points[i].pitch;
				odomKeyPoses6D->points[j].yaw = LG.currentOdomKeyPoses6D->points[i].yaw;
				if(odomKeyPoses6D->points[j].intensity != 1){
					for(auto k=0;k<timeLaser.size();++k){
						if(fabs(timeLaser[k] - timeOdometry[j])<0.005){
							globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
	    					*globalMapKeyFrames = *transformPointCloud(recentLidarCloud[k],&odomKeyPoses6D->points[j]);
	    					*pubGlobalMap += *globalMapKeyFrames;
				       		break;
						}
					}
				}
			}
			sensor_msgs::PointCloud2 cloudMsgTemp;
			pcl::toROSMsg(*pubGlobalMap, cloudMsgTemp);
			cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaser[timeLaser.size()-1]);
			cloudMsgTemp.header.frame_id = "/camera_init";
			pubLaserCloudSurround_rtk.publish(cloudMsgTemp);
			globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
			begin_index = begin_current_id+LG.currentOdomKeyPoses6D->points.size();
			nav_msgs::Odometry odom;
			odom.header.frame_id = "/camera_init";
			odom.pose.pose.orientation.x = odomKeyPoses6D->points[LG.currentOdomKeyPoses6D->points.size()-1+begin_current_id].roll - roll;
			odom.pose.pose.orientation.y = odomKeyPoses6D->points[LG.currentOdomKeyPoses6D->points.size()-1+begin_current_id].pitch - pitch;
			odom.pose.pose.orientation.z = odomKeyPoses6D->points[LG.currentOdomKeyPoses6D->points.size()-1+begin_current_id].yaw - yaw;
			odom.pose.pose.position.x = odomKeyPoses6D->points[LG.currentOdomKeyPoses6D->points.size()-1+begin_current_id].x;
			odom.pose.pose.position.y = odomKeyPoses6D->points[LG.currentOdomKeyPoses6D->points.size()-1+begin_current_id].y;
			odom.pose.pose.position.z = odomKeyPoses6D->points[LG.currentOdomKeyPoses6D->points.size()-1+begin_current_id].z;

			pubLoamDeltaTheta.publish(odom);
			begindata = false;
			odomBeforOptimizePoses6D.reset(new pcl::PointCloud<PointTypePose>());
			odomAfterOptimizePoses6D.reset(new pcl::PointCloud<PointTypePose>());
			for(int k = 0;k<tmp_time_index;++k){
				timeLaser.pop_front();
				recentLidarCloud.pop_front();
			}
		}

    }

}

int main(int argc, char**argv){
	ros::init(argc, argv, "register_lidar_gps");
	ros::NodeHandle nh;
	odomKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
	odomBeforOptimizePoses6D.reset(new pcl::PointCloud<PointTypePose>());
	odomAfterOptimizePoses6D.reset(new pcl::PointCloud<PointTypePose>());
	cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
	optimizePose3D.reset(new pcl::PointCloud<PointType>());
	loamPose3D.reset(new pcl::PointCloud<PointType>());
	globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());

	ros::Subscriber subLaserCloudFullRes = 
		nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 2, laserCloudFullResHandler);
	ros::Subscriber subRegisterLidarGps = 
		nh.subscribe<lidar_gnss_mapping::odom_and_ctrl>("/laser_and_odom_init",20,registerLidarGpsCallback);

	ros::Publisher pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);
	ros::Publisher pubOptimizePoses = nh.advertise<sensor_msgs::PointCloud2>("/optimize_pose",2);
	ros::Publisher pubLoamPoses = nh.advertise<sensor_msgs::PointCloud2>("/loam_pose",2);
	pubLaserCloudSurround_rtk = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround_rtk", 2);
	pubLoamDeltaTheta = nh.advertise<nav_msgs::Odometry>("/delta_theta",2);

	ros::Rate rate(10);
	double odom_num = 0;
	bool first_write_keyPose(true);
	while(ros::ok()){
		if(cloudKeyPoses3D->points.size()!=0){
			sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudKeyPoses3D, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            cloudMsgTemp.header.frame_id = "/camera_init";
            pubKeyPoses.publish(cloudMsgTemp);

		}
		if(optimizePose3D->points.size()!=0){
			sensor_msgs::PointCloud2 cloudOptimizeTemp;
            pcl::toROSMsg(*optimizePose3D, cloudOptimizeTemp);
            cloudOptimizeTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            cloudOptimizeTemp.header.frame_id = "/camera_init";
            pubOptimizePoses.publish(cloudOptimizeTemp);
		}
		if(loamPose3D->points.size()!=0){
			sensor_msgs::PointCloud2 cloudloamTemp;
            pcl::toROSMsg(*loamPose3D, cloudloamTemp);
            cloudloamTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            cloudloamTemp.header.frame_id = "/camera_init";
            pubLoamPoses.publish(cloudloamTemp);
		}
		ros::spinOnce();
		rate.sleep();
	}
	return 0 ;
}