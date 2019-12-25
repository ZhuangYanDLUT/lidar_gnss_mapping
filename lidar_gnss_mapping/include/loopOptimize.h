#ifndef _LOOP_OPTIMIZE_H_
#define _LOOP_OPTIMIZE_H_
#include "loam_gtsam.h"
#include "pointType.h"

using namespace gtsam;
class loamGtsam{
private:
	
	NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
	ISAM2Params parameters;
    Values isamCurrentEstimate;

    noiseModel::Diagonal::shared_ptr priorNoise;
    noiseModel::Diagonal::shared_ptr odometryNoise;
    noiseModel::Diagonal::shared_ptr constraintNoise;


	pcl::PointCloud<PointTypePose>::Ptr odomKeyPoses6D;//保存要优化的位姿
	pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
	PointTypePose laser_gps;
	PointType path_data;
	bool newdata;
	bool loopClosureEnableFlag;
	
public:
	PointTypePose odometry;
	pcl::PointCloud<CurPointTypePose>::Ptr currentOdomKeyPoses6D;//校正后的位姿
	pcl::PointCloud<CurPointType>::Ptr currentCloudKeyPoses3D;
	double odom_num; //保存odom数目
	loamGtsam(){
		Vector Vector6(6);
		Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
  		priorNoise = noiseModel::Diagonal::Variances(Vector6);//生成噪声
		odometryNoise = noiseModel::Diagonal::Variances(Vector6);//噪声定义，对角线矩阵
		//参数设置
		
		parameters.relinearizeThreshold = 0.01;
		parameters.relinearizeSkip = 1;
		//生成ISAM2类型
		isam = new ISAM2(parameters);
		newdata = false;//是否为新的数据成员变量
		loopClosureEnableFlag = false;
		odomKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());		//保存变量的数组
		cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());

		currentOdomKeyPoses6D.reset(new pcl::PointCloud<CurPointTypePose>());
		currentCloudKeyPoses3D.reset(new pcl::PointCloud<CurPointType>());
		odom_num = 0;		//初始设定需要优化的里程计的数目为零
		
	}
	//void recv_data(const PointTypePose &odometry){
	//读取该类中odometry上的数据，并将其保存在要进行优化的数组中
	void recv_data(){
		//std::cout<<"recv_data"<<std::endl;
		newdata = true;
		laser_gps.x = odometry.x;
		laser_gps.y = odometry.y;
		laser_gps.z = odometry.z;
		laser_gps.time = odometry.time;
		laser_gps.roll = odometry.roll;
		laser_gps.pitch = odometry.pitch;
		laser_gps.yaw = odometry.yaw;
		laser_gps.intensity = odometry.intensity;
		laser_gps.index = odomKeyPoses6D->points.size();	//index

		odomKeyPoses6D->push_back(laser_gps);

		path_data.x = laser_gps.x;
		path_data.y = laser_gps.y;
		path_data.z = laser_gps.z;
		//std::cout<<laser_gps.intensity<<"**************************"<<std::endl;
				
	}
	/*
	数据标志位介绍
	laser_gps.intensity = 0 里程计信息由GPS发出
	laser_gps.intensity = 1 里程计信息从LOAM切换至GPS时GPS提供的里程计信息
	laser_gps.intensity = 2 里程计信息由LOAM发出
	laser_gps.intensity = 3 里程计信息从LOAM切换至GPS时LOAM提供的里程计信息
	*/
	void path_optimize(){
		//std::cout<<"path_optimize"<<std::endl;
		//当为新的数据，并且数据类别为2或者3时
		if(newdata && (laser_gps.intensity == 2 || laser_gps.intensity == 3)){
			newdata = false;
			if(odom_num == 0){
				//std::cout<<"odom_num = 0 &&(laser_gps.intensity == 2 || laser_gps.intensity == 3) "<<std::endl;
				Rot3 R = Rot3::RzRyRx(laser_gps.yaw,laser_gps.roll,laser_gps.pitch);
				Point3 t(laser_gps.z,laser_gps.x,laser_gps.y);
				NonlinearFactor::shared_ptr factor1(new PriorFactor<Pose3>(0,gtsam::Pose3(R,t),priorNoise));
				initialEstimate.insert(0, Pose3(Rot3::RzRyRx(laser_gps.yaw,laser_gps.roll,laser_gps.pitch),
	                Point3(laser_gps.z,laser_gps.x,laser_gps.y)));
				gtSAMgraph.push_back(factor1);
			}else{
				//std::cout<<"no first"<<odomKeyPoses6D->points.size()<<" * "<<laser_gps.index<<std::endl;
				//std::cout<<"no first"<<odomKeyPoses6D->points.size()<<" * "<<laser_gps.intensity<<std::endl;
				//std::cout<<"(laser_gps.intensity == 2 || laser_gps.intensity == 3) "<<std::endl;
				int pose6Dsize = laser_gps.index;
				gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(odomKeyPoses6D->points[pose6Dsize-1].yaw,odomKeyPoses6D->points[pose6Dsize-1].roll,
					odomKeyPoses6D->points[pose6Dsize-1].pitch),
					Point3(odomKeyPoses6D->points[pose6Dsize-1].z,odomKeyPoses6D->points[pose6Dsize-1].x,odomKeyPoses6D->points[pose6Dsize-1].y));

				gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(odomKeyPoses6D->points[pose6Dsize].yaw,odomKeyPoses6D->points[pose6Dsize].roll,
					odomKeyPoses6D->points[pose6Dsize].pitch),
					Point3(odomKeyPoses6D->points[pose6Dsize].z,odomKeyPoses6D->points[pose6Dsize].x,odomKeyPoses6D->points[pose6Dsize].y));

				gtSAMgraph.add(BetweenFactor<Pose3>(odom_num-1,odom_num,poseFrom.between(poseTo), odometryNoise));
				initialEstimate.insert(odom_num, Pose3(Rot3::RzRyRx(odomKeyPoses6D->points[pose6Dsize].yaw,odomKeyPoses6D->points[pose6Dsize].roll,
					odomKeyPoses6D->points[pose6Dsize].pitch),  
					Point3(odomKeyPoses6D->points[pose6Dsize].z,odomKeyPoses6D->points[pose6Dsize].x,odomKeyPoses6D->points[pose6Dsize].y)));
				
			}
			odom_num++;
		//	std::cout<<"befor updata"<<std::endl;
			isam->update(gtSAMgraph, initialEstimate);
			isam->update();
		//	std::cout<<"end updata"<<std::endl;
			gtSAMgraph.resize(0);
			initialEstimate.clear();
		}
		if(newdata && laser_gps.intensity == 1 && odom_num>1 && odomKeyPoses6D->points[laser_gps.index-1].intensity == 3){//闭环优化
			//std::cout<<" laser_gps.intensity == 1 "<<std::endl;
		//if(newdata && laser_gps.intensity == 1 && odom_num>1){//闭环优化
	//		std::cout<<"loop closure   "<<laser_gps.intensity<<std::endl;
	//		std::cout<<odomKeyPoses6D->points[laser_gps.index].intensity<<std::endl;
	//		std::cout<<odomKeyPoses6D->points[laser_gps.index-1].intensity<<std::endl;
	//		std::cout<<odomKeyPoses6D->points[laser_gps.index-2].intensity<<std::endl;
			newdata = false;
			Rot3 R = Rot3::RzRyRx(laser_gps.yaw,laser_gps.roll,laser_gps.pitch);
			Point3 t(laser_gps.z,laser_gps.x,laser_gps.y);
			NonlinearFactor::shared_ptr factor2(new PriorFactor<Pose3>(odom_num-1,gtsam::Pose3(R,t),priorNoise));

			gtSAMgraph.push_back(factor2);
			isam->update(gtSAMgraph);
			isam->update();
			gtSAMgraph.resize(0);
			odom_num++;
			loopClosureEnableFlag = true;
			std::cout<<"loop closure !!!  "<<odom_num -1<<std::endl;
		}
		
	}
	bool save_result(){//将校正过后的数据保存处理
		if(loopClosureEnableFlag){
			loopClosureEnableFlag = false;
			currentOdomKeyPoses6D.reset(new pcl::PointCloud<CurPointTypePose>());
			currentCloudKeyPoses3D.reset(new pcl::PointCloud<CurPointType>());
			odomKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());		//保存变量的数组
			cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());

			isamCurrentEstimate = isam->calculateEstimate();//得到正确的位姿估计
			for(int i=0; i<isamCurrentEstimate.size(); ++i){
				Pose3 latestEstimate;	
				CurPointTypePose thisPose6D; CurPointType thisPose3D;
				latestEstimate = isamCurrentEstimate.at<Pose3>(i);
				thisPose6D.x = latestEstimate.translation().y();
    		    thisPose6D.y = latestEstimate.translation().z();
     		    thisPose6D.z = latestEstimate.translation().x();
     		    thisPose6D.roll  = latestEstimate.rotation().pitch();
        		thisPose6D.pitch = latestEstimate.rotation().yaw();
        		thisPose6D.yaw   = latestEstimate.rotation().roll();
				thisPose3D.x = thisPose6D.x;
				thisPose3D.y = thisPose6D.y;
				thisPose3D.z = thisPose6D.z;
				currentOdomKeyPoses6D->push_back(thisPose6D);
				currentCloudKeyPoses3D->push_back(thisPose3D);
			}
			delete isam;
			isam = new ISAM2(parameters);
			odom_num = 0;
			
			return true;
		}
		return false;
	}
	void run(){
		//recv_data(odometry);
		recv_data();
		path_optimize();
		save_result();
	}

};
#endif

