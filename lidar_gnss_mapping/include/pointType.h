#ifndef _POINT_TYPE_H_
#define _POINT_TYPE_H_


#include <opencv/cv.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

using namespace std;

typedef pcl::PointXYZI PointType;
typedef pcl::PointXYZ CurPointType;
//////////////////////////////////////////////
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    double index;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time) (double, index, index)
)

typedef PointXYZIRPYT  PointTypePose;
//////////////////////////////////////////////////
struct PointXYZRPY
{
    PCL_ADD_POINT4D

    float roll;
    float pitch;
    float yaw;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRPY,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) 
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   
)

typedef PointXYZRPY  CurPointTypePose;
typedef PointXYZIRPYT  PointTypePose;

#endif
