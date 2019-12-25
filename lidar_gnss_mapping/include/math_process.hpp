#ifndef __data_process_h__
#define __data_process_h__

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
typedef geometry_msgs::Point ROSPoi3d;
typedef geometry_msgs::Vector3 ROSVec3d;
void QuatMultiply(double w1, double x1, double y1, double z1, double w2, double x2,
                  double y2, double z2, double &w3, double &x3, double &y3,
                  double &z3) {
  w3 = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
  x3 = w1 * x2 + x1 * w2 + z1 * y2 - y1 * z2;
  y3 = w1 * y2 + y1 * w2 + x1 * z2 - z1 * x2;
  z3 = w1 * z2 + z1 * w2 + y1 * x2 - x1 * y2;
}
ROSVec3d VecMinus(ROSVec3d src1, ROSVec3d src2) {
  static ROSVec3d rst;
  rst.x = src1.x - src2.x;
  rst.y = src1.y - src2.y;
  rst.z = src1.z - src2.z;
  return rst;
}
ROSVec3d VecPlus(ROSVec3d src1, ROSVec3d src2) {
  static ROSVec3d rst;
  rst.x = src1.x + src2.x;
  rst.y = src1.y + src2.y;
  rst.z = src1.z + src2.z;
  return rst;
}
ROSPoi3d PoiMinus(ROSPoi3d src1, ROSPoi3d src2) {
  static ROSPoi3d rst;
  rst.x = src1.x - src2.x;
  rst.y = src1.y - src2.y;
  rst.z = src1.z - src2.z;
  return rst;
}
ROSPoi3d PoiPlus(ROSPoi3d src1, ROSPoi3d src2) {
  static ROSPoi3d rst;
  rst.x = src1.x + src2.x;
  rst.y = src1.y + src2.y;
  rst.z = src1.z + src2.z;
  return rst;
}

#endif