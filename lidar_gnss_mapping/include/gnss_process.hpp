#ifndef __data_process_hpp__
#define __data_process_hpp__

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
using namespace Eigen;

#ifndef __pi__
#define __pi__
const double PI = 3.1415926535898;
#endif

typedef geometry_msgs::Vector3 ROSVec3d;
typedef geometry_msgs::Point ROSPoi3d;
bool isFirstGPS(true);
double first_Xe;
double first_Ye;
double first_Ze;

//Geocentric latitude and longitude coordinates of the high-coordinate data is converted to Northeast day coordinate system coordinates
void BLH2ENU(double lat, double lon, double alt, double &x, double &y,
             double &z, double sin_lat, double cos_lat, double sin_lon,
             double cos_lon) {
    lat = lat * M_PI / 180; // To rad.
    lon = lon * M_PI / 180;
    double f = 1 / 298.257223563; // WGS84
    double A_GNSS = 6378137.0;         // WGS84
    double B_GNSS = A_GNSS * (1 - f);
    double e = sqrt(A_GNSS * A_GNSS - B_GNSS * B_GNSS) / A_GNSS;
    double N = A_GNSS / sqrt(1 - e * e * sin(lat) * sin(lat));
    // To ECEF  
    double Xe = (N + alt) * cos(lat) * cos(lon); 
    double Ye = (N + alt) * cos(lat) * sin(lon);
    double Ze = (N * (1 - (e * e)) + alt) * sin(lat);
    if(isFirstGPS){
        first_Xe = Xe;
        first_Ye = Ye;
        first_Ze = Ze;
        isFirstGPS = false;
    }
    Xe -= first_Xe;
    Ye -= first_Ye;
    Ze -= first_Ze;

    // To ENU
    x = -Xe * sin_lon + Ye * cos_lon; 
    y = -Xe * sin_lat * cos_lon - Ye * sin_lat * sin_lon + Ze * cos_lat;
    z = Xe * cos_lat * cos_lon + Ye * cos_lat * sin_lon + Ze * sin_lat;
}
void AverageFilter(ROSPoi3d &inPos, ROSPoi3d &outPos, int num) {
  static int countIn(0);
  static const int pointNum = num; // 50点
  static float *smX = new float[pointNum];
  static float *smY = new float[pointNum];
  static float *smZ = new float[pointNum];
  if (countIn < pointNum) {
    countIn++;
    smX[countIn - 1] = inPos.x;
    smY[countIn - 1] = inPos.y;
    smZ[countIn - 1] = inPos.z;
    outPos = inPos;
    return;
  }
  for (int i = 0; i < pointNum - 1; ++i) {
    smX[i] = smX[i + 1];
    smY[i] = smY[i + 1];
    smZ[i] = smZ[i + 1];
  }
  smX[pointNum - 1] = inPos.x;
  smY[pointNum - 1] = inPos.y;
  smZ[pointNum - 1] = inPos.z;
  float sumX(0), sumY(0), sumZ(0);
  for (int i = 0; i < pointNum; ++i) {
    sumX += smX[i];
    sumY += smY[i];
    sumZ += smZ[i];
  }
  outPos.x = sumX / pointNum;
  outPos.y = sumY / pointNum;
  outPos.z = sumZ / pointNum;
}
//Kalman filter
void KalmanFun(const ROSVec3d &acc, ROSPoi3d &inPos, double dt,
               ROSPoi3d &outPos, int status) {

  static bool isFirstKal(true);
  static MatrixXd A(4, 4);
  static MatrixXd B(4, 2);
  static MatrixXd u(2, 1);
  static MatrixXd Q(4, 4);
  static MatrixXd H(2, 4);
  static MatrixXd R(2, 2);
  static MatrixXd X_pdct(4, 1);
  static MatrixXd Pk_pdct(4, 4);
  static MatrixXd K(4, 2);
  static MatrixXd Z_meas(2, 1);
  static MatrixXd X_evlt(4, 1);
  static MatrixXd Pk_evlt(4, 4);
  if (isFirstKal) {

    X_pdct.setZero();
    Pk_pdct.setZero();
    K.setZero();
    Z_meas.setZero(); 
    X_evlt.setZero(); 
    Pk_evlt.setZero();
    isFirstKal = false;
  }
  A << 1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1;

  B << 0.5 * dt * dt, 0, 0, 0.5 * dt * dt, dt, 0, 0, dt;

  u << acc.x, acc.y;

  Q << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0,
      0.01; 

  H << 1, 0, 0, 0, 0, 1, 0,
      0; 
  if (status < 2) {
    R << 1, 0, 0, 1; 
  } else if (status == 2) {
    R << 0.5, 0, 0, 0.5; 
  } else {
    R << 0.01, 0, 0, 0.01; 
  }


  X_pdct = A * X_evlt + B * u;
  Pk_pdct = A * Pk_evlt * A.transpose() + Q;

  MatrixXd tmp(2, 2);
  tmp = H * Pk_pdct * H.transpose() + R;
  K = Pk_pdct * H.transpose() * tmp.inverse();

  Z_meas << inPos.x, inPos.y; 
  X_evlt = X_pdct + K * (Z_meas - H * X_pdct);
  Pk_evlt = (MatrixXd::Identity(4, 4) - K * H) * Pk_pdct;

  outPos.x = X_evlt(0, 0);
  outPos.y = X_evlt(1, 0);
  outPos.z = inPos.z;
}

float GetMeanValue(const int num, const ROSPoi3d *que, const int &len,
                   const int &cur_index) {
  float sum(0);
  for (int i = 0; i < num; ++i) {
    sum += que[(cur_index - i + len) % len].z;
  }
  return sum / num;
}

float GetVarianceValue(const int num, const ROSPoi3d *que, const int &len,
                       const int &cur_index) {
  float mean = GetMeanValue(num, que, len, cur_index);
  float sum(0), differ(0);
  for (int i = 0; i < num; ++i) {
    differ = que[(cur_index - i + len) % len].z - mean;
    sum += differ * differ;
  }
  return sum / num;
}

void GetMeanAndVarianceValue(const int num, const ROSPoi3d *que, const int &len,
                             const int &cur_index, float &mean,
                             float &variance) {
  mean = GetMeanValue(num, que, len, cur_index);
  float sum(0), differ(0);
  for (int i = 0; i < num; ++i) {
    differ = que[(cur_index - i + len) % len].z - mean;
    sum += differ * differ;
  }
  variance = sum / num;
}

bool IsOutliers(ROSPoi3d *que, const int len, const int cur_index) {
  static int point_count(0);
  static float mean(0), variance(0);
  if (point_count < 40) {
    ++point_count;
  } else {
    GetMeanAndVarianceValue(40, que, len, cur_index, mean, variance);
  }
  // printf("%f %f\n", mean, variance);
  if (variance > 0.2) {
    return true;
  } else {
    return false;
  }
}
#endif