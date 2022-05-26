// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _IMU_PREINTEGRATION_CLASS_H_
#define _IMU_PREINTEGRATION_CLASS_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include "utils.h"

class ImuPreintegrationClass{
public:
  double sum_dt;
  Eigen::Matrix3d delta_R;
  Eigen::Vector3d delta_p;
  Eigen::Vector3d delta_v;  
  Eigen::Vector3d b_a;
  Eigen::Vector3d b_g;

  Eigen::Matrix<double, 15, 15> covariance;
  Eigen::Matrix<double, 15, 15> jacobian;

  ImuPreintegrationClass(const Eigen::Vector3d b_a_in, const Eigen::Vector3d b_g_in, 
    const double ACC_N_in, const double GYR_N_in, const double ACC_W_in, const double GYR_W_in);
  void addImuData(const double dt, const Eigen::Vector3d acc, const Eigen::Vector3d gyr);
  void update(const Eigen::Vector3d& b_a_in, const Eigen::Vector3d& b_g_in);

  //call only during initialization
  std::vector<Eigen::Vector3d> getAcc(void);
  std::vector<Eigen::Vector3d> getGyr(void);
private:

  std::vector<Eigen::Vector3d> acc_buf;
  std::vector<Eigen::Vector3d> gyr_buf;  
  std::vector<double> dt_buf;
  Eigen::Vector3d last_acc;
  Eigen::Vector3d last_gyr;
  Eigen::Matrix<double, 18, 18> noise;

  void initWithImu(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr);
  void propagate(const double dt, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr);
};

#endif // _IMU_PREINTEGRATION_CLASS_H_

