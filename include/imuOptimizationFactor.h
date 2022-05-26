// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _IMU_OPTIMIZATION_FACTOR_H_
#define _IMU_OPTIMIZATION_FACTOR_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utils.h"
#include "imuPreintegrationClass.h"

class ImuPreintegrationFactor : public ceres::SizedCostFunction<15, 15, 15>{
  public:
    ImuPreintegrationFactor(ImuPreintegrationClass& imu_preintegrator_in):imu_preintegrator(imu_preintegrator_in){}
    
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    ImuPreintegrationClass& imu_preintegrator;
};

#endif // _IMU_OPTIMIZATION_FACTOR_H_

