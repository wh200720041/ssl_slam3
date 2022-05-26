// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _PARAMETER_FACTOR_H_
#define _PARAMETER_FACTOR_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "utils.h"

class PoseParameterization : public ceres::LocalParameterization {
public:
    PoseParameterization() {}
    virtual ~PoseParameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const { return 15; }
    virtual int LocalSize() const { return 15; }
};

class ConstantPoseParameterization : public ceres::LocalParameterization {
public:
    
    ConstantPoseParameterization() {}
    virtual ~ConstantPoseParameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x, double* jacobian) const;
    virtual int GlobalSize() const { return 15; }
    virtual int LocalSize() const { return 15; }
};


#endif // _PARAMETER_FACTOR_H_

