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

#if CERES_VERSION_MAJOR >= 3 || (CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1)
class PoseParameterization : public ceres::Manifold {
public:

    PoseParameterization() {}
    virtual ~PoseParameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool PlusJacobian(const double* x, double* jacobian) const;
    virtual bool Minus(const double* x, const double* delta, double* x_minus_delta) const;
    virtual bool MinusJacobian(const double* x, double* jacobian) const;
    virtual int AmbientSize() const { return 15; }
    virtual int TangentSize() const { return 15; }
};
class ConstantPoseParameterization : public ceres::Manifold {
public:
    
    ConstantPoseParameterization() {}
    virtual ~ConstantPoseParameterization() {}
    virtual bool Plus(const double* x, const double* delta, double* x_plus_delta) const;
    virtual bool PlusJacobian(const double* x, double* jacobian) const;
    virtual bool Minus(const double* x, const double* delta, double* x_minus_delta) const;
    virtual bool MinusJacobian(const double* x, double* jacobian) const;
    virtual int AmbientSize() const { return 15; }
    virtual int TangentSize() const { return 15; }
};
#else
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

#endif

#endif // _PARAMETER_FACTOR_H_

