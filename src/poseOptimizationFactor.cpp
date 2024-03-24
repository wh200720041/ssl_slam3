// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "poseOptimizationFactor.h"
#if CERES_VERSION_MAJOR >= 3 || (CERES_VERSION_MAJOR >= 2 && CERES_VERSION_MINOR >= 1)
bool PoseParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> rot(x);
    Eigen::Map<const Eigen::Vector3d> pos(x + 3);
    Eigen::Map<const Eigen::Vector3d> vel(x + 6);
    Eigen::Map<const Eigen::Vector3d> b_a(x + 9);
    Eigen::Map<const Eigen::Vector3d> b_g(x + 12);
    
    Eigen::Map<const Eigen::Vector3d> delta_rot(delta);
    Eigen::Map<const Eigen::Vector3d> delta_pos(delta + 3);
    Eigen::Map<const Eigen::Vector3d> delta_vel(delta + 6);
    Eigen::Map<const Eigen::Vector3d> delta_b_a(delta + 9);
    Eigen::Map<const Eigen::Vector3d> delta_b_g(delta + 12);

    Eigen::Map<Eigen::Vector3d> rot_new(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> pos_new(x_plus_delta + 3);
    Eigen::Map<Eigen::Vector3d> vel_new(x_plus_delta + 6);
    Eigen::Map<Eigen::Vector3d> b_a_new(x_plus_delta + 9);
    Eigen::Map<Eigen::Vector3d> b_g_new(x_plus_delta + 12);

    rot_new = rot + Utils::Jr_so3_inv(rot) * delta_rot;
    pos_new = pos + delta_pos;
    vel_new = vel + delta_vel;
    b_a_new = b_a + delta_b_a;
    b_g_new = b_g + delta_b_g;
    return true;
}

bool PoseParameterization::Minus(const double *x, const double *delta, double *x_minus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> rot(x);
    Eigen::Map<const Eigen::Vector3d> pos(x + 3);
    Eigen::Map<const Eigen::Vector3d> vel(x + 6);
    Eigen::Map<const Eigen::Vector3d> b_a(x + 9);
    Eigen::Map<const Eigen::Vector3d> b_g(x + 12);
    
    Eigen::Map<const Eigen::Vector3d> delta_rot(delta);
    Eigen::Map<const Eigen::Vector3d> delta_pos(delta + 3);
    Eigen::Map<const Eigen::Vector3d> delta_vel(delta + 6);
    Eigen::Map<const Eigen::Vector3d> delta_b_a(delta + 9);
    Eigen::Map<const Eigen::Vector3d> delta_b_g(delta + 12);

    Eigen::Map<Eigen::Vector3d> rot_new(x_minus_delta);
    Eigen::Map<Eigen::Vector3d> pos_new(x_minus_delta + 3);
    Eigen::Map<Eigen::Vector3d> vel_new(x_minus_delta + 6);
    Eigen::Map<Eigen::Vector3d> b_a_new(x_minus_delta + 9);
    Eigen::Map<Eigen::Vector3d> b_g_new(x_minus_delta + 12);

    rot_new = rot - Utils::Jr_so3_inv(rot) * delta_rot;
    pos_new = pos - delta_pos;
    vel_new = vel - delta_vel;
    b_a_new = b_a - delta_b_a;
    b_g_new = b_g - delta_b_g;
    return true;
}

bool PoseParameterization::PlusJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    return true;
}

bool PoseParameterization::MinusJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    return true;
}

bool ConstantPoseParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> rot(x);
    Eigen::Map<const Eigen::Vector3d> pos(x + 3);
    Eigen::Map<const Eigen::Vector3d> vel(x + 6);
    Eigen::Map<const Eigen::Vector3d> b_a(x + 9);
    Eigen::Map<const Eigen::Vector3d> b_g(x + 12);
    
    Eigen::Map<const Eigen::Vector3d> delta_rot(delta);
    Eigen::Map<const Eigen::Vector3d> delta_pos(delta + 3);
    Eigen::Map<const Eigen::Vector3d> delta_vel(delta + 6);
    Eigen::Map<const Eigen::Vector3d> delta_b_a(delta + 9);
    Eigen::Map<const Eigen::Vector3d> delta_b_g(delta + 12);

    Eigen::Map<Eigen::Vector3d> rot_new(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> pos_new(x_plus_delta + 3);
    Eigen::Map<Eigen::Vector3d> vel_new(x_plus_delta + 6);
    Eigen::Map<Eigen::Vector3d> b_a_new(x_plus_delta + 9);
    Eigen::Map<Eigen::Vector3d> b_g_new(x_plus_delta + 12);

    rot_new = rot;
    pos_new = pos;
    // vel_new = vel; 
    // b_a_new = b_a;
    // b_g_new = b_g;
    vel_new = vel + delta_vel;
    b_a_new = b_a + delta_b_a;
    b_g_new = b_g + delta_b_g;
    return true;
}

bool ConstantPoseParameterization::Minus(const double *x, const double *delta, double *x_minus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> rot(x);
    Eigen::Map<const Eigen::Vector3d> pos(x + 3);
    Eigen::Map<const Eigen::Vector3d> vel(x + 6);
    Eigen::Map<const Eigen::Vector3d> b_a(x + 9);
    Eigen::Map<const Eigen::Vector3d> b_g(x + 12);
    
    Eigen::Map<const Eigen::Vector3d> delta_rot(delta);
    Eigen::Map<const Eigen::Vector3d> delta_pos(delta + 3);
    Eigen::Map<const Eigen::Vector3d> delta_vel(delta + 6);
    Eigen::Map<const Eigen::Vector3d> delta_b_a(delta + 9);
    Eigen::Map<const Eigen::Vector3d> delta_b_g(delta + 12);

    Eigen::Map<Eigen::Vector3d> rot_new(x_minus_delta);
    Eigen::Map<Eigen::Vector3d> pos_new(x_minus_delta + 3);
    Eigen::Map<Eigen::Vector3d> vel_new(x_minus_delta + 6);
    Eigen::Map<Eigen::Vector3d> b_a_new(x_minus_delta + 9);
    Eigen::Map<Eigen::Vector3d> b_g_new(x_minus_delta + 12);

    rot_new = rot;
    pos_new = pos;
    // vel_new = vel; 
    // b_a_new = b_a;
    // b_g_new = b_g;
    vel_new = vel - delta_vel;
    b_a_new = b_a - delta_b_a;
    b_g_new = b_g - delta_b_g;
    return true;
}

bool ConstantPoseParameterization::PlusJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    return true;
}

bool ConstantPoseParameterization::MinusJacobian(const double *x, double *jacobian) const
{
    Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    return true;
}

#else

bool PoseParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const {
    Eigen::Map<const Eigen::Vector3d> rot(x);
    Eigen::Map<const Eigen::Vector3d> pos(x + 3);
    Eigen::Map<const Eigen::Vector3d> vel(x + 6);
    Eigen::Map<const Eigen::Vector3d> b_a(x + 9);
    Eigen::Map<const Eigen::Vector3d> b_g(x + 12);
    
    Eigen::Map<const Eigen::Vector3d> delta_rot(delta);
    Eigen::Map<const Eigen::Vector3d> delta_pos(delta + 3);
    Eigen::Map<const Eigen::Vector3d> delta_vel(delta + 6);
    Eigen::Map<const Eigen::Vector3d> delta_b_a(delta + 9);
    Eigen::Map<const Eigen::Vector3d> delta_b_g(delta + 12);

    Eigen::Map<Eigen::Vector3d> rot_new(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> pos_new(x_plus_delta + 3);
    Eigen::Map<Eigen::Vector3d> vel_new(x_plus_delta + 6);
    Eigen::Map<Eigen::Vector3d> b_a_new(x_plus_delta + 9);
    Eigen::Map<Eigen::Vector3d> b_g_new(x_plus_delta + 12);

    rot_new = rot + Utils::Jr_so3_inv(rot) * delta_rot;
    pos_new = pos + delta_pos;
    vel_new = vel + delta_vel;
    b_a_new = b_a + delta_b_a;
    b_g_new = b_g + delta_b_g;
    return true;
}

bool PoseParameterization::ComputeJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    return true;
}

//init pose, only bias and speed is changable
bool ConstantPoseParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const {
    Eigen::Map<const Eigen::Vector3d> rot(x);
    Eigen::Map<const Eigen::Vector3d> pos(x + 3);
    Eigen::Map<const Eigen::Vector3d> vel(x + 6);
    Eigen::Map<const Eigen::Vector3d> b_a(x + 9);
    Eigen::Map<const Eigen::Vector3d> b_g(x + 12);
    
    Eigen::Map<const Eigen::Vector3d> delta_rot(delta);
    Eigen::Map<const Eigen::Vector3d> delta_pos(delta + 3);
    Eigen::Map<const Eigen::Vector3d> delta_vel(delta + 6);
    Eigen::Map<const Eigen::Vector3d> delta_b_a(delta + 9);
    Eigen::Map<const Eigen::Vector3d> delta_b_g(delta + 12);

    Eigen::Map<Eigen::Vector3d> rot_new(x_plus_delta);
    Eigen::Map<Eigen::Vector3d> pos_new(x_plus_delta + 3);
    Eigen::Map<Eigen::Vector3d> vel_new(x_plus_delta + 6);
    Eigen::Map<Eigen::Vector3d> b_a_new(x_plus_delta + 9);
    Eigen::Map<Eigen::Vector3d> b_g_new(x_plus_delta + 12);

    rot_new = rot;
    pos_new = pos;
    // vel_new = vel; 
    // b_a_new = b_a;
    // b_g_new = b_g;
    vel_new = vel + delta_vel;
    b_a_new = b_a + delta_b_a;
    b_g_new = b_g + delta_b_g;
    return true;
}

bool ConstantPoseParameterization::ComputeJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    return true;
}

#endif
