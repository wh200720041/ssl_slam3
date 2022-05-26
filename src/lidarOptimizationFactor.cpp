// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "lidarOptimizationFactor.h"

LidarOdometryFactor::LidarOdometryFactor(Eigen::Isometry3d odom_in, Eigen::Matrix<double, 6, 1> covariance_in){
    odom = odom_in;
    sqrt_info = covariance_in.asDiagonal().inverse();
}

bool LidarOdometryFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d ri(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Matrix3d Ri = Utils::so3ToR(ri);
    Eigen::Vector3d Pi(parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d rj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Matrix3d Rj = Utils::so3ToR(rj);
    Eigen::Vector3d Pj(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Matrix3d Rij = odom.linear();
    Eigen::Vector3d Pij = odom.translation();

    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);

    Eigen::Vector3d so3 = Utils::RToso3(Rij.transpose() * Ri.transpose() * Rj);
    residual.block<3, 1>(0, 0) = so3;
    residual.block<3, 1>(3, 0) = Rij.transpose() * ( Ri.transpose() * (Pj - Pi)- Pij);
    residual = sqrt_info * residual;
    if (jacobians != NULL){  
        if (jacobians[0] != NULL){
            Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> jacobian_i(jacobians[0]);
            jacobian_i.setZero();
            jacobian_i.block<3, 3>(0, 0) = - Utils::Jr_so3_inv(so3) * Rj.transpose() * Ri;
            jacobian_i.block<3, 3>(3, 0) = Rij.transpose() * Utils::skew( Ri.transpose() * (Pj - Pi));
            jacobian_i.block<3, 3>(3, 3) = - Rij.transpose() * Ri.transpose();
            jacobian_i = sqrt_info * jacobian_i;

            if (jacobian_i.maxCoeff() > 1e8 || jacobian_i.minCoeff() < -1e8)
                ROS_WARN("numerical unstable in odom factor");
        }
        if (jacobians[1] != NULL){
            Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> jacobian_j(jacobians[1]);
            jacobian_j.setZero();
            jacobian_j.block<3, 3>(0, 0) = Utils::Jr_so3_inv(so3);
            jacobian_j.block<3, 3>(3, 3) = Rij.transpose() * Ri.transpose();
            jacobian_j = sqrt_info * jacobian_j;

            if (jacobian_j.maxCoeff() > 1e8 || jacobian_j.minCoeff() < -1e8)
                ROS_WARN("numerical unstable in odom factor");
        }
    }
    return true;
}

LidarEdgeFactor::LidarEdgeFactor(Eigen::Vector3d curr_point_in, Eigen::Vector3d last_point_a_in, Eigen::Vector3d last_point_b_in, double covariance_in){
    curr_point = curr_point_in; 
    last_point_a = last_point_a_in;
    last_point_b = last_point_b_in;
    sqrt_info = 1.0 / covariance_in;
}

bool LidarEdgeFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d ri(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Vector3d Pi(parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Matrix3d Ri = Utils::so3ToR(ri);

    Eigen::Vector3d lp = Ri * curr_point + Pi; 
    Eigen::Vector3d nu = (lp - last_point_a).cross(lp - last_point_b);
    Eigen::Vector3d de = last_point_a - last_point_b;
    residuals[0] = sqrt_info * nu.norm() / de.norm();
    
    if(jacobians != NULL){
        if(jacobians[0] != NULL){
            Eigen::Matrix<double, 3, 15> dp_by_so3xyz = Eigen::Matrix<double, 3, 15>::Zero();
            dp_by_so3xyz.block<3, 3>(0, 0) = - Ri * Utils::skew(curr_point);
            dp_by_so3xyz.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
            Eigen::Map<Eigen::Matrix<double, 1, 15, Eigen::RowMajor> > jacobian_i(jacobians[0]);
            jacobian_i.setZero();
            jacobian_i = - sqrt_info * nu.transpose() / nu.norm() * Utils::skew(de) * dp_by_so3xyz / de.norm();
        }
    }  

    return true;
}   

LidarSurfFactor::LidarSurfFactor(Eigen::Vector3d curr_point_in, Eigen::Vector3d plane_unit_norm_in, double negative_OA_dot_norm_in, double covariance_in){
    curr_point = curr_point_in;
    plane_unit_norm = plane_unit_norm_in;
    negative_OA_dot_norm = negative_OA_dot_norm_in;
    sqrt_info = 1.0 / covariance_in;
}

bool LidarSurfFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d ri(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Vector3d Pi(parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Matrix3d Ri = Utils::so3ToR(ri);
    Eigen::Vector3d lp = Ri * curr_point + Pi;
    residuals[0] = sqrt_info * (plane_unit_norm.dot(lp) + negative_OA_dot_norm);

    if(jacobians != NULL){
        if(jacobians[0] != NULL){
            Eigen::Matrix<double, 3, 15> dp_by_so3xyz = Eigen::Matrix<double, 3, 15>::Zero();
            dp_by_so3xyz.block<3,3>(0, 0) = - Ri * Utils::skew(curr_point);
            dp_by_so3xyz.block<3,3>(0, 3) = Eigen::Matrix3d::Identity();
            Eigen::Map<Eigen::Matrix<double, 1, 15, Eigen::RowMajor> > jacobian_i(jacobians[0]);
            jacobian_i.setZero();
            jacobian_i = sqrt_info * plane_unit_norm.transpose() * dp_by_so3xyz;
        }
    }
    return true;
}   
