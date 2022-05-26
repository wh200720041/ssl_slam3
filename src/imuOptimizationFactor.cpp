// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "imuOptimizationFactor.h"

bool ImuPreintegrationFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    Eigen::Vector3d ri(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Matrix3d Ri = Utils::so3ToR(ri);
    Eigen::Vector3d Pi(parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector3d Vi(parameters[0][6], parameters[0][7], parameters[0][8]);
    Eigen::Vector3d Bai(parameters[0][9], parameters[0][10], parameters[0][11]);
    Eigen::Vector3d Bgi(parameters[0][12], parameters[0][13], parameters[0][14]);

    Eigen::Vector3d rj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Matrix3d Rj = Utils::so3ToR(rj);
    Eigen::Vector3d Pj(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d Vj(parameters[1][6], parameters[1][7], parameters[1][8]);
    Eigen::Vector3d Baj(parameters[1][9], parameters[1][10], parameters[1][11]);
    Eigen::Vector3d Bgj(parameters[1][12], parameters[1][13], parameters[1][14]);

    Eigen::Matrix3d dr_dbg = imu_preintegrator.jacobian.template block<3, 3>(0, 12);
    Eigen::Matrix3d dp_dba = imu_preintegrator.jacobian.template block<3, 3>(3, 9);
    Eigen::Matrix3d dp_dbg = imu_preintegrator.jacobian.template block<3, 3>(3, 12);
    Eigen::Matrix3d dv_dba = imu_preintegrator.jacobian.template block<3, 3>(6, 9);
    Eigen::Matrix3d dv_dbg = imu_preintegrator.jacobian.template block<3, 3>(6, 12);
    double sum_dt = imu_preintegrator.sum_dt;
    double sum_dt2 = sum_dt * sum_dt;
    Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(imu_preintegrator.covariance.inverse()).matrixL().transpose();

    if (imu_preintegrator.jacobian.maxCoeff() > 1e8 || imu_preintegrator.jacobian.minCoeff() < -1e8){
        ROS_WARN("numerical unstable in preintegration");
        std::cout << imu_preintegrator.jacobian << std::endl;
    }

    //calculate cost
    Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
    Eigen::Vector3d dba = Bai - imu_preintegrator.b_a;
    Eigen::Vector3d dbg = Bgi - imu_preintegrator.b_g;

    Eigen::Matrix3d corrected_delta_R = imu_preintegrator.delta_R * Utils::so3ToR(dr_dbg * dbg);
    Eigen::Vector3d corrected_delta_p = imu_preintegrator.delta_p + dp_dba * dba + dp_dbg * dbg;
    Eigen::Vector3d corrected_delta_v = imu_preintegrator.delta_v + dv_dba * dba + dv_dbg * dbg;

    // residual 1 = log[delta_r' * Ri' * Rj]
    // residual 2 = Ri'* (0.5 * g *dt2 + Pj - Pi -Vi * dt) - delta_p
    // residual 3 = Ri'* (g * dt + Vj - Vi) - delta_v
    // residual 4 = b_aj - b_ai
    // residual 5 = b_gj - b_gi
    Eigen::Vector3d rot_residual = Utils::RToso3(corrected_delta_R.transpose() * Ri.transpose() * Rj);
    residual.block<3, 1>(0, 0) = rot_residual;
    residual.block<3, 1>(3, 0) = Ri.transpose() * (0.5 * Utils::gravity * sum_dt2 + Pj - Pi - Vi * sum_dt) - corrected_delta_p;
    residual.block<3, 1>(6, 0) = Ri.transpose() * (Utils::gravity * sum_dt + Vj - Vi) - corrected_delta_v;
    residual.block<3, 1>(9, 0) = Baj - Bai;
    residual.block<3, 1>(12, 0) = Bgj - Bgi;
    residual = sqrt_info * residual;

    //check if it is necessary to repropagate
#if 1
    if ((Bai - imu_preintegrator.b_a).norm() > 0.10 ||
        (Bgi - imu_preintegrator.b_g).norm() > 0.01)
    {
        std::cout<<"change bias "<< Bai.norm()<<"->"<<imu_preintegrator.b_a.norm() << " and "<< Bgi.norm() <<"->"<< imu_preintegrator.b_g.norm() <<std::endl;
        imu_preintegrator.update(Bai, Bgi);
    }
#endif

    if (jacobians != NULL){  
        if (jacobians[0] != NULL){
            Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> jacobian_i(jacobians[0]);
            jacobian_i.setZero();

            jacobian_i.block<3, 3>(0, 0) = - Utils::Jr_so3_inv(rot_residual) * Rj.transpose() * Ri;
            jacobian_i.block<3, 3>(0, 12) = - Utils::Jr_so3_inv(rot_residual) * Rj.transpose() * Ri * corrected_delta_R * dr_dbg;
            jacobian_i.block<3, 3>(3, 0) = Utils::skew(Ri.transpose() * (0.5 * Utils::gravity * sum_dt2 + Pj - Pi - Vi * sum_dt));
            jacobian_i.block<3, 3>(3, 3) = - Ri.transpose();
            jacobian_i.block<3, 3>(3, 6) = - Ri.transpose() * sum_dt;
            jacobian_i.block<3, 3>(3, 9) = -dp_dba;
            jacobian_i.block<3, 3>(3, 12) = -dp_dbg;
            jacobian_i.block<3, 3>(6, 0) = Utils::skew(Ri.transpose() * (Utils::gravity * sum_dt + Vj - Vi));
            jacobian_i.block<3, 3>(6, 6) = - Ri.transpose();
            jacobian_i.block<3, 3>(6, 9) = -dv_dba;
            jacobian_i.block<3, 3>(6, 12) = -dv_dbg;
            jacobian_i.block<3, 3>(9, 9) = - Eigen::Matrix3d::Identity();
            jacobian_i.block<3, 3>(12, 12) = - Eigen::Matrix3d::Identity();
            jacobian_i = sqrt_info * jacobian_i;

            if (jacobian_i.maxCoeff() > 1e8 || jacobian_i.minCoeff() < -1e8){
                ROS_WARN("numerical unstable in preintegration");
            }
        }
        if (jacobians[1] != NULL){
            Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> jacobian_j(jacobians[1]);
            jacobian_j.setZero();
            jacobian_j.block<3, 3>(0, 0) = Utils::Jr_so3_inv(rot_residual);
            jacobian_j.block<3, 3>(3, 3) = Ri.transpose();
            jacobian_j.block<3, 3>(6, 6) = Ri.transpose();
            jacobian_j.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
            jacobian_j.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();
            jacobian_j = sqrt_info * jacobian_j;

            if (jacobian_j.maxCoeff() > 1e8 || jacobian_j.minCoeff() < -1e8){
                ROS_WARN("numerical unstable in preintegration");
            }
        }
    }

    return true;
}




