// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationClass.h"

OdomEstimationClass::OdomEstimationClass(){
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose_r_arr.push_back(Utils::RToso3(pose.linear()));
    pose_t_arr.push_back(pose.translation());
    pose_v_arr.push_back(Eigen::Vector3d::Zero());
    pose_b_a_arr.push_back(Eigen::Vector3d::Zero());
    pose_b_g_arr.push_back(Eigen::Vector3d::Zero());
    imu_preintegrator_arr.clear();
    is_initialized = false;

    edge_map = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    surf_map = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    current_edge_points = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    current_surf_points = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

}

void OdomEstimationClass::init(std::string& file_path){
    common_param.loadParam(file_path);
    lidar_param.loadParam(file_path);
    imu_param.loadParam(file_path);

    if(common_param.getNearbyFrame()>POSE_BUFFER) std::cerr<<"please set POSE_BUFFER = common.nearby_frame! "<<std::endl;
    double map_resolution = lidar_param.getLocalMapResolution();
    //downsampling size
    edge_downsize_filter.setLeafSize(map_resolution, map_resolution, map_resolution);
    surf_downsize_filter.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);
}

void OdomEstimationClass::initMapWithPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr edge_in, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr surf_in){
    *edge_map += *edge_in;
    *surf_map += *surf_in;
    edge_kd_tree.setInputCloud(edge_map);
    surf_kd_tree.setInputCloud(surf_map);
}

bool OdomEstimationClass::initialize(void){
    if(pose_r_arr.size()<common_param.getInitFrame())
        return is_initialized;
    const int start_id = pose_r_arr.size() - common_param.getInitFrame();
    const int end_id = pose_r_arr.size() - 1;

    Eigen::Vector3d acc_mean(0.0,0.0,0.0);
    Eigen::Vector3d gyr_mean(0.0,0.0,0.0);
    int acc_count =0;

    // mean and std of IMU acc
    for(int i=start_id; i<end_id; i++){
        int discarded_imu =0;
        std::vector<Eigen::Vector3d> acc_buf = imu_preintegrator_arr[i].getAcc();
        std::vector<Eigen::Vector3d> gyr_buf = imu_preintegrator_arr[i].getGyr();

        for (int j = 0; j < acc_buf.size(); j++){
            acc_mean+=acc_buf[j];
            gyr_mean+=gyr_buf[j];
            acc_count++;
        }
    }
    acc_mean = acc_mean / acc_count;
    gyr_mean = gyr_mean / acc_count;

    for(int i=start_id; i<end_id;i++){
        imu_preintegrator_arr[i].update(Eigen::Vector3d::Zero(),gyr_mean);
        lidar_odom_arr[i] = Eigen::Isometry3d::Identity();
    }

    if(fabs(Utils::gravity.norm() - acc_mean.norm())>0.02)
        ROS_WARN("the gravity is wrong! measured gravity = %f", acc_mean.norm());
    else
        Utils::gravity = acc_mean;

    ROS_INFO("gravity= %f = %f,%f,%f",Utils::gravity.norm(), Utils::gravity.x(),Utils::gravity.y(),Utils::gravity.z());
    ROS_INFO("gyr bias %f, %f, %f",gyr_mean.x(),gyr_mean.y(),gyr_mean.z());
   
    is_initialized = true;
    return is_initialized;
}

void OdomEstimationClass::addImuPreintegration(std::vector<double> dt_arr, std::vector<Eigen::Vector3d> acc_arr, std::vector<Eigen::Vector3d> gyr_arr){
    ImuPreintegrationClass imu_preintegrator(pose_b_a_arr.back(),pose_b_g_arr.back(), imu_param.getAccN(), imu_param.getGyrN(), imu_param.getAccW(), imu_param.getGyrW());
    for (int i = 0; i < dt_arr.size(); ++i){
        imu_preintegrator.addImuData(dt_arr[i], acc_arr[i], gyr_arr[i]);
    }
    imu_preintegrator_arr.push_back(imu_preintegrator);

    //add pose states
    Eigen::Matrix3d last_R = Utils::so3ToR(pose_r_arr.back());
    if(is_initialized == true){
        pose_r_arr.push_back(Utils::RToso3(last_R * imu_preintegrator.delta_R));
        pose_t_arr.push_back(pose_t_arr.back() - 0.5 * Utils::gravity * imu_preintegrator.sum_dt * imu_preintegrator.sum_dt + pose_v_arr.back() * imu_preintegrator.sum_dt + last_R * imu_preintegrator.delta_p);
        pose_v_arr.push_back(pose_v_arr.back() - Utils::gravity * imu_preintegrator.sum_dt + last_R * imu_preintegrator.delta_v);
    }else{
        pose_r_arr.push_back(Eigen::Vector3d::Zero());
        pose_t_arr.push_back(Eigen::Vector3d::Zero());
        pose_v_arr.push_back(pose_v_arr.back());
    }

    Eigen::Vector3d b_a_hat = pose_b_a_arr.back();
    pose_b_a_arr.push_back(b_a_hat);
    Eigen::Vector3d b_g_hat = pose_b_g_arr.back();
    pose_b_g_arr.push_back(b_g_hat);

    lidar_odom_arr.push_back(Eigen::Isometry3d::Identity());
}

void OdomEstimationClass::addLidarFeature(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr edge_in, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr surf_in){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsized_edge = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsized_surf = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    edge_downsize_filter.setInputCloud(edge_in);
    edge_downsize_filter.filter(*downsized_edge);
    surf_downsize_filter.setInputCloud(surf_in);
    surf_downsize_filter.filter(*downsized_surf); 

    Eigen::Isometry3f T_bl = common_param.getTbl().cast<float>();
    pcl::transformPointCloud(*downsized_edge, *current_edge_points, T_bl);   
    pcl::transformPointCloud(*downsized_surf, *current_surf_points, T_bl);    

}

void OdomEstimationClass::addEdgeCost(ceres::Problem& problem, ceres::LossFunction *loss_function, double* pose){
    int edge_num=0;
    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.linear() = Utils::so3ToR(Eigen::Vector3d(pose[0],pose[1],pose[2]));
    T_wb.translation() = Eigen::Vector3d(pose[3],pose[4],pose[5]);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_edge = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*current_edge_points, *transformed_edge, T_wb.cast<float>());
    for (int i = 0; i < (int)transformed_edge->points.size(); i++){
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        edge_kd_tree.nearestKSearch(transformed_edge->points[i], 5, pointSearchInd, pointSearchSqDis); 
        if (pointSearchSqDis[4] < 1.0)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++){
                Eigen::Vector3d tmp(edge_map->points[pointSearchInd[j]].x,
                                    edge_map->points[pointSearchInd[j]].y,
                                    edge_map->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++){
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(current_edge_points->points[i].x, current_edge_points->points[i].y, current_edge_points->points[i].z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]){ 
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                ceres::CostFunction *cost_function = new LidarEdgeFactor(curr_point, point_a, point_b, lidar_param.getEdgeN());  
                problem.AddResidualBlock(cost_function, loss_function, pose);
                edge_num++;   
            }                           
        }
    }
    if(edge_num<20){
        std::cout<<"not enough correct edge points"<<std::endl;
    }
}

void OdomEstimationClass::addSurfCost(ceres::Problem& problem, ceres::LossFunction *loss_function, double* pose){
    int surf_num=0;
    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.linear() = Utils::so3ToR(Eigen::Vector3d(pose[0],pose[1],pose[2]));
    T_wb.translation() = Eigen::Vector3d(pose[3],pose[4],pose[5]);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_surf = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*current_surf_points, *transformed_surf, T_wb.cast<float>());
    
    for (int i = 0; i < (int) transformed_surf->points.size(); i++){
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        surf_kd_tree.nearestKSearch(transformed_surf->points[i], 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < 1.0){
            for (int j = 0; j < 5; j++){
                matA0(j, 0) = surf_map->points[pointSearchInd[j]].x;
                matA0(j, 1) = surf_map->points[pointSearchInd[j]].y;
                matA0(j, 2) = surf_map->points[pointSearchInd[j]].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * surf_map->points[pointSearchInd[j]].x +
                         norm(1) * surf_map->points[pointSearchInd[j]].y +
                         norm(2) * surf_map->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(current_surf_points->points[i].x, current_surf_points->points[i].y, current_surf_points->points[i].z);
            if (planeValid)
            {
                ceres::CostFunction *cost_function = new LidarSurfFactor(curr_point, norm, negative_OA_dot_norm, lidar_param.getSurfN());    
                problem.AddResidualBlock(cost_function, loss_function, pose);
                surf_num++;
            }
        }

    }
    if(surf_num<20){
        std::cout<<"not enough correct surf points"<<std::endl;
    }
}

void OdomEstimationClass::addImuCost(ImuPreintegrationClass& imu_integrator, ceres::Problem& problem, ceres::LossFunction *loss_function, double* pose1, double* pose2){
    ImuPreintegrationFactor* imu_factor = new ImuPreintegrationFactor(imu_integrator);
    problem.AddResidualBlock(imu_factor, loss_function, pose1, pose2);
}

void OdomEstimationClass::addOdometryCost(const Eigen::Isometry3d& odom, ceres::Problem& problem, ceres::LossFunction *loss_function, double* pose1, double* pose2){
    LidarOdometryFactor* odom_factor = new LidarOdometryFactor(odom, lidar_param.getOdomN());
    problem.AddResidualBlock(odom_factor, loss_function, pose1, pose2);
}

void OdomEstimationClass::optimize(void){
    if(imu_preintegrator_arr.size()!= lidar_odom_arr.size() || lidar_odom_arr.size() != pose_r_arr.size()-1)
        ROS_WARN("pose num and imu num are not properly aligned");

    const int pose_size = pose_r_arr.size()>common_param.getNearbyFrame()?common_param.getNearbyFrame():pose_r_arr.size();
    const int start_id = pose_r_arr.size() - pose_size;
    const int end_id = pose_r_arr.size() - 1;

    double pose[POSE_BUFFER][15];
    for(int i = start_id; i <= end_id; i++){
        const int pose_id = i - start_id;
        pose[pose_id][0] = pose_r_arr[i].x();
        pose[pose_id][1] = pose_r_arr[i].y();
        pose[pose_id][2] = pose_r_arr[i].z();
        pose[pose_id][3] = pose_t_arr[i].x();
        pose[pose_id][4] = pose_t_arr[i].y();
        pose[pose_id][5] = pose_t_arr[i].z();
        pose[pose_id][6] = pose_v_arr[i].x();
        pose[pose_id][7] = pose_v_arr[i].y();
        pose[pose_id][8] = pose_v_arr[i].z();
        pose[pose_id][9] = pose_b_a_arr[i].x();
        pose[pose_id][10] = pose_b_a_arr[i].y();
        pose[pose_id][11] = pose_b_a_arr[i].z();
        pose[pose_id][12] = pose_b_g_arr[i].x();
        pose[pose_id][13] = pose_b_g_arr[i].y();
        pose[pose_id][14] = pose_b_g_arr[i].z();
    }

    for (int iterCount = 0; iterCount < 3; iterCount++){
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.5);
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);

        for(int i = start_id; i <= end_id; i++){
            const int pose_id = i - start_id;
            if(pose_id == 0)
                problem.AddParameterBlock(pose[pose_id], 15, new ConstantPoseParameterization()); 
            else
                problem.AddParameterBlock(pose[pose_id], 15, new PoseParameterization()); 
        }

        //add imu cost factor
        for (int i = start_id; i < end_id; i++){
            const int pose_id = i - start_id;
            addImuCost(imu_preintegrator_arr[i], problem, loss_function, pose[pose_id], pose[pose_id+1]);
        }

        addEdgeCost(problem, loss_function, pose[pose_size-1]);
        addSurfCost(problem, loss_function, pose[pose_size-1]);

        // add odometry cost factor
        for (int i = start_id; i < end_id - 1; i++){
            const int pose_id = i - start_id;
            addOdometryCost(lidar_odom_arr[i], problem, loss_function, pose[pose_id], pose[pose_id+1]);
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 6;
        options.gradient_check_relative_precision = 1e-4;
        options.max_solver_time_in_seconds = 0.08;
        options.num_threads = common_param.getCoreNum();
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
    }  

    for(int i = start_id; i<= end_id; i++){
        const int pose_id = i - start_id;
        pose_r_arr[i].x() = pose[pose_id][0];
        pose_r_arr[i].y() = pose[pose_id][1];
        pose_r_arr[i].z() = pose[pose_id][2];
        pose_t_arr[i].x() = pose[pose_id][3];
        pose_t_arr[i].y() = pose[pose_id][4];
        pose_t_arr[i].z() = pose[pose_id][5];
        pose_v_arr[i].x() = pose[pose_id][6];
        pose_v_arr[i].y() = pose[pose_id][7];
        pose_v_arr[i].z() = pose[pose_id][8];
        pose_b_a_arr[i].x() = pose[pose_id][9];
        pose_b_a_arr[i].y() = pose[pose_id][10];
        pose_b_a_arr[i].z() = pose[pose_id][11];
        pose_b_g_arr[i].x() = pose[pose_id][12];
        pose_b_g_arr[i].y() = pose[pose_id][13];
        pose_b_g_arr[i].z() = pose[pose_id][14];
    }
    
    for(int i = start_id; i < end_id; i++){
        imu_preintegrator_arr[i].update(pose_b_a_arr[i], pose_b_g_arr[i]);
    }
    
    // update odom
    for(int i = end_id - 1; i < end_id; i++){
        Eigen::Matrix3d last_R = Utils::so3ToR(pose_r_arr[i]);
        lidar_odom_arr[i].linear() = last_R.transpose() * Utils::so3ToR(pose_r_arr[i+1]);
        lidar_odom_arr[i].translation() = last_R.transpose() * (pose_t_arr[i+1] - pose_t_arr[i]);
    }

    // update map 
    Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
    current_pose.linear() = Utils::so3ToR(pose_r_arr.back());
    current_pose.translation() = pose_t_arr.back();
    updateLocalMap(current_pose);
}

void OdomEstimationClass::updateLocalMap(Eigen::Isometry3d& transform){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_edge = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*current_edge_points, *transformed_edge, transform.cast<float>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_surf = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*current_surf_points, *transformed_surf, transform.cast<float>());
    
    *edge_map += *transformed_edge;
    *surf_map += *transformed_surf;

    double x_min = transform.translation().x() - lidar_param.getLocalMapSize();
    double y_min = transform.translation().y() - lidar_param.getLocalMapSize();
    double z_min = transform.translation().z() - lidar_param.getLocalMapSize();
    double x_max = transform.translation().x() + lidar_param.getLocalMapSize();
    double y_max = transform.translation().y() + lidar_param.getLocalMapSize();
    double z_max = transform.translation().z() + lidar_param.getLocalMapSize();
    
    pcl::CropBox<pcl::PointXYZRGB> crop_box_filter;
    crop_box_filter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    crop_box_filter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    crop_box_filter.setNegative(false);    

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr edge_map_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr surf_map_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
    crop_box_filter.setInputCloud(edge_map);
    crop_box_filter.filter(*edge_map_temp);
    crop_box_filter.setInputCloud(surf_map);
    crop_box_filter.filter(*surf_map_temp);

    edge_downsize_filter.setInputCloud(edge_map_temp);
    edge_downsize_filter.filter(*edge_map);    
    surf_downsize_filter.setInputCloud(surf_map_temp);
    surf_downsize_filter.filter(*surf_map);

    edge_kd_tree.setInputCloud(edge_map);
    surf_kd_tree.setInputCloud(surf_map);

}
