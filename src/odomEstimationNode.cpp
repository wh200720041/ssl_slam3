//ros lib
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>

//std lib
#include <chrono>
#include <thread>
#include <queue>
#include <condition_variable>
#include "odomEstimationClass.h"
#include "utils.h"

ros::Publisher pub_odom;
OdomEstimationClass odom_estimator;

std::condition_variable con;
std::mutex sensor_lock; 
std::queue<sensor_msgs::ImuConstPtr> imu_buf;
double last_imu_time = 0;
void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
    sensor_lock.lock();
    if(imu_msg->header.stamp.toSec() <= last_imu_time){
        ROS_WARN("imu message in disorder!");
    }
    last_imu_time = imu_msg->header.stamp.toSec();
    imu_buf.push(imu_msg);
    sensor_lock.unlock();
    con.notify_one();
}

std::queue<sensor_msgs::PointCloud2ConstPtr> surf_points_buf;
std::queue<sensor_msgs::PointCloud2ConstPtr> edge_points_buf;
double last_lidar_time = 0.0;
void edgeCloudHandler(const sensor_msgs::PointCloud2ConstPtr &lidar_msg){
    sensor_lock.lock();
    if(lidar_msg->header.stamp.toSec() <= last_lidar_time){
        ROS_WARN("odom message in disorder!");
        last_lidar_time = lidar_msg->header.stamp.toSec();
    }
    edge_points_buf.push(lidar_msg);
    sensor_lock.unlock();
    con.notify_one();
}
void surfCloudHandler(const sensor_msgs::PointCloud2ConstPtr &lidar_msg){
    sensor_lock.lock();
    surf_points_buf.push(lidar_msg);
    sensor_lock.unlock();
    con.notify_one();
}


std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::vector<sensor_msgs::PointCloud2ConstPtr>>> message_syn_buf;
std::mutex message_lock;
//note that there may be multiple odoms if processing speed is slow
double last_synchronization_time = 0.0;
bool synchronizeSensors(void){
    while(true){
        // no measurements, quit
        if (imu_buf.empty() || edge_points_buf.empty() || surf_points_buf.empty())
            return false;

        double lidar_time = edge_points_buf.front()->header.stamp.toSec();
        double imu_time = imu_buf.front()->header.stamp.toSec();
        if(lidar_time>imu_buf.back()->header.stamp.toSec()){
            return false;;              
        }        
        // check point cloud buf 
        if(edge_points_buf.front()->header.stamp.toSec()<surf_points_buf.front()->header.stamp.toSec()){
            ROS_WARN("time stamp unaligned error and odom discarded, edge points time < surf points time"); 
            edge_points_buf.pop();
            continue;              
        }
        if(edge_points_buf.front()->header.stamp.toSec()>surf_points_buf.front()->header.stamp.toSec()){
            ROS_WARN("time stamp unaligned error and odom discarded, edge points time > surf points time"); 
            surf_points_buf.pop();
            continue;              
        }

        //if received odom for the first time, initilize system only if imu also received
        if(last_synchronization_time == 0.0){
            //if the imu data is also received, change odom time and initialize system
            if(imu_time<=lidar_time){
                last_synchronization_time = lidar_time;
                ROS_WARN("system started!");
            }
            //clear imu buf
            while(!imu_buf.empty() && imu_buf.front()->header.stamp.toSec() < lidar_time)
                imu_buf.pop();
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr edge_points_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr surf_points_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*edge_points_buf.front(), *edge_points_in);
            pcl::fromROSMsg(*surf_points_buf.front(), *surf_points_in);
            odom_estimator.initMapWithPoints(edge_points_in, surf_points_in);
            edge_points_buf.pop();
            surf_points_buf.pop();
            
            // publish first odometry
            nav_msgs::Odometry fusionOdometry;
            fusionOdometry.header.frame_id = "map"; 
            fusionOdometry.child_frame_id = "odom"; 
            fusionOdometry.header.stamp = ros::Time(last_synchronization_time);
            fusionOdometry.pose.pose.orientation.x = 0.0;
            fusionOdometry.pose.pose.orientation.y = 0.0;
            fusionOdometry.pose.pose.orientation.z = 0.0;
            fusionOdometry.pose.pose.orientation.w = 1.0;
            fusionOdometry.pose.pose.position.x = 0.0;
            fusionOdometry.pose.pose.position.y = 0.0;
            fusionOdometry.pose.pose.position.z = 0.0;
            pub_odom.publish(fusionOdometry);
            continue;
        }

        // when all sensor information is ready, process data 
        if(last_synchronization_time != 0.0 && imu_buf.back()->header.stamp.toSec()>=lidar_time){
            std::vector<sensor_msgs::ImuConstPtr> imu_buf_temp;
            std::vector<sensor_msgs::PointCloud2ConstPtr> lidar_buf_temp;
            while (!imu_buf.empty() && imu_buf.front()->header.stamp.toSec() <= lidar_time){
                imu_buf_temp.push_back(imu_buf.front());
                imu_buf.pop();
            }
            if (imu_buf_temp.empty())
                ROS_WARN("no imu between two image");
            lidar_buf_temp.push_back(edge_points_buf.front());
            lidar_buf_temp.push_back(surf_points_buf.front());
            edge_points_buf.pop();
            surf_points_buf.pop();
            message_syn_buf.push_back(std::pair<std::vector<sensor_msgs::ImuConstPtr>, std::vector<sensor_msgs::PointCloud2ConstPtr>>(imu_buf_temp, lidar_buf_temp));
            return true;
        }
    }

}

void odom_estimation(){
    while (true){
        while(message_syn_buf.size()==0){
            std::unique_lock<std::mutex> lk(sensor_lock);
            //if data is not ready, wait for new data input
            con.wait(lk, [&]{return synchronizeSensors();});
        }
        // process data
        double last_time = last_synchronization_time;
        std::vector<double> dt_arr;
        std::vector<Eigen::Vector3d> acc_arr;
        std::vector<Eigen::Vector3d> gyr_arr;
        last_synchronization_time = message_syn_buf[0].second[0]->header.stamp.toSec();
        
        for (sensor_msgs::ImuConstPtr& imu_msg: message_syn_buf[0].first){
            double dt = imu_msg->header.stamp.toSec() - last_time;
            last_time = imu_msg->header.stamp.toSec();
            if(dt>0.1) ROS_WARN("IMU time error");
            dt_arr.push_back(dt);
            acc_arr.push_back(Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z));
            gyr_arr.push_back(Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z));
        }
        if(message_syn_buf[0].first.back()->header.stamp.toSec()<message_syn_buf[0].second[0]->header.stamp.toSec()){
            double dt = message_syn_buf[0].second[0]->header.stamp.toSec() - last_time;
            if(dt>0.1) ROS_WARN("IMU time error");
            dt_arr.push_back(dt);
            acc_arr.push_back(Eigen::Vector3d(message_syn_buf[0].first.back()->linear_acceleration.x, message_syn_buf[0].first.back()->linear_acceleration.y, message_syn_buf[0].first.back()->linear_acceleration.z));
            gyr_arr.push_back(Eigen::Vector3d(message_syn_buf[0].first.back()->angular_velocity.x, message_syn_buf[0].first.back()->angular_velocity.y, message_syn_buf[0].first.back()->angular_velocity.z));
            ROS_WARN_ONCE("imu stamp is not synchronized! perform auto synchronization");
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr edge_points_in(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr surf_points_in(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*(message_syn_buf[0].second[0]), *edge_points_in);
        pcl::fromROSMsg(*(message_syn_buf[0].second[1]), *surf_points_in);

        odom_estimator.addImuPreintegration(dt_arr, acc_arr, gyr_arr);
        odom_estimator.addLidarFeature(edge_points_in, surf_points_in);
        message_syn_buf.erase(message_syn_buf.begin());
        //globa optimization to find the pose
        if(odom_estimator.is_initialized == false){
            if(odom_estimator.initialize()) 
                ROS_WARN("system initialized!");
        }else{
            static TicToc timer("odom estimation");
            timer.tic();
            odom_estimator.optimize();
            timer.toc(300);
        }
        
        Eigen::Quaterniond q_current = Utils::so3Toq(odom_estimator.pose_r_arr.back());
        Eigen::Vector3d t_current = odom_estimator.pose_t_arr.back();
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(t_current.x(), t_current.y(), t_current.z()));
        tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));

        // publish odometry
        nav_msgs::Odometry fusionOdometry;
        fusionOdometry.header.frame_id = "map"; 
        fusionOdometry.child_frame_id = "odom"; 
        fusionOdometry.header.stamp = ros::Time(last_synchronization_time);
        fusionOdometry.pose.pose.orientation.x = q_current.x();
        fusionOdometry.pose.pose.orientation.y = q_current.y();
        fusionOdometry.pose.pose.orientation.z = q_current.z();
        fusionOdometry.pose.pose.orientation.w = q_current.w();
        fusionOdometry.pose.pose.position.x = t_current.x();
        fusionOdometry.pose.pose.position.y = t_current.y();
        fusionOdometry.pose.pose.position.z = t_current.z();
        pub_odom.publish(fusionOdometry);

    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "odom_estimation_node");
    ros::NodeHandle nh("~");
    
    std::string file_path;
    nh.getParam("/file_path", file_path); 
    odom_estimator.init(file_path);

    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("/camera/imu", 100, imuCallback);
    ros::Subscriber sub_edge_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, edgeCloudHandler);
    ros::Subscriber sub_surf_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, surfCloudHandler);

    pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 100);

    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    return 0;
}
