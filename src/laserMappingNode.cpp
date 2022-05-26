// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "laserMappingClass.h"
#include "param.h"
#include "utils.h"

LaserMappingClass laserMapping;
std::mutex mutex_lock;
std::queue<nav_msgs::OdometryConstPtr> odometryBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
Eigen::Isometry3d last_pose = Eigen::Isometry3d::Identity();
ros::Publisher map_pub;
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    mutex_lock.lock();
    odometryBuf.push(msg);
    mutex_lock.unlock();
}

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

int update_count = 0;
int frame_id=0;
void laser_mapping(){
    while(1){
        if(!odometryBuf.empty() && !pointCloudBuf.empty()){

            //read data
            mutex_lock.lock();
            if(!pointCloudBuf.empty() && pointCloudBuf.front()->header.stamp.toSec()<odometryBuf.front()->header.stamp.toSec()){
                ROS_WARN("time stamp unaligned error and pointcloud discarded, pls check your data --> laser mapping node"); 
                pointCloudBuf.pop();
                mutex_lock.unlock();
                continue;              
            }

            if(!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec()){
                odometryBuf.pop();
                ROS_INFO("time stamp unaligned with path final, pls check your data --> laser mapping node");
                mutex_lock.unlock();
                continue;  
            }

            //if time aligned 
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;

            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.linear() = Eigen::Quaterniond(odometryBuf.front()->pose.pose.orientation.w,odometryBuf.front()->pose.pose.orientation.x,odometryBuf.front()->pose.pose.orientation.y,odometryBuf.front()->pose.pose.orientation.z).toRotationMatrix();  
            current_pose.translation() = Eigen::Vector3d(odometryBuf.front()->pose.pose.position.x,odometryBuf.front()->pose.pose.position.y,odometryBuf.front()->pose.pose.position.z);
            pointCloudBuf.pop();
            odometryBuf.pop();
            mutex_lock.unlock();
            
            update_count++;
            Eigen::Isometry3d delta_transform = last_pose.inverse() * current_pose;
            double displacement = delta_transform.translation().squaredNorm();
            Eigen::Quaterniond q_temp(delta_transform.linear());
            double angular_change = 2 * acos(q_temp.w());

            if(displacement>0.2 || angular_change>15 / 180.0 * M_PI){
                // ROS_INFO("update map %f,%f",displacement,angular_change * M_PI / 180.0);
                last_pose = current_pose;
                static TicToc timer("laser mapping");
                timer.tic();
                laserMapping.updateCurrentPointsToMap(pointcloud_in,current_pose);
                timer.toc(30);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_map = laserMapping.getMap();
                sensor_msgs::PointCloud2 PointsMsg;
                pcl::toROSMsg(*pc_map, PointsMsg);
                PointsMsg.header.stamp = pointcloud_time;
                PointsMsg.header.frame_id = "map";
                map_pub.publish(PointsMsg); 
            }else if(update_count%100==20){
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_map = laserMapping.getMap();
                sensor_msgs::PointCloud2 PointsMsg;
                pcl::toROSMsg(*pc_map, PointsMsg);
                PointsMsg.header.stamp = pointcloud_time;
                PointsMsg.header.frame_id = "map";
                map_pub.publish(PointsMsg); 
            }

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    std::string file_path;
    nh.getParam("/file_path", file_path); 
    laserMapping.init(file_path);
    last_pose.translation().x() = 10;
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_filtered", 100, velodyneHandler);
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 100, odomCallback);

    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 100);
    std::thread laser_mapping_process{laser_mapping};

    ros::spin();

    return 0;
}
