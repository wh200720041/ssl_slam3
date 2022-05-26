// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef _LASER_MAPPING_H_
#define _LASER_MAPPING_H_

//PCL lib
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>

//eigen  lib
#include <Eigen/Dense>
#include <Eigen/Geometry>

//c++ lib
#include <string>
#include <math.h>
#include <vector>

//local lib
#include "param.h"

class LaserMappingClass 
{
    public:
    	LaserMappingClass();
		void init(std::string& file_path);
		void updateCurrentPointsToMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, const Eigen::Isometry3d& pose_current);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr getMap(void);

	private:
		int origin_in_map_x;
		int origin_in_map_y;
		int origin_in_map_z;
		int map_width;
		int map_height;
		int map_depth;
		LidarParam lidar_param;
		std::vector<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>>> map;
		pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter;
		
		void addWidthCellNegative(void);
		void addWidthCellPositive(void);
		void addHeightCellNegative(void);
		void addHeightCellPositive(void);
		void addDepthCellNegative(void);
		void addDepthCellPositive(void);
		void checkPoints(int& x, int& y, int& z);
		void noiseFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out);
};


#endif // _LASER_MAPPING_H_

