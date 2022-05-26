// Author of SSL_SLAM3: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "laserMappingClass.h"

void LaserMappingClass::init(std::string& file_path){
	lidar_param.loadParam(file_path);
	for(int i=0;i<lidar_param.getMapCellWidthRange()*2+1;i++){
		std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> map_height_temp;
		for(int j=0;j<lidar_param.getMapCellHeightRange()*2+1;j++){
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> map_depth_temp;
			for(int k=0;k<lidar_param.getMapCellDepthRange()*2+1;k++){
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
				map_depth_temp.push_back(point_cloud_temp);	
			}
			map_height_temp.push_back(map_depth_temp);
		}
		map.push_back(map_height_temp);
	}

	origin_in_map_x = lidar_param.getMapCellWidthRange();
	origin_in_map_y = lidar_param.getMapCellHeightRange();
	origin_in_map_z = lidar_param.getMapCellDepthRange();
	map_width = lidar_param.getMapCellWidthRange()*2+1;
	map_height = lidar_param.getMapCellHeightRange()*2+1;
	map_depth = lidar_param.getMapCellDepthRange()*2+1;

	//downsampling size
	double map_resolution = lidar_param.getMapResolution();
	downSizeFilter.setLeafSize(map_resolution, map_resolution, map_resolution);
}

void LaserMappingClass::addWidthCellNegative(void){
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> map_height_temp;
	for(int j=0; j < map_height;j++){
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> map_depth_temp;
		for(int k=0;k< map_depth;k++){
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map_height_temp.push_back(map_depth_temp);
	}
	map.insert(map.begin(), map_height_temp);

	origin_in_map_x++;
	map_width++;
}
void LaserMappingClass::addWidthCellPositive(void){
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> map_height_temp;
	for(int j=0; j < map_height;j++){
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> map_depth_temp;
		for(int k=0;k< map_depth;k++){
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map_height_temp.push_back(map_depth_temp);
	}
	map.push_back(map_height_temp);
	map_width++;
}

void LaserMappingClass::addHeightCellNegative(void){
	for(int i=0; i < map_width;i++){
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> map_depth_temp;
		for(int k=0;k<map_depth;k++){
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map[i].insert(map[i].begin(), map_depth_temp);
	}
	origin_in_map_y++;
	map_height++;
}

void LaserMappingClass::addHeightCellPositive(void){
	for(int i=0; i < map_width;i++){
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> map_depth_temp;
		for(int k=0;k<map_depth;k++){
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_temp;
			map_depth_temp.push_back(point_cloud_temp);
		}
		map[i].push_back(map_depth_temp);
	}
	map_height++;
}

void LaserMappingClass::addDepthCellNegative(void){
	for(int i=0; i < map_width;i++){
		for(int j=0;j< map_height;j++){
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_temp;
			map[i][j].insert(map[i][j].begin(), point_cloud_temp);
		}
	}
	origin_in_map_z++;
	map_depth++;
}

void LaserMappingClass::addDepthCellPositive(void){
	for(int i=0; i < map_width;i++){
		for(int j=0;j< map_height;j++){
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_temp;
			map[i][j].push_back(point_cloud_temp);
		}
	}
	map_depth++;
}

//extend map is points exceed size
void LaserMappingClass::checkPoints(int& x, int& y, int& z){

	while(x + lidar_param.getMapCellWidthRange()> map_width-1){
		addWidthCellPositive();
	}
	while(x - lidar_param.getMapCellWidthRange()<0){
		addWidthCellNegative();
		x++;
	}
	while(y + lidar_param.getMapCellHeightRange()> map_height-1){
		addHeightCellPositive();
	}
	while(y - lidar_param.getMapCellHeightRange()<0){
		addHeightCellNegative();
		y++;
	}
	while(z + lidar_param.getMapCellDepthRange()> map_depth-1){
		addDepthCellPositive();
	}
	while(z - lidar_param.getMapCellDepthRange()<0){
		addDepthCellNegative();
		z++;
	}

	//initialize 
	//create object if area is null
	for(int i=x-lidar_param.getMapCellWidthRange();i<x+lidar_param.getMapCellWidthRange()+1;i++){
		for(int j=y-lidar_param.getMapCellHeightRange();j<y+lidar_param.getMapCellHeightRange()+1;j++){
			for(int k=z-lidar_param.getMapCellDepthRange();k<z+lidar_param.getMapCellDepthRange()+1;k++){
				if(map[i][j][k]==NULL){
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
					map[i][j][k] = point_cloud_temp;
				}

			}
				
		}

	}
}

void LaserMappingClass::noiseFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_out){
    //manually remove ceiling, you can remove this part if use for other cases
    pcl::CropBox<pcl::PointXYZRGB> crop_box_filter;
    crop_box_filter.setMin(Eigen::Vector4f(-10.0, -0.8, -10.0, 1.0));
    crop_box_filter.setMax(Eigen::Vector4f(10.0, 10.0, 10.0, 1.0));
    crop_box_filter.setNegative(false);    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_without_ceiling(new pcl::PointCloud<pcl::PointXYZRGB>());
    crop_box_filter.setInputCloud(pc_in);
    crop_box_filter.filter(*pc_without_ceiling);

	downSizeFilter.setInputCloud(pc_without_ceiling);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr down_sized_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
	downSizeFilter.filter(*down_sized_pc);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (down_sized_pc);
	sor.setMeanK (30);
	sor.setStddevMulThresh (lidar_param.getMapResolution() * 50);
	sor.filter (*pc_out);

}

//update points to map 
void LaserMappingClass::updateCurrentPointsToMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, const Eigen::Isometry3d& pose_current){
	
	int currentPosIdX = int(std::floor(pose_current.translation().x() / lidar_param.getMapCellWidth() + 0.5)) + origin_in_map_x;
	int currentPosIdY = int(std::floor(pose_current.translation().y() / lidar_param.getMapCellHeight() + 0.5)) + origin_in_map_y;
	int currentPosIdZ = int(std::floor(pose_current.translation().z() / lidar_param.getMapCellDepth() + 0.5)) + origin_in_map_z;

	//check is submap is null
	checkPoints(currentPosIdX,currentPosIdY,currentPosIdZ);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	
	noiseFilter(pc_in, cloud_filtered);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::transformPointCloud(*cloud_filtered, *transformed_pc, pose_current.cast<float>());
	
	//save points
	for (int i = 0; i < (int)transformed_pc->points.size(); i++)
	{
		pcl::PointXYZRGB point_temp = transformed_pc->points[i];
		//for visualization only
		//point_temp.intensity = std::min(1.0 , std::max(pc_in->points[i].z+2.0, 0.0) / 5);
		int currentPointIdX = int(std::floor(point_temp.x / lidar_param.getMapCellWidth() + 0.5)) + origin_in_map_x;
		int currentPointIdY = int(std::floor(point_temp.y / lidar_param.getMapCellHeight() + 0.5)) + origin_in_map_y;
		int currentPointIdZ = int(std::floor(point_temp.z / lidar_param.getMapCellDepth() + 0.5)) + origin_in_map_z;

		map[currentPointIdX][currentPointIdY][currentPointIdZ]->push_back(point_temp);
		
	}

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr LaserMappingClass::getMap(void){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudMap = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new  pcl::PointCloud<pcl::PointXYZRGB>());
	for (int i = 0; i < map_width; i++){
		for (int j = 0; j < map_height; j++){
			for (int k = 0; k < map_depth; k++){
				if(map[i][j][k]!=NULL){
					*laserCloudMap += *(map[i][j][k]);
				}
			}
		}
	}
	return laserCloudMap;
}

LaserMappingClass::LaserMappingClass(){

}
