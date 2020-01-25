#pragma once
#include <pcl/io/ply_io.h>
#include <string>
#include <vector>
#include <utility> 

using namespace std;


class Processing
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformationMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plyReader(string&);
	pcl::PointCloud<pcl::PointXYZ>::Ptr startScanning(string&);
	pcl::PointCloud<pcl::PointXYZ>::Ptr linuxPlyReader(string&);
	void plyWriter(string, pcl::PointCloud<pcl::PointXYZ>::Ptr);
	pcl::PointCloud<pcl::PointXYZ>::Ptr removeBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void determineRemovalParameters(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	pcl::PointCloud<pcl::PointXYZ>::Ptr uptRemoveBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr);

	pcl::PointCloud<pcl::PointXYZ>::Ptr extractGround(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void positioning();
	void determineAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	// Always rotate x first!
	float angle_x;
	float angle_y;

	float x_min;
	float x_max;
	float z_min;
	float z_max;
	float y_min;
	float y_max;
};