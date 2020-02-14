#pragma once
#include <pcl/io/ply_io.h>
#include <string>
#include "ReferenceModel.h"

using namespace std;


class Processing
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformationMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plyReader(string&);
	pcl::PointCloud<pcl::PointXYZ>::Ptr startScanning(string&);
	pcl::PointCloud<pcl::PointXYZ>::Ptr removeBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void positioning();
	ReferenceModel doICP(vector <ReferenceModel>);

private:
	void determineRemovalParameters(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	pcl::PointCloud<pcl::PointXYZ>::Ptr extractGround(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void determineAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	// Always rotate x first!
	float angle_x = 0;
	float angle_y = 0;

	float x_min = 0;
	float x_max = 0;
	float z_min = 0;
	float z_max = 0;
	float y_min = 0;
	float y_max = 0;
};