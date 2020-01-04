#include <pcl/io/ply_io.h>
#include <string>
using namespace std;
#pragma once
class Processing
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformationMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	pcl::PointCloud<pcl::PointXYZ>::Ptr plyReader(string);
	void removeBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void cropItembox();
};