#include <pcl/io/ply_io.h>

#pragma once
class Processing
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformationMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud);
	void plyReader();
	void removeBackground();
	void cropItembox();
	void compareToReferences();
};