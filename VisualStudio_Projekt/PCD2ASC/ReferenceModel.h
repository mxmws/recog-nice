#pragma once
#include <pcl/io/ply_io.h>

class ReferenceModel
{
public:
	ReferenceModel(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	float scoreSimilarity(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	float scoring;
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr referenceCloud;
	pcl::PointCloud<pcl::PointXYZ> result;
};

