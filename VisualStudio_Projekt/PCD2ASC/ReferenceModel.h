#include <pcl/io/ply_io.h>

#pragma once
class ReferenceModel
{
public:
	ReferenceModel(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	float scoreSimilarity(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	float scoring;
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr referenceCloud;
	pcl::PointCloud<pcl::PointXYZ> Result;
};

