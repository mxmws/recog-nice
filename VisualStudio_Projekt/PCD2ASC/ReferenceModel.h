#include <pcl/io/ply_io.h>
#include <string>
using namespace std;

#pragma once
class ReferenceModel
{
public:
	std::string filename;
	ReferenceModel() = default;
	ReferenceModel(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	float scoreSimilarity(pcl::PointCloud<pcl::PointXYZ>::Ptr);
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr referenceCloud;
	pcl::PointCloud<pcl::PointXYZ> Result;
};

