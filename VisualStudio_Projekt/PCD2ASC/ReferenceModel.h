#include <string>
#include <pcl/io/ply_io.h>

#pragma once
class ReferenceModel
{
public:
	std::string filename;
	ReferenceModel(std::string);
	float scoreSimilarity(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void compareToReferences();
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr referenceCloud;
};

