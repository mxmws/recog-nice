#include <pcl/io/ply_io.h>
#include <string>
using namespace std;

#pragma once
class ReferenceModel
{
public:
	std::string filename;
	ReferenceModel() = default;
	ReferenceModel(string);
	float scoreSimilarity(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void compareToReferences(string, string); // Only as reference
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr referenceCloud;
	pcl::PointCloud<pcl::PointXYZ> Result;
};

