#include <pcl/io/ply_io.h>
#include <string>
using namespace std;

#pragma once
class ReferenceModel
{
public:
	std::string filename;
	ReferenceModel() = default;
	ReferenceModel(std::string);
	float scoreSimilarity(pcl::PointCloud<pcl::PointXYZ>::Ptr);
	void compareToReferences(string FileName1, string FileName2);
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr referenceCloud;
};

