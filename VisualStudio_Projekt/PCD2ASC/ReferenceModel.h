#pragma once
#include <pcl/io/ply_io.h>
#include <string>

class ReferenceModel
{
public:
	ReferenceModel(pcl::PointCloud<pcl::PointXYZ>::Ptr, std::string&); // Constructor
	float scoreSimilarity(pcl::PointCloud<pcl::PointXYZ>::Ptr); // ICP function
	float getScoring() const { return scoring; } // Getter to get results from outside of the class
	std::string getName() const { return name; }
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr referenceCloud; // Pointer to a point cloud as target source
	pcl::PointCloud<pcl::PointXYZ> result; // Point cloud as a result of the ICP function
	float scoring; // How good the ICP result is (better if closer to zero)
	std::string name;
};