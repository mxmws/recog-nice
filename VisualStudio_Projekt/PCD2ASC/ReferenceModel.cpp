#include "ReferenceModel.h"
#include <string>


ReferenceModel::ReferenceModel(std::string filename)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr referenceCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PLYReader Reader;
	Reader.read(filename, *referenceCloud);
}

float ReferenceModel::scoreSimilarity(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
	float scoring;

	return scoring;
}

