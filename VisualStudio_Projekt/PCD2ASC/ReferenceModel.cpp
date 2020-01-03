#include "ReferenceModel.h"
#include <string>
#include <pcl/point_types.h>		// for ICP
#include <pcl/registration/icp.h>	// for ICP

using namespace std;

ReferenceModel::ReferenceModel(std::string filename, std::string filename1)
{
	pcl::PLYReader Reader;
	Reader.read(filename, *referenceCloud);
	Reader.read(filename1, *referenceCloud1);
}

float ReferenceModel::scoreSimilarity() // No parameter because it uses the private variables from the class
{
	float scoring;

	// Using ICP to determine closeness of 2 .ply data
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp2;
	icp2.setInputSource(referenceCloud);
	icp2.setInputTarget(referenceCloud1);

	// Saving the result in a new point cloud
	icp2.align(Result);

	// Giving the results back to the user
	cout << endl << referenceCloud << " with " << referenceCloud1 << ": ";
	cout << endl << "Has converged: " << icp2.hasConverged();
	scoring = icp2.getFitnessScore();	// Changing the value of scoring based on the score from ICP
	cout << " with score: " << scoring << endl;
	cout << "Transformation Matrix: " << endl << icp2.getFinalTransformation() << endl;
	cout << "Scoring: ";
	return scoring;
}

// Old ICP Test (as reference)
void ReferenceModel::compareToReferences(string FileNameStr1, string FileNameStr2) {
#pragma region Test program from PCL
	// Quellcode von PCL Dokumentation
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the CloudIn data
	cloud_in->width = 5;
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize(cloud_in->width * cloud_in->height);
	for (std::size_t i = 0; i < cloud_in->points.size(); ++i)
	{
		cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
		cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
	}
	std::cout << "Saved " << cloud_in->points.size() << " data points to input:"
		<< std::endl;
	for (std::size_t i = 0; i < cloud_in->points.size(); ++i) std::cout << "    " <<
		cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
		cloud_in->points[i].z << std::endl;
	*cloud_out = *cloud_in;
	std::cout << "size:" << cloud_out->points.size() << std::endl;
	for (std::size_t i = 0; i < cloud_in->points.size(); ++i)
		cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
	std::cout << "Transformed " << cloud_in->points.size() << " data points:"
		<< std::endl;
	for (std::size_t i = 0; i < cloud_out->points.size(); ++i)
		std::cout << "    " << cloud_out->points[i].x << " " <<
		cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
#pragma endregion

	// ICP for our own ply models ------------
	// New PointCloud Objects
	pcl::PointCloud<pcl::PointXYZ>::Ptr to_check(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr to_check_with(new pcl::PointCloud<pcl::PointXYZ>);

	// filling PointCloud Objects with .ply Data
	pcl::PLYReader reader;
	reader.read(FileNameStr1, *to_check);
	for (int i = 0; i < (*to_check).size(); i++)
	{
		pcl::PointXYZ pt(to_check->points[i].x, to_check->points[i].y, to_check->points[i].z);
	}

	reader.read(FileNameStr2, *to_check_with);
	for (int i = 0; i < (*to_check_with).size(); i++)
	{
		pcl::PointXYZ pt(to_check_with->points[i].x, to_check_with->points[i].y, to_check_with->points[i].z);
	}

	// Using ICP to determine closeness of 2 .ply data
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp2;
	icp2.setInputSource(to_check);
	icp2.setInputTarget(to_check_with);

	pcl::PointCloud<pcl::PointXYZ> Result;
	icp2.align(Result);
	cout << endl << FileNameStr1 << " with " << FileNameStr2 << ": ";
	cout << endl << "Has converged: " << icp2.hasConverged() << " with score: " << icp2.getFitnessScore() << endl;
	cout << icp2.getFinalTransformation() << endl;
}
