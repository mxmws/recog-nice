#include "Processing.h"
//#include <stdio.h>
//#include <tchar.h>
//#include <stdlib.h>
#include <iostream>             // for std::cout
#include <string>
#include <fstream>
#include <algorithm>
#include <pcl/io/pcd_io.h>      // header that contains the definitions for PCD I/O operations
#include <iterator>
#include <math.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>		// for ICP
#include <pcl/registration/icp.h>	// for ICP


//for some reason transformationMatrix works without these includes
//#include <pcl/point_cloud.h>
//#include <pcl/console/parse.h>
//#include <pcl/common/transforms.h>

using namespace std;



void showHelp(char * program_name)
{
	std::cout << std::endl;
	std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
	std::cout << "-h:  Show this help." << std::endl;
}


int Processing::transformationMatrix(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PLYReader Reader;
	Reader.read("DeoPLY.ply", *source_cloud);



	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

	float theta = M_PI / 4; // The angle of rotation in radians

	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	// Define a translation of 2.5 meters on the x axis.
	transform_2.translation() << 1100, -850, 0.0;

	// The same rotation matrix as before; theta radians around Z axis
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));

	// Print the transformation
	printf("\nMethod #2: using an Affine3f\n");
	std::cout << transform_2.matrix() << std::endl;

	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);



	string writePath = "DeoTransformed.ply";
	pcl::io::savePLYFileBinary(writePath, *transformed_cloud);

	return 0;
}



// Loading PLY file into PointCloud object and saves it as PLY-Kopie
void Processing::plyReader()
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);

	pcl::PLYReader Reader;

	Reader.read("DeoPLY.ply", *cloud_ptr);

	string writePath = "DeoKOPIE.ply";

	pcl::io::savePLYFileBinary(writePath, *cloud_ptr);
}

void Processing::removeBackground()
{
	//new PointCloud object
	pcl::PointCloud<pcl::PointXYZ>::Ptr p_obstacles(new pcl::PointCloud<pcl::PointXYZ>);

	//filling PointCLoud object with PLY data
	pcl::PLYReader Reader;
	Reader.read("Scan_BackgroundRemoval.ply", *p_obstacles);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	for (int i = 0; i < (*p_obstacles).size(); i++)
	{
		pcl::PointXYZ pt(p_obstacles->points[i].x, p_obstacles->points[i].y, p_obstacles->points[i].z);
		if ((pt.z < 10) || (pt.y < 50))						// remove points whose x-coordinate is >10??? 
		{
			inliers->indices.push_back(i);
		}
	}
	extract.setInputCloud(p_obstacles);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*p_obstacles);

	string writePath = "Scan_BackgroundRemoval_TestKOPIE.ply";

	pcl::io::savePLYFileBinary(writePath, *p_obstacles);
}

 //ICP Test
void Processing::compareToReferences() {
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
	reader.read("DeoPLY.ply", *to_check);
	for (int i = 0; i < (*to_check).size(); i++)
	{
		pcl::PointXYZ pt(to_check->points[i].x, to_check->points[i].y, to_check->points[i].z);
	}

	reader.read("DeoKOPIE.ply", *to_check_with);
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
	cout << endl << "Has converged: " << icp2.hasConverged() << " with score: " << icp2.getFitnessScore() << endl;
	cout << icp2.getFinalTransformation() << endl;
}


	

