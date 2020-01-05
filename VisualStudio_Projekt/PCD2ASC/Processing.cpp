#include "Processing.h"

#include <pcl/common/transforms.h>	// for transformationMatrix
#include <pcl/console/parse.h>		// for transformationMatrix
#include <pcl/filters/crop_box.h>	// for removing background via crop Box
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>			// header that contains the definitions for PCD I/O operations
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>		// for transformationMatrix
#include <pcl/point_types.h>		// for ICP
#include <pcl/registration/icp.h>	// for ICP
#include <string>
#include <filesystem>

using namespace std;


pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::transformationMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud)
{
	//float theta = M_PI / 4; // The angle of rotation in radians
	float theta = -0.6;

	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	// translation is 0 on all axis
	transform_2.translation() << 0.0, 0.0, 0.0;

	// theta radians around X axis
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));

	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

	return transformed_cloud;
}



// Loading PLY file into PointCloud object and returns it
pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::plyReader(string filename)
{
	//navigates to testScans
	experimental::filesystem::path filepath = canonical(experimental::filesystem::path("..") / ".." / "testScans");
	filepath.append(filename);

	//creates new PointCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader Reader;
	Reader.read(filepath.u8string(), *cloud_ptr);
	return cloud_ptr;
}

void Processing::plyWriter(string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
	//navigates to testScans
	experimental::filesystem::path filepath = canonical(experimental::filesystem::path("..") / ".." / "testScans");
	filepath.append(filename);

	//creates new PointCloud
	pcl::io::savePLYFileBinary(filepath.u8string(), *pointcloud);
}


pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::cropItembox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::CropBox<pcl::PointXYZ> boxFilter;

	// X = depth, Y = width, Z = height
								//(minX, minY, minZ, 1.0))
	//boxFilter.setMin(Eigen::Vector4f(1000, 250, 5, 1.0));
								//(maxX, maxY, maxZ, 1.0))
	//boxFilter.setMax(Eigen::Vector4f(2000, 500, 250, 1.0));

									//(minX, minY, minZ, 1.0))
	boxFilter.setMin(Eigen::Vector4f(0,0,0,1.0));	
									//(maxX, maxY, maxZ, 1.0))
	boxFilter.setMax(Eigen::Vector4f(10000,-10000, 10000, 1.0));
	boxFilter.setInputCloud(cloud);

	//create a new filtered point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>);
	boxFilter.filter(*cloudFiltered);

	//string writePath = "Scan_BackgroundRemoval_TestKOPIE.ply";
	string writePath = "ball_1_filtered.ply";

	pcl::io::savePLYFileBinary(writePath, *cloudFiltered);
	return cloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::removeBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr p_obstacles)
{
	//new PointCloud object
	//pcl::PointCloud<pcl::PointXYZ>::Ptr p_obstacles(new pcl::PointCloud<pcl::PointXYZ>);

	//filling PointCLoud object with PLY data
	//pcl::PLYReader Reader;
	//Reader.read("ball_1.ply", *p_obstacles);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	for (int i = 0; i < (*p_obstacles).size(); i++)
	{
		// positive X to the right from center, positive Y points upwards from center, positive Z points backwards
		// z coordinate not working
		pcl::PointXYZ pt(p_obstacles->points[i].x, p_obstacles->points[i].y, p_obstacles->points[i].z);
		if ((pt.x > 0.6||pt.x < -0.07 || pt.y < 0.02 || pt.z > 0 ))						// remove points whose x-coordinate is ...
		{
			inliers->indices.push_back(i);
		}
	}
	extract.setInputCloud(p_obstacles);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*p_obstacles);

	string writePath = "ball_1_filtered.ply";
	pcl::io::savePLYFileBinary(writePath, *p_obstacles);

	return p_obstacles;
}

