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




/**
 * Loading ply file into PointCloud object and return it.
 *
 * @param Takes the "filename" as a string.
 * @return Returns a PointCloud object.
 * Sources:	https://stackoverflow.com/questions/30764222/how-to-read-ply-file-using-pcl
 */
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

/**
 * Saves a PointCloud as ply file
 *
 * @param Takes a string for "filename" and a PointCloud object.
 * @return void
 */
void Processing::plyWriter(string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
	//navigates to testScans
	experimental::filesystem::path filepath = canonical(experimental::filesystem::path("..") / ".." / "testScans");
	filepath.append(filename);

	//saves PointCloud as ply file
	pcl::io::savePLYFileBinary(filepath.u8string(), *pointcloud);
}


/**
 * Removes points whose coordinates match the given parameters.
 *
 * @param Takes a PointCloud object.
 * @return Return the PointCLoud object without background.
 * Sources: http://docs.pointclouds.org/trunk/classpcl_1_1_extract_indices.html
			https://stackoverflow.com/questions/44921987/removing-points-from-a-pclpointcloudpclpointxyzrgb
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::removeBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	cout << "Removing background...\n";
	//Points to be removed saved in PointIndices
	pcl::PointIndices::Ptr ToBeRemoved(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	for (int i = 0; i < (*cloud).size(); i++)
	{
		// positive X to the right from center, positive Y points upwards from center, positive Z points backwards
		pcl::PointXYZ pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		// remove points whose x/y/z-coordinate is ...
		if ((pt.x > 0.6 || pt.x < -0.13 || pt.y < -0.805 || pt.z < -1.5 ))						
		{
			ToBeRemoved->indices.push_back(i);
		}
	}
	extract.setInputCloud(cloud);
	extract.setIndices(ToBeRemoved);
	//Remove points
	extract.setNegative(true);
	extract.filter(*cloud);
	cout << "Done...\n";
	return cloud;
}

