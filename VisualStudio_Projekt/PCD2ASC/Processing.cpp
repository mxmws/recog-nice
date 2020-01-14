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
#include <vector>
#include <utility> 

using namespace std;


/**
* Loading ply file into PointCloud object and return it.
*
* @param filename name of the file that is going to be opened
* @return PointCloud::Ptr Returns a PointCloud object.
*
* Sources:	https://stackoverflow.com/questions/30764222/how-to-read-ply-file-using-pcl
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::plyReader(string& filename)
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
* @param filename name of the file that is going to be saved
* @param pointcloud Takes a string for "filename" and a PointCloud object.
*
* Sources: http://docs.pointclouds.org/1.7.0/classpcl_1_1_p_l_y_writer.html
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
* Rotates point cloud to make removeBackground possible
*
* @param source_cloud Takes a PointCloud object.
* @return Returns the rotated PointCloud object.
*
* Sources: http://pointclouds.org/documentation/tutorials/matrix_transform.php
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::transformationMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud)
{
	cout << "Rotating PointCloud..." << endl;
	//float theta = M_PI / 4; // The angle of rotation in radians
	const float theta = -0.6;

	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	// translation is 0 on all axis
	transform_2.translation() << 0.0, 0.0, 0.0;

	// theta radians around X axis
	transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));

	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);
	
	cout << "Done..." << endl;
	return transformed_cloud;
}

/**
 * Removes points whose coordinates match the given parameters.
 *
 * @param source_cloud Takes a PointCloud object.
 * @return Returns the PointCLoud object without background.
 *
 * Sources: http://docs.pointclouds.org/trunk/classpcl_1_1_extract_indices.html
			https://stackoverflow.com/questions/44921987/removing-points-from-a-pclpointcloudpclpointxyzrgb
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::removeBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud)
{
	cout << "Removing background..." << endl;
	//Points to be removed saved in PointIndices
	pcl::PointIndices::Ptr ToBeRemoved(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	for (int i = 0; i < (*source_cloud).size(); i++)
	{
		// positive X to the right from center, positive Y points upwards from center, positive Z points backwards
		pcl::PointXYZ pt(source_cloud->points[i].x, source_cloud->points[i].y, source_cloud->points[i].z);
		// remove points whose x/y/z-coordinate is ...
		if ((pt.x > 0.6 || pt.x < -0.13 || pt.y < -0.805 || pt.z < -1.5 ))						
		{
			ToBeRemoved->indices.push_back(i);
		}
	}
	extract.setInputCloud(source_cloud);
	extract.setIndices(ToBeRemoved);
	//Remove points
	extract.setNegative(true);
	extract.filter(*source_cloud);
	cout << "Done..." << endl;
	return source_cloud;
}

vector<float> Processing::getRemovalParameters(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud)
{
	float x_min = source_cloud->points[0].x;
	float x_max = source_cloud->points[0].x;
	float z_min = source_cloud->points[0].z;
	float z_max = source_cloud->points[0].z;

	for (int i = 0; i < (source_cloud->size())-1; i++)
	{
		// positive X to the right from center, positive Y points upwards from center, positive Z points backwards
		pcl::PointXYZ pt(source_cloud->points[i].x, source_cloud->points[i].y, source_cloud->points[i].z);
		//for x (width)
		if (x_min > source_cloud->points[i+1].x)
		{
			x_min = source_cloud->points[i+1].x;
		}
		else if (x_max < source_cloud->points[i+1].x)
		{
			x_max = source_cloud->points[i+1].x;
		}
		//for z (depth)
		if (z_min > source_cloud->points[i + 1].z)
		{
			z_min = source_cloud->points[i + 1].z;
		}
		else if (z_max < source_cloud->points[i + 1].z)
		{
			z_max = source_cloud->points[i + 1].z;
		}

	}
	vector <float> parameter = { x_min, x_max, z_min, z_max};
	return parameter;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::TESTremoveBackground
(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, vector<float> parameter)
{
	cout << "Removing background..." << endl;
	//Points to be removed saved in PointIndices
	pcl::PointIndices::Ptr ToBeRemoved(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	for (int i = 0; i < (*source_cloud).size(); i++)
	{
		// positive X to the right from center, positive Y points upwards from center, positive Z points backwards
		pcl::PointXYZ pt(source_cloud->points[i].x, source_cloud->points[i].y, source_cloud->points[i].z);
		// remove points whose x/y/z-coordinate is ...
		if (pt.x < parameter[0] || pt.x > parameter[1] || pt.z < parameter[2] || pt.z > parameter[3])
		{
			ToBeRemoved->indices.push_back(i);
		}
	}
	extract.setInputCloud(source_cloud);
	extract.setIndices(ToBeRemoved);
	//Remove points
	extract.setNegative(true);
	extract.filter(*source_cloud);
	cout << "Done..." << endl;
	return source_cloud;
}