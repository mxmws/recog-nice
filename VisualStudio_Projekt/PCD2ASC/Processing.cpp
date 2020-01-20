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

#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>

using namespace std;

// PLY READER und PLY WRITER auskommentiert damit der Raspi läuft

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

pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::linuxPlyReader(string& filename)
{
	//creates new PointCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader Reader;
	//reading ply file
	Reader.read(filename, *cloud_ptr);
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
	Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();

	// translation is 0 on all axis
	transform_2.translation() << 0.0, 0.0, 0.0;

	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	// rotate around x axis
	transform_2.rotate(Eigen::AngleAxisf(angle_x, Eigen::Vector3f::UnitX()));
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_2);

	// rotate around y axis
	transform_2.rotate(Eigen::AngleAxisf(angle_y, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform_2);

	transform_3.rotate(Eigen::AngleAxisf(3.141/2, Eigen::Vector3f::UnitY()));
	pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform_3);

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

void Processing::determineRemovalParameters(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud)
{
	Processing::x_min = source_cloud->points[0].x;
	Processing::x_max = source_cloud->points[0].x;
	Processing::z_min = source_cloud->points[0].z;
	Processing::z_max = source_cloud->points[0].z;
	Processing::y_max = source_cloud->points[0].y;

	for (int i = 0; i < (source_cloud->size())-1; i++)
	{
		// positive X to the right from center, positive Y points upwards from center, positive Z points backwards
		pcl::PointXYZ pt(source_cloud->points[i].x, source_cloud->points[i].y, source_cloud->points[i].z);
		//for x (width)
		if (Processing::x_min > source_cloud->points[i+1].x)
		{
			Processing::x_min = source_cloud->points[i+1].x;
		}
		else if (Processing::x_max < source_cloud->points[i+1].x)
		{
			Processing::x_max = source_cloud->points[i+1].x;
		}
		//for z (depth)
		if (Processing::z_min > source_cloud->points[i + 1].z)
		{
			Processing::z_min = source_cloud->points[i + 1].z;
		}
		else if (Processing::z_max < source_cloud->points[i + 1].z)
		{
			Processing::z_max = source_cloud->points[i + 1].z;
		}
		//for y (height)
		if (Processing::y_max < source_cloud->points[i + 1].y)
		{
			Processing::y_max = source_cloud->points[i + 1].y;
		}

	}

}


pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::uptRemoveBackground
(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud)
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
		if (pt.x < x_min || pt.x > x_max || pt.z < z_min || pt.z > z_max /*|| pt.y < y_max*/)
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






pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::extractGround(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_1, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_2)
{
	cout << "extracting ground... (luckily this won't take 20 Minutes anymore)" << endl;

	float resolution = 0.1f;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

	octree.setInputCloud(source_cloud_2);
	octree.addPointsFromInputCloud();

	pcl::PointXYZ searchPoint;

	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;

	pcl::PointIndices::Ptr ToBeRemoved(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	for (int i = 0; i < (*source_cloud_1).size(); i++)
	{
		searchPoint = source_cloud_1->points[i];
		pcl::PointXYZ pt2 = source_cloud_2->points[i];

		//if is about twice as fast and true in most cases
		if (sqrt(pow(searchPoint.x - pt2.x, 2) + pow(searchPoint.y - pt2.y, 2) + pow(searchPoint.z - pt2.z, 2)) < 0.05)
		{
			ToBeRemoved->indices.push_back(i);
		}
		else
		{
			octree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);

			if (pointNKNSquaredDistance[0]<0.002)
			{
				ToBeRemoved->indices.push_back(i);
			}
		}

	}

	extract.setInputCloud(source_cloud_1);
	extract.setIndices(ToBeRemoved);
	//Remove points
	extract.setNegative(true);
	extract.filter(*source_cloud_1);


	pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter(true);
	rorfilter.setInputCloud(source_cloud_1);
	rorfilter.setRadiusSearch(0.1);
	rorfilter.setMinNeighborsInRadius(10);
	rorfilter.filter(*source_cloud_1);

	cout << "Done..." << endl;
	return source_cloud_1;
}


void Processing::determineAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud)
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(source_cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch(0.03);

	// Compute the features
	ne.compute(*cloud_normals);




	float sum_rotation_x = 0.0;
	float sum_angle_to_x = 0.0;

	for (int i = 2; i < cloud_normals->size(); i++)
	{
		sum_rotation_x += atan(cloud_normals->points[i].normal_y / cloud_normals->points[i].normal_z);

		sum_angle_to_x += acos(cloud_normals->points[i].normal_x
			/ sqrt(pow(cloud_normals->points[i].normal_x, 2)
				+ pow(cloud_normals->points[i].normal_y, 2)
				+ pow(cloud_normals->points[i].normal_z, 2)));
	}

	Processing::angle_y = sum_angle_to_x / (cloud_normals->size() - 2);

	Processing::angle_x = cos(Processing::angle_y) > 0 ?
		sum_rotation_x / (cloud_normals->size() - 2) : -(sum_rotation_x / (cloud_normals->size() - 2));

}

void Processing::positioning()
{
	/*
	string sourceCloudFile = "currentScan.ply";
	
	cout << "Press enter to scan" << endl;
	cin.get();
	//scan function here...
	cout << "Done..." << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr plain = linuxPlyReader(sourceCloudFile);

	cout << "Place four objects to mark the space you want to use and press enter to scan" << endl;
	cin.get();
	//scan function here...
	cout << "Done..." << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr objects = linuxPlyReader(sourceCloudFile);
	*/
	
	string sourceCloudFile = "plain.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr plain = linuxPlyReader(sourceCloudFile);
	sourceCloudFile = "toilet_paper.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr toilet_paper = linuxPlyReader(sourceCloudFile);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = extractGround(plain, toilet_paper);

	determineAngle(cloud);
	determineRemovalParameters(transformationMatrix(cloud));

	
	//wird später entfernt
	toilet_paper = transformationMatrix(toilet_paper);
	pcl::io::savePLYFileBinary("toilet_paperTransformed.ply", *toilet_paper);
	toilet_paper = uptRemoveBackground(toilet_paper);
	
	//plyWriter("result3out.ply", cloud);
	pcl::io::savePLYFileBinary("toilet_paperTransformedCut.ply", *toilet_paper);

}