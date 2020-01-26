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

//outlier removal
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>

//plane model segmentation:
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)

#else
#include "CRealsenseScan.h"
#endif


using namespace std;


/**
* Loading ply file into PointCloud object and return it.
*
* @param filename name of the file that is going to be opened
* @return Returns a PointCloud object.
*
* Sources:	https://stackoverflow.com/questions/30764222/how-to-read-ply-file-using-pcl
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::plyReader(string& filename)
{
	//creates new PointCloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader Reader;
	//reading ply file
	Reader.read(filename, *cloud_ptr);
	return cloud_ptr;
}


/**
* Loading ply file into PointCloud object and return it.
*
* @param sourceCloudFile name of the file that is going to be opened if on Windows
* @return	On Windows:	file opened
*			On Linux:	most recent scan
*
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::startScanning(string& sourceCloudFile)
{
	#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
	//open file to simulate a scan on windows
	return plyReader(sourceCloudFile);
	
	#else

	//make a scan
	CRealsenseScan scan;
	string filename = "halloDasIstUnserScan.ply";
	if (scan.performScanAndSave(filename) == false)
	{
		cout << "Plug in you camera first and hit enter again" << endl;
		cin.get();
		startScanning(sourceCloudFile);
	}

	//open the scan
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = plyReader(filename);

	
	//return if scan is not faulty
	for (int i = 0; i < cloud->size(); i++)
	{
		if (cloud->points[i].z < -4.0)
		{
			return cloud;
		}
	}
	//recursion if scan was faulty
	startScanning(sourceCloudFile);
	
	#endif
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
 * Determines maximum and minimum x/y/z coordinates of a pointcloud.
 * 
 * @param source_cloud Takes a PointCloud object.
 *
 */
void Processing::determineRemovalParameters(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud)
{
	//variables in which the results are saved
	Processing::x_min = source_cloud->points[0].x;
	Processing::x_max = source_cloud->points[0].x;
	Processing::z_min = source_cloud->points[0].z;
	Processing::z_max = source_cloud->points[0].z;
	Processing::y_min = source_cloud->points[0].y;
	Processing::y_max = source_cloud->points[0].y;
	
	//go through each point of the pointcloud
	for (int i = 0; i < (source_cloud->size())-1; i++)
	{
		// positive X to the right from center, positive Y points upwards from center, positive Z points backwards
		pcl::PointXYZ pt(source_cloud->points[i].x, source_cloud->points[i].y, source_cloud->points[i].z);
		//for x (width)
		//determine minimum and maximum x/y/z coordinates
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
		if (Processing::y_min > source_cloud->points[i + 1].y)
		{
			Processing::y_min = source_cloud->points[i + 1].y;
		}
		else if (Processing::y_max < source_cloud->points[i + 1].y)
		{
			Processing::y_max = source_cloud->points[i + 1].y;
		}
	}

}

/**
 * Removes everything outside the maximum/minium-x/y/z parameters to keep only the relevant item.
 *
 * @param source_cloud Takes a PointCloud object.
 * @return Pointcloud with only the relevant item.
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::uptRemoveBackground
(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud)
{
	cout << "Removing background..." << endl;
	//Points to be removed saved in PointIndices
	pcl::PointIndices::Ptr ToBeRemoved(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	for (int i = 0; i < source_cloud->size(); i++)
	{
		// positive X to the right from center, positive Y points upwards from center, positive Z points backwards
		pcl::PointXYZ pt(source_cloud->points[i].x, source_cloud->points[i].y, source_cloud->points[i].z);
		// remove points whose x/y/z-coordinate is ...
		if (pt.x < x_min || pt.x > x_max || pt.y < y_min || pt.y > y_max || pt.z > (z_min - 0.01))
		{
			ToBeRemoved->indices.push_back(i);
		}
	}
	extract.setInputCloud(source_cloud);
	//Remove points which are outside the parameters
	extract.setIndices(ToBeRemoved);
	extract.setNegative(true);
	extract.filter(*source_cloud);
	cout << "Done..." << endl;
	return source_cloud;
}


/**
* Subtracts two pointclouds from each other to the flat ground. 
*
* @param source_cloud_1 for example the plain ground
* @param source_cloud_2 for example objects placed on the ground marking the area that needs to be extracted from source_cloud_1
*
* @return extracted ground
*
* Sources: http://pointclouds.org/documentation/tutorials/octree.php
* http://pointclouds.org/documentation/tutorials/planar_segmentation.php
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr Processing::extractGround(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_1, pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_2)
{
	cout << "extracting ground..." << endl;

	//using spacial partitioning to increase performance from pow(n, 2) to n*log(n)
	float resolution = 0.1f;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
	octree.setInputCloud(source_cloud_2);
	octree.addPointsFromInputCloud();
	pcl::PointXYZ searchPoint;
	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;

	pcl::PointIndices::Ptr ToBeRemoved(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	
	//remove all points from source_cloud_1 that match points in source_cloud_2
	for (int i = 0; i < source_cloud_1->size(); i++)
	{
		searchPoint = source_cloud_1->points[i];

		//Comparing points from both clouds with the same ID is way faster compared to octree and true in most cases
		if(i < source_cloud_2->size())
		{
			pcl::PointXYZ pt2 = source_cloud_2->points[i];

			//measure distance between two points from both clouds with the same ID
			if (sqrt(pow(searchPoint.x - pt2.x, 2) + pow(searchPoint.y - pt2.y, 2) + pow(searchPoint.z - pt2.z, 2)) < 0.05)
			{
				ToBeRemoved->indices.push_back(i);
				continue;
			}
		}

		//search for nearest point in other cloud
		octree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);

		//check if close enough
		if (pointNKNSquaredDistance[0]<0.002)
		{
			ToBeRemoved->indices.push_back(i);
		}

	}

	//remove the points that are similar in both clouds
	extract.setInputCloud(source_cloud_1);
	extract.setIndices(ToBeRemoved);
	//Remove points
	extract.setNegative(true);
	extract.filter(*source_cloud_1);

	//determine a plane from the points left using plane model segmentation
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(source_cloud_1);
	seg.segment(*inliers, *coefficients);

	//remove points that are not part of the plane
	pcl::ExtractIndices<pcl::PointXYZ> extract2;
	extract2.setInputCloud(source_cloud_1);
	extract2.setIndices(inliers);
	//Remove points
	extract2.setNegative(false);
	extract2.filter(*source_cloud_1);

	cout << "Done..." << endl;
	return source_cloud_1;
}


/**
* Determines the angle of a pointcloud that is flat like a plane
*
* @param source_cloud Takes a PointCloud object, preferably a flat one.
*
* Sources: http://pointclouds.org/documentation/tutorials/normal_estimation.php
*/
void Processing::determineAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud)
{
	cout << "determineAngle..." << endl;
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
	int amount_of_relevant_points = cloud_normals->size();

	for (int i = 2; i < cloud_normals->size(); i++)
	{
		//check if normal has a valid value
		if(isnan(cloud_normals->points[i].normal_x) || isnan(cloud_normals->points[i].normal_y) || isnan(cloud_normals->points[i].normal_z))
		{
			amount_of_relevant_points--;
			continue;
		}

		//determine angle for x and y for current normal and add it to variables
		sum_rotation_x += atan(cloud_normals->points[i].normal_y / cloud_normals->points[i].normal_z);
		sum_angle_to_x += acos(cloud_normals->points[i].normal_x
			/ sqrt(pow(cloud_normals->points[i].normal_x, 2)
				+ pow(cloud_normals->points[i].normal_y, 2)
				+ pow(cloud_normals->points[i].normal_z, 2)));
	}

	//determine angle_y and angle_x by averaging
	Processing::angle_y = sum_angle_to_x / amount_of_relevant_points;
	Processing::angle_x = cos(Processing::angle_y) > 0 ?
		sum_rotation_x / amount_of_relevant_points : -(sum_rotation_x / amount_of_relevant_points);

	cout << "angle_x: " << angle_x << endl;
	cout << "angle_y: " << angle_y << endl;
	
	cout << "done..." << endl;
}


/**
* Makes two scans and uses them to determine the parameters for future scans
*/
void Processing::positioning()
{
	cout << "Press enter to scan" << endl;
	cin.get();
	string sourceCloudFile = "plain.ply";
	const pcl::PointCloud<pcl::PointXYZ>::Ptr plain = startScanning(sourceCloudFile);
	cout << "Done..." << endl;
	
	pcl::io::savePLYFileBinary("plain.ply", *plain);//save for debugging

	cout << "Place objects to mark the space you want to use for future scans and press enter to scan" << endl;
	cin.get();
	sourceCloudFile = "objects.ply";
	const pcl::PointCloud<pcl::PointXYZ>::Ptr objects = startScanning(sourceCloudFile);
	cout << "Done..." << endl;
	
	pcl::io::savePLYFileBinary("objects.ply", *objects);//save for debugging

	//extract ground ground from first scan that is different in second scan
	pcl::PointCloud<pcl::PointXYZ>::Ptr extractedGround = extractGround(plain, objects);

	pcl::io::savePLYFileBinary("extractedGround.ply", *extractedGround);//save for debugging
	
	determineAngle(extractedGround);
	determineRemovalParameters(transformationMatrix(extractedGround));
}