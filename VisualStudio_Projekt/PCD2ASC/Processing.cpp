#include "Processing.h"
#include <stdio.h>
#include <tchar.h>


#include <stdlib.h>
#include <iostream>             // for std::cout
#include <string>
#include <fstream>
#include <algorithm>
#include <pcl/io/pcd_io.h>      // header that contains the definitions for PCD I/O operations
#include <iterator>
#include <math.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>		// for ICP
#include <pcl/registration/icp.h>	// for ICP
using namespace std;

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
	Reader.read("Scan_Entfernen_Test.ply", *p_obstacles);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	for (int i = 0; i < (*p_obstacles).size(); i++)
	{
		pcl::PointXYZ pt(p_obstacles->points[i].x, p_obstacles->points[i].y, p_obstacles->points[i].z);
		if (pt.x < 10)							// remove points whose x-coordinate is >10??? 
		{
			inliers->indices.push_back(i);
		}
	}
	extract.setInputCloud(p_obstacles);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*p_obstacles);

	string writePath = "Scan_Entfernen_TestKOPIE.ply";

	pcl::io::savePLYFileBinary(writePath, *p_obstacles);
}

// ICP Test - does not work yet
//void Processing::compareToReferences() {
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointNormal>);
//	pcl::PCDReader Reader;
//	Reader.read("DeoPLY.ply", *cloud_ptr);
//	
//	pcl::IterativeClosestPoint<pcl::PLYReader, pcl::PLYReader> icp;
//	icp.setInputSource(*Reader);
//
//}

	//does not work yet
//void Processing::cloudViewer()
//{
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
//	pcl::PLYReader Reader;
//	Reader.read("DeoPLY.ply", *cloud);
//	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
//	viewer.showCloud(cloud);
//	while (!viewer.wasStopped())
//	{
//	}
//}

