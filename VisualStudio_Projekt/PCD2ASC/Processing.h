#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>		// for ICP
#include <pcl/registration/icp.h>	// for ICP
#include <pcl/filters/crop_box.h>	//for removing background via crop Box

#pragma once
class Processing
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformationMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud);
	void plyReader();
	void removeBackground();
	void cropItembox();
	void compareToReferences();
};