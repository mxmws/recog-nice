#include "ReferenceModel.h"
#include <pcl/point_types.h>		// for ICP
#include <pcl/registration/icp.h>	// for ICP

using namespace std;


/**
 * Set a new point cloud as reference to be compared with other point clouds
 *
 * @author Michael R
 *
 * @param refCloud A pointer to a new point cloud whose values are going to be compared with
 */
ReferenceModel::ReferenceModel(pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud, string& _name)
{
	referenceCloud = refCloud;
	name = _name;
}


/**
 * Score the similarity between the reference point cloud with the point cloud in the parameter
 *
 * @author Michael R
 * 
 * @param toCheckWith_ptr A pointer to a point cloud that is going to be compared to the point cloud in the class
 * @return The similarity score of two point clouds. The closer it is to 0, the more similar the two point clouds are.
 * 
 * Sources: http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
 *			http://docs.pointclouds.org/trunk/classpcl_1_1_iterative_closest_point.html
 */
float ReferenceModel::scoreSimilarity(pcl::PointCloud<pcl::PointXYZ>::Ptr toCheckWith_ptr)
{
	cout << "ICP..." << endl;
	// Using ICP to determine closeness of 2 .ply data
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaximumIterations(100); // The program will iterate a maximum of 100 times when the function is called
	icp.setMaxCorrespondenceDistance(50.0); // Set maximum distance between two correspondent points
	icp.setRANSACOutlierRejectionThreshold(0.0001); // Set distance threshold for RANSAC outlier rejection loop
	icp.setInputSource(referenceCloud);
	icp.setInputTarget(toCheckWith_ptr);

	// Saving the result in a new point cloud
	icp.align(result);

	// Giving the results back to the user
	cout << endl << &referenceCloud << " with " << &toCheckWith_ptr << ": ";
	cout << endl << "Has converged: " << icp.hasConverged();
	scoring = icp.getFitnessScore();	// Changing the value of scoring (variable from class) based on the score from ICP
	cout << " with score: " << scoring << endl;
	cout << "Transformation Matrix: " << endl << icp.getFinalTransformation() << endl;

	return scoring;
}