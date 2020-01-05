#include "ReferenceModel.h"
#include <string>
#include <pcl/point_types.h>		// for ICP
#include <pcl/registration/icp.h>	// for ICP

using namespace std;

ReferenceModel::ReferenceModel(pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud)
{
	referenceCloud = refCloud;
}

float ReferenceModel::scoreSimilarity(pcl::PointCloud<pcl::PointXYZ>::Ptr toCheckWith_ptr) // No parameter because it uses the private variables from the class
{
	float scoring;

	// Using ICP to determine closeness of 2 .ply data
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp2;
	icp2.setInputSource(referenceCloud);
	icp2.setInputTarget(toCheckWith_ptr);

	// Saving the result in a new point cloud
	icp2.align(Result);

	// Giving the results back to the user
	cout << endl << &referenceCloud << " with " << &toCheckWith_ptr << ": ";
	cout << endl << "Has converged: " << icp2.hasConverged();
	scoring = icp2.getFitnessScore();	// Changing the value of scoring based on the score from ICP
	cout << " with score: " << scoring << endl;
	cout << "Transformation Matrix: " << endl << icp2.getFinalTransformation() << endl;
	cout << "Scoring: ";
	return scoring;
}

