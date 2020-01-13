#include "ReferenceModel.h"
#include "Processing.h"
#include <iostream>							// for std::cout
#include <pcl/io/pcd_io.h>					// header that contains the definitions for PCD I/O operations
#include <pcl/filters/extract_indices.h>	// header to remove points with indices
#include <vector>
#include <utility> 
using namespace std;


int main ()
{
	// Processing
	Processing process;
	
	// Load reference Models
	string refModelFile1 = "ball_2_transformed.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr ball_2_cloud = process.plyReader(refModelFile1);
	ReferenceModel refModel1(ball_2_cloud);
	string refModelFile2 = "box_2_transformed.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr box_2_cloud = process.plyReader(refModelFile2);
	ReferenceModel refModel2(box_2_cloud);

	
	
	// Scan Box
	// 
	// Import
	string sourceCloudFile = "box_1.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud = process.plyReader(sourceCloudFile);
	// Transform
	sourceCloud = process.transformationMatrix(sourceCloud);
	// Remove background
	sourceCloud = process.removeBackground(sourceCloud);
	// Save cloud as ply (for testing)
	process.plyWriter("box_1_transformed.ply", sourceCloud);

	// ReferenceModel and ICP
	// Compare Box 1 to Ball 2 and Box 2 and output it
	refModel1.scoreSimilarity(sourceCloud);
	refModel2.scoreSimilarity(sourceCloud);
	cout << endl << "Scan 1: Box 1" << endl;
	cout << "refModel Ball 2 with Box 1: " << refModel1.scoring << endl;
	cout << "refModel Box 2 with Box 1: " << refModel2.scoring << endl;

	
	// Scan Ball
	// 
	// Import
	sourceCloudFile = "ball_1.ply";
	sourceCloud = process.plyReader(sourceCloudFile);
	// Rotate point cloud
	sourceCloud = process.transformationMatrix(sourceCloud);
	// Remove background
	sourceCloud = process.removeBackground(sourceCloud);
	// Save cloud as ply (for testing)
	process.plyWriter("ball_1_transformed.ply", sourceCloud);

	// ReferenceModel and ICP
	// compare Ball 1 to Ball 2 and give out the result to the console
	refModel1.scoreSimilarity(sourceCloud);
	refModel2.scoreSimilarity(sourceCloud);
	cout << endl << "Scan 2: Ball 1" << endl;
	cout << "refModel Ball 2 with Ball 1: " << refModel1.scoring << endl;
	cout << "refModel Box 2 with Ball 1: " << refModel2.scoring << endl;


	//Testing removal parameters
	/*string result2 = "result2.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr result2_cloud = process.plyReader(result2);

	pair<float, float> test = process.removalParameters(result2_cloud);

	cout << test.first << " " << test.second << "\n";*/


	// Test with rotated test scans from GfAI
	sourceCloudFile = "ball_1_transformed_rot.ply";
	sourceCloud = process.plyReader(sourceCloudFile);

	// ICP with rotated ball 1 scan
	refModel1.scoreSimilarity(sourceCloud);
	refModel2.scoreSimilarity(sourceCloud);
	cout << endl << "Scan 3: Ball 1 (rotated)" << endl;
	cout << "refModel Ball 2 with Ball 1 (rotated): " << refModel1.scoring << endl;
	cout << "refModel Box 2 with Ball 1 (rotated): " << refModel2.scoring << endl;

	sourceCloudFile = "box_1_transformed_rot.ply";
	sourceCloud = process.plyReader(sourceCloudFile);

	// ICP with rotated box 1 scan
	refModel1.scoreSimilarity(sourceCloud);
	refModel2.scoreSimilarity(sourceCloud);
	cout << endl << "Scan 4: Box 1 (rotated)" << endl;
	cout << "refModel Ball 2 with Box 1 (rotated): " << refModel1.scoring << endl;
	cout << "refModel Box 2 with Box 1 (rotated): " << refModel2.scoring << endl;

	// Load reference Models with rotated .ply data
	string refModelFile3 = "ball_2_transformed_rot.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr ball_2_rot_cloud = process.plyReader(refModelFile1);
	ReferenceModel refModel3(ball_2_cloud);
	string refModelFile4 = "box_2_transformed_rot.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr box_2_rot_cloud = process.plyReader(refModelFile2);
	ReferenceModel refModel4(box_2_cloud);


	// Test with rotated test scans from GfAI
	sourceCloudFile = "ball_1_transformed_rot.ply";
	sourceCloud = process.plyReader(sourceCloudFile);

	// ICP with rotated ball 1 scan
	refModel3.scoreSimilarity(sourceCloud);
	refModel4.scoreSimilarity(sourceCloud);
	cout << endl << "Scan 5: Ball 1 (rotated)" << endl;
	cout << "refModel Ball 2 (rotated) with Ball 1 (rotated): " << refModel3.scoring << endl;
	cout << "refModel Box 2 (rotated) with Ball 1 (rotated): " << refModel4.scoring << endl;

	sourceCloudFile = "box_1_transformed_rot.ply";
	sourceCloud = process.plyReader(sourceCloudFile);

	// ICP with rotated box 1 scan
	refModel3.scoreSimilarity(sourceCloud);
	refModel4.scoreSimilarity(sourceCloud);
	cout << endl << "Scan 6: Box 1 (rotated)" << endl;
	cout << "refModel Ball 2 (rotated) with Box 1 (rotated): " << refModel3.scoring << endl;
	cout << "refModel Box 2 (rotated) with Box 1 (rotated): " << refModel4.scoring << endl;
	
	cin.get();
}

