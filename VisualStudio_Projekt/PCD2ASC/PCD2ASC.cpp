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
	string refModelFile1 = "ball_2_transformed_rot.ply";//"ball_2_transformed.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr ball_2_cloud = process.plyReader(refModelFile1);
	ReferenceModel refModel1(ball_2_cloud);
	string refModelFile2 = "box_2_transformed_rot.ply";//"box_2_transformed.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr box_2_cloud = process.plyReader(refModelFile2);
	ReferenceModel refModel2(box_2_cloud);

	
	
	// Scan Box
	// 
	// Import
	string sourceCloudFile = "box_1_transformed_rot.ply";//"box_1.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud = process.plyReader(sourceCloudFile);
	//// Transform
	//sourceCloud = process.transformationMatrix(sourceCloud);
	//// Remove background
	//sourceCloud = process.removeBackground(sourceCloud);
	//// Save cloud as ply (for testing)
	//process.plyWriter("box_1_transformed.ply", sourceCloud);

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
	sourceCloudFile = "ball_1_transformed_rot.ply";//"ball_1.ply";
	//sourceCloud = process.plyReader(sourceCloudFile);
	//// Rotate point cloud
	//sourceCloud = process.transformationMatrix(sourceCloud);
	//// Remove background
	//sourceCloud = process.removeBackground(sourceCloud);
	//// Save cloud as ply (for testing)
	//process.plyWriter("ball_1_transformed.ply", sourceCloud);

	// ReferenceModel and ICP
	// compare Ball 1 to Ball 2 and give out the result to the console
	refModel1.scoreSimilarity(sourceCloud);
	refModel2.scoreSimilarity(sourceCloud);
	cout << endl << "Scan 2: Ball 1" << endl;
	cout << "refModel Ball 2 with Ball 1: " << refModel1.scoring << endl;
	cout << "refModel Box 2 with Ball 1: " << refModel2.scoring << endl;




	////Testing removal parameters
	//string result2meshlab = "result2meshlab.ply";
	//pcl::PointCloud<pcl::PointXYZ>::Ptr result2meshlab_cloud = process.plyReader(result2meshlab);

	//int z = 0;
	//for (int i = 0; i < (result2meshlab_cloud->size()); i++)
	//{
	//	z += 1;
	//}
	//cout << z << "\n";


	//vector<float> test = process.getRemovalParameters(result2meshlab_cloud);

	//cout << test[0] << " " << test[1] << " " << test[2] << " " << test[3] << "\n";

	////Testing removeBackground with new parameters
	//string toilet_paper = "toilet_paper.ply";
	//pcl::PointCloud<pcl::PointXYZ>::Ptr toilet_paper_cloud = process.plyReader(toilet_paper);
	//process.TESTremoveBackground(toilet_paper_cloud, test);
	//process.plyWriter("toilet_paper_transformed.ply", toilet_paper_cloud);


	cin.get();
}

