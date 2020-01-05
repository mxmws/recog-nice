#include "ReferenceModel.h"
#include "Processing.h"
#include <iostream>							// for std::cout
#include <pcl/io/pcd_io.h>					// header that contains the definitions for PCD I/O operations
#include <pcl/filters/extract_indices.h>	//header to remove points with indices


using namespace std;


//int _tmain(int argc, _TCHAR* argv[])
int main ()
{
	//processing
	Processing process;
	
	//load reference Models
	pcl::PointCloud<pcl::PointXYZ>::Ptr ball_2_cloud = process.plyReader("ball_2_transformed.ply");
	ReferenceModel refModel1(ball_2_cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr box_2_cloud = process.plyReader("box_2_transformed.ply");
	ReferenceModel refModel2(box_2_cloud);

	//scan 1
	//import
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud = process.plyReader("ball_1.ply");
	//transform
	source_cloud = process.transformationMatrix(source_cloud);
	//remove background
	source_cloud = process.removeBackground(source_cloud);
	//save cloud as ply
	process.plyWriter("ball_1_transformed.ply", source_cloud);

	// ReferenceModel and ICP
	// Ball 1 with Ball 2 and give out the result to the console
	refModel1.scoreSimilarity(source_cloud);
	cout << endl << "Scan 1: Ball 1" << endl;
	cout << "refModel Ball 1 with Ball 2: " << refModel1.scoring << endl;

	// Ball 1 with Box 2 and give out the result to the console
	refModel2.scoreSimilarity(source_cloud);
	cout << endl << "refModel Ball 1 with Box 2: " << refModel2.scoring << endl;


	//scan 2
	// Import and export box 1 as transformed .ply data
	//import
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud1 = process.plyReader("box_1.ply");
	//transform
	source_cloud1 = process.transformationMatrix(source_cloud1);
	//remove background
	source_cloud1 = process.removeBackground(source_cloud1);
	//save cloud as ply
	process.plyWriter("box_1_transformed.ply", source_cloud1);
	
	// ReferenceModel and ICP
	// Box 1 with Ball 2 and give out the result to the console
	refModel1.scoreSimilarity(source_cloud1);
	cout << endl << "Scan 2: Box 1" << endl;
	cout << "refModel Box 1 with Ball 2: " << refModel1.scoring << endl;

	// Box 1 with Box 2 and give out the result to the console
	refModel2.scoreSimilarity(source_cloud1);
	cout << "refModel Box 1 with Box 2: " << refModel1.scoring << endl;

	cin.get();
}

