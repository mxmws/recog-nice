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
	//import
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud = process.plyReader("ball_1.ply");
	//transform
	source_cloud = process.transformationMatrix(source_cloud);
	//remove background
	source_cloud = process.removeBackground(source_cloud);
	//save cloud as ply
	process.plyWriter("ball_1_transformed.ply", source_cloud);



	//// ReferenceModel and ICP
	//pcl::PointCloud<pcl::PointXYZ>::Ptr ball_1_cloud = process.plyReader("ball_1.ply");
	//ReferenceModel refModel1(ball_1_cloud);
	//refModel1.scoreSimilarity(source_cloud);

	////pcl::io::savePLYFileBinary("noObjectTransformed.ply", *source_cloud);
}

