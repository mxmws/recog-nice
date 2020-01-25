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
	Processing process;

	process.positioning();


	while(true)
	{
		cout << "Press enter to scan an object" << endl;
		cin.get();
		string sourceCloudFile = "test1_filter15.ply";
		pcl::PointCloud<pcl::PointXYZ>::Ptr scannedObject = process.startScanning(sourceCloudFile);
		cout << "Done..." << endl;


		scannedObject = process.transformationMatrix(scannedObject);
		scannedObject = process.uptRemoveBackground(scannedObject);
		pcl::io::savePLYFileBinary("scannedObjectCut.ply", *scannedObject);

		// Load reference model
		ReferenceModel refModel(scannedObject); // source point cloud to check with the reference models
		string checkWith1 = "ball1.ply";
		pcl::PointCloud<pcl::PointXYZ>::Ptr checkBall = process.plyReader(checkWith1); // Ref model 1 (Ball)
		string checkWith2 = "box2.ply";
		pcl::PointCloud<pcl::PointXYZ>::Ptr checkBox = process.plyReader(checkWith2); // Ref model 2 (Box)
		string checkWith3 = "car3.ply";
		pcl::PointCloud<pcl::PointXYZ>::Ptr checkCar = process.plyReader(checkWith3); // Ref model 3 (Car)

		float scores[3]; // Array to save the scoring

		// Use ICP to compare source with 3 different reference models and save the result on an array
		refModel.scoreSimilarity(checkBall);
		scores[0] = refModel.getScoring();
		refModel.scoreSimilarity(checkBox);
		scores[1] = refModel.getScoring();
		refModel.scoreSimilarity(checkCar);
		scores[2] = refModel.getScoring();

		// Give out ICP results
		cout << endl << "------- SCORES ------- (The closer to zero the better) -------" << endl << endl;
		cout << "Scanned object vs Ball: " << scores[0] << endl;
		cout << "Scanned object vs Box: " << scores[1] << endl;
		cout << "Scanned object vs Car: " << scores[2] << endl;
	}
	
	cin.get();
}

