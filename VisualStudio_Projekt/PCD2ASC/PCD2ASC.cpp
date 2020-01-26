#include "ReferenceModel.h"
#include "Processing.h"
#include <iostream>							// for std::cout
#include <pcl/io/pcd_io.h>					// header that contains the definitions for PCD I/O operations
#include <pcl/filters/extract_indices.h>	// header to remove points with indices

using namespace std;

int main ()
{
	Processing process;

	process.positioning();
	cout << "Press enter to scan an object or type 'x' to end the program" << endl;
	char inputChar = cin.get();

	// Load reference models
	string checkWith1 = "ball1.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr checkBall = process.plyReader(checkWith1); // Ref model 1 (Ball)
	string checkWith2 = "box1.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr checkBox = process.plyReader(checkWith2); // Ref model 2 (Box)
	string checkWith3 = "car1.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr checkCar = process.plyReader(checkWith3); // Ref model 3 (Car)

	// 3 point cloud reference models to check with the source
	ReferenceModel refModel1(checkBall);
	ReferenceModel refModel2(checkBox);
	ReferenceModel refModel3(checkCar);
	
	while(inputChar != 'x')
	{
		
		string sourceCloudFile = "halloDasIstUnserScan.ply";
		pcl::PointCloud<pcl::PointXYZ>::Ptr scannedObject = process.startScanning(sourceCloudFile);
		cout << "Done..." << endl;


		scannedObject = process.transformationMatrix(scannedObject);
		scannedObject = process.uptRemoveBackground(scannedObject);
		pcl::io::savePLYFileBinary("scannedObjectCut.ply", *scannedObject);

		// Use ICP to compare source with 3 different reference models and save the result on an array
		refModel1.scoreSimilarity(scannedObject);
		refModel2.scoreSimilarity(scannedObject);
		refModel3.scoreSimilarity(scannedObject);

		// Give out ICP results
		cout << endl << "------- SCORES ------- (The closer to zero the better) -------" << endl << endl;
		cout << "Scanned object vs Ball: " << refModel1.getScoring() << endl;
		cout << "Scanned object vs Box: " << refModel2.getScoring() << endl;
		cout << "Scanned object vs Car: " << refModel3.getScoring() << endl;

		// Tell the user which reference model has the highest similarity with the scanned object
		if (refModel1.getScoring() < refModel2.getScoring() && refModel1.getScoring() < refModel3.getScoring())
		{
			cout << endl;
			cout << "Your scanned object is closest to a ball, with a score of " << refModel1.getScoring() << endl;
		}
		else if (refModel2.getScoring() < refModel1.getScoring() && refModel2.getScoring() < refModel3.getScoring())
		{
			cout << endl;
			cout << "Your scanned object is closest to a box, with a score of " << refModel2.getScoring() << endl;
		}
		else
		{
			cout << endl;
			cout << "Your scanned object is closest to a car, with a score of " << refModel3.getScoring() << endl;
		}
		
		cout << "Press enter to scan an object or type 'x' to end the program" << endl;
		inputChar = cin.get();
	}
}