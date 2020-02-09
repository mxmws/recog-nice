#include "ReferenceModel.h"
#include "Processing.h"
#include <iostream>							// for std::cout
#include <pcl/io/pcd_io.h>					// header that contains the definitions for PCD I/O operations
#include <pcl/filters/extract_indices.h>	// header to remove points with indices

using namespace std;

int main ()
{
	// Enter the names of the files for the reference models here
	string refModelNames[] = { "ball1.ply", "box1.ply", "car1.ply" };

	Processing process;
	
	vector <ReferenceModel> referenceModels;

	// Fill vector with reference models
	for (string name : refModelNames)
	{
		ReferenceModel refModel(process.plyReader(name), name);
		referenceModels.push_back(refModel);
	}
	
	// Determine parameters like angle for the scan
	process.positioning();
	
	cout << "Press enter to scan an object or type 'x' to end the program" << endl;
	char inputChar = cin.get();

	
	while(inputChar != 'x')
	{

		const ReferenceModel& bestScoringModel = process.doICP(referenceModels);
		
		// Tell the user which reference model has the highest similarity with the scanned object
		cout << "Your scanned object is closest to " << bestScoringModel.getName() << ", with a score of " << bestScoringModel.getScoring() << endl;

		cout << "Press enter to scan an object or type 'x' to end the program" << endl;
		inputChar = cin.get();
	}
}