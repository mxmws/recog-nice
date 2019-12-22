
#pragma once
class Processing
{
public:
	int transformationMatrix(int argc, char** argv);
	void plyReader();
	void removeBackground();
	void cropItembox();
	void compareToReferences();
};

