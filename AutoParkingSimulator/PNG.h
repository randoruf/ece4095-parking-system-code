#pragma once
#include "RGBAPixel.h"
#include <string>

class PNG {
public:
	PNG();
	PNG(unsigned int width, unsigned int height); 
	~PNG();

	bool readFromFile(std::string const& fileName);
	bool writeToFile(std::string const& fileName);

	unsigned int getWidth() const; 
	unsigned int getHeight() const; 
	RGBAPixel& getPixel(unsigned int i, unsigned int j) const; 

private:
	unsigned int width_; 
	unsigned int height_; 
	unsigned int size_; 
	RGBAPixel* imageData_; // an array of pixel. 
};