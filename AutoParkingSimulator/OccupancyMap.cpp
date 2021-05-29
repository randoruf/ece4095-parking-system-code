#include <string>
#include <iostream>
#include "PNG.h"
#include "OccupancyMap.h"

GridIndex::GridIndex() {
	i = 0; 
	j = 0; 
}

GridIndex::GridIndex(int pi, int pj) {
	i = pi; 
	j = pj; 
}


void GridIndex::SetIndex(int pi, int pj) {
	i = pi;
	j = pj;
}


OccupancyMap::OccupancyMap() {
	mapImageFile_ = ""; 
	resolution_ = 0;
	height_ = 0;  
	width_ = 0;
	pMap = NULL;
}

void OccupancyMap::InitializeOccupancyMap(std::string mapImageFile, double resolution) {
	mapImageFile_ = mapImageFile; 
	resolution_ = resolution;


	PNG rawMap;
	if (!rawMap.readFromFile(mapImageFile)) {
		throw "Faild to open the PNG parking lot file...";
	}
	height_ = static_cast<int>(rawMap.getHeight());
	width_  = static_cast<int>(rawMap.getWidth());
	// std::cout << "y(height): " << height_ << std::endl; 
	// std::cout << "x(width): " << width_ << std::endl; 
	worldHeight_ = static_cast<double>(height_) * resolution_; 
	worldWidth_  = static_cast<double>(width_)  * resolution_;


	// start to construct the map...
	pMap = new unsigned char[width_ * height_];
	// convert the rawPNG image into grayscale, and then populate the pMap. 
	for (unsigned int i = 0; i < rawMap.getWidth(); i++) {
		for (unsigned int j = 0; j < rawMap.getHeight(); j++) {

			RGBAPixel& pixel = rawMap.getPixel(i, j);
			unsigned int index = i + (j * width_);
			pMap[index] = 255 - pixel.r;
		}
	}

}; 

OccupancyMap::~OccupancyMap() {
	delete[] pMap; 
}

/*
	Convert the world coordinate into the grid index. 
	If the resolution = 0.05, in the world coordinate, each meter can have 1/0.05 = 20 cells. 
*/
GridIndex OccupancyMap::convertworld2GridIndex(double x, double y) const{
	int i = std::ceil(x / resolution_); 
	int j = std::ceil(y / resolution_);
	return GridIndex(i, j);
}


int OccupancyMap::gridIndexToLinearIndex(GridIndex index) const{
	int linear_index;
	linear_index = index.i + (index.j * width_);
	return linear_index;
}

bool OccupancyMap::isValidGridIndex(GridIndex index) const{
	if (index.i >= 0 && index.i < width_ && index.j >= 0 && index.j < height_)
		return true;

	return false;
}

int OccupancyMap::getWidth() const {
	return width_; 
}

int OccupancyMap::getHeight() const{
	return height_; 
}

std::string OccupancyMap::getMapImageName() const {
	return mapImageFile_; 
}

double OccupancyMap::getMapResolution() const {
	return resolution_; 
}

double OccupancyMap::getWorldHeight() const {
	return worldHeight_; 
}

double OccupancyMap::getWorldWidth() const {
	return worldWidth_; 
}