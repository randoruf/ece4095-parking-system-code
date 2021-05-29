#pragma once
#include <string>

class GridIndex {
public:
	GridIndex(); 
	GridIndex(int pi, int pj); 
	void SetIndex(int pi, int pj); 
	int i, j;
};


class OccupancyMap {
public:
	OccupancyMap(); 
	void InitializeOccupancyMap(std::string mapImageFile, double resolution);
	~OccupancyMap(); 
	// the pointer of the map...(the range is [0, 255])

	GridIndex convertworld2GridIndex(double x, double y) const;  
	int gridIndexToLinearIndex(GridIndex index) const;
	bool isValidGridIndex(GridIndex index) const;

	//
	std::string getMapImageName() const; 
	double getMapResolution() const; 
	int getWidth() const; 
	int getHeight() const; 
	
	double getWorldWidth() const; 
	double getWorldHeight() const; 

	// public attributes. 
	unsigned char* pMap;
private:
	std::string mapImageFile_; 
	const int originX_ = 0, originY_ = 0;
	int height_, width_; 
	double worldHeight_, worldWidth_; 

	double resolution_; 
};

