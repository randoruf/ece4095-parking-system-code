#pragma once
#include <chrono>
#include "ParkingLot.h"
#include "Vehicle.h"

class Simulator
{
public:
	Simulator(ParkingLot &lot, unsigned int numCars);
	~Simulator() {};

	void startVisualization();
	ParkingLot& getParkingLot(); 

	void generateRandomGoals();
	void generateRandomStartPosition(); 

private:
	ParkingLot& lot_;
};


class SimulatorTimer
{
private:
	std::chrono::time_point<std::chrono::high_resolution_clock> simStart_;
	int maxSec_ = 600; // 10 mins by default...
	bool timeLimitExceeded_ = false;

public:
	SimulatorTimer();
	SimulatorTimer(int maxSec);
	
	// timer limit 
	void setTimeLimit(int maxSec); 
	int getTimeLimit(); 

	// reset the timer 
	void resetTimer();

	// if the running time exceeds the time limit....
	bool timeLimitExceeded();  
};