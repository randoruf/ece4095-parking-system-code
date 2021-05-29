#include <iostream>
#include <vector>
#include <cassert>
#include "Simulator.h"
#include "ParkingLot.h"
#include "Visualization.h"


Simulator::Simulator(ParkingLot &lot, unsigned int numCars) : lot_(lot){
	// make sure that the number of agents is a valid number.
	assert(numCars > 0); 
	assert(numCars <= lot.parkingSpaces.size());

	// generate the random intial position 
	for (unsigned int i = 0; i < numCars; i++) {
		lot.agents.push_back(Agent(lot));
		lot.agents[i].generateRandomColor(); 
	}
}

//void Simulator::generateRandomGoals() {
//	// generate the random goal position 
//	for (unsigned int i = 0; i < lot_.agents.size(); i++) {
//		lot_.agents[i].generateRandomGoalPoseAt(); 
//	}
//}

void Simulator::generateRandomStartPosition() {
	// generate the random start position 
	for (unsigned int i = 0; i < lot_.agents.size(); i++) {
		lot_.agents[i].generateRandomInitialPoseAt();
	}
}


void Simulator::startVisualization() {
	Visualization::Visualizator v(lot_);
	
	// reset the rendering  
	for (auto car = lot_.agents.begin(); car != lot_.agents.end(); car++) {
		car->carPathPointIdx = 0;
	}

	v.display();
}


ParkingLot& Simulator::getParkingLot() {
	return lot_;
}


//  simulator time limit 
SimulatorTimer::SimulatorTimer() {
	simStart_ = std::chrono::high_resolution_clock::now();
}

void SimulatorTimer::setTimeLimit(int maxSec) {
	maxSec_ = maxSec; 
}

int SimulatorTimer::getTimeLimit() {
	return maxSec_;
}


SimulatorTimer::SimulatorTimer(int maxSec) {
	simStart_ = std::chrono::high_resolution_clock::now();
	setTimeLimit(maxSec);
}

bool SimulatorTimer::timeLimitExceeded() {
	// 
	if (timeLimitExceeded_) {
		return true; 
	}
	// compute the duration.
	auto simCurr = std::chrono::high_resolution_clock::now();
	if (std::chrono::duration_cast<std::chrono::seconds>(simCurr - simStart_).count() >= maxSec_) {
		timeLimitExceeded_ = true;
		return true; 
	}
	else {
		return false;
	}
}


void SimulatorTimer::resetTimer() {
	simStart_ = std::chrono::high_resolution_clock::now();
	timeLimitExceeded_ = false;
}