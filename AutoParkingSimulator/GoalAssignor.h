#pragma once
#include "ParkingLot.h"

namespace GoalAssignor {

	class ParkingSlotInfo {
	public:
		int agentIdx = -1;
		double distanceToAgent = std::numeric_limits<double>::max();
		bool finalised = false;
	};

	int aux_factorial(int n);
	void randomAssignment(ParkingLot& lot);
	void greedyEuclideanAssignment(ParkingLot& lot);
	void priortizedEuclideanAssignment(ParkingLot& lot);
	double aux_prioritizedEuclideanAssignment(ParkingLot& lot, std::vector<int>& agentSequence);

}