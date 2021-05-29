#define _USE_MATH_DEFINES
#include <cmath>

#include "GoalAssignor.h"
#include <ompl/util/RandomNumbers.h>
#include <algorithm>
#include <limits>


int GoalAssignor::aux_factorial (int n){
	int factorial = 1;
	for (int i = 1; i <= n; ++i) {
		factorial *= i;
	}
	return factorial;
}

void GoalAssignor::randomAssignment(ParkingLot& lot) {
	// Initialization the goal assignment problem....
	for (auto& a : lot.agents) {
		a.parkingSlotIdx = -1;
		a.goalPose.setPose(0, 0, 0);
	}
	// goal assigment loop 
	std::vector<ParkingSlotInfo> slotWithAssignedAgent(lot.parkingSpaces.size(), ParkingSlotInfo());

	// ======================================================================================================
	// give each agent a random goal...
	ompl::RNG rng;
	for (int i = 0; i < static_cast<int>(lot.agents.size()); i++) {
		 int tmpParkingIdx = rng.uniformInt(0, lot.parkingSpaces.size() - 1);
		 while (slotWithAssignedAgent[tmpParkingIdx].finalised){
			 tmpParkingIdx = rng.uniformInt(0, lot.parkingSpaces.size() - 1);
		 }

		 // finalize the position
		 slotWithAssignedAgent[tmpParkingIdx].agentIdx = i;
		 slotWithAssignedAgent[tmpParkingIdx].finalised = true;
		 // copy the parking pose to the agent 
		 lot.agents[i].parkingSlotIdx = tmpParkingIdx;
		 lot.agents[i].goalPose = lot.parkingSpaces[tmpParkingIdx];
	}
}


void GoalAssignor::greedyEuclideanAssignment(ParkingLot& lot) {
	// Initialization the goal assignment problem....
	for (auto& a : lot.agents) {
		a.parkingSlotIdx = -1;
		a.goalPose.setPose(0, 0, 0);
	}
	// goal assigment loop 
	std::vector<ParkingSlotInfo> slotWithAssignedAgent(lot.parkingSpaces.size(), ParkingSlotInfo());

	// ===========================================================================================================
	while (!std::all_of(lot.agents.begin(), lot.agents.end(), [](Agent& a) { return a.parkingSlotIdx != -1; })) {
		// solve an iinstance of the problem (get the starting position of each agent. 
		for (int j = 0; j < static_cast<int>(lot.agents.size()); j++) {
			if (lot.agents[j].parkingSlotIdx != -1) {
				continue;
			}

			// get the nearest parking slot using SE(2)... 
			double minDist = std::numeric_limits<double>::max();
			unsigned minParkingSlotIdx = 0;
			for (unsigned slotIdx = 0; slotIdx < lot.parkingSpaces.size(); slotIdx++) {
				if (!slotWithAssignedAgent[slotIdx].finalised) {
					// Euclidean component 
					double dx = lot.agents[j].intialPose.getPoseX() - lot.parkingSpaces[slotIdx].getPoseX();
					double dy = lot.agents[j].intialPose.getPoseY() - lot.parkingSpaces[slotIdx].getPoseY();

					// Angular component 
					double dtheta = std::abs(lot.agents[j].intialPose.getPoseTheta() - lot.parkingSpaces[slotIdx].getPoseTheta());
					dtheta = std::min(dtheta, 2 * M_PI - dtheta);

					// weighted distance 
					double dist = pow(dx, 2) + pow(dy, 2) + (2.6)*pow(dtheta, 2);
					// std::cout << dist << "\n"; 
					if (dist < minDist) {
						minDist = dist;
						minParkingSlotIdx = slotIdx;
					}
				}
			}

			// 
			if (minDist < slotWithAssignedAgent[minParkingSlotIdx].distanceToAgent) {
				slotWithAssignedAgent[minParkingSlotIdx].agentIdx = j;
				slotWithAssignedAgent[minParkingSlotIdx].distanceToAgent = minDist;
			}

		}

		// finalization step....
		for (unsigned pi = 0; pi < slotWithAssignedAgent.size(); pi++) {
			if (slotWithAssignedAgent[pi].agentIdx != -1) {
				slotWithAssignedAgent[pi].finalised = true;
				// give the goal information to the agent....				
				auto& curr_agent = lot.agents[slotWithAssignedAgent[pi].agentIdx];
				curr_agent.parkingSlotIdx = pi;
				
				// check the angle difference 
				double dthetaUp = std::abs(curr_agent.intialPose.getPoseTheta() - lot.parkingSpaces[pi].getPoseTheta());
				dthetaUp = std::min(dthetaUp, 2 * M_PI - dthetaUp);
				// std::cout << dthetaUp * 180 / M_PI << "\n";

				double dthetaDown = std::abs(curr_agent.intialPose.getPoseTheta() + lot.parkingSpaces[pi].getPoseTheta());
				dthetaDown = std::min(dthetaDown, 2 * M_PI - dthetaDown);
				// std::cout << dthetaDown * 180 / M_PI << "\n";

				if (dthetaUp <= dthetaDown) {
					// let agent's goal pose be the parking slot assigned...
					curr_agent.goalPose = lot.parkingSpaces[pi];
				}else{
					curr_agent.goalPose.setPoseX(lot.parkingSpaces[pi].getPoseX());
					curr_agent.goalPose.setPoseTheta(-lot.parkingSpaces[pi].getPoseTheta()); 

					if (lot.parkingSpaces[pi].getPoseTheta() > 0) {
						curr_agent.goalPose.setPoseY(lot.parkingSpaces[pi].getPoseY() + lot.vehcicleInfo.carWheelBase); 
					}else {
						curr_agent.goalPose.setPoseY(lot.parkingSpaces[pi].getPoseY() - lot.vehcicleInfo.carWheelBase); 
					}
				}

			}
		}

	}
}


double GoalAssignor::aux_prioritizedEuclideanAssignment(ParkingLot& lot, std::vector<int>& agentSequence) {
	// Initialization the goal assignment problem....
	for (auto& a : lot.agents) {
		a.parkingSlotIdx = -1;
		a.goalPose.setPose(0, 0, 0);
	}
	// goal assigment loop 
	std::vector<ParkingSlotInfo> slotWithAssignedAgent(lot.parkingSpaces.size(), ParkingSlotInfo());
	double totalCost = 0;

	// ===============================================================================================
	// assign goal to agent based on its order.....
	for (int i = 0; i < static_cast<int>(agentSequence.size()); i++) {
		// the agent wants a goal 
		auto& curr_agent = lot.agents[agentSequence[i]];
		// std::cout << agentSequence[i] << ' ';

		// get the nearest parking slot using SE(2)... 
		double minDist = std::numeric_limits<double>::max();
		unsigned minParkingSlotIdx = 0;
		for (unsigned slotIdx = 0; slotIdx < lot.parkingSpaces.size(); slotIdx++) {
			if (!slotWithAssignedAgent[slotIdx].finalised) {
				// Euclidean component 
				double dx = curr_agent.intialPose.getPoseX() - lot.parkingSpaces[slotIdx].getPoseX();
				double dy = curr_agent.intialPose.getPoseY() - lot.parkingSpaces[slotIdx].getPoseY();
				// Angular component 
				double dtheta = std::abs(curr_agent.intialPose.getPoseTheta() - lot.parkingSpaces[slotIdx].getPoseTheta());
				dtheta = std::min(dtheta, 2 * M_PI - dtheta);
				// weighted distance 
				double dist = pow(dx, 2) + pow(dy, 2) + (2.6)*pow(dtheta, 2);
				// std::cout << dist << "\n"; 
				if (dist < minDist) {
					minDist = dist;
					minParkingSlotIdx = slotIdx;
				}
			}
		}
		// finalize the position
		slotWithAssignedAgent[minParkingSlotIdx].agentIdx = agentSequence[i];
		slotWithAssignedAgent[minParkingSlotIdx].finalised = true;
		// copy the parking pose to the agent 
		curr_agent.parkingSlotIdx = minParkingSlotIdx;
		curr_agent.goalPose = lot.parkingSpaces[minParkingSlotIdx];
		// the total cost (the sum of minimum cost of each agent)
		totalCost += minDist;
	}

	return totalCost; 
}


void GoalAssignor::priortizedEuclideanAssignment(ParkingLot& lot) {	
	// intialize the sequence of order... 
	std::vector<int> agentSequence(lot.agents.size(), -1);
	for (int i = 0; i < static_cast<int>(lot.agents.size()); i++) {
		agentSequence[i] = i; 
	}
	// try at most 6 random order....
	if (lot.agents.size() <= 5) {
		int maxPermutation = GoalAssignor::aux_factorial(static_cast<int>(lot.agents.size()));
		
		// the minimum cost over all permutations..... 
		double minCost = std::numeric_limits<double>::max();
		std::vector<unsigned> minCostAgentsParkingSlotIdx(lot.agents.size(), 0); 

		// Brute force 
		do {
			double tmpCost = aux_prioritizedEuclideanAssignment(lot, agentSequence); 
			if (tmpCost < minCost) {
				minCost = tmpCost;
				for (unsigned ai = 0; ai < lot.agents.size(); ai++) {
					// std::cout << lot.agents[ai].parkingSlotIdx << " " <<  std::endl; 
					minCostAgentsParkingSlotIdx[ai] = lot.agents[ai].parkingSlotIdx;
				}
			}
		} while (std::next_permutation(agentSequence.begin(), agentSequence.end()));

		// pick the minimum cost 
		for (unsigned ai = 0; ai < lot.agents.size(); ai++) {
			lot.agents[ai].parkingSlotIdx = minCostAgentsParkingSlotIdx[ai];
			
			// the agent[ai] will choose a goal.... 
			auto& curr_agent = lot.agents[ai];
			// which parking slot index will the current choose?
			auto pi = minCostAgentsParkingSlotIdx[ai]; 

			// check the angle difference 
			double dthetaUp = std::abs(curr_agent.intialPose.getPoseTheta() - lot.parkingSpaces[pi].getPoseTheta());
			dthetaUp = std::min(dthetaUp, 2 * M_PI - dthetaUp);
			// std::cout << dthetaUp * 180 / M_PI << "\n";

			double dthetaDown = std::abs(curr_agent.intialPose.getPoseTheta() + lot.parkingSpaces[pi].getPoseTheta());
			dthetaDown = std::min(dthetaDown, 2 * M_PI - dthetaDown);
			// std::cout << dthetaDown * 180 / M_PI << "\n";

			if (dthetaUp <= dthetaDown) {
				// let agent's goal pose be the parking slot assigned...
				curr_agent.goalPose = lot.parkingSpaces[pi];
			}
			else {
				// x keep the same....
				curr_agent.goalPose.setPoseX(lot.parkingSpaces[pi].getPoseX());
				// rotation the vehicle....
				curr_agent.goalPose.setPoseTheta(-lot.parkingSpaces[pi].getPoseTheta());
				// rotate and move.....
				if (lot.parkingSpaces[pi].getPoseTheta() > 0) {
					curr_agent.goalPose.setPoseY(lot.parkingSpaces[pi].getPoseY() + lot.vehcicleInfo.carWheelBase);
				}else {
					curr_agent.goalPose.setPoseY(lot.parkingSpaces[pi].getPoseY() - lot.vehcicleInfo.carWheelBase);
				}
			}
		}


	} else {
		// 1. random test up to 6 permutations.....
		int maxPermutation = 120;
		// obtain a time-based seed:
		std::time_t seed = std::chrono::system_clock::now().time_since_epoch().count();
		std::default_random_engine rng(static_cast<unsigned>(seed));
		// the minimum cost over all permutations..... 
		double minCost = std::numeric_limits<double>::max();
		std::vector<unsigned> minCostAgentsParkingSlotIdx(lot.agents.size(), 0);

		// 2. test each permutation and record the minimum permutation...
		for (int k = 0; k < maxPermutation; k++) {
			// shuffle the agent sequence....
			std::shuffle(std::begin(agentSequence), std::end(agentSequence), rng);
			// get the cost of the current agentSequence....
			double tmpCost = aux_prioritizedEuclideanAssignment(lot, agentSequence);
			// get the minCost of permutation.... 
			if (tmpCost < minCost) {
				minCost = tmpCost;
				for (unsigned ai = 0; ai < lot.agents.size(); ai++) {
					// std::cout << lot.agents[ai].parkingSlotIdx << " " <<  std::endl; 
					minCostAgentsParkingSlotIdx[ai] = lot.agents[ai].parkingSlotIdx;
				}
			}
		}

		// 3. pick the minimum cost 
		for (unsigned ai = 0; ai < lot.agents.size(); ai++) {
			lot.agents[ai].parkingSlotIdx = minCostAgentsParkingSlotIdx[ai];
			
			// the agent[ai] will choose a goal.... 
			auto& curr_agent = lot.agents[ai];
			// which parking slot index will the current choose?
			auto pi = minCostAgentsParkingSlotIdx[ai];

			// check the angle difference 
			double dthetaUp = std::abs(curr_agent.intialPose.getPoseTheta() - lot.parkingSpaces[pi].getPoseTheta());
			dthetaUp = std::min(dthetaUp, 2 * M_PI - dthetaUp);
			// std::cout << dthetaUp * 180 / M_PI << "\n";

			double dthetaDown = std::abs(curr_agent.intialPose.getPoseTheta() + lot.parkingSpaces[pi].getPoseTheta());
			dthetaDown = std::min(dthetaDown, 2 * M_PI - dthetaDown);
			// std::cout << dthetaDown * 180 / M_PI << "\n";

			if (dthetaUp <= dthetaDown) {
				// let agent's goal pose be the parking slot assigned...
				curr_agent.goalPose = lot.parkingSpaces[pi];
			}
			else {
				// x keep the same....
				curr_agent.goalPose.setPoseX(lot.parkingSpaces[pi].getPoseX());
				// rotation the vehicle....
				curr_agent.goalPose.setPoseTheta(-lot.parkingSpaces[pi].getPoseTheta());
				// rotate and move.....
				if (lot.parkingSpaces[pi].getPoseTheta() > 0) {
					curr_agent.goalPose.setPoseY(lot.parkingSpaces[pi].getPoseY() + lot.vehcicleInfo.carWheelBase);
				}
				else {
					curr_agent.goalPose.setPoseY(lot.parkingSpaces[pi].getPoseY() - lot.vehcicleInfo.carWheelBase);
				}
			}
		}

	}

}
