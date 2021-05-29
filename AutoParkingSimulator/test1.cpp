#include "test1.h"
#include "ParkingLot.h"
#include "Visualization.h"

void test1::test1() {
	ParkingLot lot("./benchmark_problem/parking_lot01_parking_space.txt");
	// generate the random intial position 
	for (unsigned int i = 0; i < 4; i++) {
		lot.agents.emplace_back(Agent(lot));
		lot.agents[i].generateRandomColor();
		lot.agents[i].generateRandomInitialPoseAt();
	}


	// 
	lot.agents[0].currPose.setPose(14.0739, 10.8, 0.0);
	lot.agents[1].currPose.setPose(22.234, 10.8415, 3.01062);
	lot.agents[2].currPose.setPose(17, 8, 3.01062);


	// 
	lot.agents[3].currPose.setPose(16, 7, 2.01062);
	std::vector<Agent*> collideCars = lot.adjacentConfictCars(lot.agents[3]); 
	if (collideCars.size() == 2 && 
		collideCars[0] == &lot.agents[0] && 
		collideCars[1] == &lot.agents[2]) {
		std::cout << "test 1.1 Looks good\n"; 
	}
	else
	{
		std::cout << "\033[31mtest 1.1 - Opps \033[0m\n"; 
	}

	Pose p1 = Pose(22.234, 10.8415, 3.01062);
	Pose p2 = Pose(14.0739, 10.8, 0.0);
	if (!lot.collisionFree(p2, p1)) {
		std::cout << "test 1.2 Looks good\n"; 
	}else{
		std::cout << "\033[31mtest 1.2 - Opps \033[0m\n";
	}


	p1.setPose(24.234, 10.8415, 3.01062); 
	p2 = lot.agents[1].currPose; 
	if (!lot.collisionFree(p1, p2)) {
		std::cout << "test 1.3 Looks good (24,234, 10.8415, 3.01062) should collide with (22.234, 10.8415, 3.01062), \n";
	}else {
		std::cout << "\033[31mtest 1.3 - Opps \033[0m\n";
	}

	// 
	// Pose p = Pose(24.234, 10.8415, 3.01062);
	// if (lot.collisionFreeAdjacentToAgent(p, lot.agents[3])) {
	//	std::cout << "test 1.4 Looks good (24.234, 10.8415, 3.01062) should ignore (22.234, 10.8415, 3.01062)\n"; 
	//}
	//else {
	//	std::cout << "\033[31mtest 1.4 - Opps \033[0m\n";
	//}


	//
	Visualization::Visualizator v(lot);
	v.display();
}
