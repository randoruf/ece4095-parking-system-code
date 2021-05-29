#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#include <iostream>
#include "PNG.h"
#include "ParkingLot.h"
#include "OccupancyMap.h"

#include "test1.h"
#include "test7.h"
#include "test9.h"
 
#include "BenchmarkGenerator.h"
#include "BenchmarkSolver.h"


int main() {
	// _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	// generate some benchmark problems. (assume there are 50 instances for each number of agents) 
	// for (unsigned i = 1; i < 6; i++) {
	//  	BenchmarkGenerator::randomStartPositionScenario(std::string("./benchmark_problem/parking_lot01_parking_space.txt"), i, 20);
	// }
	// BenchmarkGenerator::randomStartPositionScenario(std::string("./benchmark_problem/parking_lot01_parking_space.txt"), 20, 20);

	// test1::test1(); 
	// test6::test6();
	// test7::test7(); 

	test9::testGoalAssignment();
	// test9::testOneAgent();
	/// test9::testTwoAgent(); 
	// test9::testThreeAgent(); 
	// test9::testFourAgent(); 
	// test9::testFiveAgent();

	return 0; 
}