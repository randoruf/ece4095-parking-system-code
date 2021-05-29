#include "test9.h"
#include "BenchmarkSolver.h"

void test9::testGoalAssignment() {
	BenchmarkSolver benchSolver;
	benchSolver.solve_test_goal_assignment("./benchmark_problem/parking_lot01_parking_space_agent5_instance20.txt");
}


void test9::testOneAgent() {
	BenchmarkSolver benchSolver;
	benchSolver.solve_test_one_agent();
}

void test9::testTwoAgent() {
	BenchmarkSolver benchSolver;
	benchSolver.solve_test_two_agent(); 
}

void test9::testThreeAgent() {
	BenchmarkSolver benchSolver;
	benchSolver.solve("./benchmark_problem/parking_lot01_parking_space_agent3_instance20.txt", 
					   "./experimental_data/three_agent/");
}

void test9::testFourAgent() {
	BenchmarkSolver benchSolver;
	benchSolver.solve("./benchmark_problem/parking_lot01_parking_space_agent4_instance20.txt",
					  "./experimental_data/four_agent/");
}

void test9::testFiveAgent() {
	BenchmarkSolver benchSolver;
	benchSolver.solve("./benchmark_problem/parking_lot01_parking_space_agent5_instance20.txt",
		"./experimental_data/five_agent/");
}
