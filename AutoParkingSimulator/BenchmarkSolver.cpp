#include "BenchmarkSolver.h"
#include "ParkingLot.h"
#include "Visualization.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "GoalAssignor.h"
#include "Simulator.h"
#include "Planner.h"

// nothing in the class constructor... 
BenchmarkSolver::BenchmarkSolver() {}

// read the benchmark 
void BenchmarkSolver::solve_test_goal_assignment(std::string benchmarkFilename) {
	// open the benchmark file as stream 
	std::ifstream benchmarkFile (benchmarkFilename);
	std::string line;
	if (!benchmarkFile.is_open()) {
		std::cout << "\033[31m ERROR - the benchmark problem can not be opened.... \033[0m \n";
		exit(1);
	}else
	{
		std::cout << "finished loading the benchmark problem: " << benchmarkFilename << "\n";
	}

	std::getline(benchmarkFile, line);
	// the parking lot being planned (once we know how many cars in the parking lot, then a virtual parking lot can be constructed)... 
	ParkingLot lot("./benchmark_problem/" + line);

	// the number of agents 
	std::getline(benchmarkFile, line); 
	int num_agent = std::atoi(line.c_str());
	// instantiate some new vehicles to the parking lot to enable us to simulate.... 
	for (int i = 0; i < num_agent; i++) {
		lot.agents.emplace_back(Agent(lot));
		lot.agents[i].generateRandomColor();
		lot.agents[i].generateRandomInitialPoseAt();
	}

	// the number of instances 
	std::getline(benchmarkFile, line); 
	int num_instance = std::atoi(line.c_str()); 


	Visualization::Visualizator v(lot);
	// read each instance (under each instance, we have a number of agents) 
	for (int i = 0; i < num_instance; i++) {
		if (i != 0) continue; 
		/* Preparing for the problem solving...... */
		std::cout << "----------------------\nInstance: " << (i + 1) << "\n";	
		// load the benchmark problem (random initial position from the benchmark file). 
		for (int j = 0; j < num_agent; j++) {
			// get the line and convert it to a string stream to use stream operations. 
			std::getline(benchmarkFile, line);
			std::istringstream iss(line);
			// initial position of the benchmark. 
			double x, y, theta;
			iss >> x >> y >> theta;
			// update the instance of problem configuration. 
			lot.agents[j].intialPose.setPose(x, y, theta);
			lot.agents[j].currPose.setPose(x, y, theta);
		}

		// ==========================================================================================================
		GoalAssignor::randomAssignment(lot);
		v.display();

		// ==========================================================================================================
		GoalAssignor::greedyEuclideanAssignment(lot);
		v.display();

		// ==========================================================================================================
		GoalAssignor::priortizedEuclideanAssignment(lot);
		v.display(); 
	}

	benchmarkFile.close(); 
}



void BenchmarkSolver::solve_test_one_agent() {
	// open the benchmark file as stream 
	std::ifstream benchmarkFile("./benchmark_problem/parking_lot01_parking_space_agent1_instance20.txt");
	// how many agents for this problem? 
	std::string experimentNumAgent("./experimental_data/one_agent/"); 

	/********************************************************************************************************************************************
	*********************************************************************************************************************************************/
	std::string line;
	if (!benchmarkFile.is_open()) {
		std::cout << "\033[31m ERROR - the benchmark problem can not be opened.... \033[0m \n";
		exit(1);
	}else{
		std::cout << "finished loading the benchmark problem. \n";
	}

	std::getline(benchmarkFile, line);
	// the parking lot being planned (once we know how many cars in the parking lot, then a virtual parking lot can be constructed)... 
	ParkingLot lot("./benchmark_problem/" + line);
	// the number of agents 
	std::getline(benchmarkFile, line);
	int num_agent = std::atoi(line.c_str());
	// instantiate some new vehicles to the parking lot to enable us to simulate.... 
	for (int i = 0; i < num_agent; i++) {
		lot.agents.emplace_back(Agent(lot));
		lot.agents[i].generateRandomColor();
		lot.agents[i].generateRandomInitialPoseAt();
	}
	// the number of instances 
	std::getline(benchmarkFile, line);
	int num_instance = std::atoi(line.c_str());

	// the visulization 
	// Visualization::Visualizator v(lot);
	Planner::initilizePlanner(lot);

	// read each instance (under each instance, we have a number of agents) 
	for (int i = 0; i < num_instance; i++) {
		/* Preparing for the problem solving...... */
		std::cout << "----------------------\nInstance: " << (i + 1) << "\n";
		// load the benchmark problem (random initial position from the benchmark file). 
		for (int j = 0; j < num_agent; j++) {
			// get the line and convert it to a string stream to use stream operations. 
			std::getline(benchmarkFile, line);
			std::istringstream iss(line);
			// initial position of the benchmark. 
			double x, y, theta;
			iss >> x >> y >> theta;
			// update the instance of problem configuration. 
			lot.agents[j].intialPose.setPose(x, y, theta);
			lot.agents[j].currPose.setPose(x, y, theta);
		}

		// ==========================================================================================================================
		// Goal Assignment 
		for (int m = 0; m < 3; m++) {
			std::string experimentDataFilename; 

			if (m == 0) {
				// ==========================================================================================================
				GoalAssignor::randomAssignment(lot);
				experimentDataFilename = experimentNumAgent + "2020_05_20_random_goal_agent";
				// v.display();
			}

			if (m == 1) {
				// ==========================================================================================================
				GoalAssignor::greedyEuclideanAssignment(lot);
				experimentDataFilename = experimentNumAgent + "2020_05_20_greedy_euclidean_goal_agent";
				// v.display();
			}

			if (m == 2) {
				// ==========================================================================================================
				GoalAssignor::priortizedEuclideanAssignment(lot);
				experimentDataFilename = experimentNumAgent + "2020_05_20_prioritized_euclidean_goal_agent";
				// v.display(); 
			}

			// Solve the problem.....
			SimulatorTimer timer;
			Planner::firstPath(lot, timer);
			Planner::refinePath(lot, timer);
		
			// v.display(); 
			experimentDataFilename += std::to_string(num_agent);
			experimentDataFilename += "_instance";
			experimentDataFilename += std::to_string(i + 1);
			experimentDataFilename += ".txt";
			savePathSolutionToTextFile(experimentDataFilename, lot);
		}
	}

	benchmarkFile.close();
}



void BenchmarkSolver::solve_test_two_agent() {
	// open the benchmark file as stream 
	std::ifstream benchmarkFile("./benchmark_problem/parking_lot01_parking_space_agent2_instance20.txt");
	// how many agents for this problem? 
	std::string experimentNumAgent("./experimental_data/two_agent/");

	/********************************************************************************************************************************************
	*********************************************************************************************************************************************/
	std::string line;
	if (!benchmarkFile.is_open()) {
		std::cout << "\033[31m ERROR - the benchmark problem can not be opened.... \033[0m \n";
		exit(1);
	}
	else {
		std::cout << "finished loading the benchmark problem. \n";
	}

	std::getline(benchmarkFile, line);
	// the parking lot being planned (once we know how many cars in the parking lot, then a virtual parking lot can be constructed)... 
	ParkingLot lot("./benchmark_problem/" + line);
	// the number of agents 
	std::getline(benchmarkFile, line);
	int num_agent = std::atoi(line.c_str());
	// instantiate some new vehicles to the parking lot to enable us to simulate.... 
	for (int i = 0; i < num_agent; i++) {
		lot.agents.emplace_back(Agent(lot));
		lot.agents[i].generateRandomColor();
		lot.agents[i].generateRandomInitialPoseAt();
	}
	// the number of instances 
	std::getline(benchmarkFile, line);
	int num_instance = std::atoi(line.c_str());

	// the visulization 
	// Visualization::Visualizator v(lot);
	Planner::initilizePlanner(lot);

	// read each instance (under each instance, we have a number of agents) 
	for (int i = 0; i < num_instance; i++) {
		/* Preparing for the problem solving...... */
		std::cout << "----------------------\nInstance: " << (i + 1) << "\n";
		// load the benchmark problem (random initial position from the benchmark file). 
		for (int j = 0; j < num_agent; j++) {
			// get the line and convert it to a string stream to use stream operations. 
			std::getline(benchmarkFile, line);
			std::istringstream iss(line);
			// initial position of the benchmark. 
			double x, y, theta;
			iss >> x >> y >> theta;
			// update the instance of problem configuration. 
			lot.agents[j].intialPose.setPose(x, y, theta);
			lot.agents[j].currPose.setPose(x, y, theta);
		}

		// ==========================================================================================================================
		// Goal Assignment 
		for (int m = 0; m < 3; m++) {
			std::string experimentDataFilename;

			if (m == 0) {
				// ==========================================================================================================
				std::cout << "Random Goal\n";
				GoalAssignor::randomAssignment(lot);
				experimentDataFilename = experimentNumAgent + "2020_05_20_random_goal_agent";
				// v.display();
			}

			if (m == 1) {
				// ==========================================================================================================
				std::cout << "Greedy Euclidean Goal\n";
				GoalAssignor::greedyEuclideanAssignment(lot);
				experimentDataFilename = experimentNumAgent + "2020_05_20_greedy_euclidean_goal_agent";
				// v.display();
			}

			if (m == 2) {
				// ==========================================================================================================
				std::cout << "Prioritized Euclidean Goal\n";
				GoalAssignor::priortizedEuclideanAssignment(lot);
				experimentDataFilename = experimentNumAgent + "2020_05_20_prioritized_euclidean_goal_agent";
				// v.display(); 
			}
			// the file that used to record data.....
			experimentDataFilename += std::to_string(num_agent);
			experimentDataFilename += "_instance";
			experimentDataFilename += std::to_string(i + 1);
			experimentDataFilename += ".txt";


			// Solve the problem.....
			SimulatorTimer timer;
			if (Planner::firstPath(lot, timer)) {
				if (Planner::refinePath(lot, timer)) {
					savePathSolutionToTextFile(experimentDataFilename, lot);
				}
				else {
					saveFailCase(experimentDataFilename);
				}
			}
			else {
				saveFailCase(experimentDataFilename);
			}


			// 
		}
	}

	benchmarkFile.close();
}



void BenchmarkSolver::solve(std::string bmFilename, std::string dataDirectroy) {
	// open the benchmark file as stream 
	std::ifstream benchmarkFile(bmFilename);
	// how many agents for this problem? 
	std::string experimentNumAgent(dataDirectroy);

	/********************************************************************************************************************************************
	*********************************************************************************************************************************************/
	std::string line;
	if (!benchmarkFile.is_open()) {
		std::cout << "\033[31m ERROR - the benchmark problem can not be opened.... \033[0m \n";
		exit(1);
	}
	else {
		std::cout << "finished loading the benchmark problem. \n";
	}

	std::getline(benchmarkFile, line);
	// the parking lot being planned (once we know how many cars in the parking lot, then a virtual parking lot can be constructed)... 
	ParkingLot lot("./benchmark_problem/" + line);
	// the number of agents 
	std::getline(benchmarkFile, line);
	int num_agent = std::atoi(line.c_str());
	// instantiate some new vehicles to the parking lot to enable us to simulate.... 
	for (int i = 0; i < num_agent; i++) {
		lot.agents.emplace_back(Agent(lot));
		lot.agents[i].generateRandomColor();
		lot.agents[i].generateRandomInitialPoseAt();
	}
	// the number of instances 
	std::getline(benchmarkFile, line);
	int num_instance = std::atoi(line.c_str());

	// the visulization 
	// Visualization::Visualizator v(lot);
	Planner::initilizePlanner(lot);

	// read each instance (under each instance, we have a number of agents) 
	for (int i = 0; i < num_instance; i++) {
		/* Preparing for the problem solving...... */
		std::cout << "----------------------\nInstance: " << (i + 1) << "\n";
		// load the benchmark problem (random initial position from the benchmark file). 
		for (int j = 0; j < num_agent; j++) {
			// get the line and convert it to a string stream to use stream operations. 
			std::getline(benchmarkFile, line);
			std::istringstream iss(line);
			// initial position of the benchmark. 
			double x, y, theta;
			iss >> x >> y >> theta;
			// update the instance of problem configuration. 
			lot.agents[j].intialPose.setPose(x, y, theta);
			lot.agents[j].currPose.setPose(x, y, theta);
		}

		// ==========================================================================================================================
		// Goal Assignment 
		for (int m = 0; m < 3; m++) {
			std::string experimentDataFilename;

			if (m == 0) {
				// ==========================================================================================================
				std::cout << "Random Goal\n";
				GoalAssignor::randomAssignment(lot);
				experimentDataFilename = experimentNumAgent + "2020_05_20_random_goal_agent";
				// v.display();
			}

			if (m == 1) {
				// ==========================================================================================================
				std::cout << "Greedy Euclidean Goal\n";
				GoalAssignor::greedyEuclideanAssignment(lot);
				experimentDataFilename = experimentNumAgent + "2020_05_20_greedy_euclidean_goal_agent";
				// v.display();
			}

			if (m == 2) {
				// ==========================================================================================================
				std::cout << "Prioritized Euclidean Goal\n";
				GoalAssignor::priortizedEuclideanAssignment(lot);
				experimentDataFilename = experimentNumAgent + "2020_05_20_prioritized_euclidean_goal_agent";
				// v.display(); 
			}
			// the file that used to record data.....
			experimentDataFilename += std::to_string(num_agent);
			experimentDataFilename += "_instance";
			experimentDataFilename += std::to_string(i + 1);
			experimentDataFilename += ".txt";


			// Solve the problem.....
			SimulatorTimer timer;
			if (Planner::firstPath(lot, timer)) {
				if (Planner::refinePath(lot, timer)) {
					savePathSolutionToTextFile(experimentDataFilename, lot);
				}
				else {
					saveFailCase(experimentDataFilename);
				}
			}
			else {
				saveFailCase(experimentDataFilename);
			}


			// 
		}
	}

	benchmarkFile.close();
}