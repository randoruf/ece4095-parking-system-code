#include "test7.h"
#include "ParkingLot.h"
#include "Simulator.h"
#include "Visualization.h"
#include "DataRecorder.h"
#include <random>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>


namespace test7 {
	// 
	void initialPlan(ParkingLot& lot, SimulatorTimer& timer) {
		for (unsigned i = 0; i < lot.agents.size(); i++) {
			lot.agents[i].initializePlanner(); 
		}


		timer.setTimeLimit(100);
		timer.resetTimer();

		// check for collision.... 
		while (!timer.timeLimitExceeded() && !lot.allAgentsFoundPath()) {
			// 
			for (auto car = lot.agents.begin(); car != lot.agents.end(); car++) {
				// tell the planner to forget the previous query... 
				car->agentSimpleSetupPtr->getPlanner()->clearQuery();

				if (!car->foundPath()) {
					// start state
					ompl::base::ScopedState<ompl::base::SE2StateSpace> start(car->agentSimpleSetupPtr->getSpaceInformation());
					start->setX(car->intialPose.getPoseX());
					start->setY(car->intialPose.getPoseY());
					start->setYaw(car->intialPose.getPoseTheta());

					// goal state 
					ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(car->agentSimpleSetupPtr->getSpaceInformation());
					goal->setX(car->goalPose.getPoseX());
					goal->setY(car->goalPose.getPoseY());
					goal->setYaw(car->goalPose.getPoseTheta());

					// set the start and goal state  
					car->agentSimpleSetupPtr->setStartAndGoalStates(start, goal, 0.4);
					const ompl::base::PlannerTerminationCondition ptc = ompl::base::IterationTerminationCondition(500);
					ompl::base::PlannerStatus solved = car->agentSimpleSetupPtr->solve(ptc);

					if (solved == ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION)
					{
						// try to simplify the solution...
						car->agentSimpleSetupPtr->simplifySolution();
						// copy constructor to store planner's solution 
						car->plannerPathPtr = &(car->agentSimpleSetupPtr->getSolutionPath());
						car->plannerPathPtr->interpolate(static_cast<int>(car->plannerPathPtr->length() * 2));
						car->plannerPathPointIdx = 0;
						car->setFoundPath();	// set the foundPath_ value to true....
						std::cout << "found a solution" << std::endl;
					}
					else {
						std::cout << "No solution found" << std::endl;
					}
				}
			}
			std::cout << "-----------------------------------------------\n";
		}

		// OMPL failed to find out initial solution. So we can't do anything.....
		if (!lot.allAgentsFoundPath()) {
			std::cout << "\033[31m ERROR(Time Out) - Failed to plan for multi-agents.... (potentially deadlock) \033[0m \n";
			throw std::runtime_error("OMPL failed to find an initial solution...");
		}
	}

	// refine the path of each agent
	void refinePath(ParkingLot& lot, SimulatorTimer& timer) {
		// save the intial plan 
		std::string test_data_filename("./experimental_data/hello_world_01_waypoints.txt");
		// print the collision time-step  
		std::ofstream textCollisionTextFile;
		textCollisionTextFile.open("./experimental_data/hello_world_01_collided_timestep.txt");

		// the replanning procedure has a limit on the run-time... 
		timer.setTimeLimit(600);
		timer.resetTimer();

		// minimum heap used for prioritized planning....
		std::vector<Agent*> pendingCarQueue;

		// refine the collision path 
		unsigned int t = 0;

		while (!timer.timeLimitExceeded()) {
			// ============================================================================================================================================
			unsigned int numCarReachedGoal = 0;
			// check the way point of each agent at the moment t, and mark the agents that have collisions...
			for (unsigned int i = 0; i < lot.agents.size(); i++) {
				// only replan for those are still driving on the road... (we can't replan the cars have finished parking....)
				if (!lot.agents[i].reachedGoal()) {
					// if the current pose of agent[i] is not collision free, add it to the replanning list....
					int numCollisions = lot.collisionCars(lot.agents[i]);
					if (numCollisions > 0) {
						std::cout << i << std::endl;
						pendingCarQueue.push_back(&lot.agents[i]);
					}
				}
				// if agent[i] has reached its goal, we increase the counter that count how many cars have reached its goal at this moment.... 
				else {
					numCarReachedGoal++;
				}
			}
			// all cars have reached their goals, we don't need to check the path solution.....
			if (numCarReachedGoal == lot.agents.size()) {
				break;
			}


			// -============================================================================================================================================
			// some cars have collisions because the pendingAgent list is not empty..
			if (!pendingCarQueue.empty()) {
				// shufft the wating queue.... 
				std::random_shuffle(pendingCarQueue.begin(), pendingCarQueue.end());
				pendingCarQueue.back()->findConflictNeighbours(); 

				// save the moment when replanning may appear.... 
				// textCollisionTextFile << t << "\n";
				std::cout << "Collisions at t= " << t << std::endl;
				// as the motion checking is for lot.agents[i].plannerPathPointIdx + 1, 
				// it means we can go back to the previous time step 
				for (unsigned int i = 0; i < lot.agents.size(); i++) {
					if (!lot.agents[i].reachedGoal()) {
						if (lot.agents[i].waitFor == 0) {
							if (lot.agents[i].plannerPathPointIdx > 0) {
								lot.agents[i].plannerPathPointIdx--;
							}
							unsigned int& prev_t = lot.agents[i].plannerPathPointIdx;
							lot.agents[i].currPose.setPose(
								lot.agents[i].plannerPathPtr->getState(prev_t)->as<ompl::base::SE2StateSpace::StateType>()->getX(),
								lot.agents[i].plannerPathPtr->getState(prev_t)->as<ompl::base::SE2StateSpace::StateType>()->getY(),
								lot.agents[i].plannerPathPtr->getState(prev_t)->as<ompl::base::SE2StateSpace::StateType>()->getYaw()
							);
						}
					}
				}
			}

			// for (unsigned int i = 0; i < lot.agents.size(); i++) {
			//	std::cout << lot.collisionCars(lot.agents[i]) << std::endl;
			// replan until the pending queue is empty...
			if (!pendingCarQueue.empty()) {
				// get the agent 
				auto car = pendingCarQueue.back();
				car->agentSimpleSetupPtr->getPlanner()->clear(); // forget the previous path.... 
				car->agentSimpleSetupPtr->getSpaceInformation()->setStateValidityChecker(car->movingObstacleCollisionChecker); 

				// will not wait....
				car->waitFor = 0;
				car->resetFoundPath();
				// start state
				ompl::base::ScopedState<ompl::base::SE2StateSpace> start(car->agentSimpleSetupPtr->getSpaceInformation());
				start->setX(car->currPose.getPoseX());
				start->setY(car->currPose.getPoseY());
				start->setYaw(car->currPose.getPoseTheta());

				// goal state 
				ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(car->agentSimpleSetupPtr->getSpaceInformation());
				goal->setX(car->goalPose.getPoseX());
				goal->setY(car->goalPose.getPoseY());
				goal->setYaw(car->goalPose.getPoseTheta());

				// set the start and goal state  
				car->agentSimpleSetupPtr->setStartAndGoalStates(start, goal, 0.4);

				std::cout << "start to replan......\n";
				SimulatorTimer replanTimer;
				replanTimer.setTimeLimit(120);
				replanTimer.resetTimer();
				const ompl::base::PlannerTerminationCondition ptc = ompl::base::IterationTerminationCondition(1500);
				while (!replanTimer.timeLimitExceeded()) {
					ompl::base::PlannerStatus solved = car->agentSimpleSetupPtr->solve(ptc);

					if (solved == ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION) {
						// try to simplify the solution...
						car->agentSimpleSetupPtr->simplifySolution();
						// copy constructor to store planner's solution 
						car->plannerPathPtr = &(car->agentSimpleSetupPtr->getSolutionPath());
						// car->plannerPathPtr->interpolate();
						car->plannerPathPtr->interpolate(static_cast<int>(car->plannerPathPtr->length() * 2));
						car->plannerPathPointIdx = 0;
						car->setFoundPath();	// set the foundPath_ value to true....
						std::cout << "found a solution" << std::endl;
						break;
					}
					else if (solved == ompl::base::PlannerStatus::StatusType::INVALID_START) {
						savePathSolutionToTextFile(test_data_filename, lot);
						throw std::runtime_error("I don't know what is happening..... Some stopped cars try to move....");
					}
					else {
						std::cout << "No solution found" << std::endl;
					}
				}
				// once the replanning finish, use the occupancyGridmap collision checker immediattely.... 
				car->agentSimpleSetupPtr->getSpaceInformation()->setStateValidityChecker(car->occupancyGridMapCollisionChecker);

				// if the this replanning is too long, we think it should fail... 
				if (replanTimer.timeLimitExceeded()) {
					std::cout << "\033[31m ERROR(Time Out) - Failed to plan for multi-agents.... (potentially deadlock) \033[0m \n";
					throw std::runtime_error("Failed to plan for multi-agents.... (potentially deadlock)");
				}
				pendingCarQueue.pop_back();
			}

			while (!pendingCarQueue.empty()) {
				auto car = pendingCarQueue.back();
				if (car->waitFor == 0) {
					car->waitFor = (rand() % 50) + 50;
				}
				pendingCarQueue.pop_back();
			}

			savePathSolutionToTextFile(test_data_filename, lot);
			// ============================================================================================================================================
			// after the replanning procedure, we gurantee that the current pose should be collision-free..... 
			for (unsigned int i = 0; i < lot.agents.size(); i++) {
				// only prepare motion checking for those car has not reached the goal yet....
				if (!lot.agents[i].reachedGoal()) {
					// store the valid states in the agent final car path solution......, see 'append'....
					unsigned int& curr_t = lot.agents[i].plannerPathPointIdx;
					lot.agents[i].carPathPtr->append(lot.agents[i].plannerPathPtr->getState(curr_t));
					// -------------------------------------------------------------------------------------------------------
					if (lot.agents[i].waitFor > 0) {
						lot.agents[i].waitFor = lot.agents[i].waitFor - 1;
					}
					else {
						// prepare for the motion validation, note that plannerPathPointIdx+1 has not yet in the agent car path....
						if (curr_t < lot.agents[i].plannerPathPtr->getStateCount() - 1) {
							lot.agents[i].plannerPathPointIdx++;
							lot.agents[i].currPose.setPose(
								lot.agents[i].plannerPathPtr->getState(curr_t)->as<ompl::base::SE2StateSpace::StateType>()->getX(),
								lot.agents[i].plannerPathPtr->getState(curr_t)->as<ompl::base::SE2StateSpace::StateType>()->getY(),
								lot.agents[i].plannerPathPtr->getState(curr_t)->as<ompl::base::SE2StateSpace::StateType>()->getYaw()
							);
						}
					}
				}
			}

			// ============================================================================================================================================
			// increase the timer counter.....
			std::cout << "------------------------ checking ------------------------ \n";
			t++;
		}


		// 
		if (timer.timeLimitExceeded()) {
			std::cout << "\033[31m ERROR(Time Out) - Failed to plan for multi-agents.... (potentially deadlock) \033[0m \n";
			throw std::runtime_error("OMPL failed to find an initial solution...");
		}

		// 
		textCollisionTextFile.close();
	}

	//
	void test7() {
		/* 1. load the parking lot enviroment */
		ParkingLot lot("./benchmark_problem/parking_lot01_parking_space.txt");
		Simulator s(lot, 2);

		// start position 
		lot.agents[0].intialPose.setPose(3.73, 10.8, 0);
		lot.agents[1].intialPose.setPose(33.21, 10.8, 3.14159);
		lot.agents[0].currPose.setPose(3.73, 10.8, 0);
		lot.agents[1].currPose.setPose(33.21, 10.8, 3.14159);
		// goal position 
		lot.agents[0].goalPose.setPose(33.21, 10.8, 0);
		lot.agents[1].goalPose.setPose(3.73, 10.8, 3.14159);


		// each agent plans without considering other agents 
		SimulatorTimer timer;
		initialPlan(lot, timer);


		// refine the collision paths (by replanning)...
		refinePath(lot, timer);
		s.startVisualization();
		
	}
}