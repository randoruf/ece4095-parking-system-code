#include "Planner.h"


void Planner::initilizePlanner(ParkingLot& lot) {
	// before the actual planning, we should give each agent an individual planner....
	for (unsigned i = 0; i < lot.agents.size(); i++) {
		if (!lot.agents[i].agentSimpleSetupPtr) {
			lot.agents[i].initializePlanner();
		}
	}
}


bool Planner::firstPath(ParkingLot& lot, SimulatorTimer& timer) {
	// 
	for (auto& a : lot.agents) {
		// tell the planner to forget the previous query... 
		a.agentSimpleSetupPtr->getPlanner()->clearQuery();
		// once the replanning finish, use the occupancyGridmap collision checker immediattely.... 
		a.agentSimpleSetupPtr->getSpaceInformation()->setStateValidityChecker(a.occupancyGridMapCollisionChecker);
		a.plannerPathPointIdx = 0;
		// tell the planner to forget the previous path solution....
		a.carPathPointIdx = 0;
		a.carPathPtr->clear();
		ompl::base::State *startState = a.agentSimpleSetupPtr->getStateSpace()->allocState();
		startState->as<ompl::base::SE2StateSpace::StateType>()->setXY(a.intialPose.getPoseX(), a.intialPose.getPoseY());
		startState->as<ompl::base::SE2StateSpace::StateType>()->setYaw(a.intialPose.getPoseTheta());
		a.carPathPtr->append(startState);		// copy into the carPath 
		a.agentSimpleSetupPtr->getStateSpace()->freeState(startState);
		// 
		a.waitFor = 0;
		a.resetFoundPath(); 
		a.resetReachedGoal(); 
		// the current pose should be the initial pose 
		a.currPose = a.intialPose; 
	}

	// set the maximum time allowed to run (to avoid infinte loop, here I assume 3 minus, namely 180 second...)
	timer.setTimeLimit(180); 
	timer.resetTimer();

	// find an initial
	while (!timer.timeLimitExceeded() && !lot.allAgentsFoundPath()) {
		// 
		for (auto car = lot.agents.begin(); car != lot.agents.end(); car++) {
			if (!car->foundPath()) {
				// 
				// car->agentSimpleSetupPtr->getPlanner()->clearQuery();
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
				car->agentSimpleSetupPtr->setStartAndGoalStates(start, goal, 1.8);

				// termination condition for RRT* 
				// (https://ompl.kavrakilab.org/classompl_1_1base_1_1IterationTerminationCondition.html#aaefe7d1f60c659d66b73c547c59edb1bl)
				// const ompl::base::PlannerTerminationCondition ptcMaxSample = ompl::base::IterationTerminationCondition(500);

				// 
				ompl::base::PlannerStatus solved = car->agentSimpleSetupPtr->solve(8);
				if (solved == ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION){
					// try to simplify the solution...
					car->agentSimpleSetupPtr->simplifySolution();
					// copy constructor to store planner's solution 
					car->plannerPathPtr = &(car->agentSimpleSetupPtr->getSolutionPath());
					car->plannerPathPtr->interpolate(static_cast<int>(car->plannerPathPtr->length() * 2));
					car->plannerPathPointIdx = 0;
					car->setFoundPath();	// set the foundPath_ value to true....
					std::cout << "found a solution" << std::endl;
				}
				else if (solved == ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION || solved == ompl::base::PlannerStatus::StatusType::TIMEOUT) {
					std::cout << "Phase 1 : No solution found" << std::endl;
				}
				else{
					std::cout << "\033[31m ERROR(invalid planning configuration) OMPL encounters runtime error \033[0m \n";
					throw std::runtime_error("OMPL runtime error. Check your code.....");
				}
			}
		}
		std::cout << "--------------------------------------------------------------------------------\n";
	}

	// OMPL failed to find out initial solution. So we can't do anything.....
	if (!lot.allAgentsFoundPath()) {
		std::cout << "\033[31m ERROR(Time Out) - Failed to plan for multi-agents.... (potentially deadlock) \033[0m \n";
		return false;		// OMPL can't find initial solution in a given duration, so return false. 
	}

	return true; 
}


bool Planner::refinePath(ParkingLot& lot, SimulatorTimer& timer) {
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
			// get the agent to replan.......
			auto car = pendingCarQueue.back();
			car->agentSimpleSetupPtr->getPlanner()->clearQuery();

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
			car->agentSimpleSetupPtr->setStartAndGoalStates(start, goal, 1.8);
			// 
			std::cout << "start to replan......\n"; 
			SimulatorTimer replanTimer;
			replanTimer.setTimeLimit(150);
			replanTimer.resetTimer();
			// const ompl::base::PlannerTerminationCondition ptc = ompl::base::IterationTerminationCondition(1000);
			while (!replanTimer.timeLimitExceeded()) {
				// car->agentSimpleSetupPtr->getPlanner()->clearQuery();
				ompl::base::PlannerStatus solved = car->agentSimpleSetupPtr->solve(20);
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
					throw std::runtime_error("I don't know what is happening..... Some stopped cars try to move....");
				}
				else {
					std::cout << "Phase 2 : No solution found" << std::endl;
				}
			}
			// if the this replanning is too long, we think it should fail... 
			if (replanTimer.timeLimitExceeded()) {
				std::cout << "\033[31m ERROR(Time Out) - Failed to plan for multi-agents.... (potentially deadlock) \033[0m \n";
				return false;
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

	// if time is not enough, label this case as fail.... 
	if (timer.timeLimitExceeded()) {
		std::cout << "\033[31m ERROR(Time Out) - Failed to plan for multi-agents.... (potentially deadlock) \033[0m \n";
		return false;
	}
	// the path is ok!!
	return true; 
}