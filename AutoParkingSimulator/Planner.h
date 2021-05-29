#pragma once
#include <random>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>
#include "ParkingLot.h"
#include "Simulator.h"
#include "Visualization.h"
#include "DataRecorder.h"

namespace Planner
{
	// 
	void initilizePlanner(ParkingLot& lot); 

	// 
	bool firstPath(ParkingLot& lot, SimulatorTimer& timer);
	
	// 
	bool refinePath(ParkingLot& lot, SimulatorTimer& timer);

};

