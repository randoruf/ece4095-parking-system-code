#pragma once
#include <string>
#include <vector>
#include <boost/math/constants/constants.hpp>
#include "OccupancyMap.h"
#include "Vehicle.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include "PNG.h"
#include <ompl/util/RandomNumbers.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>


enum class PlannerSpace {
	ReedsShepp, 
	SE2StateSpace  // SE(2) (rotation and translation in the plane, ompl::base::SE2StateSpace),
};


class ParkingLot;


class occupancyGridMapStateValidityChecker; 
class movingCarStateValidityChecker;

class Agent
{
public:
	// OMPL simple setup 
	std::shared_ptr<ompl::geometric::SimpleSetup> agentSimpleSetupPtr = nullptr;
	// OMPL collision checking 
	std::shared_ptr<occupancyGridMapStateValidityChecker> occupancyGridMapCollisionChecker; 
	std::shared_ptr<movingCarStateValidityChecker> movingObstacleCollisionChecker;

	// the pose information 
	Pose intialPose;
	Pose currPose;
	Pose goalPose;
	int parkingSlotIdx = -1;
	
	// the valid path solution 
	std::shared_ptr<ompl::geometric::PathGeometric> carPathPtr = nullptr;
	size_t carPathPointIdx = 0; 

	// the planner path 
	ompl::geometric::PathGeometric* plannerPathPtr = NULL;
	size_t plannerPathPointIdx = 0; 

	// confict resolution.... 
	int waitFor = 0; // wait for timesteps....
	std::vector<Agent*> neighboursToCheck;

	// used to vizulize the goal 
	unsigned color[3];

	Agent(ParkingLot &lot);

	void initializePlanner();

	void generateRandomInitialPoseAt();
	void generateRandomColor();

	// used by refine planner and vizulization....
	void updateCurrPose();

	// to indicate wether the planner has found a path for the agent ...
	bool foundPath() const; 
	void setFoundPath(); 
	void resetFoundPath(); 

	// to indicate that whether the agent finished the parking 
	bool reachedGoal();
	void setReachedGoal();
	void resetReachedGoal(); 

	// 
	void findConflictNeighbours();

private:
	// to indicate whether we reached the goal...
	bool reachedGoal_ = false;
	// to indicate wether the planner has found a goal...
	bool foundPath_ = false;
	// intialize a unique random seed.
	ompl::RNG rng_;
	ParkingLot& lot_;
};

class ParkingLot
{
public:
	OccupancyMap map; 
	std::vector<Pose> parkingSpaces; 
	VehicleDimensionInfo vehcicleInfo;

	ParkingLot(const std::string mapParamFile);

	void addParkingSpace(Pose parkingPose);

	bool collisionFree(const Pose& p1, const Pose& p2) const; 
	bool collisionFree(const Pose& p1, const OccupancyMap& OccMap) const; 
	bool collisionFree(const Pose& p1, const std::vector<Agent> agents) const;
	
	bool collisionFreeStateSkipAgent(const Pose& p1, const Agent& a) const;
	bool collisionFreeStateAgentsK(Pose& p, std::vector<Agent*> agentsK);

	// 
	int collisionCars(Agent& a);
	std::vector<Agent*> adjacentConfictCars(Agent& a); 


	// 
	bool parkingSlotOverlapped(const Pose& p1) const;
	void occupancyMapWriteToPNG();

	// the agents in the parking lot....
	void updateScene();

	// to tell whether all cars in the parking lot has found a path  
	bool allAgentsFoundPath();

	// the agents in the parking lot. 
	std::vector<Agent> agents;
	
private:
	std::string mapParamFile_; 
	std::string mapImageFile_;

	// how many cars in the parking lot 
	int maxNumberCars; 
};


//--------------------------------------------------------------------------------------------------------
class occupancyGridMapStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
	occupancyGridMapStateValidityChecker(ompl::base::SpaceInformationPtr& si, ParkingLot& lot) : ompl::base::StateValidityChecker(si), lot_(lot) {}

	bool isValid(const ompl::base::State *state) const override
	{
		const ompl::base::SE2StateSpace::StateType *se2state = state->as<ompl::base::SE2StateSpace::StateType>();
		double x = se2state->getX(), y = se2state->getY();
		double theta = se2state->getYaw();

		return si_->satisfiesBounds(state) && lot_.collisionFree(Pose(x, y, theta), lot_.map);
	}

private:
	ParkingLot& lot_;
};

//--------------------------------------------------------------------------------------------------------
class movingCarStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
	movingCarStateValidityChecker(
		ompl::base::SpaceInformationPtr& si, ParkingLot& lot, Agent& agent) : ompl::base::StateValidityChecker(si), lot_(lot), agent_(agent) {}

	bool isValid(const ompl::base::State *state) const override{
		const ompl::base::SE2StateSpace::StateType *se2state = state->as<ompl::base::SE2StateSpace::StateType>();
		double x = se2state->getX(), y = se2state->getY();
		double theta = se2state->getYaw();

		// check whether (x, y, theta) is a valid state.... 
		Pose tmp_pose = Pose(x, y, theta); 
		return si_->satisfiesBounds(state) && 
			   lot_.collisionFree(tmp_pose, lot_.map) &&
			   lot_.collisionFreeStateAgentsK(tmp_pose, agent_.neighboursToCheck);
	}

private:
	ParkingLot& lot_;
	Agent& agent_; 
};