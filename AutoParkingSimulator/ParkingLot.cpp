#include "ParkingLot.h"

// 
void Agent::generateRandomInitialPoseAt()
{
	const double PI = boost::math::constants::pi<double>();
	intialPose.setPose(
		rng_.uniformReal(0, lot_.map.getWorldWidth()),
		rng_.uniformReal(0, lot_.map.getWorldHeight()),
		rng_.uniformReal(-PI, PI), 0);

	// produce a random starting position untile it is collision-free and not collide with other agents...
	while ((!lot_.collisionFree(intialPose, lot_.map)) || (!lot_.collisionFree(intialPose, lot_.agents))) {
		intialPose.setPose(
			rng_.uniformReal(0, lot_.map.getWorldWidth()),
			rng_.uniformReal(0, lot_.map.getWorldHeight()),
			rng_.uniformReal(-PI, PI), 0
		);
	}

	// place the current agent to the initial position ... 
	currPose = intialPose;
}

void Agent::generateRandomColor()
{
	// equally dive the color space 
	color[0] = rng_.uniformInt(0, 250);
	color[1] = rng_.uniformInt(0, 250);
	color[2] = rng_.uniformInt(0, 250);
}


Agent::Agent(ParkingLot &lot) : lot_(lot)
{

}

void Agent::initializePlanner() {
	// Reeds Sheep Space 
	//	 - assume the vehicle has 30 degree steering limit, 
	//	 - http://planning.cs.uiuc.edu/node658.html
	//	 - the steering radius is L/tan(theta) = 2.6 / tan(pi/6)
	ompl::base::StateSpacePtr space(std::make_shared<ompl::base::ReedsSheppStateSpace>(4.50));
	//std::cout << space->as<ompl::base::SE2StateSpace>()->getSubspaceWeight(1) << "\n";
	space->as<ompl::base::SE2StateSpace>()->setSubspaceWeight(1, 2.6);
	//std::cout << space->as<ompl::base::SE2StateSpace>()->getSubspaceWeight(1) << "\n";

	// the bounding of state space
	ompl::base::RealVectorBounds bounds(2);
	bounds.setLow(0); // the first quarature in the cartisean coordinate....
	// the x, y higher bound  
	bounds.high[0] = lot_.map.getWorldWidth();
	bounds.high[1] = lot_.map.getWorldHeight();
	// the bounding of the state space...
	space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

	// a simple setup to 
	agentSimpleSetupPtr = std::make_shared<ompl::geometric::SimpleSetup>(space);
	/* 4. set state validity checking for this space */
	ompl::base::SpaceInformationPtr si = agentSimpleSetupPtr->getSpaceInformation();
	// instantiate instances of the collision checker
	this->occupancyGridMapCollisionChecker = std::make_shared<occupancyGridMapStateValidityChecker>(si, lot_); 
	this->movingObstacleCollisionChecker = std::make_shared<movingCarStateValidityChecker>(si, lot_, *this); 
	
	// use the occupancy grid map collison checker by default...  
	si->setStateValidityChecker(this->occupancyGridMapCollisionChecker);
	// set resolution to 1% (the body of vehicle is big, so low resolution is enough....) 
	// agentSimpleSetupPtr->getSpaceInformation()->setStateValidityCheckingResolution(0.02);


	// attempt to solve the problem within 30 seconds of planning time
	std::shared_ptr<ompl::geometric::RRTstar> planner(std::make_shared<ompl::geometric::RRTstar>(agentSimpleSetupPtr->getSpaceInformation()));
	planner->setRange(8);
	planner->setGoalBias(0.1);  // chance that will choose goal 
	planner->setFocusSearch(false);
	planner->setKNearest(false);
	agentSimpleSetupPtr->setPlanner(planner);

	// real path solution (in the space)
	carPathPtr = std::make_shared<ompl::geometric::PathGeometric>(si);
	carPathPointIdx = 0;

	ompl::base::State *startState = space->allocState();
	startState->as<ompl::base::SE2StateSpace::StateType>()->setXY(intialPose.getPoseX(), intialPose.getPoseY());
	startState->as<ompl::base::SE2StateSpace::StateType>()->setYaw(intialPose.getPoseTheta());
	carPathPtr->append(startState);
	space->freeState(startState);
}


void Agent::updateCurrPose() {
	// update the current pose based on the path provided by OMPL  
	if (carPathPtr) {
		// increase the current pose pointer  
		if (carPathPointIdx < carPathPtr->getStateCount() - 1) {
			carPathPointIdx++;
		}
		// update the curent pose
		currPose.setPose(
			carPathPtr->getState(carPathPointIdx)->as<ompl::base::SE2StateSpace::StateType>()->getX(),
			carPathPtr->getState(carPathPointIdx)->as<ompl::base::SE2StateSpace::StateType>()->getY(),
			carPathPtr->getState(carPathPointIdx)->as<ompl::base::SE2StateSpace::StateType>()->getYaw()
		);
	}
}

bool Agent::foundPath() const{
	return foundPath_; 
}

void Agent::setFoundPath() {
	foundPath_ = true; 
}

void Agent::resetFoundPath() {
	foundPath_ = false; 
}


bool Agent::reachedGoal() {
	if (reachedGoal_) {
		return true;
	}


	if (!reachedGoal_ && carPathPtr) {
		// check if the current pose is the goal pose 
		if (plannerPathPointIdx == plannerPathPtr->getStateCount() - 1)
		{
			reachedGoal_ = true;
			return true;
		}
		else {
			return false;
		}
	}
	
	return false;
}

void Agent::setReachedGoal() {
	reachedGoal_ = true;
}

void Agent::resetReachedGoal() {
	reachedGoal_ = false;
}


void Agent::findConflictNeighbours() {
	this->neighboursToCheck.clear(); 
	this->neighboursToCheck = lot_.adjacentConfictCars(*this); 
	// std::cout << this->neighboursToCheck.size() << '\n'; 
}


void ParkingLot::addParkingSpace(Pose parkingPose) {
	if (collisionFree(parkingPose, map) && !parkingSlotOverlapped(parkingPose))
	{
		parkingSpaces.push_back(parkingPose);
		// std::cout << "Added a new parking space" << std::endl;
	}
	else {
		std::cerr << "Failded to add parking space at (x=" << parkingPose.getPoseX() << ", y=" << parkingPose.getPoseY() <<
			", theta=" << parkingPose.getPoseTheta() << ")" << std::endl; 
	}

}


ParkingLot::ParkingLot(const std::string mapParamFile) 
{
	// read the parameters of the map.
	mapParamFile_ = mapParamFile;
	// std::cout << mapParamFile_ << std::endl;

	std::ifstream paramFileHandle(mapParamFile);

	if (paramFileHandle) {
		std::string line;

		// the parking lot PNG file 
		std::getline(paramFileHandle, line);
		mapImageFile_ = line;
		// the resolution of the map 
		std::getline(paramFileHandle, line); 
		double resolution = std::stod(line);
		// the Occupancy Grid map 
		map.InitializeOccupancyMap(mapImageFile_, resolution);
		// std::cout << map.getMapImageName() << std::endl; 
		// std::cout << map.getMapResolution() << std::endl; 

		// how many cars in the parking lot 
		std::getline(paramFileHandle, line);
		maxNumberCars = std::stoi(line); 
		// std::cout << maxNumberCars << std::endl; 

		// the inflation radius 
		std::getline(paramFileHandle, line);
		vehcicleInfo.carinflatedRadius = std::stod(line);
		// std::cout << vehcicleInfo.carinflatedRadius << std::endl;

		// the width of the car 
		std::getline(paramFileHandle, line);
		vehcicleInfo.carWidth = std::stod(line);
		// std::cout << carWidth << std::endl; 

		// the length of the car 
		std::getline(paramFileHandle, line); 
		vehcicleInfo.carLength = std::stod(line);
		// std::cout << carLength << std::endl; 

		// the rear axle to the front end
		std::getline(paramFileHandle, line);
		vehcicleInfo.carRearAxleFrontEndDistance = std::stod(line);
		// std::cout << carRearAxleFrontEndDistance << std::endl; 

		// the rear axle to the back end 
		std::getline(paramFileHandle, line);
		vehcicleInfo.carRearAxleBackEndDistance = std::stod(line);
		// std::cout << carRearAxleBackEndDistance << std::endl; 

		// the wheel base 
		std::getline(paramFileHandle, line); 
		vehcicleInfo.carWheelBase = std::stod(line);
		// std::cout << carWheelBase << std::endl; 

		// the maximum speed 
		std::getline(paramFileHandle, line); 
		vehcicleInfo.carMaxSpeed = std::stod(line);
		// std::cout << carMaxSpeed << std::endl; 

		// the maximum steering angle 
		std::getline(paramFileHandle, line);
		vehcicleInfo.carMaxSteerAngle = std::stod(line);
		// std::cout << carMaxSteerAngle << std::endl; 

		// the maximum steering rate 
		std::getline(paramFileHandle, line); 
		vehcicleInfo.carMaxSteerRate = std::stod(line);
		// std::cout << carMaxSteerRate << std::endl; 

		// add the parking spaces into the parking space
		double x, y, theta; 
		while (paramFileHandle >> x >> y >> theta){
			// std::cout << "---------------------------------" << std::endl;
			// std::cout << "(x = " << x << ", y = " << y << ", theta = " << theta << ")" << std::endl; 
			addParkingSpace(Pose(x, y, theta));
		}

	}
	else
	{
		throw "Faild to open the paramter file of the parking lot...";
	}

	paramFileHandle.close();

	std::cout << "Finished loading the parking lot environment...." << std::endl; 
}

void ParkingLot::occupancyMapWriteToPNG(){
	PNG png;
	png.readFromFile("./parking_lot01_blank.png");


	for (int i = 0; i < map.getWidth(); i++) {
		for (int j = 0; j < map.getHeight(); j++) {
			int index = map.gridIndexToLinearIndex(GridIndex(i, j));
			RGBAPixel& pixel = png.getPixel(i, j);
			pixel.r = 255 - map.pMap[index];
			pixel.g = 255 - map.pMap[index];
			pixel.b = 255 - map.pMap[index];
			pixel.a = 255;
		}
	}






	for (size_t k = 0; k < parkingSpaces.size(); k++){

		// the first circle 
		GridIndex centreGridIdx =  map.convertworld2GridIndex(parkingSpaces[k].getPoseX(), 
															  parkingSpaces[k].getPoseY());

		int maxNumBlocks = static_cast<int>(std::floor(vehcicleInfo.carinflatedRadius / map.getMapResolution()));

		GridIndex idx;

		for (int i = -maxNumBlocks; i < maxNumBlocks; i++) {
			for (int j = -maxNumBlocks; j < maxNumBlocks; j++) {
				if (i * i + j * j > maxNumBlocks * maxNumBlocks) {
					// std::cout << "out of circle" << std::endl; 
					continue;
				}

				idx.SetIndex(centreGridIdx.i + i, centreGridIdx.j + j);
				if (map.isValidGridIndex(idx)) {
					RGBAPixel& pixel = png.getPixel(idx.i, idx.j);
					pixel.r = 0;
					pixel.g = 0;
					pixel.b = 0;
				}
			}
		}


		// the second circle 
		centreGridIdx = map.convertworld2GridIndex(
							   parkingSpaces[k].getPoseX() + vehcicleInfo.carWheelBase * cos(parkingSpaces[k].getPoseTheta()),
							   parkingSpaces[k].getPoseY() + vehcicleInfo.carWheelBase * sin(parkingSpaces[k].getPoseTheta()));
		for (int i = -maxNumBlocks; i < maxNumBlocks; i++) {
			for (int j = -maxNumBlocks; j < maxNumBlocks; j++) {
				if (i * i + j * j > maxNumBlocks * maxNumBlocks) {
					// std::cout << "out of circle" << std::endl; 
					continue;
				}

				idx.SetIndex(centreGridIdx.i + i, centreGridIdx.j + j);
				if (map.isValidGridIndex(idx)) {
					RGBAPixel& pixel = png.getPixel(idx.i, idx.j);
					pixel.r = 0;
					pixel.g = 0;
					pixel.b = 0;
				}
			}
		}
	}

	png.writeToFile("./parking_lot01_occupancy_map_playground(do not run this).png");
}


/**
 * The collision checking function used by planner.
 * The aim is to check whether a pose collides with the static occupancy grid map. 
 *
 * @param p1, the current pose of vehicle 1 (if p1 == pNew, it can be used a collision 
			  check in RRT planning algorithm).
 * @param occMap, the static occupancy grid map (can be updated by Extended Kalman Filter?). 
 * @return
 */
bool ParkingLot::collisionFree(const Pose& p1, const OccupancyMap& occMap) const {
	Pose discSet1[2];
	discSet1[0] = Pose(p1.getPoseX(), p1.getPoseY(), p1.getPoseTheta());
	double frontX = p1.getPoseX() + vehcicleInfo.carWheelBase * cos(p1.getPoseTheta());
	double frontY = p1.getPoseY() + vehcicleInfo.carWheelBase * sin(p1.getPoseTheta());
	discSet1[1] = Pose(frontX, frontY, p1.getPoseTheta());

	int maxNumBlocks = static_cast<int>(std::floor(vehcicleInfo.carinflatedRadius / map.getMapResolution()));
	GridIndex idx;

	for (int k = 0; k < 2; k++) {
		GridIndex centreGridIdx = map.convertworld2GridIndex(
			discSet1[k].getPoseX(),
			discSet1[k].getPoseY());

		for (int i = -maxNumBlocks; i < maxNumBlocks; i++) {
			for (int j = -maxNumBlocks; j < maxNumBlocks; j++) {
				if (i * i + j * j > maxNumBlocks * maxNumBlocks) {
					// std::cout << "out of circle" << std::endl; 
					continue;
				}

				idx.SetIndex(centreGridIdx.i + i, centreGridIdx.j + j);

				if (map.isValidGridIndex(idx)) {
					// the threshold is 0.75 * 255
					if (map.pMap[map.gridIndexToLinearIndex(idx)] > 192) {
						return false;
					}
				}
			}
		}
	}


	return true;
}


/**
 * The collision checking function used by planner
   The aim is to check whether two poses will overlap.
 * It assumes the shape of each vehicle can be approximately by
 * two discs. Hence, the collision is very easy (by circle collision checking)
 *	// https://developer.mozilla.org/zh-CN/docs/Games/Techniques/2D_collision_detection
 *
 * @param p1, the current pose of vehicle 1 (if p1 == pNew, it can be used as collision
			  check in RRT planning algorithm).
 * @param p2, the current pose of vehicle 2.
 * @return
 */
bool ParkingLot::collisionFree(const Pose& p1, const Pose& p2) const {
	Pose discSetA[2];
	discSetA[0] = Pose(p1.getPoseX(), p1.getPoseY(), p1.getPoseTheta());
	double frontXA = p1.getPoseX() + vehcicleInfo.carWheelBase * cos(p1.getPoseTheta());
	double frontYA = p1.getPoseY() + vehcicleInfo.carWheelBase * sin(p1.getPoseTheta());
	discSetA[1] = Pose(frontXA, frontYA, p1.getPoseTheta());


	Pose discSetB[2];
	discSetB[0] = Pose(p2.getPoseX(), p2.getPoseY(), p2.getPoseTheta());
	double frontXB = p2.getPoseX() + vehcicleInfo.carWheelBase * cos(p2.getPoseTheta());
	double frontYB = p2.getPoseY() + vehcicleInfo.carWheelBase * sin(p2.getPoseTheta());
	discSetB[1] = Pose(frontXB, frontYB, p2.getPoseTheta());

	double inflatedR2 = vehcicleInfo.carinflatedRadius * 2; 
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			double dx = discSetA[i].getPoseX() - discSetB[j].getPoseX();
			double dy = discSetA[i].getPoseY() - discSetB[j].getPoseY();
			if ((dx * dx + dy * dy) < (inflatedR2 * inflatedR2)) {
				return false;
			}
		}
	}

	return true;
}

bool ParkingLot::collisionFree(const Pose& p1, const std::vector<Agent> agents) const {
	for (const auto& a : agents) {
		if (!collisionFree(p1, a.currPose)) {
			return false;
		}
	}
	return true;
}


int ParkingLot::collisionCars(Agent& a) {
	int numCollidedCars = 0; 

	for (auto& b : agents) {
		if (&a == &b) {
			continue;
		}
		if (!collisionFree(a.currPose, b.currPose)) {
			numCollidedCars++;
		}
	}
	return numCollidedCars;
}


std::vector<Agent*> ParkingLot::adjacentConfictCars(Agent& a) {
	std::vector<Agent*> collidedCars_; 

	for (auto& b : agents) {
		if (&a == &b) {
			continue;
		}
		if (!collisionFree(a.currPose, b.currPose)) {
			collidedCars_.push_back(&b); 
		}
	}

	return collidedCars_; 
}


bool ParkingLot::collisionFreeStateAgentsK(Pose& p, std::vector<Agent*> agentsK) {
	// std::cout << p.getPoseX() << ' ' << p.getPoseY() << ' ' << p.getPoseTheta() << '\n'; 
	for (Agent* k : agentsK) {
		//std::cout << k->currPose.getPoseX() << ' ' << k->currPose.getPoseY() << ' ' << k->currPose.getPoseTheta() << '\n'; 
		if (!collisionFree(p, k->currPose)) {
			return false;
		}
	}
	return true;
}



/*****
 * Check whether pose p has collisions with agents in the parking lot.
 * However, this function can ignore the agent who is currently invoking this function.
 * as it see other agents as moving obstacles (instead of itself)......
 *****/
bool ParkingLot::collisionFreeStateSkipAgent(const Pose& p, const Agent& a) const {
	for (auto& b : agents) {
		if (&a == &b) {
			continue;
		}
		if (!collisionFree(p, b.currPose)) {
			return false;
		}
	}
	return true;
}



bool ParkingLot::parkingSlotOverlapped(const Pose& p1) const {
	for (auto c : parkingSpaces) {
		Pose p2 = Pose(c.getPoseX(), c.getPoseY(), c.getPoseTheta()); 
		if (!collisionFree(p1, p2)) {
			return true;
		}
	}
	return false;
}

void ParkingLot::updateScene() {
	for (size_t i = 0; i < agents.size(); i++) {
		agents[i].updateCurrPose();
	}
}


bool ParkingLot::allAgentsFoundPath() {
	bool found = true;
	for (size_t i = 0; i < agents.size(); i++) {
		found &= agents[i].foundPath();
	}
	return found;
}
