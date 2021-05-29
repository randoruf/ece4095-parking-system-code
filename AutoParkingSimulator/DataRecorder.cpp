#include "DataRecorder.h"

void savePathSolutionToTextFile(std::string& filename, ParkingLot& lot) {
	std::ofstream textFileStream;
	textFileStream.open(filename);
		
	// the numer of cars
	textFileStream << lot.agents.size() << "\n";

	//  color, path solution and goal position...
	for (unsigned int k = 0; k < lot.agents.size(); k++) {
		// color 
		textFileStream << lot.agents[k].color[0] << " " << lot.agents[k].color[1] << " " << lot.agents[k].color[2] << "\n";
		// goal position 
		textFileStream << lot.agents[k].goalPose.getPoseX() << " " << lot.agents[k].goalPose.getPoseY() << " " << lot.agents[k].goalPose.getPoseTheta() << "\n";
		// the length of the path solution 
		textFileStream << lot.agents[k].carPathPtr->getStateCount() << "\n";

		// path solution 
		if (lot.agents[k].carPathPtr) {
			for (unsigned int i = 0; i < lot.agents[k].carPathPtr->getStateCount(); i++) {
				textFileStream <<
					lot.agents[k].carPathPtr->getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getX() << " " <<
					lot.agents[k].carPathPtr->getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getY() << " " <<
					lot.agents[k].carPathPtr->getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getYaw() << "\n";
			}
		}
		else {
			throw std::runtime_error("ERROR: an agent does not have path!!!"); 
		}

	}

	textFileStream.close();
}

