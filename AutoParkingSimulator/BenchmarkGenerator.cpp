#include "BenchmarkGenerator.h"
#include "ParkingLot.h"


/**
 * steal from https://www.oreilly.com/library/view/c-cookbook/0596007612/ch10s15.html
 */
std::string BenchmarkGenerator::getFileNameFull(const std::string& s) {
	char sepChar = '/'; 
	// find last occurrence of content in string (reading in reverse order.....)
	size_t i = s.rfind(sepChar, s.length()); 
	if (i != std::string::npos) {
		return(s.substr(i + 1, s.length() - i)); 
	}
	// if not occurrence.... npos means null position.... 
	return (""); 
}


std::string BenchmarkGenerator::getFileName(const std::string& s) {
	std::string fullFilename = getFileNameFull(s); 
	char sepChar = '.'; 
	size_t i = fullFilename.rfind(sepChar, fullFilename.length());
	if (i != std::string::npos) { 
		return (fullFilename.substr(0, i)); 
	}
	return (""); 
}




/**
* Generate a number of benchmark problems in which each agent starts at a random position.
* In this kind of benchmark problem, a car is not allowed to go out the scene as the  
* benchmark here is for comparing multi-agent motion planning algorithms. 
* 
* A problem is specified by multiple agents. 
*	- how many agents in the scene 
*   - the start position of each agent 
*
* @param mapSlotFilename: the filename of the map 
* @param numAgent: the number of agents in the parking lot 
* @param numInstance: to make our life easier, the function generates a number of scenarios given the number of agents. 
* @return sum of `values`, or 0.0 if `values` is empty.
*/
void BenchmarkGenerator::randomStartPositionScenario(std::string mapSlotFilename, unsigned numAgent, unsigned numInstance) {

	// generate a virtual parking lot without any planning activities.... 
	ParkingLot lot(mapSlotFilename);
	// make sure that the number of agents is a valid number.
	assert(numAgent > 0);
	assert(numAgent <= lot.parkingSpaces.size());

	// save the benchmark problem... 
	std::ofstream textFileStream;
	textFileStream.open("./benchmark_problem/" + getFileName(mapSlotFilename) + ("_agent" + std::to_string(numAgent) + "_instance" + std::to_string(numInstance) + ".txt"));
	
	// the parking lot used by the problem.... 
	textFileStream << getFileNameFull(mapSlotFilename) << "\n";
	
	// the numer of cars in the parking lot
	textFileStream << numAgent << "\n";

	// create necessary car instances... 
	for (unsigned int i = 0; i < numAgent; i++) {
		lot.agents.push_back(Agent(lot));
	}

	// given the number of agents, we may have different scenarios/instances of the problem.....
	textFileStream << numInstance << "\n";
	for (unsigned int j = 0; j < numInstance; j++) {
		// textFileStream << j << "\n";
		for (auto& c : lot.agents) {
			c.generateRandomInitialPoseAt(); 
			textFileStream << c.intialPose.getPoseX() << " " << c.intialPose.getPoseY() << " " << c.intialPose.getPoseTheta() << "\n";
		}
	}


	// close the text file.... 
	textFileStream.close();
	std::cout << "Benchmark generated....\n"; 
}