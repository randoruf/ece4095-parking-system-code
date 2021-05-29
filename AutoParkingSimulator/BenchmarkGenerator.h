#pragma once
#include <string>

namespace BenchmarkGenerator {
	std::string getFileNameFull(const std::string& s);
	std::string getFileName(const std::string& s); 
	void randomStartPositionScenario(std::string mapSlotFilename, unsigned numAgent, unsigned numInstance);
}