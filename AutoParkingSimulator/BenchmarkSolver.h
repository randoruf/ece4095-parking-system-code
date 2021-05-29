#pragma once
#include <string>
#include <fstream>

class BenchmarkSolver
{	
public:
	BenchmarkSolver();
	// 
	void solve_test_goal_assignment(std::string benchmarkFilename);

	// read benchamark....
	void solve_test_one_agent();
	void solve_test_two_agent();
	void solve(std::string bmFilename, std::string dataDirectroy);


private:
	void saveFailCase(std::string& filename) {
		std::ofstream textFileStream;
		textFileStream.open(filename);
		textFileStream << "0\n";
		textFileStream.close();
	}
};

