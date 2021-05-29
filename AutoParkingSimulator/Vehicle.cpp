#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include "Vehicle.h"
#include <cassert>
#include <iostream>


Pose& Pose::operator=(const Pose& lhs) {
	x_ = lhs.getPoseX(); 
	y_ = lhs.getPoseY(); 
	theta_ = lhs.getPoseTheta(); 
	steer_ = lhs.getPoseSteer();

	return (*this);
}


Pose::Pose() {
	x_ = 0; 
	y_ = 0; 
	theta_ = 0; 
	steer_ = 0; 
}

Pose::Pose(double x, double y, double theta) {
	x_ = x; 
	y_ = y; 
	theta_ = theta; 
	steer_ = 0; 
}

Pose::Pose(double x, double y, double theta, double steerAngle) {
	x_ = x; 
	y_ = y; 
	theta_ = theta; 
	steer_ = steerAngle; 
}


void Pose::setPose(double x, double y, double theta, double steerAngle) {
	// assert(x >= 0.0);
	// assert(y >= 0.0);
	// assert(theta <= M_PI);
	// assert(theta >= -M_PI); 
	
	// OMPL may repair the solution, and that generate some impossible answer....
	if (x < 0) {
		x = 0; 
	}
	if (y < 0) {
		y = 0; 
	}

	// 
	x_ = x; 
	y_ = y; 
	theta_ = theta; 
	steer_ = steerAngle; 
}

double Pose::getPoseX() const {
	return x_; 
}

double Pose::getPoseY() const {
	return y_; 
}

double Pose::getPoseTheta() const {
	return theta_; 
}

double Pose::getPoseSteer() const {
	return steer_; 
}

void Pose::setPoseX(double x) {
	x_ = x; 
}

void Pose::setPoseY(double y) {
	y_ = y; 
}

void Pose::setPoseTheta(double theta) {
	theta_ = theta;
}

void Pose::setPoseSteer(double steer) {
	steer_ = steer;
}
