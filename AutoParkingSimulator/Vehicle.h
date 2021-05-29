#pragma once
#include <vector>

class VehicleDimensionInfo {
public:
	// the vehicle dimension can be seen at https://au.mathworks.com/help/driving/ref/sedan.html
	// the width and length of the car 
	double carWidth, carLength;
	double carRearAxleFrontEndDistance, carRearAxleBackEndDistance;
	// the distance from rear axle to the front axle, which is also called wheelBase_. 
	//	- http://lavalle.pl/rrt/gallery_carsmooth.html
	//	- https://en.wikipedia.org/wiki/Wheelbase

	// The state of the vehicle, see https://arxiv.org/pdf/1604.07446.pdf and http://lavalle.pl/rrt/gallery_carsmooth.html
	// (x, y, theta, delta) is pose of the vehicle, theta is the orientation, while delta is to indicate the steering angle. 
	// theta [-180, 180], or [-pi, pi] https://ompl.kavrakilab.org/GeometricCarPlanning_8cpp_source.html 
	// and this C4M6L3---Trajectory-Rollout-Algorithm-Slides https://www.coursera.org/lecture/motion-planning-self-driving-cars/lesson-1-trajectory-propagation-5hguf
	// delta [-MaxSteeringAngle, MinSteeringAngle]. 
	double carWheelBase;
	double carMaxSpeed, carMaxSteerAngle, carMaxSteerRate;
	double carinflatedRadius;

	VehicleDimensionInfo() {
		// the car dimension
		carWidth = 0;
		carLength = 0;
		carRearAxleFrontEndDistance = 0;
		carRearAxleBackEndDistance = 0;
		carWheelBase = 0;
		// the dynamic constraints 
		carMaxSpeed = 0;
		carMaxSteerAngle = 0;
		carMaxSteerRate = 0;
		// for collision checking 
		carinflatedRadius = 0;
	}

	VehicleDimensionInfo(
		double carWidth_,
		double carLength_,
		double carRearAxleFrontEndDistance_,
		double carRearAxleBackEndDistance_,
		double carWheelBase_,
		double carMaxSpeed_,
		double carMaxSteerAngle_,
		double carMaxSteerRate_,
		double carinflatedRadius_
	)
	{
		// the vehicle dimension 
		carWidth = carWidth_;
		carLength = carLength_;
		carRearAxleFrontEndDistance = carRearAxleFrontEndDistance_;
		carRearAxleBackEndDistance = carRearAxleBackEndDistance_;
		carWheelBase = carWheelBase_;
		// the dynamic constriants 
		carMaxSpeed = carMaxSpeed_;
		carMaxSteerAngle = carMaxSteerAngle_;
		carMaxSteerRate = carMaxSteerRate_;
		// for safe collsion checking 
		carinflatedRadius = carinflatedRadius_;
	};

};

class Pose
{
public:
	Pose();
	Pose(double x, double y, double theta);
	Pose(double x, double y, double theta, double steerAngle); 
	~Pose() = default;
	void setPose(double x, double y, double theta, double steerAngle = 0); 
	
	double getPoseX() const; 
	double getPoseY() const; 
	double getPoseTheta() const; 
	double getPoseSteer() const; 

	void setPoseX(double x);
	void setPoseY(double y); 
	void setPoseTheta(double theta); 
	void setPoseSteer(double steer); 

	Pose& operator=(const Pose& lhs); 

private:
	// orientation and steering angle are in radian. 
	double x_, y_, theta_, steer_; 
};