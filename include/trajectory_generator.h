#ifndef __TRAJECTORY_GENERATOR_H__
#define __TRAJECTORY_GENERATOR_H__

#include "trajectory_twist.h"

class TrajectoryGenerator
{
public:
	virtual TrajectoryTwist GetNextTwist(double time) = 0;
};

class CircleTrajectory : public TrajectoryGenerator
{
public:
	CircleTrajectory(double radius, double omega, double xc, double yc):radius_d_(radius),omega_d_(omega),x_c_(xc),y_c_(yc){}
	virtual ~CircleTrajectory(){}
	virtual TrajectoryTwist GetNextTwist(double time);
protected:
	double radius_d_;
	double omega_d_;
	double x_c_;
	double y_c_;
};

#endif
