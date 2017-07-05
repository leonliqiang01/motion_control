#ifndef __TRAJECTORY_TRACKING_H__
#define __TRAJECTORY_TRACKING_H__

#include "trajectory_twist.h"

class TrajectoryTracking
{
public:
	virtual TrajectoryTwist ControlOutput(const TrajectoryTwist &cmd, const TrajectoryTwist &state) = 0;
};

class LinearTracking : public TrajectoryTracking
{
public:
	LinearTracking(double xi, double omega_a):xi_(xi),omega_a_(omega_a)
	{}
	virtual ~LinearTracking(){}
	virtual TrajectoryTwist ControlOutput(const TrajectoryTwist &cmd, const TrajectoryTwist &state);

protected:
	double xi_;
	double omega_a_;
};

#endif