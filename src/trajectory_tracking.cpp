#include "trajectory_tracking.h"
#include <cmath>
#include <boost/concept_check.hpp>

//基于近似线性化的控制
TrajectoryTwist LinearTracking::ControlOutput(const TrajectoryTwist& cmd, const TrajectoryTwist& state)
{
	double e1 = cos(state.yaw_)*(cmd.x_ - state.x_) + sin(state.yaw_)*(cmd.y_ - state.y_);
	double e2 = -sin(state.yaw_)*(cmd.x_ - state.x_) + cos(state.yaw_)*(cmd.y_ - state.y_);
	double e3 = cmd.yaw_ - state.yaw_;
	if(e3 > M_PI)
		e3 -= 2*M_PI;
	else if(e3 < -M_PI)
		e3 += 2*M_PI;
	
	double k1 = 2*xi_*omega_a_;
	double k2 = (omega_a_*omega_a_ - cmd.omega_*cmd.omega_)/cmd.vd_;
	double k3 = k1;
	
	double u1 = -k1*e1;
	double u2 = -k2*e2- k3*e3;
	
	double v = cmd.vd_*cos(e3) - u1;
	double omega = cmd.omega_  - u2;
	
	TrajectoryTwist twist;
	twist.vd_ = v;
	twist.omega_ = omega;
	return twist;
}
