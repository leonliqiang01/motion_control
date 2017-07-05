#include "trajectory_generator.h"
#include <cmath>

TrajectoryTwist CircleTrajectory::GetNextTwist(double time)
{
	static double last_time = 0;
	static double last_yaw  = 0;
	TrajectoryTwist _twist;
	_twist.x_     = x_c_ + radius_d_*sin(omega_d_*time);
	_twist.y_     = y_c_ - radius_d_*cos(omega_d_*time);
	_twist.yaw_   = last_yaw + omega_d_*(time-last_time);
	
	while(_twist.yaw_ > M_PI || _twist.yaw_ < -M_PI)
	{
		if(_twist.yaw_ > M_PI) 
			_twist.yaw_ -= 2*M_PI;
		else if(_twist.yaw_ < -M_PI)
			_twist.yaw_ += 2*M_PI;
	}
	
	_twist.omega_ = omega_d_; 
	_twist.vd_    = radius_d_*omega_d_;
	
	last_yaw = _twist.yaw_;
	last_time = time;
	return _twist;
}