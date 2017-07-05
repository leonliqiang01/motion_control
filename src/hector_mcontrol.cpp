#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include <chrono>
#include "trajectory_generator.h"
#include "trajectory_tracking.h"

const double control_fre    = 20;                    //主要代码执行频率
const double control_period = 1.0f/control_fre; 

inline double getYaw(const geometry_msgs::Pose &pose)
{
	const geometry_msgs::Quaternion::_w_type& w = pose.orientation.w;
	const geometry_msgs::Quaternion::_w_type& x = pose.orientation.x;
	const geometry_msgs::Quaternion::_w_type& y = pose.orientation.y;
	const geometry_msgs::Quaternion::_w_type& z = pose.orientation.z;
	return atan2(2.*x*y + 2.*w*z, x*x + w*w - z*z - y*y);
}

void GroundTruthCallback(const nav_msgs::OdometryConstPtr& odometryMsg);
nav_msgs::Odometry ground_truth_state;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hector_mcontrol");
	ros::NodeHandle handle;
	ros::Subscriber ground_truth_sub = handle.subscribe("/ground_truth/state",2,GroundTruthCallback);
	ros::Publisher  vel_pub    = handle.advertise<geometry_msgs::Twist>("/cmd_vel",4);
	ros::Publisher  marker_pub = handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	
	visualization_msgs::Marker cmd_points, state_points;
	cmd_points.header.frame_id = state_points.header.frame_id = "world";
	cmd_points.header.stamp    = state_points.header.stamp = ros::Time::now();
	cmd_points.ns = state_points.ns = "hector_mcontrol";
	cmd_points.action = state_points.action = visualization_msgs::Marker::ADD;
	cmd_points.pose.orientation.w = state_points.pose.orientation.w = 1.0;
	
	cmd_points.id = 0;
	state_points.id = 1;
	
	cmd_points.type = visualization_msgs::Marker::POINTS;
	state_points.type = visualization_msgs::Marker::POINTS;
	
	cmd_points.scale.x = 0.2;
	cmd_points.scale.y = 0.2;
	
	state_points.scale.x = 0.2;
	state_points.scale.y = 0.2;
	
	cmd_points.color.r = 1.0f;
	cmd_points.color.a = 1.0f;
	
	state_points.color.g = 1.0f;
	state_points.color.a = 1.0f;
	
	ros::Duration(5).sleep();
	ros::Rate loop_rate(control_fre);
	
	std::shared_ptr<TrajectoryGenerator> tra_generate = std::make_shared<CircleTrajectory>(3,(2*M_PI/20.0f),0,3);
	std::shared_ptr<TrajectoryTracking>  tra_tracking = std::make_shared<LinearTracking>(0.7,1);
	
	while(ros::ok())
	{
		ros::spinOnce();
		geometry_msgs::Twist output_twist;
		
		if(ground_truth_state.pose.pose.position.z >= 1.5f)
		{
			static double time = 0;
			TrajectoryTwist cmd_twist = tra_generate->GetNextTwist(time);
			TrajectoryTwist state_twist;
			state_twist.x_ = ground_truth_state.pose.pose.position.x;
			state_twist.y_ = ground_truth_state.pose.pose.position.y;
			state_twist.yaw_ = getYaw(ground_truth_state.pose.pose);
			state_twist.vd_  = sqrt(pow(ground_truth_state.twist.twist.linear.x,2) + 
									pow(ground_truth_state.twist.twist.linear.y,2));
			state_twist.omega_ = ground_truth_state.twist.twist.angular.z;
			TrajectoryTwist output = tra_tracking->ControlOutput(cmd_twist,state_twist);
			
			output_twist.linear.x = output.vd_;
			output_twist.linear.y = 0;
			output_twist.linear.z = 0;
			output_twist.angular.z = output.omega_;
			
			geometry_msgs::Point point;
			point.x = cmd_twist.x_;
			point.y = cmd_twist.y_;
			point.z = 0;
			
// 			if(cmd_points.points.size() > 0)
// 			{
// 				cmd_points.points.pop_back();
// 			}
// 			std::cout << "cmd position is: " << cmd_twist.x_ << ',' << cmd_twist.y_ << std::endl;
			cmd_points.points.push_back(point);
			
			point.x = state_twist.x_;
			point.y = state_twist.y_;
			point.z = 0;
			state_points.points.push_back(point);
			
			time += control_period;
		}
		else
		{
			output_twist.linear.z = 0.2f;
		}
		vel_pub.publish(output_twist);
		marker_pub.publish(cmd_points);
		marker_pub.publish(state_points);
		loop_rate.sleep();
	}
	return 0;
}

void GroundTruthCallback(const nav_msgs::OdometryConstPtr& odometryMsg)
{
	ground_truth_state = *odometryMsg;
}
