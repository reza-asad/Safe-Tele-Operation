/**
 * @file safe_teleop.cpp
 * @brief Safe teleoperation library implementation
 * Created by rakesh on 28/09/18.
 */
#include <limits>
#include <safe_teleop/safe_teleop.h>
using namespace std;

namespace safe_teleop
{

SafeTeleop::SafeTeleop() :
  is_shutdown_(false),
  max_cmd_vel_age_(1.0),
  max_linear_vel_(1.0),
  max_angular_vel_(1.0),
  linear_vel_increment_(0.05),
  angular_vel_increment_(0.05),
  laser_safety_check_angle_(0.25),
  min_safety_impact_time_(0.5),
  min_safety_distance_(0.5),
  linear_vel_(0.0),
  angular_vel_(0.0),
  last_command_timestamp_(0.0)
{
  ros::NodeHandle global_nh;
  cmd_vel_pub_ = global_nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  // The subscriber callback is set to the laserScanCallback method of the instantiated object of this class
  laser_scan_sub_ = global_nh.subscribe("scan", 5, &SafeTeleop::laserScanCallback, this);

  // Initialize the step
  step_.linear.x = linear_vel_;
  step_.angular.z = angular_vel_;

  run_thread_ = boost::thread(&SafeTeleop::run, this);
  displayCurrentSpeeds();
}

SafeTeleop::~SafeTeleop()
{
  shutdown();
  // wait for the run thread to terminate
  run_thread_.join();

  geometry_msgs::Twist zero_cmd_vel;
  zero_cmd_vel.linear.x = 0;
  zero_cmd_vel.angular.z = 0;
  cmd_vel_pub_.publish(zero_cmd_vel);
}

void SafeTeleop::run()
{
  ros::Rate r(10);
  while (ros::ok() && !is_shutdown_)
  {
    auto current_timestamp = ros::Time::now().toSec();

    auto last_cmd_vel_age = current_timestamp - last_command_timestamp_;

    // If timeout publish zero velocity
    if (last_cmd_vel_age > max_cmd_vel_age_)
    {
      geometry_msgs::Twist zero_cmd_vel;
      zero_cmd_vel.linear.x = 0;
      zero_cmd_vel.angular.z = 0;
      cmd_vel_pub_.publish(zero_cmd_vel);
    }
    else
    {
      auto is_safe = checkSafety(static_cast<double>(linear_vel_));
      if (is_safe) cmd_vel_pub_.publish(step_);
    }

    r.sleep();
  }
}

void SafeTeleop::moveForward()
{
	step_.linear.x = linear_vel_;
	last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::moveBackward()
{
	step_.linear.x = -linear_vel_;
	last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::rotateClockwise()
{
	step_.angular.z = -angular_vel_;
	last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::rotateCounterClockwise()
{
  step_.angular.z = angular_vel_;
	last_command_timestamp_ = ros::Time::now().toSec();
}

void SafeTeleop::stop()
{
	linear_vel_.store(0.0);
	linear_speed_.store(0.0);
	angular_vel_.store(0.0);
	angular_speed_.store(0.0);
	step_.linear.x = linear_vel_;
	step_.angular.z = angular_vel_;
	ROS_WARN("Stop requested\r");
	last_command_timestamp_ = ros::Time::now().toSec();
}


void SafeTeleop::increaseLinearSpeed()
{
  double current = linear_vel_.load();
  double next_vel_ = current + linear_vel_increment_;
  if (next_vel_ <= max_linear_vel_) {
    linear_vel_.store(next_vel_);
    linear_speed_ = abs(linear_vel_);
  } else {
  	linear_vel_.store(max_linear_vel_);
  	linear_speed_.store(max_linear_vel_);
  }
  displayCurrentSpeeds();
}

void SafeTeleop::decreaseLinearSpeed()
{
  // check if speed limit is satisfied
  double current = linear_vel_.load();
  double next_vel_ = current - linear_vel_increment_;
  if (next_vel_ >= -max_linear_vel_) {
  	linear_vel_.store(next_vel_);
    linear_speed_ = abs(linear_vel_);
  } else {
  	linear_vel_.store(-max_linear_vel_);
  	linear_speed_.store(max_linear_vel_);
  }
  displayCurrentSpeeds();
}

void SafeTeleop::increaseAngularSpeed()
{	
	double current = angular_vel_.load();
	double next_vel_ = current + angular_vel_increment_;
	if (next_vel_ <= max_angular_vel_) {
		angular_vel_.store(next_vel_);
		angular_speed_ = abs(angular_vel_);
	} else {
		angular_vel_.store(max_angular_vel_);
		angular_speed_.store(max_angular_vel_);
	}
  displayCurrentSpeeds();
}

void SafeTeleop::decreaseAngularSpeed()
{
	double current = angular_vel_.load();
	double next_vel_ = current - angular_vel_increment_;
	if (next_vel_ >= -max_angular_vel_) {
		angular_vel_.store(next_vel_);
		angular_speed_ = abs(angular_vel_);
	} else {
		angular_vel_.store(-max_angular_vel_);
		angular_speed_.store(max_angular_vel_);
	}
  displayCurrentSpeeds();
}

bool SafeTeleop::checkSafety(double linear_vel)
{
  sensor_msgs::LaserScan laser_scan_ = getLaserScan();
  double current = step_.linear.x;
  if (current == 0) return true;
  if (current == linear_vel) {
  	int start_idx = (M_PI - laser_safety_check_angle_) / laser_scan_.angle_increment;
  	int end_idx = ceil((M_PI - laser_safety_check_angle_) / laser_scan_.angle_increment);
  	for (int i=start_idx; i <= end_idx; ++i) {
  		if ((laser_scan_.ranges[i] + linear_vel_ * min_safety_impact_time_) < min_safety_distance_) {
  			ROS_WARN("Not safe to go forward\r");
  			return false;
  		}
  	}
  	// ROS_WARN("increment: %lf \r", (double)laser_scan_.ranges.size());
  } else {
  	ROS_WARN("Going backward\r");
  }

  return true;
}

} // namespace safe_teleop_node


