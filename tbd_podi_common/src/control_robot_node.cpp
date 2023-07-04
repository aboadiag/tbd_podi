#include "ros/ros.h"
#include <tbd_podi_common/control_robot.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "control_robot");

	// Deal with the spinning outside using Async spinner
	ros::AsyncSpinner spinner(2); // Use 2 threads
	spinner.start();
	
	tbd_podi_common::ControlRobot cr;
	cr.spin();

	return 0;
}
