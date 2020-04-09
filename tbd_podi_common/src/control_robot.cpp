#include <tbd_podi_common/control_robot.h>

namespace tbd_podi_common {

	ControlRobot::ControlRobot() : private_nh("~") 
	{

		lin_max = 1.0;
		ang_max = M_PI / 5;
		lin_min = 0.01;
		ang_min = 0.05;

		ros_navigation_cmd_vel_sub = n.subscribe("cmd_vel", 1000, &ControlRobot::navigationCB, this);
		cmd_vel_pub = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);
		joy_sub = n.subscribe("joy", 1000, &ControlRobot::joyCB, this);

		rosbagTopicName = "/Podi/cmd_vel_stamped";

		emergencyStop = false;
		recording = false;
		playingBack = false;
		newEmergencyStop = false;
		newRecording = false;
		newPlayingBack = false;
		newJoyVel = false;
		arePrintingRobotPose = false;
		isRecordingRosbagOpen = false;

		private_nh.param("control_loop_rate", controlLoopHz, 100.0);
		private_nh.param("proportionalGainFactor", proportionalGainFactor, 0.90);

		if (!private_nh.getParam("rosbagRecordingBasePath", rosbagRecordingDir)) {
			ROS_ERROR("ROSParam rosbagRecordingDir not defined. Unexpected behavior may ensue when trying to record a rosbag.");
		}
	}

	ControlRobot::~ControlRobot() 
	{
	}

	void ControlRobot::spin() 
	{

		// setup local variables
		rosbag::Bag* recordingBag;
		geometry_msgs::Twist lastPlaybackSpd;
		geometry_msgs::Twist lastJoySpd;
		geometry_msgs::Twist lastNavigationSpd;
		geometry_msgs::Twist lastPublishedSpd;
		// set ros::Rate to the desire control rate
		ros::Rate loop_rate(controlLoopHz);

		// start looping until ros shutdown.
		while (!ros::isShuttingDown()) {
			// Check if emergency stop value changed
			std::unique_lock<std::mutex> emergencyStopLock(emergencyStopMutex);
			// check if there is a new emergency lock
			if (newEmergencyStop) {
				// reset the check flag
				newEmergencyStop = false;
				// check whether we have already stopped
				// if stopped, renable the robot
				if (emergencyStop) {
					emergencyStopLock.unlock();
					ControlRobot::stopRobot(recordingBag);
					lastPublishedSpd.linear.x = 0;
					lastPublishedSpd.linear.y = 0;
					lastPublishedSpd.linear.z = 0;
					lastPublishedSpd.angular.x = 0;
					lastPublishedSpd.angular.y = 0;
					lastPublishedSpd.angular.z = 0;
					ROS_INFO("EMERGENCY STOP!!!!");
				} 
				else 
				{
					ROS_INFO("Re-enabling robot.");
				}
				continue;
			} 
			else 
			{
				emergencyStopLock.unlock();
			}

			// Check if recording value changed
			std::unique_lock<std::mutex> recordingLock(recordingMutex);
			if (newRecording) {
				newRecording = false;
				if (recording) {
					recordingLock.unlock();

					// create the filename
					std::time_t rawtime;
					char buffer [80];
					std::time(&rawtime);
					std::tm* timeinfo = std::localtime(&rawtime);
					std::strftime(buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
					std::string filename = rosbagRecordingDir+buffer+std::string(".bag");
					// create the new rosbag
					recordingBag = new rosbag::Bag(filename, rosbag::bagmode::Write);
					ROS_INFO("Starting recording to rosbag %s", buffer);
					std::unique_lock<std::mutex> isRecordingRosbagOpenLock(isRecordingRosbagOpenMutex);
					isRecordingRosbagOpen = true;
					isRecordingRosbagOpenLock.unlock();
				} else {
					recordingLock.unlock();
					ROS_INFO("Closing the rosbag we were recording to");
					std::unique_lock<std::mutex> isRecordingRosbagOpenLock(isRecordingRosbagOpenMutex);
					isRecordingRosbagOpen = false;
					isRecordingRosbagOpenLock.unlock();
					recordingBag->close();
					delete recordingBag;
				}
				continue;
			} 
			else 
			{
				recordingLock.unlock();
			}

			// Check if playingBack value changed
			std::unique_lock<std::mutex> playingBackLock(playingBackMutex);
			if (newPlayingBack) 
			{
				newPlayingBack = false;
				if (playingBack) 
				{
					if (private_nh.getParam("playbackRosbagPath", playbackRosbagPath) && playbackRosbagPath.length() > 0) 
					{
						playingBackLock.unlock();
						playbackBagThread = std::thread(&ControlRobot::playbackBagFn, this, playbackRosbagPath, proportionalGainFactor);
						// clear out the last playback speed
						lastPlaybackSpd.linear.x = 0;
						lastPlaybackSpd.linear.y = 0;
						lastPlaybackSpd.linear.z = 0;
						lastPlaybackSpd.angular.x = 0;
						lastPlaybackSpd.angular.y = 0;
						lastPlaybackSpd.angular.z = 0;
					} 
					else 
					{
						playingBack = false;
						playingBackLock.unlock();
						ROS_WARN("No playback rosbag defined");
					}
				} 
				else 
				{
					playingBackLock.unlock();
					playbackBagThread.join(); // wait for the thread to terminate
					ControlRobot::stopRobot(recordingBag);
					// clear out the last playback speed
					lastPlaybackSpd.linear.x = 0;
					lastPlaybackSpd.linear.y = 0;
					lastPlaybackSpd.linear.z = 0;
					lastPlaybackSpd.angular.x = 0;
					lastPlaybackSpd.angular.y = 0;
					lastPlaybackSpd.angular.z = 0;
				}
				continue;
			} 
			else 
			{
				playingBackLock.unlock();
			}

			bool wasVelPublished = false;

			// check if a velocity was published from the joystick
			std::unique_lock<std::mutex> joyLock(joyVelMutex);
			if (newJoyVel) 
			{
				newJoyVel = false;
				lastJoySpd.linear.x = joyVel.linear.x;
				lastJoySpd.linear.y = 0;
				lastJoySpd.linear.z = 0;
				lastJoySpd.angular.x = 0;
				lastJoySpd.angular.y = 0;
				lastJoySpd.angular.z = joyVel.angular.z;
				joyLock.unlock();
				// ROS_INFO("joy lin_spd: %0.2f, ang_spd: %0.2f", lastJoySpd.linear.x, lastJoySpd.angular.z);
				wasVelPublished = true;
			} 
			else 
			{
				joyLock.unlock();
			}

			// check if a velocity was published from the rosbag
			std::unique_lock<std::mutex> playbackLock(playbackVelMutex);
			if (newPlaybackVel) 
			{
				newPlaybackVel = false;
				lastPlaybackSpd.linear.x = playbackVel.linear.x;
				lastPlaybackSpd.linear.y = 0;
				lastPlaybackSpd.linear.z = 0;
				lastPlaybackSpd.angular.x = 0;
				lastPlaybackSpd.angular.y = 0;
				lastPlaybackSpd.angular.z = playbackVel.angular.z;
				playbackLock.unlock();
				// ROS_INFO("playback lin_spd: %0.2f, ang_spd: %0.2f", lastPlaybackSpd.linear.x, lastPlaybackSpd.angular.z);
				wasVelPublished = true;
			} 
			else 
			{
				playbackLock.unlock();
			}

			// check if a velocity was published from ROS Navigation
			std::unique_lock<std::mutex> navigationLock(navigationVelMutex);
			if (newNavigationVel) 
			{
				newNavigationVel = false;
				lastNavigationSpd.linear.x = navigationVel.linear.x;
				lastNavigationSpd.linear.y = 0;
				lastNavigationSpd.linear.z = 0;
				lastNavigationSpd.angular.x = 0;
				lastNavigationSpd.angular.y = 0;
				lastNavigationSpd.angular.z = navigationVel.angular.z;
				navigationLock.unlock();
				// ROS_INFO("navigation lin_spd: %0.2f, ang_spd: %0.2f", lastNavigationSpd.linear.x, lastNavigationSpd.angular.z);
				wasVelPublished = true;
			} 
			else 
			{
				navigationLock.unlock();
			}

			// Publish the velocity
			emergencyStopLock.lock();
			if (!emergencyStop) 
			{
				emergencyStopLock.unlock();
				if (wasVelPublished) 
				{
					geometry_msgs::Twist spdToPublish;
					// Combine the multiple velocities together
					spdToPublish.linear.x = lastJoySpd.linear.x + lastPlaybackSpd.linear.x + lastNavigationSpd.linear.x;
					spdToPublish.linear.y = 0;
					spdToPublish.linear.z = 0;
					spdToPublish.angular.x = 0;
					spdToPublish.angular.y = 0;
					spdToPublish.angular.z = lastJoySpd.angular.z + lastPlaybackSpd.angular.z + lastNavigationSpd.angular.z;
					// ROS_INFO("lin_spd: %0.2f, ang_spd: %0.2f", spdToPublish.linear.x, spdToPublish.angular.z);
					lastPublishedSpd = spdToPublish;
					ControlRobot::publishVelocity(spdToPublish, recordingBag);
				} 
				else 
				{
					ControlRobot::publishVelocity(lastPublishedSpd, recordingBag);
				}
			} 
			else 
			{
				emergencyStopLock.unlock();
			}

			// Print the robot dPose
			std::unique_lock<std::mutex> printingRobotPoseLock(printingRobotPoseMutex);
			if (arePrintingRobotPose) 
			{
				double targetX = targetRobotPose.position.x;
				double targetY = targetRobotPose.position.y;
				double targetTh = tf::getYaw(targetRobotPose.orientation);
				printingRobotPoseLock.unlock();

				// transfrom the 0 vector in \base_link to \map
				tf::Stamped<tf::Pose> base_pose, robot_pose;
		    	base_pose.frame_id_ = "/base_link";
		   		base_pose.stamp_ = ros::Time::now();
		    	base_pose.setOrigin(tf::Vector3(0, 0, 0));
				base_pose.setRotation(tf::Quaternion(0, 0, 0, 1));

				try 
				{
					tf_.waitForTransform("/map", base_pose.frame_id_, base_pose.stamp_, ros::Duration(1.0));
					tf_.transformPose("/map", base_pose, robot_pose);

					// transform that robot location to x, y, theta in cm and degrees
					double robotX, robotY, robotTh;
					robotX = robot_pose.getOrigin().getX();
					robotY = robot_pose.getOrigin().getY();
					robotTh = tf::getYaw(robot_pose.getRotation());

					ROS_WARN("dPose=(%.4f, %.4f, %.4f), robotPose=(%.4f, %.4f, %.4f)", targetX-robotX, targetY-robotY, targetTh-robotTh, robotX, robotY, robotTh);

		    	} 
				catch (tf::TransformException ex) 
				{
		      		ROS_ERROR("ControlRobot::controlLoop %s", ex.what());
		    	}
			} 
			else 
			{
				printingRobotPoseLock.unlock();
			}

			// looping
			loop_rate.sleep();
			// Debug and check the cycle time
			if (loop_rate.cycleTime() > loop_rate.expectedCycleTime()){
				ROS_WARN("Looping Rate slower at %.4f instead of %.4f", loop_rate.cycleTime().toSec(),loop_rate.expectedCycleTime().toSec());
			}
		}
	}

	void ControlRobot::stopRobot(rosbag::Bag* recordingBag) 
	{
		geometry_msgs::Twist spd; // default is a zero twist
		ControlRobot::publishVelocity(spd, recordingBag);
	}

	void ControlRobot::playbackBagFn(std::string playbackRosbagPath, double p) 
	{
		ROS_INFO("Opening rosbag %s", playbackRosbagPath.c_str());
		rosbag::Bag playbackBag(playbackRosbagPath, rosbag::bagmode::Read);
		std::vector<std::string> topics;
	  	topics.push_back(std::string(rosbagTopicName));
		rosbag::View view(playbackBag, rosbag::TopicQuery(topics));
		bool firstIter = true;
		double sumErr=0.0, sumAbsErr=0.0, minErr=0.0, maxErr=0.0;
		int n = 0;
		ros::Time timeAtEndOfSleep, lastIterBagTime;
		ros::Duration elapsedTimeNotIncludingSleep, elapsedActualTime, elapsedBagTime,
		errorTime, pdErrorTime, lastErrorTime, scaledDerivativeErrorTime,
		timeToSleep, zero=ros::Duration(0.0);
		// ROS_INFO("Starting playback from rosbag %s, FYI n=%d", playbackRosbagPath.c_str(), n);
		BOOST_FOREACH(rosbag::MessageInstance const m, view) 
		{
			geometry_msgs::TwistStamped::ConstPtr spd = m.instantiate<geometry_msgs::TwistStamped>();
		  	if (spd != NULL) 
			  {
				// ROS_INFO("Playback spd lin_spd: %0.2f, ang_spd: %0.2f", spd->twist.linear.x, spd->twist.angular.z);
				if (firstIter) 
				{
					lastIterBagTime = spd->header.stamp;
					firstIter = false;
					timeAtEndOfSleep = ros::Time::now();
				} 
				else 
				{
					elapsedBagTime = spd->header.stamp - lastIterBagTime;
					lastIterBagTime = spd->header.stamp;
					pdErrorTime = ros::Duration(p * errorTime.toSec());
					elapsedTimeNotIncludingSleep = ros::Time::now() - timeAtEndOfSleep;
					timeToSleep = elapsedBagTime - elapsedTimeNotIncludingSleep - pdErrorTime;
					if (timeToSleep > zero)
						timeToSleep.sleep(); // adjust for time
					elapsedActualTime = ros::Time::now() - timeAtEndOfSleep;
					timeAtEndOfSleep = ros::Time::now();
					++n;
					lastErrorTime = errorTime;
					errorTime = elapsedActualTime - elapsedBagTime;
					sumErr = sumErr + errorTime.toSec();
					sumAbsErr = sumAbsErr + fabs(errorTime.toSec());
					minErr = std::min(errorTime.toSec(), minErr);
					maxErr = std::max(errorTime.toSec(), maxErr);
				}
				std::unique_lock<std::mutex> playingBackLock(playingBackMutex);
				if (playingBack) 
				{
					playingBackLock.unlock();
					std::unique_lock<std::mutex> playbackLock(playbackVelMutex);
					playbackVel = spd->twist;
					newPlaybackVel = true;
					playbackLock.unlock();
				} 
				else 
				{
					playingBackLock.unlock();
					newPlaybackVel = false;
					break;
				}
			}
			if (ros::isShuttingDown()){
				break;
			}
		}
		ROS_INFO("Stopping playback from rosbag %s", playbackRosbagPath.c_str());
		// Close Rosbag when done
		std::unique_lock<std::mutex> playingBackLock(playingBackMutex);
		if (playingBack)
		{
			newPlayingBack = true;
		}
		playingBack = false;
		playingBackLock.unlock();
		ROS_INFO("Closing rosbag %s", playbackRosbagPath.c_str());
		playbackBag.close();
	}

	void ControlRobot::publishVelocity(geometry_msgs::Twist &spd, rosbag::Bag* recordingBag) 
	{
		{
			std::unique_lock<std::mutex> isRecordingRosbagOpenLock(isRecordingRosbagOpenMutex);
			if (isRecordingRosbagOpen) 
			{
				isRecordingRosbagOpenLock.unlock();
				geometry_msgs::TwistStamped spdStamped;
				spdStamped.header.stamp = ros::Time::now();
				spdStamped.twist.linear.x = spd.linear.x;
				spdStamped.twist.linear.y = spd.linear.y;
				spdStamped.twist.linear.z = spd.linear.z;
				spdStamped.twist.angular.x = spd.angular.x;
				spdStamped.twist.angular.y = spd.angular.y;
				spdStamped.twist.angular.z = spd.angular.z;
				// ROS_INFO("About to write to recordingBag");
				recordingBag->write(rosbagTopicName, ros::Time::now(), spdStamped);
			}
		}
		// // ROS_INFO("lin_spd: %0.2f, ang_spd: %0.2f", spd.linear.x, spd.angular.z);
		cmd_vel_pub.publish(spd);
	}

	void ControlRobot::joyCB(const sensor_msgs::Joy &msg) {

		double lin_axes = msg.axes[1]; // left thumb joystick
		double ang_axes = msg.axes[3]; // right thumb joystick
		if (msg.buttons[4] || msg.buttons[5]) 
		{ // if any of the estop buttons are pressed -- top LB & RB buttons
			{
				std::lock_guard<std::mutex> emergencyStopLock(emergencyStopMutex);
				if (!emergencyStop)
				{
					// signal that the value changed
					newEmergencyStop = true;
				}
				// update the value to be e-stopped
				emergencyStop = true;
			}
			return;
		} else if (msg.buttons[7]) { // re-enable robot -- start button
			ROS_INFO("Middle Start Button pressed!");
			{
			std::lock_guard<std::mutex> emergencyStopLock(emergencyStopMutex);
			if (emergencyStop)
			{
				// signal that the value changed
				newEmergencyStop = true;
			}
			// update the value to no longer on emergency stop
			emergencyStop = false;
			}
			return;
		}

		// Start printing robot pose
		if (msg.buttons[8]) 
		{ // Logitech button in the middle
			ROS_INFO("Middle Logitech Button pressed!");
			double x, y, th;
			if (!private_nh.getParam("targetX", x)) 
			{
				ROS_ERROR("No targetX, setting 0.0");
				x = 0.0;
			}
			if (!private_nh.getParam("targetY", y)) 
			{
				ROS_ERROR("No targetY, setting 0.0");
				y = 0.0;
			}
			if (!private_nh.getParam("targetTh", th)) 
			{
				ROS_ERROR("No targetTh, setting 0.0");
				th = 0.0;
			}
			std::unique_lock<std::mutex> printingRobotPoseLock(printingRobotPoseMutex);
			arePrintingRobotPose = true;
			targetRobotPose.position.x = x;
			targetRobotPose.position.y = y;
			targetRobotPose.position.z = 0.0;
			targetRobotPose.orientation = tf::createQuaternionMsgFromYaw(th);
			printingRobotPoseLock.unlock();
		}

		// End printing robot pose
		if (msg.buttons[6]) 
		{ // back button
			// lock this scope
			std::lock_guard<std::mutex> printingRobotPoseLock(printingRobotPoseMutex);
			arePrintingRobotPose = false;
		}

		// if (msg.buttons[6]) { // back button -- play a beep
		// 	system("modprobe pcspkr; beep");
		// }

		if (msg.buttons[1]) // stop recording -- B button / red button
		{ 
			std::lock_guard<std::mutex> recordingLock(recordingMutex);
			if (recording)
			{
				newRecording = true;
			}
			recording = false;
			return;
		} 
		else if (msg.buttons[2]) // start recording -- X button / blue button
		{ 
			std::lock_guard<std::mutex> recordingLock(recordingMutex);
			if (!recording)
			{
				newRecording = true;
			}
			recording = true;
			return;
		}

		if (msg.buttons[0])  // stop playback -- A button / green button
		{
			std::lock_guard<std::mutex> playingBackLock(playingBackMutex);
			ROS_INFO("Received the Closing rosbag command from the joystick for playback");
			if (playingBack) newPlayingBack = true;
			playingBack = false;
			return;
		} 
		else if (msg.buttons[3]) // start playback -- Y button / yellow button
		{ 
			std::lock_guard<std::mutex> playingBackLock(playingBackMutex);
			ROS_INFO("Received the Opening rosbag command from the joystick for playback");
			if (!playingBack) newPlayingBack = true;
			playingBack = true;
			return;
		}

		geometry_msgs::Twist spd;
		spd.linear.x = lin_max * lin_axes;
		spd.linear.y = 0;
		spd.linear.z = 0;
		spd.angular.x = 0;
		spd.angular.y = 0;
		spd.angular.z = ang_max * ang_axes;

		{
			
			std::lock_guard<std::mutex> joyLock(joyVelMutex);
			joyVel = spd;
			newJoyVel = true;
		}
		return;
	}

	void ControlRobot::navigationCB(const geometry_msgs::Twist &msg) {
		std::lock_guard<std::mutex> navigationLock(navigationVelMutex);
		navigationVel = msg;
		newNavigationVel = true;
	}

} // end namespace tbd_podi_common
