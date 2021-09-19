#include <tbd_podi_common/control_robot.h>

namespace tbd_podi_common
{

    ControlRobot::ControlRobot() : private_nh("~")
    {
        // Helper Variables
        double velLatchDurationDouble;

        // Get ROS Parameters
        private_nh.param("control_loop_rate", controlLoopHz, 100.0);
        private_nh.param("proportional_gain_factor", proportionalGainFactor, 0.90);
        private_nh.param("vel_latch_duration", velLatchDurationDouble, 0.25);
        private_nh.param("enabling_switch", enablingSwitchEnabled_, true);
        private_nh.param("sim", simOperation_, false);
        if (!private_nh.getParam("joystick_type", joystickType_))
        {
            joystickType_ = "logitech-dual-action";
        }
        if (!private_nh.getParam("rosbag_recording_base_path", rosbagRecordingDir))
        {
            ROS_ERROR("ROSParam rosbagRecordingDir not defined. Unexpected behavior may ensue when trying to record a rosbag.");
        }

        // Max/Min values used in Joystick control
        lin_max = 1.0;
        ang_max = M_PI / 5;
        lin_min = 0.01;
        ang_min = 0.05;

        ros_navigation_cmd_vel_sub = n.subscribe("cmd_vel", 1, &ControlRobot::navigationCB, this);
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("output_cmd_vel", 1);
        joy_sub = n.subscribe("joy", 1, &ControlRobot::joyCB, this);
        enableMotorClient = n.serviceClient<std_srvs::Empty>("enable_motors");
        disableMotorClient = n.serviceClient<std_srvs::Empty>("disable_motors");

        // Podi needs a small movement at the beginning to calibrate it's odometry
        // if we stop the motors immediately, then it would not work
        // Reference: https://github.com/amor-ros-pkg/rosaria/issues/31
        // std_srvs::Empty srv;
        // disableMotorClient.call(srv);

        rosbagTopicName = "output_cmd_vel_stamped";

        // setup local variables
        emergencyStop = !simOperation_;
        recording = false;
        playingBack = false;
        newEmergencyStop = false;
        newRecording = false;
        newPlayingBack = false;
        newJoyVel = false;
        arePrintingRobotPose = false;
        isRecordingRosbagOpen = false;
        velLatchDuration = ros::Duration(velLatchDurationDouble);
    }

    ControlRobot::~ControlRobot()
    {
    }

    void ControlRobot::startBagRecording()
    {
        // create the filename
        std::time_t rawtime;
        char buffer[80];
        std::time(&rawtime);
        std::tm *timeinfo = std::localtime(&rawtime);
        std::strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
        std::string filename = rosbagRecordingDir + buffer + std::string(".bag");
        // create the new rosbag
        ROS_INFO("Starting recording to rosbag %s", buffer);
        {
            std::lock_guard<std::mutex> isRecordingRosbagOpenLock(isRecordingRosbagOpenMutex);
            try
            {
                currentRecordingBag = new rosbag::Bag(filename, rosbag::bagmode::Write);
                isRecordingRosbagOpen = true;
            }
            catch (rosbag::BagIOException ex)
            {
                ROS_ERROR("Unable to Open Bag at Path: %s Error:%s", filename.c_str(), ex.what());
            }
        }
    }

    void ControlRobot::stopBagRecording()
    {
        {
            
            std::lock_guard<std::mutex> isRecordingRosbagOpenLock(isRecordingRosbagOpenMutex);
            if (isRecordingRosbagOpen)
            {
                ROS_INFO("Closing the rosbag we were recording to.");
                isRecordingRosbagOpen = false;
                currentRecordingBag->close();
                delete currentRecordingBag;
            }
        }
    }

    void ControlRobot::spin()
    {
        // setup local variables
        geometry_msgs::Twist lastPlaybackSpd;
        geometry_msgs::Twist lastJoySpd;
        geometry_msgs::Twist lastNavigationSpd;
        geometry_msgs::Twist lastPublishedSpd;
        bool wasVelPublished = false;
        auto lastVelPublishTime = ros::Time::now();

        // set ros::Rate to the desire control rate
        ros::Rate loop_rate(controlLoopHz);

        // start looping until ros shutdown.
        while (!ros::isShuttingDown())
        {
            // looping at a certain rate
            loop_rate.sleep();
            // Debug and check the cycle time
            if (loop_rate.cycleTime() > loop_rate.expectedCycleTime())
            {
                ROS_WARN("looping rate is slower at %.4f instead of %.4f", loop_rate.cycleTime().toSec(), loop_rate.expectedCycleTime().toSec());
            }

            // Check if emergency stop value changed
            {
                std::lock_guard<std::mutex> emergencyStopLock(emergencyStopMutex);
                if (newEmergencyStop)
                {
                    newEmergencyStop = false;
                    if (emergencyStop)
                    {
                        ControlRobot::stopRobot();
                        lastPublishedSpd.linear.x = 0;
                        lastPublishedSpd.linear.y = 0;
                        lastPublishedSpd.linear.z = 0;
                        lastPublishedSpd.angular.x = 0;
                        lastPublishedSpd.angular.y = 0;
                        lastPublishedSpd.angular.z = 0;
                        continue;
                    }
                }
                else
                {
                    // Nothing should run at all if E-stop is set.
                    if (emergencyStop)
                    {
                        continue;
                    }
                }
            }

            // Check if recording value changed
            {
                std::unique_lock<std::mutex> recordingLock(recordingMutex);
                if (newRecording)
                {
                    newRecording = false;
                    if (recording)
                    {
                        recordingLock.unlock();
                        startBagRecording();
                    }
                    else
                    {
                        recordingLock.unlock();
                        stopBagRecording();
                    }
                }
            }

            // Check if playingBack value changed
            {
                std::unique_lock<std::mutex> playingBackLock(playingBackMutex);
                if (newPlayingBack)
                {
                    newPlayingBack = false;
                    if (playingBack)
                    {
                        if (private_nh.getParam("playback_rosbag_path", playbackRosbagPath) && !playbackRosbagPath.empty())
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
                        ControlRobot::stopRobot();
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
            }

            wasVelPublished = false;

            // check if a velocity was published from the joystick
            {
                std::lock_guard<std::mutex> joyLock(joyVelMutex);
                if (newJoyVel)
                {
                    newJoyVel = false;
                    lastJoySpd = joyVel;
                    wasVelPublished = true;
                }
            }

            // check if a velocity was published from the rosbag
            {
                std::lock_guard<std::mutex> playbackLock(playbackVelMutex);
                if (newPlaybackVel)
                {
                    newPlaybackVel = false;
                    lastPlaybackSpd = playbackVel;
                    wasVelPublished = true;
                }
            }

            // check if a velocity was published from ROS Navigation
            {
                std::lock_guard<std::mutex> navigationLock(navigationVelMutex);
                if (newNavigationVel)
                {
                    newNavigationVel = false;
                    lastNavigationSpd = navigationVel;
                    wasVelPublished = true;
                }
            }

            // Publish the velocity
            {
                // before publish, check emergency stop
                std::unique_lock<std::mutex> emergencyStopLock(emergencyStopMutex);
                if (!emergencyStop)
                {
                    emergencyStopLock.unlock();
                    // check enabling switch
                    std::unique_lock<std::mutex> enablingswitchLock(enablingSwitchMutex_);
                    if (!enablingSwitchEnabled_ || enablingSwitchState_)
                    {
                        enablingswitchLock.unlock();
                        if (wasVelPublished)
                        {
                            geometry_msgs::Twist spdToPublish;
                            // Combine the multiple velocities together
                            // TODO: Explore in future the ability to assign weights
                            spdToPublish.linear.x = lastJoySpd.linear.x + lastPlaybackSpd.linear.x + lastNavigationSpd.linear.x;
                            spdToPublish.angular.z = lastJoySpd.angular.z + lastPlaybackSpd.angular.z + lastNavigationSpd.angular.z;
                            lastPublishedSpd = spdToPublish;
                            lastVelPublishTime = ros::Time::now();
                            ControlRobot::publishVelocity(spdToPublish);
                            continue;
                        }
                        else
                        {
                            if (ros::Time::now() - lastVelPublishTime < velLatchDuration)
                            {
                                ControlRobot::publishVelocity(lastPublishedSpd);
                                continue;
                            }
                        }
                    }
                    // publish empty velocity
                    geometry_msgs::Twist spd; // default is a zero twist
                    ControlRobot::publishVelocity(spd);
                }
            }
        }
    }

    void ControlRobot::stopRobot()
    {
        geometry_msgs::Twist spd; // default is a zero twist
        ControlRobot::publishVelocity(spd);
    }

    void ControlRobot::playbackBagFn(std::string playbackRosbagPath, double p)
    {
        ROS_INFO("Opening rosbag %s", playbackRosbagPath.c_str());
        try
        {
            rosbag::Bag playbackBag(playbackRosbagPath, rosbag::bagmode::Read);
            std::vector<std::string> topics;
            topics.push_back(std::string(rosbagTopicName));
            rosbag::View view(playbackBag, rosbag::TopicQuery(topics));
            bool firstIter = true;
            double sumErr = 0.0, sumAbsErr = 0.0, minErr = 0.0, maxErr = 0.0;
            int n = 0;
            ros::Time timeAtEndOfSleep, lastIterBagTime;
            ros::Duration elapsedTimeNotIncludingSleep, elapsedActualTime, elapsedBagTime,
                errorTime, pdErrorTime, lastErrorTime, scaledDerivativeErrorTime,
                timeToSleep, zero = ros::Duration(0.0);
            // ROS_INFO("Starting playback from rosbag %s, FYI n=%d", playbackRosbagPath.c_str(), n);
            BOOST_FOREACH (rosbag::MessageInstance const m, view)
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
                if (ros::isShuttingDown())
                {
                    break;
                }
            }
            ROS_INFO("Stopping playback from rosbag %s", playbackRosbagPath.c_str());
            // Close Rosbag when done
            {
                std::lock_guard<std::mutex> playingBackLock(playingBackMutex);
                if (playingBack)
                {
                    newPlayingBack = true;
                }
                playingBack = false;
            }
            ROS_INFO("Closing rosbag %s", playbackRosbagPath.c_str());
            playbackBag.close();
        }
        catch (rosbag::BagException eb)
        {
            ROS_ERROR("Unable to open or play bag at: %s due to %s" , playbackRosbagPath.c_str(), eb.what());
            return;
        }
    }

    void ControlRobot::publishVelocity(geometry_msgs::Twist &spd)
    {
        {
            // Check if we are recording right now and write to it if we are.
            std::unique_lock<std::mutex> isRecordingRosbagOpenLock(isRecordingRosbagOpenMutex);
            if (isRecordingRosbagOpen)
            {
                geometry_msgs::TwistStamped spdStamped;
                spdStamped.header.stamp = ros::Time::now();
                spdStamped.twist = spd;
                currentRecordingBag->write(rosbagTopicName, ros::Time::now(), spdStamped);
            }
        }
        // ROS_INFO("lin_spd: %0.2f, ang_spd: %0.2f", spd.linear.x, spd.angular.z);
        cmd_vel_pub.publish(spd);
    }

    void ControlRobot::joyCB(const sensor_msgs::Joy &msg)
    {
        // check if the estop or the restart button is pressed
        if (msg.buttons[button_mapping::estop_button] || msg.buttons[button_mapping::restart_robot])
        {
            std::lock_guard<std::mutex> emergencyStopLock(emergencyStopMutex);
            if (msg.buttons[button_mapping::estop_button] && !emergencyStop)
            {
                // signal that the value changed
                newEmergencyStop = true;
                emergencyStop = true;
                // call motor disable
                std_srvs::Empty srv;
                disableMotorClient.call(srv);
                ROS_INFO("EMERGENCY STOP BUTTON PRESSED!!!");
            }
            else if (msg.buttons[button_mapping::restart_robot] && emergencyStop)
            {
                // signal that the value changed
                newEmergencyStop = true;
                emergencyStop = false;
                // call motor disable
                std_srvs::Empty srv;
                enableMotorClient.call(srv);
                ROS_INFO("ROBOT MOTORS RE-ENABLED!!!");
            }
        }

        //update the enabling switch value
        {
            std::lock_guard<std::mutex> enablingSwitchLock(enablingSwitchMutex_);
            if (msg.axes[axis_mapping::enabling_switch] < -0.5)
            {
                enablingSwitchState_ = true;
            }
            else
            {
                enablingSwitchState_ = false;
            }
        }

        if (msg.buttons[button_mapping::beep_sound])
        {
            system("modprobe pcspkr; beep");
        }

        // start or stop recording
        if (msg.buttons[button_mapping::start_record_bag] || msg.buttons[button_mapping::stop_record_bag])
        {
            std::lock_guard<std::mutex> recordingLock(recordingMutex);
            if (msg.buttons[button_mapping::start_record_bag] && !recording)
            {
                newRecording = true;
                recording = true;
            }
            else if (msg.buttons[button_mapping::stop_record_bag] && recording)
            {
                newRecording = true;
                recording = false;
            }
            return;
        }

        // start or stop playback
        if (msg.buttons[button_mapping::start_replay_bag] || msg.buttons[button_mapping::stop_replay_bag])
        {
            std::lock_guard<std::mutex> playingBackLock(playingBackMutex);
            if (msg.buttons[button_mapping::start_replay_bag] && !playingBack)
            {
                ROS_INFO("Received the openning rosbag command from the joystick for playback");
                newPlayingBack = true;
                playingBack = true;
            }
            else if (msg.buttons[button_mapping::stop_replay_bag] && playingBack)
            {
                ROS_INFO("Received the Opening rosbag command from the joystick for playback");
                newPlayingBack = true;
                playingBack = false;
            }
            return;
        }

        // handle joysitck movement commands
        geometry_msgs::Twist spd;
        spd.linear.x = lin_max * msg.axes[axis_mapping::linear_movement];
        spd.angular.z = ang_max * msg.axes[axis_mapping::rotation_movement];
        {
            std::lock_guard<std::mutex> joyLock(joyVelMutex);
            joyVel = spd;
            newJoyVel = true;
        }
        return;
    }

    void ControlRobot::navigationCB(const geometry_msgs::Twist &msg)
    {
        std::lock_guard<std::mutex> navigationLock(navigationVelMutex);
        navigationVel = msg;
        newNavigationVel = true;
    }

} // end namespace tbd_podi_common
