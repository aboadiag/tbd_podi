#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

class LaserScanFrameTransform
{
  ros::NodeHandle nh1;
  ros::NodeHandle nh2;
  std::string newLaserFrame;
  std::string newLaserTopic;
  ros::Subscriber laserTopicInput;
  ros::Publisher laserTopicOutput;
  double minAngle;
  double maxAngle;
  public:
    LaserScanFrameTransform();
    ~LaserScanFrameTransform();
    void laserMessageReceived(const sensor_msgs::LaserScan& lsmsg);
};

LaserScanFrameTransform::LaserScanFrameTransform() {
  newLaserFrame = "base_laser";
  newLaserTopic = "laser";
  laserTopicInput = nh1.subscribe("RosAria/lms1xx_1_laserscan", 1000, &LaserScanFrameTransform::laserMessageReceived, this);
  laserTopicOutput = nh2.advertise<sensor_msgs::LaserScan>(newLaserTopic, 1000);
  // set the limit of the sensor
  minAngle = -1.0 * M_PI * 5.0/8.0;
  maxAngle = M_PI * 5.0/8.0;
}

LaserScanFrameTransform::~LaserScanFrameTransform() {
  return;
}

void LaserScanFrameTransform::laserMessageReceived(const sensor_msgs::LaserScan& msg) {
  // Modify msg's header frame_id
  sensor_msgs::LaserScan newMsg = msg;
  newMsg.header.frame_id = newLaserFrame;

  // lower the range from [-3pi/4, 3pi/4] to [-pi/2, pi/2]
  std::vector<float> newRanges;
  double angle = msg.angle_min;  // rad
  // Initial distance-based clustering of lines
  for (int i = 0; i < msg.ranges.size(); i++) {
    // ROS_INFO("angle: %f", angle);
    if (angle > minAngle && angle < maxAngle) {
      newRanges.push_back(msg.ranges[i]);
    }
    angle = angle + msg.angle_increment;
  }
  newMsg.angle_min = minAngle;
  newMsg.angle_max = maxAngle;
  newMsg.ranges = newRanges;

  laserTopicOutput.publish(newMsg);
  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_scan_frame_transform");
  LaserScanFrameTransform lft;
  ros::spin();
  return 0;
}
