#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class LaserScanFrameTransform
{
  ros::NodeHandle nh;
  std::string newLaserFrame;
  std::string newLaserTopic;
  std::string inputLaserTopic;
  ros::Subscriber laserTopicInput;
  ros::Publisher laserTopicOutput;
  double minAngle;
  double maxAngle;

public:
  LaserScanFrameTransform();
  ~LaserScanFrameTransform();
  void laserMessageReceived(const sensor_msgs::LaserScan& lsmsg);
};

LaserScanFrameTransform::LaserScanFrameTransform()
{
  // get the parameters and set to default if not given
  nh.param<std::string>("output_frame_id", newLaserFrame, "");
  nh.param<double>("min_angle", minAngle, -1.0 * M_PI * 5.0 / 8.0);
  nh.param<double>("min_angle", maxAngle, M_PI * 5.0 / 8.0);

  laserTopicInput = nh.subscribe("laser", 1000, &LaserScanFrameTransform::laserMessageReceived, this);
  laserTopicOutput = nh.advertise<sensor_msgs::LaserScan>("transformed_laser", 1000);
}

LaserScanFrameTransform::~LaserScanFrameTransform()
{
  return;
}

void LaserScanFrameTransform::laserMessageReceived(const sensor_msgs::LaserScan& msg)
{
  // Modify msg's header frame_id
  sensor_msgs::LaserScan newMsg = msg;
  if (!newLaserFrame.empty())
  {
    newMsg.header.frame_id = newLaserFrame;
  }

  // lower the range from [-3pi/4, 3pi/4] to [-pi/2, pi/2]
  std::vector<float> newRanges;
  std::vector<float> newIntensities;
  double angle = msg.angle_min;  // rad
  // Initial distance-based clustering of lines
  // TODO: Check if the order is different in real life.
  // for (int i = msg.ranges.size() - 1; i >= 0; i--)
  for (int i = 0; i < msg.ranges.size(); i++)
  {
    // ROS_INFO("angle: %f", angle);
    if (angle > minAngle && angle < maxAngle)
    {
      newRanges.push_back(msg.ranges[i]);
      if (msg.intensities.size() > i){
        newIntensities.push_back(msg.intensities[i]);
      }
    }
    
    angle = angle + msg.angle_increment;
  }

  newMsg.angle_min = minAngle;
  newMsg.angle_max = maxAngle;
  newMsg.ranges = newRanges;
  newMsg.intensities = newIntensities;

  laserTopicOutput.publish(newMsg);
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_scan_frame_transform");
  LaserScanFrameTransform lft;
  ros::spin();
  return 0;
}
