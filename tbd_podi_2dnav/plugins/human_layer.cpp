
#include <tbd_podi_2dnav/human_layer.h>
#include <costmap_2d/costmap_math.h>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <visualization_msgs/Marker.h>

PLUGINLIB_EXPORT_CLASS(tbd_costmap::HumanLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace tbd_costmap
{
    HumanLayer::HumanLayer()
        : dsrv_(NULL)
    {
        costmap_ = NULL;

        // DEBUG create a human pose marker pubisher for debugging
        vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    }

    HumanLayer::~HumanLayer()
    {
        if (dsrv_)
        {
            delete dsrv_;
        }
    }

    void HumanLayer::onInitialize()
    {
        ros::NodeHandle n;
        ros::NodeHandle nh("~/" + name_);

        //initialize it
        HumanLayer::setDefaultValue(costmap_2d::FREE_SPACE);
        HumanLayer::matchSize();
        //enable the map

        current_ = true;

        // get the properities
        nh.param("topic", topicName_, std::string("/humans"));
        nh.param("inflation", inflation_, 0.5);
        nh.param("ellipse_ratio", ellipse_ratio_, 2.0);
        nh.param("ellipse_offset_ratio", ellipse_offset_ratio_, 3.0);
        nh.param("ignore_time_stamp", ignoreTimeStamp_, false);
        nh.param("observation_keep_time", keepTimeSec_, 1.0);
        nh.param("enabled", enabled_, true);

        // get information about the cost map
        operating_frame_id_ = layered_costmap_->getGlobalFrameID();
        rollingWindow_ = layered_costmap_->isRolling();

        // subscribe to the humans topic
        humansSub_ = n.subscribe(topicName_, 1, &HumanLayer::HumansCB, this);

        // start dynamic reconfigure
        // modelled after from https://github.com/ros-planning/navigation
        if (dsrv_)
        {
            delete dsrv_;
        }
        dsrv_ = new dynamic_reconfigure::Server<tbd_podi_2dnav::HumanLayerPluginConfig>(nh);
        dynamic_reconfigure::Server<tbd_podi_2dnav::HumanLayerPluginConfig>::CallbackType cb = boost::bind(
            &HumanLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void HumanLayer::reconfigureCB(tbd_podi_2dnav::HumanLayerPluginConfig &config, uint32_t level)
    {
        if (config.enabled != enabled_)
        {
            enabled_ = config.enabled;
        }
        if (config.inflation != inflation_)
        {
            inflation_ = config.inflation;
        }
        if (config.ellipse_ratio != ellipse_ratio_)
        {
            ellipse_ratio_ = config.ellipse_ratio;
        }
        if (config.ellipse_offset_ratio != ellipse_offset_ratio_)
        {
            ellipse_offset_ratio_ = config.ellipse_offset_ratio;
        }

    }

    std::vector<geometry_msgs::Point> HumanLayer::constructPolygon(geometry_msgs::Point point, geometry_msgs::Quaternion orient, double vel, double *min_x, double *min_y,
                                                                    double *max_x, double *max_y)
    {
        // the polygon around the human to be returned
        std::vector<geometry_msgs::Point> polygon;

        // get RPY from quaternion
        // convert geometry_msgs quat to tf2 quat to get angles
        tf2::Quaternion quat_tf;
        tf2::convert(orient, quat_tf);

        // convert the input quarernions into RPY
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

        // set the reference angle
        // yaw -=  M_PI;

        // find the min and max of the points in the world frame.
        double xminb, xmaxb, ymaxb, yminb;
        mapToWorld(0,0,xminb, yminb);
        mapToWorld(size_x_-1,size_y_-1,xmaxb, ymaxb);
        
        bool valid = false;

        // loop and get 8 points for a ellipse type polygon
        for (int i = 0; i < 8; i++)
        {
            geometry_msgs::Point curr_point(point);
            geometry_msgs::Point boundedPoint(point);

            double curr_angle = 2 * M_PI * (i / 8.0);

            double a, b;

            // major and minor diameters for the ellipse

            b = (inflation_ / ellipse_ratio_);
            a = inflation_ + vel; // changes based on velocity

            // ellipse offset to that their is more space in front of the person
            double offset = (a / ellipse_offset_ratio_);

            // gets x and y for an ellipse, the last term is for the rotation at an offset point (found graphically using desmos)
            // these are the parametric equations for the ellipse around the person
            // you can plug these into desmos to see them in action
            curr_point.x += a * cos(curr_angle) * cos(yaw) - b * sin(curr_angle) * sin(yaw) - offset * cos(yaw + (M_PI));
            curr_point.y += a * cos(curr_angle) * sin(yaw) + b * sin(curr_angle) * cos(yaw) - offset * sin(yaw + (M_PI));

            // we want to bound it by the map size, if not the create polygon function won't work
            boundedPoint.x = std::max(std::min(curr_point.x, xmaxb), xminb);
            boundedPoint.y = std::max(std::min(curr_point.y, ymaxb), yminb);

            valid = !(boundedPoint.x == xmaxb || boundedPoint.x == xminb || boundedPoint.y == ymaxb || boundedPoint.y == yminb);

            // check the area to be reloaded
            *min_x = std::min(boundedPoint.x, *min_x);
            *min_y = std::min(boundedPoint.y, *min_y);
            *max_x = std::max(boundedPoint.x, *max_x);
            *max_y = std::max(boundedPoint.y, *max_y);

            // add to the polygon
            polygon.push_back(boundedPoint);
        }

        if (!valid)
        {
            polygon.clear();
        }
        return polygon;
    }

    void HumanLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                  double *max_x, double *max_y)
    {

        // clear the previous generated polygon from the map
        while (previousPolygons_.size() > 0)
        {
            // get the last polygon and clear it's space
            auto lastPolygon = previousPolygons_[previousPolygons_.size() - 1];
            setConvexPolygonCost(lastPolygon, costmap_2d::FREE_SPACE);
            previousPolygons_.pop_back();
        }

        // update origin if its rolling map
        if (rollingWindow_)
        {
            updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
        }

        // lock this method
        std::lock_guard<std::mutex> operationLock(operationMutex_);

        if (ignoreTimeStamp_ || (lastMsgTime_ + ros::Duration(keepTimeSec_)) > ros::Time::now())
        {
            // loop thorugh all of the points
            for (auto &item : latestPoints_)
            {
                auto body_id = item.first;
                auto orient = latestOrients_[body_id];
                auto point = item.second;

                // try and calculate the velocity
                double vel = 0.0;
                if (previousPoints_.find(body_id) != previousPoints_.end())
                {
                    auto prevPoint = previousPoints_[body_id];

                    double dx = std::abs(point.x - prevPoint.x);
                    double dy = std::abs(point.y - prevPoint.y);
                    vel = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

                    latestVelocities_[body_id] = vel;
                }

                // update the internel costmap
                // using the center point, create a polygon
                auto polygon = constructPolygon(point, orient, vel, min_x, min_y, max_x, max_y);
                setConvexPolygonCost(polygon, costmap_2d::LETHAL_OBSTACLE);
                // we directly save the previous polygon because information about the shape such 
                // as inflation or future implementation might change our ability to recreate the points
                // while its waste more space, it causes less error.
                previousPolygons_.push_back(polygon);
            }
            previousPoints_ = latestPoints_;
            previousOrients_ = latestOrients_;
            previousVelocities_ = latestVelocities_;
        }
        else
        {
            previousPoints_.clear();
            previousOrients_.clear();
            previousVelocities_.clear();
        }
    }
    void HumanLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        this->updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    }

    void HumanLayer::activate()
    {
        onInitialize();
    }
    void HumanLayer::deactivate()
    {
        humansSub_.shutdown();
    }
    void HumanLayer::reset()
    {
        deactivate();
        //clear the whole map
        resetMaps();
        activate();
    }

    void HumanLayer::HumansCB(const tbd_ros_msgs::HumanBodyArray &msg)
    {
        // lock this method
        std::lock_guard<std::mutex> operationLock(operationMutex_);
        if (msg.bodies.size() > 0)
        {
            geometry_msgs::TransformStamped transform;
            try
            {
                // look up a transformation
                if (ignoreTimeStamp_)
                {
                    transform = tf_->lookupTransform(operating_frame_id_, msg.bodies[0].header.frame_id, msg.bodies[0].header.stamp);
                }
                else
                {
                    transform = tf_->lookupTransform(operating_frame_id_, msg.bodies[0].header.frame_id, ros::Time(0));
                }
                // collect the data
                latestPoints_.clear();
                latestOrients_.clear();
                latestVelocities_.clear();
                for (const auto &body : msg.bodies)
                {

                    // std::cout << body.body_id << std::endl;
                    auto body_id = body.body_id;

                    for (const auto &joint : body.joints)
                    {
                        // use pelvis as the center
                        if (joint.joint_id == joint.JOINT_PELVIS)
                        {
                            auto pelvisPoint = joint.pose.position;
                            auto pelvisOrient = joint.pose.orientation; // get the orientation of the person
                            // try to transform it
                            geometry_msgs::Point transformedPoint;
                            tf2::doTransform(pelvisPoint, transformedPoint, transform);

                            // std::cout << "here" << std::endl;
                            latestPoints_[body_id] = transformedPoint;
                            latestOrients_[body_id] = pelvisOrient;
                            // std::cout << "there" << std::endl;

                            // DEBUG publish the human position marker
                            visualization_msgs::Marker marker;
                            marker.header.frame_id = "PsiWorld";
                            marker.header.stamp = ros::Time();
                            marker.ns = "my_namespace";
                            marker.id = 0;
                            marker.type = visualization_msgs::Marker::ARROW;
                            marker.action = visualization_msgs::Marker::ADD;
                            marker.pose.position.x = transformedPoint.x;
                            marker.pose.position.y = transformedPoint.y;
                            marker.pose.position.z = 0;
                            marker.pose.orientation = pelvisOrient;
                            marker.scale.x = 1;
                            marker.scale.y = 0.1;
                            marker.scale.z = 0.1;
                            marker.color.a = 1.0; // Don't forget to set the alpha!
                            marker.color.r = 0.0;
                            marker.color.g = 1.0;
                            marker.color.b = 0.0;
                            vis_pub.publish( marker );

                            break;
                        }
                    }
                }
                lastMsgTime_ = msg.bodies[0].header.stamp;
            }
            catch (tf2::LookupException &ex)
            {
                ROS_ERROR("No Transform %s\n", ex.what());
            }
            catch (tf2::ExtrapolationException &ex)
            {
                ROS_ERROR("Extrapolation: %s\n", ex.what());
            }
        }
        else
        {
            latestPoints_.clear();
            latestOrients_.clear();
            latestVelocities_.clear();
        }
    }

} //namespace costmap_2d