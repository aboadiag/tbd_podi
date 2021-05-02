
#include <tbd_podi_2dnav/human_layer.h>
#include <costmap_2d/costmap_math.h>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(tbd_costmap::HumanLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace tbd_costmap
{
    HumanLayer::HumanLayer()
        : dsrv_(NULL)
    {
        costmap_ = NULL;

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
        nh.param("inflation", inflation_, 0.25);
        nh.param("ignore_time_stamp", ignoreTimeStamp_, false);
        nh.param("observation_keep_time", keepTimeSec_, 1.0);
        nh.param("enabled", enabled_, true);

        // get information about the cost map
        operating_frame_id_ = layered_costmap_->getGlobalFrameID();

        // subscribe to the humans topic
        humansSub_ = n.subscribe(topicName_, 1, &HumanLayer::HumansCB, this);

        // start dynamic reconfigure
        // modelled after from https://github.com/ros-planning/navigation
        if (dsrv_)
        {
            delete dsrv_;
        }
        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
            &HumanLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void HumanLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
        if (config.enabled != enabled_)
        {
            enabled_ = config.enabled;
        }
    }

    std::vector<geometry_msgs::Point> HumanLayer::constructPolygons(geometry_msgs::Point point, double *min_x, double *min_y,
                                                                    double *max_x, double *max_y)
    {
        std::vector<geometry_msgs::Point> polygon;
        geometry_msgs::Point topPoint(point);
        topPoint.y += inflation_;
        polygon.push_back(topPoint);
        geometry_msgs::Point leftPoint(point);
        leftPoint.x += inflation_;
        polygon.push_back(leftPoint);
        geometry_msgs::Point bottomPoint(point);
        bottomPoint.y -= inflation_;
        polygon.push_back(bottomPoint);
        geometry_msgs::Point rightPoint(point);
        rightPoint.x -= inflation_;
        polygon.push_back(rightPoint);

        *min_x = std::min(point.x - inflation_, *min_x);
        *min_y = std::min(point.y - inflation_, *min_y);
        *max_x = std::max(point.x + inflation_, *max_x);
        *max_y = std::max(point.y + inflation_, *max_y);
        return polygon;
    }

    void HumanLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                  double *max_x, double *max_y)
    {
        // clear the previous polygons
        if (previousPoints_.size() > 0)
        {
            for (const auto &point: previousPoints_)
            {
                auto polygon = constructPolygons(point, min_x, min_y, max_x, max_y);
                setConvexPolygonCost(polygon, costmap_2d::FREE_SPACE);
            }
        }

        if ( ignoreTimeStamp_ || (lastMsgTime_ + ros::Duration(keepTimeSec_)) > ros::Time::now())
        {
            for (const auto &point : latestPoints_)
            {
                // update the internel costmap
                // using the center point, create a polygon
                auto polygon = constructPolygons(point, min_x, min_y, max_x, max_y);
                setConvexPolygonCost(polygon, costmap_2d::LETHAL_OBSTACLE);
            }
            previousPoints_ = latestPoints_;
        }
        else
        {
            previousPoints_.clear();
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
                for (const auto &body : msg.bodies)
                {
                    for (const auto &joint : body.joints)
                    {
                        // use pelvis as the center
                        if (joint.joint_id == joint.JOINT_PELVIS)
                        {
                            auto pelvisPoint = joint.pose.position;
                            // try to transform it
                            geometry_msgs::Point transformedPoint;
                            tf2::doTransform(pelvisPoint, transformedPoint, transform);
                            latestPoints_.push_back(transformedPoint);
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
        }
    }

} //namespace costmap_2d