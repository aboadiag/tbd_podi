
#include <tbd_podi_2dnav/interaction_space_layer.h>
#include <costmap_2d/costmap_math.h>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_EXPORT_CLASS(tbd_costmap::InteractionSpaceLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace tbd_costmap
{
    InteractionSpaceLayer::InteractionSpaceLayer()
    {
        costmap_ = NULL;
    }

    void InteractionSpaceLayer::onInitialize()
    {
        ros::NodeHandle n;
        ros::NodeHandle nh("~/" + name_);

        //initialize it
        InteractionSpaceLayer::setDefaultValue(costmap_2d::FREE_SPACE);
        InteractionSpaceLayer::matchSize();
        //enable the map
        enabled_ = true;

        // get the properities
        nh.param("topic", topicName_, std::string("/interaction_space"));
        nh.param("ignore_time_stamp", ignoreTimeStamp_, false);
        nh.param("observation_keep_time", keepTimeSec_, 1.0);

        // subscribe to the humans topic
        spaceSub_ = n.subscribe(topicName_, 1, &InteractionSpaceLayer::InteractionSpaceCB, this);
    }

    InteractionSpaceLayer::~InteractionSpaceLayer()
    {
    }

    void InteractionSpaceLayer::registerPolygonList(std::vector<std::vector<geometry_msgs::Point>> &polygonList, unsigned char cost, double *min_x, double *min_y, double *max_x, double *max_y)
    {
        for(const auto &polygon : polygonList)
        {
            for(const auto &point : polygon)
            {
                *min_x = std::min(point.x, *min_x);
                *min_y = std::min(point.y, *min_y);
                *max_x = std::max(point.x, *max_x);
                *max_y = std::max(point.y, *max_y);
            }
            setConvexPolygonCost(polygon, cost);
        }
    }

    void InteractionSpaceLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                  double *max_x, double *max_y)
    {
        // update the bounds and polygons
        if (previousPolygons_.size() > 0)
        {
            registerPolygonList(previousPolygons_, costmap_2d::FREE_SPACE, min_x, min_y, max_x, max_y);
        }
        if (ignoreTimeStamp_ || (lastMsgTime_ + ros::Duration(keepTimeSec_)) > ros::Time::now())
        {
            registerPolygonList(latestPolygons_, costmap_2d::LETHAL_OBSTACLE, min_x, min_y, max_x, max_y);
            previousPolygons_ = latestPolygons_;
        }
        else
        {
            previousPolygons_.clear();
        }
    }

    void InteractionSpaceLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        this->updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    }

    void InteractionSpaceLayer::activate()
    {
    }
    void InteractionSpaceLayer::deactivate()
    {
    }
    void InteractionSpaceLayer::reset()
    {
    }

    void InteractionSpaceLayer::InteractionSpaceCB(const tbd_ros_msgs::InteractionSpaceArray &msg)
    {
        if (msg.spaces.size() > 0)
        {
            geometry_msgs::TransformStamped transform;
            try
            {
                // look up a transformation
                if (ignoreTimeStamp_)
                {
                    transform = tf_->lookupTransform("podi_map", msg.spaces[0].header.frame_id, msg.spaces[0].header.stamp);
                }
                else
                {
                    transform = tf_->lookupTransform("podi_map", msg.spaces[0].header.frame_id, ros::Time(0));
                }
                // we have a list of polygons
                std::vector<std::vector<geometry_msgs::Point>> polygonList;

                // now work with each spaces
                for (const auto &space : msg.spaces)
                {

                    geometry_msgs::Point transformedCenter;
                    tf2::doTransform(space.center, transformedCenter, transform);

                    // we construct a polygon here based on the information
                    std::vector<geometry_msgs::Point> polygon;
                    polygon.push_back(transformedCenter);

                    // base on the information, we make multiple polygons
                    for(const auto &member: space.members)
                    {
                        geometry_msgs::Point transformedPoint;
                        tf2::doTransform(member, transformedPoint, transform);

                        //TODO: implement a line algorithm here.

                        // for now, we use just the two points
                        polygon.push_back(transformedPoint);
                    }
                    // add to polygone
                    polygonList.push_back(polygon);
                }
                latestPolygons_ = polygonList;
                lastMsgTime_ = msg.spaces[0].header.stamp;
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
    }

} //namespace costmap_2d