
#include <tbd_podi_2dnav/interaction_space_layer.h>
#include <costmap_2d/costmap_math.h>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <math.h>
#include <geometry_msgs/Point.h>

PLUGINLIB_EXPORT_CLASS(tbd_costmap::InteractionSpaceLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace tbd_costmap
{
    InteractionSpaceLayer::InteractionSpaceLayer()
        : dsrv_(NULL)
    {
        costmap_ = NULL;
    }

    InteractionSpaceLayer::~InteractionSpaceLayer()
    {
        if (dsrv_)
        {
            delete dsrv_;
        }
    }

    void InteractionSpaceLayer::onInitialize()
    {
        ros::NodeHandle n;
        ros::NodeHandle nh("~/" + name_);

        //initialize it
        InteractionSpaceLayer::setDefaultValue(costmap_2d::FREE_SPACE);
        InteractionSpaceLayer::matchSize();
        //enable the map
        current_ = true;
        rollingWindow_ = layered_costmap_->isRolling();

        // get the properities
        nh.param("topic", topicName_, std::string("/interaction_space"));
        nh.param("ignore_time_stamp", ignoreTimeStamp_, false);
        nh.param("observation_keep_time", keepTimeSec_, 1.0);
        nh.param("enabled", enabled_, true);
        nh.param("human_slack", humanSlack_, 0.0);
        nh.param("robot_slack", robotSlack_, 0.3);
        nh.param("slack_cost", slackCost_, 100);

        // get information about the cost map
        operating_frame_id_ = layered_costmap_->getGlobalFrameID();

        // subscribe to the humans topic
        spaceSub_ = n.subscribe(topicName_, 1, &InteractionSpaceLayer::InteractionSpaceCB, this);

        // start dynamic reconfigure
        // modelled after from https://github.com/ros-planning/navigation
        if (dsrv_)
        {
            delete dsrv_;
        }
        dsrv_ = new dynamic_reconfigure::Server<tbd_podi_2dnav::InteractionLayerPluginConfig>(nh);
        dynamic_reconfigure::Server<tbd_podi_2dnav::InteractionLayerPluginConfig>::CallbackType cb = boost::bind(
            &InteractionSpaceLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void InteractionSpaceLayer::reconfigureCB(tbd_podi_2dnav::InteractionLayerPluginConfig &config, uint32_t level)
    {
        if (config.enabled != enabled_)
        {
            enabled_ = config.enabled;
        }
        if (config.robot_slack != robotSlack_)
        {
            robotSlack_ = config.robot_slack;
        }
        if (config.human_slack != humanSlack_)
        {
            humanSlack_ = config.human_slack;
        }
        if (config.slack_cost != slackCost_)
        {
            slackCost_ = config.slack_cost;
        }
    }

    void InteractionSpaceLayer::registerPolygonList(std::vector<std::vector<geometry_msgs::Point>> &polygonList, unsigned char cost, double *min_x, double *min_y, double *max_x, double *max_y)
    {
        for (auto &polygon : polygonList)
        {
            registerPolygon(polygon, cost, min_x, min_y, max_x, max_y);
        }
    }

    void InteractionSpaceLayer::registerPolygon(std::vector<geometry_msgs::Point> &polygon, unsigned char cost, double *min_x, double *min_y, double *max_x, double *max_y)
    {
        for (const auto &point : polygon)
        {
            *min_x = std::min(point.x, *min_x);
            *min_y = std::min(point.y, *min_y);
            *max_x = std::max(point.x, *max_x);
            *max_y = std::max(point.y, *max_y);
        }
        setConvexPolygonCost(polygon, cost);
    }

    void InteractionSpaceLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                             double *max_x, double *max_y)
    {
        // update origin if its rolling map
        if (rollingWindow_)
        {
            updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
        }
        // clear the previous polygons
        while (previousPolygons_.size() > 0)
        {
            registerPolygon(previousPolygons_[previousPolygons_.size() - 1], costmap_2d::FREE_SPACE, min_x, min_y, max_x, max_y);
            previousPolygons_.pop_back();
        }

        // add the interaction cost
        if (ignoreTimeStamp_ || (lastMsgTime_ + ros::Duration(keepTimeSec_)) > ros::Time::now())
        {
            registerPolygonList(latestPolygons_, (unsigned char)225, min_x, min_y, max_x, max_y);
            previousPolygons_ = latestPolygons_;
        }
    }

    void InteractionSpaceLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        this->updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    }

    void InteractionSpaceLayer::activate()
    {
        onInitialize();
    }

    void InteractionSpaceLayer::deactivate()
    {
        spaceSub_.shutdown();
    }

    void InteractionSpaceLayer::reset()
    {
        deactivate();
        //clear the whole map
        resetMaps();
        activate();
    }

    geometry_msgs::Point PointWithSlack(geometry_msgs::Point &center, geometry_msgs::Point &point, double slack)
    {
        // calculate the distance between the point and center
        double dist = sqrt(pow(center.x - point.x, 2) + pow(center.y - point.y, 2) + pow(center.z - point.z, 2));
        // the maximum allowed slack is the distance
        slack = std::min(slack, dist);

        geometry_msgs::Point toCenterVec(center);
        toCenterVec.x -= point.x;
        toCenterVec.y -= point.y;
        toCenterVec.z -= point.z;
        // normalized the vector
        double vecSum = abs(toCenterVec.x) + abs(toCenterVec.y) + abs(toCenterVec.z);
        toCenterVec.x /= vecSum;
        toCenterVec.y /= vecSum;
        toCenterVec.z /= vecSum;
        // now we add the slack
        toCenterVec.x *= slack;
        toCenterVec.y *= slack;
        toCenterVec.z *= slack;
        geometry_msgs::Point newPoint(point);
        newPoint.x += toCenterVec.x;
        newPoint.y += toCenterVec.y;
        newPoint.z += toCenterVec.z;
        return newPoint;
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
                    transform = tf_->lookupTransform(operating_frame_id_, msg.spaces[0].header.frame_id, msg.spaces[0].header.stamp);
                }
                else
                {
                    transform = tf_->lookupTransform(operating_frame_id_, msg.spaces[0].header.frame_id, ros::Time(0));
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

                    // base on the information, we make multiple polygons
                    for (const auto &member : space.human_members)
                    {
                        geometry_msgs::Point transformedPoint;
                        tf2::doTransform(member, transformedPoint, transform);
                        // now we add slack into the system
                        auto slackedPoint = PointWithSlack(transformedCenter, transformedPoint, humanSlack_);
                        polygon.push_back(slackedPoint);
                    }
                    // create agent
                    for (const auto &member : space.agent_members)
                    {
                        geometry_msgs::Point transformedPoint;
                        tf2::doTransform(member, transformedPoint, transform);
                        // now we add slack into the system
                        auto slackedPoint = PointWithSlack(transformedCenter, transformedPoint, robotSlack_);
                        polygon.push_back(slackedPoint);
                    }

                    // if there is only two points, we enlarge the space
                    //the polygon has size of three because of the center.
                    if (polygon.size() == 2)
                    {
                        // create the line's normal
                        auto lineX = polygon[1].x - polygon[0].x;
                        auto lineY = polygon[1].y - polygon[0].y;
                        auto length = sqrt(lineX * lineX + lineY * lineY);
                        lineX = lineX / length;
                        lineY = lineY / length;
                        // now we can create the line
                        double width = 0.25;
                        std::vector<geometry_msgs::Point> newPolygon;

                        geometry_msgs::Point p1, p2, p3, p4;
                        p1.x = polygon[0].x - lineY * width;
                        p1.y = polygon[0].y + lineX * width;
                        p2.x = polygon[0].x + lineY * width;
                        p2.y = polygon[0].y - lineX * width;
                        p3.x = polygon[1].x + lineY * width;
                        p3.y = polygon[1].y - lineX * width;
                        p4.x = polygon[1].x - lineY * width;
                        p4.y = polygon[1].y + lineX * width;
                        newPolygon.push_back(p1);
                        newPolygon.push_back(p2);
                        newPolygon.push_back(p3);
                        newPolygon.push_back(p4);
                        polygon = newPolygon;
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