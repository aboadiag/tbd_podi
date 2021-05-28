#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/observation_buffer.h>
#include <tbd_ros_msgs/InteractionSpaceArray.h>
#include <tbd_ros_msgs/InteractionSpace.h>
#include <tf2_ros/buffer.h>
#include <vector>
#include <geometry_msgs/Point.h>
#include <tbd_podi_2dnav/InteractionLayerPluginConfig.h>
#include <dynamic_reconfigure/server.h>

namespace tbd_costmap
{

    class InteractionSpaceLayer : public costmap_2d::CostmapLayer
    {
    private:
        ros::Subscriber spaceSub_;
        std::string topicName_;
        std::string operating_frame_id_;
        double keepTimeSec_;
        bool ignoreTimeStamp_;
        bool rollingWindow_;
        ros::Time lastMsgTime_;
        double robotSlack_;
        double humanSlack_;
        int slackCost_;
        std::vector<std::vector<geometry_msgs::Point>> latestPolygons_;
        std::vector<std::vector<geometry_msgs::Point>> previousPolygons_;
        dynamic_reconfigure::Server<tbd_podi_2dnav::InteractionLayerPluginConfig> *dsrv_;

        void registerPolygon(std::vector<geometry_msgs::Point> &polygon, unsigned char cost, double *min_x, double *min_y, double *max_x, double *max_y);
        void registerPolygonList(std::vector<std::vector<geometry_msgs::Point>> &polygonList, unsigned char cost, double *min_x, double *min_y, double *max_x, double *max_y);
        void reconfigureCB(tbd_podi_2dnav::InteractionLayerPluginConfig &config, uint32_t level);

    public:
        InteractionSpaceLayer();
        virtual ~InteractionSpaceLayer();
        // Implement all the parent methods
        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y,
                                  double *max_x, double *max_y);
        virtual void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);
        virtual void activate();
        virtual void deactivate();
        virtual void reset();

        void InteractionSpaceCB(const tbd_ros_msgs::InteractionSpaceArray &msg);
    };

} // namespace tbd_costmap