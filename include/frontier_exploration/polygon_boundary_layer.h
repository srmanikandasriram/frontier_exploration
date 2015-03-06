#ifndef BOUNDED_EXPLORE_LAYER_H_
#define BOUNDED_EXPLORE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/PolygonStamped.h>
#include <frontier_exploration/SetPolygonBoundary.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace frontier_exploration
{

  /**
  * @brief costmap_2d layer plugin that holds the state for a bounded frontier exploration task.
  * Manages the boundary polygon, superimposes the polygon on the overall exploration costmap,
  * and processes costmap to find next frontier to explore.
  */
  class PolygonBoundaryLayer : public costmap_2d::CostmapLayer
  {
  public:
    PolygonBoundaryLayer();

    ~PolygonBoundaryLayer();

    /**
    * @brief Loads default values and initialize exploration costmap.
    */
    virtual void onInitialize();

    /**
    * @brief Calculate bounds of costmap window to update
    */
    virtual void updateBounds(double origin_x, double origin_y, double origin_yaw, double *polygon_min_x, double *polygon_min_y, double *polygon_max_x,
                              double *polygon_max_y);

    /**
    * @brief Update requested costmap window
    */
    virtual void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

    /**
    * @brief Reset exploration progress
    */
    virtual void reset();

  protected:

    /**
    * @brief Load polygon boundary to draw on map with each update
    * @param req Service request
    * @param res Service response
    * @return True on service success, false otherwise
    */
    bool setPolygonBoundaryCb(frontier_exploration::SetPolygonBoundary::Request &req, frontier_exploration::SetPolygonBoundary::Response &res);

  private:

    /**
    * @brief Update the map with exploration boundary data
    * @param master_grid Reference to master costmap
    */
//    void mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

    ros::ServiceServer polygonService_;
    geometry_msgs::PolygonStamped boundary_;
    tf2_ros::Buffer tf2_;
    tf2_ros::TransformListener tf2_listener_;

    bool configured_;
    bool resize_to_boundary_;

  };

}
#endif
