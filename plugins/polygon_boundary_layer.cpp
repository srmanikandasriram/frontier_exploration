#include <frontier_exploration/polygon_boundary_layer.h>
#include <pluginlib/class_list_macros.h>

namespace frontier_exploration
{

  using costmap_2d::LETHAL_OBSTACLE;
  using costmap_2d::NO_INFORMATION;
  using costmap_2d::FREE_SPACE;

  PolygonBoundaryLayer::PolygonBoundaryLayer() : tf2_listener_(tf2_)
  {
  }

  PolygonBoundaryLayer::~PolygonBoundaryLayer()
  {
    polygonService_.shutdown();
    delete dsrv_;
    dsrv_ = 0;
  }

  void PolygonBoundaryLayer::onInitialize()
  {

    ros::NodeHandle nh_("~/" + name_);
    configured_ = false;

    bool explore_clear_space;
    nh_.param("explore_clear_space", explore_clear_space, true);
    if (explore_clear_space)
    {
      default_value_ = NO_INFORMATION;
    }
    else
    {
      default_value_ = FREE_SPACE;
    }

    matchSize();

    nh_.param<bool>("resize_to_boundary", resize_to_boundary_, true);
    polygonService_ = nh_.advertiseService("set_polygon_boundary", &PolygonBoundaryLayer::setPolygonBoundaryCb, this);

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh_);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
        &PolygonBoundaryLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

  }

  void PolygonBoundaryLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
  }


  bool PolygonBoundaryLayer::setPolygonBoundaryCb(frontier_exploration::SetPolygonBoundary::Request &req, frontier_exploration::SetPolygonBoundary::Response &res)
  {

    geometry_msgs::PolygonStamped &boundary_in = req.polygon_boundary;

    // clear existing boundary, if any
    boundary_.polygon.points.clear();

    // if empty boundary provided, all we need to do is clear
    if (boundary_in.polygon.points.empty())
    {
      return true;
    }

    try
    {
      tf2_.transform(boundary_in, boundary_, layered_costmap_->getGlobalFrameID(), ros::Duration(10.0));
    }
    catch (tf2::TransformException &ex)
    {
      //TODO
    }

    if (resize_to_boundary_)
    {
      updateOrigin(0, 0);

      //Find map size and origin by finding min/max points of polygon
      float min_x, min_y;
      min_x = min_y = std::numeric_limits<float>::infinity();
      float max_x, max_y;
      max_x = max_y = -std::numeric_limits<float>::infinity();

      for (std::vector<geometry_msgs::Point32>::iterator it = boundary_.polygon.points.begin(); it != boundary_.polygon.points.end(); ++it)
      {
        min_x = std::min(min_x, it->x);
        min_y = std::min(min_y, it->y);
        max_x = std::max(max_x, it->x);
        max_y = std::max(max_y, it->y);
      }

      //resize the costmap to polygon boundaries, don't change resolution
      int size_x, size_y;
      worldToMapNoBounds(max_x - min_x, max_y - min_y, size_x, size_y);
      layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x, min_y);
      matchSize();
    }

    configured_ = true;
    return true;
  }

  void PolygonBoundaryLayer::reset()
  {
    //reset costmap_ char array to default values
    configured_ = false;
    memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
  }


  void PolygonBoundaryLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                          double *min_y, double *max_x, double *max_y)
  {

    //check if layer is enabled and configured with a boundary
    if (!enabled_ || !configured_)
    {return;}

    //update the whole costmap
    *min_x = getOriginX();
    *min_y = getOriginY();
    *max_x = getSizeInMetersX() + getOriginX();
    *max_y = getSizeInMetersY() + getOriginY();

  }

  void PolygonBoundaryLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
  {
    //check if layer is enabled and configured with a boundary
    if (!enabled_ || !configured_)
    {return;}

    //draw lines between each point in polygon
    MarkCell marker(costmap_, LETHAL_OBSTACLE);

    //circular iterator
    for (int i = 0, j = boundary_.polygon.points.size() - 1; i < boundary_.polygon.points.size(); j = i++)
    {

      int x_1, y_1, x_2, y_2;
      worldToMapEnforceBounds(boundary_.polygon.points[i].x, boundary_.polygon.points[i].y, x_1, y_1);
      worldToMapEnforceBounds(boundary_.polygon.points[j].x, boundary_.polygon.points[j].y, x_2, y_2);

      raytraceLine(marker, x_1, y_1, x_2, y_2);
    }
    //update the master grid from the internal costmap
//        updateWithMax(master_grid, min_i, min_j, max_i, max_j);


  }

//    void BoundedExploreLayer::mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
//        if (!enabled_)
//            return;
//
//        unsigned char* master = master_grid.getCharMap();
//        unsigned int span = master_grid.getSizeInCellsX();
//
//        for (int j = min_j; j < max_j; j++)
//        {
//            unsigned int it = span*j+min_i;
//            for (int i = min_i; i < max_i; i++)
//            {
//                //only update master grid if local costmap cell is lethal/higher value, and is not overwriting a lethal obstacle in the master grid
//                if(master[it] != LETHAL_OBSTACLE && (costmap_[it] == LETHAL_OBSTACLE || costmap_[it] > master[it])){
//                    master[it] = costmap_[it];
//                }
//                it++;
//            }
//        }
//        marked_ = true;
//    }
}

PLUGINLIB_EXPORT_CLASS(frontier_exploration::PolygonBoundaryLayer, costmap_2d::Layer)