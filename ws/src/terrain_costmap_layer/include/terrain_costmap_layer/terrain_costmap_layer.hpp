#ifndef TERRAIN_COSTMAP_LAYER__TERRAIN_COSTMAP_LAYER_HPP_
#define TERRAIN_COSTMAP_LAYER__TERRAIN_COSTMAP_LAYER_HPP_

#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace terrain_costmap_layer
{

class TerrainCostmapLayer : public nav2_costmap_2d::Layer
{
public:
  TerrainCostmapLayer();

  void onInitialize() override;

  void updateBounds(
    double robot_x,
    double robot_y,
    double robot_yaw,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j) override;

  void reset() override;

  bool isClearable() override;

private:
  void terrainGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  unsigned char occupancyToNav2Cost(int occ) const;

  bool getLatestGrid(nav_msgs::msg::OccupancyGrid & grid);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_grid_sub_;

  std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid latest_grid_;
  bool has_grid_;

  std::string terrain_grid_topic_;
  std::string combination_method_;

  bool enabled_;
  bool use_unknown_;
  bool scale_occupancy_to_costmap_;

  int lethal_threshold_;
  double max_grid_age_sec_;
  double transform_tolerance_;
};

}  // namespace terrain_costmap_layer

#endif  // TERRAIN_COSTMAP_LAYER__TERRAIN_COSTMAP_LAYER_HPP_