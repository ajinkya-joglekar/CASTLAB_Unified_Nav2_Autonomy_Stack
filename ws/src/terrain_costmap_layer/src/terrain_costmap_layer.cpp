#include "terrain_costmap_layer/terrain_costmap_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2/exceptions.h"

namespace terrain_costmap_layer
{

TerrainCostmapLayer::TerrainCostmapLayer()
: has_grid_(false),
  enabled_(true),
  use_unknown_(false),
  scale_occupancy_to_costmap_(true),
  lethal_threshold_(100),
  max_grid_age_sec_(1.0),
  transform_tolerance_(0.2)
{
}

void TerrainCostmapLayer::onInitialize()
{
  auto node = node_.lock();

  if (!node) {
    throw std::runtime_error("TerrainCostmapLayer could not lock lifecycle node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("terrain_grid_topic", rclcpp::ParameterValue("/terrain/obstacle_grid"));
  declareParameter("combination_method", rclcpp::ParameterValue("max"));
  declareParameter("use_unknown", rclcpp::ParameterValue(false));
  declareParameter("scale_occupancy_to_costmap", rclcpp::ParameterValue(true));
  declareParameter("lethal_threshold", rclcpp::ParameterValue(100));
  declareParameter("max_grid_age_sec", rclcpp::ParameterValue(1.0));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.2));

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".terrain_grid_topic", terrain_grid_topic_);
  node->get_parameter(name_ + ".combination_method", combination_method_);
  node->get_parameter(name_ + ".use_unknown", use_unknown_);
  node->get_parameter(name_ + ".scale_occupancy_to_costmap", scale_occupancy_to_costmap_);
  node->get_parameter(name_ + ".lethal_threshold", lethal_threshold_);
  node->get_parameter(name_ + ".max_grid_age_sec", max_grid_age_sec_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);

  terrain_grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    terrain_grid_topic_,
    rclcpp::QoS(1),
    std::bind(&TerrainCostmapLayer::terrainGridCallback, this, std::placeholders::_1));

  current_ = true;

  RCLCPP_INFO(
    logger_,
    "TerrainCostmapLayer initialized. topic=%s, combination_method=%s, use_unknown=%s",
    terrain_grid_topic_.c_str(),
    combination_method_.c_str(),
    use_unknown_ ? "true" : "false");
}

void TerrainCostmapLayer::terrainGridCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_grid_ = *msg;
  has_grid_ = true;
}

bool TerrainCostmapLayer::getLatestGrid(nav_msgs::msg::OccupancyGrid & grid)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!has_grid_) {
    return false;
  }

  grid = latest_grid_;
  return true;
}

void TerrainCostmapLayer::updateBounds(
  double /*robot_x*/,
  double /*robot_y*/,
  double /*robot_yaw*/,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  if (!enabled_) {
    return;
  }

  nav_msgs::msg::OccupancyGrid grid;
  if (!getLatestGrid(grid)) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    return;
  }

  const std::string global_frame = layered_costmap_->getGlobalFrameID();

  rclcpp::Time grid_stamp(grid.header.stamp);
  rclcpp::Time now = node->now();

  if (max_grid_age_sec_ > 0.0) {
    const double age = (now - grid_stamp).seconds();
    if (age > max_grid_age_sec_) {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *node->get_clock(),
        2000,
        "Terrain grid too old: %.3f sec. Skipping bounds update.",
        age);
      return;
    }
  }

  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_->lookupTransform(
      global_frame,
      grid.header.frame_id,
      grid.header.stamp,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *node->get_clock(),
      2000,
      "Could not transform terrain grid bounds from %s to %s: %s",
      grid.header.frame_id.c_str(),
      global_frame.c_str(),
      ex.what());
    return;
  }

  const double res = grid.info.resolution;
  const double width_m = static_cast<double>(grid.info.width) * res;
  const double height_m = static_cast<double>(grid.info.height) * res;

  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const double origin_yaw = tf2::getYaw(grid.info.origin.orientation);

  const double c = std::cos(origin_yaw);
  const double s = std::sin(origin_yaw);

  std::vector<std::pair<double, double>> corners = {
    {0.0, 0.0},
    {width_m, 0.0},
    {0.0, height_m},
    {width_m, height_m}
  };

  for (const auto & corner : corners) {
    const double local_x = origin_x + c * corner.first - s * corner.second;
    const double local_y = origin_y + s * corner.first + c * corner.second;

    geometry_msgs::msg::PointStamped p_in;
    geometry_msgs::msg::PointStamped p_out;

    p_in.header = grid.header;
    p_in.point.x = local_x;
    p_in.point.y = local_y;
    p_in.point.z = 0.0;

    tf2::doTransform(p_in, p_out, transform);

    *min_x = std::min(*min_x, p_out.point.x);
    *min_y = std::min(*min_y, p_out.point.y);
    *max_x = std::max(*max_x, p_out.point.x);
    *max_y = std::max(*max_y, p_out.point.y);
  }
}

void TerrainCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  nav_msgs::msg::OccupancyGrid grid;
  if (!getLatestGrid(grid)) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    return;
  }

  const std::string global_frame = layered_costmap_->getGlobalFrameID();

  rclcpp::Time grid_stamp(grid.header.stamp);
  rclcpp::Time now = node->now();

  if (max_grid_age_sec_ > 0.0) {
    const double age = (now - grid_stamp).seconds();
    if (age > max_grid_age_sec_) {
      RCLCPP_WARN_THROTTLE(
        logger_,
        *node->get_clock(),
        2000,
        "Terrain grid too old: %.3f sec. Skipping cost update.",
        age);
      return;
    }
  }

  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_->lookupTransform(
      global_frame,
      grid.header.frame_id,
      grid.header.stamp,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *node->get_clock(),
      2000,
      "Could not transform terrain grid from %s to %s: %s",
      grid.header.frame_id.c_str(),
      global_frame.c_str(),
      ex.what());
    return;
  }

  const double res = grid.info.resolution;
  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const double origin_yaw = tf2::getYaw(grid.info.origin.orientation);

  const double c = std::cos(origin_yaw);
  const double s = std::sin(origin_yaw);

  for (unsigned int gy = 0; gy < grid.info.height; ++gy) {
    for (unsigned int gx = 0; gx < grid.info.width; ++gx) {
      const unsigned int index = gy * grid.info.width + gx;

      if (index >= grid.data.size()) {
        continue;
      }

      const int occ = static_cast<int>(grid.data[index]);

      if (occ < 0 && !use_unknown_) {
        continue;
      }

      const unsigned char terrain_cost = occupancyToNav2Cost(occ);

      const double cell_x = (static_cast<double>(gx) + 0.5) * res;
      const double cell_y = (static_cast<double>(gy) + 0.5) * res;

      const double local_x = origin_x + c * cell_x - s * cell_y;
      const double local_y = origin_y + s * cell_x + c * cell_y;

      geometry_msgs::msg::PointStamped p_in;
      geometry_msgs::msg::PointStamped p_out;

      p_in.header = grid.header;
      p_in.point.x = local_x;
      p_in.point.y = local_y;
      p_in.point.z = 0.0;

      tf2::doTransform(p_in, p_out, transform);

      unsigned int mx;
      unsigned int my;

      if (!master_grid.worldToMap(p_out.point.x, p_out.point.y, mx, my)) {
        continue;
      }

      if (
        static_cast<int>(mx) < min_i ||
        static_cast<int>(mx) >= max_i ||
        static_cast<int>(my) < min_j ||
        static_cast<int>(my) >= max_j)
      {
        continue;
      }

      const unsigned char old_cost = master_grid.getCost(mx, my);

      if (combination_method_ == "overwrite") {
        master_grid.setCost(mx, my, terrain_cost);
      } else {
        // Default: max combination.
        // Special handling because NO_INFORMATION=255 would otherwise dominate.
        if (terrain_cost == nav2_costmap_2d::NO_INFORMATION) {
          if (use_unknown_) {
            master_grid.setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);
          }
        } else if (old_cost == nav2_costmap_2d::NO_INFORMATION) {
          master_grid.setCost(mx, my, terrain_cost);
        } else {
          master_grid.setCost(mx, my, std::max(old_cost, terrain_cost));
        }
      }
    }
  }

  current_ = true;
}

unsigned char TerrainCostmapLayer::occupancyToNav2Cost(int occ) const
{
  if (occ < 0) {
    return nav2_costmap_2d::NO_INFORMATION;
  }

  if (occ >= lethal_threshold_) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  if (!scale_occupancy_to_costmap_) {
    return static_cast<unsigned char>(std::clamp(occ, 0, 252));
  }

  const double scaled = static_cast<double>(occ) * 252.0 / 100.0;
  const int cost = static_cast<int>(std::round(scaled));

  return static_cast<unsigned char>(std::clamp(cost, 0, 252));
}

void TerrainCostmapLayer::reset()
{
  current_ = false;
}

bool TerrainCostmapLayer::isClearable()
{
  return false;
}

}  // namespace terrain_costmap_layer

PLUGINLIB_EXPORT_CLASS(
  terrain_costmap_layer::TerrainCostmapLayer,
  nav2_costmap_2d::Layer)