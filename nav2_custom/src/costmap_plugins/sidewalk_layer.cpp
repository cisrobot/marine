#include "nav2_custom/costmap_plugins/sidewalk_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


PLUGINLIB_EXPORT_CLASS(nav2_custom::SidewalkLayer, nav2_costmap_2d::Layer)

namespace nav2_custom
{
SidewalkLayer::SidewalkLayer() {}

void SidewalkLayer::onInitialize()
{
  auto node = node_.lock();
  node->declare_parameter("topic_name", "/sidewalk_costmap");
  std::string topic = node->get_parameter("topic_name").as_string();

  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic, rclcpp::SensorDataQoS(),
    std::bind(&SidewalkLayer::costmapCallback, this, std::placeholders::_1));

  latest_costmap_ = nullptr;
  received_ = false;
}

void SidewalkLayer::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = msg;
  received_ = true;
  //RCLCPP_INFO(node_.lock()->get_logger(), "OccupancyGrid received (size: %ux%u)", msg->info.width, msg->info.height);
}

void SidewalkLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double *min_x, double *min_y, double *max_x, double *max_y)
{
  if (!received_ || !latest_costmap_) return;

  double size_x = latest_costmap_->info.width * latest_costmap_->info.resolution;
  double size_y = latest_costmap_->info.height * latest_costmap_->info.resolution;

  double origin_x = latest_costmap_->info.origin.position.x;
  double origin_y = latest_costmap_->info.origin.position.y;

  *min_x = origin_x;
  *max_x = origin_x + size_x;
  *min_y = origin_y;
  *max_y = origin_y + size_y;
}


void SidewalkLayer::updateCosts(
  nav2_costmap_2d::Costmap2D& master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!latest_costmap_) return;

  unsigned int size_x = latest_costmap_->info.width;
  unsigned int size_y = latest_costmap_->info.height;

  double origin_x = latest_costmap_->info.origin.position.x;
  double origin_y = latest_costmap_->info.origin.position.y;

  for (unsigned int y = 0; y < size_y; y++) {
    for (unsigned int x = 0; x < size_x; x++) {
      unsigned int index = x + y * size_x;
      int8_t occupancy_value = latest_costmap_->data[index];

      unsigned char cost;
      if (occupancy_value == 0) {
        cost = nav2_costmap_2d::FREE_SPACE;
      } else {
        cost = nav2_costmap_2d::LETHAL_OBSTACLE;
      }

      double resolution = latest_costmap_->info.resolution;

      // orientation (회전) 반영
      const auto &orientation = latest_costmap_->info.origin.orientation;
      tf2::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      );
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      double dx = (x + 0.5) * resolution;
      double dy = (y + 0.5) * resolution;

      // 회전 적용 (origin 좌표계를 기준으로)
      double wx = origin_x + dx * std::cos(yaw) - dy * std::sin(yaw);
      double wy = origin_y + dx * std::sin(yaw) + dy * std::cos(yaw);


      unsigned int mx, my;
      if (master_grid.worldToMap(wx, wy, mx, my)) {
        unsigned char existing_cost = master_grid.getCost(mx, my);

        // 장애물은 무조건 누적, free만 선택적으로 업데이트
        if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
          master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        } else {
          if (existing_cost == nav2_costmap_2d::NO_INFORMATION ||
              existing_cost == nav2_costmap_2d::FREE_SPACE) {
            master_grid.setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
          }
          // 기존 장애물이 있으면 그대로 유지 (절대 덮지 않음)
        }

      }
    }
  }

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}






void SidewalkLayer::reset()
{
  received_ = false;
  latest_costmap_.reset();
}
}

