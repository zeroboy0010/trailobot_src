#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

class LidarToObstacleLayerNode : public rclcpp::Node
{
public:
  LidarToObstacleLayerNode() : Node("lidar_to_obstacle_layer_node")
  {
    // Subscriber to lidar data
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LidarToObstacleLayerNode::lidar_callback, this, std::placeholders::_1));

    // Publisher for the occupancy grid
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/dynamic_obstacle_layer", 10);

    // Initialize the occupancy grid
    costmap_.header.frame_id = "base_link"; // Replace with your robot's frame
    costmap_.info.resolution = 0.05; // Example resolution
    costmap_.info.width = 500;
    costmap_.info.height = 500;
    costmap_.info.origin.position.x = -12.5; // Set origin to match the environment
    costmap_.info.origin.position.y = -12.5;
    costmap_.data.resize(costmap_.info.width * costmap_.info.height, 0); // Initialize all cells to free

    // Set the inflation radius (in meters)
    inflation_radius_ = 0.2; // Example inflation radius (20 cm)
  }

private:
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Clear previous obstacle data
    std::fill(costmap_.data.begin(), costmap_.data.end(), 0);

    // Temporary obstacle list to apply inflation later
    std::vector<std::pair<int, int>> obstacles;

    // Process lidar data to detect obstacles
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
      if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min)
      {
        double angle = msg->angle_min + i * msg->angle_increment;
        double range = msg->ranges[i];
        int x = static_cast<int>((range * std::cos(angle) - costmap_.info.origin.position.x) / costmap_.info.resolution);
        int y = static_cast<int>((range * std::sin(angle) - costmap_.info.origin.position.y) / costmap_.info.resolution);

        if (x >= 0 && x < static_cast<int>(costmap_.info.width) && y >= 0 && y < static_cast<int>(costmap_.info.height))
        {
          obstacles.emplace_back(x, y);
        }
      }
    }

    // Inflate obstacles
    int inflation_cells = static_cast<int>(inflation_radius_ / costmap_.info.resolution);
    for (const auto& [obs_x, obs_y] : obstacles)
    {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx)
      {
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy)
        {
          int nx = obs_x + dx;
          int ny = obs_y + dy;
          if (nx >= 0 && nx < static_cast<int>(costmap_.info.width) && ny >= 0 && ny < static_cast<int>(costmap_.info.height))
          {
            // Use a simple Euclidean distance check to stay within the inflation radius
            if (std::hypot(dx, dy) * costmap_.info.resolution <= inflation_radius_)
            {
              costmap_.data[ny * costmap_.info.width + nx] = 100; // Mark cell as occupied
            }
          }
        }
      }
    }

    // Update the header timestamp
    costmap_.header.stamp = this->get_clock()->now();

    // Publish the updated occupancy grid
    costmap_pub_->publish(costmap_);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  nav_msgs::msg::OccupancyGrid costmap_;
  double inflation_radius_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarToObstacleLayerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
