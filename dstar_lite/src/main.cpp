#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>

#include "dstar_lite/Dstar.h"
#include "dstar_lite/OccMapTransform.h"

using namespace cv;
using namespace std;

double quaternionToYaw(double x, double y, double z, double w)
{
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

class DstarLiteNode : public rclcpp::Node
{
public:
    DstarLiteNode() : Node("dstar_lite"), map_flag(false), startpoint_flag(false), targetpoint_flag(false), start_flag(false)
    {
        this->declare_parameter<int>("rate", 10);
        this->get_parameter("rate", rate);

        map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&DstarLiteNode::MapCallback, this, std::placeholders::_1));
        targetPoint_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&DstarLiteNode::TargetPointCallback, this, std::placeholders::_1));
        costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "dynamic_obstacle_layer", 10, std::bind(&DstarLiteNode::CostmapCallback, this, std::placeholders::_1));
        amcl_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "amcl_pose", 10, std::bind(&DstarLiteNode::AmclPoseCallback, this, std::placeholders::_1));

        mask_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("mask", 1);
        path_pub = this->create_publisher<nav_msgs::msg::Path>("nav_path", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / rate), std::bind(&DstarLiteNode::TimerCallback, this));
    }

private:
    
}



int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DstarLiteNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}