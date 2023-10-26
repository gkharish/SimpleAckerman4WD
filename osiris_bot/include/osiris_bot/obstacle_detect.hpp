#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;

class ObstacleDetector : public rclcpp::Node
{
public:
    ObstacleDetector(const rclcpp::NodeOptions &options);
    ~ObstacleDetector();

    // laser scan callback
    void laserScanCallBack(const LaserScan::SharedPtr laser_msg);
    bool isObstacleAhead();

private:
    bool obstacle_is_ahead_ = false;
    LaserScan laser_data_;
    double distance_threshold_ = 5.0;   // Obstacle distacne less than this is considered unsafe.
    rclcpp::Subscription<LaserScan>::ConstSharedPtr laser_scan_sub_;
};
