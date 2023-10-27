#include "osiris_bot/obstacle_detect.hpp"
#include "math.h"

ObstacleDetector::ObstacleDetector(const rclcpp::NodeOptions &options) :
    Node("obstacle_detect_test", options)
{
    this->declare_parameter("distance_threshold", 3.0);
    this->get_parameter("distance_threshold", distance_threshold_);

    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 1),
                                  rmw_qos_profile_sensor_data);

    // Create subscription
    std::string topic_name = "/gpu_lidar/scans";
    laser_scan_sub_        = this->create_subscription<LaserScan>(
            topic_name,
            sensor_qos,
            std::bind(&ObstacleDetector::laserScanCallBack, this, std::placeholders::_1));

    // Publish obstacle warning!
    std::string obstacle_warnig_topic = "/obstacle_ahead";
    warning_pub_ = this->create_publisher<std_msgs::msg::Bool>(obstacle_warnig_topic, sensor_qos);
}

ObstacleDetector::~ObstacleDetector() {}

void ObstacleDetector::laserScanCallBack(const LaserScan::SharedPtr laser_msg)
{
    laser_data_ = *laser_msg;

    auto message = std_msgs::msg::Bool();
    message.data = isObstacleAhead();
    warning_pub_->publish(message);
}

bool ObstacleDetector::isObstacleAhead()
{
    bool obstacle_is_ahead = false;
    std::vector<double> median_dist_accumulator;
    for (unsigned int i = 0; i < 10; ++i) {
        if (std::abs(laser_data_.ranges[i]) >= distance_threshold_)
            continue;
        median_dist_accumulator.push_back(laser_data_.ranges[i]);
    }

    if (median_dist_accumulator.size() >= 2) {
        double avg_distance =
                std::accumulate(median_dist_accumulator.begin(), median_dist_accumulator.end(), 0)
                / (median_dist_accumulator.size());

        RCLCPP_INFO(this->get_logger(), "Obstacle is ahead at distance: %f", avg_distance);
        obstacle_is_ahead = true;
    } else {
        obstacle_is_ahead = false;
    }
    return obstacle_is_ahead;
}
