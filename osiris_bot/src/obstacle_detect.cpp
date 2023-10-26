#include "osiris_bot/obstacle_detect.hpp"
#include "math.h"

ObstacleDetector::ObstacleDetector(const rclcpp::NodeOptions &options) :
    Node("obstacle_detect_test", options)
{
    auto sensor_qos = rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, 1),
                                  rmw_qos_profile_sensor_data);

    // Create subscription
    std::string topic_name = "/gpu_lidar/scans";
    laser_scan_sub_        = this->create_subscription<LaserScan>(
            topic_name,
            sensor_qos,
            std::bind(&ObstacleDetector::laserScanCallBack, this, std::placeholders::_1));
}

ObstacleDetector::~ObstacleDetector() {}

void ObstacleDetector::laserScanCallBack(const LaserScan::SharedPtr laser_msg)
{
    laser_data_ = *laser_msg;
    std::cout << "Laser Data: " << laser_data_.angle_increment << std::endl;

    double median_dist_from_bot = 0.0;
    std::vector<double> median_dist_accumulator;
    for (unsigned int i = 0; i < 10; ++i) {
        if (std::abs(laser_msg->ranges[i]) >= distance_threshold_)
            continue;
        // median_dist_from_bot += laser_msg->ranges[i];
        median_dist_accumulator.push_back(laser_msg->ranges[i]);
    }
    if (median_dist_accumulator.size() >= 2) {
        std::cout << "Obstacle is ahead at " << median_dist_from_bot << std::endl;
    }

    // median_dist_from_bot /= 10;
    // std::cout << "Median distance of the wall: " << median_dist_from_bot << std::endl;

    // if (median_dist_from_bot <= distance_threshold_) {
    //     std::cout << "Obstacle is ahead at " << median_dist_from_bot << std::endl;
    //     obstacle_is_ahead_ = true;
    // }
}

bool ObstacleDetector::isObstacleAhead()
{
    return obstacle_is_ahead_;
}
