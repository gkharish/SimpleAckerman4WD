#include "osiris_bot/obstacle_detect.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<ObstacleDetector>(options));
    rclcpp::shutdown();
    return 0;
}
