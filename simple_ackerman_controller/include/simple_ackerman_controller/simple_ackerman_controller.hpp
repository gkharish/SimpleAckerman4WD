#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"

// #include "exo_controllers/visibility_control.h"
#include "std_msgs/msg/int16.hpp"

namespace simple_ackerman_controller
{
// Constants
constexpr auto DEFAULT_CMD_TOPIC_NAME  = "/cmd_vel";
constexpr auto DEFAULT_ODOM_TOPIC_NAME = "/odom";
constexpr auto DEFAULT_TF_TOPIC_NAME   = "/tf";

// Aliases
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using Twist          = geometry_msgs::msg::TwistStamped;

struct WheelHandle {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> state_velocity;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> cmd_velocity;
};

struct SteeringHandle {
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> state_position;
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> cmd_position;
};

// The controller inherits from the main ControllerInterface class
class SimpleAckermanController : public controller_interface::ControllerInterface
{
public:
    SimpleAckermanController();

    CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(const rclcpp::Time &time,
                                             const rclcpp::Duration &period) override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    void applyControlLaw(double &linear_x,
                         double &linear_y,
                         double &angular,
                         double dt,
                         std::vector<WheelHandle> &registered_wheel_handles,
                         std::vector<SteeringHandle> &registered_steering_handles);

    std::tuple<double, double> twist_to_ackermann(double Vx, double theta_dot);

protected:
    // Handle for each wheel
    std::vector<WheelHandle> _registered_wheel_handles;
    std::vector<std::string> _wheel_names;

    // Handle for each steering
    std::vector<SteeringHandle> _registered_steering_handles;
    std::vector<std::string> _steering_names;

    // ROS topic names
    std::string _cmd_topic_name = DEFAULT_CMD_TOPIC_NAME;
    std::string _tf_topic_name  = DEFAULT_TF_TOPIC_NAME;

    // Velocity subscribers
    bool _subscriber_is_active = false;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
            _velocity_command_unstamped_subscriber = nullptr;

    realtime_tools::RealtimeBox<std::shared_ptr<Twist>> _received_velocity_msg_ptr{nullptr};

    Twist _goal_twist;
    geometry_msgs::msg::Twist _cmd_received;
    bool _use_stamped_vel = true;

    // vehicle params
    double _wheel_base;
    double _wheel_track;
    double _wheel_radius;

    // Protected methods
    bool reset();
    void on_cmd_stamped(const std::shared_ptr<Twist> msg);
    void on_cmd_unstamped(const std::shared_ptr<geometry_msgs::msg::Twist> msg);

    CallbackReturn configure_wheels(std::vector<WheelHandle> &registered_wheel_handles,
                                    std::vector<SteeringHandle> &registered_steering_handles);
    const char *feedback_type() const;

    // =====================================
};
}   // namespace simple_ackerman_controller
