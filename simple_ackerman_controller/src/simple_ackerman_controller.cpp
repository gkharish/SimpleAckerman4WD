
#include "simple_ackerman_controller/simple_ackerman_controller.hpp"

#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace simple_ackerman_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

SimpleAckermanController::SimpleAckermanController() : controller_interface::ControllerInterface()
{
}

CallbackReturn SimpleAckermanController::on_init()
{
    try {
        auto_declare<std::string>("cmd_topic", _cmd_topic_name);
        auto_declare<std::string>("tf_topic", _tf_topic_name);

        auto_declare<std::vector<std::string>>("wheel_names", std::vector<std::string>());
        auto_declare<std::vector<std::string>>("steering_names", std::vector<std::string>());
    } catch (const std::exception &e) {
        fprintf(stderr, "Exception thrown while init: %s \n", e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

// Populate hardware interface command configuration
InterfaceConfiguration SimpleAckermanController::command_interface_configuration() const
{
    std::vector<std::string> conf_names;
    for (const auto &wheel_name : _wheel_names) {
        conf_names.push_back(wheel_name + "/" + HW_IF_VELOCITY);
    }
    for (const auto &steering_name : _steering_names) {
        conf_names.push_back(steering_name + "/" + HW_IF_POSITION);
    }
    return {interface_configuration_type::INDIVIDUAL, conf_names};
}

// Populate hardware interface state configuration
InterfaceConfiguration SimpleAckermanController::state_interface_configuration() const
{
    std::vector<std::string> conf_names;
    for (const auto &wheel_name : _wheel_names) {
        conf_names.push_back(wheel_name + "/" + HW_IF_VELOCITY);
    }
    for (const auto &steering_name : _steering_names) {
        conf_names.push_back(steering_name + "/" + HW_IF_POSITION);
    }
    return {interface_configuration_type::INDIVIDUAL, conf_names};
}

double convert_trans_rot_vel_to_steering_angle(double Vx, double theta_dot, double wheelbase)
{
    if (theta_dot == 0 || Vx == 0) {
        return 0;
    }
    return std::atan(theta_dot * wheelbase / Vx);
}

std::tuple<double, double> twist_to_ackermann(double Vx, double theta_dot)
{
    // using naming convention in http://users.isr.ist.utl.pt/~mir/cadeiras/robmovel/Kinematics.pdf
    double alpha, Ws;
    double wheelbase = 0.4;   // TODO: Get the data from urdf or vehicle params
    double radius    = 0.1;
    if (Vx == 0 && theta_dot != 0) {   // is spin action
        alpha = theta_dot > 0 ? M_PI_2 : -M_PI_2;
        Ws    = abs(theta_dot) * wheelbase / radius;
        return std::make_tuple(alpha, Ws);
    }

    alpha = convert_trans_rot_vel_to_steering_angle(Vx, theta_dot, wheelbase);
    Ws    = Vx / (radius * std::cos(alpha));
    return std::make_tuple(alpha, Ws);
}

void SimpleAckermanController::applyControlLaw(
        double &linear_x,
        double &linear_y,
        double &angular,
        double dt,
        std::vector<WheelHandle> &registered_wheel_handles,
        std::vector<SteeringHandle> &registered_steering_handles)
{
    // Calculated steering
    double theta = 0.02;

    // Calculate wheel speed
    double ws = 1.0;

    auto val = twist_to_ackermann(linear_x, angular);
    ws       = std::get<1>(val);
    theta    = std::get<0>(val);

    // Assign wheel speed to both the back wheels
    registered_wheel_handles[0].cmd_velocity.get().set_value(ws);
    registered_wheel_handles[1].cmd_velocity.get().set_value(ws);
    registered_wheel_handles[2].cmd_velocity.get().set_value(ws);
    registered_wheel_handles[3].cmd_velocity.get().set_value(ws);

    // Assign steering to both the front wheels
    registered_steering_handles[0].cmd_position.get().set_value(theta);
    registered_steering_handles[1].cmd_position.get().set_value(theta);
}

// Main function called at each control iteration
controller_interface::return_type SimpleAckermanController::update(
        const rclcpp::Time &time,
        const rclcpp::Duration & /*period*/)
{
    static auto previous_update_timestamp = time;
    double update_dt_sec                  = (time - previous_update_timestamp).seconds();
    previous_update_timestamp             = time;

    std::shared_ptr<Twist> last_speed_cmd;
    _received_velocity_msg_ptr.get(last_speed_cmd);

    auto _last_command = *last_speed_cmd;
    double linear_x    = _cmd_received.linear.x;
    double linear_y    = _cmd_received.linear.y;
    double angular     = _cmd_received.angular.z;

    applyControlLaw(linear_x,
                    linear_y,
                    angular,
                    update_dt_sec,
                    _registered_wheel_handles,
                    _registered_steering_handles);
    return controller_interface::return_type::OK;
}

// Configuration callback
CallbackReturn SimpleAckermanController::on_configure(const rclcpp_lifecycle::State &)
{
    std::cout << "DEBUG in on_configure: " << std::endl;
    _cmd_topic_name = get_node()->get_parameter("cmd_topic").as_string();
    _tf_topic_name  = get_node()->get_parameter("tf_topic").as_string();

    // Wheel and steering names. Should be given in the order FL, FR, BL, BR to work properly
    _wheel_names    = get_node()->get_parameter("wheel_names").as_string_array();
    _steering_names = get_node()->get_parameter("steering_names").as_string_array();

    if (!reset()) {
        return CallbackReturn::ERROR;
    }

    // Fill velocity pointers
    const Twist empty_twist;
    _received_velocity_msg_ptr.set(std::make_shared<Twist>(empty_twist));
    _cmd_received.linear.x  = 0.0;
    _cmd_received.angular.z = 0.0;
    _velocity_command_unstamped_subscriber =
            get_node()->create_subscription<geometry_msgs::msg::Twist>(
                    _cmd_topic_name,
                    rclcpp::SystemDefaultsQoS(),
                    std::bind(&SimpleAckermanController::on_cmd_unstamped,
                              this,
                              std::placeholders::_1));

    return CallbackReturn::SUCCESS;
}

CallbackReturn SimpleAckermanController::on_activate(const rclcpp_lifecycle::State &previous_state)
{
    std::cout << "DEBUG in on_activate: " << std::endl;
    // Configure the wheel motor handles
    const auto wheel_result =
            configure_wheels(_registered_wheel_handles, _registered_steering_handles);

    if (wheel_result == CallbackReturn::ERROR) {
        return CallbackReturn::ERROR;
    }
    if (_registered_wheel_handles.empty() || _registered_steering_handles.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Either  wheel interfaces, steering interfaces are non existent");
        return CallbackReturn::ERROR;
    }

    // Put flags ON and inform the user
    _subscriber_is_active = true;
    RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SimpleAckermanController::on_deactivate(
        const rclcpp_lifecycle::State &previous_state)
{
    // Put subscriber flag OFF
    _subscriber_is_active = false;
    return CallbackReturn::SUCCESS;
}

CallbackReturn SimpleAckermanController::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
    std::cout << "DEBUG in on_cleanup: " << std::endl;
    // Reset class attributes
    if (!reset()) {
        return CallbackReturn::ERROR;
    }

    // Clean shared pointers
    _received_velocity_msg_ptr.set(std::make_shared<Twist>());
    return CallbackReturn::SUCCESS;
}

CallbackReturn SimpleAckermanController::on_error(const rclcpp_lifecycle::State &previous_state)
{
    std::cout << "DEBUG in on_error: " << std::endl;
    // Reset class attributes
    if (!reset()) {
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn SimpleAckermanController::on_shutdown(const rclcpp_lifecycle::State &previous_state)
{
    return CallbackReturn::SUCCESS;
}

// Subscriber callback for Twist message
void SimpleAckermanController::on_cmd_unstamped(
        const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
    _cmd_received = *msg;
}
// Cleanup class attributes
bool SimpleAckermanController::reset()
{
    // Clear hardware interface handlers
    _registered_wheel_handles.clear();
    _registered_steering_handles.clear();

    // Clear subscribers
    return true;
}

// Configure all joints by assigning the right hardware interfaces
CallbackReturn SimpleAckermanController::configure_wheels(
        std::vector<WheelHandle> &registered_wheel_handles,
        std::vector<SteeringHandle> &registered_steering_handles)
{
    auto logger = get_node()->get_logger();

    // Register wheel handles
    registered_wheel_handles.reserve(_wheel_names.size());
    for (const auto &wheel_name : _wheel_names) {
        // Register state handle
        const auto state_handle =
                std::find_if(state_interfaces_.cbegin(),
                             state_interfaces_.cend(),
                             [&wheel_name](const auto &interface) {
                                 return interface.get_prefix_name() == wheel_name
                                        && interface.get_interface_name() == HW_IF_VELOCITY;
                             });
        if (state_handle == state_interfaces_.cend()) {
            RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
            return CallbackReturn::ERROR;
        }

        // Register command handle
        const auto command_handle =
                std::find_if(command_interfaces_.begin(),
                             command_interfaces_.end(),
                             [&wheel_name](const auto &interface) {
                                 return interface.get_prefix_name() == wheel_name
                                        && interface.get_interface_name() == HW_IF_VELOCITY;
                             });
        if (command_handle == command_interfaces_.end()) {
            RCLCPP_ERROR(
                    logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
            return CallbackReturn::ERROR;
        }

        // Append handles
        registered_wheel_handles.emplace_back(
                WheelHandle{std::ref(*state_handle), std::ref(*command_handle)});
    }

    // Register steering handles
    registered_steering_handles.reserve(_steering_names.size());
    for (const auto &steering_name : _steering_names) {
        // Register state handle
        const auto state_handle =
                std::find_if(state_interfaces_.cbegin(),
                             state_interfaces_.cend(),
                             [&steering_name](const auto &interface) {
                                 return interface.get_prefix_name() == steering_name
                                        && interface.get_interface_name() == HW_IF_POSITION;
                             });
        if (state_handle == state_interfaces_.cend()) {
            RCLCPP_ERROR(
                    logger, "Unable to obtain joint state handle for %s", steering_name.c_str());
            return CallbackReturn::ERROR;
        }

        // Register command handle
        const auto command_handle =
                std::find_if(command_interfaces_.begin(),
                             command_interfaces_.end(),
                             [&steering_name](const auto &interface) {
                                 return interface.get_prefix_name() == steering_name
                                        && interface.get_interface_name() == HW_IF_POSITION;
                             });
        if (command_handle == command_interfaces_.end()) {
            RCLCPP_ERROR(
                    logger, "Unable to obtain joint command handle for %s", steering_name.c_str());
            return CallbackReturn::ERROR;
        }

        // Append handles
        registered_steering_handles.emplace_back(
                SteeringHandle{std::ref(*state_handle), std::ref(*command_handle)});
    }

    return CallbackReturn::SUCCESS;
}
}   // namespace simple_ackerman_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(simple_ackerman_controller::SimpleAckermanController,
                            controller_interface::ControllerInterface)
