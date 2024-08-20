#ifndef SPACENAV_TELEOP_NEEDLE_CONTROLLERS_HPP_
#define SPACENAV_TELEOP_NEEDLE_CONTROLLERS_HPP_

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "spacenav_teleop_needle_controller/visibility_control.h"

#include "geometry_msgs/msg/twist.hpp"

#include <Eigen/Dense>

namespace needle_controllers{

  class SpacenavTeleopController : public controller_interface::ControllerInterface{

  public:

    SPACENAV_TELEOP_NEEDLE_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;
    SPACENAV_TELEOP_NEEDLE_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    SPACENAV_TELEOP_NEEDLE_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    SPACENAV_TELEOP_NEEDLE_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    

    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period);
    
    SPACENAV_TELEOP_NEEDLE_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    SPACENAV_TELEOP_NEEDLE_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  protected:

    template <typename T>
    using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

    bool isActive() const { return active_; };

    std::vector<std::string> joint_names_;
    std::string cmd_interface_type_;
    std::string end_effector_link_;
    std::string robot_base_link_;
    std::string robot_description_;

    bool initialized_ = {false};
    bool configured_ = {false};
    bool active_ = {false};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr tgt_velocity_sub_;
    void targetVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr vw);
    
    void writeJointControlCmds();

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_cmd_vel_handles_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_pos_handles_;

    Eigen::Vector3d simulated_joint_cmd_;
    
  };
  
}

#endif
