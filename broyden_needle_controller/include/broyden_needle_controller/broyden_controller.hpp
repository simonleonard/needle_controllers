#ifndef BROYDEN_NEEDLE_CONTROLLERS_HPP_
#define BROYDEN_NEEDLE_CONTROLLERS_HPP_

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "broyden_needle_controller/visibility_control.h"

#include "geometry_msgs/msg/point_stamped.hpp"

#include <Eigen/Dense>

namespace needle_controllers{

  class BroydenController : public controller_interface::ControllerInterface{

  public:

    BROYDEN_NEEDLE_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;
    BROYDEN_NEEDLE_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    BROYDEN_NEEDLE_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    BROYDEN_NEEDLE_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    

    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period);
    
    BROYDEN_NEEDLE_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    BROYDEN_NEEDLE_CONTROLLER_PUBLIC
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

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr tgt_point_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr msr_point_sub_;
    void targetPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr target);
    void measuredPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr measured);
    
    void writeJointControlCmds();

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_cmd_vel_handles_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_pos_handles_;

    void broydenUpdate( const Eigen::Vector3d& x_j, const Eigen::Vector3d& y_j );
    Eigen::Vector3d simulated_joint_cmd_;
    Eigen::Vector3d x_i, y_i;
    Eigen::Matrix3d J;
    
  };
  
}

#endif
