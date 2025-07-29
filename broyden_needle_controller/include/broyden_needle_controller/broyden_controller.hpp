#ifndef BROYDEN_NEEDLE_CONTROLLERS_HPP_
#define BROYDEN_NEEDLE_CONTROLLERS_HPP_

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "broyden_needle_controller/visibility_control.h"

#include <moveit_msgs/msg/cartesian_trajectory.hpp>

#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>

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

    bool isExecuting() const { return executing_; };
    bool isActive() const { return active_; };

    std::vector<std::string> joint_names_;
    std::string cmd_interface_type_;
    std::string reference_frame_;
    std::vector<std::string> reference_interface_names_;
    std::string tracked_frame_;
    std::vector<std::string> tracked_interface_names_;
    std::string robot_description_;

    std::vector<moveit_msgs::msg::CartesianTrajectoryPoint> points;
    std::vector<moveit_msgs::msg::CartesianTrajectoryPoint>::const_iterator current_point;
    rclcpp::Time run_time_;
    
    bool initialized_ = {false};
    bool configured_ = {false};
    bool active_ = {false};
    bool executing_ = {false};

    rclcpp::Subscription<moveit_msgs::msg::CartesianTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr needle_sub_;
    geometry_msgs::msg::Point msr_needle_;
    bool use_feedback_state_interface_ = {true};
    bool valid_feedback_ = {false};
    
    void trajectoryCallback(const moveit_msgs::msg::CartesianTrajectory::SharedPtr trajectory);
    void needleCallback(const geometry_msgs::msg::Point::SharedPtr trajectory);
    
    void writeJointControlCmds();

    // This is to measure the positions
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> msr_pos_handles_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_vel_handles_;

    void broydenUpdate( const Eigen::Vector3d& x_j,
			const Eigen::Vector3d& y_j,
			const Eigen::Vector3d& ys );
    Eigen::Vector3d simulated_joint_cmd_;
    Eigen::Vector3d x_i, y_i;
    Eigen::Matrix3d J;
    
    tf2::Vector3 GetStatePosition( const std::string& frame_name );
    tf2::Quaternion GetStateQuaternion( const std::string& frame_name );
    Eigen::Vector3d getTargetXYZ();
    Eigen::Vector3d getRobotXYZ();

  };
  
}

#endif
