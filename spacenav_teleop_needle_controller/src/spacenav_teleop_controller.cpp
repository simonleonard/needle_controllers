#include <string>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/parameter.hpp"
#include "spacenav_teleop_needle_controller/spacenav_teleop_controller.hpp"

namespace needle_controllers{
  
  controller_interface::CallbackReturn SpacenavTeleopController::on_init(){
    std::cout << "SpacenavTeleopController::on_init" << std::endl;
    if(!initialized_){
      auto_declare<std::string>("robot_description", "");
      auto_declare<std::string>("robot_base_link", "");
      auto_declare<std::string>("end_effector_link", "");
      auto_declare<std::string>("interface_name", "");
      auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
      initialized_ = true;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }
  
  controller_interface::CallbackReturn SpacenavTeleopController::on_configure(const rclcpp_lifecycle::State&){
    
    std::cout << "SpacenavTeleopController::on_configure" << std::endl;
    if(configured_)
      { return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; }
 
    robot_description_ = get_node()->get_parameter("robot_description").as_string();
    if(robot_description_.empty()){
      RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
      return controller_interface::CallbackReturn::ERROR;
    }
      
    robot_base_link_ = get_node()->get_parameter("robot_base_link").as_string();
    if(robot_base_link_.empty()){
      RCLCPP_ERROR(get_node()->get_logger(), "robot_base_link is empty");
      return controller_interface::CallbackReturn::ERROR;
    }
    
    end_effector_link_ = get_node()->get_parameter("end_effector_link").as_string();
    if(end_effector_link_.empty()){
      RCLCPP_ERROR(get_node()->get_logger(), "end_effector_link is empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if(joint_names_.empty()){
      RCLCPP_ERROR(get_node()->get_logger(), "joints array is empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    cmd_interface_type_ = get_node()->get_parameter("interface_name").as_string();
    if(cmd_interface_type_.empty()){
      RCLCPP_ERROR(get_node()->get_logger(), "No command_interfaces specified");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    //for(std::size_t i=0; i<joint_names_.size(); i++ ){
    //  std::cout << joint_names_[i] << std::endl;
    //}
    
    tgt_velocity_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::Twist>(get_node()->get_name() +
								 std::string("/cmd_tip"), 3,
								 std::bind(&SpacenavTeleopController::targetVelocityCallback,
									   this, std::placeholders::_1));

    configured_ = true;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  
  controller_interface::CallbackReturn SpacenavTeleopController::on_activate(const rclcpp_lifecycle::State&){
    if(active_)
      return controller_interface::CallbackReturn::SUCCESS;

    if(!controller_interface::get_ordered_interfaces(command_interfaces_, joint_names_, cmd_interface_type_, joint_cmd_vel_handles_)){
      RCLCPP_ERROR(get_node()->get_logger(),
		   "Expected %zu '%s' command interfaces, got %zu.",
		   joint_names_.size(), cmd_interface_type_.c_str(),
		   joint_cmd_vel_handles_.size());
      return CallbackReturn::ERROR;
    }

    if(!controller_interface::get_ordered_interfaces(state_interfaces_, joint_names_,
						     hardware_interface::HW_IF_POSITION,
						     joint_state_pos_handles_)){
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu '%s' state interfaces, got %zu.",
		   joint_names_.size(), hardware_interface::HW_IF_POSITION,
		   joint_state_pos_handles_.size());
      return CallbackReturn::ERROR;
    }

    simulated_joint_cmd_ << 0.0, 0.0, 0.0;
    writeJointControlCmds();

    active_ = true;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  
  controller_interface::CallbackReturn SpacenavTeleopController::on_deactivate(const rclcpp_lifecycle::State&){
    std::cout << "SpacenavTeleopController::on_deactivate" << std::endl;
    if(active_){
      joint_cmd_vel_handles_.clear();
      joint_state_pos_handles_.clear();
      this->release_interfaces();
    }
    active_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  
  controller_interface::InterfaceConfiguration SpacenavTeleopController::command_interface_configuration() const{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(joint_names_.size() * cmd_interface_type_.size());
    for (const auto & joint_name : joint_names_){
      conf.names.push_back(joint_name + std::string("/").append(cmd_interface_type_));
    }
    return conf;
  }

  controller_interface::InterfaceConfiguration SpacenavTeleopController::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(joint_names_.size());  // Only position
    for(const auto & joint_name : joint_names_){
      conf.names.push_back(joint_name + "/position");
    }
    return conf;
  }
  
  void SpacenavTeleopController::targetVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr vw){
    if(!this->isActive()){
      return;
    }

    if(std::isnan(vw->linear.x) || std::isnan(vw->linear.y) || std::isnan(vw->linear.z) ){
      auto & clock = *get_node()->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000, "NaN detected in target point. Ignoring input.");
      return;
    }

    simulated_joint_cmd_(0) = -vw->linear.y;
    simulated_joint_cmd_(1) =  vw->linear.x;
    simulated_joint_cmd_(2) =  vw->linear.z;
    
  }
  
  controller_interface::return_type SpacenavTeleopController::update(const rclcpp::Time&, const rclcpp::Duration&){

    if(!active_){
      for(std::size_t i=0; i<joint_names_.size(); ++i)
	{ simulated_joint_cmd_(i) = 0.0; }
    }
    
    writeJointControlCmds();
    return controller_interface::return_type::OK;
  }

  void SpacenavTeleopController::writeJointControlCmds(){
    for(std::size_t i=0; i<joint_names_.size(); ++i){
      joint_cmd_vel_handles_[i].get().set_value( simulated_joint_cmd_(i) );
    }
  }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(needle_controllers::SpacenavTeleopController, controller_interface::ControllerInterface)
