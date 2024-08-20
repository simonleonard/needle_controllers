 #include <string>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/parameter.hpp"
#include "broyden_needle_controller/broyden_controller.hpp"

namespace needle_controllers{
  
  controller_interface::CallbackReturn BroydenController::on_init(){
    std::cout << "BroydenController::on_init" << std::endl;
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
  
  controller_interface::CallbackReturn BroydenController::on_configure(const rclcpp_lifecycle::State& previous_state){
    
    std::cout << "BroydenController::on_configure" << std::endl;
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
    /*
    std::cout << "robot description: " << robot_description_ << std::endl;
    std::cout << "robot base link: " << robot_base_link_ << std::endl;
    std::cout << "end effector link: " << end_effector_link_ << std::endl;
    std::cout << "command interface: " << cmd_interface_type_ << std::endl;
    */
    for(int i=0; i<joint_names_.size(); i++ ){
      std::cout << joint_names_[i] << std::endl;
    }
    
    tgt_point_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::PointStamped>(get_node()->get_name() +
									std::string("/cmd_tip"), 3,
									std::bind(&BroydenController::targetPointCallback,
										  this, std::placeholders::_1));
    msr_point_sub_ =
      get_node()->create_subscription<geometry_msgs::msg::PointStamped>(get_node()->get_name() +
									std::string("/msr_tip"), 3,
									std::bind(&BroydenController::measuredPointCallback,
										  this, std::placeholders::_1));

    configured_ = true;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  
  controller_interface::CallbackReturn BroydenController::on_activate(const rclcpp_lifecycle::State& previous_state){
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
    x_i << 0.0, 0.0, 0.0;
    y_i << 0.0, 0.0, 0.0;
    J = Eigen::Matrix3d::Identity();
    
    writeJointControlCmds();
    
    active_ = true;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  
  controller_interface::CallbackReturn BroydenController::on_deactivate(const rclcpp_lifecycle::State& previous_state){
    std::cout << "BroydenController::on_deactivate" << std::endl;
    if(active_){
      joint_cmd_vel_handles_.clear();
      joint_state_pos_handles_.clear();
      this->release_interfaces();
    }
    active_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  
  controller_interface::InterfaceConfiguration BroydenController::command_interface_configuration() const{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(joint_names_.size() * cmd_interface_type_.size());
    for (const auto & joint_name : joint_names_){
      conf.names.push_back(joint_name + std::string("/").append(cmd_interface_type_));
    }
    return conf;
  }

  controller_interface::InterfaceConfiguration BroydenController::state_interface_configuration() const{
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(joint_names_.size());  // Only position
    for(const auto & joint_name : joint_names_){
      conf.names.push_back(joint_name + "/position");
    }
    return conf;
  }
  
  void BroydenController::targetPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr target){
    if(!this->isActive()){
      return;
    }

    if(std::isnan(target->point.x) || std::isnan(target->point.y) || std::isnan(target->point.z) ){
      auto & clock = *get_node()->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000, "NaN detected in target point. Ignoring input.");
      return;
    }

    if(target->header.frame_id != robot_base_link_){
      auto & clock = *get_node()->get_clock();
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), clock, 3000,
			   "Got target point in wrong reference frame. Expected: %s but got %s",
			   robot_base_link_.c_str(), target->header.frame_id.c_str());
      return;
    }
  }
  
  void BroydenController::measuredPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr measured){
    if(!this->isActive()){
      return;
    }

    if(std::isnan(measured->point.x) || std::isnan(measured->point.y) || std::isnan(measured->point.z) ){
      auto & clock = *get_node()->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000, "NaN detected in measured point. Ignoring input.");
      return;
    }

    if(measured->header.frame_id != robot_base_link_){
      auto & clock = *get_node()->get_clock();
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), clock, 3000,
			   "Got measured point in wrong reference frame. Expected: %s but got %s",
			   robot_base_link_.c_str(), measured->header.frame_id.c_str());
      return;
    }
    
  }

  controller_interface::return_type BroydenController::update(const rclcpp::Time&, const rclcpp::Duration& period){
    //static double t=0.0;

    if(active_){
      
      //for(std::size_t i=0; i<joint_names_.size(); ++i){
      //simulated_joint_cmd_(i) = sin(t);
      //}
      //t+=0.1;

      Eigen::Vector3d x_j, y_j;
      broydenUpdate(x_j, y_j);
    }
    
    //std::cout << J << std::endl;
    writeJointControlCmds();
    return controller_interface::return_type::OK;
  }

  void BroydenController::writeJointControlCmds(){

    for(std::size_t i=0; i<joint_names_.size(); ++i){
      joint_cmd_vel_handles_[i].get().set_value( simulated_joint_cmd_(i) );
      //std::cout << joint_cmd_vel_handles_[i].get().get_value() << " ";
    }
    //std::cout << std::endl << std::endl;
  }

  void BroydenController::broydenUpdate( const Eigen::Vector3d& x_j, const Eigen::Vector3d& y_j ){

    // y_t = f(x_t)
    // dy = y_t - y_t-1 
    Eigen::Vector3d dx = x_j - x_i;
    Eigen::Vector3d dy = y_j - y_i;

    if( 0 < dx.norm() )
      { J = J + ((dy - J*dx)/dx.norm())*dx.transpose(); }
    
    //std::cout << J << std::endl;
    // JacobiSVD: thin U and V are only available when your matrix has a dynamic number of columns.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double cond = svd.singularValues()(0)/svd.singularValues()(svd.singularValues().size()-1);
    if( cond < 1e6 ){

      Eigen::Vector3d rhs(0, 0.5, 0);
      simulated_joint_cmd_ = svd.solve(rhs);
      //std::cout << svd.matrixU() << std::endl << svd.singularValues() << std::endl << svd.matrixV() << std::endl;
      //std::cout << simulated_joint_cmd_ << std::endl << std::endl;
    }
        
  }
  
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(needle_controllers::BroydenController, controller_interface::ControllerInterface)
