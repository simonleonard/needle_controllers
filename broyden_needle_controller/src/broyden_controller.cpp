 #include <string>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/parameter.hpp"
#include "broyden_needle_controller/broyden_controller.hpp"

namespace needle_controllers{
  
  controller_interface::CallbackReturn BroydenController::on_init(){
    RCLCPP_INFO_STREAM( get_node()->get_logger(), "Initializing." );
    
    if(!initialized_){
      auto_declare<std::string>("robot_description", "");
      
      auto_declare<std::string>("command_interface", "");
      auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
      
      auto_declare<std::string>("reference_frame", "");
      auto_declare<std::vector<std::string>>("reference_interfaces", std::vector<std::string>());
      
      auto_declare<std::string>("tracked_frame", "");
      auto_declare<std::vector<std::string>>("tracked_interfaces", std::vector<std::string>());
      
      initialized_ = true;
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }
  
  controller_interface::CallbackReturn BroydenController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/){
    
    RCLCPP_INFO_STREAM( get_node()->get_logger(), "Configuring." );

    if(configured_)
      { return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; }

    robot_description_ = get_node()->get_parameter("robot_description").as_string();
    if(robot_description_.empty()){
      RCLCPP_ERROR(get_node()->get_logger(), "robot_description is empty");
      return controller_interface::CallbackReturn::ERROR;
    }
      
    cmd_interface_type_ = get_node()->get_parameter("command_interface").as_string();
    if(cmd_interface_type_.empty()){
      RCLCPP_ERROR(get_node()->get_logger(), "No command_interfaces specified");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    if(joint_names_.empty()){
      RCLCPP_ERROR(get_node()->get_logger(), "joints array is empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    tracked_frame_ = get_node()->get_parameter("tracked_frame").as_string();
    if(tracked_frame_.empty()){
      RCLCPP_ERROR(get_node()->get_logger(), "tracked_frame is empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    tracked_interface_names_ = get_node()->get_parameter("tracked_interfaces").as_string_array();
    if(tracked_interface_names_.empty()){
      RCLCPP_ERROR(get_node()->get_logger(), "tracked interfaces array is empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    reference_frame_ = get_node()->get_parameter("reference_frame").as_string();
    if(reference_frame_.empty()){
      RCLCPP_ERROR(get_node()->get_logger(), "reference_frame is empty");
      return controller_interface::CallbackReturn::ERROR;
    }
    
    reference_interface_names_ = get_node()->get_parameter("reference_interfaces").as_string_array();
    if(reference_interface_names_.empty()){
      RCLCPP_ERROR(get_node()->get_logger(), "reference interfaces array is empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    /*
    std::cout << "robot description: " << robot_description_ << std::endl;
    std::cout << "reference frame: " << reference_frame_ << std::endl;
    std::cout << "tracked frame: " << tracked_frame_ << std::endl;
    std::cout << "command interface: " << cmd_interface_type_ << std::endl;

    for(auto& name : joint_names_ ){
      std::cout << name << std::endl;
    }
    
    for(auto& name : state_interface_names_ ){
      std::cout << name << std::endl;
    }
    */
    
    trajectory_sub_ =
      get_node()->create_subscription<moveit_msgs::msg::CartesianTrajectory>
      (get_node()->get_name() +
       std::string("/needle_trajectory"), 3,
       std::bind(&BroydenController::trajectoryCallback,
		 this, std::placeholders::_1));
    
    configured_ = true;
    
    return controller_interface::CallbackReturn::SUCCESS;
  }
  
  controller_interface::CallbackReturn BroydenController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/){

    RCLCPP_INFO_STREAM( get_node()->get_logger(), "Activating." );
    
    if(active_)
      return controller_interface::CallbackReturn::SUCCESS;

    for(std::size_t i=0; i<command_interfaces_.size(); i++ )
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "Command interface: " << command_interfaces_[i].get_name() );
    
    if(!controller_interface::get_ordered_interfaces(command_interfaces_,
						     joint_names_,
						     cmd_interface_type_,
						     cmd_vel_handles_)){
      RCLCPP_ERROR(get_node()->get_logger(),
		   "Expected %zu '%s' command interfaces, got %zu.",
		   joint_names_.size(),
		   cmd_interface_type_.c_str(),
		   cmd_vel_handles_.size());
      return CallbackReturn::ERROR;
    }

    for( auto name : reference_interface_names_ )
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "Reference interface: " << name);
    
    for( auto name : tracked_interface_names_ )
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "Tracked interface: " << name);
    
    for( std::size_t i=0; i<state_interfaces_.size(); i++ )
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "State interface: " <<  state_interfaces_[i].get_name());

    // gather all the state interfaces:
    // reference
    std::vector<std::string> state_interface_names(reference_interface_names_);
    // tracked 
    state_interface_names.insert(state_interface_names.end(),
				 tracked_interface_names_.begin(),
				 tracked_interface_names_.end() );
    // joint positions
    for( auto joint : joint_names_ )
      { state_interface_names.push_back( joint + "/position" ); }
				 
    if(!controller_interface::get_ordered_interfaces(state_interfaces_,
						     state_interface_names,
						     "",//controller_interface::interface_configuration_type::ALL,
						     msr_pos_handles_)){
      RCLCPP_ERROR_STREAM( get_node()->get_logger(),
			   "Expected " << state_interface_names.size()
			   << state_interface_names.size() << " "
			   << hardware_interface::HW_IF_POSITION << " state interfaces. Got "
			   << msr_pos_handles_.size() );
      return CallbackReturn::ERROR;
    }
    

    // The needle is in the Aurora's frame. Need to transform to zframe
    x_i = getRobotXYZ();
    y_i = getTargetXYZ();
    //J = Eigen::Matrix3d::Identity();
    J << -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0;
    
    simulated_joint_cmd_ << 0.0, 0.0, 0.0;
    writeJointControlCmds();
    
    active_ = true;
    
    return controller_interface::CallbackReturn::SUCCESS;
  }
  
  controller_interface::CallbackReturn BroydenController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/){
    RCLCPP_INFO_STREAM( get_node()->get_logger(), "Deactivating." );
    if(active_){
      cmd_vel_handles_.clear();
      msr_pos_handles_.clear();
      this->release_interfaces();
    }
    active_ = false;
    return controller_interface::CallbackReturn::SUCCESS;
  }
  
  controller_interface::InterfaceConfiguration BroydenController::command_interface_configuration() const{
    // configure the joints commands interface
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    conf.names.reserve(joint_names_.size() * cmd_interface_type_.size());
    for( const auto & joint_name : joint_names_ ){
      conf.names.push_back(joint_name + "/velocity");
    }
    return conf;
  }

  controller_interface::InterfaceConfiguration BroydenController::state_interface_configuration() const{
    // configure the position state interface
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for(const auto & joint_name : joint_names_){
      conf.names.push_back(joint_name + "/position");
    }

    // This is redundant with the activate stuff.
    conf.names.push_back(tracked_frame_ + "/pose.position.x");
    conf.names.push_back(tracked_frame_ + "/pose.position.y");
    conf.names.push_back(tracked_frame_ + "/pose.position.z");

    conf.names.push_back(reference_frame_ + "/pose.position.x");
    conf.names.push_back(reference_frame_ + "/pose.position.y");
    conf.names.push_back(reference_frame_ + "/pose.position.z");
    conf.names.push_back(reference_frame_ + "/pose.orientation.x");
    conf.names.push_back(reference_frame_ + "/pose.orientation.y");
    conf.names.push_back(reference_frame_ + "/pose.orientation.z");
    conf.names.push_back(reference_frame_ + "/pose.orientation.w");

    return conf;
  }

  //
  void BroydenController::trajectoryCallback(const moveit_msgs::msg::CartesianTrajectory::SharedPtr trajectory){
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Received trajectory.");
    
    if(!this->isActive()){
      RCLCPP_WARN_STREAM(get_node()->get_logger(), "Received a trajectory but controller is inactive.");
      return;
    }

    if(this->isExecuting()){
      RCLCPP_WARN_STREAM(get_node()->get_logger(), "Received a trajectory while executing a previous one.");
      return;
    }

    if( reference_frame_ != trajectory->header.frame_id ){
      RCLCPP_WARN_STREAM(get_node()->get_logger(),
			 "Received trajectory with frame_id " << trajectory->header.frame_id <<
			 " expected " << reference_frame_ );
      return;
    }

    if( tracked_frame_ != trajectory->tracked_frame ){
      RCLCPP_WARN_STREAM(get_node()->get_logger(),
			 "Received trajectory with tracked frame " << trajectory->tracked_frame <<
			 " expected " << tracked_frame_ );
      return;
    }

    points.clear();
    current_point = points.begin();
    rclcpp::Duration time_from_start(0, 0);

    for(auto p : trajectory->points ){
      std::cout << p.point.pose.position.x << " "
		<< p.point.pose.position.y << " "
		<< p.point.pose.position.z << std::endl;
      if( !std::isfinite(p.point.pose.position.x) ||
	  !std::isfinite(p.point.pose.position.y) ||
	  !std::isfinite(p.point.pose.position.z) ){
	RCLCPP_WARN_STREAM(get_node()->get_logger(), "Non-finite coordinate detected in trajectory point. Ignoring input.");
	return;
      }

      if( rclcpp::Duration(p.time_from_start) < time_from_start ){
	RCLCPP_WARN_STREAM(get_node()->get_logger(), "Trajectory point has non-increasing time from start.");
	return;
      }
      time_from_start = p.time_from_start;
      
    }
    
    points = trajectory->points;
    current_point = points.begin();
    run_time_ = rclcpp::Time(0, 0);
    executing_ = true;
  }
  
  controller_interface::return_type BroydenController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/){
    
    if(this->isActive() && this->isExecuting()){

      // next target is ahead in time
      //if( run_time_ < (run_time_ + current_point->time_from_start) ){
      
      Eigen::Vector3d x_j = getRobotXYZ();
      Eigen::Vector3d y_j = getTargetXYZ();
      Eigen::Vector3d ys(current_point->point.pose.position.x,
			 current_point->point.pose.position.y,
			 current_point->point.pose.position.z );
      broydenUpdate( x_j, y_j, ys-y_j );
      x_i = x_j;
      y_i = y_j;
      //Eigen::Vector3d x_j = getRobotXYZ();
      //std::cout << "Robot: " << x_j << std::endl;
      //Eigen::Vector3d y_j = getTargetXYZ();
      //std::cout << "Target: " << y_j << std::endl;
	

	
	/*
	RCLCPP_WARN_STREAM(get_node()->get_logger(),
			   "X: " << current_point->point.pose.position.x <<
			   "Y: " << current_point->point.pose.position.y <<
			   "Z: " << current_point->point.pose.position.z);
	*/
	//std::cout << J << std::endl;
      current_point++;
	
      if( current_point == points.end() ){
	executing_ = false;
	simulated_joint_cmd_ << 0.0, 0.0, 0.0;
      }
      writeJointControlCmds();
	
    }
    
    return controller_interface::return_type::OK;
  }

  void BroydenController::writeJointControlCmds(){

    for(std::size_t i=0; i<joint_names_.size(); ++i){
      cmd_vel_handles_[i].get().set_value( simulated_joint_cmd_(i) );
      //std::cout << cmd_vel_handles_[i].get().get_value() << " ";
    }
    //std::cout << std::endl << std::endl;
  }

  tf2::Vector3 BroydenController::GetStatePosition( const std::string& frame_name ){
    double x, y, z;
    //bool foundx=false, foundy=false, foundz=false;
    
    for( auto interface : msr_pos_handles_ ){
      //std::cout << interface.get().get_name() << std::endl;
      //std::cout << interface.get().get_interface_name() << std::endl;
      //std::cout << interface.get().get_full_name() << std::endl;
      //std::cout << interface.get().get_prefix_name() << std::endl;
      //std::cout << interface.get().get_value() << std::endl;

      if( interface.get().get_prefix_name().find( frame_name )      != std::string::npos && // find the tracked frame
	  interface.get().get_interface_name().find( "position.x" ) != std::string::npos){  // find the x pos
	x = interface.get().get_value();
      }
      else{
	//RCLCPP_WARN_STREAM(get_node()->get_logger(), "Could not find interface " << frame_name << "/position.x.");
      }
      if( interface.get().get_prefix_name().find( frame_name )      != std::string::npos && // find the tracked frame
	  interface.get().get_interface_name().find( "position.y" ) != std::string::npos){  // find the y pos
	y = interface.get().get_value();
      }
      else{
	//RCLCPP_WARN_STREAM(get_node()->get_logger(), "Could not find interface " << frame_name << "/position.y.");
      }
      if( interface.get().get_prefix_name().find( frame_name )      != std::string::npos && // find the tracked frame
	  interface.get().get_interface_name().find( "position.z" ) != std::string::npos){  // find the z pos
	z = interface.get().get_value();
      }
      else{
	//RCLCPP_WARN_STREAM(get_node()->get_logger(), "Could not find interface " << frame_name << "/position.z.");
      }

    }
    tf2::Vector3 xyz(x, y, z);
    
    return xyz;

  }
  
  tf2::Quaternion BroydenController::GetStateQuaternion( const std::string& frame_name ){
    double qw, qx, qy, qz;
    //bool foundqw=false, foundqx=false, foundqy=false, foundqz=false;
    
    for( auto interface : msr_pos_handles_ ){
      //std::cout << interface.get().get_name() << std::endl;
      //std::cout << interface.get().get_interface_name() << std::endl;
      //std::cout << interface.get().get_full_name() << std::endl;
      //std::cout << interface.get().get_prefix_name() << std::endl;
      //std::cout << interface.get().get_value() << std::endl;

      if( interface.get().get_prefix_name().find( frame_name )      != std::string::npos &&   // find the tracked frame
	  interface.get().get_interface_name().find( "orientation.w" ) != std::string::npos){ // find the x pos
	qw = interface.get().get_value();
	//foundqw=true;
      }
      else{
	//RCLCPP_WARN_STREAM(get_node()->get_logger(), "Could not find interface " << frame_name << "/orientation.w.");
      }
      if( interface.get().get_prefix_name().find( frame_name )      != std::string::npos &&   // find the tracked frame
	  interface.get().get_interface_name().find( "orientation.x" ) != std::string::npos){ // find the x pos
	qx = interface.get().get_value();
	//foundqx=true;
      }
      else{
	//RCLCPP_WARN_STREAM(get_node()->get_logger(), "Could not find interface " << frame_name << "/orientation.x.");
      }
      if( interface.get().get_prefix_name().find( frame_name )      != std::string::npos &&   // find the tracked frame
	  interface.get().get_interface_name().find( "orientation.y" ) != std::string::npos){ // find the y pos
	qy = interface.get().get_value();
	//foundqy=true;
      } 
      else{
	//RCLCPP_WARN_STREAM(get_node()->get_logger(), "Could not find interface " << frame_name << "/orientation.y.");
      }
      if( interface.get().get_prefix_name().find( frame_name )      != std::string::npos &&   // find the tracked frame
	  interface.get().get_interface_name().find( "orientation.z" ) != std::string::npos){ // find the z pos
	qz = interface.get().get_value();
	//foundqz=true;
      }
      else{
	//RCLCPP_WARN_STREAM(get_node()->get_logger(), "Could not find interface " << frame_name << "/orientation.z.");
      }

    }

    tf2::Quaternion q(qx, qy, qz, qw);
    
    return q;

  }

  Eigen::Vector3d BroydenController::getTargetXYZ(){

    // Get needle position:
    tf2::Vector3 needlexyz = GetStatePosition( tracked_frame_ );
    tf2::Vector3 refxyz = GetStatePosition( reference_frame_ );
    tf2::Quaternion refq = GetStateQuaternion( reference_frame_ );    
    tf2::Transform ref( refq, refxyz );
    
    // The needle is in the Aurora's frame. Need to transform to zframe
    needlexyz = ref.inverse() * needlexyz;
    //std::cout << needlexyz.getX() << " " << needlexyz.getY() << " " << needlexyz.getZ() << std::endl;

    return Eigen::Vector3d ( needlexyz.getX(), needlexyz.getY(), needlexyz.getZ() );
    
  }
  
  Eigen::Vector3d BroydenController::getRobotXYZ(){

    double x, y, z;
    bool foundx=false, foundy=false, foundz=false;
    
    for( auto interface : msr_pos_handles_ ){
      //std::cout << interface.get().get_name() << std::endl;
      //std::cout << interface.get().get_interface_name() << std::endl;
      //std::cout << interface.get().get_full_name() << std::endl;
      //std::cout << interface.get().get_prefix_name() << std::endl;
      //std::cout << interface.get().get_value() << std::endl;
      if( interface.get().get_prefix_name().find( "horizontal_joint" ) != std::string::npos && // find the tracked frame
	  interface.get().get_interface_name().find( "position" )      != std::string::npos){  // find the x pos
	x = interface.get().get_value();
	//foundx=true;
      }
      else{
	//RCLCPP_WARN_STREAM(get_node()->get_logger(), "Could not find interface horizontal_joint/position");
      }
      if( interface.get().get_prefix_name().find( "insertion_joint" )      != std::string::npos && // find the tracked frame
	  interface.get().get_interface_name().find( "position" ) != std::string::npos){  // find the y pos
	y = interface.get().get_value();
	//foundy=true;
      }
      else{
	//RCLCPP_WARN_STREAM(get_node()->get_logger(), "Could not find interface insertion_joint/position");
      }
      if( interface.get().get_prefix_name().find( "vertical_joint" )      != std::string::npos && // find the tracked frame
	  interface.get().get_interface_name().find( "position" ) != std::string::npos){  // find the z pos
	z = interface.get().get_value();
	//foundz=true;
      }
      else{
	//RCLCPP_WARN_STREAM(get_node()->get_logger(), "Could not find interface vertical_joint/position");
      }

    }
    return Eigen::Vector3d( x, y, z );
  }
  
  void BroydenController::broydenUpdate( const Eigen::Vector3d& x_j,
					 const Eigen::Vector3d& y_j,
					 const Eigen::Vector3d& ys ){

    
    Eigen::Vector3d dx = x_j - x_i;
    Eigen::Vector3d dy = y_j - y_i;

    if( 0 < dx.norm() )
      { J = J + ((dy - J*dx)/dx.norm())*dx.transpose(); }
    
    //std::cout << J << std::endl;
    // JacobiSVD: thin U and V are only available when your matrix has a dynamic number of columns.
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double cond = svd.singularValues()(0)/svd.singularValues()(svd.singularValues().size()-1);
    std::cout << "J: " << std::endl << J << std::endl;
    if( cond < 1e6 ){
      simulated_joint_cmd_ = svd.solve(ys);
      //std::cout << svd.matrixU() << std::endl << svd.singularValues() << std::endl << svd.matrixV() << std::endl;
    }
    else{
      RCLCPP_WARN_STREAM(get_node()->get_logger(), "Jacobian near singular. Cond(J) = " << cond << ".");                        
    }
        
  }
  
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(needle_controllers::BroydenController, controller_interface::ControllerInterface)
