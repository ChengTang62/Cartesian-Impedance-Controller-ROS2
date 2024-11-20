#include <cartesian_impedance_controller/cartesian_impedance_controller_ros.h>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace cartesian_impedance_controller
{
  CartesianImpedanceControllerRos::CartesianImpedanceControllerRos() : traj_duration_(0, 0){}
  double saturateValue(double x, double x_min, double x_max){return std::min(std::max(x, x_min), x_max);}

  // void setFilterCoefficient(double cutoff_frequency, double dt) {
  //   double rc = 1.0 / (2 * M_PI * cutoff_frequency);
  //   alpha_ = dt / (dt + rc);
  // }

  void EigenVectorToWrench(const Eigen::Matrix<double, 6, 1> &v, geometry_msgs::msg::Wrench *wrench){
    wrench->force.x = v(0);
    wrench->force.y = v(1);
    wrench->force.z = v(2);
    wrench->torque.x = v(3);
    wrench->torque.y = v(4);
    wrench->torque.z = v(5);}

  bool CartesianImpedanceControllerRos::initJointHandles(){
    std::vector<std::string> joint_names;
    auto node = get_node();

    node->get_parameter("joints", joint_names);
    if (joint_names.empty()){
        RCLCPP_ERROR(node->get_logger(), "Invalid or no 'joints' parameter provided, aborting controller init!");
        return false;}
    std::string hw_interface_type;
    node->get_parameter("hardware_interface", hw_interface_type);
    RCLCPP_INFO(node->get_logger(), "Available state interfaces:");
    for (const auto &interface : state_interfaces_) {
        RCLCPP_INFO(node->get_logger(), "%s", interface.get_name().c_str());
    }
    for (const auto &joint_name : joint_names){
      RCLCPP_INFO(node->get_logger(), "Requesting interface: joint: %s, interface: %s", joint_name.c_str(), hw_interface_type.c_str());
      try {
          std::string full_interface_name = joint_name + "/" + hw_interface_type;
          auto command_interface_handle = std::find_if(
              command_interfaces_.begin(),
              command_interfaces_.end(),
              [&](const auto &command_interface) {
                  return command_interface.get_name() == full_interface_name;});
          if (command_interface_handle == command_interfaces_.end()) {
              RCLCPP_ERROR(node->get_logger(), "Unable to get command interface: %s", full_interface_name.c_str());
              return false;}
          joint_command_handles_.emplace_back(std::move(*command_interface_handle));
          auto state_interface_handle = std::find_if(
              state_interfaces_.begin(),
              state_interfaces_.end(),
              [&](const auto &state_interface) {
                  return state_interface.get_name() == full_interface_name;});
          if (state_interface_handle == state_interfaces_.end()) {
              RCLCPP_ERROR(node->get_logger(), "Unable to get state interface: %s", joint_name.c_str());
              return false;}
          joint_state_handles_.emplace_back(std::move(*state_interface_handle));} 
      catch (const std::exception &ex){
          RCLCPP_ERROR(node->get_logger(), "Exception getting joint handles: %s", ex.what());
          return false;}}
    RCLCPP_INFO(node->get_logger(), "Number of joints specified in parameters: %zu", joint_names.size());
    this->setNumberOfJoints(joint_names.size());
    return true;}

  bool CartesianImpedanceControllerRos::initMessaging(){
    auto node = get_node();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    this->sub_cart_stiffness_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "set_cartesian_stiffness", qos,
        std::bind(&CartesianImpedanceControllerRos::cartesianStiffnessCb, this, std::placeholders::_1));
    this->sub_cart_wrench_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "set_cartesian_wrench", qos,
        std::bind(&CartesianImpedanceControllerRos::wrenchCommandCb, this, std::placeholders::_1));
    this->sub_damping_factors_ = node->create_subscription<geometry_msgs::msg::Wrench>(
        "set_damping_factors", qos,
        std::bind(&CartesianImpedanceControllerRos::cartesianDampingFactorCb, this, std::placeholders::_1));
    this->sub_controller_config_ = node->create_subscription<cartesian_impedance_controller::msg::ControllerConfig>(
        "set_config", qos,
        std::bind(&CartesianImpedanceControllerRos::controllerConfigCb, this, std::placeholders::_1));
    this->sub_reference_pose_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "reference_pose", qos,
        std::bind(&CartesianImpedanceControllerRos::referencePoseCb, this, std::placeholders::_1));
    this->pub_torques_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("commanded_torques", 10);
    this->pub_state_ = node->create_publisher<cartesian_impedance_controller::msg::ControllerState>("controller_state", 10);
    return true;}

  bool CartesianImpedanceControllerRos::initRBDyn(){
    std::string urdf_string;
    auto node = get_node();
    while (!node->get_parameter("robot_description", urdf_string)) {
      RCLCPP_INFO_ONCE(node->get_logger(), "Waiting for 'robot_description' parameter to be available on the ROS parameter server.");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));}
    RCLCPP_INFO(node->get_logger(), "Robot description parameter received.");
    try{
      this->rbdyn_wrapper_.init_rbdyn(urdf_string, end_effector_);}
    catch (const std::runtime_error &e){
      RCLCPP_ERROR(node->get_logger(), "Error when initializing RBDyn: %s", e.what());
      return false;}
    RCLCPP_INFO(node->get_logger(), "Number of joints found in URDF: %d", this->rbdyn_wrapper_.n_joints());
    if (this->rbdyn_wrapper_.n_joints() < this->n_joints_){
      RCLCPP_ERROR(node->get_logger(), "Number of joints in the URDF is smaller than the number of controlled joints. %d < %zu", this->rbdyn_wrapper_.n_joints(), this->n_joints_);
      return false;}
    else if (this->rbdyn_wrapper_.n_joints() > this->n_joints_){
      RCLCPP_WARN(node->get_logger(), "Number of joints in the URDF is greater than the number of controlled joints: %d > %zu. Assuming that the actuated joints come first.", this->rbdyn_wrapper_.n_joints(), this->n_joints_);}
    return true;}

  bool CartesianImpedanceControllerRos::initTrajectories(){
      auto node = get_node();
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
      this->sub_trajectory_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
          "joint_trajectory", qos,
          std::bind(&CartesianImpedanceControllerRos::trajCb, this, std::placeholders::_1));
      this->traj_as_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
          node, 
          "follow_joint_trajectory",
          std::bind(&CartesianImpedanceControllerRos::trajGoalCb, this, std::placeholders::_1, std::placeholders::_2),
          std::bind(&CartesianImpedanceControllerRos::trajCancelCb, this, std::placeholders::_1),
          std::bind(&CartesianImpedanceControllerRos::trajAcceptedCb, this, std::placeholders::_1));
      return true;}

  controller_interface::CallbackReturn CartesianImpedanceControllerRos::on_init(){
      auto node = get_node();
      RCLCPP_INFO(node->get_logger(), "Initializing Cartesian Impedance Controller");
      // setFilterCoefficient(50.0, 0.001); 
      try {
          tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
          tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
          tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
      } catch (const std::exception &e) {
          RCLCPP_ERROR(node->get_logger(), "Error initializing TF components: %s", e.what());
          return controller_interface::CallbackReturn::ERROR;
      }
      // parameter_event_subscriber_ = get_node()->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      //     "/parameter_events",
      //     10,
      //     std::bind(&CartesianImpedanceControllerRos::onParameterEvent, this, std::placeholders::_1));
      return controller_interface::CallbackReturn::SUCCESS;}

  controller_interface::CallbackReturn CartesianImpedanceControllerRos::on_configure(const rclcpp_lifecycle::State & /*previous_state*/){
      auto node = get_node();
      if (!node->get_parameter("end_effector", end_effector_)) {
          RCLCPP_ERROR(node->get_logger(), "No 'end_effector' parameter provided, aborting configuration!");
          return controller_interface::CallbackReturn::ERROR;}
      RCLCPP_INFO(node->get_logger(), "End effector link: %s", end_effector_.c_str());
      if (!this->initMessaging() || !this->initRBDyn()){
          RCLCPP_ERROR(node->get_logger(), "Failed to configure Cartesian Impedance Controller.");
          return controller_interface::CallbackReturn::ERROR;}
      RCLCPP_INFO(node->get_logger(), "Messaging and RBDyn initialized.");
      this->root_frame_ = this->rbdyn_wrapper_.root_link();
      if (this->n_joints_ < 6){RCLCPP_WARN(node->get_logger(), "Number of joints is below 6. Functions might be limited.");}
      if (this->n_joints_ < 7){RCLCPP_WARN(node->get_logger(), "Number of joints is below 7. No redundant joint for nullspace.");}
      this->tau_m_ = Eigen::VectorXd(this->n_joints_);
      bool enable_trajectories = true;
      RCLCPP_INFO(node->get_logger(), "Initializing trajectories.");
      //TODO: implement dynamic reconfigure
      // if (dynamic_reconfigure && !this->initDynamicReconfigure()){
      //     RCLCPP_ERROR(node->get_logger(), "Failed to initialize dynamic reconfigure.");
      //     return controller_interface::CallbackReturn::ERROR;}
      if (enable_trajectories && !this->initTrajectories()){
          RCLCPP_ERROR(node->get_logger(), "Failed to initialize trajectories.");
          return controller_interface::CallbackReturn::ERROR;}
      RCLCPP_INFO(node->get_logger(), "Cartesian Impedance Controller configured.");
      return controller_interface::CallbackReturn::SUCCESS;}

  controller_interface::CallbackReturn CartesianImpedanceControllerRos::on_activate(const rclcpp_lifecycle::State & /*previous_state*/){
      auto node = get_node();
      if (!this->initJointHandles()){
          RCLCPP_ERROR(node->get_logger(), "Failed to initialize joint handles.");
          return controller_interface::CallbackReturn::ERROR;}
      RCLCPP_INFO(node->get_logger(), "Initialized joint handles.");
      this->updateState();
      RCLCPP_INFO(node->get_logger(), "State updated.");
      this->initDesiredPose(this->position_, this->orientation_);
      RCLCPP_INFO(node->get_logger(), "Initialized desired pose.");
      this->initNullspaceConfig(this->q_);
      RCLCPP_INFO(node->get_logger(), "Started Cartesian Impedance Controller");
      return controller_interface::CallbackReturn::SUCCESS;}

  controller_interface::CallbackReturn CartesianImpedanceControllerRos::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
      auto node = get_node();
      this->sub_cart_stiffness_.reset();
      this->sub_cart_wrench_.reset();
      this->sub_damping_factors_.reset();
      this->sub_controller_config_.reset();
      this->sub_reference_pose_.reset();
      if (this->traj_as_goal_ && this->traj_as_goal_->is_active()) {
          auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
          this->traj_as_goal_->canceled(result);
          RCLCPP_INFO(node->get_logger(), "Canceled current trajectory goal");}
      this->traj_as_.reset();
      this->pub_torques_.reset();
      this->pub_state_.reset();
      RCLCPP_INFO(node->get_logger(), "Deactivated Cartesian Impedance Controller");
      return controller_interface::CallbackReturn::SUCCESS;}

controller_interface::InterfaceConfiguration CartesianImpedanceControllerRos::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    std::vector<std::string> joint_names;
    get_node()->get_parameter("joints", joint_names);
    std::string hw_interface_type;
    get_node()->get_parameter("hardware_interface", hw_interface_type);
    for (const auto &joint_name : joint_names) {
        command_interfaces_config.names.push_back(joint_name + "/" + hw_interface_type);
    }
    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration CartesianImpedanceControllerRos::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    std::vector<std::string> joint_names;
    get_node()->get_parameter("joints", joint_names);
    for (const auto &joint_name : joint_names) {
        state_interfaces_config.names.push_back(joint_name + "/position");
        state_interfaces_config.names.push_back(joint_name + "/effort");
        state_interfaces_config.names.push_back(joint_name + "/external_torque");
    }
    return state_interfaces_config;
}
  controller_interface::return_type CartesianImpedanceControllerRos::update(const rclcpp::Time & /*time*/, const rclcpp::Duration &period){
    if (this->traj_running_){trajUpdate();}
    this->updateState();
    this->calculateCommandedTorques();
    for (size_t i = 0; i < this->n_joints_; ++i){
      this->joint_command_handles_[i].set_value(this->tau_c_(i));}
    publishMsgsAndTf();
    return controller_interface::return_type::OK;}


  bool CartesianImpedanceControllerRos::getFk(const Eigen::VectorXd &q, Eigen::Vector3d *position,
                                              Eigen::Quaterniond *orientation){
    rbdyn_wrapper::EefState ee_state;
    // If the URDF contains more joints than there are controlled, only the state of the controlled ones are known
    if (this->rbdyn_wrapper_.n_joints() != this->n_joints_){
      Eigen::VectorXd q_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
      q_rb.head(q.size()) = q;
      ee_state = this->rbdyn_wrapper_.perform_fk(q_rb);}
    else{
      ee_state = this->rbdyn_wrapper_.perform_fk(q);}
    *position = ee_state.translation;
    *orientation = ee_state.orientation;
    return true;}

  bool CartesianImpedanceControllerRos::getJacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                                                    Eigen::MatrixXd *jacobian){
    if (this->rbdyn_wrapper_.n_joints() != this->n_joints_)
    {
      Eigen::VectorXd q_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
      q_rb.head(q.size()) = q;
      Eigen::VectorXd dq_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
      dq_rb.head(dq.size()) = dq;
      *jacobian = this->rbdyn_wrapper_.jacobian(q_rb, dq_rb);
    }
    else
    {
      *jacobian = this->rbdyn_wrapper_.jacobian(q, dq);
    }
    *jacobian = jacobian_perm_ * *jacobian;
    return true;}

//new implementation
void CartesianImpedanceControllerRos::updateState() {
    auto node = get_node();
    RCLCPP_INFO(node->get_logger(), "Updating State.");
    // for (size_t i = 0; i < joint_state_handles_.size(); ++i){
    //     const auto &handle = joint_state_handles_[i];
    //     RCLCPP_INFO(node->get_logger(), "Joint handle [%zu]: Name: %s, Value: %f",
    //                 i, handle.get_name().c_str(), handle.get_value());
    // }
    for (size_t i = 0; i < this->n_joints_; ++i) {
        this->q_(i) = this->joint_state_handles_[i * 3 + 0].get_value();
        this->tau_m_(i) = this->joint_state_handles_[i * 3 + 2].get_value();
    }
    // estimateVelocity(rclcpp::Duration(1.0 / 1000.0)); // 1 kHz loop
    getJacobian(this->q_, filtered_velocity_, &this->jacobian_);
    getFk(this->q_, &this->position_, &this->orientation_);}

  // void CartesianImpedanceControllerRos::estimateVelocity(const rclcpp::Duration& period) {
  //   double dt = period.seconds();
  //   for (size_t i = 0; i < this->n_joints_; ++i) {
  //       velocity_(i) = (this->q_(i) - prev_position_(i)) / dt;
  //       filtered_velocity_(i) = alpha_ * velocity_(i) + (1.0 - alpha_) * prev_filtered_velocity_(i);}
  //   prev_position_ = this->q_;
  //   prev_filtered_velocity_ = filtered_velocity_;}

  void CartesianImpedanceControllerRos::controllerConfigCb(const cartesian_impedance_controller::msg::ControllerConfig::SharedPtr msg){
    auto node = get_node();
    this->setStiffness(msg->cartesian_stiffness, msg->nullspace_stiffness, false);
    this->setDampingFactors(msg->cartesian_damping_factors, msg->nullspace_damping_factor);
    if (msg->q_d_nullspace.size() == this->n_joints_){
      Eigen::VectorXd q_d_nullspace(this->n_joints_);
      for (size_t i = 0; i < this->n_joints_; i++){
        q_d_nullspace(i) = msg->q_d_nullspace.at(i);}
      this->setNullspaceConfig(q_d_nullspace);}
    else{
      RCLCPP_WARN(node->get_logger(), "Nullspace configuration does not have the correct amount of entries. Got %zu expected %zu. Ignoring.", msg->q_d_nullspace.size(), this->n_joints_);}}

  void CartesianImpedanceControllerRos::cartesianDampingFactorCb(const geometry_msgs::msg::Wrench::SharedPtr msg){
    this->setDampingFactors(*msg, this->damping_factors_[6]);}

  void CartesianImpedanceControllerRos::referencePoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    auto node = get_node();
    if (!msg->header.frame_id.empty() && msg->header.frame_id != this->root_frame_){
      RCLCPP_WARN(node->get_logger(), "Reference poses need to be in the root frame '%s'. Ignoring.", this->root_frame_.c_str());
      return;}
    Eigen::Vector3d position_d;
    position_d << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    const Eigen::Quaterniond last_orientation_d_target(this->orientation_d_);
    Eigen::Quaterniond orientation_d;
    orientation_d.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(this->orientation_d_.coeffs()) < 0.0){
      this->orientation_d_.coeffs() << -this->orientation_d_.coeffs();}
    this->setReferencePose(position_d, orientation_d);}

  void CartesianImpedanceControllerRos::cartesianStiffnessCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg){
    this->setStiffness(msg->wrench, this->nullspace_stiffness_target_);}

  void CartesianImpedanceControllerRos::setDampingFactors(const geometry_msgs::msg::Wrench &cart_damping, double nullspace){
    CartesianImpedanceController::setDampingFactors(saturateValue(cart_damping.force.x, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.force.y, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.force.z, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.torque.x, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.torque.y, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.torque.z, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(nullspace, dmp_factor_min_, dmp_factor_max_));}

  void CartesianImpedanceControllerRos::setStiffness(const geometry_msgs::msg::Wrench &cart_stiffness, double nullspace, bool auto_damping){
    CartesianImpedanceController::setStiffness(saturateValue(cart_stiffness.force.x, trans_stf_min_, trans_stf_max_),
                                               saturateValue(cart_stiffness.force.y, trans_stf_min_, trans_stf_max_),
                                               saturateValue(cart_stiffness.force.z, trans_stf_min_, trans_stf_max_),
                                               saturateValue(cart_stiffness.torque.x, rot_stf_min_, rot_stf_max_),
                                               saturateValue(cart_stiffness.torque.y, rot_stf_min_, rot_stf_max_),
                                               saturateValue(cart_stiffness.torque.z, rot_stf_min_, rot_stf_max_),
                                               saturateValue(nullspace, ns_min_, ns_max_), auto_damping);}

  void CartesianImpedanceControllerRos::wrenchCommandCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg){
    auto node = get_node();
    Eigen::Matrix<double, 6, 1> F;
    F << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
    if (!msg->header.frame_id.empty() && msg->header.frame_id != this->root_frame_){
      if (!transformWrench(&F, msg->header.frame_id, this->root_frame_)){
        RCLCPP_ERROR(node->get_logger(), "Could not transform wrench. Not applying it.");
        return;}}
    else if (msg->header.frame_id.empty()){
      if (!transformWrench(&F, this->wrench_ee_frame_, this->root_frame_)){
        RCLCPP_ERROR(node->get_logger(), "Could not transform wrench. Not applying it.");
        return;}}
    this->applyWrench(F);}

  bool CartesianImpedanceControllerRos::transformWrench(Eigen::Matrix<double, 6, 1> *cartesian_wrench,
                                                        const std::string &from_frame, const std::string &to_frame) const{
      try{
          auto node = get_node();
          geometry_msgs::msg::TransformStamped transform;
          transform = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
          Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform);
          Eigen::Vector3d v_f(cartesian_wrench->operator()(0), cartesian_wrench->operator()(1), cartesian_wrench->operator()(2));
          Eigen::Vector3d v_t(cartesian_wrench->operator()(3), cartesian_wrench->operator()(4), cartesian_wrench->operator()(5));
          Eigen::Vector3d v_f_rot = transform_eigen.rotation() * v_f;
          Eigen::Vector3d v_t_rot = transform_eigen.rotation() * v_t;
          *cartesian_wrench << v_f_rot[0], v_f_rot[1], v_f_rot[2], v_t_rot[0], v_t_rot[1], v_t_rot[2];
          return true;}
      catch (const tf2::TransformException &ex){
          static rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
          RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *clock, 1000, "%s", ex.what());
          return false;}}

  void CartesianImpedanceControllerRos::publishMsgsAndTf(){
      auto node = get_node();
      auto commanded_torques_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
      commanded_torques_msg->data.resize(this->n_joints_);
      for (size_t i = 0; i < this->n_joints_; i++){
          commanded_torques_msg->data[i] = this->tau_c_(i);}
      this->pub_torques_->publish(*commanded_torques_msg);
      const Eigen::Matrix<double, 6, 1> error{this->getPoseError()};
      if (this->verbose_print_){
          RCLCPP_INFO(node->get_logger(), "\nCartesian Position:\n%f %f %f\nError:\n%f %f %f\nCartesian Stiffness:\n%f %f %f\nCartesian damping:\n%f %f %f\nNullspace stiffness:\n%f\nq_d_nullspace:\n%f\n",
                      this->position_(0), this->position_(1), this->position_(2), error(0), error(1), error(2),
                      this->cartesian_stiffness_(0, 0), this->cartesian_stiffness_(1, 1), this->cartesian_stiffness_(2, 2),
                      this->cartesian_damping_(0, 0), this->cartesian_damping_(1, 1), this->cartesian_damping_(2, 2),
                      this->nullspace_stiffness_, this->q_d_nullspace_(0));}
      if (this->verbose_tf_ && (node->now() - this->tf_last_time_).seconds() > 1.0){
          geometry_msgs::msg::TransformStamped transform_stamped;
          transform_stamped.header.stamp = node->now();
          transform_stamped.header.frame_id = this->root_frame_;
          transform_stamped.child_frame_id = this->end_effector_ + "_ee_fk";
          tf2::Transform tf_transform(tf2::Quaternion(this->orientation_.x(), this->orientation_.y(), this->orientation_.z(), this->orientation_.w()),
                                      tf2::Vector3(this->position_.x(), this->position_.y(), this->position_.z()));
          transform_stamped.transform.translation.x = tf_transform.getOrigin().x();
          transform_stamped.transform.translation.y = tf_transform.getOrigin().y();
          transform_stamped.transform.translation.z = tf_transform.getOrigin().z();
          transform_stamped.transform.rotation.x = tf_transform.getRotation().x();
          transform_stamped.transform.rotation.y = tf_transform.getRotation().y();
          transform_stamped.transform.rotation.z = tf_transform.getRotation().z();
          transform_stamped.transform.rotation.w = tf_transform.getRotation().w();
          tf_broadcaster_->sendTransform(transform_stamped);
          transform_stamped.child_frame_id = this->end_effector_ + "_ee_ref_pose";
          tf2::Transform tf_transform_d(tf2::Quaternion(this->orientation_d_.x(), this->orientation_d_.y(), this->orientation_d_.z(), this->orientation_d_.w()),
                                        tf2::Vector3(this->position_d_.x(), this->position_d_.y(), this->position_d_.z()));
          transform_stamped.transform.translation.x = tf_transform_d.getOrigin().x();
          transform_stamped.transform.translation.y = tf_transform_d.getOrigin().y();
          transform_stamped.transform.translation.z = tf_transform_d.getOrigin().z();
          transform_stamped.transform.rotation.x = tf_transform_d.getRotation().x();
          transform_stamped.transform.rotation.y = tf_transform_d.getRotation().y();
          transform_stamped.transform.rotation.z = tf_transform_d.getRotation().z();
          transform_stamped.transform.rotation.w = tf_transform_d.getRotation().w();
          tf_broadcaster_->sendTransform(transform_stamped);
          this->tf_last_time_ = node->now();}
      if (this->verbose_state_){
          auto state_msg = std::make_shared<cartesian_impedance_controller::msg::ControllerState>();
          state_msg->header.stamp = node->now();
          state_msg->current_pose.position = tf2::toMsg(this->position_);
          state_msg->current_pose.orientation = tf2::toMsg(this->orientation_);
          state_msg->reference_pose.position = tf2::toMsg(this->position_d_);
          state_msg->reference_pose.orientation = tf2::toMsg(this->orientation_d_);
          Eigen::Vector3d position_error = error.head(3);
          state_msg->pose_error.position = tf2::toMsg(position_error);
          Eigen::Quaterniond q = Eigen::AngleAxisd(error(3), Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(error(4), Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(error(5), Eigen::Vector3d::UnitZ());
          state_msg->pose_error.orientation = tf2::toMsg(q);
          EigenVectorToWrench(this->cartesian_stiffness_.diagonal(), &state_msg->cartesian_stiffness);
          EigenVectorToWrench(this->cartesian_damping_.diagonal(), &state_msg->cartesian_damping);
          EigenVectorToWrench(this->getAppliedWrench(), &state_msg->commanded_wrench);
          for (size_t i = 0; i < this->n_joints_; i++){
              state_msg->joint_state.position.at(i) = this->q_(i);
              state_msg->joint_state.velocity.at(i) = this->dq_(i);
              state_msg->joint_state.effort.at(i) = this->tau_m_(i);
              state_msg->commanded_torques.at(i) = this->tau_c_(i);
              state_msg->nullspace_config.at(i) = this->q_d_nullspace_(i);
              state_msg->commanded_torques.at(i) = this->tau_c_(i);}
          state_msg->nullspace_stiffness = this->nullspace_stiffness_;
          state_msg->nullspace_damping = this->nullspace_damping_;
          const Eigen::Matrix<double, 6, 1> dx = this->jacobian_ * this->dq_;
          state_msg->cartesian_velocity = sqrt(dx(0) * dx(0) + dx(1) * dx(1) + dx(2) * dx(2));
          this->pub_state_->publish(*state_msg);}}

  void CartesianImpedanceControllerRos::trajStart(const trajectory_msgs::msg::JointTrajectory &trajectory){
    auto node = get_node();
    this->traj_duration_ = rclcpp::Duration(trajectory.points[trajectory.points.size() - 1].time_from_start);
    RCLCPP_INFO(node->get_logger(), "Starting a new trajectory with %zu points that takes %f s.", trajectory.points.size(), this->traj_duration_.seconds());
    this->trajectory_ = trajectory;
    this->traj_running_ = true;
    this->traj_start_ = node->now();
    this->traj_index_ = 0;
    trajUpdate();
    if (this->nullspace_stiffness_ < 5.){
      RCLCPP_WARN(node->get_logger(), "Nullspace stiffness is low. The joints might not follow the planned path.");}}

  void CartesianImpedanceControllerRos::trajCb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg){
      auto node = get_node();
      RCLCPP_INFO(node->get_logger(), "Received trajectory message from trajectory topic.");
      if (this->traj_as_goal_ && this->traj_as_goal_->is_active()) {
          auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
          this->traj_as_goal_->canceled(result);
          RCLCPP_INFO(node->get_logger(), "Preempted running action server goal.");}
      trajStart(*msg);}


  rclcpp_action::GoalResponse CartesianImpedanceControllerRos::trajGoalCb(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal) {
      auto node = get_node();
      RCLCPP_INFO(node->get_logger(), "Received goal request with trajectory size %zu", goal->trajectory.points.size());
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;}

  rclcpp_action::CancelResponse CartesianImpedanceControllerRos::trajCancelCb(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle){
    auto node = get_node();
    RCLCPP_INFO(node->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;}

  void CartesianImpedanceControllerRos::trajAcceptedCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle){
    std::thread{std::bind(&CartesianImpedanceControllerRos::execute, this, std::placeholders::_1), goal_handle}.detach();}

  void CartesianImpedanceControllerRos::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle){
    auto node = get_node();
    RCLCPP_INFO(node->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    trajStart(goal->trajectory);
    auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    if (rclcpp::ok() && goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(node->get_logger(), "Goal canceled");
      return;}
    goal_handle->succeed(result);
    RCLCPP_INFO(node->get_logger(), "Goal succeeded");}

  void CartesianImpedanceControllerRos::trajUpdate() {
    auto node = get_node();
    if (!this->traj_running_) { return; }
    rclcpp::Time now = node->now();
    if (now > (this->traj_start_ + trajectory_.points.at(this->traj_index_).time_from_start)) {
        Eigen::VectorXd q = Eigen::VectorXd::Map(
            trajectory_.points.at(this->traj_index_).positions.data(),
            trajectory_.points.at(this->traj_index_).positions.size());
        if (this->verbose_print_) {
            RCLCPP_INFO(node->get_logger(),
                        "Index %u q_nullspace: %f %f %f %f %f %f %f",
                        this->traj_index_, q(0), q(1), q(2), q(3), q(4), q(5), q(6));}
        getFk(q, &this->position_d_target_, &this->orientation_d_target_);
        this->setNullspaceConfig(q);
        this->traj_index_++;}
    if (now > (this->traj_start_ + this->traj_duration_)) {
        RCLCPP_INFO(node->get_logger(), "Finished executing trajectory.");
        if (this->traj_as_goal_ && this->traj_as_goal_->is_active()) {
            auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
            this->traj_as_goal_->succeed(result);}
        this->traj_running_ = false;}}
} // namespace cartesian_impedance_controller
PLUGINLIB_EXPORT_CLASS(cartesian_impedance_controller::CartesianImpedanceControllerRos, controller_interface::ControllerInterface)
