#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/parameter.hpp>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
// #include <realtime_tools/realtime_buffer.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <cartesian_impedance_controller/cartesian_impedance_controller.h>
#include <cartesian_impedance_controller/rbdyn_wrapper.h>

#include <cartesian_impedance_controller/msg/controller_config.hpp>
#include <cartesian_impedance_controller/msg/controller_state.hpp>

namespace cartesian_impedance_controller
{
  /*! \brief The ROS control implementation of the Cartesian impedance controller
  * 
  * It utilizes a list of joint names and the URDF description to control these joints.
  */
  class CartesianImpedanceControllerRos
      : public controller_interface::ControllerInterface, public CartesianImpedanceController
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(CartesianImpedanceControllerRos);
    CartesianImpedanceControllerRos();

    /*! \brief Initializes the controller
    *
    * - Reads ROS parameters
    * - Initializes
    *   - joint handles
    *   - ROS messaging
    *   - RBDyn
    *   - rqt_reconfigure
    *   - Trajectory handling
    * \param[in] hw           Hardware interface
    * \param[in] node_handle  Node Handle
    * \return             True on success, false on failure
    */
    controller_interface::CallbackReturn on_init() override;

    /*! \brief Starts the controller
    *
    * Updates the states and sets the desired pose and nullspace configuration to the current state.
    * \param[in] time Not used
    */
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    /*! \brief Stops the controller
    *
    * Stops the controller and resets the state.
    * \param[in] time Not used
    */
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    /*! \brief Periodically called update function
    *
    * Updates the state and the trajectory. Calculated new commands and sets them.
    * Finally publishes ROS messages and tf transformations.
    * \param[in] time   Not used
    * \param[in] period Control period
    */
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;


  private:
    /*! \brief Initializes the joint handles
    *
    * Fetches the joint names from the parameter server and initializes the joint handles.
    * \param[in] hw Hardware interface to obtain handles
    * \param[in] nh Nodehandle
    * \return True on success, false on failure.
    */
    bool initJointHandles();

    /*! \brief Initializes messaging
    *
    * Initializes realtime publishers and the subscribers.
    * \param[in] nh Nodehandle
    * \return True on success, false on failure.
    */
    bool initMessaging();

    /*! \brief Initializes RBDyn
    *
    * Reads the robot URDF and initializes RBDyn.
    * \param[in] nh Nodehandle
    * \return True on success, false on failure.
    */
    bool initRBDyn();

    /*! \brief Initializes trajectory handling
    *
    * Subscribes to joint trajectory topic and starts the trajectory action server.
    * \param[in] nh Nodehandle
    * \return Always true.
    */
    bool initTrajectories();

    /*! \brief Get forward kinematics solution.
    *
    * Calls RBDyn to get the forward kinematics solution.
    * \param[in]  q            Joint position vector
    * \param[out] position     End-effector position
    * \param[out] orientation  End-effector orientation
    * \return Always true.
    */
    bool getFk(const Eigen::VectorXd &q, Eigen::Vector3d *position, Eigen::Quaterniond *rotation);

    /*! \brief Get Jacobian from RBDyn
    *
    * Gets the Jacobian for given joint positions and joint velocities.
    * \param[in]  q         Joint position vector        
    * \param[in]  dq        Joint velocity vector
    * \param[out] jacobian  Calculated Jacobian
    * \return True on success, false on failure.
    */
    bool getJacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, Eigen::MatrixXd *jacobian);

    /*! \brief Updates the state based on the joint handles.
    *
    * Gets latest joint positions, velocities and efforts and updates the forward kinematics as well as the Jacobian. 
    */
    void updateState();

    /*! \brief Sets damping for Cartesian space and nullspace.
    *
    * Long
    * \param[in] cart_damping   Cartesian damping [0,1]
    * \param[in] nullspace      Nullspace damping [0,1]
    */
    void setDampingFactors(const geometry_msgs::msg::Wrench &cart_damping, double nullspace);

    /*! \brief Sets Cartesian and nullspace stiffness
    *
    * Sets Cartesian and nullspace stiffness. Allows to set if automatic damping should be applied.
    * \param[in] cart_stiffness Cartesian stiffness
    * \param[in] nullspace      Nullspace stiffness
    * \param[in] auto_damping   Apply automatic damping 
    */
    void setStiffness(const geometry_msgs::msg::Wrench &cart_stiffness, double nullspace, bool auto_damping = true);

    /*! \brief Callback for parameter changes
    *
    * Handles parameter changes and updates the controller parameters.
    * \param[in] event The parameter event.
    */
    void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

    /*! \brief Message callback for Cartesian damping.
    *
    * Calls setDampingFactors function.
    * @sa setDampingFactors.
    * \param[in] msg Received message
    */
    void cartesianDampingFactorCb(const geometry_msgs::msg::Wrench::SharedPtr msg);

    /*! \brief Message callback for Cartesian stiffness.
    *
    * Calls setStiffness function.
    * @sa setStiffness
    * \param[in] msg Received message
    */
    void cartesianStiffnessCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

    /*! \brief Message callback for the whole controller configuration.
    *
    * Sets stiffness, damping and nullspace.
    * @sa setDampingFactors, setStiffness
    * \param[in] msg Received message
    */
    void controllerConfigCb(const cartesian_impedance_controller::msg::ControllerConfig::SharedPtr msg);

    /*! \brief Message callback for a Cartesian reference pose.
    *
    * Accepts new reference poses in the root frame - ignores them otherwise.
    * Sets the reference target pose.
    * @sa setReferencePose.
    * \param[in] msg Received message
    */
    void referencePoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /*! \brief Message callback for Cartesian wrench messages.
    *
    * If the wrench is not given in end-effector frame, it will be transformed in the root frame. Once when a new wrench message arrives.
    * Sets the wrench using the base library.
    * @sa applyWrench.
    * \param[in] msg Received message
    */
    void wrenchCommandCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

    /*! \brief Transforms the wrench in a target frame.
    *
    * Takes a vector with the wrench and transforms it to a given coordinate frame. E.g. from_frame= "world" , to_frame = "bh_link_ee"

    * @sa wrenchCommandCb
    * \param[in] cartesian_wrench Vector with the Cartesian wrench
    * \param[in] from_frame       Source frame
    * \param[in] to_frame         Target frame
    * \return True on success, false on failure.
    */
    bool transformWrench(Eigen::Matrix<double, 6, 1> *cartesian_wrench, const std::string &from_frame, const std::string &to_frame) const;

    /*! \brief Verbose printing; publishes ROS messages and tf frames.
     *
     * Always publishes commanded torques.
     * Optional: request publishes tf frames for end-effector forward kinematics and the reference pose.
     * Optional: verbose printing
     * Optional: publishes state messages
     */
    void publishMsgsAndTf();

    // realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>> traj_msg_external_point_ptr_;
    bool new_msg_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_command_subscriber_;
    void trajStart(const trajectory_msgs::msg::JointTrajectory &trajectory);
    void trajCb(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    rclcpp_action::GoalResponse trajGoalCb(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse trajCancelCb(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void trajAcceptedCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void trajUpdate();
    void declareParametersFromYAML();
    void declareParameters();

    std::vector<hardware_interface::LoanedCommandInterface> joint_command_handles_; 
    std::vector<hardware_interface::LoanedStateInterface> joint_state_handles_;
    rbdyn_wrapper rbdyn_wrapper_;   //!< Wrapper for RBDyn library for kinematics 
    std::string end_effector_;      //!< End-effector link name
    std::string robot_description_; //!< URDF of the robot
    std::string root_frame_;        //!< Base frame obtained from URDF
    Eigen::VectorXd tau_m_;         //!< Measured joint torques

    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_cart_stiffness_;    //!< Cartesian stiffness subscriber
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_cart_wrench_;       //!< Cartesian wrench subscriber
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sub_damping_factors_;           //!< Damping subscriber
    rclcpp::Subscription<cartesian_impedance_controller::msg::ControllerConfig>::SharedPtr sub_controller_config_;  //!< Controller configuration subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_reference_pose_;    //!< Cartesian reference pose subscriber
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_subscriber_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;     //!< tf transformation listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string wrench_ee_frame_;           //!< Frame for the application of the commanded wrench 

    const double trans_stf_min_{0};     //!< Minimum translational stiffness
    const double trans_stf_max_{1500};  //!< Maximum translational stiffness
    const double rot_stf_min_{0};       //!< Minimum rotational stiffness
    const double rot_stf_max_{100};     //!< Maximum rotational stiffness
    const double ns_min_{0};            //!< Minimum nullspace stiffness
    const double ns_max_{100};          //!< Maximum nullspace stiffness
    const double dmp_factor_min_{0.001};       //!< Minimum damping factor
    const double dmp_factor_max_{2.0};         //!< Maximum damping factor

    // The Jacobian of RBDyn comes with orientation in the first three lines. Needs to be interchanged.
    const Eigen::VectorXi perm_indices_ =
      (Eigen::Matrix<int, 6, 1>() << 3, 4, 5, 0, 1, 2).finished(); //!< Permutation indices to switch position and orientation
    const Eigen::PermutationMatrix<Eigen::Dynamic, 6> jacobian_perm_ =
      Eigen::PermutationMatrix<Eigen::Dynamic, 6>(perm_indices_); //!< Permutation matrix to switch position and orientation entries

    // Trajectory handling
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_trajectory_;  //!< Subscriber for a single trajectory
    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr traj_as_; //!< Trajectory action server
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> traj_as_goal_;  //!< Trajectory action server goal
    trajectory_msgs::msg::JointTrajectory trajectory_; //!< Currently played trajectory
    rclcpp::Time traj_start_;          //!< Time the current trajectory is started 
    rclcpp::Duration traj_duration_;   //!< Duration of the current trajectory
    unsigned int traj_index_{0};    //!< Index of the current trajectory point
    bool traj_running_{false};      //!< True when running a trajectory 

    // Extra output
    bool verbose_print_{false};       //!< Verbose printing enabled
    bool verbose_state_{false};       //!< Verbose state messages enabled
    bool verbose_tf_{false};          //!< Verbose tf pubishing enabled
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_torques_;  //!< Realtime publisher for commanded torques
    rclcpp::Publisher<cartesian_impedance_controller::msg::ControllerState>::SharedPtr pub_state_;  //!< Realtime publisher for controller state

    rclcpp::Time tf_last_time_; //!< Last published tf message
  };
} // namespace cartesian_impedance_controller