/**
 * @File: sim_manager_node.cpp
 * @Date: February 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Definition of the manager node for
 * running headless Monte Carlo simulations
 **/

// Local headers
#include <planner_interface/sim_manager_node.hpp>
#include <uav_interfaces/srv/set_clock_params.hpp>
#include <uav_interfaces/msg/uav_waypoints.hpp>
#include <uav_interfaces/msg/uav_waypoint.hpp>

// Global headers
#include <memory>
#include <chrono>
#include <string>
#include <sstream>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using namespace std::placeholders;
using namespace std::chrono_literals;
namespace planner_interface{
/**
 * @brief Constructor to initialize all parameters, flags, and interfaces
 */
SimManagerNode::SimManagerNode() : Node("sim_manager_node") {
  // Declare and set parameters
  this->declare_parameter("total_runs", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("clock_speed", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("use_pdvg", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("use_kalman_filter", rclcpp::PARAMETER_BOOL);

  this->declare_parameter("nominal_velocity", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("nominal_down", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("pdvg_x_pos", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("pdvg_y_pos", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("manual_wp_x", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("manual_wp_y", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("path_type", rclcpp::PARAMETER_STRING);
  this->declare_parameter("min_radius", rclcpp::PARAMETER_DOUBLE);

  this->m_total_runs_  = this->get_parameter("total_runs").as_int();
  this->m_clock_speed_ = this->get_parameter("clock_speed").as_double();
  this->m_use_pdvg_    = this->get_parameter("use_pdvg").as_bool();

  if (this->m_use_pdvg_){
    // Set PDVG waypoint
    this->m_pdvg_point_.position.point.x = this->get_parameter("pdvg_x_pos").as_double();
    this->m_pdvg_point_.position.point.y = this->get_parameter("pdvg_y_pos").as_double();
    this->m_pdvg_point_.position.point.z = this->get_parameter("nominal_down").as_double();
    this->m_pdvg_point_.airspeed         = this->get_parameter("nominal_velocity").as_double();
  } else {
    // Set manual waypoints
    auto x_points = this->get_parameter("manual_wp_x").as_double_array();
    auto y_points = this->get_parameter("manual_wp_y").as_double_array();

    for (size_t pt = 0; pt < x_points.size(); pt++){
      uav_interfaces::msg::UavWaypoint point;
      point.position.header.frame_id = "ned";
      point.position.point.x = x_points[pt];
      point.position.point.y = y_points[pt];
      point.position.point.z = this->get_parameter("nominal_down").as_double();
      point.airspeed         = this->get_parameter("nominal_velocity").as_double();

      // Put point in vector
      this->m_manual_waypoints_.points.emplace_back(point);
    }

    this->m_manual_waypoints_.type = this->get_parameter("path_type").as_string();
    this->m_manual_waypoints_.min_radius = this->get_parameter("min_radius").as_double();
  }


  // Initialize other member variables
  this->m_current_run_number_           = 0;
  this->m_finished_run_                 = true;  // True for initial run
  this->m_sim_ended_                    = false;
  this->m_path_ready_                   = false;
  this->m_pdvg_initialized_             = false;
  this->m_pdvg_cb_complete_             = true;
  this->m_truth_state_listener_stopped_ = true;
  this->m_nav_state_listener_stopped_   = true;
  this->m_auto_sense_reset_             = true;

  // Initialize all service clients
  this->m_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->m_empty_req_ = std::make_shared<std_srvs::srv::Empty::Request>();

  // For changing clock speed
  this->m_clock_client_ = this->create_client<uav_interfaces::srv::SetClockParams>(
    "set_clock_params",
    rmw_qos_profile_services_default,
    this->m_cb_group_);

  // For republishing waypoints
  this->m_pdvg_client_ = this->create_client<std_srvs::srv::Trigger>(
    "run_pdvg",
    rmw_qos_profile_services_default,
    this->m_cb_group_);

  // For resetting kalman filter and in turn the /uav_state_estimate values
  this->m_kf_reset_client_ = this->create_client<std_srvs::srv::Empty>(
    "kalman_filter_reset_srv_topic",
    rmw_qos_profile_services_default,
    this->m_cb_group_);

  // For pausing operation of the kalman filter
  this->m_kf_start_pause_client_ = this->create_client<std_srvs::srv::Empty>(
    "kalman_filter_start_pause_srv_topic",
    rmw_qos_profile_services_default,
    this->m_cb_group_);

  // For toggling operation of the state_listener (default off)
  this->m_truth_state_listener_client_ = this->create_client<std_srvs::srv::Empty>(
    "truth_state_listener_toggle_topic",
    rmw_qos_profile_services_default,
    this->m_cb_group_);

  this->m_nav_state_listener_client_ = this->create_client<std_srvs::srv::Empty>(
    "nav_state_listener_toggle_topic",
    rmw_qos_profile_services_default,
    this->m_cb_group_);

  // For resetting truth state
  this->m_state_reset_client_ = this->create_client<std_srvs::srv::Empty>(
    "reset_state",
    rmw_qos_profile_services_default,
    this->m_cb_group_);

  // For resetting and/or ending state listener
  this->m_truth_state_listener_reset_client_ = this->create_client<std_srvs::srv::SetBool>(
    "reset_truth_listener",
    rmw_qos_profile_services_default,
    this->m_cb_group_);

  this->m_nav_state_listener_reset_client_ = this->create_client<std_srvs::srv::SetBool>(
    "reset_nav_listener",
    rmw_qos_profile_services_default,
    this->m_cb_group_);

  // For toggling operation of dynamics node (default off)
  this->m_dynamics_toggle_client_ = this->create_client<std_srvs::srv::Empty>(
    "toggle_execution",
    rmw_qos_profile_services_default,
    this->m_cb_group_);

  // For resetting autopilot controller
  this->m_autopilot_reset_client_ = this->create_client<std_srvs::srv::Empty>(
    "reset_autopilot",
    rmw_qos_profile_services_default,
    this->m_cb_group_);

  // Initialize publishers
  this->m_waypoints_pub_ =
    this->create_publisher<uav_interfaces::msg::UavWaypoints>("waypoints", 1);

  this->m_diagnostics_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);

  this->m_diagnostics_timer_ =
    rclcpp::create_timer(
      this,
      this->get_clock(),
      1.0s,
      std::bind(&SimManagerNode::publish_diagnostics_, this),
      this->m_cb_group_);

  // Initialize subscribers
  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = this->m_cb_group_;

  // Subscriber that checks if the UAV has reached the end of the path
  this->m_end_run_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "end_path_reached",
    10,
    std::bind(&SimManagerNode::end_run_callback_, this, _1),
    sub_options);

  // Subscriber that checks whether the path manager is ready with the newest path
  this->m_path_ready_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "path_ready",
    10,
    std::bind(&SimManagerNode::path_ready_callback_, this, _1),
    sub_options);

  // Set clock speed
  // Check if service is available (give a bit more time since this is in startup)
  while(!this->m_clock_client_->wait_for_service(1s)){
    // Notify user if ROS shutdown has happened
    if (!rclcpp::ok()){
      this->log_diagnostics_(DiagnosticStatus::ERROR, "ROS Shutdown occurred");
      exit(EXIT_FAILURE);
    }
    // Otherwise, the service is unavailable
    this->log_diagnostics_(DiagnosticStatus::WARN, "Set clock params service unavailable");
  }
  while(!this->m_dynamics_toggle_client_->wait_for_service(1s)){
    this->check_service("Dynamics Toggle");
  }
  while(!this->m_truth_state_listener_client_->wait_for_service(1s)){
    this->check_service("Truth State Listener Toggle");
  }
  while(!this->m_nav_state_listener_client_->wait_for_service(1s)){
    this->check_service("Nav State Listener Toggle");
  }
  while(this->get_parameter("use_kalman_filter").as_bool() and not this->m_kf_start_pause_client_->wait_for_service(1s)){
    this->check_service("Kalman Filter Start/Pause");
  }
  while(!this->m_state_reset_client_->wait_for_service(1s)){
    this->check_service("Reset state");
  }
  while(this->get_parameter("use_kalman_filter").as_bool() and not this->m_kf_reset_client_->wait_for_service(1s)){
    this->check_service("Reset KF state");
  }
  while(!this->m_autopilot_reset_client_->wait_for_service(1s)){
    this->check_service("Reset Autopilot Controller");
  }
  while(!this->m_truth_state_listener_reset_client_->wait_for_service(1s)){
    this->check_service("Truth State Listener Reset");
  }
  while(!this->m_nav_state_listener_reset_client_->wait_for_service(1s)){
    this->check_service("Nav State Listener Reset");
  }

  // Make request
  auto clock_request = std::make_shared<uav_interfaces::srv::SetClockParams::Request>();
  clock_request->time_scale = this->m_clock_speed_;

  this->m_clock_client_->async_send_request(
    clock_request,
    std::bind(&SimManagerNode::set_clock_callback_, this, _1));

  // Publish initial status
  this->m_diagnostic_status_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  this->m_diagnostic_status_.name = "Sim Manager";
  this->m_diagnostic_status_.hardware_id = "sim";
  this->m_diagnostic_status_.message = "Initialized";
  this->publish_diagnostics_();
}

/* === PUBLIC FUNCTIONS === */
/**
 * @brief Public function used to start runs
 */
void SimManagerNode::start_run(){
  // Make all necessary service requests to get things going
  // Set flags since starting services again
  std::lock_guard<std::mutex>(this->m_flag_mutex_);
  this->m_finished_run_ = false;
  this->m_truth_state_listener_stopped_ = false;
  this->m_nav_state_listener_stopped_ = false;
  this->m_auto_sense_reset_ = false;

  // At this point, ready to follow path, so start data collection services again
  this->reset();
  std::this_thread::sleep_for(std::chrono::seconds(10));
  this->toggle_services_();  // Back on
}

/**
 * @brief Reset to initial conditions after a run is finished
 */
void SimManagerNode::reset(){
  // Reset truth state (sensors node)
  this->m_state_reset_client_->async_send_request(
    this->m_empty_req_,
    std::bind(&SimManagerNode::kf_reset_callback_, this, _1));

  // Reset kalman filter state
  this->m_kf_reset_client_->async_send_request(
    this->m_empty_req_,
    std::bind(&SimManagerNode::kf_reset_callback_, this, _1));

  // Reset autopilot controller
  this->m_autopilot_reset_client_->async_send_request(
    this->m_empty_req_,
    std::bind(&SimManagerNode::reset_autopilot_callback_, this, _1));

  // Reset truth state listener
  auto reset_req = std::make_shared<std_srvs::srv::SetBool::Request>();
  reset_req->data = false;  // Send false to indicate not end of sim

  this->m_truth_state_listener_reset_client_->async_send_request(
    reset_req,
    std::bind(&SimManagerNode::truth_state_listener_reset_callback_, this, _1));

  // Reset nav state listener
  this->m_nav_state_listener_reset_client_->async_send_request(
    reset_req,
    std::bind(&SimManagerNode::nav_state_listener_reset_callback_, this, _1));

  // Rerun PDVG or manual waypoints to reset path following

  if (this->m_use_pdvg_){
    if (!this->m_pdvg_client_->wait_for_service(1s)){
      this->check_service("Run PDVG");
    }

    this->m_pdvg_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      std::bind(&SimManagerNode::pdvg_callback_, this, _1));
  } else {
    // Republish manual waypoints
    this->publish_manual_waypoints_();
  }
}

/**
 * @brief Start up the PDVG on first run. After initialization, calls to run PDVG
 * will simply reset the path manager to follow the original path again.
 */
void SimManagerNode::initialize_path(){
  // Publish the waypoint
  if (this->m_use_pdvg_){
    this->publish_pdvg_point_();
    // Make call to run pdvg
    if (!this->m_pdvg_client_->wait_for_service(1s)){
      this->check_service("Run PDVG");
    }

    auto trigger = std::make_shared<std_srvs::srv::Trigger::Request>();
    this->m_pdvg_client_->async_send_request(
      trigger,
      std::bind(&SimManagerNode::pdvg_callback_, this, _1));

    std::lock_guard<std::mutex>(this->m_flag_mutex_);
    this->m_pdvg_cb_complete_ = false;
  } else {
    // Publish manual waypoints and then wait for set path
    this->publish_manual_waypoints_();
  }
}

/**
 * @brief Check whether the simulation has ended so shutdown can occur
 */
bool SimManagerNode::sim_ended(){
  std::lock_guard<std::mutex>(this->m_flag_mutex_);
  return this->m_sim_ended_ && this->m_truth_state_listener_stopped_ && this->m_nav_state_listener_stopped_ && this->m_auto_sense_reset_;
}

/**
 * @brief Check whether an individual simulation run has finished
 */
bool SimManagerNode::finished_run(){
  std::lock_guard<std::mutex>(this->m_flag_mutex_);
  return this->m_finished_run_;
}

/**
 * @brief Check if the PDVG Interface node has a path planned and published
 */
bool SimManagerNode::get_path_ready(){
  std::lock_guard<std::mutex>(this->m_flag_mutex_);
  return this->m_path_ready_ && this->m_truth_state_listener_stopped_ && this->m_nav_state_listener_stopped_ && this->m_auto_sense_reset_;
}

/**
 * @brief Returns true if PDVG initialized or if manual waypoints ready
 */
bool SimManagerNode::path_initialized(){
  std::lock_guard<std::mutex>(this->m_flag_mutex_);
  return (this->get_path_ready()) || (this->pdvg_initialized() && this->pdvg_callback_complete());
}

/**
 * @brief Returns true if the PDVG was properly initialized, false otherwise
 */
bool SimManagerNode::pdvg_initialized(){
  std::lock_guard<std::mutex>(this->m_flag_mutex_);
  return this->m_pdvg_initialized_;
}

/**
 * @brief Returns true if the PDVG callback is complete
 */
bool SimManagerNode::pdvg_callback_complete(){
  std::lock_guard<std::mutex>(this->m_flag_mutex_);
  return this->m_pdvg_cb_complete_;
}

/**
 * @brief If a service is unavailable, find the reason why and report as a log message
 *
 * @param srv_name: Name of the service in question
 */
void SimManagerNode::check_service(const std::string &srv_name){
  // Notify user if ROS shutdown has happened
  if (!rclcpp::ok()){
    this->log_diagnostics_(DiagnosticStatus::ERROR, "ROS Shutdown occurred");
    exit(EXIT_FAILURE);
  }
  // Otherwise, the service is unavailable
  std::string msg = srv_name + " service unavailable";
  this->log_diagnostics_(DiagnosticStatus::WARN, msg.c_str());
  // exit(EXIT_FAILURE);
}

/**
 * @brief Toggle the dynamics, data recording, and kalman filter operation
 */
void SimManagerNode::toggle_services_(){
  using namespace std::chrono_literals;
  // Toggle dynamics (/toggle_execution)
  this->m_truth_state_listener_client_->async_send_request(
    this->m_empty_req_,
    std::bind(&SimManagerNode::truth_state_listener_toggle_callback_, this, _1));

  this->m_nav_state_listener_client_->async_send_request(
    this->m_empty_req_,
    std::bind(&SimManagerNode::nav_state_listener_toggle_callback_, this, _1));

  this->m_dynamics_toggle_client_->async_send_request(
    this->m_empty_req_,
    std::bind(&SimManagerNode::dyn_toggle_callback_, this, _1));

  this->m_kf_start_pause_client_->async_send_request(
    this->m_empty_req_,
    std::bind(&SimManagerNode::kf_start_pause_callback_, this, _1));
}

/* === CALLBACKS === */
/**
 * @brief Reset when run has ended and start another run
 */
void SimManagerNode::end_run_callback_(const std_msgs::msg::Empty&){
  std::lock_guard<std::mutex>(this->m_flag_mutex_);
  // Toggle services off
  this->toggle_services_();
  this->m_current_run_number_++;

  // Calculate average time per run
  // Get average times in minutes
  auto end_run_time = std::chrono::steady_clock::now();
  auto time_diff =
    std::chrono::duration_cast<std::chrono::seconds>(end_run_time - this->m_run_start_time_);

  double numerator =
    (this->m_average_run_time_ * (this->m_current_run_number_-1)) + time_diff.count();

  // Convert manually to get fractional minutes
  double denominator = 60 * this->m_current_run_number_;

  this->m_average_run_time_ = numerator/denominator;

  // Create string with progress displayed
  std::stringstream progress;
  progress << "(" << this->m_current_run_number_ << "/" << this->m_total_runs_
           << ") runs completed || Expected time remaining: " << std::fixed << std::setprecision(2)
           << this->m_average_run_time_ * (this->m_total_runs_ - this->m_current_run_number_)
           << " min.";
  this->log_diagnostics_(DiagnosticStatus::OK, progress.str());

  // If number of runs exceeds the total desired, shut down
  if (this->m_current_run_number_ >= m_total_runs_){
    // Stop truth state listener operation
    if (!this->m_truth_state_listener_reset_client_->wait_for_service(1s)){
      this->check_service("State Listener Reset");
    }

    auto reset_req = std::make_shared<std_srvs::srv::SetBool::Request>();
    reset_req->data = true;  // Send false to indicate not end of sim

    this->m_truth_state_listener_reset_client_->async_send_request(
      reset_req,
      std::bind(&SimManagerNode::truth_state_listener_reset_callback_, this, _1));

    // Stop nav state listener operation
    if (!this->m_nav_state_listener_reset_client_->wait_for_service(1s)){
      this->check_service("Nav Listener Reset");
    }

    this->m_nav_state_listener_reset_client_->async_send_request(
      reset_req,
      std::bind(&SimManagerNode::nav_state_listener_reset_callback_, this, _1));

    // Trigger shut down
    this->m_sim_ended_ = true;
    this->m_finished_run_ = false;  // Prevent additional run attempts if kept waiting
    return;
  }

  // Set flags such that another run can be started
  this->m_finished_run_ = true;
  this->m_path_ready_ = false;
}

/**
 * @brief Set flag as to whether the path manager is ready
 */
void SimManagerNode::path_ready_callback_(const std_msgs::msg::Empty&){
  // Upon getting a publish command from the path manager, the path is ready
  std::lock_guard<std::mutex>(this->m_flag_mutex_);
  this->m_path_ready_ = true;
}

/**
 * @brief Set flags to know when UavWaypoints have been published
 *
 * @param response: Trigger response from PDVG interface
 */
void SimManagerNode::pdvg_callback_(const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response){
  std::lock_guard<std::mutex>(this->m_flag_mutex_);

  // If response is successful, set pdvg ready flag

  if (response.get()->success){
    // Set initialized flag if it hasn't been already
    if (!this->m_pdvg_initialized_){
      this->m_pdvg_initialized_ = true;
    }
  }

  this->m_pdvg_cb_complete_ = true;
}

/**
 * @brief If needed, process response value if failed, else continue
 *
 * @param response: Response with info about whether or not setting clock speed succeeded
 */
void SimManagerNode::set_clock_callback_(const rclcpp::Client<uav_interfaces::srv::SetClockParams>::SharedFuture /*response*/){
  // If successful, begin sim
  // Get initial time
  this->m_run_start_time_ = std::chrono::steady_clock::now();
}

/**
 * @brief Arbitrary callback for handling empty response
 *
 * @param response: Empty response
 */
void SimManagerNode::kf_reset_callback_(const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture /*response*/){}

/**
 * @brief Arbitrary callback for handling empty response
 *
 * @param response: Empty response
 */
void SimManagerNode::kf_start_pause_callback_(const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture /*response*/){}

/**
 * @brief Arbitrary callback for handling empty response
 *
 * @param response: Empty response
 */
void SimManagerNode::state_reset_callback_(const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture /*response*/){}

/**
 * @brief Arbitrary callback for handling empty response
 *
 * @param response: Empty response
 */
void SimManagerNode::dyn_toggle_callback_(const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture /*response*/){

}

/**
 * @brief Arbitrary callback for handling empty response
 *
 * @param response: Empty response
 */
void SimManagerNode::truth_state_listener_toggle_callback_(const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture /*response*/){}

/**
 * @brief Arbitrary callback for handling empty response
 *
 * @param response: Empty response
 */
void SimManagerNode::nav_state_listener_toggle_callback_(const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture /*response*/){}

/**
 * @brief Arbitrary callback for handling quick response. Entering callback assumes
 * successful toggle of the state listener
 */
void SimManagerNode::truth_state_listener_reset_callback_(const rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture /*response*/){
  std::lock_guard<std::mutex>(this->m_flag_mutex_);
  this->m_truth_state_listener_stopped_ = true;
}

/**
 * @brief Arbitrary callback for handling quick response. Entering callback assumes
 * successful toggle of the state listener
 */
void SimManagerNode::nav_state_listener_reset_callback_(const rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture /*response*/){
  std::lock_guard<std::mutex>(this->m_flag_mutex_);
  this->m_nav_state_listener_stopped_ = true;
}

/**
 * @brief Arbitrary callback for handling quick response. Entering callback assumes
 * successful reset of autopilot
 */
void SimManagerNode::reset_autopilot_callback_(const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture /*response*/){
  std::lock_guard<std::mutex>(this->m_flag_mutex_);
  this->m_auto_sense_reset_ = true;
}

/* == Publishing functions == */
/**
 * @brief Publish the target point for PDVG interface to use in solution
 */
void SimManagerNode::publish_pdvg_point_(){
  std::lock_guard<std::mutex>(this->m_flag_mutex_);
  this->m_path_ready_ = false;

  // Make UavWaypoints object
  uav_interfaces::msg::UavWaypoints wp;
  wp.points.emplace_back(this->m_pdvg_point_);

  // Publish the point
  this->m_waypoints_pub_->publish(wp);
}

/**
 * @brief Publish manual waypoints if the PDVG interface is not being used
 */
void SimManagerNode::publish_manual_waypoints_(){
  // Publish waypoints created at initialization
  std::lock_guard<std::mutex>(this->m_flag_mutex_);
  this->m_path_ready_ = false;

  // Create unique id
  boost::uuids::uuid uuid = this->m_uuid_gen_();

  this->m_manual_waypoints_.id = boost::uuids::to_string(uuid);
  this->m_waypoints_pub_->publish(this->m_manual_waypoints_);
}

/**
 * @brief Callback to publish current diagnostic information
 */
void SimManagerNode::publish_diagnostics_(){
  auto msg = diagnostic_msgs::msg::DiagnosticArray();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "sim_manager_node";
  msg.status.emplace_back(this->m_diagnostic_status_);

  this->m_diagnostics_pub_->publish(msg);
}

/**
 * @brief Update the current diagnostic status of the node
 *
 * @param status: Status to make current status (WARN, OK, ERROR)
 * @param message: Message to display in diagnostic monitor
 */
void SimManagerNode::log_diagnostics_(const diagnostic_msgs::msg::DiagnosticStatus::_level_type &level,
                      const std::string &message){
  this->m_diagnostic_status_.level = level;
  this->m_diagnostic_status_.message = message;
  this->publish_diagnostics_();
}
}  // end planner_interface

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);

  // Create node and loop rate
  auto node = std::make_shared<planner_interface::SimManagerNode>();
  rclcpp::Rate loop_rate(1);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  bool first_run_started = false;
  bool reset_called = false;

  while(rclcpp::ok() && !node->sim_ended()){
    // If ready to start (everything initialized and run over)
    if (node->finished_run()){
      if(!first_run_started){         // If haven't done first run yet, do setup
        // If path isn't initialized and callback is complete, run again
        if (!node->path_initialized()){
          node->initialize_path();
        }

        if (node->get_path_ready()){      // Verify path is ready before starting
          node->start_run();
          first_run_started = true;
        }
      } else {                        // Otherwise, just operate as normal
        if (!reset_called){
          node->reset();                // Reset, wait for path ready, then start
          reset_called = true;
        }
        if (node->get_path_ready()){
          node->start_run();
          reset_called = false;
        }
      }
    }
    executor.spin_some();
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
