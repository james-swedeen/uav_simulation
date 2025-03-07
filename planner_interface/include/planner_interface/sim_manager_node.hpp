/**
 * @File: sim_manager_node.hpp
 * @Date: February 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Declaration of the manager node for
 * running headless Monte Carlo simulations
 **/

#ifndef PLANNER_INTERFACE__SIM_MANAGER_NODE_HPP_
#define PLANNER_INTERFACE__SIM_MANAGER_NODE_HPP_

// Local headers
#include <uav_interfaces/srv/set_clock_params.hpp>
#include <uav_interfaces/msg/uav_waypoint.hpp>
#include <uav_interfaces/msg/uav_waypoints.hpp>

// Global headers
#include <chrono>
#include <memory>
#include <mutex>
#include <boost/uuid/uuid_generators.hpp>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace planner_interface{
class SimManagerNode : public rclcpp::Node{
public:
  /**
   * @brief Constructor to initialize all parameters, flags, and interfaces
   */
  SimManagerNode();

  /**
   * @brief Public function used to start runs
   */
  void start_run();

  /**
   * @brief Reset to initial conditions after a run is finished
   */
  void reset();

  /**
   * @brief Start up the PDVG on first run. After initialization, calls to run PDVG
   * will simply reset the path manager to follow the original path again.
   */
  void initialize_path();

  /**
   * @brief Check whether the simulation has ended so shutdown can occur
   */
  bool sim_ended();

  /**
   * @brief Check whether an individual simulation run has finished
   */
  bool finished_run();

  /**
   * @brief Check if a path is planned and published
   */
  bool get_path_ready();

  /**
   * @brief Returns true if PDVG initialized or if manual waypoints ready
   */
  bool path_initialized();

  /**
   * @brief Returns true if the PDVG was properly initialized, false otherwise
   */
  bool pdvg_initialized();

  /**
   * @brief Returns true if the PDVG callback is complete
   */
  bool pdvg_callback_complete();

private:
  /* === VARIABLES === */
  /* == Parameters == */
  uint64_t m_total_runs_;    // Number of runs to perform
  double   m_clock_speed_;   // Scaling factor for simulation clock speed
  bool     m_use_pdvg_;      // Flag indicating whether PDVG is being used or manual waypoint entry

  uav_interfaces::msg::UavWaypoint m_pdvg_point_;  // Target point for PDVG planning
  uav_interfaces::msg::UavWaypoints m_manual_waypoints_; // Waypoint path for manual planning
  boost::uuids::random_generator m_uuid_gen_;            // UUID generator for waypoint path

  /* == Flags and Counters == */
  uint64_t m_current_run_number_;  // Current run being performed. Used for progress calculation
  std::chrono::time_point<std::chrono::steady_clock> m_run_start_time_;
  double m_average_run_time_;      // Average time per run

  bool m_finished_run_;                  // Flag to indicate whether run was finished
  bool m_sim_ended_;                     // Flag for checking whether to shutdown operations
  bool m_path_ready_;                    // Flag that indicates if the path manager is ready
  bool m_pdvg_initialized_;              // Flag indicating if the first PDVG usage was successful
  bool m_pdvg_cb_complete_;              // Flag indicating if the PDVG callback is complete
  bool m_truth_state_listener_stopped_;  // Flag indicating if state listener is done
  bool m_nav_state_listener_stopped_;    // Flag indicating if state listener is done
  bool m_auto_sense_reset_;              // Flag indicating if autopilot and sensor updates are paused

  std::mutex m_flag_mutex_;        // Mutex used to prevent race conditions on flags


  /* == Service Clients == */
  // Generic empty request
  std::shared_ptr<std_srvs::srv::Empty::Request> m_empty_req_;

  // Service used to set simulation clock speed
  rclcpp::Client<uav_interfaces::srv::SetClockParams>::SharedPtr m_clock_client_;

  // Service used to activate PDVG and get waypoints from it
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_pdvg_client_;

  // Service used to reset truth and nav state so another run can begin
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr m_kf_reset_client_;

  // For pausing operation of the kalman filter
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr m_kf_start_pause_client_;

  // For resetting truth state
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr m_state_reset_client_;

  // For toggling operation of the state_listener (default off)
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr m_truth_state_listener_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr m_nav_state_listener_client_;

  // For resetting and/or ending state_listener
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_truth_state_listener_reset_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_nav_state_listener_reset_client_;

  // For toggling operation of dynamics node (default off)
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr m_dynamics_toggle_client_;

  // For resetting autopilot controller
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr m_autopilot_reset_client_;

  /* == Publishers == */
  // Publisher used to republish waypoints at the start of every simulation run
  rclcpp::Publisher<uav_interfaces::msg::UavWaypoints>::SharedPtr m_waypoints_pub_;

  // Publisher to show diagnostic information
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr m_diagnostics_timer_;  // Timer to set publish period of diagnostics
  diagnostic_msgs::msg::DiagnosticStatus m_diagnostic_status_;  // Variable to maintain status updates


  /* == Subscribers == */
  // Subcriber to know when last waypoint has been reached
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_end_run_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_path_ready_sub_;

  rclcpp::CallbackGroup::SharedPtr m_cb_group_;

  /* === FUNCTIONS === */
  /**
   * @brief If a service is unavailable, find the reason why and report as a log message
   *
   * @param srv_name: Name of the service in question
   */
  void check_service(const std::string &srv_name);

  /**
   * @brief Toggle the dynamics, data recording, and kalman filter operation
   */
  void toggle_services_();

  /* == Callbacks == */
  /**
   * @brief Reset when run has ended and start another run
   */
  void end_run_callback_(const std_msgs::msg::Empty&);

  /**
   * @brief Set flag as to whether the path manager is ready
   */
  void path_ready_callback_(const std_msgs::msg::Empty&);

  /**
   * @brief Set flags to know when UavWaypoints have been published
   *
   * @param response: Trigger response from PDVG interface
   */
  void pdvg_callback_(const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response);

  /**
   * @brief If needed, process response value if failed, else continue
   *
   * @param response: Response with info about whether or not setting clock speed succeeded
   */
  void set_clock_callback_(const rclcpp::Client<uav_interfaces::srv::SetClockParams>::SharedFuture response);

  /**
   * @brief Arbitrary callback for handling empty response
   *
   * @param response: Empty response
   */
  void kf_reset_callback_(const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture response);

  /**
   * @brief Arbitrary callback for handling empty response
   *
   * @param response: Empty response
   */
  void kf_start_pause_callback_(const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture response);

  /**
   * @brief Arbitrary callback for handling empty response
   *
   * @param response: Empty response
   */
  void state_reset_callback_(const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture response);

  /**
   * @brief Arbitrary callback for handling empty response
   *
   * @param response: Empty response
   */
  void dyn_toggle_callback_(const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture response);

  /**
   * @brief Arbitrary callback for handling empty response
   *
   * @param response: Empty response
   */
  void truth_state_listener_toggle_callback_(const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture response);

  /**
   * @brief Arbitrary callback for handling quick response. Entering callback assumes
   * successful toggle of the state listener
   */
  void truth_state_listener_reset_callback_(const rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response);

  /**
   * @brief Arbitrary callback for handling empty response
   *
   * @param response: Empty response
   */
  void nav_state_listener_toggle_callback_(const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture response);

  /**
   * @brief Arbitrary callback for handling quick response. Entering callback assumes
   * successful toggle of the state listener
   */
  void nav_state_listener_reset_callback_(const rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response);

  /**
   * @brief Arbitrary callback for handling quick response. Entering callback assumes
   * successful reset of autopilot
   */
  void reset_autopilot_callback_(const rclcpp::Client<std_srvs::srv::Empty>::SharedFuture response);

  /* == Publishing functions == */
  /**
   * @brief Publish the target point for PDVG interface to use in solution
   */
  void publish_pdvg_point_();

  /**
   * @brief Publish manual waypoints if the PDVG interface is not being used
   */
  void publish_manual_waypoints_();

  /**
   * @brief Publish current diagnostic information
   */
  void publish_diagnostics_();

  /**
   * @brief Update the current diagnostic status of the node
   *
   * @param status: Status to make current status (WARN, OK, ERROR)
   * @param message: Message to display in diagnostic monitor
   */
  void log_diagnostics_(const diagnostic_msgs::msg::DiagnosticStatus::_level_type &level,
                        const std::string &message);
};
}

#endif  // PLANNER_INTERFACE__SIM_MANAGER_NODE_HPP_