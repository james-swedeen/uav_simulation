/**
 * @File: state_listener_csv_node.hpp
 * @Date: January 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Declaration of a listener node for saving truth or navigation
 * state data to CSV files.
 *
 * Last Updated: 4/24/23
 **/

#ifndef PLANNER_INTERFACE__STATE_LISTENER_CSV_NODE_HPP_
#define PLANNER_INTERFACE__STATE_LISTENER_CSV_NODE_HPP_

// Global headers
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <uav_interfaces/msg/uav_state.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace planner_interface{
/**
 * @brief The StateListenerCSVNode provides a way to save
 * off data during headless runs. A few parameters dictate
 * where the data is saved and how big the files can be before
 * creating new files.
 */
class StateListenerCSVNode : public rclcpp::Node{
public:

  /**
   * @brief Construct a new state listener by setting up topics and creating
   * a data directory
   */
  StateListenerCSVNode();

  /**
   * @brief Close files when ROS shutdown occurs to prevent file corruption
   */
  inline void shutdown_callback();

private:

  /* === VARIABLES === */
  /* == Parameters == */
  // Prefixes for file and directory identification
  const static std::string  TRUTH_FILE_PREFIX;
  const static std::string  NAV_FILE_PREFIX;
  const static std::string  RUN_DIR_NAME;

  bool                  m_new_run_;             // Flag set to true if new run started
  uint64_t              m_row_limit_;           // Max number of rows before new file
  uint64_t              m_csv_precision_;       // Number of sig-figs when writing files
  std::string           m_data_directory_top_;  // Top level directory
  std::filesystem::path m_run_directory_;       // Directory with run number

  /* == Files == */
  std::ofstream m_file_;          // File to store truth state data to
  std::string   m_file_prefix_;   // Prefix to use for creating files (truth or nav)
  uint64_t      m_row_count_;     // Row count for truth state file
  uint64_t      m_file_count_;    // File counter for incrementing file number
  uint64_t      m_run_number_;    // Number of the current run

  /* == Start/Stop Operations == */
  bool m_is_running_;

  /* == Tools for Clean Data == */
  double m_prev_time_;  // Track previous time so as to prevent storing repeated data

  // Modify this as new parts of the state get added to the UavState message
  struct StateHeaders{
    // Time header
    inline static const std::string TIME = "Time (s)";

    // Position headers
    inline static const std::string POS_X = "Position X";
    inline static const std::string POS_Y = "Position Y";
    inline static const std::string POS_Z = "Position Z";

    // Orientation Headers
    inline static const std::string PHI   =   "Euler Phi";
    inline static const std::string THETA = "Euler Theta";
    inline static const std::string PSI   =   "Euler Psi";

    // Velocity headers
    inline static const std::string VEL_X = "Velocity X";
    inline static const std::string VEL_Y = "Velocity Y";
    inline static const std::string VEL_Z = "Velocity Z";

    // Gyro Bias headers
    inline static const std::string GYRO_X = "Gyro Bias X";
    inline static const std::string GYRO_Y = "Gyro Bias Y";
    inline static const std::string GYRO_Z = "Gyro Bias Z";
  };

  /* == Subscribers == */
  // Truth and navigation state subscriptions
  rclcpp::Subscription<uav_interfaces::msg::UavState>::SharedPtr m_state_sub_;

  // Toggle and reset services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_toggle_collection_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_reset_srv_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr m_cb_group_;

  /* === FUNCTIONS === */
  /* == Callbacks == */
  /**
   * @brief Callback that writes state data to file
   *
   * @param state: Published state data
   */
  void state_callback_(const uav_interfaces::msg::UavState &state);

  /**
   * @brief Change a flag to pause operation once a path is finished
   *
   * @param request: Empty, not utilized
   * @param response: Empty, not utilized
   */
  void toggle_execution_callback_(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /**
   * @brief Update run number and create a new run directory
   *
   * @param request: Empty, not utilized
   * @param response: Empty, not utilized
   */
  void run_complete_callback_(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  /* == Helper functions == */
  /**
   * @brief Write the headers to a newly opened CSV file
   *
   * @param file: File object to write headers to
   */
  inline void write_csv_headers_();

  /**
   * @brief Write new state data to file
   *
   * @param state: State data to write to CSV file
   * @param file: File object to write headers to
   */
  inline void write_data_(const uav_interfaces::msg::UavState &state);

  /**
   * @brief Create a new CSV file
   *
   * @param file: File object to create new file from
   * @param file_prefix: String to put at front of file (e.g. "truth", or "nav")
   * @param file_number: Number to put at end of file (e.g. "001", "002", etc.)
   */
  inline void create_file_();
};
}  // namespace planner_interface

#endif  // PLANNER_INTERFACE__STATE_LISTENER_CSV_NODE_HPP_