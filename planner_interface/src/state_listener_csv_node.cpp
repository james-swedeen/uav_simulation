/**
 * @File: state_listener_csv_node.cpp
 * @Date: January 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Definition of a listener node for saving truth or navigation
 * state data to CSV files.
 *
 * Last Updated: 4/24/23
 **/

// Local headers
#include <planner_interface/state_listener_csv_node.hpp>

// Global headers
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <uav_interfaces/msg/uav_state.hpp>

namespace planner_interface{

// Define the file and directory prefixes
const std::string StateListenerCSVNode::TRUTH_FILE_PREFIX = "truth_";
const std::string StateListenerCSVNode::NAV_FILE_PREFIX   = "nav_";
const std::string StateListenerCSVNode::RUN_DIR_NAME      = "run_";

/**
 * @brief Construct a new state listener by setting up topics and creating
 * a data directory
 */
StateListenerCSVNode::StateListenerCSVNode() : rclcpp::Node("state_listener_csv_node"){
  using namespace std::placeholders;

  // Declare and get parameters
  this->declare_parameter("row_limit", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("data_directory", rclcpp::PARAMETER_STRING);
  this->declare_parameter("precision", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("get_truth_data", rclcpp::PARAMETER_BOOL);

  this->m_row_limit_          = this->get_parameter("row_limit").as_int();
  this->m_data_directory_top_ = this->get_parameter("data_directory").as_string();
  this->m_csv_precision_      = this->get_parameter("precision").as_int();

  // Set the data to collect
  bool use_truth = this->get_parameter("get_truth_data").as_bool();
  if (use_truth){
    this->m_file_prefix_ = StateListenerCSVNode::TRUTH_FILE_PREFIX;
  } else {
    this->m_file_prefix_ = StateListenerCSVNode::NAV_FILE_PREFIX;
  }

  // Initialize other member variables
  this->m_is_running_ = false; // Start in a "paused" state
  this->m_new_run_    = true;  // Force file creation on first callback
  this->m_row_count_  = 0;
  this->m_file_count_ = 0;
  this->m_run_number_ = 0;

  // Set up callback group
  auto sub_options = rclcpp::SubscriptionOptions();
  this->m_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Initialize subscribers
  // This can be either truth or nav state depending on node creation
  this->m_state_sub_ = this->create_subscription<uav_interfaces::msg::UavState>(
    "state_topic",
    100,
    std::bind(&StateListenerCSVNode::state_callback_, this, _1),
    sub_options);

  // Initialize services
  this->m_toggle_collection_srv_ = this->create_service<std_srvs::srv::Empty>(
    "state_listener_toggle_topic",
    std::bind(&StateListenerCSVNode::toggle_execution_callback_, this, _1, _2),
    rmw_qos_profile_services_default, this->m_cb_group_);

  this->m_reset_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "reset_listener",
    std::bind(&StateListenerCSVNode::run_complete_callback_, this, _1, _2),
    rmw_qos_profile_services_default, this->m_cb_group_);

  // Create run folder
  std::stringstream run_dir;
  run_dir << StateListenerCSVNode::RUN_DIR_NAME <<
             std::setw(5) << std::setfill('0') << this->m_run_number_;
  this->m_run_directory_ = std::filesystem::path(this->m_data_directory_top_) / run_dir.str();

  // Safely attempt to create directory
  try{
    std::filesystem::create_directory(this->m_run_directory_);
  } catch (std::filesystem::filesystem_error const &e) {
    std::stringstream error;
    error << e.what() << "\n" << "Path: " << this->m_run_directory_.string();
    RCLCPP_ERROR(this->get_logger(), error.str().c_str());
    exit(EXIT_FAILURE);
  }
}

/* === FUNCTIONS === */
/* == Callbacks == */
/**
 * @brief Callback that writes state data to file
 *
 * @param state: Published state data
 */
void StateListenerCSVNode::state_callback_(const uav_interfaces::msg::UavState &state){
  if (this->m_is_running_){
    // If row count exceeded limit, make a new file
    if ((this->m_row_count_ > this->m_row_limit_) || this->m_new_run_){
      this->m_file_.close();

      // Check if a new run is starting
      if (this->m_new_run_){
        this->m_file_count_ = 0;
        this->m_new_run_ = false;
      } else {
        this->m_file_count_++;
      }

      this->create_file_();

      this->m_row_count_ = 0;
    }

    // Write state to file
    this->write_data_(state);
    this->m_row_count_++;
  }
}

/**
 * @brief Change a flag to pause operation once a path is finished
 *
 * @param request: Empty, not utilized
 * @param response: Empty, not utilized
 */
void StateListenerCSVNode::toggle_execution_callback_(
  const std::shared_ptr<std_srvs::srv::Empty::Request> /* request */,
  std::shared_ptr<std_srvs::srv::Empty::Response>      /* response */){
    // Toggle the is_running flag
    this->m_is_running_ = !this->m_is_running_;
}

/**
 * @brief Update run number and create a new run directory
 *
 * @param request: Empty, not utilized
 * @param response: Empty, not utilized
 */
void StateListenerCSVNode::run_complete_callback_(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response>      /* response */){
  if(this->m_file_.is_open()){
    this->m_file_.close();
    // Increment run number
    this->m_run_number_++;

    // Write COMPLETED file
    std::ofstream complete_file((this->m_run_directory_ / "COMPLETED").string(), std::ios::out);
    complete_file.close();

    // If a request comes in with data field = false, haven't completed simulations, so start new run
    if (!request->data){
      std::stringstream run_dir;
      run_dir << StateListenerCSVNode::RUN_DIR_NAME <<
                std::setw(5) << std::setfill('0') << this->m_run_number_;

      this->m_run_directory_ = std::filesystem::path(this->m_data_directory_top_) / run_dir.str();

      // Safely attempt to create directory
      try{
        std::filesystem::create_directory(this->m_run_directory_);
      } catch (std::filesystem::filesystem_error const &e) {
        std::stringstream error;
        error << e.what() << "\n" << "Path: " << this->m_run_directory_.string();
        RCLCPP_ERROR(this->get_logger(), error.str().c_str());
        exit(EXIT_FAILURE);
      }

      this->m_new_run_ = true;
    } else {  // If request->data came in as true, simulation is over, shutdown the node
      rclcpp::shutdown();
    }
  }
}

/* == Helper functions == */
/**
 * @brief Write the headers to a newly opened CSV file
 *
 * @param file: File object to write headers to
 */
inline void StateListenerCSVNode::write_csv_headers_(){
  // Assuming file is already open
  // For each header name in the struct write header to CSV
  // Time header
  this->m_file_ << StateListenerCSVNode::StateHeaders::TIME << ",";

  // Position headers
  this->m_file_ << StateListenerCSVNode::StateHeaders::POS_X << ","
       << StateListenerCSVNode::StateHeaders::POS_Y << ","
       << StateListenerCSVNode::StateHeaders::POS_Z << ",";

  // Orientation headers
  this->m_file_ << StateListenerCSVNode::StateHeaders::PHI   << ","
       << StateListenerCSVNode::StateHeaders::THETA << ","
       << StateListenerCSVNode::StateHeaders::PSI   << ",";

  // Velocity headers
  this->m_file_ << StateListenerCSVNode::StateHeaders::VEL_X << ","
       << StateListenerCSVNode::StateHeaders::VEL_Y << ","
       << StateListenerCSVNode::StateHeaders::VEL_Z << ",";

  // Gyro Bias headers
  this->m_file_ << StateListenerCSVNode::StateHeaders::GYRO_X << ","
       << StateListenerCSVNode::StateHeaders::GYRO_Y << ","
       << StateListenerCSVNode::StateHeaders::GYRO_Z << "\n";
}

/**
 * @brief Write new state data to file
 *
 * @param state: State data to write to CSV file
 * @param file: File object to write headers to
 */
inline void StateListenerCSVNode::write_data_(const uav_interfaces::msg::UavState &state){
  // Assuming file is already open
  // Write state's timestamp
  double time = state.pose.header.stamp.sec + (state.pose.header.stamp.nanosec * 1e-9);

  // Check for repeating timestamp
  if (time == this->m_prev_time_){
    return;
  }

  // Update prev_time for next iteration
  this->m_prev_time_ = time;

  this->m_file_ << time << ",";

  // Write 3 position elements to CSV
  this->m_file_ << state.pose.pose.position.x << ","
       << state.pose.pose.position.y << ","
       << state.pose.pose.position.z << ",";

  // Write orientation elements to CSV
  this->m_file_ << state.phi   << ","
       << state.theta << ","
       << state.psi   << ",";

  // Write linear velocities to CSV
  this->m_file_ << state.twist.twist.linear.x << ","
       << state.twist.twist.linear.y << ","
       << state.twist.twist.linear.z << ",";

  // Write gyro bias to CSV
  this->m_file_ << state.gyro_bias.vector.x << ","
       << state.gyro_bias.vector.y << ","
       << state.gyro_bias.vector.z << "\n";
}

/**
 * @brief Create a new CSV file
 *
 * @param file: File object to create new file from
 * @param file_prefix: String to put at front of file (e.g. "truth", or "nav")
 * @param file_number: Number to put at end of file (e.g. "001", "002", etc.)
 */
inline void StateListenerCSVNode::create_file_(){
  // Open file with new name specified by timestamp with prefix at beginning
  std::stringstream filename_stream;
  std::string filename;

  // Make file name and create the file
  filename_stream << this->m_file_prefix_ << std::setw(5) << std::setfill('0') << this->m_file_count_;
  filename_stream << ".csv";

  filename = (this->m_run_directory_ / filename_stream.str()).string();
  this->m_file_.open(filename, std::ios::out);

  // Check if file is open
  if (!this->m_file_.is_open()){
    throw(std::runtime_error("Error opening file: " + filename));
    exit(EXIT_FAILURE);
  }

  // Set file precision and write headers
  this->m_file_ << std::fixed << std::setprecision(this->m_csv_precision_);
  this->write_csv_headers_();
}

/**
 * @brief Close files when ROS shutdown occurs to prevent file corruption
 */
inline void StateListenerCSVNode::shutdown_callback(){
  // Close files before shutdown completes
  RCLCPP_WARN(this->get_logger(), "Closing files before shutdown!");
  this->m_file_.close();
}
}  // namespace planner_interface

int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  // Make the node
  auto node = std::make_shared<planner_interface::StateListenerCSVNode>();
  rclcpp::on_shutdown(std::bind(&planner_interface::StateListenerCSVNode::shutdown_callback, node));

  // Spin up node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
}
