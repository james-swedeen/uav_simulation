/**
 * @File: lincov_analysis_panel_node.cpp
 * @Date: November 2022
 * @Author: Daren Swasey
 *
 * @brief
 * Implementation file for LinCov analysis panel node used by
 * LinCov Analysis Panel
 */

// Local Headers
#include "rviz_plugins/lincov_analysis_panel_node.hpp"
#include "uav_interfaces/srv/toggle_ellipse.hpp"
#include "uav_interfaces/srv/lin_cov_monte_carlo_call.hpp"
#include "uav_interfaces/srv/select_analysis_plot.hpp"

// C++ Headers
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

// ROS2 Headers
#include <rclcpp/rclcpp.hpp>

namespace rviz_plugins{
using LinCovMonteCarloCall = uav_interfaces::srv::LinCovMonteCarloCall;
using namespace std::chrono_literals;
using namespace std::placeholders;

const std::string LinCovAnalysisPanelNode::m_success_message_ = "Analysis Complete";

/**
 * @brief Initialize the node and connect service clients to servers
 */
LinCovAnalysisPanelNode::LinCovAnalysisPanelNode()
  : Node("lincov_analysis_panel_node"){
  // Set up flags
  this->m_analysis_done_ = true;
  this->m_first_analysis_success_ = false;
  this->m_toggle_req_done_ = true;
  this->m_plot_req_done_ = true;
  this->m_analysis_status_ = this->INACTIVE;

  // Make mutually exclusive callback group because concurrent callbacks aren't needed
  this->m_client_cb_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Initialize all the service requesters
  this->m_analysis_req_ptr_ = this->create_client<LinCovMonteCarloCall>(
    "run_analysis",
    rmw_qos_profile_services_default,
    this->m_client_cb_);

  this->m_plot_req_ptr_ = this->create_client<uav_interfaces::srv::SelectAnalysisPlot>(
    "plot_results",
    rmw_qos_profile_services_default,
    this->m_client_cb_);

  this->m_toggle_ellipse_req_ptr_ = this->create_client<uav_interfaces::srv::ToggleEllipse>(
    "toggle_uncertainty_ellipses",
    rmw_qos_profile_services_default,
    this->m_client_cb_);
}

/* === REQUESTS === */
/**
 * @brief Service request for LinCov or Monte Carlo analysis from lincov_interface node
 *
 * @details Makes a request to the lincov_interface node to perform a
 * LinCov or Monte Carlo analysis. Turns off uncertainty ellipses if they are enabled.
 * Disables all buttons and boxes that require analysis data.
 */
void LinCovAnalysisPanelNode::analysis_request(const uint64_t num_monte_carlo_runs, const uint64_t analysis_type){
  // Set to false so button is disabled
  this->m_analysis_done_ = false;
  auto request = std::make_shared<LinCovMonteCarloCall::Request>();

  // Set parameters for the call: Number of Monte Carlo runs, and what analysis to perform
  request->num_monte_carlo_runs = num_monte_carlo_runs;
  request->analysis_type = analysis_type;

  if (!this->m_analysis_req_ptr_->wait_for_service(1s)) {
    // Service considered finished and failed
    this->m_analysis_done_ = true;
    this->m_analysis_status_ = LinCovAnalysisPanelNode::FAILURE;

    // If ROS is shutdown before the service is activated, show this error
    if (!rclcpp::ok()) {
      this->m_status_message_ = "Service response interrupted";
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), this->m_status_message_.c_str());
      return;
    }
    // Print in the screen some information so the user knows what is happening
    this->m_status_message_ = "Service unavailable";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), this->m_status_message_.c_str());
    return;
  }

  // Service server is available, send the request
  auto future = this->m_analysis_req_ptr_->async_send_request(
    request,
    std::bind(&LinCovAnalysisPanelNode::analysis_callback, this, _1));
}

/**
 * @brief Makes a plotting request to the lincov_interface node
 *
 *
 * @param save_to_pdf Flag. If true, save all plots to pdf. Else, display plots.
 * @param downsample_factor How many MC runs to downsample by (applicable only if not
 * saving to pdf)
 * @param plot_types List of integers indicating what plot types should be plotted. These
 * numbers are associated with the PlotTypeInds struct from the plot_statistics_tools.hpp
 * file in the rapidly-exploring-random-tree repo in the kalman_filter package
 * @param data_types List of integers indicating what data types should be plotted.
 * The enumeration for this set of types is specified in the SelectAnalysisPlot Request definition
 */
void LinCovAnalysisPanelNode::plot_request(const bool               save_to_pdf,
                                           const uint32_t           downsample_factor,
                                           const std::vector<bool> &plot_types,
                                           const std::vector<bool> &data_types){
  // Create request to generate matplotlib plots
  auto request = std::make_shared<uav_interfaces::srv::SelectAnalysisPlot::Request>();

  // Process all combinations if saving to PDF
  if (save_to_pdf){
    request->plot_types.fill(true);
    request->data_types.fill(true);
  } else {
    // Initialize data
    request->plot_types.fill(false);
    request->data_types.fill(false);

    // Verify checkboxes are correct with respect to information in request
    if (plot_types.size() != request->plot_types.size()){
      std::string error = "SelectAnalysisPlot plot_types size should be (" +
        std::to_string(request->plot_types.size()) +
        "), but given plot_types size of (" + std::to_string(plot_types.size()) +
        ") from dropdown menu!" +
        " Please verify panel plot_type checkbox is filled with correct plot types" +
        " and the order is correct with respect to items in SelectPlotAnalysis Request";
      RCLCPP_ERROR(this->get_logger(), error.c_str());
      exit(EXIT_FAILURE);
    }

    if (data_types.size() != request->data_types.size()){
      std::string error = "SelectAnalysisPlot data_types size should be (" +
        std::to_string(request->data_types.size()) +
        "), but given data_types size of (" + std::to_string(data_types.size()) +
        ") from dropdown menu!" +
        " Please verify panel data_type checkbox is filled with correct data types" +
        " and the order is correct with respect to items in SelectPlotAnalysis Request";
      RCLCPP_ERROR(this->get_logger(), error.c_str());
      exit(EXIT_FAILURE);
    }

    // Set plot types and data types
    std::copy(plot_types.begin(), plot_types.end(), request->plot_types.begin());
    std::copy(data_types.begin(), data_types.end(), request->data_types.begin());

  }

  request->is_pdvg_request   = false;
  request->save_to_pdf       = save_to_pdf;
  request->mc_run_downsample = downsample_factor;

  if (!this->m_plot_req_ptr_->wait_for_service(1s)) {
      // If ROS is shutdown before the service is activated, show this error
    if (!rclcpp::ok()) {
      this->m_status_message_ = "Service response interrupted";
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), this->m_status_message_.c_str());
      return;
    }
    // Timeout expired, may need to try again
    this->m_status_message_ = "Service unavailable";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), this->m_status_message_.c_str());
    return;
  }

  // Service is available, send the request
  auto future = this->m_plot_req_ptr_->async_send_request(
    request,
    std::bind(&LinCovAnalysisPanelNode::plot_callback, this, _1));

  // Set flag so button is off while generating plots
  this->m_plot_req_done_ = false;
}

/**
 * @brief Service request to the lincov_interface toggle ellipse server
 *
 * @details Makes a service request to the toggle ellipse server in the
 * lincov_interface node with a spatial resolution parameter specified by the user.
 * Sets the m_toggle_req_done flag to false if the request can be sent, which
 * disables the toggle button until the callback is complete.
 *
 * @param spatial_resolution: Spatial resolution for generating uncertainty ellipses at
 */
void LinCovAnalysisPanelNode::toggle_ellipse_request(const double spatial_resolution){
  // Make request to generate matplotlib plots
  auto request = std::make_shared<uav_interfaces::srv::ToggleEllipse::Request>();

  // Set the spatial resolution for the request
  request->requested_res = spatial_resolution;

  if (!this->m_plot_req_ptr_->wait_for_service(1s)) {
    // Service considered failed
    this->m_toggle_req_done_ = true;

      // If ROS is shutdown before the service is activated, show this error
    if (!rclcpp::ok()) {
      this->m_status_message_ = "Service response interrupted";
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), this->m_status_message_.c_str());
      return;
    }
    // Print in the screen some information so the user knows what is happening
    this->m_status_message_ = "Service unavailable";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), this->m_status_message_.c_str());
    return;
  }

  // Set flag to false so button is disabled until request completion
  this->m_toggle_req_done_ = false;

  // Service server is available, send the request
  auto future = this->m_toggle_ellipse_req_ptr_->async_send_request(
    request,
    std::bind(&LinCovAnalysisPanelNode::toggle_ellipse_callback, this, _1));
}

/* === GETTERS AND FLAG CHECKS === */
/**
 * @brief Returns status of the flag that indicates an analysis completion
 *
 * @return True if an analysis has never been called or completed
 * False if an analysis is in progress
 */
bool LinCovAnalysisPanelNode::analysis_button_enabled(){
  return this->m_analysis_done_;
}

/**
 * @brief Indicates whether the "Generate Plots" button should be enabled.
 *
 * @return True if there is no active analysis and at least one analysis
 * has been completed. False if analysis is in progress or an analysis
 * has never been performed.
 */
bool LinCovAnalysisPanelNode::plot_button_enabled(){
  return this->m_analysis_done_ && this->m_first_analysis_success_ && this->m_plot_req_done_;
}

/**
 * @brief Indicates whether the uncertainty ellipse toggle button should be enabled.
 *
 * @return True if the toggle request has been completed, there is no ongoing
 * analysis, and at least one analysis has been completed. False otherwise
 */
bool LinCovAnalysisPanelNode::toggle_ellipses_enabled(){
  return this->m_toggle_req_done_ && this->m_analysis_done_ && this->m_first_analysis_success_;
}

/**
 * @brief Get the status of the most recent analysis
 *
 * @details Get the status of the most recent analysis. The result is then
 * used as an indicator of what color to briefly show on the button.
 *
 * Possible states are SUCCESS, FAILURE, and INACTIVE
 *
 * @return Current status value of LinCovAnalysisPanelNode
 */
uint8_t LinCovAnalysisPanelNode::get_analysis_status(){
  return this->m_analysis_status_;
}

/**
 * @brief Get the fail message from the analysis to show in the button used to
 * request the analysis
 *
 * @details Get the fail message from the analysis to show in the button used to
 * request the analysis. The resultant message is typically used as temporary replacement
 * text in the button that requested a service to indicate to the user what went wrong.
 *
 * @return std::string Message containing some failure information regarding the most recent
 * analysis attempt
 */
std::string LinCovAnalysisPanelNode::get_status_message(){
  return this->m_status_message_;
}

/* === SETTERS === */
/**
 * @brief Sets the analysis status variable to inactive
 */
void LinCovAnalysisPanelNode::set_analysis_status_inactive(){
  this->m_analysis_status_ = LinCovAnalysisPanelNode::INACTIVE;
}

/* === CALLBACKS === */
/**
 * @brief Callback that handles results of analysis requests.
 *
 * @details If the request to do an analysis was successfully completed,
 * set flags indicating the first analysis finished (if first time in callback)
 * and the current analysis is done. These flags indicate that all other buttons
 * can be enabled if the analysis was successful.
 *
 * @param future: Future returned from lincov_interface
 */
void LinCovAnalysisPanelNode::analysis_callback(
  const rclcpp::Client<LinCovMonteCarloCall>::SharedFuture future){
  // Get the response's success field to see if all checks passed
  if (future.get()->success) {
    // Set status so results can be shown to the user
    this->m_status_message_ = future.get()->message;
    this->m_analysis_status_ = LinCovAnalysisPanelNode::SUCCESS;

    // After first successful completion, other buttons can be used
    this->m_first_analysis_success_ = true;
  } else {
    // Set message and status so results can be shown to the user
    this->m_status_message_ = const_cast<char*>(future.get()->message.c_str());
    this->m_analysis_status_ = LinCovAnalysisPanelNode::FAILURE;
  }

  // Set flags so buttons can be enabled again
  this->m_analysis_done_ = true;
}

/**
 * @brief Check success of the request for matplotlib plotting and print out info message
 *
 * @param future: Future indicating status of plotting request
 */
void LinCovAnalysisPanelNode::plot_callback(const rclcpp::Client<uav_interfaces::srv::SelectAnalysisPlot>::SharedFuture future){
  // Get the response's success field to see if all checks passed
  if (!future.get()->success) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), future.get()->message.c_str());
  }

  // Set flag for button enable again
  this->m_plot_req_done_ = true;
}

/**
 * @brief Callback that handles results of toggling ellipse and sets flags for GUI
 *
 * @param future: Future indicating status of toggling uncertainty ellipses
 */
void LinCovAnalysisPanelNode::toggle_ellipse_callback(
  const rclcpp::Client<uav_interfaces::srv::ToggleEllipse>::SharedFuture future){
  // Get the response's success field to see if all checks passed
  if (!future.get()->success) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
      "Unexpected error occurred in lincov_interface node when toggling ellipses...");
  }

  // Set flag to true so button can be reenabled
  this->m_toggle_req_done_ = true;
}
}  // end namespace rviz_plugins
