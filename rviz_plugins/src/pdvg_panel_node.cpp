/**
 * @File: pdvg_panel_node.cpp
 * @Date: January 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Implements the attributes of an
 * RVIZ panel node used for requesting PDVG path planning
 * in simulation
 */

// Local Headers
#include "rviz_plugins/pdvg_panel_node.hpp"
#include "uav_interfaces/srv/select_analysis_plot.hpp"

// C++ Headers
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// ROS2 Headers
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

using SelectAnalysisPlot = uav_interfaces::srv::SelectAnalysisPlot;

namespace rviz_plugins{
/* == CONSTRUCTOR == */
/**
 * @brief Construct a new PDVGPanelNode and initialize member variables
 */
PDVGPanelNode::PDVGPanelNode() : Node("pdvg_panel_node"){
  // Initialize parameters and flags
  this->m_pdvg_plan_done_ = true;
  this->m_pdvg_first_success_ = false;
  this->m_plot_req_done_ = true;
  this->m_status_message_ = "";
  this->m_pdvg_status_ = PDVGPanelNode::INACTIVE;

  // Create mutually exclusive callback group
  this->m_client_cb_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Make the service client for the PDVG planner
  this->m_pdvg_client_ = this->create_client<std_srvs::srv::Trigger>(
    "run_pdvg",
    rmw_qos_profile_services_default,
    this->m_client_cb_);

  // Make service client for plotting
  this->m_plot_client_ = this->create_client<uav_interfaces::srv::SelectAnalysisPlot>(
    "pdvg_plot",
    rmw_qos_profile_services_default,
    this->m_client_cb_);
}

/* == SERVICE REQUESTS == */
/**
 * @brief Makes service request to PDVG node
 *
 * @details This function makes a service request to the PDVG interface node
 * and updates the status and status message variables.
 */
void PDVGPanelNode::pdvg_request(){
  // Set status to processing so panel button takes on proper state
  this->m_pdvg_status_ = PDVGPanelNode::PROCESSING;

  // Make a Trigger request to send
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // Check to make sure service is actually available
  if (!this->m_pdvg_client_->service_is_ready()) {
    // Service considered finished and failed
    this->m_pdvg_status_ = PDVGPanelNode::FAILURE;

    // If ROS is shutdown before the service is activated, show this error
    if (!rclcpp::ok()) {
      this->m_status_message_ = "Service interrupted";
      return;
    }
    // Print in the screen some information so the user knows what is happening
    this->m_status_message_ = "Service unavailable";
    return;
  }

  // Service server is available, send the request
  auto future = this->m_pdvg_client_->async_send_request(
    request,
    std::bind(&PDVGPanelNode::pdvg_callback, this, std::placeholders::_1));
}

/**
 * @brief Makes a plotting request to the pdvg_interface node
 *
 * @param save_to_pdf Flag. If true, save all plots to pdf. Else, display plots.
 * @param plot_types List of integers indicating what plot types should be plotted. These
 * numbers are associated with the PlotTypeInds struct from the plot_statistics_tools.hpp
 * file in the rapidly-exploring-random-tree repo in the kalman_filter package
 * @param data_types List of integers indicating what data types should be plotted.
 * The enumeration for this set of types is specified in the SelectAnalysisPlot Request
 */
void PDVGPanelNode::plot_request(const bool               save_to_pdf,
                                 const bool               make_pd_plots,
                                 const bool               make_error_budget,
                                 const std::vector<bool> &plot_types,
                                 const std::vector<bool> &data_types){
  using namespace std::chrono_literals;
  using namespace std::placeholders;
  using Request = SelectAnalysisPlot::Request;

  auto request = std::make_shared<Request>();

  // Fill the request vectors with data from panel
  request->is_pdvg_request = true;
  request->save_to_pdf = save_to_pdf;
  request->make_pd_plots = make_pd_plots;
  request->make_pdvg_error_budget = make_error_budget;

  if (save_to_pdf){
    request->plot_types.fill(true);

    // Not all should be filled with true for PDVG
    for (size_t i = Request::PDVG_PLOT_TYPE_LEN; i < request->plot_types.size(); i++){
      request->plot_types[i] = false;
    }
    request->data_types.fill(true);
  } else {
    // PD plots do not rely on plot or data types
    if (!make_pd_plots && !make_error_budget){
      // Initialize data
      request->plot_types.fill(false);
      request->data_types.fill(false);

      // Verify checkboxes are correct with respect to information in request
      if (plot_types.size() != Request::PDVG_PLOT_TYPE_LEN){
        std::string error = "SelectAnalysisPlot plot_types size should be (" +
          std::to_string(Request::PDVG_PLOT_TYPE_LEN) +
          "), but given plot_types size of (" + std::to_string(plot_types.size()) +
          ") from dropdown menu!" +
          " Please verify panel plot_type checkbox is filled with correct plot types" +
          " and the order is correct with respect to items for PDVG in SelectPlotAnalysis Request";
        RCLCPP_ERROR(this->get_logger(), error.c_str());
        exit(EXIT_FAILURE);
      }

      if (data_types.size() != request->data_types.size()){
        std::string error = "SelectAnalysisPlot data_types size should be (" +
          std::to_string(request->data_types.size()) +
          "), but given data_types size of (" + std::to_string(data_types.size()) +
          ") from dropdown menu!" +
          " Please verify panel data_type checkbox is filled with correct data types" +
          " and the order is correct with respect to items for PDVG in SelectPlotAnalysis Request";
        RCLCPP_ERROR(this->get_logger(), error.c_str());
        exit(EXIT_FAILURE);
      }

      // Set plot types and data types
      std::copy(plot_types.begin(), plot_types.end(), request->plot_types.begin());
      std::copy(data_types.begin(), data_types.end(), request->data_types.begin());
    }
  }

  if (!this->m_plot_client_->wait_for_service(1s)) {
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
  auto future = this->m_plot_client_->async_send_request(
    request,
    std::bind(&PDVGPanelNode::plot_callback, this, _1));

  // Set flag so button is off while generating plots
  this->m_plot_req_done_ = false;
}

/* == GETTERS AND SETTERS == */
/**
 * @brief Get the pdvg service status value
 *
 * @return uint8_t containing status state value
 */
uint8_t PDVGPanelNode::get_pdvg_service_status(){
  // Return the node state
  return this->m_pdvg_status_;
}

/**
 * @brief Get the pdvg status message
 *
 * @return std::string containing the status message obtained
 * from the PDVG service callback or otherwise
 */
std::string PDVGPanelNode::get_pdvg_status_message(){
  return this->m_status_message_;
}

/**
 * @brief Set the pdvg status to inactive
 *
 * @details This setter is provided so when the panel completes
 * its interactions with the buttons after a service request,
 * the node status can be set back to inactive.
 *
 * Setting the status to INACTIVE is the only interaction between
 * panel and node that directly changes a member variable of the node.
 */
void PDVGPanelNode::set_pdvg_status_inactive(){
  // Set blank status message and state to INACTIVE
  this->m_status_message_ = "";
  this->m_pdvg_status_ = PDVGPanelNode::INACTIVE;
}

/**
 * @brief Return true if PDVG button can be enabled
 *
 * @return True if the PDVG service callback is complete, stops early, or
 * service is not in progress. False otherwise.
 */
bool PDVGPanelNode::pdvg_button_enabled(){
  // The PDVG button can be enabled only if the status is inactive
  return (this->m_pdvg_status_ == PDVGPanelNode::INACTIVE);
}

/**
 * @brief Return true if plotting buttons can be enabled
 *
 * @return True if at least one successful PDVG run has been completed
 * and not waiting on a plotting request.
 */
bool PDVGPanelNode::plot_button_enabled(){
  return this->m_plot_req_done_ && this->m_pdvg_plan_done_ && this->m_pdvg_first_success_;
}

/* === CALLBACKS === */
/**
 * @brief Callback updates flags to indicate desired state of PDVG panel
 *
 * @details Simply updates the status flag and sets the status message.
 * Status message is designed to be displayed only in the button that
 * made the service request
 *
 * @param response: Service response containing a status flag and message
 */
void PDVGPanelNode::pdvg_callback(const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response){
  // Get the returned message
  this->m_status_message_ = response.get()->message;

  // Get the response's success field to see if all checks passed
  if (response.get()->success) {
    // Set status so results can be shown to the user
    this->m_pdvg_status_ = PDVGPanelNode::SUCCESS;
    this->m_pdvg_first_success_ = true;
  } else {
    // Set status to failure
    this->m_pdvg_status_ = PDVGPanelNode::FAILURE;
  }

  this->m_pdvg_plan_done_ = true;
}

/**
 * @brief Check success of the request for matplotlib plotting and print out info message
 *
 * @param response: Future indicating status of plotting request
 */
void PDVGPanelNode::plot_callback(const rclcpp::Client<SelectAnalysisPlot>::SharedFuture /* response */){
  // Set flag for button enable again
  this->m_plot_req_done_ = true;
}
}  // namespace rviz_plugins
