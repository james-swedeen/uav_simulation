/**
 * @File: pdvg_panel.hpp
 * @Date: January 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Header file that defines the attributes of an
 * RVIZ panel node used for requesting PDVG path planning
 * in simulation
 */

#ifndef RVIZ_PLUGINS__PDVG_PANEL_NODE_HPP_
#define RVIZ_PLUGINS__PDVG_PANEL_NODE_HPP_

// C++ Headers
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// ROS2 Headers
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/trigger.hpp>

// Local Headers
#include "uav_interfaces/srv/select_analysis_plot.hpp"

namespace rviz_plugins{
/**
 * @brief Node utilized to make PDVG plans and plots
 */
class PDVGPanelNode : public rclcpp::Node{
public:
  /* == CONSTRUCTOR == */
  /**
   * @brief Construct a new PDVGPanelNode and initialize member variables
   */
  PDVGPanelNode();

  /* == SERVICE REQUESTS == */
  /**
   * @brief Makes service request to PDVG node
   *
   * @details This function makes a service request to the PDVG interface node
   * and updates the status and status message variables.
   */
  void pdvg_request();

  /**
   * @brief Makes a plotting request to the pdvg_interface node
   *
   * @param save_to_pdf Flag. If true, save all plots to pdf. Else, display plots.
   * @param plot_types List of integers indicating what plot types should be plotted. These
   * numbers are associated with the PlotTypeInds struct from the plot_statistics_tools.hpp
   * file in the rapidly-exploring-random-tree repo in the kalman_filter package
   * @param data_types List of integers indicating what data types should be plotted.
   * The enumeration for this set of types is specified in the SelectPDVGPlot Request
   */
  void plot_request(const bool               save_to_pdf = false,
                    const bool               make_pd_plots = false,
                    const bool               make_error_budget = false,
                    const std::vector<bool> &plot_types = {},
                    const std::vector<bool> &data_types = {});

  /* == GETTERS AND SETTERS == */
  /**
   * @brief Get the pdvg service status value
   *
   * @return uint8_t containing status state value
   */
  uint8_t get_pdvg_service_status();

  /**
   * @brief Get the pdvg status message
   *
   * @return std::string containing the status message obtained
   * from the PDVG service callback or otherwise
   */
  std::string get_pdvg_status_message();

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
  void set_pdvg_status_inactive();

  /**
   * @brief Return true if PDVG button can be enabled
   *
   * @return True if the PDVG service callback is complete, stops early, or
   * service is not in progress. False otherwise.
   */
  bool pdvg_button_enabled();

  /**
   * @brief Return true if plotting buttons can be enabled
   *
   * @return True if at least one successful PDVG run has been completed
   * and not waiting on a plotting request.
   */
  bool plot_button_enabled();

  /* === VARIABLES === */
  // Status variables for checking return status of service from outside this class
  static const uint8_t SUCCESS = 0;
  static const uint8_t FAILURE = 1;
  static const uint8_t INACTIVE = 2;
  static const uint8_t PROCESSING = 3;

  // Message used to indicate to user return status of a service call, used in Panel
  std::string m_status_message_;

private:
  /* === VARIABLES === */
  // Service client for requesting PDVG plans
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr m_pdvg_client_;

  // Service client for requesting plots
  rclcpp::Client<uav_interfaces::srv::SelectAnalysisPlot>::SharedPtr m_plot_client_;

  // Used to define a callback group that all callbacks for the node will utilize
  rclcpp::CallbackGroup::SharedPtr m_client_cb_;

  // Status flags and related
  uint8_t m_pdvg_status_;  // Either SUCCESS (0), FAILURE (1), or INACTIVE (2)

  bool m_pdvg_plan_done_;  // Flag indicating if PDVG plan request is finished
  bool m_pdvg_first_success_;  // Flag indicating at least one PDVG plan has succeeded
  bool m_plot_req_done_;  // Flag used for reenabling panel buttons after request completes

  /* == CALLBACKS == */
  /**
   * @brief Callback updates flags to indicate desired state of PDVG panel
   *
   * @details Simply updates the status flag and sets the status message.
   * Status message is designed to be displayed only in the button that
   * made the service request
   *
   * @param response: Service response containing a status flag and message
   */
  void pdvg_callback(const rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response);

  /**
   * @brief Check success of the request for matplotlib plotting and print out info message
   *
   * @param response: Future indicating status of plotting request
   */
  void plot_callback(const rclcpp::Client<uav_interfaces::srv::SelectAnalysisPlot>::SharedFuture response);
};
}  // namespace rviz_plugins
#endif  // RVIZ_PLUGINS__PDVG_PANEL_NODE_HPP_
