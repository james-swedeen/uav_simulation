/**
 * @File: lincov_analysis_panel_node.hpp
 * @Date: November 2022
 * @Author: Daren Swasey
 *
 * @brief
 * Header file for LinCov analysis panel node used by
 * LinCov Analysis Panel
 */

#ifndef RVIZ_PLUGINS__LINCOV_ANALYSIS_PANEL_NODE_HPP_
#define RVIZ_PLUGINS__LINCOV_ANALYSIS_PANEL_NODE_HPP_

// C++ Headers
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// ROS 2 Headers
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

// Local Headers
#include "uav_interfaces/srv/toggle_ellipse.hpp"
#include "uav_interfaces/srv/lin_cov_monte_carlo_call.hpp"
#include "uav_interfaces/srv/select_analysis_plot.hpp"

namespace rviz_plugins{
/**
 * @brief ros2 node used to interact with the lincov_interface node
 *
 * @details This node is designed to interact with the lincov_interface node
 * from the planner_interface package. The main interaction happens via service
 * clients and their respective callbacks. Several flags and variables
 * are maintained to provide an indicator of the return status from each of
 * the service requests. Some of these indicators are provided via public
 * accessors so the LinCovAnalysisPanel class can appropriately indicate
 * to the user the results of any service requests.
 *
 */
class LinCovAnalysisPanelNode : public rclcpp::Node{
  public:
    /**
     * @brief Initialize the node and connect service clients to servers
     */
    LinCovAnalysisPanelNode();

    // Status variables for checking return status of analysis from outside this class
    static const uint8_t SUCCESS = 0;
    static const uint8_t FAILURE = 1;
    static const uint8_t INACTIVE = 2;

    // Message used to indicate to user that an analysis was successful
    static const std::string m_success_message_;

    /* === REQUESTS === */
    /**
     * @brief Service request for LinCov or Monte Carlo analysis from lincov_interface node
     *
     * @details Makes a request to the lincov_interface node to perform a
     * LinCov or Monte Carlo analysis. Turns off uncertainty ellipses if they are enabled.
     * Disables all buttons and boxes that require analysis data.
     */
    void analysis_request(const uint64_t num_monte_carlo_runs, const uint64_t analysis_type);

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
     * The enumeration for this set of types is specified in the SelectAnalysisPlot Request
     */
    void plot_request(const bool               save_to_pdf,
                      const uint32_t           downsample_factor,
                      const std::vector<bool> &plot_types = {},
                      const std::vector<bool> &data_types = {});

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
    void toggle_ellipse_request(const double spatial_resolution);

    /* === GETTERS AND FLAG CHECKS === */
    /**
     * @brief Returns status of the flag that indicates an analysis completion
     *
     * @return True if an analysis has never been called or completed
     * False if an analysis is in progress
     */
    bool analysis_button_enabled();

    /**
     * @brief Indicates whether the "Generate Plots" button should be enabled.
     *
     * @return True if there is no active analysis and at least one analysis
     * has been completed. False if analysis is in progress or an analysis
     * has never been performed.
     */
    bool plot_button_enabled();

    /**
     * @brief Indicates whether the uncertainty ellipse toggle button should be enabled.
     *
     * @return True if the toggle request has been completed, there is no ongoing
     * analysis, and at least one analysis has been completed. False otherwise
     */
    bool toggle_ellipses_enabled();

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
    uint8_t get_analysis_status();

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
    std::string get_status_message();

    /* === SETTERS === */
    /**
     * @brief Sets the analysis status variable to inactive
     */
    void set_analysis_status_inactive();

  private:
    bool m_analysis_done_;            // Flag to indicate when an analysis has finished
    bool m_first_analysis_success_;   // Flag to indicate when first analysis has finished

    // Flag to indicate status of service request to toggle uncertainty ellipses
    bool m_toggle_req_done_;

    bool m_plot_req_done_;  // Flag for status of plotting request

    // Status variable that can hold values of SUCCESS (0), FAILURE (1), or INACTIVE (2)
    // Helps the Panel know how to modify GUI elements in accordance with results of requests
    // to lincov_interface
    uint8_t m_analysis_status_;

    // Status message to print out when an analysis finishes
    std::string m_status_message_;


    // Service client for LinCov analysis
    rclcpp::Client<uav_interfaces::srv::LinCovMonteCarloCall>::SharedPtr m_analysis_req_ptr_;

    // Service client for matplotlib plotting analysis results
    rclcpp::Client<uav_interfaces::srv::SelectAnalysisPlot>::SharedPtr m_plot_req_ptr_;

    // Service client for toggling uncertainty ellipses
    rclcpp::Client<uav_interfaces::srv::ToggleEllipse>::SharedPtr m_toggle_ellipse_req_ptr_;

    // Used to define a callback group that all callbacks for the node will utilize
    rclcpp::CallbackGroup::SharedPtr m_client_cb_;

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
    void analysis_callback(
      const rclcpp::Client<uav_interfaces::srv::LinCovMonteCarloCall>::SharedFuture future);

    /**
     * @brief Check success of the request for matplotlib plotting and print out info message
     *
     * @param future: Future indicating status of plotting request
     */
    void plot_callback(const rclcpp::Client<uav_interfaces::srv::SelectAnalysisPlot>::SharedFuture future);

    /**
     * @brief Callback that handles results of toggling ellipse and sets flags for GUI
     *
     * @param future: Future indicating status of toggling uncertainty ellipses
     */
    void toggle_ellipse_callback(const rclcpp::Client<uav_interfaces::srv::ToggleEllipse>::SharedFuture future);
};
}  // end namespace rviz_plugins

#endif  // RVIZ_PLUGINS__LINCOV_ANALYSIS_PANEL_NODE_HPP_
