/**
 * @File: lincov_analysis_panel.hpp
 * @Date: December 2022
 * @Author: Daren Swasey
 *
 * @brief
 * Header file for LinCov analysis panel used in rviz2
 * for simulation
 */

#ifndef RVIZ_PLUGINS__LINCOV_ANALYSIS_PANEL_HPP_
#define RVIZ_PLUGINS__LINCOV_ANALYSIS_PANEL_HPP_

// C++ Headers
#include <chrono>
#include <memory>
#include <string>

// QT GUI Headers
#include <QLineEdit>
#include <QTimer>
#include <QToolButton>

// ROS 2 Headers
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"

// Local Headers
#include "rviz_plugins/lincov_analysis_panel_node.hpp"
#include "uav_interfaces/srv/toggle_ellipse.hpp"
#include "uav_interfaces/srv/lin_cov_monte_carlo_call.hpp"
#include "rviz_plugins/gui_tools/q_dropdown_checkbox.hpp"

namespace rviz_plugins{
/**
 * @brief Class that manages an rviz panel used to access lincov_interface functionality
 *
 * @details This class is the manager class for GUI-based elements of an
 * rviz panel that interact with certain features in the lincov_interface node.
 * The GUI elements provided by this class give the rviz user a way of performing
 * LinCov analyses and generating visual data from the analysis results. The
 * class hosts its own ros2 node as an interface between itself and the
 * lincov_interface node. It utilizes the Qt library for the GUI functionality
 *
 */
class LinCovAnalysisPanel : public rviz_common::Panel{
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT

  public:
    /**
     * @brief Create object and all buttons. Connect all signals and slots
     *
     * @details Sets default values for flags and service requests. Defines
     * all buttons and other GUI elements with their respective layouts and
     * settings. Connect GUI element signals to slots provided by the Panel
     * object. Start the timer used to spin the node member variable.
     *
     * QWidget subclass constructors usually take a parent widget
     * parameter (which usually defaults to 0).  At the same time,
     * pluginlib::ClassLoader creates instances by calling the default
     * constructor (with no arguments).  Taking the parameter and giving
     * a default of 0 lets the default constructor work and also lets
     * someone using the class for something else to pass in a parent
     * widget as they normally would with Qt.
     *
     * @param parent: Parent QWidget object of this panel
     */
    explicit LinCovAnalysisPanel( QWidget* parent = 0 );

  protected Q_SLOTS:

    /**
     * @brief Starts LinCov analysis when "LinCov Analysis" button is clicked
     *
     * @details Once the "LinCov Analysis" button has been clicked, this function
     * is used to call the LinCovAnalysisPanelNode::analysis_request() function.
     * This function also disables buttons so during the LinCov analysis, other
     * service requests cannot be made. Buttons are reenabled via flag checks
     * in the main thread.
     */
    void lincov_button_clicked();

    /**
     * @brief Starts Monte Carlo analysis when "Monte Carlo Analysis" button is clicked
     *
     * @details Once the "Monte Carlo Analysis" button has been clicked, this function
     * is used to call the LinCovAnalysisPanelNode::analysis_request() function.
     * This function also disables buttons so during the LinCov analysis, other
     * service requests cannot be made. Buttons are reenabled via flag checks
     * in the main thread.
     *
     * For Monte Carlo to run, a LinCov analysis must also be made. This is because
     * the Monte Carlo analysis is a verification of the LinCov analysis, so they should
     * always be calculated side-by-side.
     */
    void mc_button_clicked();

    /**
     * @brief Make a call for a service request to plot analysis results
     *
     * @details Disables the "Generate Plots" button and makes a service request
     * via the LinCovAnalysisPanelNode::plot_request() function. Results will show
     * only LinCov if only the LinCov analysis was requested, and will show results
     * for both if a Monte Carlo analysis was requested.
     */
    void plot_button_clicked();

    /**
     * @brief Make a call for a service request to generate pdf of all plots
     *
     * @details Disables the "Generate PDF" button and makes a service request
     * via the LinCovAnalysisPanelNode::plot_request() function.
     */
    void pdf_button_clicked();

    /**
     * @brief Update desired spatial resolution for plotting uncertainty ellipses
     *
     * @details Updates the spatial resolution internal variable. Update takes
     * place after the "Enter" key has been pressed when entering the number
     * or after a mouse click occurs outside the text entry box.
     */
    void update_resolution();

    /**
     * @brief Update the number of Monte Carlo runs specified by user
     *
     * @details Updates the Monte Carlo runs internal variable. Update takes
     * place after the "Enter" key has been pressed when entering the number
     * or after a mouse click occurs outside the text entry box.
     */
    void update_monte_carlo_runs();

    /**
     * @brief Update the Monte Carlo data downsample rate
     *
     * @details Updates the Monte Carlo downsample rate value.
     * This value is used to reduce the number of total samples used
     * to make MC plots
     */
    void update_mc_downsample();

    /**
     * @brief Enable or disable uncertainty ellipses
     *
     * @details Toggle the uncertainty ellipses on or off and set the state of
     * the button to display the appropriate message.
     */
    void toggle_ellipses();

    /**
     * @brief Perform state checks for GUI and spin up LinCovAnalysisPanelNode
     *
     * @details Evaluate the state of each service utilized by the
     * LinCovAnalysisPanelNode. For plotting and uncertainty ellipses toggle
     * buttons, enable if indicators show the service call is complete.
     *
     * For the analysis buttons, change their state and color depending
     * on the success or failure of the analysis request. Implements
     * non-blocking methods to show button colors so rviz continues to run
     * while the button state changes are taking place.
     *
     * Function gets called periodically depending on the tiner value specified
     * for the QTimer object in this class.
     */
    void spin_some();


  protected:
    // Buttons
    QToolButton* m_lincov_button_;    // Request LinCov analysis
    QToolButton* m_mc_button_;        // Request Monte Carlo analysis
    QToolButton* m_plot_button_;      // Request to plot analysis results
    QToolButton* m_pdf_button_;       // Request a pdf with all plots in it
    QToolButton* m_ellipse_button_;   // Toggle uncertainty ellipses

    // Text box for setting spatial resolution of published LinCov RVIZ markers
    QLineEdit* m_res_editor_;

    // Text box for setting number of Monte Carlo runs to do during analysis
    QLineEdit* m_num_monte_carlo_editor_;

    // Text box for setting downsample factor for individual Monte Carlo runs
    QLineEdit* m_mc_downsample_editor_;

    // Dropdown lists of checkboxes for choosing plots to generate
    gui_tools::QDropdownCheckbox* m_plot_type_list_;  // Type of plot
    gui_tools::QDropdownCheckbox* m_data_type_list_;  // Which data to use with plot type

    // Timer for calling spin_some
    QTimer* m_lincov_timer_;

  private:
    // Internal ros2 node for making service requests to lincov_interface node
    std::shared_ptr<LinCovAnalysisPanelNode> m_node_;

    // Executor to run the ros2 node from
    rclcpp::executors::MultiThreadedExecutor m_executor_;

    // Spatial resolution value for uncertainty ellipse service request
    double m_res_value_;

    // Default number of Monte Carlo runs
    uint16_t m_num_monte_carlo_runs_;

    // Factor by which to downsample Monte Carlo plot data
    uint16_t m_mc_downsample_value_;

    // Default strings for each of the buttons
    static const std::string m_lincov_button_default_str_;
    static const std::string m_mc_button_default_str_;
    static const std::string m_plot_button_default_str_;
    static const std::string m_pdf_button_default_str_;
    static const std::string m_ellipse_turn_on_str_;
    static const std::string m_ellipse_turn_off_str_;

    // Indicates if the uncertainty ellipses are currently displayed
    bool m_ellipses_enabled_;

    // Indicates whether an analysis button is currently flashing
    // a success or failure message
    bool m_reporting_status_;

    // Time points utilized for timing analysis button color changes
    std::chrono::steady_clock::time_point m_report_start_time_;

    // Holds value corresponding to constants in LinCovMonteCarloCall requests,
    // either LINCOV (0) or BOTH (1). Used to track which button error messages
    // should be displayed on
    uint64_t m_last_button_clicked_;

    /**
     * @brief Enable a button and set its message
     *
     * @param button: The button to modify
     * @param message: The message to set on the button
     */
    void m_enable_button_(QToolButton* button, const std::string &message);

    /**
     * @brief Disable a button and set its message
     *
     * @param button: The button to modify
     * @param message: The message to set on the button
     */
    void m_disable_button_(QToolButton* button, const std::string &message);

    /**
     * @brief Make a button red and change text to an error message
     *
     * @param button: Button to modify
     * @param message: Error message to display in the button text
     */
    void m_button_show_error_(QToolButton* button, const std::string &message);

    /**
     * @brief Make a button green and change text to a success message
     *
     * @param button: Button to modify
     * @param message: Success message to display in the button text
     */
    void m_button_show_success_(QToolButton* button, const std::string &message);
};
}  // namespace rviz_plugins
#endif  // RVIZ_PLUGINS__LINCOV_ANALYSIS_PANEL_HPP_
