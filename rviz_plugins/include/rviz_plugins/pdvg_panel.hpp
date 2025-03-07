/**
 * @File: pdvg_panel.hpp
 * @Date: January 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Header file that defines the attributes of an
 * RVIZ panel used for requesting PDVG path planning
 * in simulation
 */

#ifndef RVIZ_PLUGINS__PDVG_PANEL_HPP_
#define RVIZ_PLUGINS__PDVG_PANEL_HPP_

// Local Headers
#include <rviz_plugins/pdvg_panel_node.hpp>
#include <rviz_plugins/gui_tools/q_dropdown_checkbox.hpp>

// C++ Headers
#include <chrono>
#include <memory>
#include <string>

// ROS2 Headers
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

// QT GUI Headers
#include <QTimer>
#include <QToolButton>

namespace rviz_plugins{
/**
 * @brief Panel for interacting with PDVG planner node
 *
 * @details This panel acts as an interface between the user of the simulation
 * scenario defined in the pd_planner_launch package and a PDVG planner node.
 * The panel simply provides a button for the user to click when the user
 * wants a plan generated.
 *
 * Panel provides feedback on the status of the planner request through the
 * text on the button.
 */
class PDVGPanel : public rviz_common::Panel{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT

public:
  /**
   * @brief Construct a new PDVGPanel object
   *
   * @details Sets default values for object parameters and creates
   * a node to interact with a PDVG planner. Defines
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
  explicit PDVGPanel(QWidget* parent = 0);

protected Q_SLOTS:

  /**
   * @brief Calls request function in PDVGPanelNode
   */
  void pdvg_button_clicked();

  /**
   * @brief Make request to plot obstacle checker data in pdvg_interface
   */
  void plot_pd_clicked();

  /**
   * @brief Make request to plot error budget in pdvg_interface
   */
  void error_budget_clicked();

  /**
   * @brief Spin the PDVG node and update button states
   *
   * @details This function is called by the timeout of the QTimer.
   * The function checks the status of the service requests made through
   * the PDVGPanelNode and updates button color and text to indicate the
   * service status to the user. After updating button state, spin the node.
   */
  void spin_some();

private:
  /* === VARIABLES === */
  /* == GUI TOOLS == */
  QToolButton* m_pdvg_button_;  // Used to request PDVG planning

  // Used to request plots of probability of detection results
  QToolButton* m_plot_pd_button_;

  // Used to request error budget plots
  QToolButton* m_error_budget_button_;

  // Timer that sets how often GUI events will be handled
  QTimer* m_spin_timer_;

  // Time points utilized for timing analysis button color changes
  std::chrono::steady_clock::time_point m_report_start_time_;

  // Default strings
  static const std::string m_pdvg_button_str_;
  static const std::string m_plot_pd_button_str_;
  static const std::string m_error_budget_button_str_;

  std::string m_node_status_message_;  // Hold status messages from node

  /* == ROS2 == */
  // Variables to create and spin node
  std::shared_ptr<PDVGPanelNode> m_pdvg_node_;
  rclcpp::executors::MultiThreadedExecutor m_executor_;

  /* == FLAGS == */
  // Used to indicate if service request status is being shown on a button
  bool m_reporting_status_;

  /* === BUTTON UPDATE FUNCTIONS === */
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
   * @brief Disable all buttons in the visuals section of the panel
   */
  void m_disable_all_viz_buttons_();

  /**
   * @brief Make a button red and change text to an error message
   *
   * @param message: Error message to display in the button text
   */
  void m_button_show_error_(QToolButton* button, const std::string &message);

  /**
   * @brief Make a button green and change text to a success message
   *
   * @param message: Success message to display in the button text
   */
  void m_button_show_success_(QToolButton* button, const std::string &message);
};
}  // namespace rviz_plugins
#endif  // RVIZ_PLUGINS__PDVG_PANEL_HPP_
