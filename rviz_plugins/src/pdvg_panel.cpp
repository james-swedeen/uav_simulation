/**
 * @File: pdvg_panel.cpp
 * @Date: January 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Implements the attributes of an
 * RVIZ panel used for requesting PDVG path planning
 * in simulation
 */

// Local Headers
#include "rviz_plugins/pdvg_panel.hpp"
#include "rviz_plugins/pdvg_panel_node.hpp"
#include "rviz_plugins/gui_tools/q_dropdown_checkbox.hpp"

// C++ Headers
#include <chrono>
#include <memory>
#include <string>

// ROS2 Headers
#include <rviz_common/panel.hpp>

// QT GUI Headers
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QTimer>
#include <QToolButton>

namespace rviz_plugins{
const std::string PDVGPanel::m_pdvg_button_str_ = "Make PDVG Plan";
const std::string PDVGPanel::m_plot_pd_button_str_ = "Plot PD";
const std::string PDVGPanel::m_error_budget_button_str_ = "Plot Error Budget";

/* === FUNCTIONS === */
/* == CONSTRUCTOR == */
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
PDVGPanel::PDVGPanel(QWidget* parent)
  : rviz_common::Panel(parent){
  // Set default values
  this->m_reporting_status_ = false;
  this->m_node_status_message_ = "";

  // Start up PDVG node and add it to the executor
  this->m_pdvg_node_ = std::make_shared<PDVGPanelNode>();
  this-> m_executor_.add_node(this->m_pdvg_node_);

  // Make PDVG button and set initial text
  this->m_pdvg_button_ = new QToolButton();
  this->m_pdvg_button_->setText(this->m_pdvg_button_str_.c_str());
  this->m_pdvg_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  // Make divider between plan button and visual tools
  QFrame* divider = new QFrame;
  divider->setFrameShape(QFrame::HLine);
  divider->setFrameShadow(QFrame::Sunken);

  // Make label for Visual Tools section
  QLabel* plotting_label = new QLabel;
  plotting_label->setText("Visual Tools");
  plotting_label->setAlignment(Qt::AlignCenter | Qt::AlignVCenter);
  plotting_label->setStyleSheet("font-weight: bold");

  // Create plot PD button
  this->m_plot_pd_button_ = new QToolButton();
  this->m_plot_pd_button_->setText(this->m_plot_pd_button_str_.c_str());
  this->m_plot_pd_button_->setToolTip(
    "Generate on-screen plots of probability of detection info for current path");

  this->m_plot_pd_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  this->m_plot_pd_button_->setEnabled(false);  // Set initial state to disabled

  // Add plot buttons to horizontal layout
  QHBoxLayout* plot_button_layout = new QHBoxLayout();
  plot_button_layout->addWidget(this->m_plot_pd_button_);

  // Create error budget button
  this->m_error_budget_button_ = new QToolButton();
  this->m_error_budget_button_->setText(this->m_error_budget_button_str_.c_str());
  this->m_error_budget_button_->setToolTip("Generate error budget plot");
  this->m_error_budget_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  this->m_error_budget_button_->setEnabled(false);  // Set initial state to disabled

  // Create top-level layout for all widgets to be added to
  QVBoxLayout* layout = new QVBoxLayout();

  layout->addWidget(this->m_pdvg_button_);
  layout->addWidget(divider);
  layout->addWidget(plotting_label);
  layout->addLayout(plot_button_layout);
  layout->addWidget(this->m_error_budget_button_);

  // Create all contents of layout
  setLayout(layout);

  // Make timer for spinning node
  this->m_spin_timer_ = new QTimer(this);

  // Connect signals and slots
  connect(this->m_pdvg_button_, SIGNAL(clicked()), this, SLOT(pdvg_button_clicked()));
  connect(this->m_plot_pd_button_, SIGNAL(clicked()), this, SLOT(plot_pd_clicked()));
  connect(this->m_error_budget_button_, SIGNAL(clicked()), this, SLOT(error_budget_clicked()));
  connect(this->m_spin_timer_, SIGNAL(timeout()), this, SLOT(spin_some()));

  // Start the timer
  this->m_spin_timer_->start(100);
}

/* == SLOTS == */
/**
 * @brief Calls request function in PDVGPanelNode
 */
void PDVGPanel::pdvg_button_clicked(){
  // Ensure button is in correct state
  this->m_reporting_status_ = false;
  this->m_disable_button_(this->m_pdvg_button_, "Generating Plan...");
  this->m_disable_all_viz_buttons_();

  // Make the service request to the PDVG planner
  this->m_pdvg_node_->pdvg_request();
}


/**
 * @brief Make request to plot obstacle checker data in pdvg_interface
 */
void PDVGPanel::plot_pd_clicked(){
  // Disable plotting buttons
  this->m_disable_all_viz_buttons_();

  // Selected plots don't matter here, just set make_pd_plots true
  this->m_pdvg_node_->plot_request(false, true);
}

/**
 * @brief Make request to plot error budget in pdvg_interface
 */
void PDVGPanel::error_budget_clicked(){
  // Disable plotting buttons
  this->m_disable_all_viz_buttons_();

  // Selected plots don't matter here, just set make_error_budgets true
  this->m_pdvg_node_->plot_request(false, false, true);
}


/**
 * @brief Spin the PDVG node and update button states
 *
 * @details This function is called by the timeout of the QTimer.
 * The function checks the status of the service requests made through
 * the PDVGPanelNode and updates button color and text to indicate the
 * service status to the user. After updating button state, spin the node.
 */
void PDVGPanel::spin_some(){
  using namespace std::chrono_literals;
  using milliseconds = std::chrono::milliseconds;
  using steady_clock = std::chrono::steady_clock;

  // Enable buttons again if the node is in INACTIVE state or plotting completed
  if (this->m_pdvg_node_->pdvg_button_enabled()){
    this->m_enable_button_(this->m_pdvg_button_, this->m_pdvg_button_str_);
  }
  if (this->m_pdvg_node_->plot_button_enabled()){
    this->m_enable_button_(this->m_plot_pd_button_, this->m_plot_pd_button_str_);
    this->m_enable_button_(this->m_error_budget_button_, this->m_error_budget_button_str_);
  }

  // If the PDVG node has a SUCCESS or FAILURE status
  auto status = this->m_pdvg_node_->get_pdvg_service_status();
  if (status == PDVGPanelNode::FAILURE || status == PDVGPanelNode::SUCCESS){
    // Set first timepoint to begin reporting interval
    if (!this->m_reporting_status_){
      this->m_reporting_status_ = true;
      this->m_report_start_time_ = steady_clock::now();

      auto status_message = this->m_pdvg_node_->get_pdvg_status_message();

      // Change button color and display status message
      if (status == PDVGPanelNode::FAILURE){
        this->m_button_show_error_(
          this->m_pdvg_button_, status_message);
      } else {
        this->m_button_show_success_(
          this->m_pdvg_button_, status_message);
      }
    }

    // Get current time
    auto current_time = steady_clock::now();

    // If the interval between the start and current time is at least 2 seconds
    auto period =
      std::chrono::duration_cast<milliseconds>(current_time - this->m_report_start_time_);
    if (period > 2000ms){
      // Set status to inactive to enable button again
      this->m_pdvg_node_->set_pdvg_status_inactive();
      this->m_reporting_status_ = false;
    }
  }

  // Do any work available to node
  this->m_executor_.spin_some();
}

/* == BUTTON UPDATE FUNCTIONS == */
/**
 * @brief Enable a button and set its message
 *
 * @param button: The button to modify
 * @param message: The message to set on the button
 */
void PDVGPanel::m_enable_button_(QToolButton* button, const std::string &message){
  // Enable button and set text to message
  button->setEnabled(true);
  button->setText(message.c_str());

  // Set button style to default (i.e. set default colors)
  button->setStyleSheet("");
  button->repaint();  // Update button
}

/**
 * @brief Disable a button and set its message
 *
 * @param button: The button to modify
 * @param message: The message to set on the button
 */
void PDVGPanel::m_disable_button_(QToolButton* button, const std::string &message){
  // Disable button and set text to message
  button->setEnabled(false);
  button->setText(message.c_str());
}

/**
 * @brief Disable all buttons in the visuals section of the panel
 */
void PDVGPanel::m_disable_all_viz_buttons_(){
  this->m_disable_button_(this->m_plot_pd_button_, this->m_plot_pd_button_str_);
  this->m_disable_button_(this->m_error_budget_button_, this->m_error_budget_button_str_);
}

/**
 * @brief Make a button red and change text to an error message
 *
 * @param message: Error message to display in the button text
 */
void PDVGPanel::m_button_show_error_(QToolButton* button, const std::string &message){
  this->m_disable_button_(button, message);

  // Set button color to red and the text color to white
  button->setStyleSheet("background-color: red; color: white");
  button->repaint();  // Update button
}

/**
 * @brief Make a button green and change text to a success message
 *
 * @param message: Success message to display in the button text
 */
void PDVGPanel::m_button_show_success_(QToolButton* button, const std::string &message){
  this->m_disable_button_(button, message);

  // Set button color to green and the text color to white
  button->setStyleSheet("background-color: green; color: white");
  button->repaint();  // Update button
}
}  // end namespace rviz_plugins

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PDVGPanel, rviz_common::Panel)
