/**
 * @File: lincov_analysis_panel.cpp
 * @Date: December 2022
 * @Author: Daren Swasey
 *
 * @brief
 * Implementation file for LinCov analysis panel used in rviz2
 * for simulation
 */

// Local Headers
#include "rviz_plugins/lincov_analysis_panel_node.hpp"
#include "rviz_plugins/lincov_analysis_panel.hpp"
#include "rviz_plugins/gui_tools/q_dropdown_checkbox.hpp"
#include "uav_interfaces/srv/toggle_ellipse.hpp"
#include "uav_interfaces/srv/lin_cov_monte_carlo_call.hpp"

// C++ Headers
#include <chrono>
#include <string>
#include <vector>

// QT GUI Headers
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QFrame>

namespace rviz_plugins{
using namespace std::chrono_literals;
using namespace std::placeholders;
using LinCovMonteCarloCall = uav_interfaces::srv::LinCovMonteCarloCall;
using QDropdownCheckbox = rviz_plugins::gui_tools::QDropdownCheckbox;

// Default strings for each of the buttons
const std::string LinCovAnalysisPanel::m_lincov_button_default_str_ = "LinCov Analysis";
const std::string LinCovAnalysisPanel::m_mc_button_default_str_     = "Monte Carlo Analysis";
const std::string LinCovAnalysisPanel::m_plot_button_default_str_   = "Gen. Plots";
const std::string LinCovAnalysisPanel::m_pdf_button_default_str_    = "Gen. PDF (All Plots)";
const std::string LinCovAnalysisPanel::m_ellipse_turn_on_str_       = "Turn Uncertainty Ellipses On";
const std::string LinCovAnalysisPanel::m_ellipse_turn_off_str_      = "Turn Uncertainty Ellipses Off";

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
LinCovAnalysisPanel::LinCovAnalysisPanel(QWidget* parent)
  : rviz_common::Panel(parent){
  this->m_node_ = std::make_shared<LinCovAnalysisPanelNode>();
  this-> m_executor_.add_node(this->m_node_);

  // Set default values
  this->m_res_value_ = 200.0;
  this->m_ellipses_enabled_ = false;
  this->m_reporting_status_ = false;
  this->m_last_button_clicked_ = LinCovMonteCarloCall::Request::LINCOV_ONLY;
  this->m_num_monte_carlo_runs_ = 10;
  this->m_mc_downsample_value_ = 1;

  // Create buttons
  this->m_lincov_button_ = new QToolButton();
  this->m_lincov_button_->setText(this->m_lincov_button_default_str_.c_str());
  this->m_lincov_button_->setToolTip("Perform a LinCov analysis");
  this->m_lincov_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  // Monte Carlo button
  this->m_mc_button_ = new QToolButton();
  this->m_mc_button_->setText(this->m_mc_button_default_str_.c_str());
  this->m_mc_button_->setToolTip("Perform a Monte Carlo analysis");
  this->m_mc_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

  // Create text box for number of Monte Carlo runs
  std::stringstream default_value;
  QHBoxLayout* mc_num_runs_box = new QHBoxLayout;
  mc_num_runs_box->addWidget(new QLabel("Monte Carlo Runs: "));
  this->m_num_monte_carlo_editor_ = new QLineEdit();
  default_value << this->m_num_monte_carlo_runs_;
  this->m_num_monte_carlo_editor_->insert(default_value.str().c_str());
  mc_num_runs_box->addWidget(this->m_num_monte_carlo_editor_);
  this->m_num_monte_carlo_editor_->setEnabled(true);

  // Create text box for editing Monte Carlo downsample factor
  QHBoxLayout* mc_downsample_box = new QHBoxLayout;
  mc_downsample_box->addWidget(new QLabel("MC Run Downsample Factor:"));
  this->m_mc_downsample_editor_ = new QLineEdit();

  // Set default text into editor
  default_value.str("");
  default_value << this->m_mc_downsample_value_;
  this->m_mc_downsample_editor_->insert(default_value.str().c_str());
  mc_downsample_box->addWidget(this->m_mc_downsample_editor_);
  this->m_mc_downsample_editor_->setEnabled(true);

  // Arrange plot and pdf buttons horizontally
  QHBoxLayout* plot_button_layout = new QHBoxLayout;

  // Plotting button
  this->m_plot_button_ = new QToolButton();
  this->m_plot_button_->setText(this->m_plot_button_default_str_.c_str());
  this->m_plot_button_->setToolTip("Make LinCov/Monte Carlo plots on screen");
  this->m_plot_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  this->m_plot_button_->setEnabled(false);  // Set initial state to disabled

  // PDF button
  this->m_pdf_button_ = new QToolButton();
  this->m_pdf_button_->setText(this->m_pdf_button_default_str_.c_str());
  this->m_pdf_button_->setToolTip("Save LinCov/Monte Carlo plots to file");
  this->m_pdf_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  this->m_pdf_button_->setEnabled(false);  // Set initial state to disabled

  plot_button_layout->addWidget(this->m_plot_button_);
  plot_button_layout->addWidget(this->m_pdf_button_);

  // Uncertainty ellipse toggle button
  this->m_ellipse_button_ = new QToolButton();
  this->m_ellipse_button_->setText(this->m_ellipse_turn_on_str_.c_str());
  this->m_ellipse_button_->setToolTip("Toggle 3-sigma uncertainty ellipse visibility");
  this->m_ellipse_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  this->m_ellipse_button_->setEnabled(false);  // Set initial state to disabled

  // Create text box for editing spatial resolution of uncertainty ellipses
  QHBoxLayout* spatial_res_box = new QHBoxLayout;
  spatial_res_box->addWidget(new QLabel("Ellipse Spacing (m):"));
  this->m_res_editor_ = new QLineEdit();

  // Set initial value to value set in constructor
  default_value.str("");
  default_value << this->m_res_value_;
  this->m_res_editor_->insert(default_value.str().c_str());
  spatial_res_box->addWidget(this->m_res_editor_);
  this->m_res_editor_->setEnabled(false);

  // Create checkbox list for plot type
  this->m_plot_type_list_ = new QDropdownCheckbox;
  this->m_plot_type_list_->set_default_text("Select type(s)...");

  // Add all required checkboxes and force display to show default string
  QStringList type_options = {"States", "Truth Disp", "Nav Disp", "Error",
                              "Truth Disp Off Ref", "Nav Disp Off Ref"};
  QStringList type_tips = {
    "States of selected data along reference values",
    "3-sigma truth state dispersions boundaries for LinCov and/or Monte Carlo.\
    Includes truth state dispersions for each Monte Carlo run if Monte Carlo analysis was performed",
    "3-sigma navigation state dispersions boundaries for LinCov and/or Monte Carlo.\
    Includes navigation state dispersions for each Monte Carlo run if Monte Carlo analysis was performed",
    "3-sigma estimated error boundaries",
    "Individual truth state dispersions with respect to reference trajectory 3-sigma dispersion bounds (MONTE CARLO ONLY)",
    "Individual navigation state dispersions with respect to reference trajectory 3-sigma dispersion bounds (MONTE CARLO ONLY)"
  };

  std::vector<std::pair<QString, QString>> type_info;
  for (int i = 0; i < type_options.size(); i++){
    type_info.emplace_back(std::pair<QString, QString>(type_options[i], type_tips[i]));
  }

  this->m_plot_type_list_->add_checkboxes(type_info);
  this->m_plot_type_list_->set_marked_idxs();

  // Create checkbox list for plot category
  this->m_data_type_list_ = new QDropdownCheckbox;
  this->m_data_type_list_->set_default_text("Select data...");

  // Add all required checkboxes and force display to show default string
  QStringList data_options = {"Position", "Euler", "Velocity", "Heading Bias",
                             "Abs Pressure Bias", "Feature Range Bias",
                             "Feature Bearing Bias", "GPS Position Bias",
                             "Gyro Bias", "Accel Bias"};
  QStringList data_tips = {
    "NED states",
    "Roll, pitch, yaw states",
    "NED velocity states",
    "Heading angle bias state",
    "Absolute pressure bias state",
    "Feature range bias state",
    "Feature bearing bias state",
    "GPS position bias state",
    "Gyroscope bias state",
    "Accelerometer bias state"
  };

  std::vector<std::pair<QString, QString>> data_info;
  for (int i = 0; i < data_options.size(); i++){
    data_info.emplace_back(std::pair<QString, QString>(data_options[i], data_tips[i]));
  }
  this->m_data_type_list_->add_checkboxes(data_info);
  this->m_data_type_list_->set_marked_idxs();

  // Add labels and checkbox lists to a layout
  QHBoxLayout* plot_selection_layout = new QHBoxLayout;
  QVBoxLayout* plot_type_layout = new QVBoxLayout;
  QVBoxLayout* plot_data_layout = new QVBoxLayout;

  QLabel* plot_type_label = new QLabel;
  QLabel* data_type_label = new QLabel;

  plot_type_label->setText("Plot Types");
  plot_type_label->setAlignment(Qt::AlignCenter | Qt::AlignVCenter);

  data_type_label->setText("Data Types");
  data_type_label->setAlignment(Qt::AlignCenter | Qt::AlignVCenter);

  // Make vertical layout for Type widgets
  plot_type_layout->addWidget(plot_type_label);
  plot_type_layout->addWidget(this->m_plot_type_list_);

  // Make vertical layout for Category widgets
  plot_data_layout->addWidget(data_type_label);
  plot_data_layout->addWidget(this->m_data_type_list_);

  // Place both vertical layouts in a horizontal layout
  plot_selection_layout->addLayout(plot_type_layout);
  plot_selection_layout->addLayout(plot_data_layout);

  // Make divider to indicate separation between analysis buttons and visual tools buttons
  QFrame* divider = new QFrame;
  divider->setFrameShape(QFrame::HLine);
  divider->setFrameShadow(QFrame::Sunken);

  // Make label for clearer distinction of separate GUI parts
  QLabel* plotting_label = new QLabel;
  plotting_label->setText("Visual Tools");
  plotting_label->setAlignment(Qt::AlignCenter | Qt::AlignVCenter);
  plotting_label->setStyleSheet("font-weight: bold");

  // Create top level layout (order of adding is order of appearance in GUI)
  QVBoxLayout* layout = new QVBoxLayout;

  layout->addWidget(this->m_lincov_button_);
  layout->addWidget(this->m_mc_button_);
  layout->addLayout(mc_num_runs_box);
  layout->addWidget(divider);
  layout->addWidget(plotting_label);
  layout->addLayout(plot_selection_layout);
  layout->addLayout(mc_downsample_box);
  layout->addLayout(plot_button_layout);
  layout->addWidget(this->m_ellipse_button_);
  layout->addLayout(spatial_res_box);

  // Create all contents of layout(s)
  setLayout(layout);

  // Create timer for spinning panel and node
  this->m_lincov_timer_ = new QTimer(this);

  // Connect signals on buttons and text edit boxes to the appropriate functions
  connect(this->m_lincov_button_,          SIGNAL(clicked()),         this, SLOT(lincov_button_clicked()));
  connect(this->m_mc_button_,              SIGNAL(clicked()),         this, SLOT(mc_button_clicked()));
  connect(this->m_plot_button_,            SIGNAL(clicked()),         this, SLOT(plot_button_clicked()));
  connect(this->m_pdf_button_,             SIGNAL(clicked()),         this, SLOT(pdf_button_clicked()));
  connect(this->m_ellipse_button_,         SIGNAL(clicked()),         this, SLOT(toggle_ellipses()));
  connect(this->m_lincov_timer_,           SIGNAL(timeout()),         this, SLOT(spin_some()));
  connect(this->m_res_editor_,             SIGNAL(editingFinished()), this, SLOT(update_resolution()));
  connect(this->m_num_monte_carlo_editor_, SIGNAL(editingFinished()), this, SLOT(update_monte_carlo_runs()));
  connect(this->m_mc_downsample_editor_,   SIGNAL(editingFinished()), this, SLOT(update_mc_downsample()));

  // Start the timer
  this->m_lincov_timer_->start(100);
}

/* === PANEL SLOTS === */
/**
 * @brief Starts LinCov analysis when "LinCov Analysis" button is clicked
 *
 * @details Once the "LinCov Analysis" button has been clicked, this function
 * is used to call the LinCovAnalysisPanelNode::analysis_request() function.
 * This function also disables buttons so during the LinCov analysis, other
 * service requests cannot be made. Buttons are reenabled via flag checks
 * in the main thread.
 */
void LinCovAnalysisPanel::lincov_button_clicked(){
  // Ensure other button is in the correct state
  this->m_reporting_status_ = false;
  this->m_enable_button_(this->m_mc_button_, this->m_lincov_button_default_str_);

  // Set flag so GUI updates the correct button with return status of analysis
  this->m_last_button_clicked_ = LinCovMonteCarloCall::Request::LINCOV_ONLY;

  // Turn off ellipses if they are on
  if (this->m_ellipses_enabled_){
    this->toggle_ellipses();
  }

  this->m_node_->analysis_request(this->m_num_monte_carlo_runs_, LinCovMonteCarloCall::Request::LINCOV_ONLY);

  // Update text and disable button so multiple calls aren't issued
  this->m_disable_button_(this->m_lincov_button_, "LinCov Processing...");
  this->m_disable_button_(this->m_mc_button_, this->m_mc_button_default_str_);
  this->m_disable_button_(this->m_plot_button_, this->m_plot_button_default_str_);
  this->m_disable_button_(this->m_pdf_button_, this->m_pdf_button_default_str_);
  this->m_ellipse_button_->setEnabled(false);
  this->m_res_editor_->setEnabled(false);
}

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
void LinCovAnalysisPanel::mc_button_clicked(){
  // Ensure other button is in the correct state
  this->m_reporting_status_ = false;
  this->m_enable_button_(this->m_lincov_button_, this->m_lincov_button_default_str_);

  // Set flag so GUI updates the correct button with return status of analysis
  this->m_last_button_clicked_ = LinCovMonteCarloCall::Request::BOTH;

  // Turn off ellipses if they are on
  if (this->m_ellipses_enabled_){
    this->toggle_ellipses();
  }

  this->m_node_->analysis_request(this->m_num_monte_carlo_runs_, LinCovMonteCarloCall::Request::BOTH);
  // Update text and disable button so multiple calls aren't issued
  this->m_disable_button_(this->m_mc_button_, "Monte Carlo Processing...");
  this->m_disable_button_(this->m_lincov_button_, this->m_lincov_button_default_str_);
  this->m_disable_button_(this->m_plot_button_, this->m_plot_button_default_str_);
  this->m_disable_button_(this->m_pdf_button_, this->m_pdf_button_default_str_);
  this->m_ellipse_button_->setEnabled(false);
  this->m_res_editor_->setEnabled(false);
}

/**
 * @brief Make a call for a service request to plot analysis results
 *
 * @details Disables the "Generate Plots" button and makes a service request
 * via the LinCovAnalysisPanelNode::plot_request() function. Results will show
 * only LinCov if only the LinCov analysis was requested, and will show results
 * for both if a Monte Carlo analysis was requested.
 */
void LinCovAnalysisPanel::plot_button_clicked(){
  // Disable the "Generate Plots" button and make the request
  this->m_disable_button_(this->m_plot_button_, this->m_plot_button_default_str_);
  this->m_disable_button_(this->m_pdf_button_, this->m_pdf_button_default_str_);

  // Make plot request with currently selected plot types and data
  this->m_node_->plot_request(
    false,
    this->m_mc_downsample_value_,
    this->m_plot_type_list_->get_checked(),
    this->m_data_type_list_->get_checked());
}

/**
 * @brief Make a call for a service request to generate pdf of all plots
 *
 * @details Disables the "Generate PDF" button and makes a service request
 * via the LinCovAnalysisPanelNode::plot_request() function.
 */
void LinCovAnalysisPanel::pdf_button_clicked(){
  // Disable plotting-related buttons to prevent request buffering
  this->m_disable_button_(this->m_plot_button_, this->m_plot_button_default_str_);
  this->m_disable_button_(this->m_pdf_button_, this->m_pdf_button_default_str_);

  // Make the request
  this->m_node_->plot_request(true, this->m_mc_downsample_value_);
}

/**
 * @brief Update desired spatial resolution for plotting uncertainty ellipses
 *
 * @details Updates the spatial resolution internal variable. Update takes
 * place after the "Enter" key has been pressed when entering the number
 * or after a mouse click occurs outside the text entry box.
 */
void LinCovAnalysisPanel::update_resolution(){
  // Attempt to convert the textbox to a float
  bool converted = false;
  float output = this->m_res_editor_->text().toFloat(&converted);

  // If possible then store the value
  if(converted && output > 0.0f) {
    this->m_res_value_ = output;
  }

  // Put the value back in the box
  std::stringstream text;
  text << this->m_res_value_;
  this->m_res_editor_->clear();
  this->m_res_editor_->insert(text.str().c_str());
}

/**
 * @brief Update the number of Monte Carlo runs specified by user
 *
 * @details Updates the Monte Carlo runs internal variable. Update takes
 * place after the "Enter" key has been pressed when entering the number
 * or after a mouse click occurs outside the text entry box.
 */
void LinCovAnalysisPanel::update_monte_carlo_runs(){
  // Attempt to convert the textbox to a float
  bool converted = false;
  float output = this->m_num_monte_carlo_editor_->text().toFloat(&converted);

  // If possible then store the value
  if(converted && output > 0) {
    this->m_num_monte_carlo_runs_ = output;
  }

  // Put the z_value back in the box
  std::stringstream text;
  text << this->m_num_monte_carlo_runs_;
  this->m_num_monte_carlo_editor_->clear();
  this->m_num_monte_carlo_editor_->insert(text.str().c_str());
}

/**
 * @brief Update the Monte Carlo data downsample rate
 *
 * @details Updates the Monte Carlo downsample rate value.
 * This value is used to reduce the number of total samples used
 * to make MC plots
 */
void LinCovAnalysisPanel::update_mc_downsample(){
  // Attempt to convert the textbox to a float
  bool converted = false;
  float output = this->m_mc_downsample_editor_->text().toInt(&converted);

  // If possible then store the value
  if(converted && output > 0) {
    this->m_mc_downsample_value_ = output;
  }

  // Put the z_value back in the box
  std::stringstream text;
  text << this->m_mc_downsample_value_;
  this->m_mc_downsample_editor_->clear();
  this->m_mc_downsample_editor_->insert(text.str().c_str());
}

/**
 * @brief Enable or disable uncertainty ellipses
 *
 * @details Toggle the uncertainty ellipses on or off and set the state of
 * the button to display the appropriate message.
 */
void LinCovAnalysisPanel::toggle_ellipses(){
  std::vector<std::string> ellipse_strings = {this->m_ellipse_turn_on_str_, this->m_ellipse_turn_off_str_};
  this->m_ellipses_enabled_ = !this->m_ellipses_enabled_;  // Toggle

  // Disable button until callback finished and set text to corresponding state of toggle (off or on)
  // Use bool to select what current message should be
  this->m_disable_button_(this->m_ellipse_button_,
                          ellipse_strings[this->m_ellipses_enabled_]);

  // Disable setting resolution until callback finishes
  this->m_res_editor_->setEnabled(false);
  this->m_node_->toggle_ellipse_request(this->m_res_value_);
}

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
void LinCovAnalysisPanel::spin_some(){
  using milliseconds = std::chrono::milliseconds;
  using steady_clock = std::chrono::steady_clock;
  // Re-enable buttons and change text back to original if most recent analysis is complete.
  if (this->m_node_->analysis_button_enabled() && !this->m_reporting_status_){
    this->m_enable_button_(this->m_lincov_button_, this->m_lincov_button_default_str_);
    this->m_enable_button_(this->m_mc_button_, this->m_mc_button_default_str_);
  }
  if (this->m_node_->plot_button_enabled()){
    this->m_enable_button_(this->m_plot_button_, this->m_plot_button_default_str_);
    this->m_enable_button_(this->m_pdf_button_, this->m_pdf_button_default_str_);
  }
  if (this->m_node_->toggle_ellipses_enabled()){
    // Whatever current state ellipse button is in, the text should be correct, just enable
    this->m_ellipse_button_->setEnabled(true);
    this->m_res_editor_->setEnabled(true);
  }

  // If the analysis node has a SUCCESS or FAILURE status
  auto status = this->m_node_->get_analysis_status();
  QToolButton* button;
  if (this->m_last_button_clicked_ == LinCovMonteCarloCall::Request::LINCOV_ONLY) {
    button = this->m_lincov_button_;
  } else {
    button = this->m_mc_button_;
  }
  if (status == LinCovAnalysisPanelNode::FAILURE || status == LinCovAnalysisPanelNode::SUCCESS){
    // Set first timepoint to begin reporting interval
    if (!this->m_reporting_status_){
      this->m_reporting_status_ = true;
      this->m_report_start_time_ = steady_clock::now();

      auto status_message = this->m_node_->get_status_message();

      // Change button color and display status message
      if (status == LinCovAnalysisPanelNode::FAILURE){
        this->m_button_show_error_(
          button, status_message);
      } else {
        this->m_button_show_success_(
          button, status_message);
      }
    }

    // Get current time
    auto current_time = steady_clock::now();

    // If the interval between the start and current time is at least 2 seconds
    auto period =
      std::chrono::duration_cast<milliseconds>(current_time - this->m_report_start_time_);
    if (period > 2000ms){
      // Set status to inactive to enable button again
      this->m_node_->set_analysis_status_inactive();
      this->m_reporting_status_ = false;
    }
  }

  // Perform work currently available to node
  this->m_executor_.spin_some();
}

/* === PANEL BUTTON UPDATE FUNCTIONS === */
/**
 * @brief Enable a button and set its message
 *
 * @param button: The button to modify
 * @param message: The message to set on the button
 */
void LinCovAnalysisPanel::m_enable_button_(QToolButton* button, const std::string &message){
  // Change the text to the value of the incoming message and enable the button
  button->setText(message.c_str());
  button->setEnabled(true);

  // Set button color to default
  button->setStyleSheet("");
  button->repaint();  // Update button
}

/**
 * @brief Disable a button and set its message
 *
 * @param button: The button to modify
 * @param message: The message to set on the button
 */
void LinCovAnalysisPanel::m_disable_button_(QToolButton* button, const std::string &message){
  // Change the text to the value of the incoming message and disable the button
  button->setText(message.c_str());
  button->setEnabled(false);
}

/**
 * @brief Make a button red and change text to an error message
 *
 * @param button: Button to modify
 * @param message: Error message to display in the button text
 */
void LinCovAnalysisPanel::m_button_show_error_(QToolButton* button, const std::string &message){
  // Disable the specified button with a failure message returned in the service callback
  this->m_disable_button_(button, message);

  // Set button color to red and the text color to white
  button->setStyleSheet("background-color: red; color: white");
  button->repaint();  // Update button
}

/**
 * @brief Make a button green and change text to a success message
 *
 * @param button: Button to modify
 * @param message: Success message to display in the button text
 */
void LinCovAnalysisPanel::m_button_show_success_(QToolButton* button, const std::string &message){
  // Disable the specified button with a failure message returned in the service callback
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
PLUGINLIB_EXPORT_CLASS(rviz_plugins::LinCovAnalysisPanel, rviz_common::Panel)
