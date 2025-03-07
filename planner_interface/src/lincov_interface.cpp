/**
 * @File: lincov_interface.cpp
 * @Date: January 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Interface implementation for running LinCov estimations and
 * Monte Carlo verifications in pd_planner simulation.
 */

// Local headers
#include <kalman_filter/dynamics/basic_model.hpp>
#include <kalman_filter/controllers/open_loop_controller.hpp>
#include <kalman_filter/sensors/inertial_measurements/open_loop_imu.hpp>
#include <kalman_filter/sensors/measurements/controllers/all_sensors_controller.hpp>
#include <kalman_filter/sensors/measurements/feature_range.hpp>
#include <kalman_filter/sensors/measurements/feature_bearing.hpp>
#include <kalman_filter/noise/multi_noise.hpp>
#include <kalman_filter/mappings/simple_mapping.hpp>
#include <kalman_filter/run_monte_carlo.hpp>
#include <planner_interface/csv_tools.hpp>
#include <planner_interface/lincov_interface.hpp>
#include <planner_interface/make_statistics_csv.hpp>
#include <rrt_search/edge_generators/fillets/imu/euler_spiral_imu_edge_generator.hpp>
#include <rrt_search/helpers/connect_waypoints.hpp>
#include <rrt_search/helpers/rrt_math.hpp>
#include <uav_interfaces/msg/uav_waypoints.hpp>
#include <uav_interfaces/srv/toggle_ellipse.hpp>
#include <uav_interfaces/srv/select_analysis_plot.hpp>

// Eigen headers
#include <Eigen/Dense>

// Global headers
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// ROS2 Headers
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::placeholders;
using BasicModelDim = kf::dynamics::BasicModelDim;
using LinCovMonteCarloCall = uav_interfaces::srv::LinCovMonteCarloCall;
using SelectAnalysisPlot = uav_interfaces::srv::SelectAnalysisPlot;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

namespace planner_interface{

const std::string LinCovInterface::m_diag_msg_available_ = "Available for service call";
const std::string LinCovInterface::m_diag_msg_busy_ = "Busy...";

/* === PUBLIC FUNCTIONS === */
/**
 * @brief Construct a new LinCov Interface object
 *
 * @details Create callback groups and all interfaces to the node
 *
 */
LinCovInterface::LinCovInterface()
: Node("lincov_interface"){
  using namespace std::chrono_literals;
  // Initialize all flags. All but m_params_initialized are for interactions
  // with the LinCov Analysis panel
  this->m_have_waypoints_ = false;
  this->m_plot_ready_ = false;
  this->m_ellipses_enabled_ = false;
  this->m_params_initialized_ = false;

  // Define callback groups and set options for subscribers
  this->m_analysis_cb_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = this->m_analysis_cb_group_;

  // Create subscriber
  this->m_waypoints_sub_ = this->create_subscription<uav_interfaces::msg::UavWaypoints>(
    "waypoints", 5, std::bind(&LinCovInterface::waypoints_sub_callback, this, _1), sub_options
  );

  // Get the directory for storing plot data
  this->declare_parameter("plotting_data_directory", rclcpp::PARAMETER_STRING);
  this->m_plot_data_directory_ = this->get_parameter("plotting_data_directory").as_string();

  // Create publisher
  this->m_lincov_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("lincov_viz", 1);

  // Create diagnostics publisher
  this->m_diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
  this->m_diag_timer_ =
    rclcpp::create_timer(this,
      this->get_clock(),
      1.0s,
      std::bind(&LinCovInterface::publish_diagnostics, this),
      this->m_analysis_cb_group_);

  // Create services
  // Service used for requesting LinCov analysis
  this->m_button_server_ptr_ = this->create_service<LinCovMonteCarloCall>(
    "run_analysis",
    std::bind(&LinCovInterface::analysis_callback, this, _1, _2),
    rmw_qos_profile_services_default, this->m_analysis_cb_group_);

  // Service used for requesting to plot data after LinCov analysis
  this->m_plotting_button_ptr_ = this->create_service<SelectAnalysisPlot>(
    "plot_results",
    std::bind(&LinCovInterface::gen_plots_callback, this, _1, _2),
    rmw_qos_profile_services_default, this->m_analysis_cb_group_);

  // Service used for turning uncertainty ellipses on or off
  this->m_toggle_ellipses_ptr_ = this->create_service<uav_interfaces::srv::ToggleEllipse>(
    "toggle_uncertainty_ellipses",
    std::bind(&LinCovInterface::toggle_ellipses_callback, this, _1, _2),
    rmw_qos_profile_services_default, this->m_analysis_cb_group_);

  // Service for making requests to CSV plotting node
  this->m_csv_plot_client_ = this->create_client<SelectAnalysisPlot>(
    "csv_plotter",
    rmw_qos_profile_services_default,
    this->m_analysis_cb_group_
  );

  this->m_diagnostic_status_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  this->m_diagnostic_status_.name = "LinCov Interface";
  this->m_diagnostic_status_.hardware_id = "sim";
  this->m_diagnostic_status_.message = "Initialized";
}

/**
 * @brief Set up info for selected plots from analysis results
 *
 * @details Set up plot info for LinCov or Monte Carlo analysis depending
 * on the current state of the data. The plotAllStatistics function has
 * a few overloads that are used here for cleanliness.
 * LinCov data can be plotted alone if desired, but to plot Monte Carlo data,
 * the LinCov analysis must also have been completed. This function checks
 * the flags that indicate which analyses have been performed and generates
 * the relevant plots.
 */
void LinCovInterface::plot(){
  using REF = BasicModelDim::REF;
  using TRUTH = BasicModelDim::TRUTH;
  using TRUTH_DISP = BasicModelDim::TRUTH_DISP;
  using ERROR = BasicModelDim::ERROR;
  using PlotTypes = uav_interfaces::srv::SelectAnalysisPlot_Request;
  using PlotTitles = planner_interface::csvtools::PlotTitles;
  using namespace std::chrono_literals;

  // Make bool for if truth/nav disp off ref is requested
  bool mc_only_requested =
    (this->m_plot_types_[PlotTypes::TRUTH_DISP_OFF_REF] ||
     this->m_plot_types_[PlotTypes::NAV_DISP_OFF_REF]);

  // If a Monte Carlo run hasn't been done yet and the PDF is requested
  // or MC only plot is requested, force MC call so data used is for the current
  // set of waypoints.
  if (!this->m_did_monte_carlo_ && (this->m_save_plot_pdf_ || mc_only_requested)){
    this->monte_carlo_analysis();
  }
  this->log_diagnostics(DiagnosticStatus::OK, "Generating plots...");

  // Make vector to hold plot information specified by user
  std::vector<std::pair<std::string, std::tuple<Eigen::Index,
                                                Eigen::Index,
                                                Eigen::Index,
                                                Eigen::Index>>> plot_data_names;

  /*
   * In this section, look at the requested information
   * and only add that information to the list of things
   * to generate plots for.
   */
  std::lock_guard<std::mutex> lock(this->m_data_mutex_);
  if (this->m_plot_types_[PlotTypes::STATE] && !this->m_did_monte_carlo_){
    RCLCPP_WARN(this->get_logger(),
    "Warning: If state plots are selected for LinCov-only plots, data is only produced for Position, Euler, and Velocity data types");
  }

  if (this->m_plot_data_types_[PlotTypes::POSITION]){
    plot_data_names.emplace_back(
      std::make_pair(PlotTitles::POSITION,
        std::make_tuple(3, REF::POS_START_IND, TRUTH_DISP::POS_START_IND, ERROR::POS_START_IND))
    );
  }

  if (this->m_plot_data_types_[PlotTypes::EULER]){
    plot_data_names.emplace_back(
      std::make_pair(PlotTitles::EULER,
        std::make_tuple(3, REF::EULER_START_IND, TRUTH_DISP::EULER_START_IND, ERROR::EULER_START_IND))
    );
  }

  if (this->m_plot_data_types_[PlotTypes::VELOCITY]){
    plot_data_names.emplace_back(
      std::make_pair(PlotTitles::VELOCITY,
        std::make_tuple(3, REF::VEL_START_IND, TRUTH_DISP::VEL_START_IND, ERROR::VEL_START_IND))
    );
  }

  if (this->m_plot_data_types_[PlotTypes::HEADING]){
    plot_data_names.emplace_back(
      std::make_pair(PlotTitles::HEADING,
        std::make_tuple(1, -1, TRUTH_DISP::HEADING_BIAS_IND, ERROR::HEADING_BIAS_IND))
    );
  }

  if (this->m_plot_data_types_[PlotTypes::ABS_PRESSURE]){
    plot_data_names.emplace_back(
      std::make_pair(PlotTitles::ABS_PRESSURE,
        std::make_tuple(1, -1, TRUTH_DISP::ABS_PRESSURE_BIAS_IND, ERROR::ABS_PRESSURE_BIAS_IND))
    );
  }

  if (this->m_plot_data_types_[PlotTypes::FEATURE_RANGE]){
    plot_data_names.emplace_back(
      std::make_pair(PlotTitles::FEATURE_RANGE,
        std::make_tuple(1, -1, TRUTH_DISP::FEATURE_RANGE_BIAS_IND, ERROR::FEATURE_RANGE_BIAS_IND))
    );
  }

  if (this->m_plot_data_types_[PlotTypes::FEATURE_BEARING]){
    plot_data_names.emplace_back(
      std::make_pair(PlotTitles::FEATURE_BEARING,
        std::make_tuple(3, -1, TRUTH_DISP::FEATURE_BEARING_BIAS_START_IND, ERROR::FEATURE_BEARING_BIAS_START_IND))
    );
  }

  if (this->m_plot_data_types_[PlotTypes::GPS]){
    plot_data_names.emplace_back(
      std::make_pair(PlotTitles::GPS,
        std::make_tuple(3, -1, TRUTH_DISP::GPS_POS_BIAS_START_IND, ERROR::GPS_POS_BIAS_START_IND))
    );
  }

  if (this->m_plot_data_types_[PlotTypes::GYRO]){
    plot_data_names.emplace_back(
      std::make_pair(PlotTitles::GYRO,
        std::make_tuple(3, -1, TRUTH_DISP::GYRO_BIAS_START_IND, ERROR::GYRO_BIAS_START_IND))
    );
  }

  if (this->m_plot_data_types_[PlotTypes::ACCEL]){
    plot_data_names.emplace_back(
      std::make_pair(PlotTitles::ACCEL,
        std::make_tuple(3, -1, TRUTH_DISP::ACCEL_BIAS_START_IND, ERROR::ACCEL_BIAS_START_IND))
    );
  }

  // If a Monte Carlo analysis has been run...
  if (this->m_did_monte_carlo_){
    // Generate a temporary function to map input to output
    const auto temp_func = [](const Eigen::Matrix<double,
                              Eigen::Dynamic,
                              BasicModelDim::TRUTH_DIM,
                              Eigen::RowMajor>& input) -> Eigen::Matrix<double, Eigen::Dynamic, BasicModelDim::TRUTH_DISP_DIM, Eigen::RowMajor>
    {
      Eigen::Matrix<double, Eigen::Dynamic, BasicModelDim::TRUTH_DISP_DIM, Eigen::RowMajor> output;
      output.resize(input.rows(), Eigen::NoChange);

      // Store all the non-quaternion states in the function output
      output.template leftCols<TRUTH_DISP::EULER_START_IND>() =
        input.template leftCols<TRUTH::QUAT_START_IND>();

      output.template rightCols<BasicModelDim::TRUTH_DISP_DIM-TRUTH_DISP::EULER_END_IND-1>() =
          input.template rightCols<BasicModelDim::TRUTH_DIM-TRUTH::QUAT_END_IND-1>();

      // Store the quaternion state in the function output after converting to Euler
      output.template middleCols<3>(BasicModelDim::TRUTH_DISP::EULER_START_IND) =
        kf::math::quat::quaternionToRollPitchYaw(input.template middleCols<4>(TRUTH::QUAT_START_IND));

      return output;
    };

    // Plot Monte Carlo and LinCov via CSV files
    planner_interface::csv::makeAllStatisticsCSV<BasicModelDim, double, Eigen::RowMajor>(
      this->m_monte_carlo_output,
      this->m_lincov_sim_vec_,
      this->m_tools_.mappings,
      temp_func,
      temp_func,
      plot_data_names,
      this->m_plot_types_,
      this->m_plot_data_directory_);
  } else {
    // Plot LinCov-only via CSV files
    // Only the first 4 options in the plot types vector have corresponding LinCov-only plots, copy over
    std::array<bool, 4> lincov_plot_types;
    std::copy_n(this->m_plot_types_.begin(), 4, lincov_plot_types.begin());

    planner_interface::csv::makeAllStatisticsCSV<BasicModelDim, double, Eigen::RowMajor>(
      this->m_lincov_sim_vec_,
      this->m_tools_.mappings,
      plot_data_names,
      lincov_plot_types,
      this->m_plot_data_directory_);
  }

  // Make request for CSV-based plotting
  if (!this->m_csv_plot_client_->wait_for_service(1s)) {
    // If ROS is shutdown before the service is activated, show this error
    if (!rclcpp::ok()) {
      this->log_diagnostics(DiagnosticStatus::ERROR, "ROS shutdown occurred");
      return;
    }
    // Print in the screen some information so the user knows what is happening
    this->log_diagnostics(DiagnosticStatus::WARN, "CSV plotting unavailable");
    return;
  }

  // This is just a passthrough of previous information from GUI request
  auto request = std::make_shared<SelectAnalysisPlot::Request>();
  request->save_to_pdf = this->m_save_plot_pdf_;
  request->plot_types = this->m_plot_types_;
  request->data_types = this->m_plot_data_types_;
  request->mc_run_downsample = this->m_mc_run_downsample_;
  request->data_directory = this->m_plot_data_directory_;

  this->m_csv_plot_client_->async_send_request(
    request,
    std::bind(&LinCovInterface::csv_plot_callback, this, _1)
  );
}

/* === PRIVATE FUNCTIONS === */

/* === PARAMETER DECLARATION FUNCTIONS === */
/**
 * @brief Declare and initialize trajectory-related parameters
 *
 * @details Gets and sets trajectory parameters and the analysis
 * step size. Node is designed to run only
 * if these parameters are passed in at launch
 */
void LinCovInterface::declare_trajectory_params(){
  // Declare trajectory-related parameters and time step
  this->declare_parameter("trajectory.nominal_velocity", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("trajectory.max_curvature", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("trajectory.max_curvature_rate", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("trajectory.nominal_pitch", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("trajectory.gravity", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("analysis_time_step", rclcpp::PARAMETER_DOUBLE);

  // Assign parameter values to node variables
  this->m_nominal_velocity_    = this->get_parameter("trajectory.nominal_velocity").as_double();
  this->m_max_curvature_       = this->get_parameter("trajectory.max_curvature").as_double();
  this->m_max_curvature_rate_  = this->get_parameter("trajectory.max_curvature_rate").as_double();
  this->m_nominal_pitch_       = this->get_parameter("trajectory.nominal_pitch").as_double();
  this->m_nominal_pitch_       = rrt::math::angleToRadian<double>(this->m_nominal_pitch_);
  this->m_gravity_             = this->get_parameter("trajectory.gravity").as_double();
  this->m_analysis_dt_         = this->get_parameter("analysis_time_step").as_double();
}

/**
 * @brief Declare and initialize feature-related parameters
 *
 * @details Gets and sets NED positions and range for each buoy.
 * Node is designed to run only if these parameters are passed in at launch.
 */
void LinCovInterface::declare_feature_params(){
  // Declare buoy/feature-related parameters
  this->declare_parameter("features.feature_locations_x", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("features.feature_locations_y", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("features.feature_locations_z", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("features.feature_ranges", rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Assign parameter values to node variables
  this->m_feature_locations_x_  = this->get_parameter("features.feature_locations_x").as_double_array();
  this->m_feature_locations_y_  = this->get_parameter("features.feature_locations_y").as_double_array();
  this->m_feature_locations_z_  = this->get_parameter("features.feature_locations_z").as_double_array();
  this->m_feature_ranges_       = this->get_parameter("features.feature_ranges").as_double_array();

  auto x_len = this->m_feature_locations_x_.size();

  // Verify these are all the same length
  if (!((x_len == this->m_feature_locations_y_.size()) &&
        (x_len == this->m_feature_locations_z_.size()) &&
        (x_len == this->m_feature_ranges_.size()))) {
    throw(std::runtime_error("Feature location/range parameter lengths are not the same"));
  }
}

/**
 * @brief Declare and initialize vizualization parameters
 *
 * @details Gets and sets parameters for the color, scale, and frame id
 * for plotting uncertainty ellipses. Node is designed to run only if
 * these parameters are passed in at launch.
 */
void LinCovInterface::declare_viz_params(){
  // Declare parameters for plotting uncertainty ellipses
  this->declare_parameter("ellipses.color", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("ellipses.test_scale", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("ellipses.frame_id", rclcpp::PARAMETER_STRING);

  // Assign parameter values to node variables
  auto color = this->get_parameter("ellipses.color").as_double_array();

  // Verify color array has the correct length
  if (color.size() != 4){
    throw(std::runtime_error("Input color array has the wrong dimensions! It should be of size 4"));
  }

  this->m_ellipse_frame_id_ =     this->get_parameter("ellipses.frame_id").as_string();
  std::copy(color.begin(), color.end(), this->m_rviz_ellipse_rgba_.begin());
}

/* === TOOL CREATION FUNCTIONS === */
/**
 * @brief Create tools for performing LinCov or Monte Carlo analysis
 */
void LinCovInterface::init_tools(){
  // Initialize all lincov components that don't change between calls
  this->init_inertial_measure_tools();
  this->init_dynamics_tool();
  this->init_controller_tool();
  this->init_measurement_controller_tool();
  this->init_mappings_tool();
}

/**
 * @brief Create inertial measurements function for tools
 *
 * @details Makes a measurement function for IMU readings for both LinCov
 * and Monte Carlo analyses. Also declares and gets parameters required for initializing
 * analysis state vectors.
 */
void LinCovInterface::init_inertial_measure_tools(){
  this->m_tools_.inertial_measurements = std::make_shared<kf::sensors::OpenLoopIMU<BasicModelDim, double, Eigen::RowMajor>>();

  const auto accel_noise = kf::noise::makeNormalDistribution<3, double, Eigen::RowMajor>(this->shared_from_this(), "imu.accelerometer.noise");
  const auto gyro_noise = kf::noise::makeNormalDistribution<3, double, Eigen::RowMajor>(this->shared_from_this(), "imu.gyroscope.noise");

  this->m_accel_std_dev_ = this->get_parameter("imu.accelerometer.noise.standard_deviation").as_double_array()[0];
  this->m_gyro_std_dev_ = this->get_parameter("imu.gyroscope.noise.standard_deviation").as_double_array()[0];

  this->m_tools_.inertial_measurements_noise =
    std::make_shared<kf::noise::MultiNoise<BasicModelDim::INER_MEAS_NOISE_DIM>>(
      std::vector<std::pair<kf::noise::NoiseBasePtr<1>,Eigen::Index>>(),
      std::vector<std::pair<kf::noise::NoiseBasePtr<2>,Eigen::Index>>(),
      std::vector<std::pair<kf::noise::NoiseBasePtr<3>,Eigen::Index>>({
        std::make_pair(accel_noise, BasicModelDim::INER_MEAS_NOISE::ACCEL_START_IND),
        std::make_pair(gyro_noise,  BasicModelDim::INER_MEAS_NOISE::GYRO_START_IND)}));
}

/**
 * @brief Create dynamics for tools and assign result to m_tools_.dynamics_func
 *
 * @details Uses all sensor bias noise vectors and time constants to create a
 * function that defines the navigation and truth state dynamics
 */
void LinCovInterface::init_dynamics_tool(){
  // Make truth process noise function
  const auto compass_bias_noise         = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<1, double>(this->shared_from_this(), "kalman_filter.dynamics.compass_bias");
  const auto abs_pressure_bias_noise    = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<1, double>(this->shared_from_this(), "kalman_filter.dynamics.abs_pressure_bias");
  const auto feature_range_bias_noise   = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<1, double>(this->shared_from_this(), "kalman_filter.dynamics.feature_range_bias");
  const auto feature_bearing_bias_noise = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3, double>(this->shared_from_this(), "kalman_filter.dynamics.feature_bearing_bias");
  const auto gps_position_bias_noise    = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3, double>(this->shared_from_this(), "kalman_filter.dynamics.gps_pos_bias");
  const auto accel_bias_noise           = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3, double>(this->shared_from_this(), "kalman_filter.dynamics.accel_bias");
  const auto gyro_bias_noise            = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3, double>(this->shared_from_this(), "kalman_filter.dynamics.gyro_bias");

  this->m_tools_.truth_process_noise =
    std::make_shared<kf::noise::MultiNoise<BasicModelDim::TRUTH_NOISE_DIM>>(
      std::vector<std::pair<kf::noise::NoiseBasePtr<1>,Eigen::Index>>({
        std::make_pair(compass_bias_noise,       BasicModelDim::TRUTH_NOISE::HEADING_BIAS_IND),
        std::make_pair(abs_pressure_bias_noise,  BasicModelDim::TRUTH_NOISE::ABS_PRESSURE_BIAS_IND),
        std::make_pair(feature_range_bias_noise, BasicModelDim::TRUTH_NOISE::FEATURE_RANGE_BIAS_IND),
        }),
      std::vector<std::pair<kf::noise::NoiseBasePtr<2>,Eigen::Index>>(),
      std::vector<std::pair<kf::noise::NoiseBasePtr<3>,Eigen::Index>>({
        std::make_pair(feature_bearing_bias_noise, BasicModelDim::TRUTH_NOISE::FEATURE_BEARING_BIAS_START_IND),
        std::make_pair(gps_position_bias_noise,    BasicModelDim::TRUTH_NOISE::GPS_POS_BIAS_START_IND),
        std::make_pair(accel_bias_noise,           BasicModelDim::TRUTH_NOISE::ACCEL_BIAS_START_IND),
        std::make_pair(gyro_bias_noise,            BasicModelDim::TRUTH_NOISE::GYRO_BIAS_START_IND),
        }));

  // Assign parameter values to node variables
  this->m_compass_time_const_          = this->get_parameter("kalman_filter.dynamics.compass_bias.time_constant").as_double_array()[0];
  this->m_abs_pressure_time_const_     = this->get_parameter("kalman_filter.dynamics.abs_pressure_bias.time_constant").as_double_array()[0];
  this->m_accel_time_const_            = this->get_parameter("kalman_filter.dynamics.accel_bias.time_constant").as_double_array()[0];
  this->m_gyro_time_const_             = this->get_parameter("kalman_filter.dynamics.gyro_bias.time_constant").as_double_array()[0];
  this->m_gps_position_time_const_     = this->get_parameter("kalman_filter.dynamics.gps_pos_bias.time_constant").as_double_array()[0];
  this->m_feature_range_time_const_    = this->get_parameter("kalman_filter.dynamics.feature_range_bias.time_constant").as_double_array()[0];
  this->m_feature_bearing_time_const_  = this->get_parameter("kalman_filter.dynamics.feature_bearing_bias.time_constant").as_double_array()[0];

  // Create the dynamics function using sensor and trajectory parameters
  this->m_tools_.dynamics =
    std::make_shared<kf::dynamics::BasicModel<kf::dynamics::BasicModelDim, double, Eigen::RowMajor>>(
    this->m_nominal_velocity_,
    this->m_gravity_,
    this->m_compass_time_const_,
    this->m_abs_pressure_time_const_,
    this->m_feature_range_time_const_,
    this->m_feature_bearing_time_const_,
    this->m_gps_position_time_const_,
    this->m_accel_time_const_,
    this->m_gyro_time_const_);
}

/**
 * @brief Create controller for tools and assign result to m_tools_.controller
 */
void LinCovInterface::init_controller_tool(){
  // Create the controller tool
  this->m_tools_.controller =
    std::make_shared<kf::control::OpenLoopController<BasicModelDim, double, Eigen::RowMajor>>();
}

/**
 * @brief Create measurements controller for tools
 *
 * @details Utilizes sensor parameters (i.e. if sensor is enabled, sensor's
 * measurement period, sensor noise parameters, and other sensor-specific
 * parameters) to provide tool for updating error covariance and augmented
 * covariance matrices. Also creates/uses objects that contain information about each
 * buoy/feature
 */
void LinCovInterface::init_measurement_controller_tool(){
  // Begin by creating vectors to store data for each individual buoy/feature
  const size_t n_features = this->m_feature_locations_x_.size();
  std::vector<kf::sensors::MeasurementBasePtr<1, BasicModelDim, double, Eigen::RowMajor>> feature_range_model;
  std::vector<kf::sensors::MeasurementBasePtr<2, BasicModelDim, double, Eigen::RowMajor>> feature_bearing_model;
  std::vector<kf::noise::NoiseBasePtr<        1,                double, Eigen::RowMajor>> feature_range_noise_vec;
  std::vector<kf::noise::NoiseBasePtr<        2,                double, Eigen::RowMajor>> feature_bearing_noise_vec;
  feature_range_model.resize(n_features);
  feature_bearing_model.resize(n_features);
  feature_range_noise_vec.resize(n_features);
  feature_bearing_noise_vec.resize(n_features);

  // Get initial feature range parameters
  this->declare_parameter("sensors.feature_range.enabled",            rclcpp::PARAMETER_BOOL);
  this->declare_parameter("sensors.feature_range.measurement_period", rclcpp::PARAMETER_DOUBLE);

  // Get initial feature bearing parameters
  this->declare_parameter("sensors.feature_bearing.enabled", rclcpp::PARAMETER_BOOL);
  this->declare_parameter("sensors.feature_bearing.measurement_period", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("sensors.feature_bearing.camera_offset", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("sensors.feature_bearing.camera_viewing_angles", rclcpp::PARAMETER_DOUBLE_ARRAY);

  const bool    range_enabled               = this->get_parameter("sensors.feature_range.enabled").as_bool();
  const double  range_measurement_period    = this->get_parameter("sensors.feature_range.measurement_period").as_double();
  const bool    bearing_enabled             = this->get_parameter("sensors.feature_bearing.enabled").as_bool();
  const double  bearing_measurement_period  = this->get_parameter("sensors.feature_bearing.measurement_period").as_double();

  const std::vector<double> feature_camera_offset_vec =     this->get_parameter("sensors.feature_bearing.camera_offset").as_double_array();
  const std::vector<double> feature_camera_viewing_angles = this->get_parameter("sensors.feature_bearing.camera_viewing_angles").as_double_array();

  // Make shared ptr to noise. It will be the same across all buoys
  auto feature_range_noise = kf::noise::makeNormalDistribution<1, double, Eigen::RowMajor>(this->shared_from_this(), "sensors.feature_range.noise");
  auto feature_bearing_noise = kf::noise::makeNormalDistribution<2, double, Eigen::RowMajor>(this->shared_from_this(), "sensors.feature_bearing.noise");

  // For every buoy/feature defined in the parameters
  for(size_t feature_it = 0; feature_it < n_features; ++feature_it)
  {
    feature_range_noise_vec[  feature_it] = feature_range_noise;
    feature_bearing_noise_vec[feature_it] = feature_bearing_noise;

    // Store the NED location
    Eigen::Matrix<double, 1, 3, Eigen::RowMajor> feature_location;
    feature_location[0] = this->m_feature_locations_x_[feature_it];
    feature_location[1] = this->m_feature_locations_y_[feature_it];
    feature_location[2] = this->m_feature_locations_z_[feature_it];

    // Create tools for getting vehicle-to-feature measurments
    if (range_enabled){
      // If the range is enabled make tool to take measurements
      feature_range_model[feature_it] =
      std::make_shared<kf::sensors::FeatureRange<BasicModelDim, true, true, double, Eigen::RowMajor>>(
        range_measurement_period,
        this->m_feature_ranges_[feature_it],
        feature_location);
    } else {
      // Otherwise, make the tool but disable measurements
      feature_range_model[feature_it] =
      std::make_shared<kf::sensors::FeatureRange<BasicModelDim, false, true, double, Eigen::RowMajor>>(
        range_measurement_period,
        this->m_feature_ranges_[feature_it],
        feature_location);
    }

    if (bearing_enabled){
      // If the range is enabled make tool to take measurements
      feature_bearing_model[feature_it] =
      std::make_shared<kf::sensors::FeatureBearing<BasicModelDim, true, true, double, Eigen::RowMajor>>(
        bearing_measurement_period,
        Eigen::Map<const Eigen::Matrix<double, 1, 3, Eigen::RowMajor>>(feature_camera_offset_vec.data()),
        Eigen::Map<const Eigen::Matrix<double, 1, 3, Eigen::RowMajor>>(feature_camera_viewing_angles.data()),
        this->m_feature_ranges_[feature_it],
        feature_location);
    } else {
      // Otherwise, make the tool but disable measurements
      feature_bearing_model[feature_it] =
      std::make_shared<kf::sensors::FeatureBearing<BasicModelDim, false, true, double, Eigen::RowMajor>>(
        bearing_measurement_period,
        Eigen::Map<const Eigen::Matrix<double, 1, 3, Eigen::RowMajor>>(feature_camera_offset_vec.data()),
        Eigen::Map<const Eigen::Matrix<double, 1, 3, Eigen::RowMajor>>(feature_camera_viewing_angles.data()),
        this->m_feature_ranges_[feature_it],
        feature_location);
    }
  }

  // Create measurement controller given all the sensor parameters
  this->m_tools_.measurement_controller =
    std::make_shared<kf::sensors::AllSensorsController<BasicModelDim, double, Eigen::RowMajor>>(
      kf::sensors::makeGPS<BasicModelDim, true, double>(this->shared_from_this(), "sensors.gps"),
      kf::noise::makeNormalDistribution<3>(this->shared_from_this(), "sensors.gps.noise"),
      kf::sensors::makeHeading<BasicModelDim, true, double>(this->shared_from_this(), "sensors.compass"),
      kf::noise::makeNormalDistribution<1>(this->shared_from_this(), "sensors.compass.noise"),
      kf::sensors::makeAbsolutePressure<BasicModelDim, true, double>(this->shared_from_this(), "sensors.abs_pressure"),
      kf::noise::makeNormalDistribution<1>(this->shared_from_this(), "sensors.abs_pressure.noise"),
      kf::sensors::makeGroundVelocity<BasicModelDim, double>(this->shared_from_this(), "sensors.ground_velocity"),
      kf::noise::makeNormalDistribution<1>(this->shared_from_this(), "sensors.ground_velocity.noise"),
      feature_range_model,
      feature_range_noise_vec,
      feature_bearing_model,
      feature_bearing_noise_vec);
}

/**
 * @brief Create mappings tool
 *
 * @details Create mappings tool that is used to extract parts of the reference state
 * vector and maps them to either a truth state or navigation state vector
 */
void LinCovInterface::init_mappings_tool(){
  // Create the mappings tool
  this->m_tools_.mappings =
    std::make_shared<kf::map::SimpleMapping<BasicModelDim, double, Eigen::RowMajor>>();
}

/* === ANALYSIS FUNCTIONS === */
/**
 * @brief Create the reference trajectory
 *
 * @details Creates the reference trajectory that will be used by both LinCov
 * and Monte Carlo analyses. This function must be used within a try-catch block.
 * If the path generated between 3 waypoints results in an angle too sharp for the UAV,
 * throws an error.
 */
void LinCovInterface::make_reference_trajectory(){
  using EdgeGen = rrt::edge::EulerSpiralIMUEdgeGeneratord;

  // Create a container for the reference trajectory
  Eigen::Matrix<double, Eigen::Dynamic, BasicModelDim::REF_DIM, Eigen::RowMajor> ref_trajectory;

  // Extract nominal down value from the published waypoints
  double nominal_down = this->m_waypoints_.points[0].position.point.z;

  // Make an path edge generator with the trajectory parameters
  rrt::edge::IMUSignalEdgeGeneratorPtrd edge_gen =
  std::make_shared<EdgeGen>(
    this->m_nominal_velocity_*this->m_analysis_dt_,
    this->m_max_curvature_,
    this->m_max_curvature_rate_,
    this->m_nominal_velocity_,
    this->m_nominal_pitch_,
    nominal_down,
    this->m_gravity_);

  // Create a list waypoints from the waypoints message as Eigen matrices
  std::list<Eigen::Matrix<double, 1, BasicModelDim::REF_DIM, Eigen::RowMajor>> waypoints;

  // Create waypoints from the waypoints message
  auto waypt_num = this->m_waypoints_.points.size();
  for (size_t waypt_it = 0; waypt_it < waypt_num; waypt_it++){
    waypoints.emplace_back();
    waypoints.back().setZero();
    waypoints.back()[EdgeGen::DIM::NORTH] = this->m_waypoints_.points[waypt_it].position.point.x;
    waypoints.back()[EdgeGen::DIM::EAST] = this->m_waypoints_.points[waypt_it].position.point.y;
  }
  // Set the orientation of each waypoint
  for (auto waypt_it = std::next(waypoints.begin()); waypt_it !=waypoints.end(); waypt_it++){
    (*waypt_it) = edge_gen->setOrientation((*waypt_it), (*std::prev(waypt_it)));
  }
  // Create the reference trajectory and store in a node variable
  this->m_ref_trajectory_ = rrt::connectWaypointsFillets<BasicModelDim::REF_DIM, double, Eigen::RowMajor>(waypoints, edge_gen);
}

/**
 * @brief Initialize LinCov and Monte Carlo state vectors
 *
 * @details Initializes the state vectors that will hold analysis results.
 * It is initialized with a reference trajectory and starting error and augmented
 * covariance matrices. Initialized values for the augmented covariance matrix depend
 * on accelerometer and gyroscope bias standard deviation parameters.
 */
void LinCovInterface::make_state_vectors(){
  // Make aliases for these numbers so code below is more readable
  using BMD   = BasicModelDim;
  using MC    = BMD::MC;
  //using ERROR = BMD::ERROR;
  using REF   = BMD::REF;
  using TRUTH = BMD::TRUTH;
  using LC    = BMD::LINCOV;

  // Make MC state vector
  this->m_init_sim_vec_.setZero();

  // Set the reference trajectory
  this->m_init_sim_vec_.middleCols<BMD::REF_DIM>(BMD::REF_START_IND) =
    this->m_ref_trajectory_.leftCols<BMD::REF_DIM>().row(0);

  // Set the time
  this->m_init_sim_vec_(0, BMD::TIME_IND) = 0;

  // Set all measurement columns to zero
  this->m_init_sim_vec_.middleCols<BMD::NUM_MEAS_DIM>(BMD::NUM_MEAS_START_IND).setConstant(1);

  // Set the starting state
  this->m_init_sim_vec_.row(0).middleCols<3>(MC::TRUTH_START_IND + TRUTH::POS_START_IND) =
      this->m_init_sim_vec_.row(0).middleCols<3>(BMD::REF_START_IND + REF::POS_START_IND);

  this->m_init_sim_vec_.row(0).middleCols<4>(MC::TRUTH_START_IND + TRUTH::QUAT_START_IND) =
    kf::math::quat::rollPitchYawToQuaternion(
      this->m_init_sim_vec_.row(0).middleCols<3>(BMD::REF_START_IND + REF::EULER_START_IND));

  this->m_init_sim_vec_.row(0).middleCols<3>(MC::TRUTH_START_IND + TRUTH::VEL_START_IND) =
    this->m_init_sim_vec_.row(0).middleCols<3>(BMD::REF_START_IND + REF::VEL_START_IND);

  this->m_init_sim_vec_.row(0).middleCols<6>(
    MC::TRUTH_START_IND + TRUTH::GYRO_BIAS_START_IND).setZero();

  this->m_init_sim_vec_.row(0).middleCols<BMD::NAV_DIM>(MC::NAV_START_IND) =
    this->m_init_sim_vec_.row(0).middleCols<BMD::TRUTH_DIM>(MC::TRUTH_START_IND);
  Eigen::Map<Eigen::Matrix<double, BMD::ERROR_DIM, BMD::ERROR_DIM, Eigen::RowMajor>> mc_error_covariance(
        this->m_init_sim_vec_.block<1, BMD::ERROR_COV_LEN>(0, BMD::ERROR_COV_START_IND).data());
  mc_error_covariance.setZero();

  // LinCov state vector
  // Resize LinCov state vector based on the size of the reference trajectory
  this->m_lincov_sim_vec_.resize(this->m_ref_trajectory_.rows(), Eigen::NoChange);
  this->m_lincov_sim_vec_.setConstant(std::numeric_limits<double>::quiet_NaN());

  // Set time
  this->m_lincov_sim_vec_(0, BMD::TIME_IND) = 0;

  // Set all measurement columns to zero
  this->m_lincov_sim_vec_.middleCols<BMD::NUM_MEAS_DIM>(BMD::NUM_MEAS_START_IND).setConstant(1);

  // Set reference trajectory
  this->m_lincov_sim_vec_.middleCols<BMD::REF_DIM>(BMD::REF_START_IND) = this->m_ref_trajectory_;

  // Set starting covariance matrix to zero
  Eigen::Map<Eigen::Matrix<double, BMD::ERROR_DIM, BMD::ERROR_DIM, Eigen::RowMajor>> lc_error_covariance(
  this->m_lincov_sim_vec_.block<1, BMD::ERROR_COV_LEN>(0, BMD::ERROR_COV_START_IND).data());
  lc_error_covariance.setZero();

  // Set starting augmented covariance matrix to zero
  Eigen::Map<Eigen::Matrix<double, LC::AUG_DIM, LC::AUG_DIM, Eigen::RowMajor>> init_aug_covariance(
    this->m_lincov_sim_vec_.template block<1, LC::AUG_COV_LEN>(0, LC::AUG_COV_START_IND).data());
  init_aug_covariance.setZero();

  // Update augmented covariance with inital values
  //init_aug_covariance.block<3, 3>(ERROR::GYRO_BIAS_START_IND, ERROR::GYRO_BIAS_START_IND) =
  //  (Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Identity().array() * std::pow(this->m_gyro_std_dev_, 2)).matrix();
  //init_aug_covariance.block<3, 3>(ERROR::ACCEL_BIAS_START_IND, ERROR::ACCEL_BIAS_START_IND) =
  //  (Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Identity().array() * std::pow(this->m_accel_std_dev_, 2)).matrix();
  //init_aug_covariance.block<BMD::ERROR_DIM, BMD::ERROR_DIM>(BMD::ERROR_DIM, BMD::ERROR_DIM) = lc_error_covariance;
}

/**
 * @brief Runs a LinCov analysis
 *
 * @details Runs a LinCov analysis. Proper measurement function
 * is assigned prior to analysis call. Flag is set after completion
 * to ensure future service servers respond appropriately to requests.
 */
void LinCovInterface::lincov_analysis(){
  this->m_data_mutex_.lock();

  // Run LinCov analysis with initialized tools and state vector
  kf::runLinCov<BasicModelDim, kf::Versions(kf::Versions::OPEN_LOOP bitor kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION), double, Eigen::RowMajor>(
    this->m_lincov_sim_vec_,
    this->m_tools_);

  this->m_did_lincov_ = true;
  this->m_data_mutex_.unlock();
}

/**
 * @brief Runs a Monte Carlo analysis
 *
 * @details Runs a Monte Carlo analysis. Proper measurement function
 * is assigned prior to analysis call. Flag is set after completion
 * to ensure future service servers respond appropriately to requests.
 */
void LinCovInterface::monte_carlo_analysis(){
  this->m_data_mutex_.lock();

  // Run the Monte Carlo simulation
  kf::runMonteCarloSims<BasicModelDim, kf::Versions(kf::Versions::OPEN_LOOP bitor kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION), double, Eigen::RowMajor>(
    this->m_ref_trajectory_.leftCols<BasicModelDim::REF_DIM>(),
    this->m_init_sim_vec_[BasicModelDim::TIME_IND],
    this->m_init_sim_vec_[BasicModelDim::TIME_IND],
    this->m_init_sim_vec_.middleCols<BasicModelDim::TRUTH_DIM>(BasicModelDim::MC::TRUTH_START_IND),
    this->m_init_sim_vec_.middleCols<BasicModelDim::NAV_DIM>(BasicModelDim::MC::NAV_START_IND),
    Eigen::Map<Eigen::Matrix<double, BasicModelDim::ERROR_DIM, BasicModelDim::ERROR_DIM, Eigen::RowMajor>>(
        this->m_init_sim_vec_.block<1, BasicModelDim::ERROR_COV_LEN>(0, BasicModelDim::ERROR_COV_START_IND).data()),
    this->m_num_monte_carlo_runs_,
    this->m_tools_,
    this->m_monte_carlo_output);

  // Set flag so future service requests are handled appropriately
  this->m_did_monte_carlo_ = true;
  this->m_data_mutex_.unlock();
}

/* === VISUALIZATION FUNCTIONS === */
/**
 * @brief Publishes uncertainty ellipses MarkerArray to "lincov_viz" topic
 */
void LinCovInterface::lincov_publish(){
  this->m_lincov_publisher_->publish(this->m_lincov_ellipses_);
}

/**
 * @brief Create MarkerArray for visualization in RVIZ at a specified spatial resolution
 *
 * @param spatial_resolution: A const double value determined by the user via the
 * LinCovAnalysis panel in rviz. Value specifies the spacing (m) between uncertainty
 * ellipses that are generated from the results of the LinCov analysis.
 *
 * @details Creates a MarkerArray with ellipse-shaped markers. Each ellipse shape is defined
 * by the covariance values at that particular point in time along the generated LinCov
 * trajectory. The major and minor axes are found by finding the smaller of the N or E
 * covariances. Eigenvectors defining the rotated ellipse are then found. The ellipses
 * are then scaled and rotated to properly represent the information in the
 * navigation state dispersion covariance matrix.
 */
void LinCovInterface::lincov_generate_markers(const double spatial_resolution){
  using Marker = visualization_msgs::msg::Marker;

  auto NORTH = BasicModelDim::REF::NORTH_IND;
  auto EAST = BasicModelDim::REF::EAST_IND;
  auto DOWN = BasicModelDim::REF::DOWN_IND;
  auto YAW = BasicModelDim::REF::YAW_IND;

  // Get covariances
  auto nav_covariance = kf::math::navStateDispersionCovariance<BasicModelDim, double, Eigen::RowMajor>(this->m_lincov_sim_vec_);
  size_t lincov_len = nav_covariance.size();
  size_t time_step = static_cast<size_t>(spatial_resolution / (this->m_nominal_velocity_*this->m_analysis_dt_));

  // Clear any existing markers
  lincov_delete_markers();

  // Get the timestamp to apply to all the markers
  auto timestamp = this->get_clock()->now();

  // Iterate over all time steps for which an uncertainty ellipse should be generated
  // Start at time_step, as starting at the beginning gives covariance of 0 and
  // results in a bad Marker scale
  int32_t marker_counter = 0;
  for (size_t time_it = time_step; time_it < lincov_len; time_it += time_step){
    // Create covariance matrix
    auto cov_matrix = Eigen::Matrix<double, 2, 2, Eigen::RowMajor>();
    cov_matrix(0, 0) = nav_covariance[time_it](NORTH, NORTH);
    cov_matrix(0, 1) = nav_covariance[time_it](NORTH, EAST);
    cov_matrix(1, 0) = nav_covariance[time_it](EAST, NORTH);
    cov_matrix(1, 1) = nav_covariance[time_it](EAST, EAST);

    // Can't add a marker with 0 covariance in x or y, so skip this iteration
    if (cov_matrix(0, 0) == 0 || cov_matrix(1, 1) == 0){
      continue;
    }

    // Do eigenvalue decomposition to find major and minor axes of uncertainty ellipse
    auto eigen_solver = Eigen::EigenSolver<decltype(cov_matrix)>();
    auto minor_cardinal = Eigen::Matrix<double, 2, 1>();
    double angle = 0;
    if (cov_matrix(0, 0) < cov_matrix(1, 1)){
      // N direction is minor axis since it is smaller
      // Make unit vector in minor axis direction
      minor_cardinal(0, 0) = 1.0;
      minor_cardinal(1, 0) = 0.0;
    } else {
      // Otherwise E direction is minor axis
      // Make unit vector in minor axis direction
      minor_cardinal(0, 0) = 0.0;
      minor_cardinal(1, 0) = 1.0;
    }

    // Get minimum eigenvalue index and corresponding eigenvectors
    eigen_solver.compute(cov_matrix, true);
    auto eigenvalues = eigen_solver.eigenvalues();
    auto eigenvectors = eigen_solver.eigenvectors();
    auto lambda1 = eigenvalues.row(0).norm();
    auto lambda2 = eigenvalues.row(1).norm();
    auto vec1 = eigenvectors.col(0);
    auto vec2 = eigenvectors.col(1);

    // Eigenvector pointing direction is reversed as they are technically
    // referenced in standard cartesian xyz instead of NED.
    // Get the angle between the minor-axis unit vector and the eigenvector
    if (lambda1 < lambda2){
      angle = std::acos(vec2.dot(minor_cardinal).real());
    } else {
      angle = std::acos(vec1.dot(minor_cardinal).real());
    }

    // Given variances of x/y nav state, get 3-sigma
    double x_3sigma = std::sqrt(cov_matrix(0, 0))*3.0;
    double y_3sigma = std::sqrt(cov_matrix(1, 1))*3.0;

    // Define the Marker for the uncertainty ellipse
    Marker ellipse;
    ellipse.header.frame_id = this->m_ellipse_frame_id_;
    ellipse.header.stamp = timestamp;
    ellipse.id = marker_counter;
    ellipse.ns = ellipse.header.frame_id + "/lincov_ellipses/" + std::to_string(marker_counter++);

    ellipse.type = Marker::CYLINDER;
    ellipse.action = Marker::ADD;
    ellipse.pose.position.x = this->m_ref_trajectory_(time_it, NORTH);
    ellipse.pose.position.y = this->m_ref_trajectory_(time_it, EAST);
    ellipse.pose.position.z = this->m_ref_trajectory_(time_it, DOWN);

    // Get the quaternion-based rotation values from RPY
    auto rpy = Eigen::Matrix<double, 1, 3, Eigen::RowMajor>();
    rpy.setZero();
    rpy(0, 2) = this->m_ref_trajectory_(time_it, YAW) + angle;
    tf2::Quaternion quat;
    quat.setRPY(rpy(0, 0), rpy(0, 1), rpy(0, 2));
    ellipse.pose.orientation = tf2::toMsg(quat);

    double test_scale = this->get_parameter("ellipses.test_scale").as_double();
    // Scale cylinders in x and y direction by respective 3-sigma values
    ellipse.scale.x = x_3sigma*test_scale;
    ellipse.scale.y = y_3sigma*test_scale;
    ellipse.scale.z = 1;

    // Set the color of the ellipse
    ellipse.color.r = this->m_rviz_ellipse_rgba_[0];
    ellipse.color.g = this->m_rviz_ellipse_rgba_[1];
    ellipse.color.b = this->m_rviz_ellipse_rgba_[2];
    ellipse.color.a = this->m_rviz_ellipse_rgba_[3];

    // Set ellipses to remain permanently until explicitly deleted
    ellipse.lifetime.sec = 0;
    ellipse.lifetime.nanosec = 0;

    // Any changes to the frame the ellipses are in will be reflected in
    // ellipse position if the frame changes
    ellipse.frame_locked = true;

    // Store result in member MarkerArray variable
    this->m_lincov_ellipses_.markers.emplace_back(ellipse);
  }
  this->lincov_publish();
}

/**
 * @brief Delete existing covariance markers and publish changes
 *
 * @details Any existing markers are cleared every time new ones are generated.
 * The Markers must be cleared by setting the Marker action to Marker::DELETE
 * and publishing, then clearing the MarkerArray vector so it no longer contains
 * the previous Marker data.
 */
void LinCovInterface::lincov_delete_markers(){
  using Marker = visualization_msgs::msg::Marker;

  // Delete all previous Markers, publish, then make new ones
  size_t num_ellipses = this->m_lincov_ellipses_.markers.size();
  for (size_t delete_it = 0; delete_it < num_ellipses; delete_it++){
    this->m_lincov_ellipses_.markers[delete_it].action = Marker::DELETE;
    lincov_publish();
  }

  // Set vector size back to 0
  this->m_lincov_ellipses_.markers.clear();
}

/* === CALLBACK FUNCTIONS === */
/**
 * @brief Performs LinCov or Monte Carlo analysis in response to a service request
 *
 * @param request:  A LinCovMonteCarloCall request. Request holds information about
 * type of analysis and parameters for that analysis.
 *
 * @param response: LinCovMonteCarloCall response that sends back a success flag
 * with a message (if necessary) to the rviz LinCov panel node.
 *
 * @details Depending on state of node, all parameters are initialized, tools
 * generated, reference trajectory created, and state vectors are initialized.
 * Once all this information is ready, the analysis requested by the user
 * is called.
 *
 * Due to the nature of the node, this callback is a blocking callback.
 * It is not anticipated that this node needs to do anything else during
 * the analysis call, as other features of the node are dependent on having
 * the results of the analysis to function properly.
 */
void LinCovInterface::analysis_callback(
  const std::shared_ptr<LinCovMonteCarloCall::Request> request,
  std::shared_ptr<LinCovMonteCarloCall::Response> response){
    this->log_diagnostics(DiagnosticStatus::OK, this->m_diag_msg_busy_);

    // Perform analysis only if waypoints were published
    if(this->m_have_waypoints_){
      if(!this->m_params_initialized_){
        // Perform LinCov initialization if this is the first analysis in node lifetime
        this->declare_trajectory_params();
        try{
          this->declare_feature_params();
          this->declare_viz_params();
        }
        catch(std::runtime_error &e){
          RCLCPP_ERROR(this->get_logger(), e.what());
          this->log_diagnostics(DiagnosticStatus::ERROR, "Please verify parameters are correct in configuration files");

          // If parameters aren't specified correctly, node won't function, so exit process
          exit(EXIT_FAILURE);
        }
        this->init_tools();
        this->m_params_initialized_ = true;
      }

      // Set number of MC runs to do
      auto prev_runs = this->m_num_monte_carlo_runs_;
      this->m_num_monte_carlo_runs_ = request->num_monte_carlo_runs;

      // Try making the reference trajectory. If angles defined by a set of 3 waypoints is
      // too small for the UAV to turn in, an error will be thrown.
      try{
        this->make_reference_trajectory();
      }
      catch(std::runtime_error &e){
        RCLCPP_ERROR(this->get_logger(), e.what());
        this->log_diagnostics(DiagnosticStatus::WARN, "Trajectory generation failed. Please clear waypoints and try again.");
        this->m_have_waypoints_ = false;
        response->success = false;
        response->message = "Trajectory generation failed";
        return;
      }

      // Run the LinCov analysis if the request was for LinCov only
      if (request->analysis_type == LinCovMonteCarloCall::Request::LINCOV_ONLY){
        // Only run the analysis if it hasn't already been done
        if (!this->m_did_lincov_){
          // Initialize the state vector
          this->make_state_vectors();
          this->lincov_analysis();
        }
      } else {
        // If a LinCov analysis hasn't been done yet, do it
        if (!this->m_did_lincov_){
          // Initialize the state vector
          this->make_state_vectors();
          this->lincov_analysis();
        }
        if (!this->m_did_monte_carlo_ || (prev_runs != this->m_num_monte_carlo_runs_)){
          this->monte_carlo_analysis();
        }
      }

      // Change flag to indicate plotting can now be done
      this->m_plot_ready_ = true;
      response->success = true;
      response->message = "Analysis Complete";
      this->log_diagnostics(DiagnosticStatus::OK, this->m_diag_msg_available_);
    } else {
      // Otherwise indicate that no new waypoints have been published since the last
      // run of a LinCov analysis
      response->success = false;
      response->message = "No waypoints were published";
      this->log_diagnostics(DiagnosticStatus::WARN,
                            "No waypoints were published. Place some waypoints and try again");
    }
}

/**
 * @brief Generate plots if conditions are met
 *
 * @param request: A Trigger request. Data associated with request is not used.
 * @param response: Trigger response that sends back a success bool with a message
 *                  (if necessary) to the rviz LinCov panel node.
 *
 * @details Upon receiving a request to plot, flags are checked to determine if
 * plotting can actually be performed. Callback then calls plotting function.
 */
void LinCovInterface::gen_plots_callback(
  const std::shared_ptr<SelectAnalysisPlot::Request> request,
  std::shared_ptr<SelectAnalysisPlot::Response> response){
    this->log_diagnostics(DiagnosticStatus::OK, this->m_diag_msg_busy_);
    // Store information about which plots to make
    this->m_save_plot_pdf_ = request->save_to_pdf;

    this->m_plot_data_types_  = std::move(request->data_types);
    this->m_plot_types_       = std::move(request->plot_types);
    this->m_mc_run_downsample_ = request->mc_run_downsample;

    // Set plotting types to be used
    // If an analysis has been performed, allow plotting
    if (this->m_plot_ready_) {
      // Generate the plots
      this->plot();
      response->message = "Waiting for CSVPlotter";
      response->success = true;
    } else {
      // There's nothing to plot, so return false to the service
      response->success = false;
      response->message = "No data for plotting";
    }
    this->log_diagnostics(DiagnosticStatus::OK, this->m_diag_msg_available_);
}

/**
 * @brief Simple callback for plotting via the csv_plotter node
 *
 * @param future Response from the csv_plotter node with info about request completion
 */
void LinCovInterface::csv_plot_callback(
  const rclcpp::Client<SelectAnalysisPlot>::SharedFuture future){
    if (!future.get()->success){
      this->log_diagnostics(DiagnosticStatus::ERROR, future.get()->message);
    } else {
      this->log_diagnostics(DiagnosticStatus::OK, "Plot operation successful");
    }
}

/**
 * @brief Generates uncertainty ellipses or deletes them
 *
 * @param request: ToggleEllipse request that contains spatial resolution
 * @param response: Trigger response that sends back a success bool with a message
 *                  (if necessary) to the rviz LinCov panel node.
 *
 * @details If the ellipses are on already, call the lincov_delete_markers() function
 * and update the m_ellipses_enabled_ flag to false. Otherwise, call the generate
 * markers function and update the m_ellipses_enabled_ flag to true.
 */
void LinCovInterface::toggle_ellipses_callback(
  const std::shared_ptr<uav_interfaces::srv::ToggleEllipse::Request> request,
  std::shared_ptr<uav_interfaces::srv::ToggleEllipse::Response> response){
    // If the ellipses are currently off
    if (!this->m_ellipses_enabled_){
      // Make new ellipses at requested spatial resolution
      this->lincov_generate_markers(request->requested_res);
      response->message = "Ellipses on";

      // Toggle ellipses flag
      this->m_ellipses_enabled_ = true;
    } else {
      // Delete (turn off) markers
      lincov_delete_markers();
      response->message = "Ellipses off";

      // Toggle ellipses flag
      this->m_ellipses_enabled_ = false;
    }

    // This callback should never fail
    response->success = true;
    response->message = "Ellipses toggled";
}

/**
 * @brief Saves a /waypoints message if one is published
 *
 * @param waypoints: A UavWaypoints message retrieved from the /waypoints publisher
 *
 * @details Store waypoints published from the rviz Waypoints panel for use in making
 * the reference trajectory prior to an analysis. Set the m_have_waypoints_ flag
 * to true.
 */
void LinCovInterface::waypoints_sub_callback(const uav_interfaces::msg::UavWaypoints &waypoints){
  // Assign publish waypoints to node variable
  this->m_data_mutex_.lock();

  if (waypoints.points.empty()) {
    this->log_diagnostics(DiagnosticStatus::WARN, "No waypoints were published!");
  } else if (waypoints.points.size() < 3) {
    this->log_diagnostics(DiagnosticStatus::WARN, "At least 3 waypoints need to be published!");
  } else {
    this->m_waypoints_ = waypoints;

    // Reset flags for plotting and analysis checking
    this->m_did_lincov_ = false;
    this->m_did_monte_carlo_ = false;

    // Change flag to true so analysis can be performed
    this->m_have_waypoints_ = true;
    this->log_diagnostics(DiagnosticStatus::OK, "Waypoints received");
  }
  this->m_data_mutex_.unlock();
}

/**
 * @brief Callback to publish current diagnostic information
 */
void LinCovInterface::publish_diagnostics(){
  this->m_diag_mutex_.lock();
  auto msg = diagnostic_msgs::msg::DiagnosticArray();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "lincov_interface";
  msg.status.emplace_back(this->m_diagnostic_status_);
  this->m_diag_mutex_.unlock();

  this->m_diagnostics_pub_->publish(msg);
}

/**
 * @brief Update the current diagnostic status of the node
 *
 * @param status: Status to make current status (WARN, OK, ERROR)
 * @param message: Message to display in diagnostic monitor
 */
void LinCovInterface::log_diagnostics(const diagnostic_msgs::msg::DiagnosticStatus::_level_type &level,
                      const std::string &message){
  this->m_diag_mutex_.lock();
  this->m_diagnostic_status_.level = level;
  this->m_diagnostic_status_.message = message;
  this->m_diag_mutex_.unlock();
  this->publish_diagnostics();
}
}  // end namespace planner_interface

int main(int argc, char **argv){
  rclcpp::init(argc, argv);

  // Make the node
  auto node = std::make_shared<planner_interface::LinCovInterface>();

  // Spin up node
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  rclcpp::Rate loop_rate(2);

  while(rclcpp::ok()){
    executor.spin_some();
    loop_rate.sleep();
  }

  rclcpp::shutdown();
}
