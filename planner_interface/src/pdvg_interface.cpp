/**
 * @File: pdvg_interface.cpp
 * @Date: January 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Interface implementation for making PDVG plans
 * in rviz simulation
 **/

// Local Headers
#include <planner_interface/csv_tools.hpp>
#include <planner_interface/make_statistics_csv.hpp>
#include <planner_interface/pdvg_interface.hpp>
#include <kalman_filter/noise/normal_distribution.hpp>
#include <kalman_filter/sensors/inertial_measurements/open_loop_imu.hpp>
#include <kalman_filter/noise/multi_noise.hpp>
#include <kalman_filter/controllers/open_loop_controller.hpp>
#include <kalman_filter/sensors/measurements/controllers/gps_heading_altitude_controller.hpp>
#include <kalman_filter/sensors/measurements/gps.hpp>
#include <kalman_filter/sensors/measurements/heading.hpp>
#include <kalman_filter/sensors/measurements/absolute_pressure.hpp>
#include <kalman_filter/mappings/simple_mapping.hpp>
#include <kalman_filter/helpers/plot_statistics.hpp>
#include <uav_interfaces/srv/select_analysis_plot.hpp>
#include <uav_interfaces/msg/uav_waypoint.hpp>
#include <uav_interfaces/msg/uav_waypoints.hpp>

// Local RRT Headers
#include <radar_detection/cross_sections/ellipsoid_cross_section_model.hpp>
#include <radar_detection/radar_model.hpp>
#include <rrt_search/edge_generators/fillets/imu/arc_imu_edge_generator.hpp>
#include <rrt_search/edge_generators/fillets/fillet_covariance_edge_generator.hpp>
#include <rrt_search/edge_generators/fillets/imu/euler_spiral_imu_edge_generator.hpp>
#include <rrt_search/helpers/connect_waypoints.hpp>
#include <rrt_search/obstacle_checkers/probability_detection_metric_obstacle_checker.hpp>
#include <rrt_search/search_functions/radar_visibility_graph.hpp>

// ROS2 Headers
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>

// Global Headers
#include <cmath>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <fstream>
#include <variant>
#include <vector>

// Boost Headers
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

// Eigen Headers
#include <Eigen/Dense>

using BasicModelDim = kf::dynamics::BasicModelDim;
using SelectAnalysisPlot = uav_interfaces::srv::SelectAnalysisPlot;
using CSVHeaders = planner_interface::csvtools::CSVHeaders;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using namespace std::placeholders;
namespace planner_interface{
const std::string PDVGInterface::m_diag_msg_available_ = "Available for service call";
const std::string PDVGInterface::m_diag_msg_busy_ = "Busy...";

/**
 * @brief Construct a new PDVGInterface object
 *
 * @details Create a new PDVGInterface object. Initialize flags
 * and create publishers, subscribers, and services
 */
PDVGInterface::PDVGInterface() : Node("pdvg_interface") {
  using namespace std::chrono_literals;
  // Initialize all flags. All but m_params_initialized are for interactions
  // with the LinCov Analysis panel
  this->m_have_endpoint_ = false;
  this->m_have_startpoint_ = false;
  this->m_plot_ready_ = false;
  this->m_params_initialized_ = false;
  this->m_self_publish_ = false;

  this->m_prev_start_point_(0) = std::numeric_limits<double>::infinity();
  this->m_prev_start_point_(1) = std::numeric_limits<double>::infinity();

  // Define callback groups and set options for subscribers
  this->m_planner_cb_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.callback_group = this->m_planner_cb_group_;

  // Create subscribers
  this->m_uav_waypoints_sub_ = this->create_subscription<uav_interfaces::msg::UavWaypoints>(
    "waypoints",
    1,
    std::bind(&PDVGInterface::waypoints_sub_callback, this, _1),
    sub_options);

  this->m_uav_state_sub_ = this->create_subscription<uav_interfaces::msg::UavState>(
    "uav_state",
    1,
    std::bind(&PDVGInterface::uav_state_callback, this, _1),
    sub_options);

  // Create publishers
  this->m_pdvg_path_publisher_ = this->create_publisher<uav_interfaces::msg::UavWaypoints>("waypoints", 1);

  // Create diagnostics publisher
  this->m_diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
  this->m_diag_timer_ =
    rclcpp::create_timer(this,
      this->get_clock(),
      1.0s,
      std::bind(&PDVGInterface::publish_diagnostics, this),
      this->m_planner_cb_group_);

  // Get the directory for storing plot data
  this->declare_parameter("plotting_data_directory", rclcpp::PARAMETER_STRING);
  this->m_plot_data_directory_ = this->get_parameter("plotting_data_directory").as_string();

  // Create services
  // Service used for requesting LinCov analysis
  this->m_button_server_ = this->create_service<std_srvs::srv::Trigger>(
    "run_pdvg",
    std::bind(&PDVGInterface::pdvg_callback, this, _1, _2),
    rmw_qos_profile_services_default, this->m_planner_cb_group_);

  // Service used for requesting to plot data after analysis
  this->m_plotting_server_ = this->create_service<SelectAnalysisPlot>(
    "pdvg_plot",
    std::bind(&PDVGInterface::gen_plots_callback, this, _1, _2),
    rmw_qos_profile_services_default, this->m_planner_cb_group_);

  // Service for making requests to CSV plotting node for plotting statistics
  this->m_csv_plot_client_ = this->create_client<uav_interfaces::srv::SelectAnalysisPlot>(
    "csv_plotter",
    rmw_qos_profile_services_default,
    this->m_planner_cb_group_
  );

  this->declare_viz_params();

  // Make first status update
  this->m_diagnostic_status_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  this->m_diagnostic_status_.name = "PDVG Interface";
  this->m_diagnostic_status_.hardware_id = "sim";
  this->m_diagnostic_status_.message = "Initialized";
}

/* === PARAMETER DECLARATION FUNCTIONS === */
/**
 * @brief Declare the world parameters for the PDVG analysis
 *
 * @details Declares PDVG world parameters, which includes map size,
 * nominal cross section, and starting number of sides for radar radius.
 */
void PDVGInterface::declare_pdvg_world_params(){
  // Declare world parameters
  this->declare_parameter("pdvg_world.world_bounds", rclcpp::PARAMETER_DOUBLE_ARRAY);
}

/**
 * @brief Declare and initialize trajectory-related parameters
 *
 * @details Gets and sets trajectory parameters
 */
void PDVGInterface::declare_trajectory_params(){
  // Declare trajectory-related parameters and time step
  this->declare_parameter("trajectory.nominal_velocity", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("trajectory.max_curvature", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("trajectory.max_curvature_rate", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("trajectory.nominal_pitch", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("trajectory.gravity", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("trajectory.fillet_dt", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("trajectory.line_dt", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("trajectory.use_arc_fillet", rclcpp::PARAMETER_BOOL);

  // Assign parameter values to node variables
  this->m_nominal_velocity_    = this->get_parameter("trajectory.nominal_velocity").as_double();
  this->m_max_curvature_       = this->get_parameter("trajectory.max_curvature").as_double();
  this->m_max_curvature_rate_  = this->get_parameter("trajectory.max_curvature_rate").as_double();
  this->m_nominal_pitch_       = this->get_parameter("trajectory.nominal_pitch").as_double();
  this->m_gravity_             = this->get_parameter("trajectory.gravity").as_double();
  this->m_use_arc_fillet_      = this->get_parameter("trajectory.use_arc_fillet").as_bool();

  // Nominal pitch is passed in with degrees as units, need to convert to radians
  this->m_nominal_pitch_       = rrt::math::angleToRadian<double>(this->m_nominal_pitch_);

}

/**
 * @brief Set up obstacle check parameters
 */
void PDVGInterface::declare_obstacle_checker_params(){
  // A 3-element vector describing the axis lengths of the ellipsoid detection model
  this->declare_parameter("obstacle_checker.ellipsoid_axes", rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Vectors of x, y, and z radar positions
  this->declare_parameter("obstacle_checker.radar_locations_x", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("obstacle_checker.radar_locations_y", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("obstacle_checker.radar_locations_z", rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Probability of false alarm for radars
  this->declare_parameter("obstacle_checker.prob_false_alarms", rclcpp::PARAMETER_DOUBLE);

  // Vector of radar constants. Vector length should match number of radars
  this->declare_parameter("obstacle_checker.radar_consts", rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Standard deviation for radar position and radar constant
  // In parameters, each element represents the x, y, AND z std_dev of a single radar
  this->declare_parameter("obstacle_checker.radar_pos_std_dev", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("obstacle_checker.radar_const_std_dev", rclcpp::PARAMETER_DOUBLE);

  // Probability of detection threshold for obstacle checker.
  this->declare_parameter("obstacle_checker.pd_threshold", rclcpp::PARAMETER_DOUBLE);

  // The multiple of the standard deviation that has to be below pd_threshold to succeed PDVG plan
  this->declare_parameter("obstacle_checker.std_dev_multiple", rclcpp::PARAMETER_DOUBLE);

  // Nominal cross section
  this->declare_parameter("obstacle_checker.nominal_cross_section", rclcpp::PARAMETER_DOUBLE);

  // Starting number of sides for obstacle checking in PDVG process
  this->declare_parameter("obstacle_checker.starting_num_sides", rclcpp::PARAMETER_INTEGER);
}

/**
 * @brief Declare and initialize noise-related parameters
 *
 * @details Gets and sets all bias noise terms for sensors and extracts
 * the time constants.
 */
void PDVGInterface::declare_noise_params(){
  // Declare sensor-bias-related parameters
  this->m_compass_bias_noise_ =
    std::make_shared<kf::noise::NoiseWrapper<1, double>>(
          kf::noise::makeFirstOrderGaussMarkovDrivingNoise<1, double>(this->shared_from_this(), "kalman_filter.dynamics.compass_bias"),
          CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::HEADING_BIAS]);
  this->m_abs_pressure_bias_noise_ =
    std::make_shared<kf::noise::NoiseWrapper<1, double>>(
          kf::noise::makeFirstOrderGaussMarkovDrivingNoise<1, double>(this->shared_from_this(), "kalman_filter.dynamics.abs_pressure_bias"),
          CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::ABS_PRESSURE]);
  this->m_feature_range_bias_noise_ =
    std::make_shared<kf::noise::NoiseWrapper<1, double>>(
          kf::noise::makeFirstOrderGaussMarkovDrivingNoise<1, double>(this->shared_from_this(), "kalman_filter.dynamics.feature_range_bias"),
          CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::FEATURE_RANGE]);
  this->m_feature_bearing_bias_noise_ =
    std::make_shared<kf::noise::NoiseWrapper<3, double>>(
          kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3, double>(this->shared_from_this(), "kalman_filter.dynamics.feature_bearing_bias"),
          CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::FEATURE_BEARING]);
  this->m_gps_position_bias_noise_ =
    std::make_shared<kf::noise::NoiseWrapper<3, double>>(
          kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3, double>(this->shared_from_this(), "kalman_filter.dynamics.gps_pos_bias"),
          CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::GPS_BIAS]);
  this->m_accel_bias_noise_ =
    std::make_shared<kf::noise::NoiseWrapper<3, double>>(
          kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3, double>(this->shared_from_this(), "kalman_filter.dynamics.accel_bias"),
          CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::ACCEL_BIAS]);
  this->m_gyro_bias_noise_ =
    std::make_shared<kf::noise::NoiseWrapper<3, double>>(
          kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3, double>(this->shared_from_this(), "kalman_filter.dynamics.gyro_bias"),
          CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::GYRO_BIAS]);

  // Assign parameter values to node variables
  this->m_compass_time_const_          = this->get_parameter("kalman_filter.dynamics.compass_bias.time_constant").as_double_array()[0];
  this->m_abs_pressure_time_const_     = this->get_parameter("kalman_filter.dynamics.abs_pressure_bias.time_constant").as_double_array()[0];
  this->m_accel_time_const_            = this->get_parameter("kalman_filter.dynamics.accel_bias.time_constant").as_double_array()[0];
  this->m_gyro_time_const_             = this->get_parameter("kalman_filter.dynamics.gyro_bias.time_constant").as_double_array()[0];
  this->m_gps_position_time_const_     = this->get_parameter("kalman_filter.dynamics.gps_pos_bias.time_constant").as_double_array()[0];
  this->m_feature_range_time_const_    = this->get_parameter("kalman_filter.dynamics.feature_range_bias.time_constant").as_double_array()[0];
  this->m_feature_bearing_time_const_  = this->get_parameter("kalman_filter.dynamics.feature_bearing_bias.time_constant").as_double_array()[0];
}

/**
 * @brief Declare visualization parameters
 */
void PDVGInterface::declare_viz_params(){
  this->declare_parameter("visualization.frame_id", rclcpp::PARAMETER_STRING);
}

/* === TOOL CREATION FUNCTIONS === */
/**
 * @brief Call all tool initialization functions
 */
void PDVGInterface::init_tools(){
  // Make these tools before edge-gen tool can be made
  this->init_inertial_measure_tools();
  this->init_dynamics_tool();
  this->init_controller_tool();
  this->init_measurement_controller_tool();
  this->init_mappings_tool();

  // Make edge gen and obstacle checker tools
  this->init_edge_generator();
  this->init_obstacle_checker();
}

/**
 * @brief Create inertial measurements function for tools
 */
void PDVGInterface::init_inertial_measure_tools(){
  // Make IMU measurements function and get std_dev for making initial state vector
  this->m_accel_meas_noise_ =
    std::make_shared<kf::noise::NoiseWrapper<3, double>>(
      kf::noise::makeNormalDistribution<3, double>(this->shared_from_this(), "imu.accelerometer.noise"),
      CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::ACCEL]);
  this->m_gyro_meas_noise_ =
    std::make_shared<kf::noise::NoiseWrapper<3, double>>(
      kf::noise::makeNormalDistribution<3, double>(this->shared_from_this(), "imu.gyroscope.noise"),
      CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::GYRO]);

  this->m_edge_gen_tools_.inertial_measurements =
    std::make_shared<kf::sensors::OpenLoopIMU<BasicModelDim, double, Eigen::RowMajor>>();

  this->m_accel_std_dev_ = this->get_parameter("imu.accelerometer.noise.standard_deviation").as_double_array()[0];
  this->m_gyro_std_dev_ = this->get_parameter("imu.gyroscope.noise.standard_deviation").as_double_array()[0];

  this->m_edge_gen_tools_.inertial_measurements_noise =
    std::make_shared<kf::noise::MultiNoise<kf::dynamics::BasicModelDim::INER_MEAS_NOISE_DIM>>(
      std::vector<std::pair<kf::noise::NoiseBasePtr<1>,Eigen::Index>>(),
      std::vector<std::pair<kf::noise::NoiseBasePtr<2>,Eigen::Index>>(),
      std::vector<std::pair<kf::noise::NoiseBasePtr<3>,Eigen::Index>>({
        std::make_pair(this->m_gyro_meas_noise_, kf::dynamics::BasicModelDim::INER_MEAS::GYRO_START_IND),
        std::make_pair(this->m_accel_meas_noise_, kf::dynamics::BasicModelDim::INER_MEAS::ACCEL_START_IND),
        }));
}

/**
 * @brief Create dynamics for tools
 *
 * @details Uses all sensor bias noise vectors and time constants to create a
 * function that defines the navigation and truth state dynamics
 */
void PDVGInterface::init_dynamics_tool(){
  // Create the dynamics function using sensor and trajectory parameters
  this->m_edge_gen_tools_.dynamics =
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

  // Create truth process noise function
  this->m_edge_gen_tools_.truth_process_noise =
    std::make_shared<kf::noise::MultiNoise<kf::dynamics::BasicModelDim::TRUTH_NOISE_DIM>>(
      std::vector<std::pair<kf::noise::NoiseBasePtr<1>,Eigen::Index>>({
        std::make_pair(this->m_compass_bias_noise_, kf::dynamics::BasicModelDim::TRUTH_NOISE::HEADING_BIAS_IND),
        std::make_pair(this->m_abs_pressure_bias_noise_, kf::dynamics::BasicModelDim::TRUTH_NOISE::ABS_PRESSURE_BIAS_IND),
        std::make_pair(this->m_feature_range_bias_noise_, kf::dynamics::BasicModelDim::TRUTH_NOISE::FEATURE_RANGE_BIAS_IND),
        }),
      std::vector<std::pair<kf::noise::NoiseBasePtr<2>,Eigen::Index>>(),
      std::vector<std::pair<kf::noise::NoiseBasePtr<3>,Eigen::Index>>({
        std::make_pair(this->m_feature_bearing_bias_noise_, kf::dynamics::BasicModelDim::TRUTH_NOISE::FEATURE_BEARING_BIAS_START_IND),
        std::make_pair(this->m_gps_position_bias_noise_, kf::dynamics::BasicModelDim::TRUTH_NOISE::GPS_POS_BIAS_START_IND),
        std::make_pair(this->m_gyro_bias_noise_, kf::dynamics::BasicModelDim::TRUTH_NOISE::GYRO_BIAS_START_IND),
        std::make_pair(this->m_accel_bias_noise_, kf::dynamics::BasicModelDim::TRUTH_NOISE::ACCEL_BIAS_START_IND),
        }));
}

/**
 * @brief Create open-loop controller for tools
 */
void PDVGInterface::init_controller_tool(){
  // Create the controller tool
  this->m_edge_gen_tools_.controller =
    std::make_shared<kf::control::OpenLoopController<BasicModelDim, double, Eigen::RowMajor>>();
}

/**
 * @brief Create measurements controller for tools
 *
 * @details Utilizes sensor parameters (i.e. if sensor is enabled, sensor's
 * measurement period, sensor noise parameters, and other sensor-specific
 * parameters) to provide tool for updating error covariance and augmented
 * covariance matrices.
 */
void PDVGInterface::init_measurement_controller_tool(){
  using Controller = kf::sensors::GPSHeadingAltitudeController<BasicModelDim, false, double, Eigen::RowMajor>;

  // Set up variables for use in plotting error budget
  this->declare_parameter("sensors.gps.measurement_period", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("sensors.compass.measurement_period", rclcpp::PARAMETER_DOUBLE);

  auto gps_noise_meas_period = this->get_parameter("sensors.gps.measurement_period").as_double();
  auto compass_noise_meas_period = this->get_parameter("sensors.compass.measurement_period").as_double();

  // Create noise objects
  this->m_gps_meas_noise_ =
    std::make_shared<kf::noise::NoiseWrapper<3, double>>(
      kf::noise::makeNormalDistribution<3, double>(this->shared_from_this(), "sensors.gps.noise"),
      CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::GPS]);

  this->m_compass_meas_noise_ =
    std::make_shared<kf::noise::NoiseWrapper<1, double>>(
      kf::noise::makeNormalDistribution<1, double>(this->shared_from_this(), "sensors.compass.noise"),
      CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::HEADING]);

  this->m_abs_pressure_meas_noise_ =
    std::make_shared<kf::noise::NoiseWrapper<1, double>>(
      kf::noise::makeNormalDistribution<1, double>(this->shared_from_this(), "sensors.abs_pressure.noise"),
      CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::ABS_PRESSURE]);

  // Create the measurement controller
  this->m_edge_gen_tools_.measurement_controller =
    std::make_shared<Controller>(
      std::make_shared<kf::sensors::GPS<BasicModelDim, true, true, double>>(gps_noise_meas_period),
      this->m_gps_meas_noise_,
      std::make_shared<kf::sensors::Heading<BasicModelDim, true, true, double>>(compass_noise_meas_period),
      this->m_compass_meas_noise_,
      kf::sensors::makeAbsolutePressure<BasicModelDim, true, double>(this->shared_from_this(), "sensors.abs_pressure"),
      this->m_abs_pressure_meas_noise_
    );
}

/**
 * @brief Create mappings tool
 *
 * @details Create mappings tool that is used to extract parts of the reference state
 * vector and maps them to either a truth state or navigation state vector
 */
void PDVGInterface::init_mappings_tool(){
  // Create the mappings tool
  this->m_edge_gen_tools_.mappings =
    std::make_shared<kf::map::SimpleMapping<BasicModelDim, double, Eigen::RowMajor>>();
}

/**
 * @brief Make an edge generator
 *
 * @details Makes line and fillet tools for use in the PDVG edge generator tool
 */
void PDVGInterface::init_edge_generator(){
  // Get time steps for fillets and lines
  auto fillet_dt = this->get_parameter("trajectory.fillet_dt").as_double();
  auto line_dt = this->get_parameter("trajectory.line_dt").as_double();

  // Make fillet edge generator
  rrt::edge::FilletEdgeGeneratorPtr<BasicModelDim::REF_DIM, double> fillet_edge_gen;
  rrt::edge::FilletEdgeGeneratorPtr<BasicModelDim::REF_DIM, double> line_edge_gen;
  if (this->m_use_arc_fillet_){
    fillet_edge_gen =
      std::make_shared<rrt::edge::ArcIMUEdgeGenerator<double>>(
        this->m_nominal_velocity_ * fillet_dt,
        double(1)/this->m_max_curvature_,
        this->m_nominal_velocity_,
        this->m_nominal_pitch_,
        this->m_nominal_down_,
        this->m_gravity_);

    // Make line edge generator
    line_edge_gen =
      std::make_shared<rrt::edge::ArcIMUEdgeGenerator<double>>(
        this->m_nominal_velocity_ * line_dt,
        double(1)/this->m_max_curvature_,
        this->m_nominal_velocity_,
        this->m_nominal_pitch_,
        this->m_nominal_down_,
        this->m_gravity_);
  } else {
    fillet_edge_gen =
      std::make_shared<rrt::edge::EulerSpiralIMUEdgeGenerator<double>>(
        this->m_nominal_velocity_ * fillet_dt,
        this->m_max_curvature_,
        this->m_max_curvature_rate_,
        this->m_nominal_velocity_,
        this->m_nominal_pitch_,
        this->m_nominal_down_,
        this->m_gravity_);

    // Make line edge generator
    line_edge_gen =
      std::make_shared<rrt::edge::EulerSpiralIMUEdgeGenerator<double>>(
        this->m_nominal_velocity_ * line_dt,
        this->m_max_curvature_,
        this->m_max_curvature_rate_,
        this->m_nominal_velocity_,
        this->m_nominal_pitch_,
        this->m_nominal_down_,
        this->m_gravity_);
  }

  // Create edge generator
  this->m_pdvg_tools_.edge_generator =
    std::make_shared<rrt::edge::FilletCovarianceEdgeGenerator<BasicModelDim,
                                                              kf::Versions(kf::Versions::OPEN_LOOP bitor kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),
                                                              kf::Versions(kf::Versions::OPEN_LOOP bitor kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),
                                                              double>>(
      std::min<double>(line_dt, fillet_dt), line_edge_gen, fillet_edge_gen, this->m_edge_gen_tools_, this->m_edge_gen_tools_);
}

/**
 * @brief Set up obstacle checker for the PDVG planner
 */
void PDVGInterface::init_obstacle_checker(){
  using EllipsoidCSM = rd::EllipsoidCrossSectionModel<double, Eigen::Dynamic>;
  using RadarModelPtr = rd::RadarModelPtr<double, Eigen::Dynamic>;
  using RadarModel = rd::RadarModel<double, Eigen::Dynamic>;
  using EigenScalar = Eigen::Matrix<double, 1, 1, Eigen::RowMajor>;

  // Get all relevant parameters
  auto ellipsoid_axes       = this->get_parameter("obstacle_checker.ellipsoid_axes").as_double_array();
  auto prob_false_alarms    = this->get_parameter("obstacle_checker.prob_false_alarms").as_double();
  auto radar_consts         = this->get_parameter("obstacle_checker.radar_consts").as_double_array();
  auto radar_x_pos          = this->get_parameter("obstacle_checker.radar_locations_x").as_double_array();
  auto radar_y_pos          = this->get_parameter("obstacle_checker.radar_locations_y").as_double_array();
  auto radar_z_pos          = this->get_parameter("obstacle_checker.radar_locations_z").as_double_array();
  auto radar_pos_std_dev    = this->get_parameter("obstacle_checker.radar_pos_std_dev").as_double();
  auto radar_const_std_dev  = this->get_parameter("obstacle_checker.radar_const_std_dev").as_double();
  auto pd_threshold         = this->get_parameter("obstacle_checker.pd_threshold").as_double();
  auto std_dev_mult         = this->get_parameter("obstacle_checker.std_dev_multiple").as_double();

  // Verify sizes are all correct
  {
    auto axes_size = ellipsoid_axes.size();
    auto radar_consts_size = radar_consts.size();
    auto x_pos_size = radar_x_pos.size();
    auto y_pos_size = radar_y_pos.size();
    auto z_pos_size = radar_z_pos.size();

    std::string x_size_str = std::to_string(x_pos_size);
    try{
      if (axes_size != 3){
        throw std::length_error(
          "Expected ellipsoid_axes vector of size 3. Got size " + std::to_string(axes_size));
      } else if (y_pos_size != x_pos_size){
        std::string y_size_str = std::to_string(y_pos_size);
        throw std::length_error(
          "Given radar_locations_x vector of size " + x_size_str + ", expected 'radar_locations_y' size to be the same but got size of " + y_size_str);
      } else if (z_pos_size != x_pos_size){
        std::string z_size_str = std::to_string(z_pos_size);
        throw std::length_error(
          "Given radar_locations_x vector of size " + x_size_str + ", expected 'radar_locations_z' size to be the same but got size of " + z_size_str);
      } else if (radar_consts_size != x_pos_size){
        std::string consts_size_str = std::to_string(radar_consts_size);
        throw std::length_error(
          "Given radar_locations_x vector of size " + x_size_str + ", expected 'radar_consts' size to be the same but got size of " + consts_size_str);
      }
    }
    catch (const std::exception& e){
      RCLCPP_ERROR(this->get_logger(), e.what());
      this->log_diagnostics(DiagnosticStatus::ERROR, "Please verify parameters are correct in configuration files");

      // If parameters aren't specified correctly, node won't function, so exit process
      exit(EXIT_FAILURE);
    }
  }

  // Get number of radars defined from parameters
  size_t num_radars = radar_x_pos.size();

  // Make all components of radars that are constant
  rd::CrossSectionModelPtr<double, Eigen::Dynamic> cross_section_model =
    std::make_shared<EllipsoidCSM>(ellipsoid_axes[0],
                                   ellipsoid_axes[1],
                                   ellipsoid_axes[2]);

  // Make covariance matrix diagonal values
  const Eigen::Matrix<double, 1, 3, Eigen::RowMajor> radar_pos_cov_vec =
    {radar_pos_std_dev,
     radar_pos_std_dev,
     radar_pos_std_dev};

  // Put radar standard deviations in diagonal of a matrix
  const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> radar_pos_cov =
    radar_pos_cov_vec.array().sqrt().matrix().asDiagonal();

  const EigenScalar radar_const_cov = EigenScalar::Constant(std::sqrt(radar_const_std_dev));

  // Make noise vector for radar positions
  this->m_radar_pos_noise_ =
    std::make_shared<kf::noise::NoiseWrapper<3, double>>(
      std::make_shared<kf::noise::NormalDistribution<3, false, true, false, double>>(
        Eigen::Matrix<double, 1, 3, Eigen::RowMajor>::Zero(),
        radar_pos_cov),
        CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::RADAR_POS]);

  // Make noise vector for radar constant
  this->m_radar_const_noise_ =
    std::make_shared<kf::noise::NoiseWrapper<1, double>>(
      std::make_shared<kf::noise::NormalDistribution<1, false, true, false, double>>(
        EigenScalar::Zero(),
        radar_const_cov),
      CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::RADAR_CONST]);

  // Make obstacle checker object
  this->m_obs_checker_ =
    std::make_shared<rrt::obs::ProbabilityDetectionMetricObstacleChecker<kf::dynamics::BasicModelDim, false, false, double>>(cross_section_model,
                                                                                                                             pd_threshold,
                                                                                                                             std_dev_mult);
  // Create and add all radars to obstacle checker
  for (size_t r = 0; r < num_radars; r++){
    // Create radar model
    RadarModelPtr radar_model  = std::make_shared<RadarModel>(prob_false_alarms, radar_consts[r]);

    // Make Eigen Vector of radar pose
    Eigen::Matrix<double, 1, 3, Eigen::RowMajor> radar_pose =
      {radar_x_pos[r], radar_y_pos[r], radar_z_pos[r]};

    // Add radar to obstacle checker
    this->m_obs_checker_->addRadar(radar_model,
                                   radar_pose,
                                   this->m_radar_pos_noise_,
                                   this->m_radar_const_noise_);
  }

  // Set obstacle checker in tools to the one that just finished creation
  this->m_pdvg_tools_.obstacle_checker = this->m_obs_checker_;
}

/* === PLAN GENERATION FUNCTIONS === */
/**
 * @brief Set up initial conditions for starting a PDVG planner
 *
 * @details Sets up all the required values for the starting point
 * for the PDVG planner operations. This includes the time, starting pose,
 * and covariance matrices.
 */
void PDVGInterface::set_initial_conditions(){
  // Define aliases to increase readability
  using BMD   = BasicModelDim;
  using ERROR = BMD::ERROR;
  using REF   = BMD::REF;
  using LC    = BMD::LINCOV;

  // Initialize vector
  this->m_starting_point_.setConstant(std::numeric_limits<double>::quiet_NaN());

  // Set initial time
  this->m_starting_point_[0] = 0;

  // Prepare starting pose with UavState message
  this->m_starting_point_.middleCols<BMD::NUM_MEAS_DIM>(BMD::NUM_MEAS_START_IND).setZero();
  auto x_pos = this->m_uav_state_.pose.pose.position.x;
  auto y_pos = this->m_uav_state_.pose.pose.position.y;
  auto uav_state_down = this->m_uav_state_.pose.pose.position.z;
  this->m_start_point_ = {x_pos, y_pos};

  // Extract yaw and pitch
  tf2::Quaternion q(
    this->m_uav_state_.pose.pose.orientation.w,
    this->m_uav_state_.pose.pose.orientation.x,
    this->m_uav_state_.pose.pose.orientation.y,
    this->m_uav_state_.pose.pose.orientation.z
  );

  double roll, uav_state_pitch, uav_state_yaw;
  tf2::Matrix3x3 rot_mat(q);
  rot_mat.getRPY(roll, uav_state_pitch, uav_state_yaw);  // Ignore roll

  // Set starting pose, current pitch and yaw
  this->m_starting_point_.middleCols<BMD::REF_DIM>(BMD::REF_START_IND).setZero();
  this->m_starting_point_.middleCols<2>(BMD::REF_START_IND + REF::POS_START_IND) = this->m_start_point_;
  this->m_starting_point_[BMD::REF_START_IND + REF::DOWN_IND]  = uav_state_down;
  this->m_starting_point_[BMD::REF_START_IND + REF::PITCH_IND] = uav_state_pitch;
  this->m_starting_point_[BMD::REF_START_IND + REF::YAW_IND]   = uav_state_yaw;

  // Set starting error covariance and starting augmented covariance
  Eigen::Map<Eigen::Matrix<double, BMD::ERROR_DIM, BMD::ERROR_DIM, Eigen::RowMajor>> error_covariance(
    this->m_starting_point_.block<1, BMD::ERROR_COV_LEN>(0, BMD::ERROR_COV_START_IND).data());
  error_covariance.setZero();

  Eigen::Map<Eigen::Matrix<double, LC::AUG_DIM, LC::AUG_DIM, Eigen::RowMajor>> init_aug_covariance(
    this->m_starting_point_.block<1, LC::AUG_COV_LEN>(0, LC::AUG_COV_START_IND).data());
  init_aug_covariance.setZero();

  init_aug_covariance.block<3, 3>(ERROR::GYRO_BIAS_START_IND, ERROR::GYRO_BIAS_START_IND) =
    (Eigen::Matrix<double,
                   3,
                   3,
                   Eigen::RowMajor>::Identity().array() * std::pow(this->m_gyro_std_dev_, 2)).matrix();

  init_aug_covariance.block<3, 3>(ERROR::ACCEL_BIAS_START_IND, ERROR::ACCEL_BIAS_START_IND) =
    (Eigen::Matrix<double,
                   3,
                   3,
                   Eigen::RowMajor>::Identity().array() * std::pow(this->m_accel_std_dev_, 2)).matrix();
}

/**
 * @brief Run the PDVG planner
 */
void PDVGInterface::run_pdvg(){
  using DIM = kf::dynamics::BasicModelDim;
  // Get world parameters
  std::vector<double> world_bounds = this->get_parameter("pdvg_world.world_bounds").as_double_array();
  size_t starting_sides            = this->get_parameter("obstacle_checker.starting_num_sides").as_int();
  double nominal_cross_section     = this->get_parameter("obstacle_checker.nominal_cross_section").as_double();

  // Run PDVG
  this->m_pdvg_solution_ =
    rrt::search::radarVisGraphSearch<DIM, false, false, double, Eigen::RowMajor>(
      this->m_obs_checker_,
      this->m_pdvg_tools_.edge_generator,
      this->m_starting_point_,
      this->m_target_waypoint_,
      Eigen::Matrix<double, 2, 2, Eigen::RowMajor>(world_bounds.data()),
      nominal_cross_section,
      starting_sides);

  // Publish markers to show solution path in RVIZ
  this->publish_solution_path();
}

/* === VISUALIZATION FUNCTIONS === */
/**
 * @brief Call function to generate CSV file for the probability of detection plot
 */
void PDVGInterface::make_pd_plot(){

  planner_interface::csv::makePDPlotCSV<BasicModelDim, double, Eigen::RowMajor>(
    this->waypoints_to_trajectory<BasicModelDim, double, Eigen::RowMajor>(),
    this->m_obs_checker_,
    this->m_plot_data_directory_);
}

/**
 * @brief Make call to functions that create CSV file for error budget information
 */
void PDVGInterface::plot_error_budget(){
  // return;
  auto std_dev_multiple = this->get_parameter("obstacle_checker.std_dev_multiple").as_double();

  // Add all noise sources to object
  std::list<kf::plot::GeneralNoiseWrapper<double, Eigen::RowMajor>> noise_sources;
  noise_sources.emplace_back(this->m_compass_bias_noise_);
  noise_sources.emplace_back(this->m_compass_meas_noise_);
  noise_sources.emplace_back(this->m_abs_pressure_bias_noise_);
  noise_sources.emplace_back(this->m_feature_range_bias_noise_);
  noise_sources.emplace_back(this->m_feature_bearing_bias_noise_);
  noise_sources.emplace_back(this->m_gps_position_bias_noise_);
  noise_sources.emplace_back(this->m_gps_meas_noise_);
  noise_sources.emplace_back(this->m_gyro_bias_noise_);
  noise_sources.emplace_back(this->m_gyro_meas_noise_);
  noise_sources.emplace_back(this->m_accel_bias_noise_);
  noise_sources.emplace_back(this->m_accel_meas_noise_);
  noise_sources.emplace_back(this->m_radar_pos_noise_);
  noise_sources.emplace_back(this->m_radar_const_noise_);

  auto obs_checker = this->m_obs_checker_;

  // Create lambda function for returning x-sigma std deviation of probability of detection
  auto data_extraction_func =
  [&obs_checker, &std_dev_multiple] (const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, BasicModelDim::LINCOV::FULL_STATE_LEN, Eigen::RowMajor>>& state_vector) -> Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor>
  {
    Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor> probability_of_detection;
    Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor> probability_of_detection_std;
    Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor> radar_cross_section;
    Eigen::Matrix<double, 1, Eigen::Dynamic, Eigen::RowMajor> range;

    obs_checker->getPlotInfo(state_vector, probability_of_detection, probability_of_detection_std, radar_cross_section, range);

    return probability_of_detection_std.array() * std_dev_multiple;
  };

  // Save data to file
  planner_interface::csv::makePDVGErrorBudgetCSVKickoff<BasicModelDim, kf::Versions(kf::Versions::OPEN_LOOP bitor kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION), double, Eigen::RowMajor>(
    this->waypoints_to_trajectory<BasicModelDim,
                                       double,
                                       Eigen::RowMajor>().middleCols<BasicModelDim::REF_DIM>(BasicModelDim::REF_START_IND),
    this->m_starting_point_,
    this->m_edge_gen_tools_,
    noise_sources,
    data_extraction_func,
    this->m_plot_data_directory_);
}

/**
 * @brief Publish a UavWaypoints object for the solution path
 */
void PDVGInterface::publish_solution_path(){
  this->m_pdvg_path_markers_.id = boost::uuids::to_string(this->m_uuid_gen_());
  this->m_pdvg_path_markers_.type = "fillet";
  this->m_pdvg_path_markers_.min_radius = 1/this->m_max_curvature_;

  // Get the frame to publish markers in
  auto frame_id = this->get_parameter("visualization.frame_id").as_string();

  this->m_pdvg_path_markers_.points.clear();

  // Assign publishing information
  auto timestamp = this->get_clock()->now();
  uav_interfaces::msg::UavWaypoint waypoint;

  Eigen::Index north_idx = 0;
  Eigen::Index east_idx = 1;
  for (auto wp_it = this->m_pdvg_solution_.begin(); wp_it != this->m_pdvg_solution_.end(); ++wp_it){
    // Put all the waypoint into a UavWaypoint
    waypoint.position = geometry_msgs::msg::PointStamped();
    waypoint.position.point.x = (*wp_it)(north_idx);
    waypoint.position.point.y = (*wp_it)(east_idx);
    waypoint.position.point.z = this->m_nominal_down_;

    waypoint.airspeed = this->m_nominal_velocity_;

    // Header information
    waypoint.position.header.frame_id = frame_id;
    waypoint.position.header.stamp = timestamp;

    // Store to overall path
    this->m_pdvg_path_markers_.points.emplace_back(waypoint);
  }

  // Publish all markers
  this->m_self_publish_ = true;
  this->m_pdvg_path_publisher_->publish(this->m_pdvg_path_markers_);
}

/**
 * @brief Check to see if the starting and ending points are the same as before
 *
 * @return true if same start and end are being used
 * @return false otherwise
 */
bool PDVGInterface::is_same_init_conditions(){
  double x_prev_start = std::floor(this->m_prev_start_point_(0));
  double y_prev_start = std::floor(this->m_prev_start_point_(1));
  double x_curr_start = std::floor(this->m_start_point_(0));
  double y_curr_start = std::floor(this->m_start_point_(1));

  double x_prev_end = std::floor(this->m_prev_target_(0));
  double y_prev_end = std::floor(this->m_prev_target_(1));
  double x_curr_end = std::floor(this->m_target_waypoint_(0));
  double y_curr_end = std::floor(this->m_target_waypoint_(1));

  bool start_same = (x_prev_start == x_curr_start) && (y_prev_start == y_curr_start);
  bool end_same = (x_prev_end == x_curr_end) && (y_prev_end == y_curr_end);

  return (start_same && end_same);
}

/**
 * @brief Convert received PDVG waypoints to trajectory for plotting purposes
 *
 * @tparam DIM_S: BasicModelDim used to access all parts of state vectors
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @return Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>
 */
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>
PDVGInterface::waypoints_to_trajectory(){
  // Convert current waypoints to full state length type
  std::list<Eigen::Matrix<SCALAR, 1, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>> waypoints;

  // Set initial point
  waypoints.emplace_front(this->m_starting_point_);
  const auto waypoints_min_end = this->m_pdvg_solution_.cend();

  // Copy values of the NED position of waypoints
  for(auto waypoint_it = std::next(this->m_pdvg_solution_.cbegin()); waypoint_it != waypoints_min_end; ++waypoint_it){
    waypoints.emplace_back();
    waypoints.back().template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) = *waypoint_it;
  }

  const auto waypoints_end = waypoints.end();

  // Set orientation of all waypoints
  for(auto waypoint_it = std::next(waypoints.begin()); waypoint_it != waypoints_end; ++waypoint_it){
    (*waypoint_it) = this->m_pdvg_tools_.edge_generator->setOrientation(*waypoint_it, *std::prev(waypoint_it));
  }

  // Set initial position to that of second waypoint
  waypoints.front().template middleCols<3>(DIM_S::REF_START_IND + DIM_S::REF::EULER_START_IND) =
    (*std::next(waypoints.cbegin())).template middleCols<3>(DIM_S::REF_START_IND + DIM_S::REF::EULER_START_IND);

  // Declare variable to return
  Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS> trajectory =
    rrt::connectWaypointsFillets<DIM_S::LINCOV::FULL_STATE_LEN, SCALAR, OPTIONS>(
      waypoints, this->m_pdvg_tools_.edge_generator);

  return trajectory;
}

/* === CALLBACK FUNCTIONS === */
/**
 * @brief Set up and execute pdvg planner
 *
 * @details Declare all parameters, initialize tools, and run the
 * pdvg planner. After finishing, send a response with the appropriate
 * values
 *
 * @param request: A trigger request. Nothing comes in request so it is ignored
 * @param response: Trigger response with success and message members
 */
void PDVGInterface::pdvg_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response){
  this->m_data_mutex_.lock();
  this->log_diagnostics(DiagnosticStatus::OK, this->m_diag_msg_busy_);
  // If the start and endpoints have been sent
  if (this->m_have_startpoint_ && this->m_have_endpoint_){
    // Declare all parameters if a request hasn't been made before
    if (!this->m_params_initialized_){
      try {
        this->declare_noise_params();
        this->declare_obstacle_checker_params();
        this->declare_trajectory_params();
        this->declare_pdvg_world_params();
      } catch (std::runtime_error &e) {
        RCLCPP_ERROR(this->get_logger(), e.what());
        this->log_diagnostics(DiagnosticStatus::ERROR, "Please verify parameters are correct in configuration files");

        // If parameters aren't specified correctly, node won't function, so exit process
        exit(EXIT_FAILURE);
      }
      // Initialize all tools for edge generator and obstacle checker
      this->init_tools();
      this->m_params_initialized_ = true;
    }

    // Set up initial conditions for PDVG planner and run it
    this->set_initial_conditions();

    // If previous conditions match current:
    if (!this->is_same_init_conditions()){
      try {
        this->run_pdvg();

        // Set response info to indicate successful completion
        response->message = "Analysis Complete";
        response->success = true;

        // Store data so CSVPlotter can use later
        this->make_pd_plot();
        this->plot_error_budget();
      } catch (std::runtime_error &e) {
        RCLCPP_ERROR(this->get_logger(), e.what());
        response->message = "Plan failed";
        response->success = false;
      }
    } else {
      this->publish_solution_path();
      response->message = "Reused Previous Solution";
      response->success = true;
    }

    // Update the "previous" points
    this->m_prev_start_point_ = this->m_start_point_;
    this->m_prev_target_      = this->m_target_waypoint_;

  } else {
    // UAV waypoint hasn't been found, or end point hasn't been published
    response->message = "Missing start point or end point";
    response->success = false;
  }
  this->log_diagnostics(DiagnosticStatus::OK, this->m_diag_msg_available_);
  this->m_data_mutex_.unlock();
}

/**
 * @brief Set up call to plotting functions
 *
 * @param request: A SelectAnalysisPlot request
 * @param response: Simple response with success and message members
 *
 * @details Upon receiving a request to plot, flags are checked to determine if
 * plotting can actually be performed. Functions are called to store the appropriate
 * data in CSV files. The service request is forwarded to the Python CSVPlotter
 * node for plotting the data in matplotlib
 */
void PDVGInterface::gen_plots_callback(
  const std::shared_ptr<SelectAnalysisPlot::Request> request,
  std::shared_ptr<SelectAnalysisPlot::Response> /* response */){
  using namespace std::chrono_literals;
  this->log_diagnostics(DiagnosticStatus::OK, this->m_diag_msg_busy_);

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

  // Make passthrough request with data directory included
  auto p_request = std::make_shared<SelectAnalysisPlot::Request>();
  p_request = request;
  p_request->data_directory = this->m_plot_data_directory_;

  this->m_csv_plot_client_->async_send_request(
    p_request,
    std::bind(&PDVGInterface::csv_plot_client_callback, this, _1));
  this->log_diagnostics(DiagnosticStatus::OK, this->m_diag_msg_available_);
}

/**
 * @brief Simple callback for plotting via the csv_plotter node
 *
 * @param response Response from the csv_plotter node with info about request completion
 */
void PDVGInterface::csv_plot_client_callback(
  const rclcpp::Client<uav_interfaces::srv::SelectAnalysisPlot>::SharedFuture response){
    if (!response.get()->success){
      this->log_diagnostics(DiagnosticStatus::ERROR, response.get()->message);
    } else {
      this->log_diagnostics(DiagnosticStatus::OK, "Plot operation successful");
    }
}

/**
 * @brief Update the pdvg starting point based on a UavState message
 *
 * @param state: Current state of the UAV
 */
void PDVGInterface::uav_state_callback(const uav_interfaces::msg::UavState &state){
  this->m_data_mutex_.lock();
  this->m_uav_state_ = state;

  // Set flag so PDVG analysis can start
  this->m_have_startpoint_ = true;
  this->m_data_mutex_.unlock();
}

/**
 * @brief Saves a /waypoints message if one is published
 *
 * @param waypoints: A UavWaypoints message retrieved from the /waypoints publisher
 *
 * @details Store waypoints published from the rviz Waypoints panel for use in making
 * the reference trajectory prior to an analysis. Set the m_have_endpoint_ flag
 * to true.
 */
void PDVGInterface::waypoints_sub_callback(const uav_interfaces::msg::UavWaypoints &waypoints){
  // Assign publish waypoints to node variable
  this->m_data_mutex_.lock();
  if (!waypoints.points.empty() && !this->m_self_publish_) {
    auto x_pos = waypoints.points.back().position.point.x;
    auto y_pos = waypoints.points.back().position.point.y;
    this->m_nominal_down_ = waypoints.points.back().position.point.z;
    this->m_target_waypoint_ = {x_pos, y_pos};

    // Change flag to true so analysis can be performed
    this->m_have_endpoint_ = true;
    this->log_diagnostics(DiagnosticStatus::OK, "Waypoints received");
  } else if (this->m_self_publish_) {
    this->log_diagnostics(DiagnosticStatus::OK, "PDVG waypoints published");
  } else {
    this->log_diagnostics(DiagnosticStatus::WARN, "No waypoints were published!");
  }
  this->m_self_publish_ = false;
  this->m_data_mutex_.unlock();
}

/**
 * @brief Callback to publish current diagnostic information
 */
void PDVGInterface::publish_diagnostics(){
  this->m_diag_mutex_.lock();
  auto msg = diagnostic_msgs::msg::DiagnosticArray();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "pdvg_interface";
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
void PDVGInterface::log_diagnostics(const diagnostic_msgs::msg::DiagnosticStatus::_level_type &level,
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
  auto node = std::make_shared<planner_interface::PDVGInterface>();

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
