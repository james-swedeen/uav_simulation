/**
 * @File: pdvg_interface.hpp
 * @Date: January 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Interface definition for making PDVG plans
 * in rviz simulation
 **/

#ifndef PLANNER_INTERFACE__PDVG_INTERFACE_HPP_
#define PLANNER_INTERFACE__PDVG_INTERFACE_HPP_

// Local headers
#include <uav_interfaces/msg/uav_waypoints.hpp>
#include <uav_interfaces/msg/uav_state.hpp>
#include <uav_interfaces/srv/select_analysis_plot.hpp>
#include <kalman_filter/dynamics/dynamics_base.hpp>
#include <kalman_filter/dynamics/basic_model.hpp>
#include <kalman_filter/noise/noise_base.hpp>
#include <kalman_filter/noise/noise_wrapper.hpp>
#include <kalman_filter/helpers/tools.hpp>
#include <rrt_search/helpers/fillet_tools.hpp>
#include <rrt_search/obstacle_checkers/probability_detection_metric_obstacle_checker.hpp>

// Boost Headers
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

// Eigen headers
#include <Eigen/Dense>

// Global headers
#include <list>
#include <string>
#include <vector>
#include <memory>
#include <mutex>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

using SelectAnalysisPlot = uav_interfaces::srv::SelectAnalysisPlot;
namespace planner_interface{

/**
 * @brief Node for generating PDVG plans and plan visualizations
 *
 * @details PDVGInterface node's functionality is called via service
 * request from the PDVG Panel in the rviz_plugins package. The node
 * uses code from the rapidly-exploring-random-tree repo to perform
 * PDVG planning. The node also provides visualizations of completed
 * plans in the form of plots and rviz markers
 */
class PDVGInterface : public rclcpp::Node{
  public:
    /**
     * @brief Construct a new PDVGInterface object
     *
     * @details Create a new PDVGInterface object. Initialize flags
     * and create publishers, subscribers, and services
     */
    PDVGInterface();

  private:
    /* === VARIABLES === */

    /* === FLAGS === */
    // True if at least one waypoint has been published from the Waypoints
    // panel in rviz. Set back to false after an plan has been completed, requiring
    // additional ending waypoint to be published before the next plan.
    bool m_have_endpoint_;

    // True if at least one UavState message has been received since the start
    // of the scenario. Because state is continually published, this will
    // only be set to true once, and will remain true
    bool m_have_startpoint_;

    // Indicates if this node has data available for plotting.
    // Becomes true after the first LinCov analysis has been performed and
    // does not change thereafter. This allows for plotting the same results
    // again if the user needs to reopen the plots.
    bool m_plot_ready_;

    // Indicates whether the LinCov parameters have been initialized. Initialization
    // only happens once, thus after the initialization, this flag stays true.
    bool m_params_initialized_;

    // To prevent grabbing waypoints published by own node that would cause
    // waypoints callback to trigger
    bool m_self_publish_;

    /* === MUTEX === */
    std::mutex m_data_mutex_;
    std::mutex m_diag_mutex_;

    /* === SERVERS/CLIENTS === */
    // Service server for generating PDVG plan from rviz panel
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_button_server_;

    // Service server for plotting from rviz panel
    rclcpp::Service<uav_interfaces::srv::SelectAnalysisPlot>::SharedPtr m_plotting_server_;

    // Client for requesting CSVPlotter plots
    rclcpp::Client<uav_interfaces::srv::SelectAnalysisPlot>::SharedPtr m_csv_plot_client_;

    /* === SUBSCRIBERS === */
    // Make subscription to current Uav State to get starting point
    rclcpp::Subscription<uav_interfaces::msg::UavState>::SharedPtr m_uav_state_sub_;

    // Make subscription to waypoints to get ending point
    rclcpp::Subscription<uav_interfaces::msg::UavWaypoints>::SharedPtr m_uav_waypoints_sub_;

    /* === PUBLISHERS === */
    // UavWaypoints (/waypoints)
    rclcpp::Publisher<uav_interfaces::msg::UavWaypoints>::SharedPtr m_pdvg_path_publisher_;

    // Diagnostics (/diagnostics)
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_diagnostics_pub_;

    // Diagnostics helpers
    diagnostic_msgs::msg::DiagnosticStatus m_diagnostic_status_;
    rclcpp::TimerBase::SharedPtr m_diag_timer_;
    static const std::string m_diag_msg_available_;
    static const std::string m_diag_msg_busy_;

    /* === CALLBACK GROUPS === */
    // Callback groups
    rclcpp::CallbackGroup::SharedPtr m_planner_cb_group_;

    /* === TOOLS AND OUTPUT VECTORS === */
    // Tools object in which the PDVG edge generator will be defined
    rrt::search::FilletTools<kf::dynamics::BasicModelDim::LINCOV::FULL_STATE_LEN,
                             double> m_pdvg_tools_;

    // Tools object used for creating the edge generator
    kf::Tools<kf::dynamics::BasicModelDim, double> m_edge_gen_tools_;

    // UUID generator for publishing paths
    boost::uuids::random_generator m_uuid_gen_;

    // Results vector for PDVG planner
    std::list<Eigen::Matrix<double,
                            1,
                            2,
                            Eigen::RowMajor>> m_pdvg_solution_;

    // Obstacle checker
    rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<
      kf::dynamics::BasicModelDim,
      false,
      false,
      double,
      Eigen::RowMajor> m_obs_checker_;

    /* === TRAJECTORY PARAMETERS === */
    double m_nominal_velocity_;     // Nominal UAV velocity in m/s
    double m_max_curvature_;        // Max curvature for the UAV
    double m_max_curvature_rate_;   // Max curvature rate for UAV
    double m_nominal_pitch_;        // Nominal UAV pitch angle (radians)
    double m_nominal_down_;         // Nominal down to generate trajectory at
    double m_gravity_;              // Gravitational constant (m/s^2)
    bool   m_use_arc_fillet_;        // Flag for using arc fillet

    // UavState
    uav_interfaces::msg::UavState m_uav_state_;

    // Published start and end points for PDVG planner
    Eigen::Matrix<double,
                  1,
                  kf::dynamics::BasicModelDim::LINCOV::FULL_STATE_LEN,
                  Eigen::RowMajor> m_starting_point_;

    Eigen::Matrix<double, 1, 2, Eigen::RowMajor> m_start_point_;
    Eigen::Matrix<double, 1, 2, Eigen::RowMajor> m_prev_start_point_;

    // Vector to store the target waypoint pose in waypoint callback
    Eigen::Matrix<double, 1, 2, Eigen::RowMajor> m_target_waypoint_;
    Eigen::Matrix<double, 1, 2, Eigen::RowMajor> m_prev_target_;

    /* === IMU PARAMETERS === */
    // Steady state standard deviation values for accelerometer and gyroscope bias
    double m_accel_std_dev_;
    double m_gyro_std_dev_;

    /* === NOISE === */
    // These are used to create vectors that define the noise parameters
    // of each of the sensors for the Kalman filter dynamics.
    kf::noise::NoiseWrapperPtr<1, double> m_compass_bias_noise_;
    kf::noise::NoiseWrapperPtr<1, double> m_compass_meas_noise_;
    kf::noise::NoiseWrapperPtr<1, double> m_abs_pressure_bias_noise_;
    kf::noise::NoiseWrapperPtr<1, double> m_abs_pressure_meas_noise_;
    kf::noise::NoiseWrapperPtr<1, double> m_feature_range_bias_noise_;
    kf::noise::NoiseWrapperPtr<3, double> m_feature_bearing_bias_noise_;
    kf::noise::NoiseWrapperPtr<3, double> m_gps_position_bias_noise_;
    kf::noise::NoiseWrapperPtr<3, double> m_gps_meas_noise_;
    kf::noise::NoiseWrapperPtr<3, double> m_accel_bias_noise_;
    kf::noise::NoiseWrapperPtr<3, double> m_accel_meas_noise_;
    kf::noise::NoiseWrapperPtr<3, double> m_gyro_bias_noise_;
    kf::noise::NoiseWrapperPtr<3, double> m_gyro_meas_noise_;
    kf::noise::NoiseWrapperPtr<3, double> m_radar_pos_noise_;
    kf::noise::NoiseWrapperPtr<1, double> m_radar_const_noise_;


    /* === SENSOR TIME CONSTANTS === */
    // First Order Gauss Markov (FOGM) time constants for each sensor
    double m_compass_time_const_;
    double m_abs_pressure_time_const_;
    double m_accel_time_const_;
    double m_gyro_time_const_;
    double m_feature_range_time_const_;
    double m_feature_bearing_time_const_;
    double m_gps_position_time_const_;

    /* === VISUALIZATION === */
    uav_interfaces::msg::UavWaypoints m_pdvg_path_markers_;  // Markers that make up PDVG path
    std::string m_plot_data_directory_;  // String indicating what directory to store CSV files in

    /* === FUNCTIONS === */
    /* === PARAMETER DECLARATION FUNCTIONS === */
    /**
     * @brief Declare the world parameters for the PDVG analysis
     *
     * @details Declares PDVG world parameters, which includes map size,
     * nominal cross section, and starting number of sides for radar radius.
     */
    void declare_pdvg_world_params();

    /**
     * @brief Declare and initialize trajectory-related parameters
     *
     * @details Gets and sets trajectory parameters
     */
    void declare_trajectory_params();

    /**
     * @brief Set up obstacle check parameters
     */
    void declare_obstacle_checker_params();

    /**
     * @brief Declare and initialize noise-related parameters
     *
     * @details Gets and sets all bias noise terms for sensors and extracts
     * the time constants.
     */
    void declare_noise_params();

    /**
     * @brief Declare visualization parameters
     */
    void declare_viz_params();

    /* === TOOL CREATION FUNCTIONS === */
    /**
     * @brief Call all tool initialization functions
     */
    void init_tools();

    /**
     * @brief Create inertial measurements function for tools
     */
    void init_inertial_measure_tools();

    /**
     * @brief Create dynamics for tools
     *
     * @details Uses all sensor bias noise vectors and time constants to create a
     * function that defines the navigation and truth state dynamics
     */
    void init_dynamics_tool();

    /**
     * @brief Create open-loop controller for tools
     */
    void init_controller_tool();

    /**
     * @brief Create measurements controller for tools
     *
     * @details Utilizes sensor parameters (i.e. if sensor is enabled, sensor's
     * measurement period, sensor noise parameters, and other sensor-specific
     * parameters) to provide tool for updating error covariance and augmented
     * covariance matrices.
     */
    void init_measurement_controller_tool();

    /**
     * @brief Create mappings tool
     *
     * @details Create mappings tool that is used to extract parts of the reference state
     * vector and maps them to either a truth state or navigation state vector
     */
    void init_mappings_tool();

    /**
     * @brief Make an edge generator
     *
     * @details Makes line and fillet tools for use in the PDVG edge generator tool
     */
    void init_edge_generator();

    /**
     * @brief Set up obstacle checker for the PDVG planner
     */
    void init_obstacle_checker();

    /* === PLAN GENERATION FUNCTIONS === */
    /**
     * @brief Set up initial conditions for starting a PDVG planner
     *
     * @details Sets up all the required values for the starting point
     * for the PDVG planner operations. This includes the time, starting pose,
     * and covariance matrices.
     */
    void set_initial_conditions();

    /**
     * @brief Run the PDVG planner
     */
    void run_pdvg();

    /* === VISUALIZATION FUNCTIONS === */
    /**
     * @brief Call function to generate CSV file for the probability of detection plot
     */
    void make_pd_plot();

    /**
     * @brief Make call to functions that create CSV file for error budget information
     */
    void plot_error_budget();

    /**
     * @brief Publish a UavWaypoints object for the solution path
     */
    void publish_solution_path();

    /**
     * @brief Check to see if the starting and ending points are the same as before
     *
     * @return true if same start and end are being used
     * @return false otherwise
     */
    bool is_same_init_conditions();

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
    waypoints_to_trajectory();

    /* === CALLBACKS === */
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
    void pdvg_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);

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
    void gen_plots_callback(const std::shared_ptr<uav_interfaces::srv::SelectAnalysisPlot::Request> request,
                            std::shared_ptr<uav_interfaces::srv::SelectAnalysisPlot::Response> response);

    /**
     * @brief Simple callback for plotting via the csv_plotter node
     *
     * @param response Response from the csv_plotter node with info about request completion
     */
    void csv_plot_client_callback(const rclcpp::Client<uav_interfaces::srv::SelectAnalysisPlot>::SharedFuture response);

    /**
     * @brief Update the pdvg starting point based on a UavState message
     *
     * @param state: Current state of the UAV
     */
    void uav_state_callback(const uav_interfaces::msg::UavState &state);

    /**
     * @brief Saves a /waypoints message if one is published
     *
     * @param waypoints: A UavWaypoints message retrieved from the /waypoints publisher
     *
     * @details Store waypoints published from the rviz Waypoints panel for use in making
     * the reference trajectory prior to an analysis. Set the m_have_endpoint_ flag
     * to true.
     */
    void waypoints_sub_callback(const uav_interfaces::msg::UavWaypoints &waypoints);

    /**
     * @brief Publish current diagnostic information
     */
    void publish_diagnostics();

    /**
     * @brief Update the current diagnostic status of the node
     *
     * @param status: Status to make current status (WARN, OK, ERROR)
     * @param message: Message to display in diagnostic monitor
     */
    void log_diagnostics(const diagnostic_msgs::msg::DiagnosticStatus::_level_type &level,
                         const std::string &message);
};
}  // namespace planner_interface
#endif  // PLANNER_INTERFACE__PDVG_INTERFACE_HPP_
