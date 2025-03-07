/**
 * @File: lincov_interface.hpp
 * @Date: November 2022
 * @Author: Daren Swasey
 *
 * @brief
 * Interface definition for running LinCov estimations and
 * Monte Carlo verifications in pd_planner simulation.
 **/

#ifndef PLANNER_INTERFACE__LINCOV_INTERFACE_HPP_
#define PLANNER_INTERFACE__LINCOV_INTERFACE_HPP_

// Local headers
#include "uav_interfaces/msg/uav_waypoints.hpp"
#include "uav_interfaces/srv/toggle_ellipse.hpp"
#include "uav_interfaces/srv/lin_cov_monte_carlo_call.hpp"
#include "uav_interfaces/srv/select_analysis_plot.hpp"
#include "kalman_filter/dynamics/basic_model.hpp"
#include "kalman_filter/helpers/tools.hpp"

// Eigen headers
#include <Eigen/Dense>

// Global headers
#include <string>
#include <vector>
#include <memory>
#include <mutex>

// ROS2 headers
#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace planner_interface{

/**
 * @brief
 * Performs LinCov and/or Monte Carlo analyses for a
 * user-given waypoint setup in pd_planner_launch simulation.
 *
 * @details
 * The LinCovInterface node collects data necessary for a
 * LinCov and/or Monte Carlo analysis upon request. The request comes in from
 * the node in the lincov_analysis_panel rviz plugin provided
 * by the rviz_plugins package.
 *
 * This node subscribes to /waypoints and utilizes that information
 * to generate trajectories and run the analyses.
 *
 * Output is presented in two forms. Plots can be generated
 * using matplotlib, showing the error covariance 3-sigma bounds
 * for several sets of states.
 *
 * Other visual outputs are 3-sigma uncertainty ellipses
 * visualized in rviz. These can be extremely small
 * depending on the scale of the simulation environment.
 *
 * These different visualization methods can be selected in rviz
 * using the LinCovAnalysis Panel provided in the rviz_plugins
 * package.
 */

class LinCovInterface : public rclcpp::Node{
  public:
    /**
     * @brief Construct a new LinCov Interface object
     *
     * @details Create callback groups and all interfaces to the node
     *
     */
    LinCovInterface();

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
    void plot();

  private:
    /* === VARIABLES === */

    // Waypoints published in rviz during simulation.
    uav_interfaces::msg::UavWaypoints m_waypoints_;

    /* === FLAGS === */
    // True if waypoints have been published from the Waypoints panel in rviz
    // Set back to false after an analysis has been completed, requiring
    // additional waypoints to be published before the next analysis.
    bool m_have_waypoints_;

    // Indicates if this node has data available for plotting.
    // Becomes true after the first LinCov analysis has been performed and
    // does not change thereafter. This allows for plotting the same results
    // again if the user needs to reopen the plots.
    bool m_plot_ready_;

    // Indicates whether the uncertainty ellipses are currently visible
    // Used for toggle logic in toggle_ellipses_callback
    bool m_ellipses_enabled_;

    // Indicates whether the LinCov parameters have been initialized. Initialization
    // only happens once, thus after the initialization, this flag stays true.
    bool m_params_initialized_;

    // Indicates whether a Monte Carlo analysis has been performed for the current
    // waypoints.
    bool m_did_monte_carlo_;

    // Indicates whether a LinCov analysis has been performed for the current
    // waypoints.
    bool m_did_lincov_;

    /* === MUTEX === */
    std::mutex m_data_mutex_;
    std::mutex m_diag_mutex_;

    /* === SERVERS === */
    //  Service server for RVIZ button to start LinCov
    rclcpp::Service<uav_interfaces::srv::LinCovMonteCarloCall>::SharedPtr m_button_server_ptr_;

    // Service server for plotting (request from GUI)
    rclcpp::Service<uav_interfaces::srv::SelectAnalysisPlot>::SharedPtr m_plotting_button_ptr_;

    // Service server for showing uncertainty ellipses
    rclcpp::Service<uav_interfaces::srv::ToggleEllipse>::SharedPtr m_toggle_ellipses_ptr_;

    // Service client for plotting with CSV
    rclcpp::Client<uav_interfaces::srv::SelectAnalysisPlot>::SharedPtr m_csv_plot_client_;

    // Callback groups for button and subscribers to be used
    rclcpp::CallbackGroup::SharedPtr m_analysis_cb_group_;

    /* === SUBSCRIBERS === */
    //  UavWaypoints (/waypoints)
    rclcpp::Subscription<uav_interfaces::msg::UavWaypoints>::SharedPtr m_waypoints_sub_;

    /* === PUBLISHERS === */
    // MarkerArray (/lincov_viz)
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_lincov_publisher_;

    // Diagnostics (/diagnostics)
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_diagnostics_pub_;

    // Diagnostics helpers
    diagnostic_msgs::msg::DiagnosticStatus m_diagnostic_status_;
    rclcpp::TimerBase::SharedPtr m_diag_timer_;
    static const std::string m_diag_msg_available_;
    static const std::string m_diag_msg_busy_;

    /* === MONTE CARLO === */
    // Number of Monte Carlo runs to do when analysis requested
    uint64_t m_num_monte_carlo_runs_;

    // Initial Monte Carlo vector
    Eigen::Matrix<double,
                  1,
                  kf::dynamics::BasicModelDim::MC::FULL_STATE_LEN,
                  Eigen::RowMajor> m_init_sim_vec_;

    /* === TOOLS AND OUTPUT VECTORS === */
    // Object that contains necessary helper tools to run LinCov and/or Monte Carlo
    kf::Tools<kf::dynamics::BasicModelDim, double, Eigen::RowMajor> m_tools_;

    // Matrix that contains reference trajectory over time, where time moves forward
    // along the row dimension of the vector.
    Eigen::Matrix<double, Eigen::Dynamic, kf::dynamics::BasicModelDim::REF_DIM, Eigen::RowMajor> m_ref_trajectory_;

    // Matrix that contains the results of the LinCov analysis. Time is indexed along
    // the row dimension.
    Eigen::Matrix<double,
                  Eigen::Dynamic,
                  kf::dynamics::BasicModelDim::LINCOV::FULL_STATE_LEN,
                  Eigen::RowMajor> m_lincov_sim_vec_;

    // Monte Carlo output vector
    std::vector<Eigen::Matrix<
                double,
                Eigen::Dynamic,
                kf::dynamics::BasicModelDim::MC::FULL_STATE_LEN,
                Eigen::RowMajor>> m_monte_carlo_output;

    /* === TRAJECTORY PARAMETERS === */
    double m_analysis_dt_;         // Step size (s) to run LinCov analysis at
    double m_nominal_velocity_;    // Nominal UAV velocity in m/s
    double m_max_curvature_;       // Max curavature for the UAV
    double m_max_curvature_rate_;  // Max curvature rate for UAV
    double m_nominal_pitch_;       // Nominal UAV pitch angle (radians)
    double m_gravity_;             // Gravitational constant (m/s^2)

    /* === IMU PARAMETERS === */
    // Steady state standard deviation values for accelerometer and gyroscope bias
    double m_accel_std_dev_;
    double m_gyro_std_dev_;

    /* === SENSOR TIME CONSTANTS === */
    // First Order Gauss Markov (FOGM) time constants for each sensor
    double m_compass_time_const_;
    double m_abs_pressure_time_const_;
    double m_accel_time_const_;
    double m_gyro_time_const_;
    double m_feature_range_time_const_;
    double m_feature_bearing_time_const_;
    double m_gps_position_time_const_;

    /* === FEATURE (BUOY) PARAMETERS === */
    std::vector<double> m_feature_locations_x_;   // North position of buoys
    std::vector<double> m_feature_locations_y_;   // East position of buoys
    std::vector<double> m_feature_locations_z_;   // Down position of buoys

    // Ranges of each of the buoys defined above in the feature_locations arrays.
    std::vector<double> m_feature_ranges_;

    /* === VISUALIZATION PARAMETERS === */
    // Array containing the uncertainty ellipse markers.
    visualization_msgs::msg::MarkerArray m_lincov_ellipses_;

    // Array defining the color of the uncertainty ellipses in RGBA format.
    std::array<double, 4> m_rviz_ellipse_rgba_;

    // Which frame id to publish the uncertainty ellipses in.
    std::string m_ellipse_frame_id_;

    /* Array of bools indicating which of the data types specified in
    SelectAnalysisPlot service interface to plot */
    uav_interfaces::srv::SelectAnalysisPlot::Request::_data_types_type m_plot_data_types_;

    /*  Plot types to make as specified by PlotTypeInds in plot_statistics.hpp
    from rrt code, also shown in comments in SelectAnalysisPlot service interface */
    uav_interfaces::srv::SelectAnalysisPlot::Request::_plot_types_type  m_plot_types_;

    // Directory in which to store data for plotting
    std::string m_plot_data_directory_;

    // Downsampling factor for Monte Carlo plotting
    size_t m_mc_run_downsample_;

    // Flag for saving to pdf or not in csv_plotter
    bool m_save_plot_pdf_;

    /* === PRIVATE FUNCTIONS === */
    /* === PARAMETER DECLARATION FUNCTIONS === */
    /**
     * @brief Declare and initialize trajectory-related parameters
     *
     * @details Gets and sets trajectory parameters and the analysis
     * step size. Node is designed to run only
     * if these parameters are passed in at launch
     */
    void declare_trajectory_params();

     /**
      * @brief Declare and initialize feature-related parameters
      *
      * @details Gets and sets NED positions and range for each buoy.
      * Node is designed to run only if these parameters are passed in at launch.
      */
    void declare_feature_params();

    /**
     * @brief Declare and initialize vizualization parameters
     *
     * @details Gets and sets parameters for the color, scale, and frame id
     * for plotting uncertainty ellipses. Node is designed to run only if
     * these parameters are passed in at launch.
     */
    void declare_viz_params();

    /* === TOOL CREATION FUNCTIONS === */

    /**
     * @brief Create tools for performing LinCov or Monte Carlo analysis
     */
    void init_tools();

    /**
     * @brief Create inertial measurements function for tools
     *
     * @details Makes a measurement function for IMU readings for both LinCov
     * and Monte Carlo analyses. These measurement functions are assigned to the
     * m_tools object for their respective analysis just before the analysis is
     * performed. Also declares and gets parameters required for initializing
     * analysis state vectors.
     */
    void init_inertial_measure_tools();

    /**
     * @brief Create dynamics for tools and assign result to m_tools_.dynamics_func
     *
     * @details Uses all sensor bias noise vectors and time constants to create a
     * function that defines the navigation and truth state dynamics
     */
    void init_dynamics_tool();

    /**
     * @brief Create controller for tools and assign result to m_tools_.controller
     */
    void init_controller_tool();

    /**
     * @brief Create measurements controller for tools
     *
     * @details Utilizes sensor parameters (i.e. if sensor is enabled, sensor's
     * measurement period, sensor noise parameters, and other sensor-specific
     * parameters) to provide tool for updating error covariance and augmented
     * covariance matrices. Also creates/uses objects that contain information about each
     * buoy/feature
     */
    void init_measurement_controller_tool();

    /**
     * @brief Create mappings tool
     *
     * @details Create mappings tool that is used to extract parts of the reference state
     * vector and maps them to either a truth state or navigation state vector
     */
    void init_mappings_tool();

    /* === ANALYSIS FUNCTIONS === */

    /**
     * @brief Create the reference trajectory
     *
     * @details Creates the reference trajectory that will be used by both LinCov
     * and Monte Carlo analyses. This function must be used within a try-catch block.
     * If the path generated between 3 waypoints results in an angle too sharp for the UAV,
     * throws an error.
     */
    void make_reference_trajectory();

    /**
     * @brief Initialize LinCov and Monte Carlo state vectors
     *
     * @details Initializes the state vectors that will hold analysis results.
     * It is initialized with a reference trajectory and starting error and augmented
     * covariance matrices. Initialized values for the augmented covariance matrix depend
     * on accelerometer and gyroscope bias standard deviation parameters.
     */
    void make_state_vectors();

    /**
     * @brief Runs a LinCov analysis
     *
     * @details Runs a LinCov analysis. Proper measurement function
     * is assigned prior to analysis call. Flag is set after completion
     * to ensure future service servers respond appropriately to requests.
     */
    void lincov_analysis();

    /**
     * @brief Runs a Monte Carlo analysis
     *
     * @details Runs a Monte Carlo analysis. Proper measurement function
     * is assigned prior to analysis call. Flag is set after completion
     * to ensure future service servers respond appropriately to requests.
     */
    void monte_carlo_analysis();

    /* === VISUALIZATION FUNCTIONS === */
    /**
     * @brief Publishes uncertainty ellipses MarkerArray to "lincov_viz" topic
     */
    void lincov_publish();

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
    void lincov_generate_markers(const double spatial_resolution);

    /**
     * @brief Delete existing covariance markers and publish changes
     *
     * @details Any existing markers are cleared every time new ones are generated.
     * The Markers must be cleared by setting the Marker action to Marker::DELETE
     * and publishing, then clearing the MarkerArray vector so it no longer contains
     * the previous Marker data.
     */
    void lincov_delete_markers();

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
    void analysis_callback(
      const std::shared_ptr<uav_interfaces::srv::LinCovMonteCarloCall::Request> request,
      std::shared_ptr<uav_interfaces::srv::LinCovMonteCarloCall::Response> response);

    /**
     * @brief Sets flags so main thread can plot analysis results
     *
     * @param request: A Trigger request. Data associated with request is not used.
     * @param response: Trigger response that sends back a success bool with a message
     *                  (if necessary) to the rviz LinCov panel node.
     *
     * @details Upon receiving a request to plot, flags are checked to determine if
     * plotting can actually be performed. This callback makes some simple checks for
     * plotting conditions and returns quickly.
     *
     * The matplotlib plotting requires that the plotting be performed in the main
     * thread, so this callback cannot call the plot function itself.
     */
    void gen_plots_callback(
      const std::shared_ptr<uav_interfaces::srv::SelectAnalysisPlot::Request> request,
      std::shared_ptr<uav_interfaces::srv::SelectAnalysisPlot::Response> response);

    /**
     * @brief Simple callback for plotting via the csv_plotter node
     *
     * @param future Response from the csv_plotter node with info about request completion
     */
    void csv_plot_callback(
      const rclcpp::Client<uav_interfaces::srv::SelectAnalysisPlot>::SharedFuture future);

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
    void toggle_ellipses_callback(
      const std::shared_ptr<uav_interfaces::srv::ToggleEllipse::Request> request,
      std::shared_ptr<uav_interfaces::srv::ToggleEllipse::Response> response);


    /**
     * @brief Saves a /waypoints message if one is published
     *
     * @param waypoints: A UavWaypoints message retrieved from the /waypoints publisher
     *
     * @details Store waypoints published from the rviz Waypoints panel for use in making
     * the reference trajectory prior to an analysis. Set the m_have_waypoints_ flag
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
#endif  // PLANNER_INTERFACE__LINCOV_INTERFACE_HPP_
