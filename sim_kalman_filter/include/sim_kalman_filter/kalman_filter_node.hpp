/**
 * @File: kalman_filter_node.hpp
 * @Date: September 2022
 * @Author: James Swedeen
 *
 * @brief
 * Implements a real time node that performs the tasks of a kalman filter.
 **/

#ifndef SIM_KALMAN_FILTER_KALMAN_FILTER_NODE_HPP
#define SIM_KALMAN_FILTER_KALMAN_FILTER_NODE_HPP

/* C++ Headers */
#include<list>
#include<mutex>

/* ROS Headers */
#include<std_srvs/srv/empty.hpp>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2/LinearMath/Matrix3x3.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include<rclcpp/rclcpp.hpp>

/* Our Headers */
#include<sensor_msgs/msg/imu.hpp>
#include<uav_interfaces/msg/gps.hpp>
#include<uav_interfaces/msg/pressure.hpp>
#include<uav_interfaces/msg/compass.hpp>
#include<uav_interfaces/msg/uav_state.hpp>
#include<uav_interfaces/msg/line_of_sight_array.hpp>

/* Kalman Filter Headers */
#include<kalman_filter/helpers/versions.hpp>
#include<kalman_filter/helpers/integrator_step.hpp>
#include<kalman_filter/helpers/propagate_error_covariance.hpp>
#include<kalman_filter/helpers/measurement_update.hpp>
#include<kalman_filter/noise/normal_distribution.hpp>
#include<kalman_filter/dynamics/basic_model.hpp>
#include<kalman_filter/mappings/simple_mapping.hpp>
#include<kalman_filter/sensors/measurements/gps.hpp>
#include<kalman_filter/sensors/measurements/heading.hpp>
#include<kalman_filter/sensors/measurements/absolute_pressure.hpp>
#include<kalman_filter/sensors/measurements/ground_velocity.hpp>
#include<kalman_filter/sensors/measurements/feature_range.hpp>
#include<kalman_filter/sensors/measurements/feature_bearing.hpp>
#include<kalman_filter/sensors/inertial_measurements/open_loop_imu.hpp>

/* Local Headers */
#include<sim_kalman_filter/kalman_filter_state.hpp>
#include<sim_kalman_filter/diagnostics_wrapper.hpp>

namespace skf
{
template<kf::Versions VERSION>
class KalmanFilterNode
 : public rclcpp::Node
{
public:
  /**
   * @Default Constructor
   *
   * @brief
   * Starts initialization of the object. Note that finishSetup needs be be called before the node is ready for use.
   **/
  KalmanFilterNode();
  /**
   * @Copy Constructor
   **/
  KalmanFilterNode(const KalmanFilterNode&) = delete;
  /**
   * @Move Constructor
   **/
  KalmanFilterNode(KalmanFilterNode&&) = delete;
  /**
   * @Deconstructor
   **/
  ~KalmanFilterNode() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  KalmanFilterNode& operator=(const KalmanFilterNode&) = delete;
  KalmanFilterNode& operator=(KalmanFilterNode&&)      = delete;
  /**
   * @finishSetup
   *
   * @brief
   * Needs to be called before spinning the node. It finishes the internal initialization of the class.
   **/
  inline void finishSetup();
private:
  // Helpers
  inline static constexpr const Eigen::Index UAV_STATE_DIM     = 28; // The number of elements in the navigation state vector
  inline static constexpr const Eigen::Index ERROR_STATE_DIM   = 24; // The number of elements in the error state vector
  inline static constexpr const Eigen::Index NUM_BIASES        = 15; // The number of elements of the navigation state vector that are sensor bias terms
  inline static constexpr const Eigen::Index NUM_PROCESS_NOISE = kf::dynamics::BasicModelDim::TRUTH_NOISE_DIM; // The number of process noise sources in the system
  struct UAV_IND
  {
  public:
    inline static constexpr const Eigen::Index NORTH_IND                      = 0; // The north component of the UAV's position in NED
    inline static constexpr const Eigen::Index EAST_IND                       = 1; // The east component of the UAV's position in NED
    inline static constexpr const Eigen::Index DOWN_IND                       = 2; // The down component of the UAV's position in NED
    inline static constexpr const Eigen::Index QUAT_W_IND                     = 3; // The w component of the quaternion that goes from the UAV's body frame to the NED frame
    inline static constexpr const Eigen::Index QUAT_X_IND                     = 4; // The x component of the quaternion that goes from the UAV's body frame to the NED frame
    inline static constexpr const Eigen::Index QUAT_Y_IND                     = 5; // The y component of the quaternion that goes from the UAV's body frame to the NED frame
    inline static constexpr const Eigen::Index QUAT_Z_IND                     = 6; // The z component of the quaternion that goes from the UAV's body frame to the NED frame
    inline static constexpr const Eigen::Index NORTH_VEL_IND                  = 7; // The north component of the UAV's NED velocity vector
    inline static constexpr const Eigen::Index EAST_VEL_IND                   = 8; // The east component of the UAV's NED velocity vector
    inline static constexpr const Eigen::Index DOWN_VEL_IND                   = 9; // The down component of the UAV's NED velocity vector
    inline static constexpr const Eigen::Index HEADING_BIAS_IND               = 10; // The heading sensor/compass's bias term
    inline static constexpr const Eigen::Index ABS_PRESSURE_BIAS_IND          = 11; // The absolute pressure sensors bias term
    inline static constexpr const Eigen::Index FEATURE_RANGE_BIAS_IND         = 12; // The feature range sensor's bias term
    inline static constexpr const Eigen::Index FEATURE_BEARING_ROLL_BIAS_IND  = 13; // The roll component of the feature bearing sensor's bias term
    inline static constexpr const Eigen::Index FEATURE_BEARING_PITCH_BIAS_IND = 14; // The pitch component of the feature bearing sensor's bias term
    inline static constexpr const Eigen::Index FEATURE_BEARING_YAW_BIAS_IND   = 15; // The yaw component of the feature bearing sensor's bias term
    inline static constexpr const Eigen::Index GPS_NORTH_BIAS_IND             = 16; // The north component of the GPS position sensor's bias term
    inline static constexpr const Eigen::Index GPS_EAST_BIAS_IND              = 17; // The east component of the GPS position sensor's bias term
    inline static constexpr const Eigen::Index GPS_DOWN_BIAS_IND              = 18; // The down component of the GPS position sensor's bias term
    inline static constexpr const Eigen::Index GYRO_BIAS_X_IND                = 19; // The x component of the gyroscope's bias term
    inline static constexpr const Eigen::Index GYRO_BIAS_Y_IND                = 20; // The y component of the gyroscope's bias term
    inline static constexpr const Eigen::Index GYRO_BIAS_Z_IND                = 21; // The z component of the gyroscope's bias term
    inline static constexpr const Eigen::Index ACCEL_BIAS_X_IND               = 22; // The x component of the accelerometer's bias term
    inline static constexpr const Eigen::Index ACCEL_BIAS_Y_IND               = 23; // The y component of the accelerometer's bias term
    inline static constexpr const Eigen::Index ACCEL_BIAS_Z_IND               = 24; // The z component of the accelerometer's bias term

    inline static constexpr const Eigen::Index X_ANG_VEL                      = 25; // The x component of the UAV's angular velocity in the UAV's body frame
    inline static constexpr const Eigen::Index Y_ANG_VEL                      = 26; // The y component of the UAV's angular velocity in the UAV's body frame
    inline static constexpr const Eigen::Index Z_ANG_VEL                      = 27; // The z component of the UAV's angular velocity in the UAV's body frame
  };
  using SENSOR_HIST_TYPE = std::list<KalmanFilterState<UAV_STATE_DIM,ERROR_STATE_DIM,double,Eigen::RowMajor>>;

  std::unique_ptr<DiagnosticsWrapper> diag; // For publishing diagnostic information
  rclcpp::CallbackGroup::SharedPtr    m_cb_group; // Callback group for all inbound and outbound messages
  /* Sensor history */
  SENSOR_HIST_TYPE           sensor_history; // Holds all of the sensor readings this node receives and orders them in time
  SENSOR_HIST_TYPE::iterator newest_valid_state; // The iterator that correlates to the oldest state in sensor_history that has a valid UAV state estimate
  std::mutex                 sensor_history_mux; // Protects the sensor_history from race conditions
  /* Sensor readings */
  // Used to listen for sensor readings
  rclcpp::Subscription<sensor_msgs::   msg::Imu>::             SharedPtr imu_sub;
  rclcpp::Subscription<uav_interfaces::msg::Gps>::             SharedPtr gps_sub;
  rclcpp::Subscription<uav_interfaces::msg::Pressure>::        SharedPtr pressure_sub;
  rclcpp::Subscription<uav_interfaces::msg::Magnetometer>::    SharedPtr magnetometer_sub;
  rclcpp::Subscription<uav_interfaces::msg::Compass>::         SharedPtr compass_sub;
  rclcpp::Subscription<uav_interfaces::msg::LineOfSightArray>::SharedPtr feature_los_sub;
  /* For collecting wind and buoy information */
  uav_interfaces::msg::UavState::ConstSharedPtr                  newest_truth_state; // Holds the newest truth state received
  std::mutex                                                     truth_state_mux; // Protects newest_truth_state from race conditions
  rclcpp::Subscription<uav_interfaces::msg::UavState>::SharedPtr truth_state_sub; // Listens to the UAV's truth state
  /* Kalman Filter Tools */
  kf::dynamics::BasicModelPtr<kf::dynamics::BasicModelDim,double,Eigen::RowMajor>                             dynamics_model; // Models the UAV's dynamics
  Eigen::Matrix<double,NUM_PROCESS_NOISE,NUM_PROCESS_NOISE,Eigen::RowMajor>                                   truth_process_noise_cov; // The covariance of the truth state process noise
              kf::sensors::GPSPtr<             kf::dynamics::BasicModelDim,true,true,double,Eigen::RowMajor>  gps_model; // Models the GPS's position reading
              kf::sensors::GroundVelocityPtr<  kf::dynamics::BasicModelDim,true,     double,Eigen::RowMajor>  ground_velocity_model; // Models the GPS's ground velocity reading
              kf::sensors::AbsolutePressurePtr<kf::dynamics::BasicModelDim,true,true,double,Eigen::RowMajor>  absolute_pressure_model; // Models the absolute pressure reading
              kf::sensors::HeadingPtr<         kf::dynamics::BasicModelDim,true,true,double,Eigen::RowMajor>  heading_model; // Models the compass's reading
  std::vector<kf::sensors::FeatureRangePtr<    kf::dynamics::BasicModelDim,true,true,double,Eigen::RowMajor>> feature_range_model; // Models the feature range sensor
  std::vector<kf::sensors::FeatureBearingPtr<  kf::dynamics::BasicModelDim,true,true,double,Eigen::RowMajor>> feature_bearing_model; // Models the feature bearing sensor
  kf::map::SimpleMappingPtr<                   kf::dynamics::BasicModelDim,          double,Eigen::RowMajor>  state_mappings; // Maps between the various states that are relevant to a kalman filter
  double                                                                                                      integration_time_step; // The time step that the dynamics will be propagated at
  Eigen::Matrix<double,1,NUM_BIASES,Eigen::RowMajor>                                                          biases_bounds; // The max distance from zero each bias term is allowed to go
  /* For starting, pausing, and resetting the simulation */
  bool                                                                                     running; // True if the simulation is running and false if it is paused
  rclcpp::Time                                                                             start_time_paused; // The start of the last time the simulation was paused
  rclcpp::Duration                                                                         duration_paused; // The total duration that the simulation has spent paused
  std::unique_ptr<KalmanFilterState<UAV_STATE_DIM,ERROR_STATE_DIM,double,Eigen::RowMajor>> initial_state_est; // The first state estimate made
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                                         start_pause_srv; // Subscribes to the services that start, pause, and reset the simulation
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                                         reset_srv;
  /* For publishing state estimate */
  rclcpp::Publisher<uav_interfaces::msg::UavState>::SharedPtr state_est_pub;
  rclcpp::TimerBase::SharedPtr                                state_est_pub_loop;
  /* Windows the length of the sensor history */
  rclcpp::Duration             sensor_history_length; // Any messages further from the current estimate then this duration are deleted
  rclcpp::TimerBase::SharedPtr sensor_history_windowing_loop;
  /**
   * @Sensor Callbacks
   *
   * @brief
   * Each reads in a sensor reading and stores it in the sensor history.
   **/
  void imuCallback(       const sensor_msgs::   msg::Imu::             ConstSharedPtr& imu_msg);
  void gpsCallback(       const uav_interfaces::msg::Gps::             ConstSharedPtr& gps_msg);
  void pressureCallback(  const uav_interfaces::msg::Pressure::        ConstSharedPtr& pressure_msg);
  void compassCallback(   const uav_interfaces::msg::Compass::         ConstSharedPtr& compass_msg);
  void featureLosCallback(const uav_interfaces::msg::LineOfSightArray::ConstSharedPtr& feature_los_msg);
  /**
   * @truthStateCallback
   *
   * @brief
   * Used to update the internally held newest truth state.
   **/
  void truthStateCallback(const uav_interfaces::msg::UavState::ConstSharedPtr& truth_state_msg);
  /**
   * @Simulation Control Callbacks
   *
   * @brief
   * The callbacks that get called when the simulation gets paused, started, and reset.
   **/
  void startPauseCallback(const std_srvs::srv::Empty::Request:: SharedPtr req,
                                std_srvs::srv::Empty::Response::SharedPtr res);
  void resetCallback(const std_srvs::srv::Empty::Request:: SharedPtr req,
                           std_srvs::srv::Empty::Response::SharedPtr res);
  /**
   * @stateEstPubCallback
   *
   * @brief
   * The callback that calculated and publishes the current state estimate.
   **/
  void stateEstPubCallback();
  /**
   * @sensorHistoryWindowingLoop
   *
   * @brief
   * Loop that handles deleting old sensor history when needed.
   **/
  void sensorHistoryWindowingLoop();
  /**
   * @findStateEst
   *
   * @brief
   * Used to find the state estimate at the given time. Also updates the state estimates for each message
   * in the sensor history up until the given time.
   *
   * @parameters
   * sim_time: The time to find the state estimate at
   *
   * @return
   * The state estimate at that time.
   **/
  inline Eigen::Matrix<double,1,UAV_STATE_DIM,Eigen::RowMajor> findStateEst(const rclcpp::Time& sim_time);
  /**
   * @extractIMUVec
   *
   * @brief
   * Extracts the IMU reading into Eigen vectors.
   *
   * @parameters
   * imu_msg: The IMU message to get the needed information from
   * imu_reading: The IMU reading as a vector [x_gyro, y_gyro, z_gyro, x_accel, y_accel, z_accel]
   * imu_noise_covariance: The reading's noise covariance matrix
   **/
  inline static void extractImuVec(const sensor_msgs::msg::Imu::ConstSharedPtr&          imu_msg,
                                   Eigen::Ref<Eigen::Matrix<double,1,6,Eigen::RowMajor>> imu_reading,
                                   Eigen::Ref<Eigen::Matrix<double,6,6,Eigen::RowMajor>> imu_noise_covariance) noexcept;
  /**
   * @integrateState
   *
   * @brief
   * Used to integrate the navigation state and error covariance between messages.
   *
   * @parameters
   * start_time: The starting time for the integration
   * end_time: The target time for the integration to get to
   * cur_imu_reading: The most up to date IMU reading
   * cur_imu_cov: The most up to date IMU covariance
   * starting_nav_state: The navigation state at the beginning of the integration
   * starting_error_cov: The error covariance at the beginning of the integration
   * ending_nav_state: The navigation state at the end of the integration
   * ending_error_cov: The error covariance at the end of the integration
   **/
  inline void integrateState(const double                                                                                   start_time,
                             const double                                                                                   end_time,
                             const Eigen::Ref<const Eigen::Matrix<double,1,              6,              Eigen::RowMajor>>& cur_imu_reading,
                             const Eigen::Ref<const Eigen::Matrix<double,6,              6,              Eigen::RowMajor>>& cur_imu_cov,
                             const Eigen::Ref<const Eigen::Matrix<double,1,              UAV_STATE_DIM,  Eigen::RowMajor>>& starting_nav_state,
                             const Eigen::Ref<const Eigen::Matrix<double,ERROR_STATE_DIM,ERROR_STATE_DIM,Eigen::RowMajor>>& starting_error_cov,
                                   Eigen::Ref<      Eigen::Matrix<double,1,              UAV_STATE_DIM,  Eigen::RowMajor>>  ending_nav_state,
                                   Eigen::Ref<      Eigen::Matrix<double,ERROR_STATE_DIM,ERROR_STATE_DIM,Eigen::RowMajor>>  ending_error_cov) noexcept;
  /**
   * @stateMsgToVec
   *
   * @brief
   * Translates an UavState message into an Eigen state vector.
   *
   * @parameters
   * state_msg: The state message to convert
   *
   * @return
   * The state in a vector.
   **/
  inline static Eigen::Matrix<double,1,UAV_STATE_DIM,Eigen::RowMajor>
    stateMsgToVec(const uav_interfaces::msg::UavState::ConstSharedPtr& state_msg) noexcept;
  /**
   * @findClosestImuReading
   *
   * @brief
   * Finds the nearest imu reading to the provided starting iterator.
   *
   * @parameters
   * newest_valid_state_it: The iterator to the newest valid state in the sensor history buffer
   * imu_reading: Vector to be filled with the newest IMU reading
   * imu_cov: Matrix to be filled with the newest IMU reading's covariance
   *
   * @return
   * True if an IMU reading was found and false otherwise.
   **/
  inline bool findClosestImuReading(const SENSOR_HIST_TYPE::const_iterator&               newest_valid_state_it,
                                    Eigen::Ref<Eigen::Matrix<double,1,6,Eigen::RowMajor>> imu_reading,
                                    Eigen::Ref<Eigen::Matrix<double,6,6,Eigen::RowMajor>> imu_cov) noexcept;
  /**
   * @insertMeasurement
   *
   * @brief
   * Helper function that inserts the given measurement at the correct location in the sensor history.
   *
   * @parameters
   * new_measurement: A new measurement to be added to the sensor history
   **/
  inline void insertMeasurement(const KalmanFilterState<UAV_STATE_DIM,ERROR_STATE_DIM,double,Eigen::RowMajor>& new_measurement);
};


/**
 * @Default Constructor
 *
 * @brief
 * Starts initialization of the object. Note that finishSetup needs be be called before the node is ready for use.
 **/
template<kf::Versions VERSION>
KalmanFilterNode<VERSION>::KalmanFilterNode()
 : Node("kalman_filter"),
   m_cb_group(this->create_callback_group(rclcpp::CallbackGroupType::Reentrant)),
   newest_valid_state(this->sensor_history.end()),
   state_mappings(std::make_shared<kf::map::SimpleMapping<kf::dynamics::BasicModelDim,double,Eigen::RowMajor>>()),
   running(false),
   duration_paused(0, 0),
   sensor_history_length(100, 0)
{
  kf::monteCarloValid<VERSION>();

  this->declare_parameter("imu_topic",                     rclcpp::PARAMETER_STRING);
  this->declare_parameter("gps_topic",                     rclcpp::PARAMETER_STRING);
  this->declare_parameter("pressure_topic",                rclcpp::PARAMETER_STRING);
  this->declare_parameter("compass_topic",                 rclcpp::PARAMETER_STRING);
  this->declare_parameter("feature_los_topic",             rclcpp::PARAMETER_STRING);
  this->declare_parameter("truth_state_topic",             rclcpp::PARAMETER_STRING);
  this->declare_parameter("dynamics.gravity_mag",          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("dynamics.air_density",          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("start_pause_srv_topic",         rclcpp::PARAMETER_STRING);
  this->declare_parameter("reset_srv_topic",               rclcpp::PARAMETER_STRING);
  this->declare_parameter("state_est_topic",               rclcpp::PARAMETER_STRING);
  this->declare_parameter("state_est_pub_frequency",       rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("integration_time_step",         rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("sensor_history_length_sec",     rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("feature_locations_x",           rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("feature_locations_y",           rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("feature_locations_z",           rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("feature_camera_offset_vec",     rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("feature_camera_viewing_angles", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("heading_bias_bounds",           rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("abs_pressure_bias_bounds",      rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("feature_range_bias_bounds",     rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("feature_bearing_bias_bounds",   rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("gps_position_bias_bounds",      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("gyroscope_bias_bounds",         rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("accelerometer_bias_bounds",     rclcpp::PARAMETER_DOUBLE_ARRAY);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = this->m_cb_group;

  // Subscribe to truth state
  this->truth_state_sub = this->create_subscription<uav_interfaces::msg::UavState>(
                            this->get_parameter("truth_state_topic").as_string(),
                            1,
                            std::bind(&KalmanFilterNode<VERSION>::truthStateCallback, this, std::placeholders::_1),
                            sub_options);

  // Subscribe to sensors
  this->imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                    this->get_parameter("imu_topic").as_string(),
                    100,
                    std::bind(&KalmanFilterNode<VERSION>::imuCallback, this, std::placeholders::_1),
                    sub_options);
  this->gps_sub = this->create_subscription<uav_interfaces::msg::Gps>(
                    this->get_parameter("gps_topic").as_string(),
                    100,
                    std::bind(&KalmanFilterNode<VERSION>::gpsCallback, this, std::placeholders::_1),
                    sub_options);
  this->pressure_sub = this->create_subscription<uav_interfaces::msg::Pressure>(
                         this->get_parameter("pressure_topic").as_string(),
                         100,
                         std::bind(&KalmanFilterNode<VERSION>::pressureCallback, this, std::placeholders::_1),
                         sub_options);
  this->compass_sub = this->create_subscription<uav_interfaces::msg::Compass>(
                        this->get_parameter("compass_topic").as_string(),
                        100,
                        std::bind(&KalmanFilterNode<VERSION>::compassCallback, this, std::placeholders::_1),
                        sub_options);
  this->feature_los_sub = this->create_subscription<uav_interfaces::msg::LineOfSightArray>(
                            this->get_parameter("feature_los_topic").as_string(),
                            100,
                            std::bind(&KalmanFilterNode<VERSION>::featureLosCallback, this, std::placeholders::_1),
                            sub_options);

  // Setup kalman filter tools
  // Sensor models
  this->gps_model = std::make_shared<kf::sensors::GPS<kf::dynamics::BasicModelDim,true,true,double,Eigen::RowMajor>>(
                      std::numeric_limits<double>::quiet_NaN());
  this->ground_velocity_model = std::make_shared<kf::sensors::GroundVelocity<kf::dynamics::BasicModelDim,true,double,Eigen::RowMajor>>(
                                  std::numeric_limits<double>::quiet_NaN());
  this->absolute_pressure_model = std::make_shared<kf::sensors::AbsolutePressure<kf::dynamics::BasicModelDim,true,true,double,Eigen::RowMajor>>(
                                    std::numeric_limits<double>::quiet_NaN(),
                                    this->get_parameter("dynamics.gravity_mag").as_double(),
                                    this->get_parameter("dynamics.air_density").as_double());
  this->heading_model = std::make_shared<kf::sensors::Heading<kf::dynamics::BasicModelDim,true,true,double,Eigen::RowMajor>>(
                          std::numeric_limits<double>::quiet_NaN());
  {
    std::vector<double> feature_locations_x;
    std::vector<double> feature_locations_y;
    std::vector<double> feature_locations_z;
    std::vector<double> feature_camera_offset_vec;
    std::vector<double> feature_camera_viewing_angles;

    feature_locations_x           = this->get_parameter("feature_locations_x").          as_double_array();
    feature_locations_y           = this->get_parameter("feature_locations_y").          as_double_array();
    feature_locations_z           = this->get_parameter("feature_locations_z").          as_double_array();
    feature_camera_offset_vec     = this->get_parameter("feature_camera_offset_vec").    as_double_array();
    feature_camera_viewing_angles = this->get_parameter("feature_camera_viewing_angles").as_double_array();
    assert(feature_locations_x.size() == feature_locations_y.size());
    assert(feature_locations_x.size() == feature_locations_z.size());
    assert(3 == feature_camera_offset_vec.    size());
    assert(3 == feature_camera_viewing_angles.size());

    const size_t num_features = feature_locations_x.size();
    this->feature_range_model.  resize(num_features);
    this->feature_bearing_model.resize(num_features);
    for(size_t feature_it = 0; feature_it < num_features; ++feature_it)
    {
      Eigen::Matrix<double,1,3,Eigen::RowMajor> feature_location;
      feature_location[0] = feature_locations_x[feature_it];
      feature_location[1] = feature_locations_y[feature_it];
      feature_location[2] = feature_locations_z[feature_it];

      // Make feature sensor models for this feature location
      this->feature_range_model[feature_it] =
        std::make_shared<kf::sensors::FeatureRange<kf::dynamics::BasicModelDim,true,true,double,Eigen::RowMajor>>(
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::quiet_NaN(),
          feature_location);

      this->feature_bearing_model[feature_it] =
        std::make_shared<kf::sensors::FeatureBearing<kf::dynamics::BasicModelDim,true,true,double,Eigen::RowMajor>>(
          std::numeric_limits<double>::quiet_NaN(),
          Eigen::Map<Eigen::Matrix<double,1,3,Eigen::RowMajor>>(feature_camera_offset_vec.    data()),
          Eigen::Map<Eigen::Matrix<double,1,3,Eigen::RowMajor>>(feature_camera_viewing_angles.data()),
          std::numeric_limits<double>::quiet_NaN(),
          feature_location);
    }
  }
  // Get the integration time step
  this->integration_time_step = this->get_parameter("integration_time_step").as_double();
  // Get the bias bounds
  {
    std::vector<double> temp_vec;

    // Heading bias bounds
    this->biases_bounds[0] = this->get_parameter("heading_bias_bounds").as_double();
    // Absolute pressure bias bounds
    this->biases_bounds[1] = this->get_parameter("abs_pressure_bias_bounds").as_double();
    // Feature range bias bounds
    this->biases_bounds[2] = this->get_parameter("feature_range_bias_bounds").as_double();
    // Feature bearing bias bounds
    temp_vec = this->get_parameter("feature_bearing_bias_bounds").as_double_array();
    assert(3 == temp_vec.size());
    this->biases_bounds(Eigen::seq(UAV_IND::FEATURE_BEARING_ROLL_BIAS_IND-UAV_IND::HEADING_BIAS_IND,
                                   UAV_IND::FEATURE_BEARING_YAW_BIAS_IND- UAV_IND::HEADING_BIAS_IND)) =
      Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp_vec.data());
    // GPS position bias bounds
    temp_vec = this->get_parameter("gps_position_bias_bounds").as_double_array();
    assert(3 == temp_vec.size());
    this->biases_bounds(Eigen::seq(UAV_IND::GPS_NORTH_BIAS_IND-UAV_IND::HEADING_BIAS_IND,
                                   UAV_IND::GPS_DOWN_BIAS_IND- UAV_IND::HEADING_BIAS_IND)) =
      Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp_vec.data());
    // Gyroscope bias bounds
    temp_vec = this->get_parameter("gyroscope_bias_bounds").as_double_array();
    assert(3 == temp_vec.size());
    this->biases_bounds(Eigen::seq(UAV_IND::GYRO_BIAS_X_IND-UAV_IND::HEADING_BIAS_IND,
                                   UAV_IND::GYRO_BIAS_Z_IND-UAV_IND::HEADING_BIAS_IND)) =
      Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp_vec.data());
    // Accelerometer bias bounds
    temp_vec = this->get_parameter("accelerometer_bias_bounds").as_double_array();
    assert(3 == temp_vec.size());
    this->biases_bounds(Eigen::seq(UAV_IND::ACCEL_BIAS_X_IND-UAV_IND::HEADING_BIAS_IND,
                                   UAV_IND::ACCEL_BIAS_Z_IND-UAV_IND::HEADING_BIAS_IND)) =
      Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp_vec.data());

    assert((this->biases_bounds.array() >= 0).all());
  }
}

/**
 * @finishSetup
 *
 * @brief
 * Needs to be called before spinning the node. It finishes the internal initialization of the class.
 **/
template<kf::Versions VERSION>
inline void KalmanFilterNode<VERSION>::finishSetup()
{
  // Interface options that use m_cp_group
  rclcpp::SubscriptionOptions sub_options;
  rclcpp::PublisherOptions    pub_options;
  sub_options.callback_group = this->m_cb_group;
  pub_options.callback_group = this->m_cb_group;

  // Setup diagnostics tool
  this->diag = std::make_unique<DiagnosticsWrapper>(this->shared_from_this(), "Kalman Filter");

  // Setup truth process noise
  {
    const auto compass_bias_noise         = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<1,double,Eigen::RowMajor>(this->shared_from_this(), "dynamics.compass_bias");
    const auto abs_pressure_bias_noise    = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<1,double,Eigen::RowMajor>(this->shared_from_this(), "dynamics.abs_pressure_bias");
    const auto feature_range_bias_noise   = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<1,double,Eigen::RowMajor>(this->shared_from_this(), "dynamics.feature_range_bias");
    const auto feature_bearing_bias_noise = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,double,Eigen::RowMajor>(this->shared_from_this(), "dynamics.feature_bearing_bias");
    const auto gps_pos_bias_noise         = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,double,Eigen::RowMajor>(this->shared_from_this(), "dynamics.gps_pos_bias");
    const auto gyro_bias_noise            = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,double,Eigen::RowMajor>(this->shared_from_this(), "dynamics.gyro_bias");
    const auto accel_bias_noise           = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,double,Eigen::RowMajor>(this->shared_from_this(), "dynamics.accel_bias");

    this->truth_process_noise_cov.setZero();
    this->truth_process_noise_cov(kf::dynamics::BasicModelDim::TRUTH_NOISE::HEADING_BIAS_IND,
                                  kf::dynamics::BasicModelDim::TRUTH_NOISE::HEADING_BIAS_IND) = compass_bias_noise->getCovariance()[0];
    this->truth_process_noise_cov(kf::dynamics::BasicModelDim::TRUTH_NOISE::ABS_PRESSURE_BIAS_IND,
                                  kf::dynamics::BasicModelDim::TRUTH_NOISE::ABS_PRESSURE_BIAS_IND) = abs_pressure_bias_noise->getCovariance()[0];
    this->truth_process_noise_cov(kf::dynamics::BasicModelDim::TRUTH_NOISE::FEATURE_RANGE_BIAS_IND,
                                  kf::dynamics::BasicModelDim::TRUTH_NOISE::FEATURE_RANGE_BIAS_IND) = feature_range_bias_noise->getCovariance()[0];
    this->truth_process_noise_cov.template block<3,3>(kf::dynamics::BasicModelDim::TRUTH_NOISE::FEATURE_BEARING_BIAS_START_IND,
                                                      kf::dynamics::BasicModelDim::TRUTH_NOISE::FEATURE_BEARING_BIAS_START_IND) = feature_bearing_bias_noise->getCovariance();
    this->truth_process_noise_cov.template block<3,3>(kf::dynamics::BasicModelDim::TRUTH_NOISE::GPS_POS_BIAS_START_IND,
                                                      kf::dynamics::BasicModelDim::TRUTH_NOISE::GPS_POS_BIAS_START_IND) = gps_pos_bias_noise->getCovariance();
    this->truth_process_noise_cov.template block<3,3>(kf::dynamics::BasicModelDim::TRUTH_NOISE::GYRO_BIAS_START_IND,
                                                      kf::dynamics::BasicModelDim::TRUTH_NOISE::GYRO_BIAS_START_IND) = gyro_bias_noise->getCovariance();
    this->truth_process_noise_cov.template block<3,3>(kf::dynamics::BasicModelDim::TRUTH_NOISE::ACCEL_BIAS_START_IND,
                                                      kf::dynamics::BasicModelDim::TRUTH_NOISE::ACCEL_BIAS_START_IND) = accel_bias_noise->getCovariance();
  }

  // Setup dynamics model
  this->dynamics_model = std::make_shared<kf::dynamics::BasicModel<kf::dynamics::BasicModelDim,double,Eigen::RowMajor>>(
                           std::numeric_limits<double>::quiet_NaN(),
                           this->get_parameter("dynamics.gravity_mag").as_double(),
                           this->get_parameter("dynamics.compass_bias.time_constant").        as_double_array()[0],
                           this->get_parameter("dynamics.abs_pressure_bias.time_constant").   as_double_array()[0],
                           this->get_parameter("dynamics.feature_range_bias.time_constant").  as_double_array()[0],
                           this->get_parameter("dynamics.feature_bearing_bias.time_constant").as_double_array()[0],
                           this->get_parameter("dynamics.gps_pos_bias.time_constant").        as_double_array()[0],
                           this->get_parameter("dynamics.accel_bias.time_constant").          as_double_array()[0],
                           this->get_parameter("dynamics.gyro_bias.time_constant").           as_double_array()[0]);

  const double heading_bias_ss_var         = (std::pow(this->get_parameter("dynamics.compass_bias.standard_deviation").      as_double_array()[0], 2)*this->get_parameter("dynamics.compass_bias.time_constant").      as_double_array()[0])/double(2);
  const double abs_pressure_bias_ss_var    = (std::pow(this->get_parameter("dynamics.abs_pressure_bias.standard_deviation"). as_double_array()[0], 2)*this->get_parameter("dynamics.abs_pressure_bias.time_constant"). as_double_array()[0])/double(2);
  const double feature_range_bias_ss_var   = (std::pow(this->get_parameter("dynamics.feature_range_bias.standard_deviation").as_double_array()[0], 2)*this->get_parameter("dynamics.feature_range_bias.time_constant").as_double_array()[0])/double(2);
  std::vector<double> feature_bearing_bias_ss_var(3);
  for(Eigen::Index state_it = 0; state_it < 3; ++state_it)
  {
    feature_bearing_bias_ss_var[state_it] = (std::pow(this->get_parameter("dynamics.feature_bearing_bias.standard_deviation").as_double_array()[state_it], 2)*this->get_parameter("dynamics.feature_bearing_bias.time_constant").as_double_array()[state_it])/double(2);
  }
  std::vector<double> gps_pos_bias_ss_var(3);
  for(Eigen::Index state_it = 0; state_it < 3; ++state_it)
  {
    gps_pos_bias_ss_var[state_it] = (std::pow(this->get_parameter("dynamics.gps_pos_bias.standard_deviation").as_double_array()[state_it], 2)*this->get_parameter("dynamics.gps_pos_bias.time_constant").as_double_array()[state_it])/double(2);
  }
  std::vector<double> gyro_bias_ss_var(3);
  for(Eigen::Index state_it = 0; state_it < 3; ++state_it)
  {
    gyro_bias_ss_var[state_it] = (std::pow(this->get_parameter("dynamics.gyro_bias.standard_deviation").as_double_array()[state_it], 2)*this->get_parameter("dynamics.gyro_bias.time_constant").as_double_array()[state_it])/double(2);
  }
  std::vector<double> accel_bias_ss_var(3);
  for(Eigen::Index state_it = 0; state_it < 3; ++state_it)
  {
    accel_bias_ss_var[state_it] = (std::pow(this->get_parameter("dynamics.accel_bias.standard_deviation").as_double_array()[state_it], 2)*this->get_parameter("dynamics.accel_bias.time_constant").as_double_array()[state_it])/double(2);
  }

  // Spin until an initial state estimate is received
  while(rclcpp::ok())
  {
    rclcpp::spin_some(this->shared_from_this());

    if(0 != this->newest_truth_state.use_count()) // If there is a truth state available
    {
      // Add the truth state as the initial state estimate in the sensor history
      this->start_time_paused = rclcpp::Time(this->newest_truth_state->pose.header.stamp, this->get_clock()->get_clock_type());
      this->initial_state_est = std::make_unique<KalmanFilterState<UAV_STATE_DIM,ERROR_STATE_DIM,double,Eigen::RowMajor>>(
                                     this->start_time_paused,
                                     this->stateMsgToVec(this->newest_truth_state),
                                     Eigen::Matrix<double,1,ERROR_STATE_DIM,Eigen::RowMajor>({1000, 1000, 1000,
                                                                                              0.25, 0.25, 0.25,
                                                                                              15, 15, 15,
                                                                                              heading_bias_ss_var,
                                                                                              abs_pressure_bias_ss_var,
                                                                                              feature_range_bias_ss_var,
                                                                                              feature_bearing_bias_ss_var[0], feature_bearing_bias_ss_var[1], feature_bearing_bias_ss_var[2],
                                                                                              gps_pos_bias_ss_var[0], gps_pos_bias_ss_var[1], gps_pos_bias_ss_var[2],
                                                                                              gyro_bias_ss_var[0], gyro_bias_ss_var[1], gyro_bias_ss_var[2],
                                                                                              accel_bias_ss_var[0], accel_bias_ss_var[1], accel_bias_ss_var[2],
                                                                                              }).asDiagonal().toDenseMatrix());
      this->sensor_history.emplace_front(*this->initial_state_est);
      this->newest_valid_state = this->sensor_history.begin();
      break;
    }
    //this->diag->logDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting to receive truth state");
    //RCLCPP_WARN(this->get_logger(), "No truth state messages have been received, needed to produce output");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Setup sensor history windowing
  this->sensor_history_length = std::chrono::duration<double,std::ratio<1>>(this->get_parameter("sensor_history_length_sec").as_double());
  this->sensor_history_windowing_loop =
    rclcpp::create_timer(this,
                         this->get_clock(),
                         std::chrono::duration<double,std::ratio<1>>(this->sensor_history_length.seconds()/double(10)),
                         std::bind(&KalmanFilterNode<VERSION>::sensorHistoryWindowingLoop, this),
                         this->m_cb_group);

  // Setup simulation control services
  this->start_pause_srv = this->create_service<std_srvs::srv::Empty>(
                            this->get_parameter("start_pause_srv_topic").as_string(),
                            std::bind(&KalmanFilterNode<VERSION>::startPauseCallback, this, std::placeholders::_1, std::placeholders::_2));
  this->reset_srv = this->create_service<std_srvs::srv::Empty>(
                      this->get_parameter("reset_srv_topic").as_string(),
                      std::bind(&KalmanFilterNode<VERSION>::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
  // Setup publishing loop
  this->state_est_pub = this->create_publisher<uav_interfaces::msg::UavState>(
                          this->get_parameter("state_est_topic").as_string(),
                          10,
                          pub_options);
  this->state_est_pub_loop =
    rclcpp::create_timer(this,
                         this->get_clock(),
                         std::chrono::duration<double,std::ratio<1>>(double(1)/this->get_parameter("state_est_pub_frequency").as_double()),
                         std::bind(&KalmanFilterNode<VERSION>::stateEstPubCallback, this),
                         this->m_cb_group);
  // Log that the kalman filter is in the paused state
  this->diag->logDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK, "Paused, waiting for call to toggle_execution");
}

/**
 * @Sensor Callbacks
 *
 * @brief
 * Each reads in a sensor reading and stores it in the sensor history.
 **/
template<kf::Versions VERSION>
void KalmanFilterNode<VERSION>::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
{
  std::lock_guard<std::mutex> lock(this->sensor_history_mux);
  if(this->running) // Only accept reading if the simulation is running
  {
    const rclcpp::Time msg_time(rclcpp::Time(imu_msg->header.stamp, this->get_clock()->get_clock_type()) - this->duration_paused);
    // Add reading to sensor history
    this->insertMeasurement(KalmanFilterState<UAV_STATE_DIM,ERROR_STATE_DIM,double,Eigen::RowMajor>(msg_time, imu_msg));
  }
}

/**
 * @Sensor Callbacks
 *
 * @brief
 * Each reads in a sensor reading and stores it in the sensor history.
 **/
template<kf::Versions VERSION>
void KalmanFilterNode<VERSION>::gpsCallback(const uav_interfaces::msg::Gps::ConstSharedPtr& gps_msg)
{
  std::lock_guard<std::mutex> lock(this->sensor_history_mux);
  if(this->running) // Only accept reading if the simulation is running
  {
    const rclcpp::Time msg_time(rclcpp::Time(gps_msg->position.header.stamp, this->get_clock()->get_clock_type()) - this->duration_paused);
    // Add reading to sensor history
    this->insertMeasurement(KalmanFilterState<UAV_STATE_DIM,ERROR_STATE_DIM,double,Eigen::RowMajor>(msg_time, gps_msg));
  }
}

/**
 * @Sensor Callbacks
 *
 * @brief
 * Each reads in a sensor reading and stores it in the sensor history.
 **/
template<kf::Versions VERSION>
void KalmanFilterNode<VERSION>::pressureCallback(const uav_interfaces::msg::Pressure::ConstSharedPtr& pressure_msg)
{
  std::lock_guard<std::mutex> lock(this->sensor_history_mux);
  if(this->running) // Only accept reading if the simulation is running
  {
    const rclcpp::Time msg_time(rclcpp::Time(pressure_msg->stamp, this->get_clock()->get_clock_type()) - this->duration_paused);
    // Add reading to sensor history
    this->insertMeasurement(KalmanFilterState<UAV_STATE_DIM,ERROR_STATE_DIM,double,Eigen::RowMajor>(msg_time, pressure_msg));
  }
}

/**
 * @Sensor Callbacks
 *
 * @brief
 * Each reads in a sensor reading and stores it in the sensor history.
 **/
template<kf::Versions VERSION>
void KalmanFilterNode<VERSION>::compassCallback(const uav_interfaces::msg::Compass::ConstSharedPtr& compass_msg)
{
  std::lock_guard<std::mutex> lock(this->sensor_history_mux);
  if(this->running) // Only accept reading if the simulation is running
  {
    const rclcpp::Time msg_time(rclcpp::Time(compass_msg->header.stamp, this->get_clock()->get_clock_type()) - this->duration_paused);
    // Add reading to sensor history
    this->insertMeasurement(KalmanFilterState<UAV_STATE_DIM,ERROR_STATE_DIM,double,Eigen::RowMajor>(msg_time, compass_msg));
  }
}

/**
 * @Sensor Callbacks
 *
 * @brief
 * Each reads in a sensor reading and stores it in the sensor history.
 **/
template<kf::Versions VERSION>
void KalmanFilterNode<VERSION>::
  featureLosCallback(const uav_interfaces::msg::LineOfSightArray::ConstSharedPtr& feature_los_msg)
{
  std::lock_guard<std::mutex> lock(this->sensor_history_mux);
  if(this->running) // Only accept reading if the simulation is running
  {
    const rclcpp::Time msg_time(rclcpp::Time(feature_los_msg->header.stamp, this->get_clock()->get_clock_type()) - this->duration_paused);
    // Add reading to sensor history
    this->insertMeasurement(KalmanFilterState<UAV_STATE_DIM,ERROR_STATE_DIM,double,Eigen::RowMajor>(msg_time, feature_los_msg));
  }
}

/**
 * @truthStateCallback
 *
 * @brief
 * Used to update the internally held newest truth state.
 **/
template<kf::Versions VERSION>
void KalmanFilterNode<VERSION>::
  truthStateCallback(const uav_interfaces::msg::UavState::ConstSharedPtr& truth_state_msg)
{
  std::lock_guard<std::mutex> lock(this->truth_state_mux);

  // If this is the first truth state received or newer then the one buffered update buffered truth state
  if((0 == this->newest_truth_state.use_count()) or
     (rclcpp::Time(truth_state_msg->pose.header.stamp) > rclcpp::Time(this->newest_truth_state->pose.header.stamp)))
  {
    this->newest_truth_state = truth_state_msg;
  }
}

/**
 * @Simulation Control Callbacks
 *
 * @brief
 * The callbacks that get called when the simulation gets paused, started, and reset.
 **/
template<kf::Versions VERSION>
void KalmanFilterNode<VERSION>::startPauseCallback(const std_srvs::srv::Empty::Request:: SharedPtr /* req */,
                                                         std_srvs::srv::Empty::Response::SharedPtr /* res */)
{
  const rclcpp::Time          srv_time = this->now();
  std::lock_guard<std::mutex> lock(this->sensor_history_mux);
  if(this->running) // Currently not paused
  {
    this->start_time_paused = srv_time;
    this->diag->logDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK, "Paused");
  }
  else // Currently paused
  {
    // Update the length of time that has been spent paused
    this->duration_paused = this->duration_paused + (srv_time - this->start_time_paused);
    assert(this->duration_paused >= rclcpp::Duration(0, 0));
    this->diag->logDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK, "Running");
  }
  this->running = not this->running; // Switch from pause to not pause or vice versa
}

/**
 * @Simulation Control Callbacks
 *
 * @brief
 * The callbacks that get called when the simulation gets paused, started, and reset.
 **/
template<kf::Versions VERSION>
void KalmanFilterNode<VERSION>::resetCallback(const std_srvs::srv::Empty::Request:: SharedPtr /* req */,
                                                    std_srvs::srv::Empty::Response::SharedPtr /* res */)
{
  const rclcpp::Time          srv_time = this->now();
  std::lock_guard<std::mutex> lock(this->sensor_history_mux);
  // Clear history
  this->sensor_history.clear();
  this->duration_paused = rclcpp::Duration(0, 0);
  // Get ready to run again
  this->sensor_history.emplace_front(srv_time, this->initial_state_est->nav_state, this->initial_state_est->covariance);
  this->newest_valid_state = this->sensor_history.begin();
  this->start_time_paused  = srv_time;
  this->diag->logDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK, "State estimate has been reset");
}

/**
 * @stateEstPubCallback
 *
 * @brief
 * The callback that calculated and publishes the current state estimate.
 **/
template<kf::Versions VERSION>
void KalmanFilterNode<VERSION>::stateEstPubCallback()
{
  // Get current time and wind info
  const rclcpp::Time est_time = this->now();
  this->truth_state_mux.lock();
  const uav_interfaces::msg::UavState::ConstSharedPtr truth_state = this->newest_truth_state;
  this->truth_state_mux.unlock();
  // Integrate estimate forward to current time
  this->sensor_history_mux.lock();
  const Eigen::Matrix<double,1,UAV_STATE_DIM,Eigen::RowMajor> state_est = this->findStateEst(est_time);
  this->sensor_history_mux.unlock();
  // Publish estimate
  uav_interfaces::msg::UavState output_msg;

  output_msg.pose.header.stamp    = est_time;
  output_msg.pose.header.frame_id = truth_state->pose.header.frame_id;
  output_msg.pose.pose.position.x = state_est[UAV_IND::NORTH_IND];
  output_msg.pose.pose.position.y = state_est[UAV_IND::EAST_IND];
  output_msg.pose.pose.position.z = state_est[UAV_IND::DOWN_IND];
  {
    const Eigen::Matrix<double,1,3,Eigen::RowMajor> roll_pitch_yaw =
      kf::math::quat::quaternionToRollPitchYaw(state_est.template middleCols<4>(UAV_IND::QUAT_W_IND));

    output_msg.phi   = roll_pitch_yaw[0];
    output_msg.theta = roll_pitch_yaw[1];
    output_msg.psi   = roll_pitch_yaw[2];

    tf2::Quaternion ros_quat;
    ros_quat.setRPY(roll_pitch_yaw[0], roll_pitch_yaw[1], roll_pitch_yaw[2]);

    output_msg.pose.pose.orientation = tf2::toMsg(ros_quat);
  }
  output_msg.twist.header.stamp    = est_time;
  output_msg.twist.header.frame_id = truth_state->twist.header.frame_id;
  {
    const Eigen::Matrix<double,1,3,Eigen::RowMajor> vel_in_body =
      kf::math::quat::quaternionToDirectionCosineMatrix(state_est.template middleCols<4>(UAV_IND::QUAT_W_IND)).transpose() *
      state_est.template middleCols<3>(UAV_IND::NORTH_VEL_IND).transpose();

    output_msg.twist.twist.linear.x = vel_in_body[0];
    output_msg.twist.twist.linear.y = vel_in_body[1];
    output_msg.twist.twist.linear.z = vel_in_body[2];
  }
  output_msg.twist.twist.angular.x     = state_est[UAV_IND::X_ANG_VEL];
  output_msg.twist.twist.angular.y     = state_est[UAV_IND::Y_ANG_VEL];
  output_msg.twist.twist.angular.z     = state_est[UAV_IND::Z_ANG_VEL];
  output_msg.gyro_bias.header.stamp    = est_time;
  output_msg.gyro_bias.header.frame_id = truth_state->gyro_bias.header.frame_id;
  output_msg.gyro_bias.vector.x        = state_est[UAV_IND::GYRO_BIAS_X_IND];
  output_msg.gyro_bias.vector.y        = state_est[UAV_IND::GYRO_BIAS_Y_IND];
  output_msg.gyro_bias.vector.z        = state_est[UAV_IND::GYRO_BIAS_Z_IND];
  // TODO: Needs accelerometer bias
  output_msg.w_n   = truth_state->w_n;
  output_msg.w_e   = truth_state->w_e;
  output_msg.v_a   = truth_state->v_a;
  output_msg.v_g   = truth_state->v_g;
  output_msg.alpha = truth_state->alpha;
  output_msg.beta  = truth_state->beta;
  output_msg.gamma = truth_state->gamma;
  output_msg.chi   = truth_state->chi;

  this->state_est_pub->publish(output_msg);
}

/**
 * @sensorHistoryWindowingLoop
 *
 * @brief
 * Loop that handles deleting old sensor history when needed.
 **/
template<kf::Versions VERSION>
void KalmanFilterNode<VERSION>::sensorHistoryWindowingLoop()
{
  std::lock_guard<std::mutex> lock(this->sensor_history_mux);
  rclcpp::Time                cutoff_time = this->newest_valid_state->getTime() - this->sensor_history_length;
  this->sensor_history.remove_if([&cutoff_time] (const KalmanFilterState<UAV_STATE_DIM,ERROR_STATE_DIM,double,Eigen::RowMajor>& it) -> bool
                                 {
                                   return it.getTime() < cutoff_time;
                                 });
}

/**
 * @findStateEst
 *
 * @brief
 * Used to find the state estimate at the given time. Also updates the state estimates for each message
 * in the sensor history up until the given time.
 *
 * @parameters
 * sim_time: The time to find the state estimate at
 *
 * @return
 * The state estimate at that time.
 **/
template<kf::Versions VERSION>
inline Eigen::Matrix<double,1,KalmanFilterNode<VERSION>::UAV_STATE_DIM,Eigen::RowMajor>
  KalmanFilterNode<VERSION>::findStateEst(const rclcpp::Time& sim_time)
{
  const rclcpp::Time time(sim_time - this->duration_paused); // The time that this estimate is for
  // Check edge cases
  assert(time >= this->newest_valid_state->getTime());
  if(1 == this->sensor_history.size()) // Only initial state estimate available
  {
    if(this->running)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "No measurements have been received, using initial state estimate");
      this->diag->logDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                                "No measurements have been received, using initial state estimate");
    }
    return this->sensor_history.cbegin()->nav_state;
  }
  assert(this->sensor_history.end() != this->newest_valid_state);

  // Find newest imu reading
  Eigen::Matrix<double,1,6,Eigen::RowMajor> newest_imu_reading;
  Eigen::Matrix<double,6,6,Eigen::RowMajor> newest_imu_cov;
  if(not this->findClosestImuReading(this->newest_valid_state, newest_imu_reading, newest_imu_cov))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "No IMU measurements have been received, using initial state estimate");
    this->diag->logDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                              "No IMU measurements have been received, using initial state estimate");
    return this->sensor_history.cbegin()->nav_state;
  }
  if(this->running)
  {
    // Propagate state forward until it gets to the last message before the target time
    SENSOR_HIST_TYPE::iterator next_state_it = std::next(this->newest_valid_state);
    while((this->sensor_history.end() != next_state_it) and (next_state_it->getTime() <= time))
    {
      const double prev_time = this->newest_valid_state->getTime().seconds();
      const double cur_time  = next_state_it->           getTime().seconds();

      // Propagate dynamics from time of last message to the time of the next message
      this->integrateState(prev_time,
                           cur_time,
                           newest_imu_reading,
                           newest_imu_cov,
                           this->newest_valid_state->nav_state,
                           this->newest_valid_state->covariance,
                           next_state_it->nav_state,
                           next_state_it->covariance);

      // Apply measurement update
      switch(next_state_it->getMeasurementType())
      {
        case MeasurementType::GPS:
          {
            auto nav_state = next_state_it->nav_state.template leftCols<kf::dynamics::BasicModelDim::NAV_DIM>();
            // Apply NED measurement
            Eigen::Matrix<double,1,3,Eigen::RowMajor> true_measurement_gps;
            true_measurement_gps[0] = next_state_it->getGPSMeasurement()->position.point.x;
            true_measurement_gps[1] = next_state_it->getGPSMeasurement()->position.point.y;
            true_measurement_gps[2] = next_state_it->getGPSMeasurement()->position.point.z;

            kf::applyMeasurement<3,false,kf::dynamics::BasicModelDim,false,double,Eigen::RowMajor>(
              this->gps_model,
              this->state_mappings,
              cur_time,
              true_measurement_gps,
              Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(next_state_it->getGPSMeasurement()->position_transient_covariance.data()),
              nav_state,
              next_state_it->covariance);
            // TODO: Add course angle measurement
            // Apply Ground Velocity measurement
            /*kf::applyMeasurement<1,kf::dynamics::BasicModelDim,double,Eigen::RowMajor>(
              this->ground_velocity_model,
              this->state_mappings,
              cur_time,
              Eigen::Map<const Eigen::Matrix<double,1,1,Eigen::RowMajor>>(&next_state_it->getGPSMeasurement()->ground_velocity),
              Eigen::Map<const Eigen::Matrix<double,1,1,Eigen::RowMajor>>(&next_state_it->getGPSMeasurement()->ground_velocity_variance),
              nav_state,
              next_state_it->covariance);*/
            break;
          }
        case MeasurementType::MAGNETOMETER:
          assert(false); // TODO: Implement this sensor
          break;
        case MeasurementType::PRESSURE:
          {
            auto nav_state = next_state_it->nav_state.template leftCols<kf::dynamics::BasicModelDim::NAV_DIM>();
            // Apply absolute pressure measurement
            kf::applyMeasurement<1,false,kf::dynamics::BasicModelDim,false,double,Eigen::RowMajor>(
              this->absolute_pressure_model,
              this->state_mappings,
              cur_time,
              Eigen::Map<const Eigen::Matrix<double,1,1,Eigen::RowMajor>>(&next_state_it->getPressureMeasurement()->abs_pressure),
              Eigen::Map<const Eigen::Matrix<double,1,1,Eigen::RowMajor>>(&next_state_it->getPressureMeasurement()->abs_variance),
              nav_state,
              next_state_it->covariance);
            // TODO: Use differential pressure measurement
            break;
          }
        case MeasurementType::COMPASS:
          {
            auto nav_state = next_state_it->nav_state.template leftCols<kf::dynamics::BasicModelDim::NAV_DIM>();
            // Apply compass measurement
            kf::applyMeasurement<1,true,kf::dynamics::BasicModelDim,false,double,Eigen::RowMajor>(
              this->heading_model,
              this->state_mappings,
              cur_time,
              Eigen::Map<const Eigen::Matrix<double,1,1,Eigen::RowMajor>>(&next_state_it->getCompassMeasurement()->angle),
              Eigen::Map<const Eigen::Matrix<double,1,1,Eigen::RowMajor>>(&next_state_it->getCompassMeasurement()->variance),
              nav_state,
              next_state_it->covariance);
            break;
          }
        case MeasurementType::IMU:
          this->extractImuVec(next_state_it->getIMUMeasurement(), newest_imu_reading, newest_imu_cov);
          next_state_it->nav_state.template rightCols<3>() = newest_imu_reading.template leftCols<3>();
          break;
        case MeasurementType::FEATURE_LOS:
          {
            auto         nav_state    = next_state_it->nav_state.template leftCols<kf::dynamics::BasicModelDim::NAV_DIM>();
            const size_t num_features = next_state_it->getLosMeasurement()->los.size();
            assert(num_features == this->feature_range_model.size());

            // For each possible feature reading
            for(size_t feature_it = 0; feature_it < num_features; ++feature_it)
            {
              if(-1 != next_state_it->getLosMeasurement()->los[feature_it].range_variance) // If there is a measurement
              {
                // Apply feature range measurement
                kf::applyMeasurement<1,false,kf::dynamics::BasicModelDim,false,double,Eigen::RowMajor>(
                  this->feature_range_model[feature_it],
                  this->state_mappings,
                  cur_time,
                  Eigen::Map<const Eigen::Matrix<double,1,1,Eigen::RowMajor>>(&next_state_it->getLosMeasurement()->los[feature_it].range),
                  Eigen::Map<const Eigen::Matrix<double,1,1,Eigen::RowMajor>>(&next_state_it->getLosMeasurement()->los[feature_it].range_variance),
                  nav_state,
                  next_state_it->covariance);
                // Apply feature bearing measurement
                kf::applyMeasurement<2,false,kf::dynamics::BasicModelDim,false,double,Eigen::RowMajor>(
                  this->feature_bearing_model[feature_it],
                  this->state_mappings,
                  cur_time,
                  Eigen::Map<const Eigen::Matrix<double,1,2,Eigen::RowMajor>>(next_state_it->getLosMeasurement()->los[feature_it].bearing.           data()),
                  Eigen::Map<const Eigen::Matrix<double,2,2,Eigen::RowMajor>>(next_state_it->getLosMeasurement()->los[feature_it].bearing_covariance.data()),
                  nav_state,
                  next_state_it->covariance);
              }
            }
            break;
          }
        default:
          assert(false);
          break;
      };
      // Apply state bounding if needed
      if constexpr(kf::applyStateBounds(VERSION))
      {
        auto bounded_states = next_state_it->nav_state(Eigen::seq(UAV_IND::HEADING_BIAS_IND, UAV_IND::ACCEL_BIAS_Z_IND));

        bounded_states = (bounded_states.array() >  this->biases_bounds.array()).select( this->biases_bounds, bounded_states);
        bounded_states = (bounded_states.array() < -this->biases_bounds.array()).select(-this->biases_bounds, bounded_states);
      }
      // Update last_valid_state_it and next_state_it
      this->newest_valid_state = next_state_it;
      next_state_it            = std::next(this->newest_valid_state);
    }
    // Propagate state to the target time
    Eigen::Matrix<double,1,UAV_STATE_DIM,Eigen::RowMajor> output;
    {
      const double start_time = this->newest_valid_state->getTime().seconds();
      const double time_diff  = time.seconds() - start_time;

      // Set the state from the last message propagated to
      output = this->newest_valid_state->nav_state;
      output.template rightCols<3>() = newest_imu_reading.template leftCols<3>();

      if(0 != time_diff)
      {
        const size_t num_steps = std::ceil(time_diff / this->integration_time_step);
        const double time_step = time_diff / double(num_steps);

        // Integrate the navigation state dynamics forward in time
        double cur_time = start_time;
        for(size_t time_it = 0; time_it < num_steps; ++time_it, cur_time += time_step)
        {
          // Propagate navigation state
          output.template leftCols<kf::dynamics::BasicModelDim::NAV_DIM>() =
            kf::integratorStep<VERSION,double,Eigen::Matrix<double,1,kf::dynamics::BasicModelDim::NAV_DIM,Eigen::RowMajor>>(
              std::bind(&kf::dynamics::BasicModel<kf::dynamics::BasicModelDim,double,Eigen::RowMajor>::getNavStateDynamics,
                        this->dynamics_model.get(),
                        cur_time,
                        std::placeholders::_1,
                        Eigen::Matrix<double,1,kf::dynamics::BasicModelDim::REF_DIM,Eigen::RowMajor>::Constant(std::numeric_limits<double>::quiet_NaN()),
                        newest_imu_reading),
              output.template leftCols<kf::dynamics::BasicModelDim::NAV_DIM>(),
              time_step);
        }
      }
    }
    // Log how long it look to find this state estimate
    {
      diagnostic_msgs::msg::DiagnosticStatus::_values_type msg_vals(1);
      msg_vals.front().key = "Time to find state estimate (sec)";
      msg_vals.front().value = std::to_string((this->now() - sim_time).seconds());

      this->diag->logDiagnostic(diagnostic_msgs::msg::DiagnosticStatus::OK,
                                "Publishing state estimate",
                                std::move(msg_vals));
    }
    return output;
  }
  else // Not running
  {
    Eigen::Matrix<double,1,UAV_STATE_DIM,Eigen::RowMajor> output;

    output = this->newest_valid_state->nav_state;
    output.template rightCols<3>() = newest_imu_reading.template leftCols<3>();

    return output;
  }
}

/**
 * @extractIMUVec
 *
 * @brief
 * Extracts the IMU reading into Eigen vectors.
 *
 * @parameters
 * imu_msg: The IMU message to get the needed information from
 * imu_reading: The IMU reading as a vector [x_gyro, y_gyro, z_gyro, x_accel, y_accel, z_accel]
 * imu_noise_covariance: The reading's noise covariance matrix
 **/
template<kf::Versions VERSION>
inline void KalmanFilterNode<VERSION>::
  extractImuVec(const sensor_msgs::msg::Imu::ConstSharedPtr&          imu_msg,
                Eigen::Ref<Eigen::Matrix<double,1,6,Eigen::RowMajor>> imu_reading,
                Eigen::Ref<Eigen::Matrix<double,6,6,Eigen::RowMajor>> imu_noise_covariance) noexcept
{
  // IMU reading itself
  imu_reading[kf::dynamics::BasicModelDim::INER_MEAS::ROLL_RATE_IND]  = imu_msg->angular_velocity.x;
  imu_reading[kf::dynamics::BasicModelDim::INER_MEAS::PITCH_RATE_IND] = imu_msg->angular_velocity.y;
  imu_reading[kf::dynamics::BasicModelDim::INER_MEAS::YAW_RATE_IND]   = imu_msg->angular_velocity.z;
  imu_reading[kf::dynamics::BasicModelDim::INER_MEAS::X_ACCEL_IND]    = imu_msg->linear_acceleration.x;
  imu_reading[kf::dynamics::BasicModelDim::INER_MEAS::Y_ACCEL_IND]    = imu_msg->linear_acceleration.y;
  imu_reading[kf::dynamics::BasicModelDim::INER_MEAS::Z_ACCEL_IND]    = imu_msg->linear_acceleration.z;

  imu_noise_covariance.setZero();
  // Gyroscope reading covariance
  imu_noise_covariance.template block<3,3>(kf::dynamics::BasicModelDim::INER_MEAS::GYRO_START_IND,
                                           kf::dynamics::BasicModelDim::INER_MEAS::GYRO_START_IND) =
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(imu_msg->angular_velocity_covariance.data());
  // Accelerometer reading covariance
  imu_noise_covariance.template block<3,3>(kf::dynamics::BasicModelDim::INER_MEAS::ACCEL_START_IND,
                                           kf::dynamics::BasicModelDim::INER_MEAS::ACCEL_START_IND) =
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor>>(imu_msg->linear_acceleration_covariance.data());
}

/**
 * @integrateState
 *
 * @brief
 * Used to integrate the navigation state and error covariance between messages.
 *
 * @parameters
 * start_time: The starting time for the integration
 * end_time: The target time for the integration to get to
 * cur_imu_reading: The most up to date IMU reading
 * cur_imu_cov: The most up to date IMU covariance
 * starting_nav_state: The navigation state at the beginning of the integration
 * starting_error_cov: The error covariance at the beginning of the integration
 * ending_nav_state: The navigation state at the end of the integration
 * ending_error_cov: The error covariance at the end of the integration
 **/
template<kf::Versions VERSION>
inline void KalmanFilterNode<VERSION>::
  integrateState(const double                                                                                   start_time,
                 const double                                                                                   end_time,
                 const Eigen::Ref<const Eigen::Matrix<double,1,              6,              Eigen::RowMajor>>& cur_imu_reading,
                 const Eigen::Ref<const Eigen::Matrix<double,6,              6,              Eigen::RowMajor>>& cur_imu_cov,
                 const Eigen::Ref<const Eigen::Matrix<double,1,              UAV_STATE_DIM,  Eigen::RowMajor>>& starting_nav_state,
                 const Eigen::Ref<const Eigen::Matrix<double,ERROR_STATE_DIM,ERROR_STATE_DIM,Eigen::RowMajor>>& starting_error_cov,
                       Eigen::Ref<      Eigen::Matrix<double,1,              UAV_STATE_DIM,  Eigen::RowMajor>>  ending_nav_state,
                       Eigen::Ref<      Eigen::Matrix<double,ERROR_STATE_DIM,ERROR_STATE_DIM,Eigen::RowMajor>>  ending_error_cov) noexcept
{
  const double time_diff = end_time - start_time;

  // Initialize the output variables
  ending_nav_state = starting_nav_state;
  ending_error_cov = starting_error_cov;
  ending_nav_state.template rightCols<3>() = cur_imu_reading.template leftCols<3>();

  if(0 != time_diff)
  {
    const size_t num_steps = std::ceil(time_diff / this->integration_time_step);
    const double time_step = time_diff / double(num_steps);

    double cur_time = start_time;
    for(size_t time_it = 0; time_it < num_steps; ++time_it, cur_time += time_step)
    {
      // Propagate navigation state
      ending_nav_state.template leftCols<kf::dynamics::BasicModelDim::NAV_DIM>() =
        kf::integratorStep<VERSION,double,Eigen::Matrix<double,1,kf::dynamics::BasicModelDim::NAV_DIM,Eigen::RowMajor>>(
          std::bind(&kf::dynamics::BasicModel<kf::dynamics::BasicModelDim,double,Eigen::RowMajor>::getNavStateDynamics,
                    this->dynamics_model.get(),
                    cur_time,
                    std::placeholders::_1,
                    Eigen::Matrix<double,1,kf::dynamics::BasicModelDim::REF_DIM,Eigen::RowMajor>::Constant(std::numeric_limits<double>::quiet_NaN()),
                    cur_imu_reading),
          ending_nav_state.template leftCols<kf::dynamics::BasicModelDim::NAV_DIM>(),
          time_step);
      // Propagate error state covariance
      ending_error_cov =
        kf::propagateErrorCovariance<kf::dynamics::BasicModelDim,VERSION,double,Eigen::RowMajor>(
          ending_error_cov,
          this->dynamics_model,
          std::make_shared<kf::sensors::OpenLoopIMU<kf::dynamics::BasicModelDim,double,Eigen::RowMajor>>(),
          this->state_mappings,
          time_step,
          cur_time,
          ending_nav_state.template leftCols<kf::dynamics::BasicModelDim::NAV_DIM>(),
          Eigen::Matrix<double,1,kf::dynamics::BasicModelDim::REF_DIM,Eigen::RowMajor>::Constant(std::numeric_limits<double>::quiet_NaN()),
          cur_imu_reading,
          cur_imu_reading,
          this->truth_process_noise_cov,
          cur_imu_cov);
    }
  }
}

/**
 * @stateMsgToVec
 *
 * @brief
 * Translates an UavState message into an Eigen state vector.
 *
 * @parameters
 * state_msg: The state message to convert
 *
 * @return
 * The state in a vector.
 **/
template<kf::Versions VERSION>
inline Eigen::Matrix<double,1,KalmanFilterNode<VERSION>::UAV_STATE_DIM,Eigen::RowMajor> KalmanFilterNode<VERSION>::
  stateMsgToVec(const uav_interfaces::msg::UavState::ConstSharedPtr& state_msg) noexcept
{
  Eigen::Matrix<double,1,UAV_STATE_DIM,Eigen::RowMajor> output;

  // UAV position states
  output[UAV_IND::NORTH_IND] = state_msg->pose.pose.position.x;
  output[UAV_IND::EAST_IND]  = state_msg->pose.pose.position.y;
  output[UAV_IND::DOWN_IND]  = state_msg->pose.pose.position.z;
  // UAV attitude states
  {
    Eigen::Matrix<double,1,1,Eigen::RowMajor> roll, pitch, yaw;
    tf2::Quaternion ros_quat;
    tf2::fromMsg(state_msg->pose.pose.orientation, ros_quat);
    tf2::Matrix3x3  ros_rot(ros_quat);
    ros_rot.getRPY(roll[0], pitch[0], yaw[0]);

    output.template middleCols<4>(UAV_IND::QUAT_W_IND) =
      kf::math::quat::normalize(kf::math::quat::rollPitchYawToQuaternion(roll, pitch, yaw));
  }
  // UAV linear velocity states
  output[UAV_IND::NORTH_VEL_IND] = state_msg->twist.twist.linear.x;
  output[UAV_IND::EAST_VEL_IND]  = state_msg->twist.twist.linear.y;
  output[UAV_IND::DOWN_VEL_IND]  = state_msg->twist.twist.linear.z;
  output.template middleCols<3>(UAV_IND::NORTH_VEL_IND) =
    kf::math::quat::quaternionToDirectionCosineMatrix(output.template middleCols<4>(UAV_IND::QUAT_W_IND)) *
    output.template middleCols<3>(UAV_IND::NORTH_VEL_IND).transpose();
  // TODO: Need to add other biases from message
  // Bias states
  output[UAV_IND::HEADING_BIAS_IND]       = 0;
  output[UAV_IND::ABS_PRESSURE_BIAS_IND]  = 0;
  output[UAV_IND::FEATURE_RANGE_BIAS_IND] = 0;
  output.template middleCols<3>(UAV_IND::FEATURE_BEARING_ROLL_BIAS_IND).setZero();
  output.template middleCols<3>(UAV_IND::GPS_NORTH_BIAS_IND).setZero();
  output[UAV_IND::GYRO_BIAS_X_IND] = state_msg->gyro_bias.vector.x;
  output[UAV_IND::GYRO_BIAS_Y_IND] = state_msg->gyro_bias.vector.y;
  output[UAV_IND::GYRO_BIAS_Z_IND] = state_msg->gyro_bias.vector.z;
  // TODO: Need to get Accel bias from message
  output[UAV_IND::ACCEL_BIAS_X_IND] = 0;
  output[UAV_IND::ACCEL_BIAS_Y_IND] = 0;
  output[UAV_IND::ACCEL_BIAS_Z_IND] = 0;
  // UAV angular velocity states
  output[UAV_IND::X_ANG_VEL] = state_msg->twist.twist.angular.x;
  output[UAV_IND::Y_ANG_VEL] = state_msg->twist.twist.angular.y;
  output[UAV_IND::Z_ANG_VEL] = state_msg->twist.twist.angular.z;

  return output;
}

/**
 * @findClosestImuReading
 *
 * @brief
 * Finds the nearest imu reading to the provided starting iterator.
 *
 * @parameters
 * newest_valid_state_it: The iterator to the newest valid state in the sensor history buffer
 * imu_reading: Vector to be filled with the newest IMU reading
 * imu_cov: Matrix to be filled with the newest IMU reading's covariance
 *
 * @return
 * True if an IMU reading was found and false otherwise.
 **/
template<kf::Versions VERSION>
inline bool KalmanFilterNode<VERSION>::
  findClosestImuReading(const SENSOR_HIST_TYPE::const_iterator&               newest_valid_state_it,
                        Eigen::Ref<Eigen::Matrix<double,1,6,Eigen::RowMajor>> imu_reading,
                        Eigen::Ref<Eigen::Matrix<double,6,6,Eigen::RowMajor>> imu_cov) noexcept
{
  bool imu_found = false;
  // Look backward in time
  for(auto reading_it = newest_valid_state_it; reading_it != this->sensor_history.begin(); --reading_it)
  {
    if(MeasurementType::IMU == reading_it->getMeasurementType())
    {
      this->extractImuVec(reading_it->getIMUMeasurement(), imu_reading, imu_cov);
      imu_found = true;
      break;
    }
  }
  if(not imu_found)
  {
    // Look forward in time
    for(auto reading_it = std::next(newest_valid_state_it); reading_it != this->sensor_history.end(); ++reading_it)
    {
      if(MeasurementType::IMU == reading_it->getMeasurementType())
      {
        this->extractImuVec(reading_it->getIMUMeasurement(), imu_reading, imu_cov);
        imu_found = true;
        break;
      }
    }
  }
  return imu_found;
}

/**
 * @insertMeasurement
 *
 * @brief
 * Helper function that inserts the given measurement at the correct location in the sensor history.
 *
 * @parameters
 * new_measurement: A new measurement to be added to the sensor history
 **/
template<kf::Versions VERSION>
inline void KalmanFilterNode<VERSION>::
  insertMeasurement(const KalmanFilterState<UAV_STATE_DIM,ERROR_STATE_DIM,double,Eigen::RowMajor>& new_measurement)
{
  if(new_measurement.getTime() > this->sensor_history.cbegin()->getTime()) // If the reading is newer then the initial state estimate
  {
    SENSOR_HIST_TYPE::iterator insert_location = std::prev(this->sensor_history.end());
    while(new_measurement < *insert_location)
    {
      --insert_location;
    }
    ++insert_location;
    const SENSOR_HIST_TYPE::iterator new_it = this->sensor_history.insert(insert_location, new_measurement);
    assert(*std::prev(new_it) < *new_it);
    assert((this->sensor_history.cend() == std::next(new_it)) or (*std::next(new_it) > *new_it));
    // If its newer then newest valid state update newest valid state to be behind this reading
    if(*new_it < *this->newest_valid_state)
    {
      this->newest_valid_state = std::prev(new_it);
    }
  }
}
} // namespace skf

#endif
/* kalman_filter_node.hpp */
