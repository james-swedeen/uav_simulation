/**
 * @File: feature_sensor_node.cpp
 * @Date: October 2022
 * @Author: James Swedeen
 **/

/* C++ Headers */
#include<vector>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Our Headers */
#include<uav_interfaces/msg/uav_state.hpp>
#include<uav_interfaces/msg/buoy_pose_radius_array.hpp>
#include<uav_interfaces/msg/line_of_sight_array.hpp>

/* Kalman Filter Headers */
#include<kalman_filter/helpers/dimension_struct.hpp>
#include<kalman_filter/noise/normal_distribution.hpp>
#include<kalman_filter/sensors/measurements/feature_range.hpp>
#include<kalman_filter/sensors/measurements/feature_bearing.hpp>

/* Local Headers */
#include<sim_kalman_filter/feature_sensor_node.hpp>

namespace skf
{
/**
 * @Default Constructor
 *
 * @brief
 * Uses ROS parameters to initialize the node internally.
 **/
FeatureSensorNode::FeatureSensorNode()
 : rclcpp::Node("feature_sensor")
{
  std::vector<double> temp_vec;

  this->declare_parameter("uav_state_topic",          rclcpp::PARAMETER_STRING);
  this->declare_parameter("buoy_state_topic",         rclcpp::PARAMETER_STRING);
  this->declare_parameter("los_topic",                rclcpp::PARAMETER_STRING);
  this->declare_parameter("los_pub_frequency",        rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("camera_offset_vec",        rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("camera_viewing_angles",    rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("range_noise_covariance",   rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("bearing_noise_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Subscribe to topics
  this->uav_state_sub = this->create_subscription<uav_interfaces::msg::UavState>(
                          this->get_parameter("uav_state_topic").as_string(),
                          rclcpp::SystemDefaultsQoS(),
                          std::bind(&FeatureSensorNode::uavStateCallback, this, std::placeholders::_1));
  this->buoy_state_sub = this->create_subscription<uav_interfaces::msg::BuoyPoseRadiusArray>(
                           this->get_parameter("buoy_state_topic").as_string(),
                           rclcpp::SystemDefaultsQoS(),
                           std::bind(&FeatureSensorNode::buoyStateCallback, this, std::placeholders::_1));
  // Get model info
  temp_vec = this->get_parameter("camera_offset_vec").as_double_array();
  assert(3 == temp_vec.size());
  this->camera_offset_vec = Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp_vec.data());

  temp_vec = this->get_parameter("camera_viewing_angles").as_double_array();
  assert(3 == temp_vec.size());
  this->camera_viewing_angles = Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp_vec.data());

  this->range_noise_covariance.setConstant(this->get_parameter("range_noise_covariance").as_double());

  temp_vec = this->get_parameter("bearing_noise_covariance").as_double_array();
  assert(4 == temp_vec.size());
  this->bearing_noise_covariance = Eigen::Map<const Eigen::Matrix<double,2,2,Eigen::RowMajor>>(temp_vec.data());
  // Start publishing
  this->los_pub = this->create_publisher<uav_interfaces::msg::LineOfSightArray>(
                    this->get_parameter("los_topic").as_string(),
                    rclcpp::SystemDefaultsQoS());
  this->los_pub_loop =
    rclcpp::create_timer(this,
                         this->get_clock(),
                         std::chrono::duration<double,std::ratio<1>>(double(1)/this->get_parameter("los_pub_frequency").as_double()),
                         std::bind(&FeatureSensorNode::losPublishingCallback, this));
}

/**
 * @uavStateCallback
 *
 * @brief
 * Used to read in the UAV state.
 **/
void FeatureSensorNode::uavStateCallback(const uav_interfaces::msg::UavState::ConstSharedPtr& uav_state_msg)
{
  // If this is the first UAV state received or it's newer then the newest UAV state update the newest UAV state buffer
  if((0 == this->newest_uav_state.use_count()) or
     (rclcpp::Time(uav_state_msg->pose.header.stamp) > rclcpp::Time(this->newest_uav_state->pose.header.stamp)))
  {
    this->newest_uav_state = uav_state_msg;
  }
}

/**
 * @buoyStateCallback
 *
 * @brief
 * Used to read in the buoy state.
 **/
void FeatureSensorNode::
  buoyStateCallback(const uav_interfaces::msg::BuoyPoseRadiusArray::ConstSharedPtr& buoy_state_msg)
{
  // If this is the first buoy state received or it's newer then the newest buoy state update the newest buoy state buffer
  if((0 == this->newest_buoy_state.use_count()) or
     (rclcpp::Time(buoy_state_msg->poses.poses[0].header.stamp) >
      rclcpp::Time(this->newest_buoy_state->poses.poses[0].header.stamp)))
  {
    this->newest_buoy_state = buoy_state_msg;
  }
}

/**
 * @losPublishingCallback
 *
 * @brief
 * Publishes the LOS data.
 **/
void FeatureSensorNode::losPublishingCallback()
{
  const rclcpp::Time pub_time = this->now();
  // Make sure the needed information is buffered internally
  if(0 == this->newest_uav_state.use_count())
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "No UAV state has been received, can't publish without it");
    return;
  }
  if(0 == this->newest_buoy_state.use_count())
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "No buoy state has been received, can't publish without it");
    return;
  }

  bool                                             feature_in_range = false;
  uav_interfaces::msg::LineOfSightArray            out_msg;
  const size_t                                     num_buoys = this->newest_buoy_state->radii.size();
  const Eigen::Matrix<double,1,11,Eigen::RowMajor> uav_state = this->getUavState();

  // Loop over each buoy
  out_msg.los.resize(num_buoys);
  for(size_t buoy_it = 0; buoy_it < num_buoys; ++buoy_it)
  {
    Eigen::Matrix<double,1,3,Eigen::RowMajor> buoy_location;
    buoy_location[0] = this->newest_buoy_state->poses.poses[buoy_it].pose.pose.position.x;
    buoy_location[1] = this->newest_buoy_state->poses.poses[buoy_it].pose.pose.position.y;
    buoy_location[2] = this->newest_buoy_state->poses.poses[buoy_it].pose.pose.position.z;

    // Make range sensor model
    kf::sensors::FeatureRange<FeatureDim,true,true,double,Eigen::RowMajor> feature_range_model(
      std::numeric_limits<double>::quiet_NaN(),
      this->newest_buoy_state->radii[buoy_it],
      buoy_location);

    if(feature_range_model.measurementReady(0, 0, uav_state)) // If the buoy is in range
    {
      feature_in_range = true;

      // Noise sources
      kf::noise::NoiseBasePtr<1,double,Eigen::RowMajor> feature_range_noise =
        std::make_shared<kf::noise::NormalDistribution<1,true,true,false,double,Eigen::RowMajor>>(
          Eigen::Matrix<double,1,1,Eigen::RowMajor>::Zero(),
          this->range_noise_covariance);
      kf::noise::NoiseBasePtr<2,double,Eigen::RowMajor> feature_bearing_noise =
        std::make_shared<kf::noise::NormalDistribution<2,true,true,false,double,Eigen::RowMajor>>(
          Eigen::Matrix<double,1,2,Eigen::RowMajor>::Zero(),
          this->bearing_noise_covariance);

      // Make bearing sensor model
      kf::sensors::FeatureBearing<FeatureDim,true,true,double,Eigen::RowMajor> feature_bearing_model(
        std::numeric_limits<double>::quiet_NaN(),
        this->camera_offset_vec,
        this->camera_viewing_angles,
        this->newest_buoy_state->radii[buoy_it],
        buoy_location);

      // Make the measurements
      out_msg.los[buoy_it].range          = feature_range_model.getMeasurement(0, uav_state, feature_range_noise->getNoise())[0];
      out_msg.los[buoy_it].range_variance = this->range_noise_covariance[0];

      Eigen::Map<Eigen::Matrix<double,1,2,Eigen::RowMajor>>(out_msg.los[buoy_it].bearing.data()) =
        feature_bearing_model.getMeasurement(0, uav_state, feature_bearing_noise->getNoise());
      Eigen::Map<Eigen::Matrix<double,2,2,Eigen::RowMajor>>(out_msg.los[buoy_it].bearing_covariance.data()) =
        this->bearing_noise_covariance;
    }
    else // Buoy not in range
    {
      out_msg.los[buoy_it].range_variance        = -1;
      out_msg.los[buoy_it].bearing_covariance[0] = -1;
    }
  }
  // If any buoys are in range, publish the sensor reading
  if(feature_in_range)
  {
    // Set message header
    out_msg.header.stamp    = pub_time;
    out_msg.header.frame_id = "feature_camera";

    this->los_pub->publish(out_msg);
  }
}
} // namespace skf

/* feature_sensor_node.cpp */
