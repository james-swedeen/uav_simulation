/**
 * @File: feature_sensor_node.hpp
 * @Date: October 2022
 * @Author: James Swedeen
 *
 * @brief
 * Subscribes to UAV state and buoy states and publishes line of sight measurements.
 **/

#ifndef SIM_KALMAN_FILTER_FEATURE_SENSOR_NODE_HPP
#define SIM_KALMAN_FILTER_FEATURE_SENSOR_NODE_HPP

/* ROS Headers */
#include<tf2/LinearMath/Quaternion.h>
#include<tf2/LinearMath/Matrix3x3.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include<rclcpp/rclcpp.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Our Headers */
#include<uav_interfaces/msg/uav_state.hpp>
#include<uav_interfaces/msg/buoy_pose_radius_array.hpp>
#include<uav_interfaces/msg/line_of_sight_array.hpp>

/* Kalman Filter Headers */
#include<kalman_filter/helpers/dimension_struct.hpp>
#include<kalman_filter/math/quaternion.hpp>

namespace skf
{
class FeatureSensorNode
 : public rclcpp::Node
{
public:
  /**
   * @Default Constructor
   *
   * @brief
   * Uses ROS parameters to initialize the node internally.
   **/
  FeatureSensorNode();
  /**
   * @Copy Constructor
   **/
  FeatureSensorNode(const FeatureSensorNode&) = delete;
  /**
   * @Move Constructor
   **/
  FeatureSensorNode(FeatureSensorNode&&) = delete;
  /**
   * @Deconstructor
   **/
  ~FeatureSensorNode() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  FeatureSensorNode& operator=(const FeatureSensorNode&) = delete;
  FeatureSensorNode& operator=(FeatureSensorNode&&)      = delete;
private:
  struct FeatureDim
   : public kf::Dimensions<0,11,11,0,10,10,0,0,0,0,false>
  {
  public:
    struct TRUTH
    {
    public:
      inline static constexpr const Eigen::Index NORTH_IND                      = 0; // The north component of the UAV's position in NED
      inline static constexpr const Eigen::Index EAST_IND                       = 1; // The east component of the UAV's position in NED
      inline static constexpr const Eigen::Index DOWN_IND                       = 2; // The down component of the UAV's position in NED
      inline static constexpr const Eigen::Index QUAT_W_IND                     = 3; // The w component of the quaternion that goes from the UAV's body frame to the NED frame
      inline static constexpr const Eigen::Index QUAT_X_IND                     = 4; // The x component of the quaternion that goes from the UAV's body frame to the NED frame
      inline static constexpr const Eigen::Index QUAT_Y_IND                     = 5; // The y component of the quaternion that goes from the UAV's body frame to the NED frame
      inline static constexpr const Eigen::Index QUAT_Z_IND                     = 6; // The z component of the quaternion that goes from the UAV's body frame to the NED frame
      inline static constexpr const Eigen::Index FEATURE_RANGE_BIAS_IND         = 7; // The feature range sensor's bias term
      inline static constexpr const Eigen::Index FEATURE_BEARING_ROLL_BIAS_IND  = 8; // The roll component of the feature bearing sensor's bias term
      inline static constexpr const Eigen::Index FEATURE_BEARING_PITCH_BIAS_IND = 9; // The pitch component of the feature bearing sensor's bias term
      inline static constexpr const Eigen::Index FEATURE_BEARING_YAW_BIAS_IND   = 10; // The yaw component of the feature bearing sensor's bias term

      inline static constexpr const Eigen::Index POS_START_IND                  = NORTH_IND;
      inline static constexpr const Eigen::Index POS_END_IND                    = DOWN_IND;
      inline static constexpr const Eigen::Index QUAT_START_IND                 = QUAT_W_IND;
      inline static constexpr const Eigen::Index QUAT_END_IND                   = QUAT_Z_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_START_IND = FEATURE_BEARING_ROLL_BIAS_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_END_IND   = FEATURE_BEARING_YAW_BIAS_IND;
    };
    struct NAV
    {
    public:
      inline static constexpr const Eigen::Index NORTH_IND                      = 0; // The north component of the UAV's position in NED
      inline static constexpr const Eigen::Index EAST_IND                       = 1; // The east component of the UAV's position in NED
      inline static constexpr const Eigen::Index DOWN_IND                       = 2; // The down component of the UAV's position in NED
      inline static constexpr const Eigen::Index QUAT_W_IND                     = 3; // The w component of the quaternion that goes from the UAV's body frame to the NED frame
      inline static constexpr const Eigen::Index QUAT_X_IND                     = 4; // The x component of the quaternion that goes from the UAV's body frame to the NED frame
      inline static constexpr const Eigen::Index QUAT_Y_IND                     = 5; // The y component of the quaternion that goes from the UAV's body frame to the NED frame
      inline static constexpr const Eigen::Index QUAT_Z_IND                     = 6; // The z component of the quaternion that goes from the UAV's body frame to the NED frame
      inline static constexpr const Eigen::Index FEATURE_RANGE_BIAS_IND         = 7; // The feature range sensor's bias term
      inline static constexpr const Eigen::Index FEATURE_BEARING_ROLL_BIAS_IND  = 8; // The roll component of the feature bearing sensor's bias term
      inline static constexpr const Eigen::Index FEATURE_BEARING_PITCH_BIAS_IND = 9; // The pitch component of the feature bearing sensor's bias term
      inline static constexpr const Eigen::Index FEATURE_BEARING_YAW_BIAS_IND   = 10; // The yaw component of the feature bearing sensor's bias term

      inline static constexpr const Eigen::Index POS_START_IND                  = NORTH_IND;
      inline static constexpr const Eigen::Index POS_END_IND                    = DOWN_IND;
      inline static constexpr const Eigen::Index QUAT_START_IND                 = QUAT_W_IND;
      inline static constexpr const Eigen::Index QUAT_END_IND                   = QUAT_Z_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_START_IND = FEATURE_BEARING_ROLL_BIAS_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_END_IND   = FEATURE_BEARING_YAW_BIAS_IND;
    };
    struct ERROR
    {
    public:
      inline static constexpr const Eigen::Index NORTH_IND                      = 0; // The north component of the UAV's position in NED
      inline static constexpr const Eigen::Index EAST_IND                       = 1; // The east component of the UAV's position in NED
      inline static constexpr const Eigen::Index DOWN_IND                       = 2; // The down component of the UAV's position in NED
      inline static constexpr const Eigen::Index ROLL_IND                       = 3; // The roll component of the UAV's attitude error
      inline static constexpr const Eigen::Index PITCH_IND                      = 4; // The pitch component of the UAV's attitude error
      inline static constexpr const Eigen::Index YAW_IND                        = 5; // The yaw component of the UAV's attitude error
      inline static constexpr const Eigen::Index FEATURE_RANGE_BIAS_IND         = 6; // The feature range sensor's bias term
      inline static constexpr const Eigen::Index FEATURE_BEARING_ROLL_BIAS_IND  = 7; // The roll component of the feature bearing sensor's bias term
      inline static constexpr const Eigen::Index FEATURE_BEARING_PITCH_BIAS_IND = 8; // The pitch component of the feature bearing sensor's bias term
      inline static constexpr const Eigen::Index FEATURE_BEARING_YAW_BIAS_IND   = 9; // The yaw component of the feature bearing sensor's bias term

      inline static constexpr const Eigen::Index POS_START_IND                  = NORTH_IND;
      inline static constexpr const Eigen::Index POS_END_IND                    = DOWN_IND;
      inline static constexpr const Eigen::Index EULER_START_IND                = ROLL_IND;
      inline static constexpr const Eigen::Index EULER_END_IND                  = YAW_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_START_IND = FEATURE_BEARING_ROLL_BIAS_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_END_IND   = FEATURE_BEARING_YAW_BIAS_IND;
    };
    struct TRUTH_DISP
    {
    public:
      inline static constexpr const Eigen::Index NORTH_IND                      = 0; // The north component of the UAV's position in NED
      inline static constexpr const Eigen::Index EAST_IND                       = 1; // The east component of the UAV's position in NED
      inline static constexpr const Eigen::Index DOWN_IND                       = 2; // The down component of the UAV's position in NED
      inline static constexpr const Eigen::Index ROLL_IND                       = 3; // The roll component of the UAV's attitude error
      inline static constexpr const Eigen::Index PITCH_IND                      = 4; // The pitch component of the UAV's attitude error
      inline static constexpr const Eigen::Index YAW_IND                        = 5; // The yaw component of the UAV's attitude error
      inline static constexpr const Eigen::Index FEATURE_RANGE_BIAS_IND         = 6; // The feature range sensor's bias term
      inline static constexpr const Eigen::Index FEATURE_BEARING_ROLL_BIAS_IND  = 7; // The roll component of the feature bearing sensor's bias term
      inline static constexpr const Eigen::Index FEATURE_BEARING_PITCH_BIAS_IND = 8; // The pitch component of the feature bearing sensor's bias term
      inline static constexpr const Eigen::Index FEATURE_BEARING_YAW_BIAS_IND   = 9; // The yaw component of the feature bearing sensor's bias term

      inline static constexpr const Eigen::Index POS_START_IND                  = NORTH_IND;
      inline static constexpr const Eigen::Index POS_END_IND                    = DOWN_IND;
      inline static constexpr const Eigen::Index EULER_START_IND                = ROLL_IND;
      inline static constexpr const Eigen::Index EULER_END_IND                  = YAW_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_START_IND = FEATURE_BEARING_ROLL_BIAS_IND;
      inline static constexpr const Eigen::Index FEATURE_BEARING_BIAS_END_IND   = FEATURE_BEARING_YAW_BIAS_IND;
    };
  };

  // UAV state info
  uav_interfaces::msg::UavState::ConstSharedPtr                  newest_uav_state; // The newest truth state message received
  rclcpp::Subscription<uav_interfaces::msg::UavState>::SharedPtr uav_state_sub;
  // Buoy state info
  uav_interfaces::msg::BuoyPoseRadiusArray::ConstSharedPtr                  newest_buoy_state; // The newest buoy state message received
  rclcpp::Subscription<uav_interfaces::msg::BuoyPoseRadiusArray>::SharedPtr buoy_state_sub;
  // Model info
  Eigen::Matrix<double,1,3,Eigen::RowMajor> camera_offset_vec; // Vector that goes from the UAV's body to the location of the camera in the UAV body frame
  Eigen::Matrix<double,1,3,Eigen::RowMajor> camera_viewing_angles; // The angle offset from the UAV body frame to the camera frame
  Eigen::Matrix<double,1,1,Eigen::RowMajor> range_noise_covariance; // The covariance of the range sensor's noise
  Eigen::Matrix<double,2,2,Eigen::RowMajor> bearing_noise_covariance; // The covariance of the bearing sensor's noise
  // Sensor publishing
  rclcpp::Publisher<uav_interfaces::msg::LineOfSightArray>::SharedPtr los_pub;
  rclcpp::TimerBase::SharedPtr                                        los_pub_loop;
  /**
   * @uavStateCallback
   *
   * @brief
   * Used to read in the UAV state.
   **/
  void uavStateCallback(const uav_interfaces::msg::UavState::ConstSharedPtr& uav_state_msg);
  /**
   * @buoyStateCallback
   *
   * @brief
   * Used to read in the buoy state.
   **/
  void buoyStateCallback(const uav_interfaces::msg::BuoyPoseRadiusArray::ConstSharedPtr& buoy_state_msg);
  /**
   * @losPublishingCallback
   *
   * @brief
   * Publishes the LOS data.
   **/
  void losPublishingCallback();
  /**
   * @getUAVState
   *
   * @brief
   * Gets the UAV state as a Eigen vector.
   **/
  inline Eigen::Matrix<double,1,11,Eigen::RowMajor> getUavState() const;
};


/**
 * @getUAVState
 *
 * @brief
 * Gets the UAV state as a Eigen vector.
 **/
inline Eigen::Matrix<double,1,11,Eigen::RowMajor> FeatureSensorNode::getUavState() const
{
  Eigen::Matrix<double,1,11,Eigen::RowMajor> output;

  output[FeatureDim::TRUTH::NORTH_IND] = this->newest_uav_state->pose.pose.position.x;
  output[FeatureDim::TRUTH::EAST_IND]  = this->newest_uav_state->pose.pose.position.y;
  output[FeatureDim::TRUTH::DOWN_IND]  = this->newest_uav_state->pose.pose.position.z;
  {
    Eigen::Matrix<double,1,1,Eigen::RowMajor> roll, pitch, yaw;
    tf2::Quaternion ros_quat;
    tf2::fromMsg(this->newest_uav_state->pose.pose.orientation, ros_quat);
    tf2::Matrix3x3  ros_rot(ros_quat);
    ros_rot.getRPY(roll[0], pitch[0], yaw[0]);

    output.template middleCols<4>(FeatureDim::TRUTH::QUAT_W_IND) =
      kf::math::quat::normalize(kf::math::quat::rollPitchYawToQuaternion(roll, pitch, yaw));
  }
  // TODO: Need to get biases from message
  output[FeatureDim::TRUTH::FEATURE_RANGE_BIAS_IND] = 0;
  output.template middleCols<3>(FeatureDim::TRUTH::FEATURE_BEARING_ROLL_BIAS_IND).setZero();

  return output;
}
} // namespace skf

#endif
/* feature_sensor_node.hpp */
