/**
 * @File: kalman_filter_state.hpp
 * @Date: September 2022
 * @Author: James Swedeen
 *
 * @brief
 * Holds the needed information about the state of the kalman filter at the time of each measurement.
 **/

#ifndef SIM_KALMAN_FILTER_KALMAN_FILTER_STATE_HPP
#define SIM_KALMAN_FILTER_KALMAN_FILTER_STATE_HPP

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Our Headers */
#include<sensor_msgs/msg/imu.hpp>
#include<uav_interfaces/msg/gps.hpp>
#include<uav_interfaces/msg/pressure.hpp>
#include<uav_interfaces/msg/magnetometer.hpp>
#include<uav_interfaces/msg/compass.hpp>
#include<uav_interfaces/msg/line_of_sight_array.hpp>

namespace skf
{
enum class MeasurementType : uint8_t
{
  DEFAULT_TYPE  = 0, // Explicitly means nothing
  INITIAL_STATE = 1, // This is the starting state estimate
  IMU           = 2, // This state holds a IMU reading
  GPS           = 3, // This state holds a GPS reading
  PRESSURE      = 4, // This state holds a pressure sensor reading
  MAGNETOMETER  = 5, // This state holds a magnetometer reading
  COMPASS       = 6, // This state holds a compass reading
  FEATURE_LOS   = 7  // This state holds a feature camera reading
};

template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class KalmanFilterState
{
public:
  /**
   * @Default Constructor
   **/
  KalmanFilterState() = delete;
  /**
   * @Copy Constructor
   **/
  KalmanFilterState(const KalmanFilterState&) noexcept = default;
  /**
   * @Move Constructor
   **/
  KalmanFilterState(KalmanFilterState&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Sets the internally held values with the values given.
   **/
  template<typename STATE_DERIVED, typename COV_DERIVED>
  inline explicit KalmanFilterState(const rclcpp::Time&                     initial_time,
                                    const Eigen::MatrixBase<STATE_DERIVED>& initial_state,
                                    const Eigen::MatrixBase<COV_DERIVED>&   initial_covariance) noexcept;

  inline explicit KalmanFilterState(const rclcpp::Time&                                          time,
                                    const sensor_msgs::msg::Imu::                ConstSharedPtr& imu_msg)          noexcept;
  inline explicit KalmanFilterState(const rclcpp::Time&                                          time,
                                    const uav_interfaces::msg::Gps::             ConstSharedPtr& gps_msg)          noexcept;
  inline explicit KalmanFilterState(const rclcpp::Time&                                          time,
                                    const uav_interfaces::msg::Pressure::        ConstSharedPtr& pressure_msg)     noexcept;
  inline explicit KalmanFilterState(const rclcpp::Time&                                          time,
                                    const uav_interfaces::msg::Magnetometer::    ConstSharedPtr& magnetometer_msg) noexcept;
  inline explicit KalmanFilterState(const rclcpp::Time&                                          time,
                                    const uav_interfaces::msg::Compass::         ConstSharedPtr& compass_msg)      noexcept;
  inline explicit KalmanFilterState(const rclcpp::Time&                                          time,
                                    const uav_interfaces::msg::LineOfSightArray::ConstSharedPtr& los_msg)          noexcept;
  /**
   * @Deconstructor
   **/
  ~KalmanFilterState() noexcept = default;
  /**
   * @Assignment Operators
   **/
  KalmanFilterState& operator=(const KalmanFilterState&) = delete;
  KalmanFilterState& operator=(KalmanFilterState&&)      = delete;
  /**
   * @getMeasurementType
   *
   * @brief
   * Used to get this state's measurement type.
   **/
  inline MeasurementType getMeasurementType() const noexcept;
  /**
   * @getTime
   *
   * @brief
   * Used to get this state's time.
   **/
  inline const rclcpp::Time& getTime() const noexcept;
  /**
   * @getMeasurement
   *
   * @brief
   * Gets the asked for measurement. Will throw an assert if asking for a measurement type that isn't this
   * objects measurement type.
   **/
  inline const sensor_msgs::   msg::Imu::             ConstSharedPtr& getIMUMeasurement()          const noexcept;
  inline const uav_interfaces::msg::Gps::             ConstSharedPtr& getGPSMeasurement()          const noexcept;
  inline const uav_interfaces::msg::Pressure::        ConstSharedPtr& getPressureMeasurement()     const noexcept;
  inline const uav_interfaces::msg::Magnetometer::    ConstSharedPtr& getMagnetometerMeasurement() const noexcept;
  inline const uav_interfaces::msg::Compass::         ConstSharedPtr& getCompassMeasurement()      const noexcept;
  inline const uav_interfaces::msg::LineOfSightArray::ConstSharedPtr& getLosMeasurement()          const noexcept;
  /**
   * @Comparison Operators
   *
   * @brief
   * Orders in terms of time (earlier is smaller). In the case of a tie the tie is broken using the order of the
   * measurement types.
   **/
  inline bool operator<(const KalmanFilterState& rhs) const noexcept;
  inline bool operator>(const KalmanFilterState& rhs) const noexcept;
  /**
   * @State Variables
   **/
  Eigen::Matrix<SCALAR,1,      STATE_DIM,OPTIONS> nav_state; // The navigation state at the time of this measurement
  Eigen::Matrix<SCALAR,COV_DIM,COV_DIM,  OPTIONS> covariance; // The error state covariance at the time of this measurement
private:
  const MeasurementType m_measurement_type; // What type of measurement this key correlates to
  const rclcpp::Time    m_time; // The time that the measurement was taken
  // Measurement Object
  const sensor_msgs::   msg::Imu::             ConstSharedPtr m_imu_msg;
  const uav_interfaces::msg::Gps::             ConstSharedPtr m_gps_msg;
  const uav_interfaces::msg::Pressure::        ConstSharedPtr m_pressure_msg;
  const uav_interfaces::msg::Magnetometer::    ConstSharedPtr m_magnetometer_msg;
  const uav_interfaces::msg::Compass::         ConstSharedPtr m_compass_msg;
  const uav_interfaces::msg::LineOfSightArray::ConstSharedPtr m_los_msg;
};


/**
 * @Constructor
 *
 * @brief
 * Sets the internally held values with the values given.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename STATE_DERIVED, typename COV_DERIVED>
inline KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::
  KalmanFilterState(const rclcpp::Time&                     initial_time,
                    const Eigen::MatrixBase<STATE_DERIVED>& initial_state,
                    const Eigen::MatrixBase<COV_DERIVED>&   initial_covariance) noexcept
 : nav_state(initial_state),
   covariance(initial_covariance),
   m_measurement_type(MeasurementType::INITIAL_STATE),
   m_time(initial_time)
{
  static_assert((int(STATE_DERIVED::RowsAtCompileTime) == 1)         or (int(STATE_DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(STATE_DERIVED::ColsAtCompileTime) == STATE_DIM) or (int(STATE_DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(COV_DERIVED::RowsAtCompileTime)   == COV_DIM)   or (int(COV_DERIVED::RowsAtCompileTime)   == Eigen::Dynamic));
  static_assert((int(COV_DERIVED::ColsAtCompileTime)   == COV_DIM)   or (int(COV_DERIVED::ColsAtCompileTime)   == Eigen::Dynamic));
  assert(initial_state.rows()      == 1);
  assert(initial_state.cols()      == STATE_DIM);
  assert(initial_covariance.rows() == COV_DIM);
  assert(initial_covariance.cols() == COV_DIM);
}

/**
 * @Constructor
 *
 * @brief
 * Sets the internally held values with the values given.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::
  KalmanFilterState(const rclcpp::Time&                          time,
                    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg) noexcept
 : m_measurement_type(MeasurementType::IMU),
   m_time(time),
   m_imu_msg(imu_msg)
{}

/**
 * @Constructor
 *
 * @brief
 * Sets the internally held values with the values given.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::
  KalmanFilterState(const rclcpp::Time&                             time,
                    const uav_interfaces::msg::Gps::ConstSharedPtr& gps_msg) noexcept
 : m_measurement_type(MeasurementType::GPS),
   m_time(time),
   m_gps_msg(gps_msg)
{}

/**
 * @Constructor
 *
 * @brief
 * Sets the internally held values with the values given.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::
  KalmanFilterState(const rclcpp::Time&                                  time,
                    const uav_interfaces::msg::Pressure::ConstSharedPtr& pressure_msg) noexcept
 : m_measurement_type(MeasurementType::PRESSURE),
   m_time(time),
   m_pressure_msg(pressure_msg)
{}

/**
 * @Constructor
 *
 * @brief
 * Sets the internally held values with the values given.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::
  KalmanFilterState(const rclcpp::Time&                                      time,
                    const uav_interfaces::msg::Magnetometer::ConstSharedPtr& magnetometer_msg) noexcept
 : m_measurement_type(MeasurementType::MAGNETOMETER),
   m_time(time),
   m_magnetometer_msg(magnetometer_msg)
{}

/**
 * @Constructor
 *
 * @brief
 * Sets the internally held values with the values given.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::
  KalmanFilterState(const rclcpp::Time&                                 time,
                    const uav_interfaces::msg::Compass::ConstSharedPtr& compass_msg) noexcept
 : m_measurement_type(MeasurementType::COMPASS),
   m_time(time),
   m_compass_msg(compass_msg)
{}

/**
 * @Constructor
 *
 * @brief
 * Sets the internally held values with the values given.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::
  KalmanFilterState(const rclcpp::Time&                                          time,
                    const uav_interfaces::msg::LineOfSightArray::ConstSharedPtr& los_msg) noexcept
 : m_measurement_type(MeasurementType::FEATURE_LOS),
   m_time(time),
   m_los_msg(los_msg)
{}

/**
 * @getMeasurementType
 *
 * @brief
 * Used to get this state's measurement type.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline MeasurementType KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::getMeasurementType() const noexcept
{
  return this->m_measurement_type;
}

/**
 * @getTime
 *
 * @brief
 * Used to get this state's time.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const rclcpp::Time& KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::getTime() const noexcept
{
  return this->m_time;
}

/**
 * @getMeasurement
 *
 * @brief
 * Gets the asked for measurement. Will throw an assert if asking for a measurement type that isn't this
 * objects measurement type.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const sensor_msgs::msg::Imu::ConstSharedPtr& KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::
  getIMUMeasurement() const noexcept
{
  assert(MeasurementType::IMU == this->getMeasurementType());
  return this->m_imu_msg;
}

/**
 * @getMeasurement
 *
 * @brief
 * Gets the asked for measurement. Will throw an assert if asking for a measurement type that isn't this
 * objects measurement type.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const uav_interfaces::msg::Gps::ConstSharedPtr& KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::
  getGPSMeasurement() const noexcept
{
  assert(MeasurementType::GPS == this->getMeasurementType());
  return this->m_gps_msg;
}

/**
 * @getMeasurement
 *
 * @brief
 * Gets the asked for measurement. Will throw an assert if asking for a measurement type that isn't this
 * objects measurement type.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const uav_interfaces::msg::Pressure::ConstSharedPtr& KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::
  getPressureMeasurement() const noexcept
{
  assert(MeasurementType::PRESSURE == this->getMeasurementType());
  return this->m_pressure_msg;
}

/**
 * @getMeasurement
 *
 * @brief
 * Gets the asked for measurement. Will throw an assert if asking for a measurement type that isn't this
 * objects measurement type.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const uav_interfaces::msg::Magnetometer::ConstSharedPtr& KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::
  getMagnetometerMeasurement() const noexcept
{
  assert(MeasurementType::MAGNETOMETER == this->getMeasurementType());
  return this->m_magnetometer_msg;
}

/**
 * @getMeasurement
 *
 * @brief
 * Gets the asked for measurement. Will throw an assert if asking for a measurement type that isn't this
 * objects measurement type.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const uav_interfaces::msg::Compass::ConstSharedPtr& KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::
  getCompassMeasurement() const noexcept
{
  assert(MeasurementType::COMPASS == this->getMeasurementType());
  return this->m_compass_msg;
}

/**
 * @getMeasurement
 *
 * @brief
 * Gets the asked for measurement. Will throw an assert if asking for a measurement type that isn't this
 * objects measurement type.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const uav_interfaces::msg::LineOfSightArray::ConstSharedPtr&
  KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::getLosMeasurement() const noexcept
{
  assert(MeasurementType::FEATURE_LOS == this->getMeasurementType());
  return this->m_los_msg;
}

/**
 * @Comparison Operators
 *
 * @brief
 * Orders in terms of time (earlier is smaller). In the case of a tie the tie is broken using the order of the
 * measurement types.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::
  operator<(const KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>& rhs) const noexcept
{
  if(this->getTime() < rhs.getTime())
  {
    return true;
  }
  if(this->getTime() == rhs.getTime())
  {
    return uint8_t(this->getMeasurementType()) < uint8_t(rhs.getMeasurementType());
  }
  return false;
}

/**
 * @Comparison Operators
 *
 * @brief
 * Orders in terms of time (earlier is smaller). In the case of a tie the tie is broken using the order of the
 * measurement types.
 **/
template<Eigen::Index STATE_DIM, Eigen::Index COV_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>::
  operator>(const KalmanFilterState<STATE_DIM,COV_DIM,SCALAR,OPTIONS>& rhs) const noexcept
{
  return not (*this < rhs);
}
} // namespace skf

#endif
/* kalman_filter_state.hpp */
