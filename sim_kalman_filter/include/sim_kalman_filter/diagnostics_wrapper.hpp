/**
 * @File: diagnostics_wrapper.hpp
 * @Date: October 2022
 * @Author: James Swedeen
 *
 * @brief
 * A wrapper class that proves functionality for publishing diagnostic messages.
 **/

#ifndef SIM_KALMAN_FILTER_DIAGNOSTICS_WRAPPER_HPP
#define SIM_KALMAN_FILTER_DIAGNOSTICS_WRAPPER_HPP

/* C++ Headers */
#include<string>
#include<chrono>
#include<mutex>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>
#include<diagnostic_msgs/msg/diagnostic_status.hpp>
#include<diagnostic_msgs/msg/diagnostic_array.hpp>

namespace skf
{
class DiagnosticsWrapper
{
public:
  /**
   * @Default Constructor
   **/
  DiagnosticsWrapper() = delete;
  /**
   * @Copy Constructor
   **/
  DiagnosticsWrapper(const DiagnosticsWrapper&) = delete;
  /**
   * @Move Constructor
   **/
  DiagnosticsWrapper(DiagnosticsWrapper&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * node: The node that this class is providing diagnostic functionality for
   * name: The name that will appear in the diagnostics window for this node
   * publishing_period: How long to wait between diagnostic publications
   * diagnostics_topic: The topic to publish diagnostic info on
   * hardware_id: The ID of the hardware being used
   **/
  template<typename DUR_REP = int64_t, typename DUR_T = std::ratio<1>>
  DiagnosticsWrapper(const rclcpp::Node::SharedPtr&              node,
                     const std::string&                          name,
                     const std::chrono::duration<DUR_REP,DUR_T>& publishing_period = std::chrono::duration<int64_t,std::ratio<1>>(1),
                     const std::string&                          diagnostics_topic = "/diagnostics",
                     const std::string&                          hardware_id = "sim");
  /**
   * @Deconstructor
   **/
  virtual ~DiagnosticsWrapper() noexcept = default;
  /**
   * @Assignment Operators
   **/
  DiagnosticsWrapper& operator=(const DiagnosticsWrapper&) = delete;
  DiagnosticsWrapper& operator=(DiagnosticsWrapper&&)      = delete;
  /**
   * @logDiagnostic
   *
   * @brief
   * Used to update the diagnostic data that this node is publishing.
   *
   * @parameters
   * level: The level of operation for the diagnostic
   * message: The message to send with the diagnostic
   * values: An array of values associated with the status
   **/
  inline void logDiagnostic(const diagnostic_msgs::msg::DiagnosticStatus::_level_type&  level,
                            const std::string&                                          message,
                            const diagnostic_msgs::msg::DiagnosticStatus::_values_type& values = diagnostic_msgs::msg::DiagnosticStatus::_values_type());
private:
  rclcpp::Node::SharedPtr                                             node; // Used to find the current time
  rclcpp::CallbackGroup::SharedPtr                                    callback_group;
  std::mutex                                                          pub_mux; // Prevents race conditions
  diagnostic_msgs::msg::DiagnosticArray                               output_msg; // Holds messages that will be published
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub;
  rclcpp::TimerBase::SharedPtr                                        diag_pub_loop;
  /**
   * @pubLoop
   *
   * @brief
   * Loop that periodically publishes the diagnostic status.
   **/
  void pubLoop();
};


/**
 * @Constructor
 *
 * @brief
 * Initializes the object for use.
 *
 * @parameters
 * node: The node that this class is providing diagnostic functionality for
 * name: The name that will appear in the diagnostics window for this node
 * publishing_period: How long to wait between diagnostic publications
 * diagnostics_topic: The topic to publish diagnostic info on
 * hardware_id: The ID of the hardware being used
 **/
template<typename DUR_REP, typename DUR_T>
DiagnosticsWrapper::DiagnosticsWrapper(const rclcpp::Node::SharedPtr&              node,
                                       const std::string&                          name,
                                       const std::chrono::duration<DUR_REP,DUR_T>& publishing_period,
                                       const std::string&                          diagnostics_topic,
                                       const std::string&                          hardware_id)
 : node(node),
   callback_group(node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
  rclcpp::PublisherOptions pub_options;
  pub_options.callback_group = this->callback_group;
  // Initialize output buffer
  this->output_msg.status.resize(1);
  this->output_msg.status.front().level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  this->output_msg.status.front().name = name;
  this->output_msg.status.front().hardware_id = hardware_id;
  // Start publisher
  this->diag_pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic,
                                                                                 rclcpp::SystemDefaultsQoS(),
                                                                                 pub_options);
  this->diag_pub_loop = rclcpp::create_timer(node,
                                             node->get_clock(),
                                             publishing_period,
                                             std::bind(&DiagnosticsWrapper::pubLoop, this),
                                             this->callback_group);
}

/**
 * @logDiagnostic
 *
 * @brief
 * Used to update the diagnostic data that this node is publishing.
 *
 * @parameters
 * level: The level of operation for the diagnostic
 * message: The message to send with the diagnostic
 * values: An array of values associated with the status
 **/
inline void DiagnosticsWrapper::logDiagnostic(const diagnostic_msgs::msg::DiagnosticStatus::_level_type&  level,
                                              const std::string&                                          message,
                                              const diagnostic_msgs::msg::DiagnosticStatus::_values_type& values)
{
  std::lock_guard<std::mutex> lock(this->pub_mux);
  // Copy information into local buffer
  this->output_msg.header.stamp           = this->node->now();
  this->output_msg.status.front().level   = level;
  this->output_msg.status.front().message = message;
  this->output_msg.status.front().values  = values;
}
} // namespace skf

#endif
/* diagnostics_wrapper.hpp */
