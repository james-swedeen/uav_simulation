/**
 * @File: feature_sensor_main.cpp
 * @Date: October 2022
 * @Author: James Swedeen
 *
 * @brief
 * Simulates the feature sensors.
 **/

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<sim_kalman_filter/feature_sensor_node.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<skf::FeatureSensorNode>());
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}

/* feature_sensor_main.cpp */
