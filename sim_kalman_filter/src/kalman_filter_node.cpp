/**
 * @File: kalman_filter_node.cpp
 * @Date: September 2022
 * @Author: James Swedeen
 *
 * @brief
 * Runs the kalman filter node for the UAV simulation.
 **/

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<sim_kalman_filter/kalman_filter_node.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<skf::KalmanFilterNode<kf::Versions::RK4_INTEGRATION>> node =
    std::make_shared<skf::KalmanFilterNode<kf::Versions::RK4_INTEGRATION>>();
  node->finishSetup();
  rclcpp::spin(node);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}

/* kalman_filter_node.cpp */
