# sim_kalman_filter
This package provides an implementation of a real-time Kalman filter that provides state estimates for the rest of the simulation.
It is implemented in ROS 2 Humble, but should work in other versions of ROS 2.

There are two nodes provided in this package, the `kalman_filter_node` and the `feature_sensor_node`.

## kalman_filter_node
This node implements a Kalman filter that is used thought the rest of the simulation to provide state estimates of the UAV.

### Parameters
| Parameter Name | Type | Description |
|----------------|------|-------------|
| `imu_topic` | String | The topic that the UAV's IMU readings are published on. |
| `gps_topic` | String | The topic that the UAV's GPS readings are published on. |
| `pressure_topic` | String | The topic that the UAV's pressure sensor readings are published on. |
| `compass_topic` | String | The topic that the UAV's digital compass readings are published on. |
| `feature_los_topic` | String | The topic that the UAV's feature range/bearing readings are published on. |
| `truth_state_topic` | String | The topic that the UAV's truth state is published on. |
| `dynamics.gravity_mag` | Float | The magnitude of the acceleration felt by the UAV from gravity. |
| `dynamics.air_density` | Float | The air density in the area that the UAV is flying. |
| `start_pause_srv_topic` | String | The topic that the toggle simulation execution service will be provided on. |
| `reset_srv_topic` | String | The topic that the simulation's reset service will be provided on. |
| `state_est_topic` | String | The topic that the this node will publish the UAV state estimate on. |
| `state_est_pub_frequency` | String | The frequency that this node will publish UAV state estimates at. |
| `integration_time_step` | Float | The time step used when integrating the dynamics forward in time. |
| `feature_locations_x` | Float Array | The north component of each feature's location, in the order that the feature appear in the LOS message. |
| `feature_locations_y` | Float Array | The east component of each feature's location, in the order that the feature appear in the LOS message. |
| `feature_locations_z` | Float Array | The down component of each feature's location, in the order that the feature appear in the LOS message. |
| `feature_camera_offset_vec` | Float Array | A three element vector that describes the position of the UAV's camera in the UAV's body frame. |
| `feature_camera_viewing_angles` | Float Array | A three element vector of roll, pitch, and yaw that represents the rotation from the UAV's body frame to the camera's frame. |
| `heading_bias_bounds` | Float | The absolute bounds on the heading sensor's bias. |
| `abs_pressure_bias_bounds` | Float | The absolute bounds on the absolute pressure sensor's bias. |
| `feature_range_bias_bounds` | Float | The absolute bounds on the feature range sensor's bias. |
| `feature_bearing_bias_bounds` | Float Array | The three element vector of absolute bounds on the feature bearing sensor's bias. |
| `gps_position_bias_bounds` | Float Array | The three element vector of absolute bounds on the GPS's position measurement bias. |
| `gyroscope_bias_bounds` | Float Array | The three element vector of absolute bounds on the gyroscope's measurement bias. |
| `accelerometer_bias_bounds` | Float Array | The three element vector of absolute bounds on the accelerometer's measurement bias. |

### Interfaces
| Topic | Interface Type | Direction | Message Type | Description |
|-------|----------------|-----------|--------------|-------------|
| The value of `truth_state_topic` | msg | Input | `uav_interfaces/UavState` | The UAV's truth state. |
| `/diagnostics` | msg | Output | `diagnostic_msgs/DiagnosticArray` | Used to publish diagnostic information as the node runs. |
| The value of `imu_topic` | msg | Input | `sensor_msgs/Imu` | The UAV's IMU readings. |
| The value of `gps_topic` | msg | Input | `uav_interfaces/Gps` | The UAV's GPS readings. |
| The value of `pressure_topic` | msg | Input | `uav_interfaces/Pressure` | The UAV's pressure sensor's readings. |
| The value of `compass_topic` | msg | Input | `uav_interfaces/Compass` | The UAV's digital compass's readings. |
| The value of `feature_los_topic` | msg | Input | `uav_interfaces/LineOfSightArray` | The UAV's feature range/bearing readings. |
| The value of `start_pause_srv_topic` | srv | Client | `std_srvs/Empty` | The simulation's toggle paused/start service. |
| The value of `reset_srv_topic` | srv | Client | `std_srvs/Empty` | The simulation's reset service. |
| The value of `state_est_topic` | msg | Output | `uav_interfaces\UavState` | The state estimate of the UAV. |

## feature_sensor_node
This node is used to publish feature sensor readings for the Kalman filter to use in it's estimation.
The two feature sensors implemented are range and bearing.
The range sensor gives the distance between the feature and the UAV.
The bearing sensor gives the line of sight vector from the UAV's camera to the feature, projected onto the focal plane of the camera.

### Parameters
| Parameter Name | Type | Description |
|----------------|------|-------------|
| `uav_state_topic` | String | The topic that the UAV's truth state is published on. |
| `buoy_state_topic` | String | The topic that the buoy's truth state is published on. |
| `los_topic` | String | The topic that this node will publish line of sight readings one. |
| `los_pub_frequency` | Float | The frequency that this node will attempt to publish line of sights reading at. |
| `camera_offset_vec` | Float Array | A three element vector that describes the position of the UAV's camera in the UAV's body frame. |
| `camera_viewing_angles` | Float Array | A three element vector of roll, pitch, and yaw that represents the rotation from the UAV's body frame to the camera's frame. |
| `range_noise_covariance` | Float | The covariance of the additive nose on the range measurements. |
| `bearing_noise_covariance` | Float Array | The four element covariance of the additive noise on the bearing measurements. |

### Interfaces
| Topic | Interface Type | Direction | Message Type | Description |
|-------|----------------|-----------|--------------|-------------|
| The value of `uav_state_topic` | msg | Input | `uav_interfaces/UavState` | The UAV's truth state. |
| The value of `buoy_state_topic` | msg | Input | `uav_interfaces/BuoyPoseRadiusArray` | The true positions and visibility radii of the features. |
| The value of `los_topic` | msg | Output | `uav_interfaces/LineOfSightArray` | The feature range and bearing measurements. |

