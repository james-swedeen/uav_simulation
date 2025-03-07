"""This module defines the parameters for trajectory planning
"""
import pd_planner_launch.params.sensors as sensors
from typing import Optional

class KalmanFilter():
    """Parameters used for the Kalman filter"""
    def __init__(self) -> None:
        self.state_est_pub_freq = 100. # The frequency that this node will publish UAV state estimates at.
        self.integration_time_step = 0.001 # The time step used when integrating the dynamics forward in time.
        self.sensor_history_length_sec = 15.0 # All measurements that are older then this are discarded from the sensor history

        # Feature sensor parameters
        self.camera_offset_vec      = [0.5, 0.0, 0.25] # A three element vector that describes the position of the UAV's camera in the UAV's body frame.
        self.camera_viewing_angles  = [0.0, 0.0, 0.0] # A three element vector of roll, pitch, and yaw that represents the rotation from the UAV's body frame to the camera's frame.

        # Bias std deviations
        self.compass_bias_std = [0.0001]
        self.abs_press_bias_std = [0.0001]
        self.feature_range_bias_std = [0.0001]
        self.feature_bearing_bias_std = [0.5, 0.5, 0.5]
        self.gps_pos_bias_std = [2.167948339, 2.167948339, 3.033150178]
        self.accel_bias_std = [3.27E-04, 3.27E-04, 3.27E-04]
        self.gyro_bias_std = [1.6160456E-06, 1.6160456E-06, 1.6160456E-06]

        # Bounds on biases
        self.heading_bias_bounds        = 0.1 # The absolute bounds on the heading sensor's bias
        self.abs_pressure_bias_bounds   = 0.1 # The absolute bounds on the absolute pressure sensor's bias.
        self.feature_range_bias_bounds  = 10. # The absolute bounds on the feature range sensor's bias.
        self.feature_bearing_bias_bounds= [0.1 for i in range(3)] # The three element vector of absolute bounds on the feature bearing sensor's bias.
        self.gps_position_bias_bounds   = [10. for i in range(3)] # The three element vector of absolute bounds on the GPS's position measurement bias.
        self.gyroscope_bias_bounds      = [1.e-3 for i in range(3)] # The three element vector of absolute bounds on the gyroscope's measurement bias.
        self.accelerometer_bias_bounds  = [1.e-3 for i in range(3)] # The three element vector of absolute bounds on the accelerometer's measurement bias.

        # Bias time constants - assumes FOGM process for bias (seconds)
        self.compass_bias_tc            = [600.0]
        self.abs_press_bias_tc          = [60.0]
        self.feature_range_bias_tc      = [600.0]
        self.feature_bearing_bias_tc    = [3600.0, 3600.0, 3600.0]
        self.gps_pos_bias_tc            = [60.0, 60.0, 60.0]
        self.accel_bias_tc              = [60.0, 60.0, 60.0]
        self.gyro_bias_tc               = [60.0, 60.0, 60.0]


    def params_from_sensors(self,   imu: Optional[sensors.Imu] = None,
                                    feat: Optional[sensors.Feature] = None,
                                    compass: Optional[sensors.Compass] = None,
                                    press: Optional[sensors.Pressure] = None,
                                    gps: Optional[sensors.Gps] = None
    ) -> None:
        """This function imports the parameters from each sensor that is provided
        """
        if imu is not None:
            self.accel_bias_std =   imu.accel_bias_std
            self.gyro_bias_std  =   imu.gyro_bias_std
            self.gyroscope_bias_bounds     = imu.gyroscope_bias_bounds
            self.accelerometer_bias_bounds = imu.accelerometer_bias_bounds
            self.accel_bias_tc = imu.accel_bias_tc
            self.gyro_bias_tc  = imu.gyro_bias_tc

        if feat is not None:
            self.camera_offset_vec     = feat.camera_offset_vec
            self.camera_viewing_angles = feat.camera_viewing_angles
            self.feature_range_bias_std  = feat.range_bias_std
            self.feature_bearing_bias_std= feat.bearing_bias_std
            self.feature_range_bias_bounds   = feat.range_bias_bounds
            self.feature_bearing_bias_bounds = feat.bearing_bias_bounds
            self.feature_range_bias_tc   = feat.range_bias_tc
            self.feature_bearing_bias_tc = feat.bearing_bias_tc

        if compass is not None:
            self.compass_bias_std = compass.bias_std
            self.heading_bias_bounds = compass.bias_bounds
            self.compass_bias_tc = compass.bias_tc

        if press is not None:
            self.abs_press_bias_std = press.abs_press_bias_std
            self.abs_pressure_bias_bounds = press.abs_pressure_bias_bounds
            self.abs_press_bias_tc = press.abs_press_bias_tc

        if gps is not None:
            self.gps_pos_bias_std         = gps.pos_bias_std
            self.gps_position_bias_bounds = gps.position_bias_bounds
            self.gps_pos_bias_tc          = gps.pos_bias_tc
