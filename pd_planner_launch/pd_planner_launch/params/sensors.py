"""This module defines the parameters for various sensors
"""
import numpy as np

######################################
############  IMU  ###################
######################################
class Imu():
    """Basic parameters for an IMU"""
    def __init__(self) -> None:
        self.accel_sigma = 250*(10**-6)*9.81  # standard deviation of accelerometers in m/s^2
        self.gyro_sigma  = float(np.radians(0.015))  # standard deviation of gyros in rad/sec
        self.gyro_x_bias = 0.#np.radians(5*np.random.uniform(-1, 1))  # bias on x_gyro
        self.gyro_y_bias = 0.#np.radians(5*np.random.uniform(-1, 1))  # bias on y_gyro
        self.gyro_z_bias = 0.#np.radians(5*np.random.uniform(-1, 1))  # bias on z_gyro

        # Bias standard deviations
        self.accel_bias_std = [3.27E-04, 3.27E-04, 3.27E-04]
        self.gyro_bias_std = [1.6160456E-06, 1.6160456E-06, 1.6160456E-06]
        self.gyroscope_bias_bounds      = [1.e-3 for i in range(3)] # The three element vector of absolute bounds on the gyroscope's measurement bias.
        self.accelerometer_bias_bounds  = [1.e-3 for i in range(3)] # The three element vector of absolute bounds on the accelerometer's measurement bias.
        self.accel_bias_tc              = [60.0, 60.0, 60.0] # FOGM process time constant for bias (seconds)
        self.gyro_bias_tc               = [60.0, 60.0, 60.0] # FOGM process time constant for bias (seconds)

######################################
############  Magnetometer  ##########
######################################
class Magnetometer():
    """Parameters for a Magnetometer sensor"""
    def __init__(self) -> None:
        self.sigma = float(np.radians(0.03)) # Standard deviation of the magnetic sensor measurement
        self.dec = float(np.radians(12.5)) # Declination of the magnetic to inertial frame
        self.inc = float(np.radians(-66)) # Inclination of the magnetic to inertial frame


######################################
############  Pressure  ##############
######################################
class Pressure():
    """Parameters for a pressure sensor"""
    def __init__(self) -> None:
        self.abs_bias    = 0. # Bias on the absolute pressure measurement
        self.abs_sigma   = 0.01*1000  # standard deviation of absolute pressure sensors in Pascals
        self.diff_bias   = 0. # Bias on the differential pressure
        self.diff_sigma  = 0.002*1000.  # standard deviation of diff pressure sensor in Pascals

        # Bias parameters
        self.abs_press_bias_std = [0.0001] # Standard deviation for the absolute pressure bias
        self.abs_pressure_bias_bounds   = 0.1 # The absolute bounds on the absolute pressure sensor's bias.
        self.abs_press_bias_tc = [60.0] # FOGM process time constant for bias (seconds)


######################################
############  GPS  ###################
######################################
class Gps():
    """Parameters for a GPS sensor"""
    def __init__(self) -> None:
        self.period          = 0.2 # Period (sec) at which gps is published
        self.k               = 1. / 1100.  # 1 / s - time constant of the process
        self.n_sigma         = 0.05 # Standard deviation of the north gps measurement
        self.e_sigma         = 0.05 # Standard deviation of the east gps measurement
        self.h_sigma         = 0.15 # Standard deviation of the height measurement
        self.Vg_sigma        = 0.005 # Standard deviation of the ground velocity measurement
        self.course_sigma    = float(self.Vg_sigma / 20.) # Standard deviation of the course angle measurement

        # Bias terms
        self.pos_bias_std           = [2.167948339, 2.167948339, 3.033150178] # Standard deviation in bias
        self.position_bias_bounds   = [10. for i in range(3)] # The three element vector of absolute bounds on the GPS's position measurement bias.
        self.pos_bias_tc            = [60.0, 60.0, 60.0] # FOGM process time constant for bias (seconds)


######################################
############  Feature ################
######################################
class Feature():
    """Parameters for sensing a feature using range and bearing """
    def __init__(self) -> None:
        self.los_pub_frequency       = 2.0
        self.camera_offset_vec       = [0.5, 0.0, 0.25] # A three element vector that describes the position of the UAV's camera in the UAV's body frame.
        self.camera_viewing_angles   = [0.0, 0.0, 0.0] # A three element vector of roll, pitch, and yaw that represents the rotation from the UAV's body frame to the camera's frame.
        self.range_noise_sigma       = 0.00001**2 # The covariance of the additive nose on the range measurements.
        self.bearing_noise_sigma     = [0.0001**2, 0.0, 0.0001**2, 0.0] # The four element covariance of the additive noise on the bearing measurements.

        # Bias terms
        self.range_bias_std     = [0.0001] # Standard deviation of the feature range measurement bias
        self.bearing_bias_std   = [0.5, 0.5, 0.5] # Standard deviation of the feature range measurement bias
        self.range_bias_bounds  = 10. # The absolute bounds on the feature range sensor's bias.
        self.bearing_bias_bounds= [0.1 for i in range(3)] # The three element vector of absolute bounds on the feature bearing sensor's bias.
        self.range_bias_tc      = [600.0] # FOGM process time constant for bias (seconds)
        self.bearing_bias_tc    = [3600.0, 3600.0, 3600.0] # FOGM process time constant for bias (seconds)

######################################
###########  Compass #################
######################################
class Compass():
    """Parameters for the digital compass sensor"""
    def __init__(self) -> None:
        self.sigma = 0.1 # Standard deviation of the compass measurement noise
        self.bias = 0.0 # Compass bias
        self.bias_std = [0.0001] # Standard deviation in the compass bias
        self.bias_bounds = 0.1 # The absolute bounds on the heading sensor's bias
        self.bias_tc = [600.0] # FOGM process time constant for bias (seconds)
