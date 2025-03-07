"""Defines parameters to be used in the simulation"""
from ament_index_python.packages import get_package_share_directory
import os
from pd_planner_launch.params.environment import Features, Radars, GpsDenied, PDVGWorld
from pd_planner_launch.params.filtering import KalmanFilter
from pd_planner_launch.params.sensors import Imu, Magnetometer, Pressure, Gps, Feature, Compass
from pd_planner_launch.params.planning import NominalTrajectory, WaypointPlanner, SimManager
from pd_planner_launch.params.csv_directory import StateListener

class ScenarioParameters():
    """ Defines all of the parameters used in the scenario.
    """
    def __init__(self) -> None:
        """ Defines all scenario parameters
        """
        # Define parameters that need to be initialize outside of the class
        # Note that these are not initialized here to ensure that the scenario parameters
        # are not dependent upon changes in choice of sensor or other parameter.
        self.gen: GeneralConfig
        self.imu: Imu
        self.mag: Magnetometer
        self.pres: Pressure
        self.gps: Gps
        self.feat: Feature
        self.comp: Compass
        self.buoys: Features
        self.radar: Radars
        self.gps_denied: GpsDenied
        self.nom_traj: NominalTrajectory
        self.wpt_planner: WaypointPlanner
        self.kf: KalmanFilter
        self.pdvg: PDVGWorld
        self.state_listener: StateListener
        self.sim_manager: SimManager
        self.reuse: ReusableParams

        # Topic definitions
        self.imu_topic                  = 'imu'
        self.gps_topic                  = 'gps'
        self.pressure_topic             = 'pressure'
        self.compass_topic              = 'compass'
        self.feature_los_topic          = 'feature_camera'
        self.truth_state_topic          = 'uav_state'
        self.start_pause_srv_topic      = 'kalman_filter_start_pause_srv_topic'
        self.reset_srv_topic            = 'kalman_filter_reset_srv_topic'
        self.state_est_topic            = 'uav_state_estimate'
        self.buoy_state_topic           = 'buoy_info'
        self.los_topic                  = 'feature_camera'
        self.uav_waypoints_topic        = 'waypoints'
        self.nav_toggle_topic           = 'nav_state_listener_toggle_topic'
        self.truth_toggle_topic         = 'truth_state_listener_toggle_topic'
        self.nav_reset_topic            = 'reset_nav_listener'
        self.truth_reset_topic          = 'reset_truth_listener'
        self.sense_state_reset_topic    = 'reset_state'
        self.toggle_dynamics_topic      = 'toggle_execution'
        self.reset_autopilot_topic      = 'reset_autopilot'
        self.diagnostics_topic          = 'diagnostics'

        # lincov_interface topic remappings
        self.lincov_analysis_topic = 'lincov_interface/run_analysis'
        self.lincov_matplotlib_topic = 'lincov_interface/plot_analysis_results'
        self.lincov_toggle_ellipse_topic = 'lincov_interface/toggle_rviz_uncertainty_ellipses'

        # pdvg_interface topic remappings
        self.pdvg_analysis_topic = 'pdvg_interface/run_pdvg'
        self.pdvg_plot_topic = 'pdvg_interface/pdvg_plots'

        # CSV plotting topic remappings
        self.csv_plotting_topic = 'csv_plotter/get_plots'

        # SimManager node topic remappings
        self.end_path_reached_topic = 'end_path_reached'
        self.path_ready_topic = 'path_ready'

class GeneralConfig():
    """Default configuration of parameters"""
    # Sim configuration
    use_sim_time = True     # True => use simulated clock
    noise_scale: float = 1.0        # Scale on the noise: 1.0 is standard, 0.0 turns off noise
    inertial_frame: str = 'ned' # The main inertial frame to be used

    # Capability usage flags
    run_kalman_filter = True # True => use Kalman filter, False => estimate is a true-state passthrough
    use_gps_denied_areas = True # True => calculate when gps denied, False => do not
    use_wind = True # True => wind will be used, false otherwise
    use_rviz = True         # True => launch rviz, False => do not
    use_feature_sensor = True # True => use the landmark / buoy feature sensor, false => do not
    use_robot_monitor = True # True => use the robot monitor node, false => do not
    visualize_buoys = True # True => run the buoy node for visualization of buoys
    visualize_radars = True # True => run the buoy node for visualization of radars
    use_lincov_interface = True # True => run the lincov interface, false => do not
    use_pdvg_planner = True # True => run the pdvg planner, false => do not
    use_state_listener = True # True => run the state listener, false => do not

    # UAV plotter parameters
    use_uav_plotter = True # True => use the uav platter, false => do not (other plotting parameters not used if false)
    plot_sensors = True # True => Plot sensor data, False => do not
    plot_nav_error = True # True => Plot the navigation error, False => do not
    time_horizon_in_plots = 100. # Time horizon (in seconds) for which to plot the state and sensor data

    # Planning stack run periods
    lc_mc_period: float = 0.01          # Period (sec) at which the LinCov/Monte Carlo analysis will run
    dynamics_sim_period: float = 0.01   # Period (sec) at which the dynamics, imu, and compass are updated
    path_follower_period = 0.1          # Period (sec) of path following command generation
    path_manager_period = 1.0           # Period (sec) for managing path segments

    def __init__(self,
        urdf_file_name: str = 'fixed_wing_uav.urdf',
        urdf_package: str = 'uav_launch',
        rviz_file_name: str = 'sim_viewer.rviz',
        rviz_package: str = 'pd_planner_launch') -> None:
        """Initialize standard variables that need additional calculation

        Inputs:
            urdf_file_name: Filename within the urdf folder of urdf_package for aircraft
            urdf_package: The package that contains the desired urdf file
            rviz_file_name: The name of the rviz file to use
            rviz_package: The package that contains the rviz file
        """
        # Define the urdf file for visualizing the uav
        self.urdf = os.path.join(
            get_package_share_directory(urdf_package),
            'urdf',
            urdf_file_name)

        # Rviz configuration
        launch_dir = get_package_share_directory(rviz_package)
        self.rviz_config = os.path.join(launch_dir, rviz_file_name)

class ReusableParams():
    """
    This class contains a collection of keys related to nodes that are commonly used in simulation
    scenarios. Each key is used to define part of a dictionary the contains the parameters for each
    of those nodes upon launch of a scenario. That dictionary is then saved as a 'yaml' file which
    can be reused later in the event that data needs to be recreated or for other reasons.
    """
    def __init__(self):
      self.buoy_publisher_key        = "buoy_publisher"
      self.radar_publisher_key       = "radar_publisher"
      self.ch07_sensors_auto_dyn_key = "ch07_sensors_auto_dyn"
      self.feature_sensor_node_key   = "feature_sensor_node"
      self.gps_denied_monitor_key    = "gps_denied_monitor"
      self.kalman_filter_node_key    = "kalman_filter_node"
      self.lincov_interface_key      = "lincov_interface"
      self.pdvg_interface_key        = "pdvg_interface"
      self.csv_plotter_par_key       = "csv_plotter_parameters"
      self.truth_state_listener_key  = "truth_state_listener_csv_parameters"
      self.nav_state_listener_key    = "nav_state_listener_csv_parameters"
      self.sim_manager_key           = "sim_manager_parameters"
