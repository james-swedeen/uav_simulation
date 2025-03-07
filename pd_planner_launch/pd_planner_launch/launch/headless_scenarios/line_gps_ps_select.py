"""Generate a wide area scenario for uav navigation testing"""
from launch import LaunchDescription
from pd_planner_launch.launch.create_uav_sim_headless import create_uav_sim_headless
from pd_planner_launch.params.environment import Features, Radars, GpsDenied, PDVGWorld
from pd_planner_launch.params.filtering import KalmanFilter
from pd_planner_launch.params.planning import NominalTrajectory, SimManager, HeadlessSimSelect
from pd_planner_launch.params.csv_directory import StateListener
from pd_planner_launch.params.scenario_params import GeneralConfig, ScenarioParameters, ReusableParams
from pd_planner_launch.params.sensors import Imu, Magnetometer, Pressure, Gps, Feature, Compass
import numpy as np

def line_gps_ps_select(reuse_file: str = None) -> LaunchDescription:
    """
    Run a scenario based on parameters set in the SimManager and HeadlessSimSelect
    classes

    Args:
    - reuse_file: Path to a file containing parameters to reuse from an old simulation

    Returns:
    - LaunchDescription with list of nodes to be spun when launch called.
    """

    # Define the scenario parameters
    params = ScenarioParameters()
    params.gen = GeneralConfig()
    params.imu = Imu()
    params.mag = Magnetometer()
    params.pres = Pressure()
    params.gps = Gps()
    params.feat = Feature()
    params.comp = Compass()
    params.buoys = Features()

    # Use "null" radar far from where the aircraft is
    params.radar = Radars(locations=[[-5000.0], [5000.0], [0.0]], \
                          ranges=[0.1], \
                          sigma=[[0.0], [0.0], [0.0]], \
                          radar_constants=[0.0]) # Using as a test, move "null" radar elsewhere
    params.gps_denied = GpsDenied()
    params.nom_traj = NominalTrajectory()
    params.kf = KalmanFilter()
    params.pdvg = PDVGWorld()
    params.state_listener = StateListener()
    params.sim_manager = SimManager()
    params.headless_options = HeadlessSimSelect()
    params.reuse = ReusableParams()

    # === SCENARIO-SPECIFIC PARAMETERS ===
    # Set scenario-specific flags
    params.gen.use_feature_sensor   = params.headless_options.use_position_sensor
    params.gen.use_gps_denied_areas = params.headless_options.use_gps_denied_areas
    params.gen.use_pdvg_planner     = False
    params.sim_manager.use_pdvg     = False
    params.gen.visualize_buoys      = params.headless_options.use_position_sensor
    params.gen.run_kalman_filter    = True
    params.gen.use_robot_monitor    = False
    params.gen.use_wind             = False
    params.gen.noise_scale          = 1.0
    params.sim_manager.clock_speed  = 0.1
    params.kf.state_est_pub_freq    = 100.0;
    params.kf.integration_time_step = 0.0001;
    params.gen.dynamics_sim_period  = 0.001;

    # Provide scenario setups based on the type of path to travel
    if params.headless_options.line_type == HeadlessSimSelect.LINE_CURVED:
      # Set waypoints
      wp_dist = 100.0*25.0;
      params.sim_manager.manual_wp_x = [wp_dist, wp_dist, 2*wp_dist, 2*wp_dist, 3*wp_dist, 3*wp_dist, 4*wp_dist, 4*wp_dist]
      params.sim_manager.manual_wp_y = [  0.0,   wp_dist, wp_dist,   -wp_dist,  -wp_dist,  wp_dist,   wp_dist,   -wp_dist]

      # Set GPS-denied region such that UAV starts with GPS and ends with GPS
      region1 = [(500.0, 500.0), (500.0, -500.0), (3000.0 + 500*np.cos(np.pi/4)*3, -500.0),
                (500, 3000-500), (0.0, 3000-500/np.cos(np.pi/4)), (2500.0, 500.0)]
      params.gps_denied.regions = [region1]

      # Set feature locations (set a buoy right at end waypoint, enter range right out of gps-denied)
      params.buoys.locations =  [[3000.0,    0.0],     # N
                                 [   0.0, 3000.0],     # E
                                 [   0.0,    0.0]]     # D

      params.buoys.sigma =      [[0.0, 0.0],     # N
                                 [0.0, 0.0],     # E
                                 [0.0, 0.0]]     # D

      params.buoys.ranges =     [500.0, 500.0]

    elif params.headless_options.line_type == HeadlessSimSelect.LINE_STRAIGHT:
      # Set waypoints
      end_point_x = 50000.0
      params.sim_manager.manual_wp_x = [end_point_x-2.0, end_point_x-1.0, end_point_x]
      params.sim_manager.manual_wp_y = [0.0, 0.0, 0.0]

      # Set GPS regions
      region1 = [(-50000.0, 100000.0), (-50000.0, -100000.0), (100000.0, -100000.0), (100000.0, 100000.0)]
      params.gps_denied.regions = [region1]

      # Set feature locations
      params.buoys.locations =  [[3000.0],  # N
                                 [0.0],     # E
                                 [0.0]]     # D

      params.buoys.sigma =      [[0.0],     # N
                                 [0.0],     # E
                                 [0.0]]     # D

      params.buoys.ranges =     [500.0]

    # Default to straight-line test
    else:
      print("Error: HeadlessSimSelect line type not recognized")

    # Make launch list
    print("Launching...")
    return LaunchDescription(create_uav_sim_headless(params=params, reuse_file=reuse_file, max_sim_time_sec=1000.0))
