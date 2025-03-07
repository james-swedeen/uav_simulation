"""Generate a wide area scenario for uav navigation testing"""
from launch import LaunchDescription
from pd_planner_launch.launch.create_uav_sim_headless import create_uav_sim_headless
from pd_planner_launch.params.environment import Features, Radars, GpsDenied, PDVGWorld
from pd_planner_launch.params.filtering import KalmanFilter
from pd_planner_launch.params.planning import NominalTrajectory, SimManager
from pd_planner_launch.params.csv_directory import StateListener
from pd_planner_launch.params.scenario_params import GeneralConfig, ScenarioParameters, ReusableParams
from pd_planner_launch.params.sensors import Imu, Magnetometer, Pressure, Gps, Feature, Compass

def headless_sim_scenario(reuse_file: str = None) -> LaunchDescription:
    """
    Creates scenario to use with a buoy_publisher node. Starts a buoy_publisher node
    and a rviz2 node to display buoy Markers and radii Markers.

    Args:
    - reuse_file: String of the path to a previous parameters file so those parameters can be reused

    Returns:
    - LaunchDescription with list of nodes to be spun when launch called.
    """

    # Define the radar parameters
    radar_locations, radar_ranges, radar_sigma, radar_constants = define_radars()

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
    params.radar = Radars(locations=radar_locations, ranges=radar_ranges, sigma=radar_sigma, radar_constants=radar_constants)
    params.gps_denied = GpsDenied()
    params.nom_traj = NominalTrajectory()
    params.kf = KalmanFilter()
    params.pdvg = PDVGWorld()
    params.state_listener = StateListener()
    params.sim_manager = SimManager()
    params.reuse = ReusableParams()

    # Update the GPS denied regions to have multiple small regions
    region1 = [(200.0,800.0),(200.0,-800.0),(800.0,-800.0),(800.0,800.0)]
    region2 = [(-4300.0,-2700.0),(-3500.0,-3000.0),(-3300.0,-2700.0),(-4100.0,-2400.0)]
    region3 = [(-3450.0,1050.0),(-2200.0,2050.0),(-2450.0,2300.0),(-3600.0,1400.0)]
    params.gps_denied.regions = [region1, region2, region3]

    # Update the curvature for the path planning
    params.nom_traj.max_curvature = 0.02
    params.sim_manager.min_radius = 1/params.nom_traj.max_curvature

    # Update the kalman filter from the sensor parameters
    params.kf.params_from_sensors(imu=params.imu, feat=params.feat, compass=params.comp,
                                  press=params.pres, gps=params.gps)

    # Set PDVG-specific parameters
    params.gen.use_feature_sensor   = False
    params.gen.visualize_radars     = True
    params.gen.use_pdvg_planner     = True
    params.gen.use_gps_denied_areas = True
    params.gen.visualize_buoys      = False
    params.gen.use_state_listener   = True

    # Use PDVG target point instead of manual waypoints
    params.sim_manager.use_pdvg = True

    print("Launching...")
    return LaunchDescription(create_uav_sim_headless(params=params, reuse_file=reuse_file))

def define_radars() -> tuple[list[list[float]], list[float], list[list[float]], list[float] ]:
    """ Defines the locations, ranges, and sigma values for the Radars to be used

    Returns:
        locations: xyz location of each radar
        ranges: the range of each radar
        sigma: std-dev of xyz positions
        radar_constants: Defined radar constants for each radar
    """
    # Set predefined radar constants for set range (m)
    rc_250 = 7.8e-12
    rc_500 = 1.21e-10
    rc_1000 = 1.8e-9

    # Test Case: PDVG Playground
    # Q1 (Simple gaps):
    gap_n_radars = 4

    gap_locs = [ [350.0, 2000.0] + [1175.0, 3500.0],
                    [3500.0]*2 + [2000.0, 2000.0]]
    gap_ranges = [500]*gap_n_radars
    gap_radar_consts = [rc_500]*gap_n_radars

    # Q2 (Weaving)
    weave_diag_radars = 2
    weave_r1_radars = 2
    weave_r2_radars = 1
    weave_n_radars = weave_diag_radars + weave_r1_radars + weave_r2_radars

    weave_diag_locs = [[-4647.0, -3941.0],
                        [353.0, 1059.0]]
    weave_diag_ranges = [500.0]*weave_diag_radars
    weave_diag_radar_consts = [rc_500]*weave_diag_radars

    weave_r1_locs = [[-2000.0, -1000.0],
                        [2500.0]*weave_r1_radars]
    weave_r1_ranges = [500.0]*weave_r1_radars
    weave_r1_radar_consts = [rc_500]*weave_r1_radars

    weave_r2_locs = [[-4000.0],
                        [4000.0]*weave_r2_radars]
    weave_r2_ranges = [500.0]*weave_r2_radars
    weave_r2_radar_consts = [rc_500]*weave_r2_radars

    weave_locs = [weave_diag_locs[0] + weave_r1_locs[0] + weave_r2_locs[0],
                    weave_diag_locs[1] + weave_r1_locs[1] + weave_r2_locs[1]]
    weave_ranges = weave_diag_ranges + weave_r1_ranges + weave_r2_ranges
    weave_radar_consts = weave_diag_radar_consts + weave_r1_radar_consts + weave_r2_radar_consts

    # Q3 (Size steps)
    steps_n_radars = 6

    steps_locs = [[-5000.0, -5000.0, -2000.0, -2400.0, -3000.0, -1200.0],
                    [-5000.0, -2000.0, -5000.0, -1400.0, -3000.0, -2600.0]]
    steps_ranges = [1000.0]*3 + [500.0]*3
    steps_radar_consts = [rc_1000]*3 + [rc_500]*3

    # Q4 (Canyon)
    canyon_n_radars = 2

    canyon_locs = [[1500.0, 4000.0],
                    [-1500.0, -3000.0]]
    canyon_ranges = [1000.0]*canyon_n_radars
    canyon_radar_consts = [rc_1000]*canyon_n_radars

    # Inner square of smaller radars
    inner_n_radars = 3
    inner_locs = [[500.0, -500.0, -500.0],
                    [500.0, 500.0, -500.0],
                    [0.0]*inner_n_radars]
    inner_ranges = [250.0]*inner_n_radars
    inner_radar_consts = [rc_250]*inner_n_radars

    num_radars = gap_n_radars + inner_n_radars + \
                    weave_n_radars + steps_n_radars + canyon_n_radars
    locations = [gap_locs[0] + inner_locs[0] + \
                    weave_locs[0] + steps_locs[0] + canyon_locs[0],  # End x locs
                    gap_locs[1] + inner_locs[1] + \
                    weave_locs[1] + steps_locs[1] + canyon_locs[1],  # End y locs
                    [0.0]*num_radars] # End z locs

    ranges = gap_ranges + inner_ranges + \
                weave_ranges + steps_ranges + canyon_ranges
    sigma = [[1.0]*num_radars,
            [1.0]*num_radars,
            [1.0]*num_radars]
    radar_constants = gap_radar_consts + inner_radar_consts + \
                       weave_radar_consts + steps_radar_consts + canyon_radar_consts


    return locations, ranges, sigma, radar_constants
