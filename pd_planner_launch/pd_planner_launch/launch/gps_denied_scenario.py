"""Generate a wide area scenario for uav navigation testing"""
from launch import LaunchDescription
from pd_planner_launch.params.environment import Features, Radars, GpsDenied
from pd_planner_launch.params.filtering import KalmanFilter
from pd_planner_launch.params.planning import NominalTrajectory
from pd_planner_launch.params.scenario_params import GeneralConfig, ScenarioParameters
from pd_planner_launch.params.sensors import Imu, Magnetometer, Pressure, Gps, Feature, Compass
from pd_planner_launch.launch.create_uav_sim import create_uav_sim

def gps_denied_scenario() -> LaunchDescription:
    """
    Creates a gps denied scenario by defining the scenario parameters and
    launching the corresponding nodes

    Returns:
    - LaunchDescription with list of nodes to be spun when launch called.
    """

    # Define the radar locations, ranges, and sigma values
    radar_locations = [  [-1000.0, 100.0, 300.0, 1600.0, 900.0],     # x-positions
                        [-500.0, 500.0, -300.0, -800.0, 300.0],             # y-positions
                        [0., 0., 0., 0.] ]                          # z-positions
    radar_ranges = [500.0, 100.0, 100.0, 600.0, 200.0]   # Range at which each feature can be sensed
    radar_sigma = [  [0.0, 0.0, 0.0, 0.0, 0.0],   # std-dev of x positions
                    [0.0, 0.0, 0.0, 0.0, 0.0],   # std-dev of y positions
                    [0.0, 0.0, 0.0, 0.0, 0.0] ]  # std-dev of z positions
    radar_constants: list[float] = []

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
    params.kf.params_from_sensors(imu=params.imu, feat=params.feat, compass=params.comp,
                                  press=params.pres, gps=params.gps)

    # Disable PDVG
    params.gen.use_pdvg_planner = False

    print("Launching...")
    return LaunchDescription(create_uav_sim(params=params))
