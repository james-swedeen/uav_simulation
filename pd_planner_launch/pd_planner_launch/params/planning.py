"""This module defines the parameters for trajectory planning
"""

from enum import IntEnum
from uav_interfaces.msg import UavWaypoints
from ament_index_python.packages import get_package_share_directory
import os

class NominalTrajectory():
    """Parameters defining the nominal operations"""
    def __init__(self) -> None:
        self.velocity: float = 25.           # Nominal velocity of the UAV (m/s)
        self.max_curvature: float = 0.005    # The maximum allowable curvature for the UAV (1/m)
        self.max_curve_rate: float = 0.0002  # The maximum allowable curvature rate for the UAV (1/(meter-second))
        self.nominal_pitch: float = 4.2      # Nominal pitch angle of the UAV expressed in degrees.
        self.fillet_dt: float = 0.01   # Time interval for generating fillet-based paths
        self.line_dt:   float = 1.0    # Time interval for generating line-based paths
        self.use_arc_fillet   = True   # Use (or not) arc fillet generator

class PlannerConfig(IntEnum):
    NULL_PLANNECONFIG = 0
    RRT = 1
    RRT_STAR = 2
    RRT_STAR_SMART = 3
    BIT = 4

class WaypointPlanner():
    """Parameters defining how the waypoint planner should work."""
    def __init__(self) -> None:
        ## Building Obstacle Parameters ##
        self.building_data_file: str = os.path.join(get_package_share_directory('occupancy_grid'), 'config', 'new_york_buildings_manhattan.csv') # File containing the locations and heights of all buildings in the environment
        self.max_north: float = 1000.0                       # The largest north value that is still considered on the map
        self.max_east: float = 1000.0                        # The largest east value that is still considered on the map
        self.min_north: float = -9700.0                      # The smallest north value that is still considered on the map
        self.min_east: float = -4650.0                       # The smallest east value that is still considered on the map
        self.nominal_altitude: float = 25.0                  # All buildings above this hight are considered obstacles
        self.occupancy_grid_resolution: float = 1.0          # How many square meters each pixel of the occupancy grid represents

        ## Planner Parameters ##
        self.planner_type: PlannerConfig = PlannerConfig.BIT # What planning algorithm will be used
        self.max_duration: float = 90.0                      # The max duration the planner will ever spend planning in seconds (Note: -1.0 means no limit)
        self.target_cost: float = 10000.0                    # If a path of length less then this is generated the planner will end immediately
        self.edge_resolution: float = 1.0                    # How many samples per a meter the edges will be checked at
        self.near_radius: float = 1000.0                     # How far two nodes can be while still being neighbors
        self.target_radius: float = 1.0                      # The radius of the target set
        self.check_target_ratio: int = 100                   # How often the target set will be sampled (Note: not used by BIT)
        self.batch_size: int = 1500                          # How many samples constitute a batch (Note: only used with BIT)
        self.number_target_samples: int = 1                  # How many samples of the target set to start with (Note: only used with BIT)

class SimManager():
    """Parameters defining the behavior of the SimManager node in headless runs"""
    def __init__(self) -> None:
        self.clock_speed:      float = 5.0
        self.total_runs:       int   = 1 # Note: This should never be set to anything but 1. The functionality is no longer supported. See the "run_mc_commands.sh" to run a batch of Monte Carlo simulations.
        self.use_pdvg:         bool  = False

        self.nominal_down:     float = -100.0
        self.nominal_velocity: float = NominalTrajectory().velocity

        # Target point for PDVG path if used
        self.pdvg_x_pos:     float = 1100.0
        self.pdvg_y_pos:     float = 1000.0

        # Manual waypoint positions for non-PDVG path (straight-line default)
        self.manual_wp_x:    list[float] = [1000.0, 2000.0, 3000.0]
        self.manual_wp_y:    list[float] = [0.0, 0.0, 0.0]
        self.path_type: str              = UavWaypoints.TYPE_FILLET
        self.min_radius: float           = 1/NominalTrajectory().max_curvature

class HeadlessSimSelect():
    """Parameters used to select the simulation type when launching headless simulations"""
    LINE_STRAIGHT: int = 0
    LINE_CURVED: int   = 1

    def __init__(self) -> None:
        # Line types

        self.line_type: int = self.LINE_CURVED  # Which type of path to use: 'straight' or 'curved'

        # Sensor options
        self.use_gps_denied_areas = False
        self.use_position_sensor = False
