"""Generate the buildings of Manhattan and plan paths through them"""

from launch import LaunchDescription, LaunchDescriptionEntity
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import multiprocessing
from pd_planner_launch.params.planning import NominalTrajectory, WaypointPlanner
from pd_planner_launch.params.scenario_params import GeneralConfig, ScenarioParameters
from pd_planner_launch.launch.create_uav_sim import create_uav_sim
from pd_planner_launch.params.sensors import Imu, Magnetometer, Pressure, Gps, Compass
from typing import List

def manhattan_path_planner() -> LaunchDescription:
    """
    Creates the Manhattan buildings in rviz2 and plans a path to a user specified
    point.

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
    params.comp = Compass()
    params.wpt_planner = WaypointPlanner()
    params.nom_traj = NominalTrajectory()

    # Set general parameters that differ from default
    params.gen.run_kalman_filter = False
    params.gen.use_gps_denied_areas = False
    params.gen.use_wind = False
    params.gen.use_feature_sensor = False
    params.gen.visualize_buoys = False
    params.gen.visualize_radars = False
    params.gen.use_lincov_interface = False
    params.gen.use_uav_plotter = False
    params.gen.use_pdvg_planner = False
    params.gen.rviz_config = os.path.join(get_package_share_directory('occupancy_grid'), 'rviz', 'manhattan_path_planner.rviz')

    # Initialize the launch and sim params
    launch_entities: List[LaunchDescriptionEntity] = []

    # Create uav sim nodes
    print("Launching uav sim")
    launch_entities += create_uav_sim(params=params)

    # Create waypoint planner nodes
    print("Launching waypoint planner")
    launch_entities += create_waypoint_planner(params=params)


    print("Launching...")
    return LaunchDescription(launch_entities)

def create_waypoint_planner(params: ScenarioParameters) -> List[LaunchDescriptionEntity]:
    """
    Creates the waypoint planner and obstacles for the planner
    """

    # Initialize node launch list
    launch_entities: List[LaunchDescriptionEntity] = []

    # Path planner node
    launch_entities.append(Node(
        package='rrt_search',
        executable='waypoint_path_planner',
        name='path_planner',
        output="screen",
        #prefix=['xterm -e gdb -ex run --args'],
        parameters=[{
            'use_sim_time': params.gen.use_sim_time,
            'planner_type': params.wpt_planner.planner_type.value,
            'batch_size': params.wpt_planner.batch_size,
            'num_target_samples': params.wpt_planner.number_target_samples,
            'turning_radius': float(1)/params.nom_traj.max_curvature,
            'nominal_altitude': params.wpt_planner.nominal_altitude,
            'nominal_airspeed': params.nom_traj.velocity,
            'uav_state_topic': params.state_est_topic,
            'goal_pose_topic': '/wp_panel/goal_pose',
            'waypoints_topic': '/waypoints',
            'replan_topic': '/replan',
            'occupancy_grid':{
                'line_width': -1.0,
                'resolution': params.wpt_planner.occupancy_grid_resolution,
                'origin': [params.wpt_planner.min_north, params.wpt_planner.min_east],
                'width': params.wpt_planner.max_north-params.wpt_planner.min_north,
                'height': params.wpt_planner.max_east-params.wpt_planner.min_east,
                'nominal_altitude': params.wpt_planner.nominal_altitude,
                'building_data_file': params.wpt_planner.building_data_file,
                },
            'sampler':{
                'target_radius': params.wpt_planner.target_radius,
                'check_target_ratio': params.wpt_planner.check_target_ratio,
                },
            'nns':{
                'leaf_size': 1000,
                'num_threads': multiprocessing.cpu_count(),
                },
            'steer':{
                'near_radius': params.wpt_planner.near_radius,
                },
            'problem':{
                'target_radius': params.wpt_planner.target_radius,
                'max_iteration': 0,
                'max_duration': params.wpt_planner.max_duration,
                'target_cost': params.wpt_planner.target_cost,
                },
            'edge':{
                'resolution': params.wpt_planner.edge_resolution,
                },
            }],
        ))
    ## Occupancy Grid Plotter
    launch_entities.append(Node(
        package='occupancy_grid',
        executable='building_occupancy_grid_node',
        name='occupancy_grid_pub',
        output='screen',
        #prefix=['xterm -e gdb -ex run --args'],
        parameters=[{
            'use_sim_time': params.gen.use_sim_time,
            'occupancy_grid':{
                'line_width': -1.0,
                'resolution': params.wpt_planner.occupancy_grid_resolution,
                'origin': [params.wpt_planner.min_north, params.wpt_planner.min_east],
                'width': params.wpt_planner.max_north-params.wpt_planner.min_north,
                'height': params.wpt_planner.max_east-params.wpt_planner.min_east,
                'nominal_altitude': params.wpt_planner.nominal_altitude,
                'building_data_file': params.wpt_planner.building_data_file,
                },
            }],
        ))
    # World manager
    launch_entities.append(Node(
        package='occupancy_grid',
        executable='data_world_plotter_node.py',
        name='world_plotter',
        parameters=[{
            'use_sim_time': params.gen.use_sim_time,
            'ts': 1.0,
            'min_north': params.wpt_planner.min_north,
            'min_east': params.wpt_planner.min_east,
            'max_north': params.wpt_planner.max_north,
            'max_east': params.wpt_planner.max_east,
            'data_file': params.wpt_planner.building_data_file,
            }],
        ))

    return launch_entities


