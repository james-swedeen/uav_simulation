"""Used to spin up all node needed to run the UAV sim"""
from launch import LaunchDescriptionEntity
from launch_ros.actions import Node
import yaml
import pd_planner_launch.launch.node_dict as node_dict
import pd_planner_launch.params.csv_directory as csv
from pathlib import Path
from pd_planner_launch.params.scenario_params import ScenarioParameters
from typing import List, Any

def create_uav_sim(params: ScenarioParameters) -> List[LaunchDescriptionEntity]:
    """
    Creates the UAV sim configured to perform waypoint following
    """

    # Initialize node launch list
    launch_entities: List[LaunchDescriptionEntity] = []

    # Create buoy Nodes
    if params.gen.visualize_buoys:
        print("Creating buoy publisher...")
        launch_entities += create_buoy_publisher(buoy_params=node_dict.buoy_publisher(params), node_name="buoy_publisher",
                            info_topic="/buoy_info", marker_topic="/buoy_markers")

    # Create radar nodes
    if params.gen.visualize_radars:
        print("Creating radar publisher...")
        launch_entities += create_buoy_publisher(buoy_params=node_dict.radar_publisher(params), node_name="radar_publisher",
                            info_topic="radar_info", marker_topic="radar_markers")

    # Create lincov_interface
    if params.gen.use_lincov_interface:
        print("Creating lincov interface...")
        launch_entities += create_lincov_interface(params, 'lincov_interface')

    # Create pdvg_planner
    if params.gen.use_pdvg_planner:
        print("Creating PDVG interface...")
        launch_entities += create_pdvg_interface(params, 'pdvg_interface')

    # Create CSV plotter if either LinCov or PDVG interfaces are used
    if params.gen.use_pdvg_planner or params.gen.use_lincov_interface:
        print("Creating CSV plotter...")
        launch_entities += create_csv_plotter(params, "csv_plotter")

    ################ Dynamics and kinematics #####################################
    launch_entities.append(
        Node(
            package='uav_launch',
            executable='transforms',
            name='transforms',
            parameters=[{
                'use_sim_time': params.gen.use_sim_time
            }]
        )
    )
    launch_entities.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[params.gen.urdf],
            parameters=[{
                'use_sim_time': params.gen.use_sim_time
            }]
        )
    )
    if params.gen.use_wind:
        launch_entities.append(
            Node(
                package='uav_launch',
                executable='ch04_wind',
                name='wind',
                parameters=[{
                    'use_sim_time': params.gen.use_sim_time
                }]
            )
        )

    # Dynamics & sensors combined with autopilot
    launch_entities.append(
        Node(
            package='uav_launch',
            executable='ch07_sensors_auto_dyn',
            name='sense_auto_dyn',
            parameters=[
                node_dict.ch07_sensors_auto_dyn(params),
                {'initialization_wait_time': 0.01},
            ],
        )
    )
    if params.gen.use_feature_sensor:
        launch_entities.append(
            Node(
                package='sim_kalman_filter',
                executable='feature_sensor_node',
                name='feature_sensor',
                output='screen',
                parameters=[node_dict.feature_sensor_node(params)]
            )
        )
    if params.gen.use_gps_denied_areas:
        launch_entities.append(
            Node(
                package='uav_launch',
                executable='gps_denied_monitor',
                name='gps_denied_monitor',
                parameters=[node_dict.gps_denied_monitor(params)]
            )
        )


    ################# UAV Control Software #######################################
    # State Estimator
    if params.gen.run_kalman_filter:
        # Model replacement Kalman filter
        launch_entities.append(
            Node(
                package='sim_kalman_filter',
                executable='kalman_filter_node',
                name='kalman_filter',
                output='screen',
                #prefix=['xterm -e gdb -ex run --args'],
                parameters=[node_dict.kalman_filter_node(params)]
            )
        )

    else:
        # Relay to define state estimate from true state
        launch_entities.append(
            Node(
                package='topic_tools',
                executable='relay',
                name="state_estimator_relay",
                parameters=[
                    {"input_topic": "uav_state"},
                    {"output_topic": "uav_state_estimate"},
                    {"use_sim_time": params.gen.use_sim_time}
                ]
            )
        )


    # Path follower
    launch_entities.append(
        Node(
            package='uav_launch',
            executable='ch10_path_follower',
            name='path_follower',
            parameters=[{
                'use_sim_time': params.gen.use_sim_time,
                'ts': params.gen.path_follower_period
            }]
        )
    )

    # Path manager
    launch_entities.append(
        Node(
            package='uav_launch',
            executable='ch11_path_manager',
            name='path_manager',
            #emulate_tty=True,
            #output="screen",
            parameters=[{
                'use_sim_time': params.gen.use_sim_time,
                'ts': params.gen.path_manager_period
            }]
        )
    )

    ################# Tools for Interacting with the Sim #########################
    launch_entities.append(
        Node(
            package='uav_launch',
            executable='ch11_waypoints_relay',
            name='waypoint_relay',
            remappings=[
            ('waypoints_in', '/wp_panel/waypoint_cmd'),
            ('waypoints_out', '/waypoints'),
         ],
            parameters=[{
                'use_sim_time': params.gen.use_sim_time
            }]
        )
    )
    if params.gen.use_rviz:
        launch_entities.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', [params.gen.rviz_config]],
                remappings=[ # Topics for node in LinCov analysis panel
                  ('/run_analysis', params.lincov_analysis_topic),
                  ('/plot_results', params.lincov_matplotlib_topic),
                  ('/run_pdvg', params.pdvg_analysis_topic),
                  ('/pdvg_plot', params.pdvg_plot_topic),
                  ('/toggle_uncertainty_ellipses', params.lincov_toggle_ellipse_topic),
                ],
                parameters=[{
                    'use_sim_time': False
                }]
            )
        )
    if params.gen.use_uav_plotter:
        launch_entities.append(
            Node(
                package='uav_launch',
                executable='uav_plotter',
                name='uav_plotter',
                parameters=[{
                    'use_sim_time': params.gen.use_sim_time,
                    't_horizon': params.gen.time_horizon_in_plots,
                    'plot_sensors': params.gen.plot_sensors,
                    'plot_nav_error': params.gen.plot_nav_error
                }]
            )
        )
    if params.gen.use_robot_monitor:
        launch_entities.append(
            Node(
                package='rqt_robot_monitor',
                executable='rqt_robot_monitor',
                name='rqt_robot_monitor',
                parameters=[{
                    'use_sim_time': False
                }]
            )
        )
        launch_entities.append(
            Node(
                package='uav_launch',
                executable='diagnotic_aggregator',
                name='diagnostic_aggregator',
                parameters=[{
                    'use_sim_time': False
                }]
            )
        )


    return launch_entities

def create_buoy_publisher(buoy_params: dict[str, Any], node_name: str,
                          info_topic: str, marker_topic: str) -> List[LaunchDescriptionEntity]:
    """
    Creates the buoy publisher

    Args:
    - buoy_params: defines the buoy locations and uncertainties, etc.
    - node_name: name of the node to be created
    - info_topic: name of the topic for publishing the buoy locations
    - marker_topic: name of the topic for publishing the visualization messages
    - params: contains the simulation time flag

    Returns:
    - LaunchDescriptionEntity containing a buoy_publisher node.
      Node configuration is dependent on if a config file was
      given to the 'scenario' input.
    """

    launch_entities: List[LaunchDescriptionEntity] = []
    launch_entities.append(
        Node(
            package='buoy_publisher',
            executable='buoy_publisher',
            name=node_name,
            parameters=[buoy_params],
            remappings=[
                ("/buoy_info", info_topic),
                ("/buoy_markers", marker_topic)
            ]
        )
    )

    return launch_entities

def create_lincov_interface(params: ScenarioParameters,
                            node_name: str) -> List[LaunchDescriptionEntity]:
    """
    Creates the lincov interface node with parameters from a ScenarioParameters
    object.

    Args:
    - params: Contains several sets of parameters used by the lincov_interface node
    - node_name: Name the node will take on when launched

    Returns:
    - LaunchDescriptionEntity containing a lincov_interface node
    """

    launch_entities: List[LaunchDescriptionEntity] = []
    launch_entities.append(
        Node(
            package='planner_interface',
            executable='lincov_interface',
            name=node_name,
            remappings=[
                ('/run_analysis', params.lincov_analysis_topic),
                ('/waypoints', params.uav_waypoints_topic),
                ('/plot_results', params.lincov_matplotlib_topic),
                ('/toggle_uncertainty_ellipses', params.lincov_toggle_ellipse_topic),
                ('/csv_plotter', params.csv_plotting_topic),
            ],
            parameters=[node_dict.lincov_interface(params)],
        ),
    )
    return launch_entities

def create_pdvg_interface(params: ScenarioParameters,
                          node_name: str) -> List[LaunchDescriptionEntity]:
    """
    Creates the pdvg interface node with parameters from a ScenarioParameters
    object.

    Args:
    - params: Contains several sets of parameters used by the pdvg_interface node
    - node_name: Name the node will take on when launched

    Returns:
    - LaunchDescriptionEntity containing a pdvg_interface node
    """
    launch_entities: List[LaunchDescriptionEntity] = []
    launch_entities.append(
        Node(
            package='planner_interface',
            executable='pdvg_interface',
            name=node_name,
            remappings=[
                ('/run_pdvg', params.pdvg_analysis_topic),
                ('/pdvg_plot', params.pdvg_plot_topic),
                ('/waypoints', params.uav_waypoints_topic),
                ('/uav_state', params.truth_state_topic),
                ('/csv_plotter', params.csv_plotting_topic),
            ],
            parameters=[node_dict.pdvg_interface(params)],
        ),
    )
    return launch_entities

def create_csv_plotter(params: ScenarioParameters,
                       node_name: str) -> List[LaunchDescriptionEntity]:
    """
    Creates a csv_plotter node to handle plotting for PDVG and LinCov/Monte Carlo
    operations

    Args:
    - params: Contains topic remappings to assign to node
    - node_name: Name the node will take on when launched

    Returns:
    - LaunchDescriptionEntity containing a csv_plotter node
    """

    launch_entities: List[LaunchDescriptionEntity] = []

    # Create directories to store csv files from PDVG runs
    csv.create_csv_directory()

    # Add the csv plotter for data storage
    launch_entities.append(
        Node(
            package='planner_interface',
            executable='csv_plotter_run.py',
            name=node_name,
            remappings=[
                ('/csv_plotter', params.csv_plotting_topic),
            ],
            parameters=[node_dict.csv_plotter_parameters(params)],
        )
    )

    return launch_entities
