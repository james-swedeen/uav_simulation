"""
Define and set up all nodes needed to run headless simulations for data collection
based on parameters set in this package.
"""
import launch
from launch import LaunchDescriptionEntity
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node, RosTimer, SetUseSimTime
from launch.event_handlers import OnProcessStart
import yaml
import pd_planner_launch.launch.node_dict as node_dict
from pathlib import Path
from pd_planner_launch.params.scenario_params import ScenarioParameters
from typing import List

def create_uav_sim_headless(params: ScenarioParameters, reuse_file: str = None, max_sim_time_sec: float = -1.0) -> List[LaunchDescriptionEntity]:
    """
    Set up a headless simulation for data collection

    Args:
    - params: Contains set of all parameters regarding the launch of this simulation
    - reuse_file: Path to a file containing parameters to reuse from an old simulation
    - max_sim_time_sec: The max number of simulation seconds to run before the simulation will shutdown, if -1 this is disabled

    Returns:
    - LaunchDescriptionEntity list containing all nodes necessary for the headless simulation
    """
    data_directory: str = None
    # If the reuse_file input is not None, create replacement dictionaries for
    # each node_dict usage
    reuse_dict: dict = None
    if reuse_file is not None:
      with open(reuse_file, 'r') as file:
        reuse_dict = yaml.safe_load(file)

    # Initialize node launch list
    launch_entities: List[LaunchDescriptionEntity] = []

    # Create buoy publisher
    if params.gen.visualize_buoys:
        print("Creating buoy publisher...")
        launch_entities += create_buoy_publisher(params, 'buoy_publisher', info_topic=params.buoy_state_topic,
                                                  marker_topic="/buoy_markers", reuse_dict=reuse_dict)

    # Create pdvg_planner
    if params.gen.use_pdvg_planner:
        print("Creating PDVG interface...")
        launch_entities += create_pdvg_interface(params, 'pdvg_interface', reuse_dict)

    # Create State Listener node for headless operation
    if params.gen.use_state_listener:
        print("Creating State Listener...")
        (data_directory, launch_entity) = create_state_listeners(params, 'state_listener', reuse_dict)
        launch_entities += launch_entity

    ################ Dynamics and kinematics #####################################
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
    ch07_params = node_dict.ch07_sensors_auto_dyn(par_st=params)

    # Update values in parameters dictionary with the old ones if a reuse file was selected
    if reuse_dict is not None:
        ch07_params.update(reuse_dict[params.reuse.ch07_sensors_auto_dyn_key])

    launch_entities.append(
        Node(
            package='uav_launch',
            executable='ch07_sensors_auto_dyn',
            name='sense_auto_dyn',
            remappings=[
              ('/reset_autopilot', params.reset_autopilot_topic),
            ],
            parameters=[
                ch07_params,
                {'initialization_wait_time': 1.0},
            ],
        )
    )

    if params.gen.use_feature_sensor:
        feature_sensor_params = node_dict.feature_sensor_node(par_st=params)

        # Update values in parameters dictionary with the old ones if a reuse file was selected
        if reuse_dict is not None:
            feature_sensor_params.update(reuse_dict[params.reuse.feature_sensor_node_key])

        launch_entities.append(
            Node(
                package='sim_kalman_filter',
                executable='feature_sensor_node',
                name='feature_sensor',
                output='screen',
                parameters=[feature_sensor_params]
            )
        )

    if params.gen.use_gps_denied_areas:
        gps_monitor_params = node_dict.gps_denied_monitor(par_st=params)

        # Update values in parameters dictionary with the old ones if a reuse file was selected
        if reuse_dict is not None:
            gps_monitor_params.update(reuse_dict[params.reuse.gps_denied_monitor_key])

        launch_entities.append(
            Node(
                package='uav_launch',
                executable='gps_denied_monitor',
                name='gps_denied_monitor',
                parameters=[gps_monitor_params]
            )
        )


    ################# UAV Control Software #######################################
    # State Estimator
    if params.gen.run_kalman_filter:
        kalman_filter_params = node_dict.kalman_filter_node(par_st=params)

        # Update values in parameters dictionary with the old ones if a reuse file was selected
        if reuse_dict is not None:
            kalman_filter_params.update(reuse_dict[params.reuse.kalman_filter_node_key])

        # Model replacement Kalman filter
        launch_entities.append(
            Node(
                package='sim_kalman_filter',
                executable='kalman_filter_node',
                name='kalman_filter',
                output='screen',
                #prefix=['xterm -e gdb -ex run --args'],
                parameters=[kalman_filter_params]
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
                    {"input_topic":  params.truth_state_topic},
                    {"output_topic": params.state_est_topic},
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

    # Create Sim Manager node
    print("Creating Sim Manager...")
    launch_entities += create_sim_manager(params, 'sim_manager', max_sim_time_sec, reuse_dict)

    # Make a parameters file for future reference regarding test data
    print("Saving parameters file...")
    save_params_file(params=params, data_dir=data_directory, reuse_dict=reuse_dict)

    return launch_entities

def create_pdvg_interface(params: ScenarioParameters,
                          node_name: str, reuse_dict: dict = None) -> List[LaunchDescriptionEntity]:
    """
    Creates the pdvg interface node with parameters from a ScenarioParameters
    object.

    Args:
    - params: Contains several sets of parameters used by the pdvg_interface node
    - node_name: Name the node will take on when launched
    - reuse_dict: Dictionary containing parameter values from previous simulations

    Returns:
    - LaunchDescriptionEntity containing a pdvg_interface node
    """
    pdvg_params = node_dict.pdvg_interface(par_st=params)

    # Update values in parameters dictionary with the old ones if a reuse file was selected
    if reuse_dict is not None:
        pdvg_params.update(reuse_dict[params.reuse.pdvg_interface_key])

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
            parameters=[pdvg_params],
        ),
    )
    return launch_entities

def create_state_listeners(params: ScenarioParameters,
                          node_name: str, reuse_dict: dict = None) -> tuple[str, List[LaunchDescriptionEntity]]:
    """
    Creates a state listener node that records truth and navigation state data
    and saves it to file.

    Args:
    - params: Contains topic remappings to assign to node
    - node_name: Name the node will take on when launched
    - reuse_dict: Dictionary containing parameter values from previous simulations

    Returns:
    - LaunchDescriptionEntity containing a state listener node
    """

    # Get parameters for the state listener that collects truth-state data
    truth_state_listener_params = node_dict.state_listener_csv_parameters(params, get_truth_data=True, create_dir=True)
    temp_data_directory = truth_state_listener_params['data_directory']

    # Update values in parameters dictionary with the old ones if a reuse file was selected
    if reuse_dict is not None:
        truth_state_listener_params.update(reuse_dict[params.reuse.truth_state_listener_key])

    # Get parameters for the state listener that collects navigation-state data
    nav_state_listener_params = node_dict.state_listener_csv_parameters(params, get_truth_data=False, create_dir=False)

    # Update values in parameters dictionary with the old ones if a reuse file was selected
    if reuse_dict is not None:
        nav_state_listener_params.update(reuse_dict[params.reuse.nav_state_listener_key])

    # Updates to dictionaries above would override data directory, so reassign here
    nav_state_listener_params['data_directory'] = \
      truth_state_listener_params['data_directory'] = temp_data_directory

    # Add the truth-state listener
    launch_entities: List[LaunchDescriptionEntity] = []
    launch_entities.append(
        Node(
            package='planner_interface',
            executable='state_listener_csv_node',
            name=node_name+"_truth",
            remappings=[
                ('/state_topic', params.truth_state_topic),
                ('/state_listener_toggle_topic', params.truth_toggle_topic),
                ('/reset_listener', params.truth_reset_topic),
            ],
            parameters=[truth_state_listener_params],
        )
    )

    # Add the navigation-state listener
    launch_entities.append(
        Node(
            package='planner_interface',
            executable='state_listener_csv_node',
            name=node_name+"_nav",
            remappings=[
                ('/state_topic', params.state_est_topic),
                ('/state_listener_toggle_topic', params.nav_toggle_topic),
                ('/reset_listener', params.nav_reset_topic),
            ],
            parameters=[nav_state_listener_params],
        )
    )

    return (temp_data_directory, launch_entities)

def create_sim_manager(params: ScenarioParameters,
        node_name: str, max_sim_time_sec: float, reuse_dict: dict = None) -> List[LaunchDescriptionEntity]:
    """
    Create a sim manager node to run the headless simulation

    Args:
    - params: Contains topic remappings to assign to node
    - node_name: Name the node will take on when launched
    - reuse_dict: Dictionary containing parameter values from previous simulations

    Returns:
    - LaunchDescriptionEntity containing a sim manager node
    """
    # Get parameters
    sim_manager_params = node_dict.sim_manager_parameters(params)
    if reuse_dict is not None:
        sim_manager_params.update(reuse_dict[params.reuse.sim_manager_key])

    # Add node
    launch_entities: List[LaunchDescriptionEntity] = []
    planner_interface_node = Node(
        package='planner_interface',
        executable='sim_manager_node',
        name=node_name,
        emulate_tty=True,
        output="screen",
        remappings=[
          ('/set_clock_params', '/set_clock_params'),
          ('/run_pdvg', params.pdvg_analysis_topic),
          ('/kalman_filter_reset_srv_topic', params.reset_srv_topic),
          ('/kalman_filter_start_pause_srv_topic', params.start_pause_srv_topic),
          ('/truth_state_listener_toggle_topic', params.truth_toggle_topic),
          ('/nav_state_listener_toggle_topic', params.nav_toggle_topic),
          ('/reset_truth_listener', params.truth_reset_topic),
          ('/reset_nav_listener', params.nav_reset_topic),
          ('/reset_state', params.sense_state_reset_topic),
          ('/toggle_execution', params.toggle_dynamics_topic),
          ('/reset_autopilot', params.reset_autopilot_topic),
          ('/end_path_reached', params.end_path_reached_topic),
          ('/path_ready', params.path_ready_topic),
          ('/diagnostics', params.diagnostics_topic),
        ],
        parameters=[sim_manager_params],
    )
    launch_entities.append(planner_interface_node)
    # Add max simulation time timer
    if max_sim_time_sec > 0:
        launch_entities.append(RegisterEventHandler(
            OnProcessStart(
                target_action=planner_interface_node,
                on_start=[
                    SetUseSimTime(value=True),
                    RosTimer(period=max_sim_time_sec, actions=[launch.actions.Shutdown()]),
                ]
            )
        ))

    return launch_entities

def create_buoy_publisher(params: ScenarioParameters, node_name: str,
                          info_topic: str, marker_topic: str, reuse_dict: dict = None) -> List[LaunchDescriptionEntity]:
    """
    Creates the buoy publisher

    Args:
    - buoy_params: defines the buoy locations and uncertainties, etc.
    - node_name: name of the node to be created
    - info_topic: name of the topic for publishing the buoy locations
    - marker_topic: name of the topic for publishing the visualization messages
    - params: contains the simulation time flag
    - reuse_dict: Dictionary containing parameter values from previous simulations

    Returns:
    - LaunchDescriptionEntity containing a buoy_publisher node.
      Node configuration is dependent on if a config file was
      given to the 'scenario' input.
    """
    buoy_params = node_dict.buoy_publisher(params)
    if reuse_dict is not None:
        buoy_params.update(reuse_dict[params.reuse.buoy_publisher_key])

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

def save_params_file(params: ScenarioParameters, data_dir: str, reuse_dict: dict = None):
    """
    Create the parameters file

    Args:
    - params: Parameters for the current simulation
    - data_dir: Directory in which to save the parameters file
    - reuse_dict: Dictonary for reusing parameters from previous tests
    """

    # Make pretty print YAML dictionary to store parameters
    params_dict = node_dict.all_params_dict(params)
    if reuse_dict is not None:
        # Update all relevant keys
        params_dict[params.reuse.buoy_publisher_key].update(reuse_dict[params.reuse.buoy_publisher_key])
        params_dict[params.reuse.sim_manager_key].update(reuse_dict[params.reuse.sim_manager_key])
        params_dict[params.reuse.kalman_filter_node_key].update(reuse_dict[params.reuse.kalman_filter_node_key])
        params_dict[params.reuse.nav_state_listener_key].update(reuse_dict[params.reuse.nav_state_listener_key])
        params_dict[params.reuse.truth_state_listener_key].update(reuse_dict[params.reuse.truth_state_listener_key])
        params_dict[params.reuse.pdvg_interface_key].update(reuse_dict[params.reuse.pdvg_interface_key])
        params_dict[params.reuse.gps_denied_monitor_key].update(reuse_dict[params.reuse.gps_denied_monitor_key])
        params_dict[params.reuse.feature_sensor_node_key].update(reuse_dict[params.reuse.feature_sensor_node_key])
        params_dict[params.reuse.ch07_sensors_auto_dyn_key].update(reuse_dict[params.reuse.ch07_sensors_auto_dyn_key])


    dir = Path(data_dir)
    str_file = dir / 'params.yaml'
    with open(str(str_file), 'w') as file:
        yaml.dump(data=params_dict, stream=file, indent=2, default_flow_style=False)
