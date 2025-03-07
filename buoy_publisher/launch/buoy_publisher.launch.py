"""Generate a launch description to demonstrate simple functionality of buoy_publisher"""
import os

import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionEntity
from launch_ros.actions import Node


class ScenarioDefinition():
    """
    Defines variables for setting up a scenario
    """
    def __init__(self) -> None:
        self.config_file: str                                   # Location of config file
        self.header_frame_id: str                               # Frame in which to publish buoys

        # The following lists must all have the same length. The same-numbered element of each
        # list corresponds to the same buoy
        self.buoy_locations_x: list[float]                      # List of buoy x-positions
        self.buoy_locations_y: list[float]                      # List of buoy y-positions
        self.buoy_ranges: list[float]                           # List of buoy ranges
        self.pose_uncertainty_x: list[float]                    # List of x-position uncertainties
        self.pose_uncertainty_y: list[float]                    # List of y-position uncertainties
        self.pose_uncertainty_z: list[float]                    # List of z-position uncertainties
        self.buoy_color: tuple[float, float, float, float]      # [R,G,B,A] color values that apply to all buoy markers
        self.radius_color: tuple[float, float, float, float]    # [R,G,B,A] color values that apply to all buoy radius markers


def generate_scenario_from_yaml(yaml_file: str) -> ScenarioDefinition:
    """
    Generate and return a ScenarioDefinition object based on config file

    Parameters:
    - yaml_file: String containing full path to yaml file that will be used a the config file

    Returns:
    - ScenarioDefinition object with config file information specified
    """
    scenario = ScenarioDefinition()
    scenario.config_file = yaml_file

    return scenario

def generate_temp_scenario() -> ScenarioDefinition:
    """
    Generate and return a ScenarioDefinition object with arbitary circle pattern
    to highlight the features of the buoy publisher

    Returns:
    - ScenarioDefinition object with arbitrary specification for buoys and markers
    """
    scenario = ScenarioDefinition()
    scenario.buoy_locations_x = [5*np.cos(x*np.pi/4) for x in range(0,8)]
    scenario.buoy_locations_y = [5*np.sin(x*np.pi/4) for x in range(0,8)]
    scenario.buoy_ranges = np.arange(1.0,5.0,0.5).tolist()
    scenario.pose_uncertainty_x = [0.1, 0.2, 0.3, 0.4]
    scenario.pose_uncertainty_y = [0.1, 0.2, 0.3, 0.4]
    scenario.pose_uncertainty_z = [0.1, 0.2, 0.3, 0.4]
    scenario.buoy_color = (170/255, 5/255, 5/255, 1.0)
    scenario.radius_color = (103/255, 199/255, 235/255, 0.25)
    scenario.header_frame_id = 'map'

    return scenario


def generate_launch_description() -> LaunchDescription:
    """
    Creates scenario to use with a buoy_publisher node. Starts a buoy_publisher node
    and a rviz2 node to display buoy Markers and radii Markers.

    Returns:
    - LaunchDescription with list of nodes to be spun when launch called.
    """
    
    # Generate scenarios using config file if desired
    buoy_pub_dir = get_package_share_directory('buoy_publisher')
    
    # Launch parameters
    use_rviz = True

    # scenario = generate_temp_scenario()
    config_file = os.path.join(buoy_pub_dir, 'buoy_config.yaml')
    scenario = generate_scenario_from_yaml(config_file)

    launch_entities: list[LaunchDescriptionEntity] = []

    # Create Nodes
    # Publisher itself
    print("Creating buoy publisher...")
    launch_entities += create_buoy_publisher(scenario=scenario)

    # Visualization
    print("Creating vizualization nodes...")
    launch_entities += create_visualization(use_rviz=use_rviz, rviz_parent_dir=buoy_pub_dir)

    print("Launching...")
    return LaunchDescription(launch_entities)


def create_buoy_publisher(scenario: ScenarioDefinition) -> list[LaunchDescriptionEntity]:
    """
    Creates the buoy publisher
    
    Args:
    - scenario: ScenarioDefinition object containing relevant information
                for starting sim
    
    Returns:
    - LaunchDescriptionEntity containing a buoy_publisher node.
      Node configuration is dependent on if a config file was
      given to the 'scenario' input.
    """

    launch_entities: list[LaunchDescriptionEntity] = []
    if scenario.config_file:    # Use config file for parameters
        launch_entities.append(
            Node(
                package='buoy_publisher',
                executable='buoy_publisher',
                name='buoy_publisher',
                parameters=[scenario.config_file]
            )
        )
    else:                       # Otherwise use defaults
        launch_entities.append(
            Node(
                package='buoy_publisher',
                executable='buoy_publisher',
                name='buoy_publisher',
                parameters=[{
                    'buoy_locations_x': scenario.buoy_locations_x,
                    'buoy_locations_y': scenario.buoy_locations_y,
                    'buoy_ranges': scenario.buoy_ranges,
                    'pose_uncertainty_x': scenario.pose_uncertainty_x,
                    'pose_uncertainty_y': scenario.pose_uncertainty_y,
                    'pose_uncertainty_z': scenario.pose_uncertainty_z,
                    'header_frame_id': scenario.header_frame_id,
                    'buoy_color': scenario.buoy_color,
                    'radius_color': scenario.radius_color
                }]
            )
        )

    return launch_entities

def create_visualization(use_rviz: bool, rviz_parent_dir: str) -> list[LaunchDescriptionEntity]:
    """
    Creates nodes for visualization depending specified type

    Args:
    - use_rviz:         If true -> Launch with rviz2 and config file || false -> Don't run with rviz2
    - rviz_parent_dir:  Parent directory of the rviz config file to be used

    Returns:
    - LaunchDescriptionEntity containing an rviz2 node loaded with a config file
    """

    launch_entities: list[LaunchDescriptionEntity] = []

    if use_rviz:
        launch_entities.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', [os.path.join(rviz_parent_dir, 'marker_viewer.rviz')]]
            )
        )
    
    return launch_entities
