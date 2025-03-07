"""Generate a wide area scenario for uav navigation testing"""
from launch import LaunchDescription

from pd_planner_launch.launch.gps_denied_scenario import gps_denied_scenario

def generate_launch_description() -> LaunchDescription:
    """
    Creates scenario to use with a buoy_publisher node. Starts a buoy_publisher node
    and a rviz2 node to display buoy Markers and radii Markers.

    Returns:
    - LaunchDescription with list of nodes to be spun when launch called.
    """
    return gps_denied_scenario()
