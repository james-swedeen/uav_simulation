"""Generate the buildings of Manhattan and plan paths through them"""

from launch import LaunchDescription
from pd_planner_launch.launch.manhattan_path_planner import manhattan_path_planner

def generate_launch_description() -> LaunchDescription:
    """
    Creates the Manhattan buildings in rviz2 and plans a path to a user specified
    point.

    Returns:
    - LaunchDescription with list of nodes to be spun when launch called.
    """
    return manhattan_path_planner()

