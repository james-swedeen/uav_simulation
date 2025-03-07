"""Generate a PDVG scenario for uav navigation testing"""
from launch import LaunchDescription

from pd_planner_launch.launch.pdvg_scenario import pdvg_scenario

def generate_launch_description() -> LaunchDescription:
    """
    Creates PDVG scenario to provide a simple PDVG planner world for testing.

    Returns:
    - LaunchDescription with list of nodes to be spun when launch called.
    """
    return pdvg_scenario()
