"""Generate a headless simulation scenario for data collection"""
from launch import LaunchDescription
from pd_planner_launch.launch.headless_scenarios import line_gps_ps_select
import sys

def generate_launch_description() -> LaunchDescription:
    """
    Creates headless sim scenario to provide a simple headless environment for testing.

    Returns:
    - LaunchDescription with list of nodes to be spun when launch called.
    """
    launch_description: LaunchDescription

    # Set up values for running specific scenarios
    reuse_file = None

    for arg in sys.argv:
      if arg.startswith("params_file:="):
        reuse_file = arg.split(':=')[1]
        print(f"Reuse file selected: {reuse_file}")


    launch_description = line_gps_ps_select.line_gps_ps_select(reuse_file)
    return launch_description
