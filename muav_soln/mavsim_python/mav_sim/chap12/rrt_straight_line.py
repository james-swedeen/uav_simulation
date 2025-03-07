"""
rrt straight line path planner for mavsim_python

mavsim_python
    - Beard & McLain, PUP, 2012
    - Last updated:
        4/3/2019 - Brady Moon
        4/11/2019 - RWB
        3/31/2020 - RWB
        4/2022 - GND
"""
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from mav_sim.chap11.draw_waypoints import DrawWaypoints
from mav_sim.chap12.draw_map import DrawMap
from mav_sim.chap12.planner_utilities import (
    column,
    distance,
    exist_feasible_path,
    find_closest_configuration,
    find_shortest_path,
    generate_random_configuration,
    plan_path,
    smooth_path,
)
from mav_sim.message_types.msg_waypoints import MsgWaypoints
from mav_sim.message_types.msg_world_map import MsgWorldMap
from mav_sim.tools.types import NP_MAT


class RRTStraightLine:
    """RRT planner for straight line plans
    """
    def __init__(self) -> None:
        """Initialize parameters
        """
        self.segment_length = 300 # standard length of path segments
        self.plot_window: gl.GLViewWidget
        self.plot_app: pg.QtGui.QApplication


    def plot_map(self, world_map: MsgWorldMap, tree: MsgWaypoints, waypoints: MsgWaypoints, \
        smoothed_waypoints: MsgWaypoints, radius: float) -> None:
        """Plots the RRT tree

        Args:
            world_map: definition of the world for planning
            tree: Current set of waypoints in rrt search tree
            waypoints: Non-smoothed, minimum length path
            smoothed_waypoints: The path (waypoints) after smoothing
            radius: minimum radius circle for the mav
        """
        scale = 4000
        # initialize Qt gui application and window
        self.plot_app = pg.QtGui.QApplication([])  # initialize QT
        self.plot_window = gl.GLViewWidget()  # initialize the view object
        self.plot_window.setWindowTitle('World Viewer')
        self.plot_window.setGeometry(0, 0, 1500, 1500)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(scale/20, scale/20, scale/20) # set the size of the grid (distance between each line)
        self.plot_window.addItem(grid) # add grid to viewer
        self.plot_window.setCameraPosition(distance=scale, elevation=50, azimuth=-90)
        self.plot_window.setBackgroundColor('k')  # set background color to black
        self.plot_window.show()  # display configured window
        #self.plot_window.raise_() # bring window to the front

        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[204, 0, 0]])/255.
        green = np.array([[0, 153, 51]])/255.
        DrawMap(world_map, self.plot_window)
        DrawWaypoints(waypoints, radius, blue, self.plot_window)
        DrawWaypoints(smoothed_waypoints, radius, red, self.plot_window)
        draw_tree(tree, green, self.plot_window)
        # draw things to the screen
        self.plot_app.processEvents()

    def update(self, start_pose: NP_MAT, end_pose: NP_MAT, Va: float, world_map: MsgWorldMap, num_paths: int = 5) -> MsgWaypoints:
        """ Creates a plan from the start pose to the end pose.

        Args:
            start_pose: starting pose of the mav
            end_pose: desired end pose of the mav
            Va: airspeed
            world_map: definition of the world for planning
            num_paths: Number of paths to find before selecting the best path

        Returns:
            waypoints: Waypoints defining the planned path
        """
        return create_rrt_plan(start_pose=start_pose, end_pose=end_pose, \
            Va=Va, world_map=world_map, segment_length=self.segment_length, \
            num_paths=num_paths)

def create_rrt_plan(start_pose: NP_MAT, end_pose: NP_MAT, Va: float, \
    world_map: MsgWorldMap, segment_length: float, num_paths: int = 5) -> MsgWaypoints:
    """Update the plan using fillets for basic motion primitives

    Implements Algorithm 12 with a small modification. Instead of stopping
    once the first path is found to the goal, it stops once `num_paths` different
    paths have been found and then selects the shortest path found for the return.

    Args:
        start_pose: starting pose of the mav
        end_pose: desired end pose of the mav
        Va: airspeed
        world_map: definition of the world for planning
        segment_length: standard length of path segments - maximum segment length
        num_paths: Number of paths to find before selecting the best path

    Returns:
        waypoints: Waypoints defining the planned path
    """
    # Initialize the tree (Algorithm 12 line 1)
    tree = MsgWaypoints()
    tree.type = 'fillet' # Could also be: tree.type = 'straight_line'
    tree.add(ned=start_pose, airspeed=Va) # add the start pose to the tree

    # check to see if start_pose connects directly to end_pose
    if (distance(start_pose, end_pose) < segment_length) \
            and exist_feasible_path(start_pose, end_pose, world_map):
        tree.add(ned=end_pose, airspeed=Va,
                    cost=distance(start_pose, end_pose),
                    parent=1, connect_to_goal=1)
        return tree

    # Algorithm 12: Plan RRT Path
    path_count = 0
    while path_count < num_paths: # Slightly modified version of Algorithm 12 line 2
        # Generate random configuration (Algorithm 12 line 3)
        random_pose_ = generate_random_configuration(world_map, end_pose.item(2))

        # find leaf on tree that is closest to random_pose (Algorithm 12 line 4)
        (pos_closest, idx, dist) = find_closest_configuration(tree, random_pose_)

        # planPath() Return the node that will be attempted to connect to tree (Algorithm 12 line 5)
        (new_pose, dist_to_new) = plan_path(pos_closest, random_pose_, segment_length, dist)

        # Check to see if a path from the closest point to the new point is feasible
        # Algorithm 12 lines 6-9
        flag_added = False # Flag for storing whether or not the node is added
        if exist_feasible_path(tree.get_ned(idx), new_pose, world_map):
            cost = tree.cost.item(idx) + dist_to_new # cost of getting to new node through tree
            tree.add(ned=new_pose, airspeed=Va, cost=cost,
                        parent=idx, connect_to_goal=0)
            flag_added = True

        # check to see if new node connects directly to end_node (Algorithm 12 lines 10-13)
        flag_connected = False # Flag indicating whether or not the node connects to the goal
        if flag_added and (distance(new_pose, end_pose) < segment_length)\
                and exist_feasible_path(new_pose, end_pose, world_map):
            tree.connect_to_goal[-1] = True  # mark node as connecting to end.
            flag_connected = True

        # Augment the number of paths found to the goal
        path_count = path_count + flag_connected

    # find path with minimum cost to end_node
    waypoints_not_smooth = find_shortest_path(tree, end_pose)

    # Smooth waypoint path prior to return
    waypoints = smooth_path(waypoints_not_smooth, world_map)
    return waypoints

def draw_tree(tree: MsgWaypoints, color: NP_MAT, window: gl.GLViewWidget) -> None:
    """Draw the tree in the given window

    Args:
        tree: Current set of waypoints in rrt search tree
        color: color of tree
        window: window in which to plot the tree
    """
    R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    points = R @ tree.ned
    for i in range(points.shape[1]):
        line_color = np.tile(color, (2, 1))
        parent = int(tree.parent.item(i))
        line_pts = np.concatenate((column(points, i).T, column(points, parent).T), axis=0)
        line = gl.GLLinePlotItem(pos=line_pts,
                                 color=line_color,
                                 width=2,
                                 antialias=True,
                                 mode='line_strip')
        window.addItem(line)
