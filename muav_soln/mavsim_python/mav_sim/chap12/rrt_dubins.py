"""
rrt dubins path planner for mavsim_python

mavsim_python
    - Beard & McLain, PUP, 2012
    - Last updated:
        4/16/2019 - RWB
        12/21 - GND
"""
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from mav_sim.chap11.draw_waypoints import DrawWaypoints
from mav_sim.chap11.dubins_parameters import DubinsParameters
from mav_sim.chap12.draw_map import DrawMap
from mav_sim.message_types.msg_waypoints import MsgWaypoints
from mav_sim.message_types.msg_world_map import MsgWorldMap
from mav_sim.tools.types import NP_MAT


class RRTDubins:
    """RRT planner using Dubin's paths as motion primitive
    """
    def __init__(self)-> None:
        """Initialize parameters
        """
        self.segment_length = 300  # standard length of path segments
        self.plot_window: gl.GLViewWidget
        self.plot_app: pg.QtGui.QApplication
        self.dubins_path = DubinsParameters()

    def update(self, start_pose: NP_MAT, end_pose: NP_MAT, Va: float, world_map: MsgWorldMap, radius: float) -> MsgWaypoints:
        """Update the plan using fillets for basic motion primitives

        Args:
            start_pose: starting pose of the mav
            end_pose: desired end pose of the mav
            Va: airspeed
            world_map: definition of the world for planning
            radius: minimum radius circle for the mav

        Returns:
            waypoints: Waypoints defining the planned path
        """
        tree = MsgWaypoints()
        tree.type = 'dubins'
        # add the start pose to the tree
        tree.add(ned=start_pose[0:3], airspeed=Va, course=start_pose.item(3))
        # check to see if start_pose connects directly to end_pose
        if self.segment_length > distance(start_pose, end_pose) >= 2 * radius \
                and self.collision(start_pose, end_pose, world_map, radius) is False:
            tree.add(ned=end_pose[0:3], airspeed=Va, course=end_pose.item(3),
                     cost=distance(start_pose, end_pose), parent=1, connect_to_goal=1)
        else:
            num_paths = 0
            while num_paths < 3:
                flag = self.extend_tree(tree, end_pose, Va, world_map, radius)
                num_paths = num_paths + flag
        # find path with minimum cost to end_node
        waypoints_not_smooth = find_minimum_path(tree, end_pose)
        waypoints = self.smooth_path(waypoints_not_smooth, world_map, radius)
        self.plot_map(world_map, tree, waypoints_not_smooth, waypoints, radius)
        return waypoints

    def extend_tree(self, tree: MsgWaypoints, end_pose: NP_MAT, Va: float, world_map: MsgWorldMap, radius: float) -> bool:
        """Extend the tree in a random direction

        Args:
            tree: Current set of waypoints in rrt search tree
            end_pose: Desired end pose, used for goal checking and for altitiude
            Va: airspeed
            world_map: definition of the world for planning
            radius: minimum radius circle for the mav

        Returns:
            flag: True if new node connects to the goal, false otherwise
        """
        # extend tree by randomly selecting pose and extending tree toward that pose
        flag1 = False
        while flag1 is False:
            random_pose_ = random_pose(world_map, end_pose.item(2))
            # find leaf on tree that is closest to new_pose
            tmp: NP_MAT = tree.ned-np.tile(random_pose_[0:3], (1, tree.num_waypoints))
            tmp1 = np.diag(tmp.T @ tmp)
            idx = int(np.argmin(tmp1))
            dist = tmp1.item(idx)
            L = np.max([np.min([np.sqrt(dist), self.segment_length]), 3*radius])
            cost = tree.cost.item(idx) + L
            tmp: NP_MAT = random_pose_[0:3]-column(tree.ned, idx)
            new_ned = column(tree.ned, idx) + L * (tmp / np.linalg.norm(tmp))
            new_chi = np.arctan2(new_ned.item(1) - tree.ned[1, idx],
                                 new_ned.item(0) - tree.ned[0, idx])
            new_pose = np.concatenate((new_ned, np.array([[new_chi]])), axis=0)
            tree_pose = np.concatenate((column(tree.ned, idx), np.array([[tree.course.item(idx)]])), axis=0)
            if self.collision(tree_pose, new_pose, world_map, radius) is False:
                tree.add(ned=new_pose[0:3], airspeed=Va, course=new_chi,
                         cost=cost, parent=idx, connect_to_goal=0)
                flag1=True
            # check to see if new node connects directly to end_node
            if (self.segment_length > distance(new_pose, end_pose) >= 2*radius)\
                    and self.collision(new_pose, end_pose, world_map, radius) is False:
                tree.connect_to_goal[-1] = 1  # mark node as connecting to end.
                flag = True
            else:
                flag = False
        return flag

    def points_along_path(self, Del: float) -> NP_MAT:
        """Returns a list of points along the dubins path at a given spacing

        Args:
            Del: angular spacing of the points along the path

        Returns:
            points: Points along path between start and end pose
        """
        initialize_points = True
        # points along start circle
        th1 = np.arctan2(self.dubins_path.p_s.item(1) - self.dubins_path.center_s.item(1),
                         self.dubins_path.p_s.item(0) - self.dubins_path.center_s.item(0))
        th1 = mod(th1)
        th2 = np.arctan2(self.dubins_path.r1.item(1) - self.dubins_path.center_s.item(1),
                         self.dubins_path.r1.item(0) - self.dubins_path.center_s.item(0))
        th2 = mod(th2)
        th = th1
        theta_list = [th]
        if self.dubins_path.dir_s > 0:
            if th1 >= th2:
                while th < th2 + 2 * np.pi:
                    th += Del
                    theta_list.append(th)
            else:
                while th < th2:
                    th += Del
                    theta_list.append(th)
        else:
            if th1 <= th2:
                while th > th2 - 2 * np.pi:
                    th -= Del
                    theta_list.append(th)
            else:
                while th > th2:
                    th -= Del
                    theta_list.append(th)

        if initialize_points:
            points = np.array([[self.dubins_path.center_s.item(0) + self.dubins_path.radius * np.cos(theta_list[0]),
                                self.dubins_path.center_s.item(1) + self.dubins_path.radius * np.sin(theta_list[0]),
                                self.dubins_path.center_s.item(2)]])
            initialize_points = False
        for angle in theta_list:
            new_point = np.array([[self.dubins_path.center_s.item(0) + self.dubins_path.radius * np.cos(angle),
                                   self.dubins_path.center_s.item(1) + self.dubins_path.radius * np.sin(angle),
                                   self.dubins_path.center_s.item(2)]])
            points = np.concatenate((points, new_point), axis=0)

        # points along straight line
        sig = 0.
        while sig <= 1:
            new_point = np.array([[(1 - sig) * self.dubins_path.r1.item(0) + sig * self.dubins_path.r2.item(0),
                                   (1 - sig) * self.dubins_path.r1.item(1) + sig * self.dubins_path.r2.item(1),
                                   (1 - sig) * self.dubins_path.r1.item(2) + sig * self.dubins_path.r2.item(2)]])
            points = np.concatenate((points, new_point), axis=0)
            sig += Del

        # points along end circle
        th2 = np.arctan2(self.dubins_path.p_e.item(1) - self.dubins_path.center_e.item(1),
                         self.dubins_path.p_e.item(0) - self.dubins_path.center_e.item(0))
        th2 = mod(th2)
        th1 = np.arctan2(self.dubins_path.r2.item(1) - self.dubins_path.center_e.item(1),
                         self.dubins_path.r2.item(0) - self.dubins_path.center_e.item(0))
        th1 = mod(th1)
        th = th1
        theta_list = [th]
        if self.dubins_path.dir_e > 0:
            if th1 >= th2:
                while th < th2 + 2 * np.pi:
                    th += Del
                    theta_list.append(th)
            else:
                while th < th2:
                    th += Del
                    theta_list.append(th)
        else:
            if th1 <= th2:
                while th > th2 - 2 * np.pi:
                    th -= Del
                    theta_list.append(th)
            else:
                while th > th2:
                    th -= Del
                    theta_list.append(th)
        for angle in theta_list:
            new_point = np.array([[self.dubins_path.center_e.item(0) + self.dubins_path.radius * np.cos(angle),
                                   self.dubins_path.center_e.item(1) + self.dubins_path.radius * np.sin(angle),
                                   self.dubins_path.center_e.item(2)]])
            points = np.concatenate((points, new_point), axis=0)
        return points

    def collision(self, start_pose: NP_MAT, end_pose: NP_MAT, world_map: MsgWorldMap, radius: float) -> bool:
        """Check to see of path from start_pose to end_pose colliding with world_map

        Args:
            start_pose: starting point on a line
            end_pose: ending point on a line
            world_map: definition of the world for planning
            radius: minimum radius circle for the mav

        Returns:
            collision_flag: True => a collision has occured, False => path is collision free
        """
        Del = 0.05
        self.dubins_path = DubinsParameters(start_pose[0:3], start_pose.item(3), end_pose[0:3], end_pose.item(3), radius)
        points = self.points_along_path(Del)
        for i in range(points.shape[1]):
            if height_above_ground(world_map, column(points, i)) <= 0:
                return True
        return False

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
        self.plot_window.raise_() # bring window to the front

        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[204, 0, 0]])/255.
        green = np.array([[0, 153, 51]])/255.
        DrawMap(world_map, self.plot_window)
        DrawWaypoints(waypoints, radius, blue, self.plot_window)
        DrawWaypoints(smoothed_waypoints, radius, red, self.plot_window)
        self.draw_tree(tree, radius, green)
        # draw things to the screen
        self.plot_app.processEvents()

    def draw_tree(self, tree: MsgWaypoints, radius: float, color: NP_MAT) -> None:
        """Draw the tree in the given window

        Args:
            tree: Current set of waypoints in rrt search tree
            radius: minimum radius circle for the mav
            color: color of tree
        """
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        Del = 0.05
        for i in range(1, tree.num_waypoints):
            parent = int(tree.parent.item(i))
            self.dubins_path = DubinsParameters(column(tree.ned, parent), tree.course[parent],
                                    column(tree.ned, i), tree.course[i], radius)
            points = self.points_along_path(Del)
            points = points @ R.T
            tree_color = np.tile(color, (points.shape[0], 1))
            tree_plot_object = gl.GLLinePlotItem(pos=points,
                                                color=tree_color,
                                                width=2,
                                                antialias=True,
                                                mode='line_strip')
            self.plot_window.addItem(tree_plot_object)

    def smooth_path(self, waypoints: MsgWaypoints, world_map: MsgWorldMap, radius: float) -> MsgWaypoints:
        """smooth the waypoint path

        Args:
            waypoints: The path to be smoothed
            world_map: definition of the world for planning
        """
        smooth = [0]  # add the first waypoint
        ptr = 1
        while ptr <= waypoints.num_waypoints - 2:
            start_pose = np.concatenate((column(waypoints.ned, smooth[-1]), np.array([[waypoints.course[smooth[-1]]]])),
                                        axis=0)
            end_pose = np.concatenate((column(waypoints.ned, ptr + 1), np.array([[waypoints.course[ptr + 1]]])), axis=0)
            if self.collision(start_pose, end_pose, world_map, radius) is True \
                    and distance(start_pose, end_pose) >= 2 * radius:
                smooth.append(ptr)
            ptr += 1
        smooth.append(waypoints.num_waypoints - 1)
        # construct smooth waypoint path
        smooth_waypoints = MsgWaypoints()
        for i in smooth:
            smooth_waypoints.add(column(waypoints.ned, i),
                                waypoints.airspeed.item(i),
                                waypoints.course.item(i),
                                np.inf,
                                -1, # Indicate that it has no parent
                                int(False)) # Indicate that not connected to goal
        smooth_waypoints.type = waypoints.type
        return smooth_waypoints


def find_minimum_path(tree: MsgWaypoints, end_pose: NP_MAT) -> MsgWaypoints:
    """Find the lowest cost path to the end node

    Args:
        tree: Current set of waypoints in rrt search tree
        end_pose: Desired end pose of the path

    Returns:
        waypoints: The shortest path
    """
    # find nodes that connect to end_node
    connecting_nodes = []
    for i in range(tree.num_waypoints):
        if tree.connect_to_goal.item(i) == 1:
            connecting_nodes.append(i)
    # find minimum cost last node
    idx = np.argmin(tree.cost[connecting_nodes])
    # construct lowest cost path order
    path = [connecting_nodes[idx]]  # last node that connects to end node
    parent_node = tree.parent.item(connecting_nodes[idx])
    while parent_node >= 1:
        path.insert(0, int(parent_node))
        parent_node = tree.parent.item(int(parent_node))
    path.insert(0, 0)
    # construct waypoint path
    waypoints = MsgWaypoints()
    for i in path:
        waypoints.add(column(tree.ned, i),
                      tree.airspeed.item(i),
                      tree.course.item(i),
                      np.inf,
                      -1, # Indicate that it has no parent
                      int(False)) # Indicate that not connected to goal
    waypoints.add(end_pose[0:3],
                  tree.airspeed[-1],
                  end_pose.item(3),
                  np.inf,
                  -1, # Indicate that it has no parent
                  int(False)) # Indicate that not connected to goal
    waypoints.type = tree.type
    return waypoints


def distance(start_pose: NP_MAT, end_pose: NP_MAT) -> float:
    """compute distance between start and end pose

    Args:
        start_pose: pose one
        end_pose: pose two

    Returns:
        d: distance between start and end poses
    """
    d = np.linalg.norm(start_pose[0:3] - end_pose[0:3])
    return float(d)


def height_above_ground(world_map: MsgWorldMap, point: NP_MAT) -> float:
    """find the altitude of point above ground level

    Args:
        world_map: definition of the world for planning
        point: location to calculate height

    Returns:
        h_agl: Height at the position (A negative value implies a collision)
    """
    point_height = -point.item(2)
    tmp = np.abs(point.item(0)-world_map.building_north)
    d_n = np.min(tmp)
    idx_n = np.argmin(tmp)
    tmp = np.abs(point.item(1)-world_map.building_east)
    d_e = np.min(tmp)
    idx_e = np.argmin(tmp)
    if (d_n<world_map.building_width) and (d_e<world_map.building_width):
        map_height = world_map.building_height[idx_n, idx_e]
    else:
        map_height = 0
    h_agl = point_height - map_height
    return float(h_agl)


def random_pose(world_map: MsgWorldMap, pd: float) -> NP_MAT:
    """Generates a random oriented pose in the world.

    The generated pose is generated randomly in the 2D plane with the
    down element (altitude) fixed

    Args:
        world_map: definition of the world for planning
        pd: The down position (i.e., altitude)
    """
    # generate a random pose
    pn = world_map.city_width * np.random.rand()
    pe = world_map.city_width * np.random.rand()
    chi = 0
    pose = np.array([[pn], [pe], [pd], [chi]])
    return pose


def mod(x: float) -> float:
    """
    force x to be between 0 and 2*pi
    """
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x


def column(A: NP_MAT, i: int) -> NP_MAT:
    """Extracts the ith column of A and return column vector
    """
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col
