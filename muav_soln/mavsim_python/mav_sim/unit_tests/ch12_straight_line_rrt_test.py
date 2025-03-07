"""ch12_straight_line_rrt_tests.py: Implements some basic tests for the chapter 12 martial."""
# pylint: disable=too-many-lines

import gc
import os
import pickle
from copy import deepcopy
from typing import Any, Dict, List, Tuple, Union, cast

from mav_sim.chap12.planner_utilities import (
    distance,
    exist_feasible_path,
    find_closest_configuration,
    find_shortest_path,
    generate_random_configuration,
    height_above_ground,
    plan_path,
    smooth_path,
)
from mav_sim.message_types.msg_waypoints import MsgWaypoints
from mav_sim.message_types.msg_world_map import MsgWorldMap

PRECISION = 1e-6

import numpy as np


def gen_rrt_straight_line_vars() -> Dict[str, List[Tuple[Dict[str, Any], Any]]]:
    """Generate test inputs for the rrt_straight_line related functions."""
    output: Dict[str, List[Tuple[Dict[str, Any], Any]]] = {}
    output["plan_path"] = []
    output["find_closest_configuration"] = []
    output["find_shortest_path"] = []
    output["smooth_path"] = []
    # Exhaustive cases
    for pd in np.arange(-125, -100.1, 25):
        for Va in np.arange(25, 35.1, 10):
            for segment_length in np.arange(250, 350, 100):
                for num_paths in np.arange(1, 5, 1):
                    # Inputs to rrt algorithm
                    world_map = MsgWorldMap()
                    start_pose = generate_random_configuration(world_map, pd)
                    while height_above_ground(world_map, start_pose) <= 0:
                        start_pose = generate_random_configuration(world_map, pd)
                    end_pose = generate_random_configuration(world_map, pd)
                    while height_above_ground(world_map, start_pose) <= 0:
                        end_pose = generate_random_configuration(world_map, pd)

                    # Initialize the tree (Algorithm 12 line 1)
                    tree = MsgWaypoints()
                    tree.type = "fillet"  # Could also be: tree.type = 'straight_line'
                    tree.add(
                        ned=start_pose, airspeed=Va
                    )  # add the start pose to the tree

                    # Algorithm 12: Plan RRT Path
                    path_count = 0
                    while (
                        path_count < num_paths
                    ):  # Slightly modified version of Algorithm 12 line 2
                        # Generate random configuration (Algorithm 12 line 3)
                        random_pose_ = generate_random_configuration(
                            world_map, end_pose.item(2)
                        )

                        # find leaf on tree that is closest to random_pose (Algorithm 12 line 4)
                        inputs = {
                            "tree": deepcopy(tree),
                            "pos_in": random_pose_,
                        }
                        outputs = find_closest_configuration(tree, random_pose_)
                        output["find_closest_configuration"].append((inputs, outputs))
                        (pos_closest, idx, dist) = outputs

                        # planPath() Return the node that will be attempted to connect to tree (Algorithm 12 line 5)
                        inputs = {
                            "start_point": pos_closest,
                            "desired_point": random_pose_,
                            "max_edge_length": segment_length,
                            "dist": dist,
                        }
                        outputs = plan_path(**inputs)
                        output["plan_path"].append((inputs, outputs))
                        (new_pose, dist_to_new) = outputs

                        # Check to see if a path from the closest point to the new point is feasible
                        # Algorithm 12 lines 6-9
                        flag_added = (
                            False  # Flag for storing whether or not the node is added
                        )
                        if exist_feasible_path(tree.get_ned(idx), new_pose, world_map):
                            cost = (
                                tree.cost.item(idx) + dist_to_new
                            )  # cost of getting to new node through tree
                            tree.add(
                                ned=new_pose,
                                airspeed=Va,
                                cost=cost,
                                parent=idx,
                                connect_to_goal=0,
                            )
                            flag_added = True

                        # check to see if new node connects directly to end_node (Algorithm 12 lines 10-13)
                        flag_connected = False  # Flag indicating whether or not the node connects to the goal
                        if (
                            flag_added
                            and (distance(new_pose, end_pose) < segment_length)
                            and exist_feasible_path(new_pose, end_pose, world_map)
                        ):
                            tree.connect_to_goal[
                                -1
                            ] = True  # mark node as connecting to end.
                            flag_connected = True

                        # Augment the number of paths found to the goal
                        path_count = path_count + flag_connected

                    # find path with minimum cost to end_node
                    inputs = {
                        "tree": deepcopy(tree),
                        "end_pose": end_pose,
                    }
                    waypoints_not_smooth = find_shortest_path(**inputs)
                    output["find_shortest_path"].append((inputs, waypoints_not_smooth))

                    # Smooth waypoint path prior to return
                    inputs = {
                        "waypoints": waypoints_not_smooth,
                        "world_map": world_map,
                    }
                    waypoints = smooth_path(**inputs)
                    output["smooth_path"].append((inputs, waypoints))

    return output


def plan_path_test(test_case: Tuple[Dict[str, Any], Any]) -> bool:
    """Test the plan_path function."""
    cal_output = plan_path(**test_case[0])

    if (
        (
            np.array(cal_output[0]).reshape(-1, 1)
            - np.array(test_case[1][0]).reshape(-1, 1)
        )
        > PRECISION
    ).any() or (np.abs(cal_output[1] - test_case[1][1]) > PRECISION):
        print("Failed test!")
        print("Calculated output:")
        print("new_point: " + str(cal_output[0]))
        print("dist_to_new: " + str(cal_output[1]))
        print("Expected output:")
        print("new_point: " + str(test_case[1][0]))
        print("dist_to_new: " + str(test_case[1][1]))
        return False
    return True


def find_closest_configuration_test(test_case: Tuple[Dict[str, Any], Any]) -> bool:
    """Test the find_closest_configuration function."""
    cal_output = find_closest_configuration(**test_case[0])

    if (
        (
            (
                np.array(cal_output[0]).reshape(-1, 1)
                - np.array(test_case[1][0]).reshape(-1, 1)
            )
            > PRECISION
        ).any()
        or (np.abs(cal_output[1] - test_case[1][1]) > PRECISION)
        or (np.abs(cal_output[2] - test_case[1][2]) > PRECISION)
    ):
        print("Failed test!")
        print("Calculated output:")
        print("pos_closest: " + str(cal_output[0]))
        print("ind: " + str(cal_output[1]))
        print("dist: " + str(cal_output[2]))
        print("Expected output:")
        print("pos_closest: " + str(test_case[1][0]))
        print("ind: " + str(test_case[1][1]))
        print("dist: " + str(test_case[1][2]))
        return False
    return True


def find_shortest_path_test(test_case: Tuple[Dict[str, Any], Any]) -> bool:
    """Test the find_shortest_path function."""
    cal_output = find_shortest_path(**test_case[0])

    if (
        (cal_output.flag_waypoints_changed != test_case[1].flag_waypoints_changed)
        or (cal_output.plot_updated != test_case[1].plot_updated)
        or (cal_output.type != test_case[1].type)
        or (cal_output.num_waypoints != test_case[1].num_waypoints)
        or ((cal_output.ned - test_case[1].ned) > PRECISION).any()
        or ((cal_output.airspeed - test_case[1].airspeed) > PRECISION).any()
        # or ((cal_output.course                 - test_case[1].course) > PRECISION).any()
        # or ((cal_output.cost - test_case[1].cost) > PRECISION).any()
        # or ((cal_output.parent - test_case[1].parent) > PRECISION).any()
        # or (
        #     (cal_output.connect_to_goal - test_case[1].connect_to_goal) > PRECISION
        # ).any()
    ):
        print("Failed test!")
        print("Calculated output:")
        print("flag_waypoints_changed: " + str(cal_output.flag_waypoints_changed))
        print("plot_updated: " + str(cal_output.plot_updated))
        print("type: " + str(cal_output.type))
        print("num_waypoints: " + str(cal_output.num_waypoints))
        print("ned: " + str(cal_output.ned))
        print("airspeed: " + str(cal_output.airspeed))
        print("course: " + str(cal_output.course))
        print("cost: " + str(cal_output.cost))
        print("parent: " + str(cal_output.parent))
        print("connect_to_goal: " + str(cal_output.connect_to_goal))
        print("Expected output:")
        print("flag_waypoints_changed: " + str(test_case[1].flag_waypoints_changed))
        print("plot_updated: " + str(test_case[1].plot_updated))
        print("type: " + str(test_case[1].type))
        print("num_waypoints: " + str(test_case[1].num_waypoints))
        print("ned: " + str(test_case[1].ned))
        print("airspeed: " + str(test_case[1].airspeed))
        print("course: " + str(test_case[1].course))
        print("cost: " + str(test_case[1].cost))
        print("parent: " + str(test_case[1].parent))
        print("connect_to_goal: " + str(test_case[1].connect_to_goal))
        return False
    return True


def smooth_path_test(test_case: Tuple[Dict[str, Any], Any]) -> bool:
    """Test the smooth_path function."""
    cal_output = smooth_path(**test_case[0])

    if (
        (cal_output.flag_waypoints_changed != test_case[1].flag_waypoints_changed)
        or (cal_output.plot_updated != test_case[1].plot_updated)
        or (cal_output.type != test_case[1].type)
        or (cal_output.num_waypoints != test_case[1].num_waypoints)
        or ((cal_output.ned - test_case[1].ned) > PRECISION).any()
        or ((cal_output.airspeed - test_case[1].airspeed) > PRECISION).any()
        # or ((cal_output.course                 - test_case[1].course) > PRECISION).any()
        or ((cal_output.cost - test_case[1].cost) > PRECISION).any()
        or ((cal_output.parent - test_case[1].parent) > PRECISION).any()
        or (
            (cal_output.connect_to_goal - test_case[1].connect_to_goal) > PRECISION
        ).any()
    ):
        print("Failed test!")
        print("Calculated output:")
        print("flag_waypoints_changed: " + str(cal_output.flag_waypoints_changed))
        print("plot_updated: " + str(cal_output.plot_updated))
        print("type: " + str(cal_output.type))
        print("num_waypoints: " + str(cal_output.num_waypoints))
        print("ned: " + str(cal_output.ned))
        print("airspeed: " + str(cal_output.airspeed))
        print("course: " + str(cal_output.course))
        print("cost: " + str(cal_output.cost))
        print("parent: " + str(cal_output.parent))
        print("connect_to_goal: " + str(cal_output.connect_to_goal))
        print("Expected output:")
        print("flag_waypoints_changed: " + str(test_case[1].flag_waypoints_changed))
        print("plot_updated: " + str(test_case[1].plot_updated))
        print("type: " + str(test_case[1].type))
        print("num_waypoints: " + str(test_case[1].num_waypoints))
        print("ned: " + str(test_case[1].ned))
        print("airspeed: " + str(test_case[1].airspeed))
        print("course: " + str(test_case[1].course))
        print("cost: " + str(test_case[1].cost))
        print("parent: " + str(test_case[1].parent))
        print("connect_to_goal: " + str(test_case[1].connect_to_goal))
        return False
    return True


def gen_test_archive() -> None:
    """Make the input/output archive of answers."""
    archive: Dict[str, List[Any]] = {}
    # Gather test cases
    print("Making archive")
    archive = gen_rrt_straight_line_vars()
    gc.enable()
    # Save to archive
    print("Saving archive")
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch12_test_archive.pkl"
        ),
        "wb",
    ) as file:
        pickle.dump(archive, file)
    gc.enable()


def run_tests(test_case: Union[int, None] = None) -> None:
    """Run tests.

    @Arguments
    test_case: Either None to run all tests or the test case that you want to run.
    """
    gc.disable()
    # Open archive
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch12_test_archive.pkl"
        ),
        "rb",
    ) as file:
        tests_archive = pickle.load(file)
    gc.enable()
    # Run tests
    print("Starting plan_path test")
    indexes: List[int] = []
    if test_case is None:
        indexes = cast(List[int], range(len(tests_archive["plan_path"])))
    elif test_case < len(tests_archive["plan_path"]):
        indexes = [test_case]
    for test_count in indexes:
        if not plan_path_test(tests_archive["plan_path"][test_count]):
            print("Failed on test id: " + str(test_count))
            break
    print("Starting find_closest_configuration test")
    indexes: List[int] = []
    if test_case is None:
        indexes = cast(
            List[int], range(len(tests_archive["find_closest_configuration"]))
        )
    elif test_case < len(tests_archive["find_closest_configuration"]):
        indexes = [test_case]
    for test_count in indexes:
        if not find_closest_configuration_test(
            tests_archive["find_closest_configuration"][test_count]
        ):
            print("Failed on test id: " + str(test_count))
            break
    print("Starting find_shortest_path test")
    indexes: List[int] = []
    if test_case is None:
        indexes = cast(List[int], range(len(tests_archive["find_shortest_path"])))
    elif test_case < len(tests_archive["find_shortest_path"]):
        indexes = [test_case]
    for test_count in indexes:
        if not find_shortest_path_test(tests_archive["find_shortest_path"][test_count]):
            print("Failed on test id: " + str(test_count))
            break
    print("Starting smooth_path test")
    indexes: List[int] = []
    if test_case is None:
        indexes = cast(List[int], range(len(tests_archive["smooth_path"])))
    elif test_case < len(tests_archive["smooth_path"]):
        indexes = [test_case]
    for test_count in indexes:
        if not smooth_path_test(tests_archive["smooth_path"][test_count]):
            print("Failed on test id: " + str(test_count))
            break
    print("End of test\n")


if __name__ == "__main__":
    #gen_test_archive()
    run_tests(None)
