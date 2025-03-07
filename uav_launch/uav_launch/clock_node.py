import rclpy
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
import time as pytime
from typing import Callable, Protocol
from uav_interfaces.srv import SetClockParams

class RosInterface(Protocol):
    """Defined the interface for a general ros interface
    """
    def take_step(self) -> Time:
        ...
class TsClass(Protocol):
    """Defines the interface to a class that has a time step property
    """
    ts: float # Time step

InterfaceConstructor = Callable[[float, Time, Node], tuple[RosInterface,TsClass]]


def run_clock_node(node_name: str, get_ros_interface: InterfaceConstructor) -> None:
    """ Runs the clock with a ros interface that has a take_step function.

    Inputs:
        node_name: Name of the node to be created
        get_ros_interfase: Returns a RosInterface node and structure with the desired time step
    """
    # Initialize ros
    rclpy.init(args=None)
    node = rclpy.create_node(node_name=node_name)

    # Create the clock service function (note that loop_period_ms is ignored)
    wall_time_scale = 1. # wall seconds / sim seconds
    def set_clock_params(req: SetClockParams.Request, res: SetClockParams.Response) -> SetClockParams.Response:
        """Stores the sim rate, but ignores everything else
        """
        if req.time_scale > 0:
            nonlocal wall_time_scale
            wall_time_scale = 1./req.time_scale # Note that time scale is sim seconds / wall seconds
            res.success = True
        else:
            res.success = False
            node.get_logger().error("Commanded time scale not positive")
        return res

    # Create the clock adjustment service service
    clock_server = node.create_service(SetClockParams, "set_clock_params",set_clock_params)

    # Initialize the clock variables
    time_latest = Time()
    clock_msg = Clock()
    clock_msg.clock = time_latest.to_msg()
    pub_clock = node.create_publisher(Clock, "/clock", 1)
    pub_clock.publish(clock_msg)

    # Create the executor
    exec = SingleThreadedExecutor()
    exec.add_node(node)

    # Initialize the dynamics node
    (ros_interface, ts_class) = get_ros_interface(0.01, time_latest, node)

    # Wait for simulation to be initialized
    init_time_wait=0.01 # Seconds
    node.declare_parameter(name="initialization_wait_time", value=init_time_wait, descriptor=ParameterDescriptor(description='Amount of time in seconds this node will wait before starting the simulation'))
    init_time_wait = node.get_parameter("initialization_wait_time").value
    wall_time = pytime.perf_counter()
    desired_wall_time = wall_time + init_time_wait
    while rclpy.ok() and desired_wall_time > pytime.perf_counter():
        exec.spin_once(timeout_sec=max(0.0, desired_wall_time - pytime.perf_counter()))

    # Loop through and publish the clock
    wall_time = pytime.perf_counter()
    min_spin_count = 10.0
    while rclpy.ok():
        # Update the sim
        time_latest = ros_interface.take_step()

        # Update the clock
        clock_msg.clock = time_latest.to_msg()
        pub_clock.publish(clock_msg)

        # Wait until time is up
        desired_wall_time = wall_time + ts_class.ts * wall_time_scale
        spin_count = 0
        while desired_wall_time > pytime.perf_counter() or spin_count < min_spin_count:
            spin_count = spin_count + 1
            exec.spin_once(timeout_sec=max(0.0, (desired_wall_time - pytime.perf_counter())/min_spin_count))
        wall_time = desired_wall_time

    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()
