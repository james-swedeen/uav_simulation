import rclpy
from rclpy.node import Node
from uav_interfaces.msg import ControlSurfaceCommands
from sensor_msgs.msg import Joy
import numpy as np

class JoyControlCommand(Node):
    """ Produces commands for the raw control of the uav

        Subscriptions:
            /joy: (sensor_msgs/msg/Joy)

        Publications:
            /command: (uav_interfaces/msg/ControlSurfaceCommands) Control surface commands for the MAV
    """

    def __init__(self) -> None:
        """ Initializes the publications, subscriptions, and node parameters
        """
        super().__init__(node_name="joy_control_command")

        # Create publisher and subscriber
        self._pub_command = self.create_publisher(ControlSurfaceCommands, "command", 1)
        self._sub_joy = self.create_subscription(Joy, "/joy", self._joy_callback, 1)

        # Define commands to issue when zero control is input (defaulted to trim input with Va = 25, gamma=0, R= inf)
        self._elevator_zero: float = -.1250436498
        self._aileron_zero: float = 0.0018375
        self._rudder_zero: float = -0.0002929
        self._throttle_zero: float = 0.6767759

        # Define the max deviation
        self._elevator_max_dev: float = 0.25
        self._aileron_max_dev: float = 0.25
        self._rudder_max_dev: float = 0.25
        self._throttle_max_dev: float = 1.-self._throttle_zero

        # Define the indices for commands within the joy.axes
        self._ind_throttle: int = 1
        self._ind_aileron: int = 0
        self._ind_elevator: int = 4
        self._ind_rudder: int = 3


    def _joy_callback(self, msg: Joy) -> None:
        '''Stores the joy command
        '''
        # Initialize the command
        cmd = ControlSurfaceCommands()

        # Create the command based on the inputs
        cmd.throttle = self._throttle_zero + msg.axes[self._ind_throttle]*self._throttle_max_dev
        cmd.aileron = self._aileron_zero + msg.axes[self._ind_aileron]*self._aileron_max_dev
        cmd.elevator = self._elevator_zero + msg.axes[self._ind_elevator]*self._elevator_max_dev
        cmd.rudder = self._rudder_zero + msg.axes[self._ind_rudder]*self._rudder_max_dev

        self._pub_command.publish(cmd)



def main(args=None):
    rclpy.init(args=args)

    # Create the kinematic node
    joy_node = JoyControlCommand()
    rclpy.spin(joy_node)

    # Shutdown the node
    joy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()