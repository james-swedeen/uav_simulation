import rclpy
from rclpy.node import Node
from uav_interfaces.msg import AutoPilotCommands
from sensor_msgs.msg import Joy
import numpy as np

class JoyAutopilotCommand(Node):
    """ Produces commands for the raw control of the uav

        Subscriptions:
            /joy: (sensor_msgs/msg/Joy)

        Publications:
            /autopilot_command: (uav_interfaces/msg/AutoPilotCommands) Commanded values for aircraft
    """

    def __init__(self) -> None:
        """ Initializes the publications, subscriptions, and node parameters
        """
        super().__init__(node_name="joy_autopilot_command")

        # Create publisher and subscriber
        self._pub_command = self.create_publisher(AutoPilotCommands, "autopilot_command", 1) # For control surface command
        self._sub_joy = self.create_subscription(Joy, "/joy", self._joy_callback, 1)

        # Define commands to issue when zero control is input
        self._airspeed_zero: float = 25. # m/s
        self._course_zero: float = 0. # radians
        self._altitude_zero: float = 100. # meters

        # Define the max deviation
        self._airspeed_max_dev: float = 10. # m/s
        self._course_max_dev: float = 3. # Radians (not all the way to pi to avoid wrap around)
        self._altitude_max_dev: float = 50. # meters

        # Define the indices for commands within the joy.axes
        self._ind_airspeed: int = 1
        self._ind_course: int = 0
        self._ind_altitude: int = 4
        self._ind_set: int = 5

        # Flags for set
        self._set_active = False # Set to true when the new values are being set


    def _joy_callback(self, msg: Joy) -> None:
        '''Stores the joy command
        '''
        # Initialize the command
        cmd = AutoPilotCommands()

        # Create the command based on the inputs
        cmd.airspeed_command = self._airspeed_zero + msg.axes[self._ind_airspeed]*self._airspeed_max_dev
        cmd.course_command = self._course_zero - msg.axes[self._ind_course]*self._course_max_dev
        cmd.altitude_command = self._altitude_zero + msg.axes[self._ind_altitude]*self._altitude_max_dev
        cmd.phi_feedforward = 0.

        # Check the set button
        if msg.axes[self._ind_set] < 1.:
            # Process a new set point (i.e., store the command)
            if not self._set_active:
                self._airspeed_zero = cmd.airspeed_command
                self._course_zero = cmd.course_command
                self._altitude_zero = cmd.altitude_command
                self._set_active = True

            # Set the command to be the stored value instead of listening to the buttons
            # This avoids repeatedly storing a new value
            cmd.airspeed_command = self._airspeed_zero
            cmd.course_command = self._course_zero
            cmd.altitude_command = self._altitude_zero
        else:
            self._set_active = False

        self._pub_command.publish(cmd)



def main(args=None):
    rclpy.init(args=args)

    # Create the kinematic node
    joy_node = JoyAutopilotCommand()
    rclpy.spin(joy_node)

    # Shutdown the node
    joy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()