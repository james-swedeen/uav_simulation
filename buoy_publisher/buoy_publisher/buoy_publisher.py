"""Node that publishes buoy information and associated markers for rviz2"""
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Quaternion, Vector3
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from uav_interfaces.msg import BuoyPoseRadiusArray
from visualization_msgs.msg import Marker, MarkerArray


class BuoyPublisher(Node):
    """
    BuoyPublisher publishes information about buoys via 2 message types containing
    pose and visual information.

    Publications:
        - /buoy_info:  PoseWithCovarianceStampedArray
            - Description:  Information on buoys contained in PoseWithCovarianceStampedArray message
            
        - /buoy_markers: MarkerArray
            - Description:  Pose of rviz2 cylinders that represent buoys and their ranges
    """

    def __init__(self) -> None:
        super().__init__('buoy_publisher')

        # Declare and intialize default parameters
        self._buoy_locations_x = [1.0, 2.0, 3.0, 4.0]
        self._buoy_locations_y = [1.0, 2.0, 3.0, 4.0]
        self._buoy_ranges = [2.0, 2.0, 2.0, 2.0]
        self._header_frame_id = 'map'
        self._buoy_marker_scale = 1.0
        self._buoy_color_param = [1.0, 0.39, 0.0, 1.0]
        self._radius_color_param = [1.0, 0.0, 0.0, 0.25]
        self._pose_uncertainty_x = [0.0, 0.0, 0.0, 0.0]
        self._pose_uncertainty_y = [0.0, 0.0, 0.0, 0.0]
        self._pose_uncertainty_z = [0.0, 0.0, 0.0, 0.0]

        # Flags
        self._scale_changed = False
        self._frame_id_changed = False


        self.declare_parameter(
            name="buoy_locations_x", 
            value=self._buoy_locations_x, 
            descriptor=ParameterDescriptor(description="Float list of buoy x-locations (m)"),
        )
        self.declare_parameter(
            name="buoy_locations_y", 
            value=self._buoy_locations_y, 
            descriptor=ParameterDescriptor(description="Float list of buoy y-locations (m)")
        )
        self.declare_parameter(
            name="buoy_ranges", 
            value=self._buoy_ranges, 
            descriptor=ParameterDescriptor(description="Float list of buoy ranges, expressed as a radius (m)")
        )
        self.declare_parameter(
            name="header_frame_id", 
            value=self._header_frame_id, 
            descriptor=ParameterDescriptor(description="Name of fixed frame to place buoy visualization in.")
        )
        self.declare_parameter(
            name="buoy_marker_scale", 
            value=self._buoy_marker_scale, 
            descriptor=ParameterDescriptor(description="Value by which to scale the buoy markers in rviz2.\
                                                        Does not scale transparent radius markers")
        )
        self.declare_parameter(
            name="buoy_color",
            value=self._buoy_color_param,
            descriptor=ParameterDescriptor(description="Color of buoy markers represented in RGBA format as [r,g,b,a]")
        )
        self.declare_parameter(
            name="radius_color",
            value=self._radius_color_param,
            descriptor=ParameterDescriptor(description="Color of buoy radii represented in RGBA format as [r,g,b,a]")
        )
        self.declare_parameter(
            name="pose_uncertainty_x",
            value=self._pose_uncertainty_x,
            descriptor=ParameterDescriptor(description="List of x-uncertainty on each buoy")
        )
        self.declare_parameter(
            name="pose_uncertainty_y",
            value=self._pose_uncertainty_y,
            descriptor=ParameterDescriptor(description="List of y-uncertainty on each buoy")
        )
        self.declare_parameter(
            name="pose_uncertainty_z",
            value=self._pose_uncertainty_z,
            descriptor=ParameterDescriptor(description="List of z-uncertainty on each buoy")
        )

        self.add_on_set_parameters_callback(self._param_callback)

        # Get parameters from config file
        self._buoy_locations_x = self.get_parameter("buoy_locations_x").value
        self._buoy_locations_y = self.get_parameter("buoy_locations_y").value
        self._buoy_ranges = self.get_parameter("buoy_ranges").value
        self._pose_uncertainty_x = self.get_parameter("pose_uncertainty_x").value
        self._pose_uncertainty_y = self.get_parameter("pose_uncertainty_y").value
        self._pose_uncertainty_z = self.get_parameter("pose_uncertainty_z").value
        self._buoy_color_param = self.get_parameter("buoy_color").value
        self._radius_color_param = self.get_parameter("radius_color").value

        # Make sure color inputs are the correct length
        assert len(self._buoy_color_param) == 4,\
            f"Length for buoy color list must be 4: [r, g, b, a], only {len(self._buoy_color_param)} provided"
        assert len(self._radius_color_param) == 4,\
            f"Length for radius color list must be 4: [r, g, b, a], only {len(self._radius_color_param)} provided"

        # Organize all other parameters to verify correct length
        len_compare_list = [self._buoy_locations_x, self._buoy_locations_y, self._buoy_ranges,\
                            self._pose_uncertainty_x, self._pose_uncertainty_y, self._pose_uncertainty_z]
        len_list = [len(x) for x in len_compare_list]
        
        # Verify these are the same length
        assert all((x==len_list[0] for x in len_list)), \
            f"Length of input parameter arrays are not the same!\n\
                Length of buoy_locations_x = {len(self._buoy_locations_x)}\n\
                Length of buoy_locations_y = {len(self._buoy_locations_y)}\n\
                Length of buoy_ranges = {len(self._buoy_ranges)}\n\
                Length of pose_uncertainty_x = {len(self._pose_uncertainty_x)}\n\
                Length of pose_uncertainty_y = {len(self._pose_uncertainty_y)}\n\
                Length of pose_uncertainty_z = {len(self._pose_uncertainty_z)}\n"


        # Assign final value of parameters
        self._buoy_color = ColorRGBA()
        self._buoy_color.r = self._buoy_color_param[0]
        self._buoy_color.g = self._buoy_color_param[1]
        self._buoy_color.b = self._buoy_color_param[2]
        self._buoy_color.a = self._buoy_color_param[3]

        self._radius_color = ColorRGBA()
        self._radius_color.r = self._radius_color_param[0]
        self._radius_color.g = self._radius_color_param[1]
        self._radius_color.b = self._radius_color_param[2]
        self._radius_color.a = self._radius_color_param[3]
        
        self._buoy_locations = np.column_stack((self._buoy_locations_x, self._buoy_locations_y))
        self._header_frame_id = self.get_parameter("header_frame_id").value

        self._pose_uncertainty = np.column_stack(\
            (self._pose_uncertainty_x, self._pose_uncertainty_y, self._pose_uncertainty_z))

        self._pos_publisher = self.create_publisher(BuoyPoseRadiusArray, 'buoy_info', 10)
        self._marker_pub = self.create_publisher(MarkerArray, 'buoy_markers', 10)
        timer_period = 1 # 1 second between callbacks
        self._buoy_info = self.create_timer(timer_period, self._timer_callback)

        # Declare objects to be published
        self._buoy_msgs = BuoyPoseRadiusArray()
        self._buoy_markers = MarkerArray()

        # Initialize messages
        self._create_buoys()

    def _create_buoys(self) -> None:
        """
        Creates buoys as part of BuoyPublisher initialization
        """
        cur_time = self.get_clock().now().to_msg()
        for idx, coordinates in enumerate(self._buoy_locations):
            msg = PoseWithCovarianceStamped()
            msg_marker = Marker()
            msg_radius = Marker()

            msg.header.frame_id = self._header_frame_id # Pull in from a config file later
            msg.header.stamp = cur_time

            msg.pose.pose.position = Point(x=coordinates[0], y=coordinates[1], z=0.0)
            msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            cov_diag = np.concatenate((self._pose_uncertainty[idx,:], [0.0, 0.0, 0.0]))
            msg.pose.covariance = np.diag(cov_diag).flatten().tolist()
            
            self._buoy_msgs.poses.poses.append(msg)
            self._buoy_msgs.radii.append(self._buoy_ranges[idx])

            # Create the markers for buoy locations
            msg_marker.header.frame_id = self._header_frame_id
            msg_marker.header.stamp = cur_time

            msg_marker.ns = f'{self._header_frame_id}/buoys/{idx}'
            msg_marker.id = idx
            
            msg_marker.type = Marker.CYLINDER
            msg_marker.action = 0

            # Position and size
            msg_marker.pose = msg.pose.pose
            msg_marker.scale = Vector3()
            msg_marker.scale.x = 1.0 * self._buoy_marker_scale
            msg_marker.scale.y = 1.0 * self._buoy_marker_scale
            msg_marker.scale.z = 1.0 * self._buoy_marker_scale

            # Set color of marker
            msg_marker.color = self._buoy_color

            # How long to exist (should be infinite)
            msg_marker.lifetime = Duration(sec=0, nanosec=0) # 0 means infinite
            msg_marker.frame_locked = True # May not be necessary, as markers aren't expected to move

            # Need to insert here to keep buoys with buoys and radii with radii
            self._buoy_markers.markers.insert(idx,msg_marker)

            # Create markers for buoy ranges
            msg_radius.header.frame_id = self._header_frame_id
            msg_radius.header.stamp = cur_time

            msg_radius.ns = f'{self._header_frame_id}/buoys/{idx}/radius'
            msg_radius.id = idx

            msg_radius.type = Marker.CYLINDER
            msg_marker.action = 0

            # Pose and size
            msg_radius.pose = msg.pose.pose
            msg_radius.scale = Vector3(x=2*self._buoy_ranges[idx], y=2*self._buoy_ranges[idx], z=0.25)

            msg_radius.color = self._radius_color

            msg_radius.lifetime = Duration(sec=0, nanosec=0)
            msg_radius.frame_locked = True
            self._buoy_markers.markers.append(msg_radius)


    def _param_callback(self, params: list[rclpy.parameter.Parameter]) -> SetParametersResult:
        """ 
        Set all of the parameters that are passed in
        Only 'header_frame_id' and 'buoy_marker_scale' are settable from command line

        Args:
        - params: List of ROS parameters to change within the node
        """
        # Default the success as true
        successful = True
        reason = ""

        # Loop through any parameters and set them accordingly
        for param in params:
            match param.name:
                case "header_frame_id":
                    if isinstance(param.value, str):
                        # Don't change if new parameters aren't different
                        if self._header_frame_id != param.value:
                            self._header_frame_id = param.value
                            self._frame_id_changed = True
                    else:
                        successful = False
                        reason = "Provided <header_frame_id> must be of type 'str'"
                case "buoy_marker_scale":
                    if isinstance(param.value, float):
                        # Don't change if new parameters aren't different
                        if self._buoy_marker_scale != param.value:
                            self._buoy_marker_scale = param.value
                            self._scale_changed = True
                    else:
                        successful = False
                        reason = "Provided <buoy_marker_scale> must be of type 'float'"

        return SetParametersResult(successful=successful, reason=reason)


    def _timer_callback(self) -> None:
        """
        Callback for periodically publishing the buoy information and MarkerArrays
        """

        cur_time = self.get_clock().now().to_msg()

        l = len(self._buoy_locations_x)
        for i in range(0, l):
            # Update time (only value needing consistent update)
            self._buoy_msgs.poses.poses[i].header.stamp = cur_time

            # Update buoys and radii. Both have 4 elements in array but different starting idx
            self._buoy_markers.markers[i].header.stamp = cur_time
            self._buoy_markers.markers[i+l].header.stamp = cur_time

            if self._scale_changed:
                # Update scale
                self._buoy_markers.markers[i].scale.x = 1.0 * self._buoy_marker_scale
                self._buoy_markers.markers[i].scale.y = 1.0 * self._buoy_marker_scale
                self._buoy_markers.markers[i].scale.z = 1.0 * self._buoy_marker_scale

            if self._frame_id_changed:
                # Update header info
                self._buoy_msgs.poses.poses[i].header.frame_id = self._header_frame_id

                # Update header similar to above so all markers are updated
                self._buoy_markers.markers[i].header.frame_id = self._header_frame_id
                self._buoy_markers.markers[i+l].header.frame_id = self._header_frame_id
        
        self._scale_changed = False
        self._frame_id_changed = False
        self._pos_publisher.publish(self._buoy_msgs)
        self._marker_pub.publish(self._buoy_markers)
    
def main() -> None:
    """Spin the buoy_publisher node"""
    rclpy.init()
    pub = BuoyPublisher()

    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
