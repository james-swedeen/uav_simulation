**Table of Contents**
- [Buoy Publisher](#buoy-publisher)
  - [`BuoyPublisher` Node](#buoypublisher-node)
    - [Parameters](#parameters)
    - [Interfaces](#interfaces)
    - [Launching Demonstration](#launching-demonstration)

# Buoy Publisher
## `BuoyPublisher` Node
This node takes positions of buoys defined by the user, either in a config file or
from a launch file, and provides information about the buoys in the `ros2` framework
via multiple publishers. One publisher gives a `BuoyPoseRadiusArray`. This is a message interface
in the `uav_interfaces` package. The `BuoyPoseRadiusArray` is composed of a
`PoseWithCovarianceStampedArray`, which is a custom message from the package
`pose_covariance_stamped_array_interface`, and a `float64[] radii` component. The other publisher
sends `tf` information for displaying the location of the buoys and their ranges in `rviz2`.

The buoy_publisher node is capable of taking in specified buoy positions specified in
a config file, or otherwise as defined in the launch file `buoy_publisher.launch.py`. If you take a
look at the code, you'll observe that there is an assertion statement to verify that the lengths of
all the buoy-related lists are the same. The `buoy_publisher` node will simply fail to run if this
condition isn't met.

### Parameters
|       Parameter      |     Type    |                                                                            Description                                                                                    |
|:---------------------|:-----------:|:--------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `buoy_locations_x`   | Float Array | List of x positions of the buoys (meters)                                                                                                                                 |
| `buoy_locations_y`   | Float Array | List of y positions of the buoys (meters)                                                                                                                                 |
| `buoy_ranges`        | Float Array | List of ranges corresponding to buoys defined in previous parameters. The `float` number provided in the elements of this list correspond to the **radius** of the range. |
| `pose_uncertainty_x` | Float Array | List of uncertainty values (for covariance matrix) for x-direction of pose                                                                                                |
| `pose_uncertainty_y` | Float Array | List of uncertainty values (for covariance matrix) for y-direction of pose                                                                                                |
| `pose_uncertainty_z` | Float Array | List of uncertainty values (for covariance matrix) for z-direction of pose                                                                                                |
| `buoy_color`         | Float Array |  List in the form \[r,g,b,a\] that describes the color and transparency of the buoy markers.                                                                              |
| `radius_color`       | Float Array | List in the form \[r,g,b,a\] that describes the color and transparency of the buoy radii.                                                                                 |
| `buoy_marker_scale`  | Float       | Value by which to scale the buoy marker size. Scale value is applied uniformly to the buoy marker cylinder.                                                               |
| `header_frame_id`    | String      | Which fixed frame id to attach the buoys to.                                                                                                                              |


### Interfaces
|      Topic     | Interface Type | Function |                Message Type                |                             Description                                 |
|:---------------|:--------------:|:--------:|:-------------------------------------------|:------------------------------------------------------------------------|
| `buoy_info`    |      msg       |  Output  | `uav_interfaces::msg::BuoyPoseRadiusArray` | Information about the buoy pose and its radius of visibility            |
| `buoy_markers` |      msg       |  Output  | `visualization_msgs::msg::MarkerArray`     | Markers for RVIZ that show the buoy itself and its radius of visibility |

### Launching Demonstration
To launch a basic demonstration of what the buoys can look like, run the command below in a sourced
workspace. The details of the parameters for the demo are all contained in the launch file itself.
[`buoy_publisher.launch.py`](launch/buoy_publisher.launch.py)

```bash
ros2 launch buoy_publisher buoy_publisher.launch.py
```

