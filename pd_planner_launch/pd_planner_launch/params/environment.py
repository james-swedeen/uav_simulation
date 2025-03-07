"""This module defines the parameters for the environment layout
"""

#####################################
###  Environmental constants #######
#####################################
GRAVITY: float = 9.81
AIR_DENSITY: float = 1.2682

#####################################
#######  Features ###################
#####################################
class Features():
    """Parameters defining the buoy locations and uncertainties"""
    def __init__(self) -> None:
        # Location and range variables
        self.frame_id = "ned"   # Frame id for the locations
        self.locations = [  [1000.0, -1000.0, 500.0, -200.0],     # x-positions
                            [500.0, 500.0, -500.0, -200.0],             # y-positions
                            [0., 0., 0., 0.] ]                          # z-positions
        self.ranges = [200.0, 500.0, 500.0, 200.0]   # Range at which each feature can be sensed
        self.sigma = [  [0.0, 0.0, 0.0, 0.0],   # std-dev of x positions
                        [0.0, 0.0, 0.0, 0.0],   # std-dev of y positions
                        [0.0, 0.0, 0.0, 0.0] ]  # std-dev of z positions

        # Plotting variables
        self.color = [1.0, 0.39, 0.0, 1.0]  # RGB-alpha used for plotting the color of the feature
        self.radius_color = [1.0, 0.39, 0.0, 1.0] # RGB-alpha used for plotting the radius depiction
        self.scale: float = 200.     # The scale at which the feature is plotted

#####################################
#######  Radars #####################
#####################################
class Radars():
    """Parameters defining the radars"""
    def __init__(self, locations: list[list[float]], ranges: list[float], sigma: list[list[float]], radar_constants: list[float]) -> None:
        # Store input variables
        self.locations = locations              # x,y,z-positions
        self.ranges = ranges                    # Range at which each feature can be sensed
        self.sigma = sigma                      # std-dev of x,y,z positions
        self.radar_constants = radar_constants  # Radar constants

        # Location and range variables
        self.frame_id:        str = "ned"  # Frame id for the locations

        ## PDVG planning variables ##
        self.ellipsoid_axes:        list[float]   # x,y,z ellipsoid detection model axes
        self.prob_false_alarms:     list[float]   # Probability of false alarm for each radar
        self.radar_pos_std_dev:     list[float]   # std-dev of radar positions
        self.radar_const_std_dev:   list[float]   # std-dev of radar constants
        self.pd_threshold:          float         # Probability of detection threshold for each radar
        self.std_dev_multiple:      float         # Multiple of the std-dev that has to be below pd_threshold
        self.starting_num_sides:    int           # How many sides are generated to start planner
        self.nominal_cross_section: float         # Nominal radar cross section for planner

        # Constants
        self.ellipsoid_axes =       [0.18, 0.17, 0.2]
        self.prob_false_alarms =    1e-9
        self.radar_pos_std_dev =    0.1
        self.radar_const_std_dev =  0.0
        self.pd_threshold =         0.1
        self.std_dev_multiple =     3.0

        # For calculation of starting radar radii
        self.starting_num_sides =    30    # Starting number of sides
        self.nominal_cross_section = 0.09

        ## Plotting variables ##
        # RGB-alpha used for plotting the color of the feature
        self.color:         list[float] = [0.0039, 0.2667, 0.1294, 1.0]

        # RGB-alpha used for plotting the radius depiction
        self.radius_color:  list[float] = [0.0039, 0.2667, 0.1294, 1.0]

        # The scale at which the feature is plotted
        self.scale:         float = 200.


#####################################
#######  GPS Denied #################
#####################################
class GpsDenied():
    """Defines regions of gps denied areas"""
    def __init__(self) -> None:
        self.period = 1.0   # Publishing period for gps denied evaluation

        # Define regions
        region1 = [(100., 0.), (5000., 0), (5000., 2100.), (100., 2100.)]
        region2 = [(0., -100.), (2000., -100), (2000., -1000.), (-2000., -1000.)]

        # Store regions as member variable
        self.regions = [region1, region2]

##################
### PDVG World ###
##################
class PDVGWorld():
    """
    Defines PDVG-specific world characteristics, as well as
    variables for visual publishing of world bounds.
    """
    def __init__(self) -> None:
        # World bounds defined as: [ xmin ,   ymin ,  xmax ,  ymax ] (x is North, y is East)
        # Test Case: PDVG Playground (takes whole default grid space in RVIZ)
        xmin = -5000.0
        xmax = 5000.0
        ymin = -5000.0
        ymax = 5000.0
        self.world_bounds =  [xmin, ymin, xmax, ymax]
