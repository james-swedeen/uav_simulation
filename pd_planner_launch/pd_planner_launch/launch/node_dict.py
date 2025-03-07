"""This module defines functions for extracting the required data from
   the paramters for each node. No data should be created in these functions,
   only reformated to the appropriate dictionaries
"""
import pd_planner_launch.params.scenario_params as par
import pd_planner_launch.params.environment as env
import pd_planner_launch.params.csv_directory as csv
from typing import Any

##################################################
############ Node Specific Dictionaries ##########
##################################################
def buoy_publisher(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """Creates a dictionary for the buoy parameters to be passed into buoy_publisher node"""
   par_dict: dict[str, Any] = {}
   par_dict["use_sim_time"] = par_st.gen.use_sim_time
   par_dict["buoy_locations_x"] = par_st.buoys.locations[0]
   par_dict["buoy_locations_y"] = par_st.buoys.locations[1]
   par_dict["buoy_locations_z"] = par_st.buoys.locations[2]
   par_dict["buoy_ranges"] = par_st.buoys.ranges
   par_dict["pose_uncertainty_x"] = par_st.buoys.sigma[0]
   par_dict["pose_uncertainty_y"] = par_st.buoys.sigma[0]
   par_dict["pose_uncertainty_z"] = par_st.buoys.sigma[0]
   par_dict["buoy_color"] = par_st.buoys.color
   par_dict["radius_color"] = par_st.buoys.radius_color
   par_dict["buoy_marker_scale"] = par_st.buoys.scale
   par_dict["header_frame_id"] = par_st.buoys.frame_id

   return par_dict

def radar_publisher(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """Creates a dictionary for the radar parameters to be passed into buoy_publisher node
      Note that the executable is buoy_publisher although the node is named radar_publisher
   """
   par_dict: dict[str, Any] = {}
   par_dict["use_sim_time"] = par_st.gen.use_sim_time
   par_dict["buoy_locations_x"] = par_st.radar.locations[0]
   par_dict["buoy_locations_y"] = par_st.radar.locations[1]
   par_dict["buoy_locations_z"] = par_st.radar.locations[2]
   par_dict["buoy_ranges"] = par_st.radar.ranges
   par_dict["pose_uncertainty_x"] = par_st.radar.sigma[0]
   par_dict["pose_uncertainty_y"] = par_st.radar.sigma[0]
   par_dict["pose_uncertainty_z"] = par_st.radar.sigma[0]
   par_dict["buoy_color"] = par_st.radar.color
   par_dict["radius_color"] = par_st.radar.radius_color
   par_dict["buoy_marker_scale"] = par_st.radar.scale
   par_dict["header_frame_id"] = par_st.radar.frame_id

   return par_dict

def ch07_sensors_auto_dyn(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """ Creates a dictionary of parameters to be passed into the ch07_sensors_auto_dyn node
   """
   # Initialize the dictionary
   par_dict: dict[str, Any] = {}
   par_dict["use_sim_time"] = par_st.gen.use_sim_time
   par_dict["ts"] = par_st.gen.dynamics_sim_period
   par_dict["noise_scale"] = par_st.gen.noise_scale
   par_dict["use_nav_state"] = par_st.gen.run_kalman_filter

   # Accelerometer
   par_dict["accel_sigma"] = par_st.imu.accel_sigma

   # Magnetometer
   par_dict["mag_sigma"] = par_st.mag.sigma
   par_dict["mag_dec"] =   par_st.mag.dec
   par_dict["mag_inc"] =   par_st.mag.inc

   # Gyro
   par_dict["gyro_sigma"]  = par_st.imu.gyro_sigma
   par_dict["gyro_x_bias"] = par_st.imu.gyro_x_bias
   par_dict["gyro_y_bias"] = par_st.imu.gyro_y_bias
   par_dict["gyro_z_bias"] = par_st.imu.gyro_z_bias

   # Pressure sensor
   par_dict["abs_pres_bias"]  = par_st.pres.abs_bias
   par_dict["abs_pres_sigma"] = par_st.pres.abs_sigma
   par_dict["diff_pres_bias"] = par_st.pres.diff_bias
   par_dict["diff_pres_sigma"]= par_st.pres.diff_sigma

   # GPS
   par_dict["gps_period"]        = par_st.gps.period
   par_dict["gps_k"]             = par_st.gps.k
   par_dict["gps_n_sigma"]       = par_st.gps.n_sigma
   par_dict["gps_e_sigma"]       = par_st.gps.e_sigma
   par_dict["gps_h_sigma"]       = par_st.gps.h_sigma
   par_dict["gps_Vg_sigma"]      = par_st.gps.Vg_sigma
   par_dict["gps_course_sigma"]  = par_st.gps.course_sigma

   # Compass
   par_dict["compass_sigma"] = par_st.comp.sigma
   par_dict["compass_bias"]  = par_st.comp.bias

   return par_dict

def feature_sensor_node(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """ Creates a dictionary of parameters to be passed into the feature_sensor_node
   """
   # Initialize the dictionary
   par_dict: dict[str, Any] = {}
   par_dict["use_sim_time"]               = par_st.gen.use_sim_time
   par_dict["uav_state_topic"]            = par_st.truth_state_topic
   par_dict["buoy_state_topic"]           = par_st.buoy_state_topic
   par_dict["los_topic"]                  = par_st.los_topic
   par_dict["los_pub_frequency"]          = par_st.feat.los_pub_frequency
   par_dict["camera_offset_vec"]          = par_st.feat.camera_offset_vec
   par_dict["camera_viewing_angles"]      = par_st.feat.camera_viewing_angles
   par_dict["range_noise_covariance"]     = par_st.feat.range_noise_sigma
   par_dict["bearing_noise_covariance"]   = par_st.feat.bearing_noise_sigma

   return par_dict

def gps_denied_monitor(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """ Creates variables needed for calculating gps denial areas for gps_denied_monitor node
   """
   # Node execution parameters
   par_dict: dict[str, Any] = {}
   par_dict["ts"] = par_st.gps_denied.period
   par_dict["use_sim_time"]: par_st.gen.use_sim_time

   # GPS denied regions
   regions = par_st.gps_denied.regions
   par_dict['num_gps_denied_regions'] = len(regions)
   count = 0
   for region in regions:
      # Form array of x values and y values
      x = []
      y = []
      for point in region:
            x.append(point[0])
            y.append(point[1])

      # Add in the parameter
      par_dict['gps_denied_'+ str(count)+'_x'] = x
      par_dict['gps_denied_'+ str(count)+'_y'] = y
      count += 1

   return par_dict

def kalman_filter_node(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """Creates a dictionary for the kalman filter parameters"""
   par_dict: dict[str, Any] = {}

   # Node execution parameters
   par_dict["state_est_pub_frequency"] = par_st.kf.state_est_pub_freq
   par_dict["integration_time_step"] = par_st.kf.integration_time_step
   par_dict["sensor_history_length_sec"] = par_st.kf.sensor_history_length_sec
   par_dict["use_sim_time"] = par_st.gen.use_sim_time

   # Setup topics
   par_dict["imu_topic"]               = par_st.imu_topic
   par_dict["gps_topic"]               = par_st.gps_topic
   par_dict["pressure_topic"]          = par_st.pressure_topic
   par_dict["compass_topic"]           = par_st.compass_topic
   par_dict["feature_los_topic"]       = par_st.feature_los_topic
   par_dict["truth_state_topic"]       = par_st.truth_state_topic
   par_dict["start_pause_srv_topic"]   = par_st.start_pause_srv_topic
   par_dict["reset_srv_topic"]         = par_st.reset_srv_topic
   par_dict["state_est_topic"]         = par_st.state_est_topic

   # Sensor specific parameters
   par_dict["feature_camera_offset_vec"] = par_st.kf.camera_offset_vec
   par_dict["feature_camera_viewing_angles"] = par_st.kf.camera_viewing_angles
   par_dict["feature_locations_x"] = par_st.buoys.locations[0]
   par_dict["feature_locations_y"] = par_st.buoys.locations[1]
   par_dict["feature_locations_z"] = par_st.buoys.locations[2]
   par_dict["heading_bias_bounds"]           = par_st.kf.heading_bias_bounds
   par_dict["abs_pressure_bias_bounds"]      = par_st.kf.abs_pressure_bias_bounds
   par_dict["feature_range_bias_bounds"]     = par_st.kf.feature_range_bias_bounds
   par_dict["feature_bearing_bias_bounds"]   = par_st.kf.feature_bearing_bias_bounds
   par_dict["gps_position_bias_bounds"]      = par_st.kf.gps_position_bias_bounds
   par_dict["gyroscope_bias_bounds"]         = par_st.kf.gyroscope_bias_bounds
   par_dict["accelerometer_bias_bounds"]     = par_st.kf.accelerometer_bias_bounds
   par_dict["dynamics"] = {
      "gravity_mag": env.GRAVITY,
      "air_density": env.AIR_DENSITY,
      "compass_bias":{
            "enabled": True,    # Note that "enabled" and "zero_mean" should be kept as true.
            "standard_deviation": par_st.kf.compass_bias_std,
            "time_constant": par_st.kf.compass_bias_tc
      },
      "abs_pressure_bias": {
            "enabled": True,    # Note that "enabled" and "zero_mean" should be kept as true.
            "standard_deviation": par_st.kf.abs_press_bias_std,
            "time_constant": par_st.kf.abs_press_bias_tc
      },
      "feature_range_bias": {
            "enabled": True,    # Note that "enabled" and "zero_mean" should be kept as true.
            "standard_deviation": par_st.kf.feature_range_bias_std,
            "time_constant": par_st.kf.feature_range_bias_tc
      },
      "feature_bearing_bias": {
            "enabled": True,    # Note that "enabled" and "zero_mean" should be kept as true.
            "standard_deviation": par_st.kf.feature_bearing_bias_std,
            "time_constant": par_st.kf.feature_bearing_bias_tc
      },
      "gps_pos_bias": {
            "enabled": True,    # Note that "enabled" and "zero_mean" should be kept as true.
            "standard_deviation": par_st.kf.gps_pos_bias_std,
            "time_constant": par_st.kf.gps_pos_bias_tc
      },
      "accel_bias": {
            "enabled": True,    # Note that "enabled" and "zero_mean" should be kept as true.
            "standard_deviation": par_st.kf.accel_bias_std,
            "time_constant": par_st.kf.accel_bias_tc
      },
      "gyro_bias": {
            "enabled": True,    # Note that "enabled" and "zero_mean" should be kept as true.
            "standard_deviation": par_st.kf.gyro_bias_std,
            "time_constant": par_st.kf.gyro_bias_tc
      }
   }
   return par_dict

def lincov_interface(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """Creates a dictionary for parameters needed in the lincov_interface node"""
   kf_dict = kalman_filter_node(par_st)
   lincov_kf_dict: dict = {}
   lincov_kf_dict["dynamics"] = {}

   # Redefine lincov-specific kalman filter parameters and reduce unused parameters
   lincov_kf_dict["dynamics"]["compass_bias"]         = kf_dict["dynamics"]["compass_bias"]
   lincov_kf_dict["dynamics"]["abs_pressure_bias"]    = kf_dict["dynamics"]["abs_pressure_bias"]
   lincov_kf_dict["dynamics"]["feature_range_bias"]   = kf_dict["dynamics"]["feature_range_bias"]
   lincov_kf_dict["dynamics"]["feature_bearing_bias"] = kf_dict["dynamics"]["feature_bearing_bias"]
   lincov_kf_dict["dynamics"]["gps_pos_bias"]         = kf_dict["dynamics"]["gps_pos_bias"]
   lincov_kf_dict["dynamics"]["accel_bias"]           = kf_dict["dynamics"]["accel_bias"]
   lincov_kf_dict["dynamics"]["gyro_bias"]            = kf_dict["dynamics"]["gyro_bias"]

   par_dict: dict[str, Any] = {
      'features':{
         'feature_locations_x': par_st.buoys.locations[0],
         'feature_locations_y': par_st.buoys.locations[1],
         'feature_locations_z': par_st.buoys.locations[2],
         'feature_ranges':      par_st.buoys.ranges,
      },
      'trajectory':              trajectory_parameters(par_st),
      'ellipses':                rviz_3sig_ellipse_parameters(par_st),
      'kalman_filter':           lincov_kf_dict,
      'analysis_time_step':      par_st.gen.lc_mc_period,
      'sensors':                 lincov_sensors_parameters(par_st),
      'imu':                     imu_parameters(par_st),
      'plotting_data_directory': csv.CSV_DATA_DIRECTORY,
   }
   return par_dict

def pdvg_interface(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """Creates a dictionary for parameters needed in the pdvg_interface node"""
   kf_dict = kalman_filter_node(par_st)
   pdvg_kf_dict: dict = {}
   pdvg_kf_dict["dynamics"] = {}

   # Redefine pdvg-specific kalman filter parameters
   pdvg_kf_dict["dynamics"]["compass_bias"]         = kf_dict["dynamics"]["compass_bias"]
   pdvg_kf_dict["dynamics"]["abs_pressure_bias"]    = kf_dict["dynamics"]["abs_pressure_bias"]
   pdvg_kf_dict["dynamics"]["feature_range_bias"]   = kf_dict["dynamics"]["feature_range_bias"]
   pdvg_kf_dict["dynamics"]["feature_bearing_bias"] = kf_dict["dynamics"]["feature_bearing_bias"]
   pdvg_kf_dict["dynamics"]["gps_pos_bias"]         = kf_dict["dynamics"]["gps_pos_bias"]
   pdvg_kf_dict["dynamics"]["accel_bias"]           = kf_dict["dynamics"]["accel_bias"]
   pdvg_kf_dict["dynamics"]["gyro_bias"]            = kf_dict["dynamics"]["gyro_bias"]

   # Do the same for sensor parameters, only need gps, abs-pressure, and compass
   sensor_dict = lincov_sensors_parameters(par_st)
   pdvg_sensor_dict: dict = {}
   pdvg_sensor_dict["gps"] = sensor_dict["gps"]
   pdvg_sensor_dict["abs_pressure"] = sensor_dict["abs_pressure"]
   pdvg_sensor_dict["compass"] = sensor_dict["compass"]

   par_dict: dict[str, Any] = {
      'obstacle_checker':{
        'radar_locations_x':   par_st.radar.locations[0],
        'radar_locations_y':   par_st.radar.locations[1],
        'radar_locations_z':   par_st.radar.locations[2],
        'ellipsoid_axes':      par_st.radar.ellipsoid_axes,
        'prob_false_alarms':   par_st.radar.prob_false_alarms,
        'radar_consts':        par_st.radar.radar_constants,
        'radar_pos_std_dev':   par_st.radar.radar_pos_std_dev,
        'radar_const_std_dev': par_st.radar.radar_const_std_dev,
        'pd_threshold':        par_st.radar.pd_threshold,
        'std_dev_multiple':    par_st.radar.std_dev_multiple,
        'nominal_cross_section': par_st.radar.nominal_cross_section,
        'starting_num_sides': par_st.radar.starting_num_sides,
      },
      'pdvg_world':{
        'world_bounds': par_st.pdvg.world_bounds,
      },
      'trajectory':             trajectory_parameters_pdvg(par_st),
      'kalman_filter':          pdvg_kf_dict,
      'sensors':                pdvg_sensor_dict,
      'imu':                    imu_parameters(par_st),
      'visualization':{
        'frame_id': par_st.gen.inertial_frame,
      },
      'plotting_data_directory': csv.CSV_DATA_DIRECTORY,
   }
   return par_dict

def csv_plotter_parameters(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """Creates a dictionary of parameters for the csv_plotter_run node"""
   par_dict: dict[str, Any] = {}
   par_dict["pd_threshold"]   = par_st.radar.pd_threshold
   par_dict["std_dev_multiple"] = par_st.radar.std_dev_multiple
   return par_dict

def state_listener_csv_parameters(par_st: par.ScenarioParameters, get_truth_data: bool, create_dir: bool) -> dict[str, Any]:
   """
   Creates a dictionary of parameters for the state_listener_csv_node node

   Args:
   - par_st: ScenarioParameters class for the current simulation
   - get_truth_data: Flag indicating whether this particular node is for truth data or not. If
     not, it will be used to collect navigation data
   - create_dir: Flag indicating whether to make a new data directory. Since two nodes are typically
     made here, one for truth and one for navigation data, only one will have this flag set to true

   Returns:
   - Dictionary of parameters for a state listener
   """

   par_dict: dict[str, Any]   = {}
   par_dict["use_sim_time"]   = par_st.gen.use_sim_time
   par_dict["row_limit"]      = par_st.state_listener.row_limit
   par_dict["precision"]      = par_st.state_listener.precision
   par_dict["get_truth_data"] = get_truth_data

   if create_dir:
      par_dict["data_directory"] = csv.create_csv_directory_headless()
      #csv.collect_test_info(par_dict["data_directory"]) TODO: Put this back

   return par_dict

def sim_manager_parameters(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """
   Creates a dictionary of parameters for the sim_manager node

   Args:
   - par_st: ScenarioParameters class for the current simulation

   Returns:
   - A dictionary with parameters that define a SimManager node
   """

   par_dict: dict[str, Any] = {}
   par_dict["total_runs"]        = par_st.sim_manager.total_runs
   par_dict["clock_speed"]       = par_st.sim_manager.clock_speed
   par_dict["use_pdvg"]          = par_st.sim_manager.use_pdvg
   par_dict["use_kalman_filter"] = par_st.gen.run_kalman_filter

   par_dict["nominal_velocity"]  = par_st.sim_manager.nominal_velocity
   par_dict["nominal_down"]      = par_st.sim_manager.nominal_down

   par_dict["pdvg_x_pos"]        = par_st.sim_manager.pdvg_x_pos
   par_dict["pdvg_y_pos"]        = par_st.sim_manager.pdvg_y_pos

   par_dict["manual_wp_x"]       = par_st.sim_manager.manual_wp_x
   par_dict["manual_wp_y"]       = par_st.sim_manager.manual_wp_y
   par_dict["path_type"]         = par_st.sim_manager.path_type
   par_dict["min_radius"]        = par_st.sim_manager.min_radius

   return par_dict

##################################################
############ Helper Functions ####################
##################################################
def trajectory_parameters(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """Creates a dictionary for the trajectory parameters"""
   par_dict: dict[str, Any] = {}
   par_dict["nominal_velocity"] = par_st.nom_traj.velocity
   par_dict["max_curvature"] = par_st.nom_traj.max_curvature
   par_dict["max_curvature_rate"] = par_st.nom_traj.max_curve_rate
   par_dict["nominal_pitch"] = par_st.nom_traj.nominal_pitch
   par_dict["gravity"] = env.GRAVITY

   return par_dict

def trajectory_parameters_pdvg(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """Creates a dictionary for the trajectory parameters for PDVG-specific operation"""
   par_dict: dict[str, Any] = trajectory_parameters(par_st)
   par_dict["fillet_dt"] = par_st.nom_traj.fillet_dt
   par_dict["line_dt"] = par_st.nom_traj.line_dt
   par_dict["use_arc_fillet"] = par_st.nom_traj.use_arc_fillet

   return par_dict

def rviz_3sig_ellipse_parameters(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """Creates dictionary for 3sigma rviz ellipses used in lincov_interface"""
   par_dict: dict[str, Any] = {}
   par_dict["color"] = [1.0, 0.0, 0.0, 1.0] # RGBA
   par_dict["test_scale"] = 1.0
   par_dict["frame_id"] = par_st.gen.inertial_frame

   return par_dict

def imu_parameters(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """Creates dictionary for lincov imu parameters in lincov_interface"""
   par_dict: dict[str, Any] = {}
   imu_dict: dict[str, Any] = {}

   # Accelerometer
   accelerometer: dict[str, Any] = {}
   accelerometer["enabled"] = True
   accelerometer["standard_deviation"] = [par_st.imu.accel_sigma for x in range(0,3)]

   # Gyroscope
   gyroscope: dict[str, Any] = {}
   gyroscope["enabled"] = True
   gyro_x_bias = par_st.imu.gyro_x_bias
   gyro_y_bias = par_st.imu.gyro_y_bias
   gyro_z_bias = par_st.imu.gyro_z_bias
   gyroscope["standard_deviation"] = [gyro_x_bias, gyro_y_bias, gyro_z_bias]

   imu_dict["accelerometer"] = {"noise": accelerometer}
   imu_dict["gyroscope"] = {"noise": gyroscope}

   return imu_dict

def lincov_sensors_parameters(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """Creates dictionary for sensors and sensor noise in lincov_interface"""
   enable_noise = True # Note that "enabled" and "zero_mean" should be kept as true.

   par_dict: dict[str, Any] = {}
   par_dict = {
      "gps":{
         "enabled": True,
         "measurement_period": par_st.gps.period,
         "noise":{
            "enabled": enable_noise,
            "standard_deviation": [par_st.gps.n_sigma, par_st.gps.e_sigma, par_st.gps.h_sigma],
            },
         },
      "compass":{
         "enabled": True,
         "measurement_period": par_st.gen.dynamics_sim_period,
         "noise":{
            "enabled": enable_noise,
            "standard_deviation": [par_st.comp.sigma],
            },
         },
      "abs_pressure":{
         "enabled": True,
         "measurement_period": par_st.gen.dynamics_sim_period,
         "gravity_magnitude": env.GRAVITY,
         "air_density": env.AIR_DENSITY,
         "noise":{
            "enabled": enable_noise,
            "standard_deviation": [par_st.pres.abs_sigma],
            },
         },
      "ground_velocity":{
         "enabled": True,
         "measurement_period": par_st.gps.period,
         "noise":{
            "enabled": enable_noise,
            "standard_deviation": [par_st.gps.Vg_sigma],
            },
         },
      "feature_range":{
         "enabled": par_st.gen.use_feature_sensor,
         "measurement_period": 1./par_st.feat.los_pub_frequency,
         "noise":{
            "enabled": enable_noise,
            "standard_deviation": [par_st.feat.range_noise_sigma],
            }
         },
      "feature_bearing":{
         "enabled": par_st.gen.use_feature_sensor,
         "measurement_period": 1./par_st.feat.los_pub_frequency,
         "camera_offset": par_st.kf.camera_offset_vec,
         "camera_viewing_angles": par_st.kf.camera_viewing_angles,
         "noise":{
            "enabled": enable_noise,
            "standard_deviation": [par_st.feat.bearing_noise_sigma[0], par_st.feat.bearing_noise_sigma[2]],
            }
         }
   }
   return par_dict

def all_params_dict(par_st: par.ScenarioParameters) -> dict[str, Any]:
   """
   Create a top-level dictionary containing all of the parameters available
   for collection in this file. Output is used to save a parameters file
   for headless runs to track parameters per data set

   Args:
   - par_st: ScenarioParameters class for the current simulation
   """
   par_dict: dict[str, Any] = {}

   par_dict[par_st.reuse.buoy_publisher_key]        = buoy_publisher(par_st)
   par_dict[par_st.reuse.radar_publisher_key]       = radar_publisher(par_st)
   par_dict[par_st.reuse.ch07_sensors_auto_dyn_key] = ch07_sensors_auto_dyn(par_st)
   par_dict[par_st.reuse.feature_sensor_node_key]   = feature_sensor_node(par_st)
   par_dict[par_st.reuse.gps_denied_monitor_key]    = gps_denied_monitor(par_st)
   par_dict[par_st.reuse.kalman_filter_node_key]    = kalman_filter_node(par_st)
   par_dict[par_st.reuse.lincov_interface_key]      = lincov_interface(par_st)
   par_dict[par_st.reuse.pdvg_interface_key]        = pdvg_interface(par_st)
   par_dict[par_st.reuse.csv_plotter_par_key]       = csv_plotter_parameters(par_st)
   par_dict[par_st.reuse.truth_state_listener_key]  = state_listener_csv_parameters(par_st, get_truth_data=True, create_dir=False)
   par_dict[par_st.reuse.nav_state_listener_key]    = state_listener_csv_parameters(par_st, get_truth_data=False, create_dir=False)
   par_dict[par_st.reuse.sim_manager_key]           = sim_manager_parameters(par_st)

   return par_dict
