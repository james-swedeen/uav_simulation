"""
Define the MCExporter class, which reads user-selected data
from a specified directory containing Monte Carlo data sets
and exports the averaged variance data for the north, east,
down, roll, pitch, and yaw states.
"""

from pathlib import Path
from scipy import interpolate
from pd_planner_launch.params.scenario_params import GeneralConfig

import argparse
import numpy as np
import pandas as pd
import csv
import math
import numpy.typing as npt
import scipy.optimize as opt


class MCExporter:
  """
  The MCExporter class is designed to handle exporting of relevant
  truth dispersion variance data.
  """

  def __init__(self, data_dir: str) -> None:
    """
    Initialize the object. After collecting some basic information,
    scan through one of the files in the selected data directory to produce a dictionary
    of available information. Generate and export variance data.

    Inputs:
    - data_dir: Data directory in which to look for data (e.g. ~/test_data_headless/data_003)
    """
    # Set file IO parameters
    self._data_dir = Path(data_dir)   # Data directory
    self._truth_prefix: str = 'truth'
    self._nav_prefix: str   = 'nav'

    # Data organization variables
    self._dt = GeneralConfig.dynamics_sim_period
    self._master_time: np.ndarray

    # Menu information variables
    self._k_group_name = 'group_name'     # Associated value is list of names of group headers (e.g. Euler Angles, Velocity, etc...)
    self._k_start_idx = 'starting_idx'    # Value is list of first indices of each respective group
    self._k_end_idx = 'ending_idx'        # Value is list of last indicees of each respective group
    self._k_name_list = 'sub_name_list'   # Value is list of names contained within overall group (e.g. X, Y, Z)

    # Make dictionary to contain above information
    self._group_dict = {self._k_group_name: [],\
                        self._k_name_list: [],\
                        self._k_start_idx: [],\
                        self._k_end_idx: []}

    self._header_list:    list[str]  = []   # List of individual headers if user selects single columns of data to plot
    self._ylabel_list:    list[str]  = []   # List of y-axis labels associated with the selected data
    self._directory_list: list[Path] = []   # List of directories containing test data that contain the "COMPLETE" file
    self._nruns = 0                         # Number of runs present in current folder

    # Update header_dict and nruns
    self._scan_headers()
    self._g_name_len = len(self._group_dict[self._k_group_name])
    self._header_len = len(self._header_list)

    # Get master time vector
    self._set_master_time()

  def _scan_headers(self) -> None:
    """
    From a given CSV file known to be in a COMPLETE run folder,
    extract all the column headers, group them by name, and save information
    to member variables.
    """
    # Get list of directories and test file
    self._directory_list = [path for path in self._data_dir.iterdir() if path.is_dir()]

    # Now with incomplete runs trimmed, get number of runs operating on
    self._nruns = len(self._directory_list)

    test_file: Path = list(self._directory_list[0].glob('*.csv'))[0]

    # Read CSV and get header list
    data = pd.read_csv(str(test_file), header=0)
    self._header_list = list(data.head())[1:]     # Exclude time header
    header_list_len = len(self._header_list)
    temp_group_name = ''                          # Use to compare against newest group name
    group_active = False                          # Indicates whether a new group has been found
    name_list = []                                # List of ending terms for each data header (e.g. X, Y, Z, or Phi, Theta, Psi)

    # Add headers to group dictionary
    for header in range(0, header_list_len):
      cur_group_name = ''

      # Get all words besides the last one to use as current group name
      h = self._header_list[header].split(sep=' ')
      h_name = h[:-1]
      hend = h[-1]
      h_nwords = len(h_name)
      for part in range(0, h_nwords):
        # Form name from all but last word
        # (e.g. "Euler Angles Phi/Theta/Psi" -> "Euler Angles")
        if part < h_nwords-1:
          cur_group_name += h_name[part] + ' '
        else:
          cur_group_name += h_name[part]

      # If group has started and the group name is the same as before
      if group_active and temp_group_name == cur_group_name:
        # Add current header to name list
        name_list.append(hend)
        continue
      elif group_active:
        # Group name no longer matches, so add ending index and name list
        self._group_dict[self._k_end_idx].append(header-1)
        self._group_dict[self._k_name_list].append(name_list)
        self._ylabel_list.extend(name_list)
        name_list = []

        # Set to false for next group to start
        group_active = False

      if not cur_group_name in self._group_dict[self._k_group_name]:
        # Add group name and group starting index to dictionary
        self._group_dict[self._k_group_name].append(cur_group_name)
        self._group_dict[self._k_start_idx].append(header)
        name_list.append(hend)
        temp_group_name = cur_group_name
        group_active = True

    # On last iteration, not going to catch remaining info, so add here.
    self._group_dict[self._k_end_idx].append(header_list_len-1)
    self._group_dict[self._k_name_list].append(name_list)
    self._ylabel_list.extend(name_list)

  def _set_master_time(self):
    """Iterate over all files to find shortest time vector to use as reference vector"""
    min_length = np.inf         # Tracks minimum length vector
    min_file_list: list[Path]   # Stores set of files associated with minimum length run

    # Iterate through run folders
    for run in range(0, len(self._directory_list)):
      # Get combined file size of truth data and combined file size of nav data
      truth_size = 0
      nav_size = 0
      for f in self._directory_list[run].iterdir():
        if self._truth_prefix in str(f):
          truth_size += f.stat().st_size
        elif self._nav_prefix in str(f):
          nav_size += f.stat().st_size

      # Read in all of time vector from smaller set of files
      if truth_size < nav_size:
        file_list = self._get_sorted_run_files(self._directory_list[run], self._truth_prefix)

      else:
        file_list = self._get_sorted_run_files(self._directory_list[run], self._nav_prefix)

      # Get time vector information
      length = 0
      for f in range(0, len(file_list)):
        time_csv = pd.read_csv(str(file_list[f]), header=0).to_numpy()
        length += len(time_csv[:,0])

      # Update shortest length
      if length < min_length:
        min_length = length
        min_file_list = file_list

    # Re-open files from minimum length file list to store time vector information
    start_time: float = 0
    end_time:   float = 0

    # Get start time
    data = pd.read_csv(str(min_file_list[0]), header=0).to_numpy()
    start_time = data[0, 0]

    # Get end time
    data = pd.read_csv(str(min_file_list[-1]), header=0).to_numpy()
    end_time = data[-1, 0]

    # Create master time vector
    #self._master_time = np.arange(0, length+1)*self._dt
    self._master_time = np.arange(start=start_time, stop=end_time, step=self._dt)


  def _get_sorted_run_files(self, run_path: Path, ftype: str = 'truth') -> list:
    """
    Return a sorted list of files in a given run folder

    Inputs:
    - run_path: Run folder to operate in
    - ftype:    File type to look at (truth or nav)

    Output:
    - file_list: List of files in ascending order of file number
    """

    # Get list of all files
    file_list = [path for path in run_path.iterdir() if ftype in str(path)]

    # Make lambda function to extract the numbers at file suffix
    fsort = lambda file: int(str(file).split('_')[-1].split('.')[0])

    # Sort file list numerically so order of data is correctly extracted
    file_list = sorted(file_list, key=fsort)

    return file_list


  def _get_data(self, header_num: int, ftype: str = 'truth') -> np.ndarray:
    """
    Iterate through all runs to get specified data type for a particular header.

    Inputs:
    - header_num: Column number corresponding to the state of interest
    - ftype:      File type to look at (truth or nav)

    Output:
    - data:       np.ndarray with specified state data for each run. Every run is
                  emplaced in the array as a column vector.
    """

    # Set up data vector, stores each run in columns
    data: np.ndarray = np.zeros([len(self._master_time), len(self._directory_list)])
    interp_data = []
    time = []

    # For each run
    for run in range(0, len(self._directory_list)):
      file_list = self._get_sorted_run_files(self._directory_list[run], ftype)

      # For each data file
      for f in range(0, len(file_list)):
        # Append header_num data to a row of output data
        try:
            temp_data = pd.read_csv(str(file_list[f])).to_numpy()
        except Exception as ex:
            print("File: " + str(file_list[f]) + " threw an exception")
            raise ex

        if not np.isfinite(temp_data).all():
            print("File: " + str(file_list[f]) + " has Nan or Inf")
            raise RuntimeError

        # Append time to row
        if f == 0:
          time = list(temp_data[:, 0])
          interp_data = list(temp_data[:, header_num])
        else:
          time.extend(temp_data[:,0])
          interp_data.extend(temp_data[:,header_num])

      # Now that data has been collected, need to convert time to master time
      time = np.array(time)
      interp_data = np.array(interp_data)

      # Remove duplicate time points
      (time, unique_inds) = np.unique(time, return_index=True)
      interp_data = interp_data[unique_inds]

      # Convert to relative time
      #time -= time[0]

      # Get interpolation function
      f_interp = interpolate.interp1d(time, interp_data, kind='quadratic', \
                                      bounds_error=False, \
                                      fill_value=(interp_data[0], interp_data[-1]))

      # Interpolate data so all values are considered at the same times as the master time vector
      data[:, run] = f_interp(self._master_time)

    return data


  def _calculate_disp(self, data: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Calculate dispersions of data

    Inputs:
    - data: Array of data of shape (N x R), where N is the length of the master time
            vector and R is the number of valid runs available in the data directory

    Outputs:
    - data_avg:       Mean of the data along the time dimension (N x 1)
    - data_variance:  Variance of the data along the time dimension (N x 1)
    """

    return (np.average(data, 1), np.var(data, 1))


  def _export_group(self, group_vector: list[int]) -> list[np.ndarray]:
    """
    Export the truth dispersion for a complete
    set of states (e.g. "Position", "Euler Angles", etc)

    Inputs:
    - group_vector:       A list of indices that correspond to the states to be exported

    Output:
    - The requested truth variance data
    """
    output_list: list[np.ndarray] = []
    if (group_vector is not None) and (len(group_vector) != 0):
        # For each state that was selected
        for group_num in group_vector:
            # Get data out based on type
            data = self._get_data(group_num, self._truth_prefix)

            # Calculate disp
            [mean, disp_data] = self._calculate_disp(data)

            output_list.append(disp_data)

    return output_list

  @staticmethod
  def _calc_cov_fogm(tau: float, q: float, p0: float, t0: float, tk: npt.NDArray[float]) -> float:
    """Calculate the covariance using the following function
        p(tk) = exp(-2*(tk-t0)/tau)p(t0) + tau/2(1-exp(-2*(tk-t0)/2))q

        Inputs:
            tau: time constant
            q: noise variance
            p0: Initial covariance value
            t0: Initial time
            tk: time in question

        Returns:
            p(tk) - covariance for the time in question
    """
    return np.exp(-2.*(tk-t0)/tau)*p0 + (tau/2.)*(1.-np.exp((-2./tau)*(tk-t0) ))*q

  @staticmethod
  def _squared_error_fogm(x: npt.NDArray[float], tvec_act: list[float], pvec_act: list[float]) -> float:
    """ Returns the squared error of using the estimated time constant and noise variance given the
        actual p vector

        Inputs:
            x: 1D array with shape (2,)
                x[0]: tau_est: Estimated time constant
                x[1]: q_est: Estimated noise variance
            pvec_act: The actual (observed) covariance
            tvec_act: the time vector for the observed covariance data

        Returns:
            summed squared error
    """
    # Calculate the error for pvec
    err = pvec_act-MCExporter._calc_cov_fogm(tau=x[0], q=x[1], p0=pvec_act[0], t0=tvec_act[0], tk=tvec_act)

    # Calculate the summed squared err
    return float(err@err)


  def _find_optimal_fogm_parameters(self, tvec: list[float], pvec: list[float]) -> tuple[float, float]:
    """Given the covariance and time vectors, the optimal time constant and noise variance is found

        Inputs:
            tvec: Time vector
            pvec: Resulting P value at each time
            x0: Initial guess

        Returns:
            tau_est: Estimated time constant
            q_est: Estimated noise variance
    """

    # Minimize using a global solver
    minimizer_kwargs = {}
    minimizer_kwargs['args'] = (tvec, pvec)
    x0 = [100.0, 1.0]
    res_pos = opt.basinhopping(func=self._squared_error_fogm, minimizer_kwargs=minimizer_kwargs, x0=x0, niter=150)
    x0 = [-100.0, 1.0]
    res_neg = opt.basinhopping(func=self._squared_error_fogm, minimizer_kwargs=minimizer_kwargs, x0=x0, niter=150)

    #print(res_pos['message'])
    #print(res_neg['message'])

    # Get the results
    if res_pos['fun'] < res_neg['fun']:
      tau_est = res_pos['x'][0]
      q_est = res_pos['x'][1]
    else:
      tau_est = res_neg['x'][0]
      q_est = res_neg['x'][1]

    return (tau_est, q_est)


  # === PUBLIC FUNCTIONS ===
  def run(self) -> None:
    """
    Run exporting operations.
    """

    #group_vector = range(1, 7)
    group_vector = range(1, 7)

    # Get all variance data
    truth_var_vec = self._export_group(group_vector)

    # Save variance data to csv
    with open(str(self._data_dir) + "/truth_variance.csv", 'w', newline='') as csvfile:
      writer = csv.DictWriter(csvfile, delimiter=',', fieldnames=['time', 'position_north', 'position_east', 'position_down', 'roll', 'pitch', 'yaw'])

      writer.writeheader()
      for time_it in range(0, len(self._master_time)):
          writer.writerow({
              'time': self._master_time[time_it],
              'position_north': truth_var_vec[0][time_it],
              'position_east': truth_var_vec[1][time_it],
              'position_down': truth_var_vec[2][time_it],
              'roll': truth_var_vec[3][time_it],
              'pitch': truth_var_vec[4][time_it],
              'yaw': truth_var_vec[5][time_it],
          })

    # Find optimal FOGM parameters
    opt_fogm_params = []
    for state_it in range(0, 6):
      opt_fogm_params.append(self._find_optimal_fogm_parameters(tvec=self._master_time, pvec=truth_var_vec[state_it]))

    # Save optimal FOGM parameters to csv
    with open(str(self._data_dir) + "/truth_variance_fogm_parameters.csv", 'w', newline='') as csvfile:
      writer = csv.DictWriter(csvfile, delimiter=',', fieldnames=['position_north_tau',
                                                                  'position_north_q',
                                                                  'position_east_tau',
                                                                  'position_east_q',
                                                                  'position_down_tau',
                                                                  'position_down_q',
                                                                  'roll_tau',
                                                                  'roll_q',
                                                                  'pitch_tau',
                                                                  'pitch_q',
                                                                  'yaw_tau',
                                                                  'yaw_q'])
      writer.writeheader()
      writer.writerow({
          'position_north_tau': opt_fogm_params[0][0],
          'position_north_q': opt_fogm_params[0][1],
          'position_east_tau': opt_fogm_params[1][0],
          'position_east_q': opt_fogm_params[1][1],
          'position_down_tau': opt_fogm_params[2][0],
          'position_down_q': opt_fogm_params[2][1],
          'roll_tau': opt_fogm_params[3][0],
          'roll_q': opt_fogm_params[3][1],
          'pitch_tau': opt_fogm_params[4][0],
          'pitch_q': opt_fogm_params[4][1],
          'yaw_tau': opt_fogm_params[5][0],
          'yaw_q': opt_fogm_params[5][1],
      })


def main(args=None):
  """Initialize a exporter and run it"""
  exporter = MCExporter(args.data_dir)
  exporter.run()


if __name__ == "__main__":
  # Get data directory from user in order to run
  parser = argparse.ArgumentParser()
  parser.add_argument("data_dir", type=str, help="Directory storing MC data")
  args = parser.parse_args()
  main(args)
