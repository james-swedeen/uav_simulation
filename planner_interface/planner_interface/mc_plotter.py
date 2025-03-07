"""
Define the MCPlotter class, which reads user-selected data
from a specified directory containing Monte Carlo data sets
and plots the selected data.
"""

from planner_interface.mc_plotter_menu import MCPlotterMenu
from pathlib import Path
from scipy import interpolate
from pd_planner_launch.params.scenario_params import GeneralConfig

import argparse
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.backends.backend_pdf as backend_pdf
import matplotlib.pyplot as plt

class MCPlotter:
  """
  The MCPlotter class is designed to handle plotting operations for user-selected data
  that is collected from headless PDVG simulations. It offers the user menu-based
  interactions to get plot information either as an interactive plot or by saving it
  to PDF.
  """

  def __init__(self, data_dir) -> None:
    """
    Initialize the plotter object. After collecting some basic information,
    scan through one of the files in the selected data directory to produce a dictionary
    of available plotting information. After information is obtained, produce the menu
    object to interact with the user. Lastly, evaluate one of the file sets to get a
    master time vector against which to interpolate all data.

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


    # Make dictionary to feed into MCPlotterMenu
    menu_init_dict = {
      '_k_group_name':  self._k_group_name,
      '_k_start_idx':   self._k_start_idx,
      '_k_end_idx':     self._k_end_idx,
      '_k_name_list':   self._k_name_list,
      '_group_dict':    self._group_dict,
      '_header_list':   self._header_list,
      '_g_name_len':    self._g_name_len,
      '_header_len':    self._header_len,
    }

    # Initialize menu
    self._menu = MCPlotterMenu(menu_init_dict)

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


  def _calculate_navigation_error(self, truth_data: np.ndarray, nav_data: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Calculate navigation error from truth and nav state data

    Inputs:
    - truth_data: Array of truth data of shape (N x R), where N is the length of the
        master time vector and R is the number of valid runs available in the data
        directory

    - nav_data: Array of navigation data of shape (N x R), where N is the length of the
        master time vector and R is the number of valid runs available in the data
        directory

    Outputs:
    - nav_error:          Error between the navigation and truth data (per run) (N x R)
    - nav_error_mean:     Mean of navigation error (N x 1)
    - nav_error_variance: Variance of navigation error over time (N x 1)
    """
    nav_error = nav_data - truth_data
    return (nav_error, np.average(nav_error, 1), np.var(nav_error, 1))

  def _plot_states(self, ax: plt.Axes, header_num: int, ftype: str, ylabel: str) -> np.ndarray:
    """
    Given an axis to plot on, plot the requested data type for a given state

    Inputs:
    - ax:         plt.Axes object with which to plot the data
    - header_num: Column number (representing a state) to extract data from
    - ftype:      File type to look at (truth or nav)
    - ylabel:     y-axis label to add to plot

    Outputs:
    - data: (N x R) data array where N is the number of time steps and R is the
        number of runs. This output is utilized to calculate navigation error later.
    """
    # Get data out based on type
    data = self._get_data(header_num, ftype)

    # Calculate disp
    [mean, disp_data] = self._calculate_disp(data)

    # Plot original truth data
    for run in range(0, self._nruns):
      ax.plot(self._master_time, data[:, run], 'y')

    # Plot Mean
    ax.plot(self._master_time, mean, 'r--', label='Mean')

    # Set plot information
    ax.set_ylabel(ylabel)

    return data

  def _plot_dispersions(self, ax: plt.Axes, header_num: int, ftype: str, ylabel: str, data: np.array = None) -> np.ndarray:
    """
    Given an axis to plot on, plot the requested data type dispersions for a given
    state with 3-sigma bounds on top

    Inputs:
    - ax:         plt.Axes object with which to plot the data
    - header_num: Column number (representing a state) to extract data from
    - ftype:      File type to look at (truth or nav)
    - ylabel:     y-axis label to add to plot
    - data:       Optionally provide the data to plot

    Outputs:
    - data: (N x R) data array where N is the number of time steps and R is the
        number of runs. This output is utilized to calculate navigation error later.
    """
    # Get data out based on type
    if data is None:
        data = self._get_data(header_num, ftype)

    # Calculate disp
    [mean, disp_data] = self._calculate_disp(data)

    # Plot original truth data
    for run in range(0, self._nruns):
      ax.plot(self._master_time, data[:, run] - mean, 'y')

    # Plot dispersion boundaries on top of it
    ax.plot(self._master_time,  3*np.sqrt(disp_data), 'r--', label='3-sigma Bounds')
    ax.plot(self._master_time, -3*np.sqrt(disp_data), 'r--')

    # Set plot information
    ax.set_ylabel(ylabel)

    return data

  def _plot_nav_error(self, ax: plt.Axes, truth_data: np.ndarray, nav_data: np.ndarray, ylabel: str) -> None:
    """
    Given an axis to plot on, plot the navigation error with 3-sigma bounds on top

    Inputs:
    - ax:         plt.Axes object with which to plot the data
    - header_num: Column number (representing a state) to extract data from
    - truth_data: Truth state data
    - nav_data:   Navigation state data
    - ylabel:     y-axis label to add to plot
    """
    [nav_error, nav_error_mean, nav_error_var] = self._calculate_navigation_error(truth_data=truth_data, nav_data=nav_data)

    # Plot original error data
    for run in range(0, self._nruns):
      ax.plot(self._master_time, nav_error[:, run], 'y')

    # Plot 3-sigma bounds
    ax.plot(self._master_time, nav_error_mean+3*np.sqrt(nav_error_var), 'r--', label='3-sigma Bounds')
    ax.plot(self._master_time, nav_error_mean-3*np.sqrt(nav_error_var), 'r--')

    # Set plot information
    ax.set_ylabel(ylabel)

  def _plot_group_plots(self, group_vector: list[int], current_plot_count: int, total_plot_count: int) -> tuple[list[plt.Figure], int]:
    """
    Plot the truth dispersion, navigation dispersion, and navigation error for a complete
    set of states (e.g. "Position", "Euler Angles", etc)

    Inputs:
    - group_vector:       A list of indices that correspond to the groups of data the user
                          selected.

    - current_plot_count: The number of plots that have been completed so far. Used for
                          providing visual progress tracking to the user.

    - total_plot_count:   The total number of expected plots. Used for providing visual
                          progress tracking to the user.

    Outputs:
    - plot_list: A list of new plt.Figures on which the data was plotted. Used for saving
                 to PDF if all plots were requested.

    - current_plot_count: An updated number for the current plot count
    """
    plot_list: list[list[plt.Figure]] = [[], [], [], [], []]
    if group_vector is not None:
      # Plot all selected groups (or save to PDF)
      if len(group_vector) != 0:
        # For each group that was selected
        for group_num in group_vector:
          plot_title = self._group_dict[self._k_group_name][group_num]

          # Create figure handles for each plot that will be used
          truth_plot = plt.figure()
          nav_plot = plt.figure()
          truth_disp_plot = plt.figure()
          nav_disp_plot = plt.figure()
          nav_error_plot = plt.figure()

          # Add to col_range because plotting function grabs time at 0
          col_range = range(self._group_dict[self._k_start_idx][group_num]+1, \
                            self._group_dict[self._k_end_idx][group_num]+2)

          n_cols = len(col_range)

          # Get set of axes for each expected plot
          mean_truth_axes = truth_plot.subplots(n_cols, 1)
          mean_nav_axes = nav_plot.subplots(n_cols, 1)
          truth_axes = truth_disp_plot.subplots(n_cols, 1)
          nav_axes = nav_disp_plot.subplots(n_cols, 1)
          error_axes = nav_error_plot.subplots(n_cols, 1)

          # Iterate over all columns in the group (plot data for one at a time)
          for col in range(0, n_cols):
            # Setup
            ylabel = self._group_dict[self._k_name_list][group_num][col]

            # Plot truth mean
            truth_data = self._plot_states(mean_truth_axes[col], col_range[col], ftype=self._truth_prefix, ylabel=ylabel)

            # Plot nav mean
            nav_data = self._plot_states(mean_nav_axes[col], col_range[col], ftype=self._nav_prefix, ylabel=ylabel)

            # Plot truth dispersions
            self._plot_dispersions(truth_axes[col], col_range[col], ftype=self._truth_prefix, ylabel=ylabel, data=truth_data)

            # Plot nav dispersions
            self._plot_dispersions(nav_axes[col], col_range[col], ftype=self._nav_prefix, ylabel=ylabel, data=nav_data)

            # Plot nav error
            self._plot_nav_error(error_axes[col], truth_data=truth_data, nav_data=nav_data, ylabel=ylabel)

            # Update status
            current_plot_count += 1
            self._menu.status_update(current_plot_count, total_plot_count)


          # Set figure information for all plot types
          mean_truth_axes[0].set_title(plot_title + " Truth Average")
          mean_truth_axes[-1].set_xlabel("Time (s)")
          mean_truth_axes[0].legend()

          mean_nav_axes[0].set_title(plot_title + " Nav Average")
          mean_nav_axes[-1].set_xlabel("Time (s)")
          mean_nav_axes[0].legend()

          truth_axes[0].set_title(plot_title + " Truth Dispersions")
          truth_axes[-1].set_xlabel("Time (s)")
          truth_axes[0].legend()

          nav_axes[0].set_title(plot_title + " Nav Dispersions")
          nav_axes[-1].set_xlabel("Time (s)")
          nav_axes[0].legend()

          error_axes[0].set_title(plot_title + " Nav Error")
          error_axes[-1].set_xlabel("Time (s)")
          error_axes[0].legend()

          # Form list of plots to return
          plot_list[0].append(truth_disp_plot)
          plot_list[1].append(nav_disp_plot)
          plot_list[2].append(nav_error_plot)
          plot_list[3].append(truth_plot)
          plot_list[4].append(nav_plot)

      # Return all axes and plot counts
      return (plot_list, current_plot_count)

    else:
      return(None, current_plot_count)

  def _plot_single_plots(self, single_vector: list[int], current_plot_count: int, total_plot_count: int) -> tuple[list[plt.Figure], int]:
    """
    Plot the truth dispersion, navigation dispersion, and navigation error for a single
    state (e.g. "Position X", "Euler Angles Theta", etc)

    Inputs:
    - single_vector:  A list of indices that correspond to the individual states
                      the user selected.

    - current_plot_count: The number of plots that have been completed so far. Used for
                          providing visual progress tracking to the user.

    - total_plot_count:   The total number of expected plots. Used for providing visual
                          progress tracking to the user.

    Outputs:
    - plot_list: A list of new plt.Figures on which the data was plotted. Used for saving
                 to PDF if all plots were requested.

    - current_plot_count: An updated number for the current plot count
    """
    plot_list: list[list[plt.Figure]] = [[], [], [], [], []]
    if single_vector is not None:
      # Plot all selected groups (or save to PDF)
      if len(single_vector) != 0:
        # For each group that was selected
        for single_num in single_vector:
          col = single_num
          plot_title = self._header_list[col]

          # Create figure handles for each plot that will be used
          truth_plot = plt.figure()
          mean_truth_axis = plt.axes()

          nav_plot = plt.figure()
          mean_nav_axis = plt.axes()

          truth_disp_plot = plt.figure()
          truth_axis = plt.axes()

          nav_disp_plot = plt.figure()
          nav_axis = plt.axes()

          nav_error_plot = plt.figure()
          error_axis = plt.axes()

          # Setup
          ylabel = self._ylabel_list[col]

          # Plot truth mean
          truth_data = self._plot_states(mean_truth_axis, col+1, ftype=self._truth_prefix, ylabel=ylabel)

          # Plot nav mean
          nav_data = self._plot_states(mean_nav_axis, col+1, ftype=self._nav_prefix, ylabel=ylabel)

          # Plot truth dispersions
          self._plot_dispersions(truth_axis, col+1, ftype=self._truth_prefix, ylabel=ylabel, data=truth_data)

          # Plot nav dispersions
          self._plot_dispersions(nav_axis, col+1, ftype=self._nav_prefix, ylabel=ylabel, data=nav_data)

          # Plot nav error
          self._plot_nav_error(error_axis, truth_data=truth_data, nav_data=nav_data, ylabel=ylabel)

          # Update status
          current_plot_count += 1
          self._menu.status_update(current_plot_count, total_plot_count)

          # Set figure information for all plot types
          mean_truth_axis.set_title(plot_title + " Truth Average")
          mean_truth_axis.set_xlabel("Time (s)")
          mean_truth_axis.legend()

          mean_nav_axis.set_title(plot_title + " Nav Average")
          mean_nav_axis.set_xlabel("Time (s)")
          mean_nav_axis.legend()

          truth_axis.set_title(plot_title + " Truth Dispersions")
          truth_axis.set_xlabel("Time (s)")
          truth_axis.legend()

          nav_axis.set_title(plot_title + " Nav Dispersions")
          nav_axis.set_xlabel("Time (s)")
          nav_axis.legend()

          error_axis.set_title(plot_title + " Nav Error")
          error_axis.set_xlabel("Time (s)")
          error_axis.legend()

          # Form list of plots to return
          plot_list[0].append(truth_disp_plot)
          plot_list[1].append(nav_disp_plot)
          plot_list[2].append(nav_error_plot)
          plot_list[3].append(truth_plot)
          plot_list[4].append(nav_plot)

      # Return all axes and plot counts
      return (plot_list, current_plot_count)

    else:
      return (None, current_plot_count)

  # === PUBLIC FUNCTIONS ===
  def run(self) -> None:
    """
    Run plotting operations via a menu until the user quits the program.
    """

    while True:
      # Create MCPlotterMenu and get inputs
      (group_vector, singles_vec, return_state) = self._menu.run()

      # Get counts for status update
      total_plot_count = 0
      current_plot_count = 0

      for g in group_vector:
        new_plots = self._group_dict[self._k_end_idx][g] - self._group_dict[self._k_start_idx][g] + 1
        total_plot_count += new_plots

      if singles_vec is not None:
        total_plot_count += len(singles_vec)

      # Begin displaying progress status to user
      self._menu.status_update(0, total=total_plot_count)

      save_to_pdf = False
      filename = ''
      default_plot_backend = matplotlib.get_backend();

      # If using automatic mode, that means all plots will be saved to a PDF
      if return_state == MCPlotterMenu.STATE_AUTO_MODE:
        save_to_pdf = True
        filename = str(Path(self._data_dir) / "all_plots.pdf")
        pdf = backend_pdf.PdfPages(filename)
        matplotlib.use('PDF')
        plt.switch_backend('PDF')

      # Get all group plots
      [g_axes, current_plot_count] = self._plot_group_plots(group_vector, current_plot_count, total_plot_count)

      if save_to_pdf and g_axes is not None:
        # Save plots to PDF
        for axis in range(0, len(g_axes[0][:])):
          pdf.savefig(g_axes[3][axis])
          pdf.savefig(g_axes[4][axis])
          pdf.savefig(g_axes[0][axis])
          pdf.savefig(g_axes[1][axis])
          pdf.savefig(g_axes[2][axis])

          # Clear figures to save memory (hopefully)
          g_axes[0][axis].clf()
          g_axes[1][axis].clf()
          g_axes[2][axis].clf()
          g_axes[3][axis].clf()
          g_axes[4][axis].clf()

          # Close plots
          plt.close(g_axes[0][axis])
          plt.close(g_axes[1][axis])
          plt.close(g_axes[2][axis])
          plt.close(g_axes[3][axis])
          plt.close(g_axes[4][axis])

      matplotlib.use(default_plot_backend)
      plt.switch_backend(default_plot_backend)

      # Print any individually selected columns
      [s_axes, current_plot_count] = self._plot_single_plots(singles_vec, current_plot_count, total_plot_count)

      if not save_to_pdf and s_axes is not None:
        # Show plots
        plt.show()
        self._menu.print_success()

      if save_to_pdf:
        pdf.close()
        self._menu.print_success(filepath=filename)


def main(args=None):
  """Initialize a plotter and run it"""
  plotter = MCPlotter(args.data_dir)
  plotter.run()


if __name__ == "__main__":
  # Get data directory from user in order to run
  parser = argparse.ArgumentParser()
  parser.add_argument("data_dir", type=str, help="Directory storing MC data")
  args = parser.parse_args()
  main(args)
