#!/usr/bin/env python3
"""
This module defines a node used to manage plotting requests
from the lincov_interface and pdvg_interface nodes.
The node provides information to the main loop, which calls
the appropriate functions to plot the requested information.
"""

import matplotlib.backends.backend_pdf as backend_pdf
import matplotlib.pyplot as plt
import pandas as pd
import planner_interface.csv_tools as csv
import rclpy
from rclpy.node import Node
from pathlib import Path
from numpy import ndarray
from multiprocessing import Process
from uav_interfaces.srv import SelectAnalysisPlot
class CSVPlotter(Node):
  """
  This class defines a simple service interface between the
  lincov_interface node and plotting capability defined outside
  of this class. The object of this node is to simply collect
  information about the types of plots requested.

  After that information is requested, the main thread can operate on
  that information via getters and setters. The plotting needs to be done
  in the main thread because of how matplotlib operates
  """
  def __init__(self) -> None:
    """
    Creates a CSVPlotter node.

    Initializes a service through which plotting requests can be made,
    and create variables used for operation
    """
    super().__init__('csv_plotter')

    # Received request. Simplifies function calls from main thread
    self._request: SelectAnalysisPlot.Request

    self._request_received = False

    # Define service
    self._plot_service = self.create_service(SelectAnalysisPlot, 'csv_plotter', self._csv_plot_callback)

  def _csv_plot_callback(self, request: SelectAnalysisPlot.Request, response: SelectAnalysisPlot.Response) -> SelectAnalysisPlot.Response:
    """
    Store information about the requested plots in a dictionary

    Returns:
    - SelectAnalysisPlot response
    """
    # Store request data
    self._request = request
    self._request_received = True

    # Return a simple response
    response.success = True
    response.message = "Saved plotting info"
    return response

  def is_plot_requested(self) -> bool:
    """ Return a flag indicating if a request has been made
    """
    # Return member variable indicating if plot request made
    return self._request_received

  def set_plot_requested(self, set_val) -> None:
    """ Setter used to set the flag that triggers plotting operations
    """
    # Set request received to set_val, used for after plotting is finished to
    # set back to false
    self._request_received = set_val

  def get_plot_info(self) -> SelectAnalysisPlot.Request:
    """
    Return the service request stored from the service callback.
    """
    return self._request

  def log_pdf_done(self) -> None:
    """
    Use logger to indicate that PDF generation has been completed
    """
    self._logger.info("\033[92mPDF Generation Complete\033[0m")

def plot_csv(plot_info: SelectAnalysisPlot.Request, pd_threshold: float, std_dev_multiple: float) -> None:
  """
  Use the information received in request
  to plot data from csv files in the data directory

  Args:
  - plot_info: SelectAnalysisPlot Request that contains information about how
               to generate plots
  - pd_threshold: The probability of detection threshold. This is fed through to the probability of
    detection graphs if all of the plots have been requested for a PDF.
  - std_dev_multiple: Standard deviation multiple for showing x-sigma borders in plots. Again, a
    passthrough variable for if a PDF has been requested.
  """
  # Extract information from dictionary
  is_pdvg_request = plot_info.is_pdvg_request
  file_names = csv.FILE_NAMES
  plot_titles = csv.PLOT_TITLES
  plot_types: list[bool] = plot_info.plot_types
  data_types: list[bool] = plot_info.data_types
  data_dir:          str = plot_info.data_directory
  save_to_pdf:      bool = plot_info.save_to_pdf
  mc_run_downsample: int = plot_info.mc_run_downsample

  # Set the correct directory names and titles to use
  if is_pdvg_request:
    directory_names = csv.DIRECTORY_NAMES_PDVG
    title_str =       csv.TITLE_STR_PDVG
  else:
    directory_names = csv.DIRECTORY_NAMES
    title_str =       csv.TITLE_STR

  type_len = len(plot_types)
  data_type_len = len(data_types)

  filename = data_dir
  plot_type_str = ""
  data_type_str = ""

  if save_to_pdf:
    # Make the pdf file
    filename = str(Path(data_dir) / csv.PDF_NAME)
    pdf = backend_pdf.PdfPages(filename)
    # Add probability of detection and error budget plots to PDF first
    plot_pd_csv(plot_info, pd_threshold, std_dev_multiple, pdf)
    plot_error_budget(plot_info, std_dev_multiple, pdf)

  for type_it in range(0, type_len):
    # If not a state type, in other words, for any plots that have 3-sigma data
    if type_it > 0:
      if plot_types[type_it] is True:
        # Open correct file on switch statement
        plot_type_str = directory_names[type_it]
        for data_it in range(0, data_type_len):
          # Select proper data type string
          if data_types[data_it] is True:
            data_type_str = file_names[data_it]

            # Access file directory
            filedir = str(Path(data_dir) / plot_type_str / data_type_str)
            filename = str(Path(filedir) / (data_type_str + csv.THREE_SIG_SUFFIX))

            # Open 3-sigma file and plot bounds
            three_sig_data = pd.read_csv(filename).to_numpy()

            # Find dimension and subtract one off for time column
            ndim = len(three_sig_data[0])-1
            dim_per_figure = 0
            lc_only = False
            mc_only = False
            if ndim == csv.COMBO_3DFILEDIM:
              dim_per_figure = 3
              lc_only = False
            elif ndim == csv.COMBO_1DFILEDIM:
              dim_per_figure = 1
              lc_only = False
            elif ndim == csv.ONETYPE_3DFILEDIM:
              dim_per_figure = 3
              lc_only = True
            elif ndim == csv.ONETYPE_1DFILEDIM:
              dim_per_figure = 1
              lc_only = True
            else:
              print("Something is wrong with number of columns in " + filename)

            # Change flags if there's only MC data
            if type_it >= csv.TRUTH_DISP_REF_IDX:
              mc_only = True
              lc_only = False

            # Start making plot
            time = three_sig_data[:, 0]
            han = plt.figure()
            if dim_per_figure > 1:
              if (not lc_only or mc_only) and not is_pdvg_request:
                # Find and plot MC runs using provided downsample rate
                mc_runs_dir = Path(filedir) / csv.MC_RUNS_DIR_NAME
                count = 0
                for run in mc_runs_dir.glob('*'):
                  if (count * (not save_to_pdf)) % mc_run_downsample == 0:
                    run_data = pd.read_csv(str(run)).to_numpy()
                    for dim in range(0, dim_per_figure):
                      plt.subplot(dim_per_figure, 1, dim+1)
                      plt.plot(time, run_data[:, dim+1], color='0.8', linewidth=1.0)

                  count += 1

              for dim in range(0, dim_per_figure):
                plt.subplot(dim_per_figure, 1, dim+1)
                # Plot positive and negative bound of data
                if not mc_only and not is_pdvg_request:
                  # Plot LC data
                  plt.plot(time, three_sig_data[:, dim+1], 'b-.', label=csv.LEGEND_LC)
                  plt.plot(time, -three_sig_data[:, dim+1], 'b-.')

                  # Plot MC data
                  if not lc_only:
                    plt.plot(time, three_sig_data[:, dim+4], 'r-.', label=csv.LEGEND_MC)
                    plt.plot(time, -three_sig_data[:, dim+4], 'r-.')
                elif is_pdvg_request:
                  # Plot PDVG data
                  plt.plot(time, three_sig_data[:, dim+1], 'b-.', label=csv.LEGEND_PDVG)
                  plt.plot(time, -three_sig_data[:, dim+1], 'b-.')
                else:
                  # Plot MC data
                  plt.plot(time, three_sig_data[:, dim+1], 'r-.', label=csv.LEGEND_MC)
                  plt.plot(time, -three_sig_data[:, dim+1], 'r-.')

              # Add finishing touches
              for dim in range(0, dim_per_figure):
                plt.subplot(dim_per_figure, 1, dim+1)
                if dim == 0:
                  plt.title(plot_titles[data_it] + " " \
                            + title_str[type_it])
                  plt.legend()

                if dim == dim_per_figure-1:
                  plt.xlabel(csv.XLABEL)

                plt.ylabel(plot_titles[data_it] + " " \
                           + csv.YLABEL_STR[dim])

            else:  # Single dimensional values
              if (not lc_only or mc_only) and not is_pdvg_request:
                # Now find and plot MC runs using provided downsample rate
                mc_runs_dir = Path(filedir) / csv.MC_RUNS_DIR_NAME
                count = 0
                for run in mc_runs_dir.glob('*'):
                  if (count * (not save_to_pdf)) % mc_run_downsample == 0:
                    run_data = pd.read_csv(str(run)).to_numpy()
                    plt.plot(time, run_data[:, 1], color='0.8', linewidth=1.0)

                  count += 1

              # Plot positive and negative bound of data
              if not mc_only and not is_pdvg_request:
                # Plot LC data
                plt.plot(time, three_sig_data[:, 1], 'b-.', label=csv.LEGEND_LC)
                plt.plot(time, -three_sig_data[:, 1], 'b-.')

                if not lc_only:
                  # Plot MC data
                  plt.plot(time, three_sig_data[:, 2], 'r-.', label=csv.LEGEND_MC)
                  plt.plot(time, -three_sig_data[:, 2], 'r-.')
              elif is_pdvg_request:
                # Plot PDVG data
                plt.plot(time, three_sig_data[:, 1], 'b-.', label=csv.LEGEND_PDVG)
                plt.plot(time, -three_sig_data[:, 1], 'b-.')
              else:
                # Plot MC data
                plt.plot(time, three_sig_data[:, 1], 'r-.', label=csv.LEGEND_MC)
                plt.plot(time, -three_sig_data[:, 1], 'r-.')

              # Add finishing touches
              plt.title(plot_titles[data_it] + " " \
                        + title_str[type_it])
              plt.legend()
              plt.xlabel(csv.XLABEL)
              plt.ylabel(plot_titles[data_it] + " " + csv.YLABEL_STR[0])

          # Save to pdf and clear memory
          if save_to_pdf:
            pdf.savefig(han)
            han.clf()
            plt.close(han)
    else:
      # Case where state plots have been requested
      if plot_types[type_it] is True:
        plot_type_str = directory_names[type_it]

        # Iterate through data types requested
        for data_it in range(0, data_type_len):
          # Select proper data type string
          if data_types[data_it] is True:
            data_type_str = file_names[data_it]
            filedir =  str(Path(data_dir) / plot_type_str / data_type_str)

            # Don't know how many files might be there, so append to lists
            legend_list: list[str] = []
            data_list: list[ndarray] = []
            color_list: list[str] = []

            # Check to see if any files exist (some combinations of requests don't
            # produce a file)
            file_count = 0
            for f in Path(filedir).iterdir():
              if f.is_file():
                file_count += 1

            if file_count == 0:
              continue

            # Find Ref, Truth, and Nav files
            for f in Path(filedir).glob('*'):
              f_str = str(f)
              if csv.REF_FILE_SUFFIX in f_str:
                legend_list.insert(0, "Reference")
                color_list.insert(0, 'b')
                data_list.insert(0, pd.read_csv(f_str).to_numpy())

              elif csv.TRUTH_FILE_SUFFIX in f_str:
                legend_list.append("Avg. Truth")
                color_list.append('g')
                data_list.append(pd.read_csv(f_str).to_numpy())

              elif csv.NAV_FILE_SUFFIX in f_str:
                legend_list.append("Avg. Nav")
                color_list.append('r')
                data_list.append(pd.read_csv(f_str).to_numpy())


            # Check dimensions from one of the files (remove time column)
            dim_per_figure = len(data_list[0][0])-1
            time = data_list[0][:, 0]
            han = plt.figure()

            if dim_per_figure > 1:
              # Plot 3D plots
              for dim in range(0, dim_per_figure):
                plt.subplot(dim_per_figure, 1, dim + 1)
                for x in range(0, len(data_list)):
                  plt.plot(time, data_list[x][:, dim+1], color_list[x])

              # Add finishing touches
              for dim in range(0, dim_per_figure):
                plt.subplot(dim_per_figure, 1, dim+1)
                if dim == 0:
                  plt.title(plot_titles[data_it] + " " \
                            + title_str[type_it])
                  plt.legend(legend_list)

                if dim == dim_per_figure-1:
                  plt.xlabel(csv.XLABEL)

                plt.ylabel(plot_titles[data_it] + " " \
                           + csv.YLABEL_STR[dim])
            else:
              # Plot 1D plots
              for x in range(0, len(data_list)):
                plt.plot(time, data_list[x][:, 1], color_list[x])

              # Add finishing touches
              plt.title(plot_titles[data_it] + " " \
                        + title_str[type_it])
              plt.legend(legend_list)
              plt.xlabel(csv.XLABEL)
              plt.ylabel(plot_titles[data_it] + " " \
                         + csv.YLABEL_STR[0])

          # Save to pdf and clear memory
          if save_to_pdf:
            pdf.savefig(han)
            han.clf()
            plt.close(han)

  # Close pdf file if save_to_plot is requested, otherwise, show all requested plots
  if save_to_pdf:
    pdf.close()
  else:
    plt.show()

def plot_pd_csv(plot_info: SelectAnalysisPlot.Request, pd_threshold: float, std_dev_mult: float = 3.0, pdf: backend_pdf.PdfPages = None) -> None:
  """
  Makes the Probability of Detection plot in either pdf or interactive form

  Args:
  - plot_info: SelectAnalysisPlot request containing info about what to do with data
  - pd_threshold: The threshold value for the probability of detection when planning
  - std_dev_mult: The standard deviation for plot
  - pdf: A PdfPages object that can be used to save to pdf
  """
  # If file exists, open the file
  filename = Path(plot_info.data_directory) / (csv.PDVG_FILE_NAME + csv.DEFAULT_FILE_SUFFIX)
  if not filename.exists():
    return

  filename = str(filename)

  # Read with pandas
  pd_data = pd.read_csv(filename).to_numpy()

  # PD plot
  han = plt.figure()
  plt.subplot(3, 1, 1)
  plt.plot(pd_data[:, csv.TIME], pd_data[:, csv.PD], 'b')                                       # Probability of detection
  plt.plot(pd_data[:, csv.TIME], pd_data[:, csv.PD] + std_dev_mult*pd_data[:, csv.PD_STD_DEV], 'b--')      # x-sigma bounds
  plt.plot([pd_data[0, csv.TIME], pd_data[-1, csv.TIME]], [pd_threshold, pd_threshold], 'r--')  # PD Threshold
  plt.xlabel("Time")
  plt.ylabel("PD")
  plt.legend(["PD", f"PD + {std_dev_mult}-sigma", "PD Threshold"])
  plt.title("Probability of Detection Plots")

  # Range plot
  plt.subplot(3, 1, 2)
  plt.plot(pd_data[:, csv.TIME], pd_data[:, csv.PD_RANGE], 'b')
  plt.xlabel("Time")
  plt.ylabel("Range")

  # Radar Cross Section plot
  plt.subplot(3, 1, 3)
  plt.plot(pd_data[:, csv.TIME], pd_data[:, csv.PD_RCS], 'b')
  plt.xlabel("Time")
  plt.ylabel("RCS")

  # Save to PDF if that was provided
  if plot_info.save_to_pdf:
    pdf.savefig(han)
    han.clf()
    plt.close(han)
  else:
    plt.show()

def plot_error_budget(plot_info: SelectAnalysisPlot.Request, std_dev_mult: float, pdf: backend_pdf.PdfPages = None) -> None:
  """
  Make a plot or pdf page of the PDVG error budget

  Args:
  - plot_info: SelectAnalysisPlot Request that holds information what to do with data
  - std_dev_mult: Standard deviation for the plotter
  - pdf:       A PDFPages object used to write plots to a PDF. Is None-type if pdf won't be made
  """
  marker_types = ['o', '*', "x"]

  # If file exists, open the file
  filename = Path(plot_info.data_directory) / (csv.ERROR_BUDGET_FILE_NAME + csv.DEFAULT_FILE_SUFFIX)
  if not filename.exists():
    return

  filename = str(filename)

  # Read with pandas
  data_all = pd.read_csv(filename)
  headers: list[str] = list(data_all.columns)
  data_np = data_all.to_numpy()

  # Add all info to the plot
  han = plt.figure()
  for col in range(1, len(headers)):
    plt.plot(data_np[:, csv.TIME],   \
             data_np[:, col],   \
             label=headers[col],\
             marker=marker_types[(col//10)%len(marker_types)])  # Force to cycle colors before marker change

  # Make it look nice
  plt.xlabel("Time")
  plt.ylabel(f"{std_dev_mult}-sigma Error")
  plt.title("PDVG Error Budget")
  plt.legend()

  # Save to PDF if that was provided
  if plot_info.save_to_pdf:
    pdf.savefig(han)
    han.clf()
    plt.close(han)
  else:
    plt.show()

def main(args=None):
  rclpy.init(args=args)

  # Make the CSV plotting node
  plotter = CSVPlotter()

  # Read in plotting variables for pdvg planner
  plotter.declare_parameter("pd_threshold", 0.)
  plotter.declare_parameter("std_dev_multiple", 0.)
  pd_threshold: float = plotter.get_parameter("pd_threshold").value
  std_dev_multiple: float = plotter.get_parameter("std_dev_multiple").value

  # Save the CPU
  plotter.create_rate(1)

  # Use processes to allow for multiple sets of plots to be made
  process_list: list[Process] = []
  while rclpy.ok():
    rclpy.spin_once(plotter)

    # Check if plots have been requested
    if plotter.is_plot_requested():
      plot_info: SelectAnalysisPlot.Request = plotter.get_plot_info()
      name = None
      if plot_info.save_to_pdf:
        name = "csv_pdf_plot"
      else:
        name = "csv_plot"
      plotter.set_plot_requested(False)

      # Decide which type of plot to make depending on flags
      if plot_info.make_pd_plots:
        p = Process(target=plot_pd_csv, args=[plot_info, pd_threshold, std_dev_multiple, None], name=name)
      elif plot_info.make_pdvg_error_budget:
        p = Process(target=plot_error_budget, args=[plot_info, std_dev_multiple, None], name=name)
      else:
        p = Process(target=plot_csv, args=[plot_info, pd_threshold, std_dev_multiple], name=name)
      process_list.append(p)
      p.start()

    # Check any existing processes and kill them if they finished
    for p in process_list:
      if not p.is_alive():
        if p.name == "csv_pdf_plot":
          plotter.log_pdf_done()

        p.kill()
        process_list.remove(p)

  plotter.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()
