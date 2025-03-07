"""
This module defines constants used for creating a data directory to store
LinCov and Monte Carlo results, and the same for reading the files that are
stored there.

Also defined are strings for plotting information that will be displayed to
the user when plotting when simulation is active.

This file is utilized mainly by csv_plotter.py, the other script in this folder,
but is also utilized in the pd_planner_launch package for creating the data
directory.
"""

##########################
### CSV File Constants ###
##########################
# Strings defining plot type directory names.
DIRECTORY_NAMES: list[str] = ["state", "truth_disp", "nav_disp",
                        "est_error", "truth_disp_off_ref", "nav_disp_off_ref"]

DIRECTORY_NAMES_PDVG: list[str] = DIRECTORY_NAMES[0:4]

# Strings defining directory name and first part of uncertainty files
FILE_NAMES: list[str] = ["position", "euler", "velocity", "heading_bias",
                        "abs_pressure_bias", "feature_range_bias",
                        "feature_bearing_bias", "gps_position_bias",
                        "gyro_bias", "accel_bias"]

# Additional file names
MC_RUNS_DIR_NAME        = "mc_runs"
PDF_NAME                = "all_plots.pdf"
PDVG_FILE_NAME          = "pdvg"
ERROR_BUDGET_FILE_NAME  = "error_budget"

# File suffixes
THREE_SIG_SUFFIX    = "_uncertainty.csv"
REF_FILE_SUFFIX     = "_ref.csv"
TRUTH_FILE_SUFFIX   = "_truth.csv"
NAV_FILE_SUFFIX     = "_nav.csv"
DEFAULT_FILE_SUFFIX = ".csv"

# Define constants for PDVG data types that appear in vectors
TIME:       int = 0
PD:         int = 1
PD_STD_DEV: int = 2
PD_RANGE:   int = 3
PD_RCS:     int = 4


# Define constants for referencing certain plot types
TRUTH_DISP_REF_IDX: int = 4  # Dispersions off Reference are plotted differently
COMBO_3DFILEDIM:    int = 6  # LC and MC data, each with XYZ components
ONETYPE_3DFILEDIM:  int = 3  # LC or MC only, with XYZ components
COMBO_1DFILEDIM:    int = 2  # LC and MC data, each with X component only
ONETYPE_1DFILEDIM:  int = 1  # LC or MC only, with X component only

##############################################
###  Matplotlib Plotting Display Constants ###
##############################################
TITLE_STR: list[str] = ["", "Truth Dispersion", "Navigation Dispersion",
                        "Est. Error", "Truth Dispersion off Reference",
                        "Navigation Dispersion off Reference"]

TITLE_STR_PDVG: list[str] = TITLE_STR[0:4]

PLOT_TITLES: list[str] = ["Position", "Euler Angles", "NED Velocity", "Heading Bias",
                              "Abs Pressure Bias", "Feature Range Bias",
                              "Feature Bearing Bias", "GPS Position Bias",
                              "Gyro Bias", "Accel Bias"]

YLABEL_STR: list[str] = ["X", "Y", "Z"]
XLABEL      = "Time (s)"
LEGEND_MC   = "3-sigma MC"
LEGEND_LC   = "3-sigma LC"
LEGEND_PDVG = "3-sigma PDVG"
