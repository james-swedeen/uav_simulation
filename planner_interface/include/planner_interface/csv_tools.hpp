/**
 * @File: csv_tools.hpp
 * @Date: January 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Defines sets of strings used to manage writing to CSV files
 **/

#ifndef PLANNER_INTERFACE__CSV_TOOLS_HPP_
#define PLANNER_INTERFACE__CSV_TOOLS_HPP_

/* C++ Headers */
#include<string>
#include<vector>

/* Eigen Headers */
#include<Eigen/Dense>

namespace planner_interface{
namespace csvtools{
/**
 * @CSVHeaders
 *
 * @brief
 * The headers to be used for each plot type in csv files
 **/
struct CSVHeaders
{
public:
  inline static const std::vector<std::string> STATE =
  {"Time", "X", "Y", "Z"};

  inline static const std::vector<std::string> LC_MC_COMBINED =
  {"Time", "3-sigma LC X", "3-sigma LC Y", "3-sigma LC Z",
   "3-sigma MC X", "3-sigma MC Y", "3-sigma MC Z"};

  inline static const std::vector<std::string> LC_ONLY =
  {"Time", "3-sigma LC X", "3-sigma LC Y", "3-sigma LC Z"};

  inline static const std::vector<std::string> MC_ONLY =
  {"Time", "3-sigma MC X", "3-sigma MC Y", "3-sigma MC Z"};

  inline static const std::vector<std::string> MC_RUN_HEADERS =
  {"Time", "X", "Y", "Z"};

  inline static const std::vector<std::string> PDVG =
  {"Time", "PD", "PD Uncertainty", "Range", "Radar Cross Section"};

  inline static const std::vector<std::string> ERROR_BUDGET =
  {"Time", "Total", "Heading Bias", "Heading", "Abs Pressure Bias", "Feature Range Bias",
   "Feature Bearing Bias", "GPS Position Bias", "GPS Position", "Gyro Bias",
   "Gyroscope", "Accel Bias", "Accelerometer", "Radar Position", "Radar Constant",
   "Initial Truth State Variance"};

  inline static const int TIME_IDX = 0;
  inline static const int FIRST_X_IDX = 1;
  inline static const int SECOND_X_IDX = 4;
  inline static const int THIRD_X_IDX = 7;

  inline static const int STATE_REF_FLAG = 0;
  inline static const int STATE_TRUTH_FLAG = 1;
  inline static const int STATE_NAV_FLAG = 2;

  // Define constants for Error Budget headers as not all are guaranteed to appear
  // in a given plotErrorBudget call
  struct ERROR_BUDGET_IDXS
  {
  public:
    inline static const int TIME            = 0;
    inline static const int TOTAL           = 1;
    inline static const int HEADING_BIAS    = 2;
    inline static const int HEADING         = 3;
    inline static const int ABS_PRESSURE    = 4;
    inline static const int FEATURE_RANGE   = 5;
    inline static const int FEATURE_BEARING = 6;
    inline static const int GPS_BIAS        = 7;
    inline static const int GPS             = 8;
    inline static const int GYRO_BIAS       = 9;
    inline static const int GYRO            = 10;
    inline static const int ACCEL_BIAS      = 11;
    inline static const int ACCEL           = 12;
    inline static const int RADAR_POS       = 13;
    inline static const int RADAR_CONST     = 14;
    inline static const int INIT_COVARIANCE = 15;
  };
};

// Define the titles to be used for all of the possible plots
struct PlotTitles
{
public:
  inline static const std::string POSITION        = "Position";
  inline static const std::string EULER           = "Euler Angles";
  inline static const std::string VELOCITY        = "NED Velocity";
  inline static const std::string HEADING         = "Heading Bias";
  inline static const std::string ABS_PRESSURE    = "Abs Pressure Bias";
  inline static const std::string FEATURE_RANGE   = "Feature Range Bias";
  inline static const std::string FEATURE_BEARING = "Feature Bearing Bias";
  inline static const std::string GPS             = "GPS Position Bias";
  inline static const std::string GYRO            = "Gyro Bias";
  inline static const std::string ACCEL           = "Accel Bias";
};

// Define strings that form the data directory structure and file names
struct CSVNames
{
public:
  // File names (data types)
  struct FileNames
  {
    inline static const std::string POSITION        = "position";
    inline static const std::string EULER           = "euler";
    inline static const std::string VELOCITY        = "velocity";
    inline static const std::string HEADING         = "heading_bias";
    inline static const std::string ABS_PRESSURE    = "abs_pressure_bias";
    inline static const std::string FEATURE_RANGE   = "feature_range_bias";
    inline static const std::string FEATURE_BEARING = "feature_bearing_bias";
    inline static const std::string GPS             = "gps_position_bias";
    inline static const std::string GYRO            = "gyro_bias";
    inline static const std::string ACCEL           = "accel_bias";
    inline static const std::string MC_RUN          = "mc_run";
    inline static const std::string PDVG            = "pdvg";
    inline static const std::string ERROR_BUDGET    = "error_budget";
  };

  // Directory names (plot types)
  struct DirectoryNames
  {
    inline static const std::string STATE               = "state";
    inline static const std::string TRUTH_DISP          = "truth_disp";
    inline static const std::string NAV_DISP            = "nav_disp";
    inline static const std::string EST_ERROR           = "est_error";
    inline static const std::string TRUTH_DISP_OFF_REF  = "truth_disp_off_ref";
    inline static const std::string NAV_DISP_OFF_REF    = "nav_disp_off_ref";
    inline static const std::string MC_RUNS             = "mc_runs";
  };

  // File suffixes
  struct FileSuffix
  {
    inline static const std::string THREE_SIG_SUFFIX     = "_uncertainty.csv";
    inline static const std::string REF_FILE_SUFFIX      = "_ref.csv";
    inline static const std::string TRUTH_FILE_SUFFIX    = "_truth.csv";
    inline static const std::string NAV_FILE_SUFFIX      = "_nav.csv";
    inline static const std::string DEFAULT_FILE_SUFFIX  = ".csv";
  };
};
}  // namespace csvtools
}  // namespace planner_interface
#endif  // PLANNER_INTERFACE__CSV_TOOLS_HPP_
