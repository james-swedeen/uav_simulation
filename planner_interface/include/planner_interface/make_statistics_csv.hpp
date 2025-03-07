/**
 * @File: make_statistics_csv.hpp
 * @Date: January 2023
 * @Author: Daren Swasey
 *
 * @brief
 * Defines functions for making CSV files from LinCov and Monte Carlo data
 * Code based on matplotlib version written by James Swedeen in the
 * rapidly-exploring-random-tree kalman_filter package
 **/

#ifndef PLANNER_INTERFACE__MAKE_STATISTICS_CSV_HPP_
#define PLANNER_INTERFACE__MAKE_STATISTICS_CSV_HPP_

/* C++ Headers */
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

/* Eigen Headers */
#include <Eigen/Dense>

/* Local Headers */
#include <kalman_filter/helpers/plot_statistics_tools.hpp>
#include <kalman_filter/helpers/tools.hpp>
#include <kalman_filter/helpers/versions.hpp>
#include <kalman_filter/mappings/mappings_base.hpp>
#include <kalman_filter/math/performance_evaluation.hpp>
#include <kalman_filter/noise/noise_wrapper.hpp>
#include <kalman_filter/run_lin_cov.hpp>
#include <planner_interface/csv_tools.hpp>
#include <rrt_search/obstacle_checkers/probability_detection_metric_obstacle_checker.hpp>

namespace planner_interface {
namespace csv {
using PlotTypeInds = struct kf::plottools::PlotTypeInds;
using PlotTitles = struct planner_interface::csvtools::PlotTitles;
using CSVNames = struct planner_interface::csvtools::CSVNames;
using CSVHeaders = struct planner_interface::csvtools::CSVHeaders;

template <class SCALAR = double>
using Vector2D = std::vector<std::vector<SCALAR>>;

template <class SCALAR = double>
using Vector3D = std::vector<Vector2D<SCALAR>>;
using NameVector = std::vector<std::pair<std::string, std::tuple<Eigen::Index,
                                                                 Eigen::Index,
                                                                 Eigen::Index,
                                                                 Eigen::Index>>>;

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
using GeneralNoiseWrapper = std::variant<kf::noise::NoiseWrapperPtr<1, SCALAR, OPTIONS>,
                                         kf::noise::NoiseWrapperPtr<3, SCALAR, OPTIONS>>;

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
using DATA_FUNC = std::function<Eigen::Matrix<SCALAR, 1, Eigen::Dynamic, OPTIONS>(const Eigen::Ref<const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>>&)>;

static const int CSV_PRECISION = 6;  // Precision to use when saving data to file

namespace helpers{
/**
 * @brief Enable noise in the noise object depending on its type
 *
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param noise: Noise object
 */
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
void setNoiseEnabled(const GeneralNoiseWrapper<SCALAR, OPTIONS>& noise) noexcept;

/**
 * @brief Disable noise in the noise object depending on its type
 *
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param noise: Noise object
 */
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
void setNoiseDisabled(const GeneralNoiseWrapper<SCALAR, OPTIONS>& noise) noexcept;

/**
 * @brief Get the noise object name
 *
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param noise: Noise object
 */
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
const std::string& getNoiseName(const GeneralNoiseWrapper<SCALAR, OPTIONS>& noise) noexcept;
}  // namespace helpers

/* === DEFINE FUNCTION PROTOTYPES === */
/**
 * @brief Get the appropriate file prefix for the provided plot category
 *
 * @param data_type Data type to be used with a plot type. String must match one of
 * those provided in PlotTitles struct
 *
 * @return std::string containing the file prefix for the respective data type
 */
std::string getPlotCategoryName(const std::string &data_type);

/**
 * @brief Write LinCov-only, Monte Carlo-only, or state plots to file
 *
 * @details Function created to reduce repeated code elsewhere. If requested data
 * involves minimal data, that is, only Monte Carlo data, only LinCov data, or only
 * data from a state plot type, then the requirements for writing to file are
 * equivalent.
 *
 * @tparam SCALAR: Template type indicating what type of data will be written to file
 * @param file: The file to be written to (expected to be open already)
 * @param headers: The vector of header strings that will be used as CSV headers
 * @param sim_len: The total number of time-steps in the simulation
 * @param data_dim: Indicates the dimension of the data, typically 1D or 3D
 * @param time: The vector containing time-steps of the data, written to file in first column
 * @param data: Data to write to file in addition to the time vector
 */
template <typename SCALAR = double>
inline void writeOneTypeOnlyToFile(
  std::ofstream&                  file,
  const std::vector<std::string>& headers,
  const size_t                    sim_len,
  const size_t                    data_dim,
  const std::vector<SCALAR>&      time,
  const Vector2D<SCALAR>&         data);

/**
 * @brief Make a CSV file containing the provided 3-sigma data from MC and/or LC
 *
 * @details Makes a CSV file with 3-sigma data that is structured differently
 * depending on input data. The input LC and MC data are given defaults. The logic
 * for making the CSV file depends on whether the lc/mc_bounds_data is empty. If a
 * state plot type is requested in this function, an error will be thrown.
 *
 * @tparam SCALAR: The object type that each dimension will be represented with
 * @param plot_type_index: Indicates which plot type is being made
 * @param sim_len: Number of steps in the simulation
 * @param time: Time vector for the current simulation
 * @param file_directory: Top-level data directory name
 * @param data_type: Type of data to use in plot (Position, Euler, etc...)
 * @param lc_bounds_data: 3-sigma data from LinCov calculations
 * @param mc_bounds_data: 3-sigma data from Monte Carlo calculations
 */
template<typename SCALAR = double>
void makeThreeSigmaCSV(const Eigen::Index         plot_type_index,
                       const size_t               sim_len,
                       const std::vector<SCALAR>& time,
                       const std::string&         file_directory,
                       const std::string&         data_type,
                       const Vector2D<SCALAR>&    lc_bounds_data = {},
                       const Vector2D<SCALAR>&    mc_bounds_data = {});

/**
 * @brief Make a CSV file containing truth, navigation, or reference state data
 *
 * @details
 *
 * @tparam SCALAR: The object type that each dimension will be represented with
 * @param sim_len: Number of steps in the simulation
 * @param time: Time vector for the current simulation
 * @param file_directory: Top-level data directory name
 * @param data_type: Type of data to use in plot (Position, Euler, etc...)
 * @param state_type_select: Flag that indicates whether to plot nav, truth, or ref data
 * @param data: The vector associated with specified state type.
 */
template<typename SCALAR = double>
void makeStateCSV(const size_t               sim_len,
                  const std::vector<SCALAR>& time,
                  const std::string&         file_directory,
                  const std::string&         data_type,
                  const int                  state_type_select,
                  const Vector2D<SCALAR>&    data);

/**
 * @brief Create a file for each individual Monte Carlo run
 *
 * @details Removes previously existing MC run files and directory and replaces
 * those with files generated from current data.
 *
 * @tparam SCALAR: Data type utilized in plotting
 * @param plot_type_index: The index to be used with the PlotTypeInds struct
 * @param sim_len: Number of steps in a simulation run
 * @param time: Vector of time values
 * @param file_directory: String that holds the top-level directory location
 * @param data_type: String indicating data type to be plotted. Used to get respective file name
 * @param mc_run_data: Data to be written to CSV file
 */
template<typename SCALAR = double>
void makeMonteCarloRunsCSV(const Eigen::Index         plot_type_index,
                           const size_t               sim_len,
                           const std::vector<SCALAR>& time,
                           const std::string&         file_directory,
                           const std::string&         data_type,
                           const Vector3D<SCALAR>&    mc_run_data);

/**
 * @brief Generate the CSV file for the probability of detection plot
 *
 * @tparam DIM_S: Dimensions for MC and LC state vectors (BasicModelDim)
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param pdvg_solution: Data structure containing information about the PDVG solution path
 * @param obs_checker: The obstacle checker used to generate pdvg_solution
 * @param data_directory: Directory to write the file to
 */
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
void makePDPlotCSV(
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>&                pdvg_solution,
    const rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S, false, false, SCALAR, OPTIONS>& obs_checker,
    const std::string&                                                                                  data_directory);

/**
 * @brief Make error budget CSV file
 *
 * @tparam SCALAR: Data type utilized in plotting
 * @param time: The vector containing time-steps of the data, written to first column
 * @param headers: Headers to be used in the CSV file
 * @param data: Vectors of each part of the data to be written to file
 * @param data_directory: Directory to write the file to
 */
template <typename SCALAR = double>
void makePDVGErrorBudgetCSV(const std::vector<SCALAR>&      time,
                            const std::vector<std::string>& headers,
                            const Vector2D<SCALAR>&         data,
                            const std::string&              data_directory);

/**
 * @brief Prepare data for making truth, nav, or error-related CSV files
 *
 * @tparam DIM_S: Dimensions for MC and LC state vectors (BasicModelDim)
 * @tparam SUB_DIM: Sub-dimension value used to generalize across different plot types
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param plot_type_index: Index of plot type for choosing how to create and manipulate data
 * @param data_dir: Directory to store the data in
 * @param names: List of names that indicate the desired data types
 * @param time: Vector for simulation time values
 * @param mappings: A helper object that maps one state vector to another
 * @param mc_state_vectors: Monte Carlo state vectors
 * @param lincov_state_vector: LinCov state vectors
 */
template<typename DIM_S, Eigen::Index SUB_DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
void makeTruthNavErrorCSV(
    const Eigen::Index                                                                            plot_type_index,
    const std::string&                                                                            data_dir,
    const NameVector&                                                                             names,
    const std::vector<SCALAR>&                                                                    time,
    const kf::map::MappingsBasePtr<DIM_S, SCALAR, OPTIONS>&                                       mappings,
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>&          lincov_state_vector,
    const std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::MC::FULL_STATE_LEN, OPTIONS>>& mc_state_vectors = {});

/**
 * @brief Prepare data for making truth, nav, or error-related CSV files
 *
 * @tparam DIM_S: Dimensions for MC and LC state vectors (BasicModelDim)
 * @tparam SUB_DIM: Sub-dimension value used to generalize across different plot types
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param data_dir: Directory to store the data in
 * @param names: List of names that indicate the desired data types
 * @param time: Vector for simulation time values
 * @param mappings: A helper object that maps one state vector to another
 * @param truth_to_plot: Functor to map data to truth values
 * @param nav_to_plot: Functor to map data to nav values
 * @param lincov_state_vector: LinCov state vectors
 * @param mc_state_vectors: Monte Carlo state vectors
 */
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
void makeStateCSVKickoff(
    const std::string&                                                                             data_dir,
    const NameVector&                                                                              names,
    const std::vector<SCALAR>&                                                                     time,
    const kf::map::MappingsBasePtr<DIM_S, SCALAR, OPTIONS>&                                        mappings,
    const std::function<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DISP_DIM, OPTIONS>(
      const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DIM, OPTIONS>&)>&                   truth_to_plot,
    const std::function<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::ERROR_DIM, OPTIONS>(
      const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::NAV_DIM, OPTIONS>&)>&                     nav_to_plot,
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>&           lincov_state_vector,
    const std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::MC::FULL_STATE_LEN, OPTIONS>>&  mc_state_vectors = {});

/**
 * @brief Prepare data for writing to CSV, MC/LC combined overload
 *
 * @tparam DIM_S: BasicModelDim used to access all parts of state vectors
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param mc_state_vectors: Monte Carlo state vectors
 * @param lincov_state_vector: LinCov state vectors
 * @param mappings: A helper object that maps one state vector to another
 * @param truth_to_plot: Functor to map data to truth values
 * @param nav_to_plot: Functor to map data to nav values
 * @param names: List of name pair/tuples that indicate the desired data types and data starting indices
 * @param plot_types: Boolean array indicating what plot types were requested
 * @param data_directory: Directory to store the data in
 */
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
void makeAllStatisticsCSV(
    const std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::MC::FULL_STATE_LEN, OPTIONS>>& mc_state_vectors,
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>&          lincov_state_vector,
    const kf::map::MappingsBasePtr<DIM_S, SCALAR, OPTIONS>&                                       mappings,
    const std::function<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DISP_DIM, OPTIONS>(
      const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DIM, OPTIONS>&)>&                  truth_to_plot,
    const std::function<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::ERROR_DIM, OPTIONS>(
      const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::NAV_DIM, OPTIONS>&)>&                    nav_to_plot,
    const NameVector&                                                                             names,
    const std::array<bool, 6>&                                                                    plot_types,
    const std::string&                                                                            data_directory);

/**
 * @brief Prepare data for writing to CSV, LC-only version overload
 *
 * @tparam DIM_S: BasicModelDim used to access all parts of state vectors
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param lincov_state_vector: LinCov state vectors
 * @param mappings: A helper object that maps one state vector to another
 * @param names: List of name pair/tuples that indicate the desired data types and data starting indices
 * @param plot_types: Boolean array indicating what plot types were requested
 * @param data_directory: Directory to store the data in
 */
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
void makeAllStatisticsCSV(
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>& lincov_state_vector,
    const kf::map::MappingsBasePtr<DIM_S, SCALAR, OPTIONS>&                              mappings,
    const NameVector&                                                                    names,
    const std::array<bool, 4>&                                                           plot_types,
    const std::string&                                                                   data_directory);

/* === FUNCTION DEFINITIONS === */
/**
 * @brief Get the appropriate file prefix for the provided plot category
 *
 * @param data_type Data type to be used with a plot type. String must match one of
 * those provided in PlotTitles struct
 *
 * @return std::string containing the file prefix for the respective data type
 */
std::string getPlotCategoryName(const std::string &data_type){
  // Select correct file name based on provided category
  if (data_type.compare(PlotTitles::POSITION) == 0) {
    return CSVNames::FileNames::POSITION;
  } else if (data_type.compare(PlotTitles::EULER) == 0) {
    return CSVNames::FileNames::EULER;
  } else if (data_type.compare(PlotTitles::VELOCITY) == 0) {
    return CSVNames::FileNames::VELOCITY;
  } else if (data_type.compare(PlotTitles::HEADING) == 0) {
    return CSVNames::FileNames::HEADING;
  } else if (data_type.compare(PlotTitles::ABS_PRESSURE) == 0) {
    return CSVNames::FileNames::ABS_PRESSURE;
  } else if (data_type.compare(PlotTitles::FEATURE_RANGE) == 0) {
    return CSVNames::FileNames::FEATURE_RANGE;
  } else if (data_type.compare(PlotTitles::FEATURE_BEARING) == 0) {
    return CSVNames::FileNames::FEATURE_BEARING;
  } else if (data_type.compare(PlotTitles::GPS) == 0) {
    return CSVNames::FileNames::GPS;
  } else if (data_type.compare(PlotTitles::GYRO) == 0) {
    return CSVNames::FileNames::GYRO;
  } else if (data_type.compare(PlotTitles::ACCEL) == 0) {
    return CSVNames::FileNames::ACCEL;
  } else {
    // Something got messed up
    throw(std::runtime_error("No plot titles match what was provided in arguments"));
  }
}

/**
 * @brief Write LinCov-only, Monte Carlo-only, or state plots to file
 *
 * @details Function created to reduce repeated code elsewhere. If requested data
 * involves minimal data, that is, only Monte Carlo data, only LinCov data, or only
 * data from a state plot type, then the requirements for writing to file are
 * equivalent.
 *
 * @tparam SCALAR: Data type utilized in plotting
 * @param file: The file to be written to (expected to be open already)
 * @param headers: The vector of header strings that will be used as CSV headers
 * @param sim_len: The total number of time-steps in the simulation
 * @param data_dim: Indicates the dimension of the data, typically 1D or 3D
 * @param time: The vector containing time-steps of the data, written to file in first column
 * @param data: Data to write to file in addition to the time vector
 */
template <typename SCALAR>
inline void writeOneTypeOnlyToFile(
  std::ofstream&                  file,
  const std::vector<std::string>& headers,
  const size_t                    sim_len,
  const size_t                    data_dim,
  const std::vector<SCALAR>&      time,
  const Vector2D<SCALAR>&         data){
      if (data_dim == 1) {
        // If data is one-dimensional
        // Write headers to file
        file << headers[CSVHeaders::TIME_IDX] << ",";
        file << headers[CSVHeaders::FIRST_X_IDX] << "\n";

        // Write data to file
        for (size_t i = 0; i < sim_len; i++){
          file << time[i] << ",";
          file << data[0][i] << "\n";
        }
      } else {
        // Data is (typically) three-dimensional
        // Write headers
        for (size_t h = 0; h < headers.size()-1; h++){
          file << headers[h] << ",";
        }
        file << headers[headers.size()-1] << "\n";

        // Write data
        for (size_t i = 0; i < sim_len; i++){
          file << time[i] << ",";  // Write time vector first

          // Write data for all dimensions
          for (size_t dim = 0; dim < data_dim-1; dim++){
            file << data[dim][i] << ",";
          }
          file << data[data_dim-1][i] << "\n";
        }
      }
}

/**
 * @brief Make a CSV file containing the provided 3-sigma data from MC and/or LC
 *
 * @details Makes a CSV file with 3-sigma data that is structured differently
 * depending on input data. The input LC and MC data are given defaults. The logic
 * for making the CSV file depends on whether the lc/mc_bounds_data is empty. If a
 * state plot type is requested in this function, an error will be thrown.
 *
 * @tparam SCALAR: Data type utilized in plotting
 * @param plot_type_index: Indicates which plot type is being made
 * @param sim_len: Number of steps in the simulation
 * @param time: Time vector for the current simulation
 * @param file_directory: Top-level data directory name
 * @param data_type: Type of data to use in plot (Position, Euler, etc...)
 * @param lc_bounds_data: 3-sigma data from LinCov calculations
 * @param mc_bounds_data: 3-sigma data from Monte Carlo calculations
 */
template<typename SCALAR>
void makeThreeSigmaCSV(const Eigen::Index         plot_type_index,
                       const size_t               sim_len,
                       const std::vector<SCALAR>& time,
                       const std::string&         file_directory,
                       const std::string&         data_type,
                       const Vector2D<SCALAR>&    lc_bounds_data,
                       const Vector2D<SCALAR>&    mc_bounds_data){
  using CSVDirNames = CSVNames::DirectoryNames;
  std::ofstream file;
  std::filesystem::path filename;
  std::filesystem::path dir(file_directory);

  // Check that dimensions match if data provided for both lc and mc
  size_t lc_dim = lc_bounds_data.size();
  size_t mc_dim = mc_bounds_data.size();
  bool lc_empty = lc_bounds_data.empty();
  bool mc_empty = mc_bounds_data.empty();
  if ((mc_dim != lc_dim) && !lc_empty && !mc_empty){
    throw(std::runtime_error("Dimension mismatch between LinCov and Monte Carlo data"));
  }

  // Get the category name
  std::string category_filename = getPlotCategoryName(data_type);

  // Create a file with the appropriate file name
  switch(plot_type_index){
    case PlotTypeInds::TRUTH_DISP_PLOTS:
      filename = dir / CSVDirNames::TRUTH_DISP;
      break;

    case PlotTypeInds::NAV_DISP_PLOTS:
      filename = dir / CSVDirNames::NAV_DISP;
      break;

    case PlotTypeInds::EST_ERROR_PLOTS:
      filename = dir / CSVDirNames::EST_ERROR;
      break;

    case PlotTypeInds::TRUTH_DISP_OFF_REF_PLOTS:
      filename = dir / CSVDirNames::TRUTH_DISP_OFF_REF;
      break;

    case PlotTypeInds::NAV_DISP_OFF_REF_PLOTS:
      filename = dir / CSVDirNames::NAV_DISP_OFF_REF;
      break;

    default:  // Should be used in the event that plot_type_index is for states
      throw(std::invalid_argument("Invalid plot type index given"));
  }
  filename = filename / category_filename / (category_filename + CSVNames::FileSuffix::THREE_SIG_SUFFIX);
  file.open(filename.string(), std::ios::out);

  // Store data in file
  if (file.is_open()){
    file << std::fixed << std::setprecision(CSV_PRECISION);

    // Use different headers and writing methods if the inputs are only one dimension
    bool x_only = (lc_dim == 1) || (mc_dim == 1);

    // If plotting both LC and MC data
    if (!mc_empty && !lc_empty) {
      // Add LC and MC headers
      auto headers = CSVHeaders::LC_MC_COMBINED;
      if (x_only) {
        file << headers[CSVHeaders::TIME_IDX] << ",";
        file << headers[CSVHeaders::FIRST_X_IDX] << ",";
        file << headers[CSVHeaders::SECOND_X_IDX] << "\n";

        // Iterate over only one dimension in data vectors
        for (size_t i = 0; i < sim_len; i++){
          file << time[i] << ",";
          file << lc_bounds_data[0][i] << ",";
          file << mc_bounds_data[0][i] << "\n";
        }
      } else {
        // Write headers
        for (size_t h = 0; h < headers.size()-1; h++){
          file << headers[h] << ",";
        }
        file << headers[headers.size()-1] << "\n";

        // Write data
        for (size_t i = 0; i < sim_len; i++){
          file << time[i] << ",";  // Add time data first

          // Add LC data
          for (size_t dim = 0; dim < mc_dim; dim++){
            file << lc_bounds_data[dim][i] << ",";
          }

          // Add MC data
          for (size_t dim = 0; dim < mc_dim-1; dim++){
            file << mc_bounds_data[dim][i] << ",";
          }
          file << mc_bounds_data[mc_dim-1][i] << "\n";
        }
      }
    } else if (!mc_empty) {
      // Add MC headers only (Truth/Nav disp off ref)
      writeOneTypeOnlyToFile<SCALAR>(file, CSVHeaders::MC_ONLY, sim_len, mc_dim, time, mc_bounds_data);
    } else if (!lc_empty) {
      // LC headers only
      writeOneTypeOnlyToFile<SCALAR>(file, CSVHeaders::LC_ONLY, sim_len, lc_dim, time, lc_bounds_data);
    } else {
      // Somehow passed in empty vectors for both LC and MC
      throw(std::invalid_argument("No LinCov or Monte Carlo data"));
    }
  } else {
    // Throw error on failure to open file
    throw(std::runtime_error("Error opening file: " + filename.string()));
  }
}

/**
 * @brief Make a CSV file containing truth, navigation, or reference state data
 *
 * @details
 *
 * @tparam SCALAR: Data type utilized in plotting
 * @param sim_len: Number of steps in the simulation
 * @param time: Time vector for the current simulation
 * @param file_directory: Top-level data directory name
 * @param data_type: Type of data to use in plot (Position, Euler, etc...)
 * @param state_type_select: Flag that indicates whether to plot nav, truth, or ref data
 * @param data: The vector associated with specified state type.
 */
template<typename SCALAR>
void makeStateCSV(const size_t               sim_len,
                  const std::vector<SCALAR>& time,
                  const std::string&         file_directory,
                  const std::string&         data_type,
                  const int                  state_type_select,
                  const Vector2D<SCALAR>&    data){
  std::ofstream file;
  std::filesystem::path filename;

  // Get the category name
  std::string category_filename = getPlotCategoryName(data_type);

  // Create a file with the appropriate file name
  std::string suffix;
  switch(state_type_select){
    case CSVHeaders::STATE_REF_FLAG:
      suffix = CSVNames::FileSuffix::REF_FILE_SUFFIX;
      break;
    case CSVHeaders::STATE_NAV_FLAG:
      suffix = CSVNames::FileSuffix::NAV_FILE_SUFFIX;
      break;
    case CSVHeaders::STATE_TRUTH_FLAG:
      suffix = CSVNames::FileSuffix::TRUTH_FILE_SUFFIX;
      break;
    default:
      throw(std::runtime_error("Incorrect value provided for state_type_select"));
  }
  filename = std::filesystem::path(file_directory) / CSVNames::DirectoryNames::STATE /
             category_filename / (category_filename + suffix);
  file.open(filename.string(), std::ios::out);

  // Store data in file
  if (file.is_open()){
    file << std::fixed << std::setprecision(CSV_PRECISION);

    auto ndim = data.size();
    writeOneTypeOnlyToFile<SCALAR>(file, CSVHeaders::STATE, sim_len, ndim, time, data);

  } else {
    // Throw error on failure to open file
    throw(std::runtime_error("Error opening file: " + filename.string()));
  }
}

/**
 * @brief Create a file for each individual Monte Carlo run
 *
 * @details Removes previously existing MC run files and directory and replaces
 * those with files generated from current data.
 *
 * @tparam SCALAR: Data type to use when writing to CSV
 * @param plot_type_index: The index to be used with the PlotTypeInds struct
 * @param sim_len: Number of steps in a simulation run
 * @param time: Vector of time values
 * @param file_directory: String that holds the top-level directory location
 * @param data_type: String indicating data type to be plotted. Used to get respective file name
 * @param mc_run_data: Data to be written to CSV file
 */
template<typename SCALAR>
void makeMonteCarloRunsCSV(const Eigen::Index         plot_type_index,
                           const size_t               sim_len,
                           const std::vector<SCALAR>& time,
                           const std::string&         file_directory,
                           const std::string&         data_type,
                           const Vector3D<SCALAR>&    mc_run_data){
  using CSVDirNames = CSVNames::DirectoryNames;

  // Get general info for writing to CSV
  std::string category_filename = getPlotCategoryName(data_type);
  auto mc_dim = mc_run_data.size();
  bool x_only = (mc_dim == 1);
  size_t n_runs = mc_run_data[0].size();
  auto headers = CSVHeaders::MC_RUN_HEADERS;
  std::filesystem::path mc_runs_dir;
  std::filesystem::path filename;
  std::filesystem::path dir(file_directory);

  // Create a file with the appropriate file name
  switch(plot_type_index){
    case PlotTypeInds::TRUTH_DISP_PLOTS:
      filename = dir / CSVDirNames::TRUTH_DISP;
      break;

    case PlotTypeInds::NAV_DISP_PLOTS:
      filename = dir / CSVDirNames::NAV_DISP;
      break;

    case PlotTypeInds::EST_ERROR_PLOTS:
      filename = dir / CSVDirNames::EST_ERROR;
      break;

    case PlotTypeInds::TRUTH_DISP_OFF_REF_PLOTS:
      filename = dir / CSVDirNames::TRUTH_DISP_OFF_REF;
      break;

    case PlotTypeInds::NAV_DISP_OFF_REF_PLOTS:
      filename = dir / CSVDirNames::NAV_DISP_OFF_REF;
      break;

    default:  // Should be used in the event that plot_type_index is for states
      throw(std::invalid_argument("Invalid plot type index given"));
  }

  mc_runs_dir = filename / category_filename / CSVDirNames::MC_RUNS;
  // Delete any previous MC runs that may have already been in the folder
  std::filesystem::remove_all(mc_runs_dir.string());
  std::filesystem::create_directory(mc_runs_dir.string());

  // Iterate over each MC run
  for (size_t run_it = 0; run_it < n_runs; run_it++){
    std::ofstream file;
    std::string file_num = std::to_string(run_it);

    filename = mc_runs_dir / (CSVNames::FileNames::MC_RUN + file_num + ".csv");
    file.open(filename.string(), std::ios::out);

    if (file.is_open()){
      file << std::fixed << std::setprecision(CSV_PRECISION);

      // Write differently if x dimension only
      if (x_only){
        // Write headers out
        file << headers[CSVHeaders::TIME_IDX] << ",";
        file << headers[CSVHeaders::FIRST_X_IDX] << "\n";

        for (size_t time_it = 0; time_it < sim_len; time_it++){
          file << time[time_it] << ",";
          file << mc_run_data[0][run_it][time_it] << "\n";
        }
      } else {
        // Write headers
        for (size_t h = 0; h < headers.size()-1; h++){
          file << headers[h] << ",";
        }
        file << headers[headers.size()-1] << "\n";

        // Write data
        for (size_t i = 0; i < sim_len; i++){
          file << time[i] << ",";  // Write time data first

          // Add MC run data
          for (size_t dim = 0; dim < mc_dim-1; dim++){
            file << mc_run_data[dim][run_it][i] << ",";
          }
          file << mc_run_data[mc_dim-1][run_it][i] << "\n";
        }
      }
    } else {
      // Throw error on failure to open file
      throw(std::runtime_error("Error opening file: " + filename.string()));
    }
  }
}

/**
 * @brief Generate the CSV file for the probability of detection plot
 *
 * @tparam DIM_S: Dimensions for MC and LC state vectors (BasicModelDim)
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param pdvg_solution: Data structure containing information about the PDVG solution path
 * @param obs_checker: The obstacle checker used to generate pdvg_solution
 * @param data_directory: Directory to write the file to
 */
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
void makePDPlotCSV(
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>&                pdvg_solution,
    const rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S, false, false, SCALAR, OPTIONS>& obs_checker,
    const std::string&                                                                                  data_directory){
  // Set up plotting info and grab data from the obstacle checker
  Eigen::Matrix<SCALAR, 1, Eigen::Dynamic, OPTIONS> probability_of_detection;
  Eigen::Matrix<SCALAR, 1, Eigen::Dynamic, OPTIONS> probability_of_detection_std;
  Eigen::Matrix<SCALAR, 1, Eigen::Dynamic, OPTIONS> radar_cross_section;
  Eigen::Matrix<SCALAR, 1, Eigen::Dynamic, OPTIONS> range;
  obs_checker->getPlotInfo(pdvg_solution,
                            probability_of_detection,
                            probability_of_detection_std,
                            radar_cross_section,
                            range);

  // Extract the time vector
  size_t sim_len = pdvg_solution.col(DIM_S::TIME_IND).rows();
  Eigen::Matrix<SCALAR, 1, Eigen::Dynamic, OPTIONS> time;
  time = pdvg_solution.col(DIM_S::TIME_IND);

  // Create CSV file
  std::ofstream file;
  std::filesystem::path filename =
    std::filesystem::path(data_directory) /
      (CSVNames::FileNames::PDVG + CSVNames::FileSuffix::DEFAULT_FILE_SUFFIX);
  std::vector<std::string> headers = CSVHeaders::PDVG;
  file.open(filename.string(), std::ios::out);

  // Write data to file if open
  if (file.is_open()){
    // Set output precision
    file << std::fixed << std::setprecision(CSV_PRECISION);

    // Write headers
    for (size_t h = 0; h < headers.size()-1; h++){
      file << headers[h] << ",";
    }
    file << headers[headers.size()-1] << "\n";

    // Write data
    for (size_t data_it = 0; data_it < sim_len; data_it++){
      file << time(data_it) << ",";
      file << probability_of_detection(data_it) << ",";
      file << probability_of_detection_std(data_it) << ",";
      file << range(data_it) << ",";
      file << radar_cross_section(data_it) << "\n";
    }
    file.close();
  } else {
    throw(std::runtime_error("Error opening file: " + filename.string()));
  }
}

/**
 * @brief Kickoff function for making PDVG error budget CSV
 *
 * @tparam DIM_S: Dimensions for MC and LC state vectors (BasicModelDim)
 * @tparam VERSION: Controls what type of simulation will be ran
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param reference_trajectory: The nominal reference trajectory for this set of simulations
 * @param nominal_initial_state: The starting_state for the simulation
 * @param tools: Holds all of the needed helper functions
 * @param noise_sources: Each source of noise in the simulation
 * @param data_extraction_func: Function that extracts the data to be plotted
 * @param data_directory: Directory to write the file to
 */
template<typename DIM_S, kf::Versions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
void makePDVGErrorBudgetCSVKickoff(
    const Eigen::Ref<const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::REF_DIM, OPTIONS>>&   reference_trajectory,
    const Eigen::Ref<const Eigen::Matrix<SCALAR, 1, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>>& nominal_initial_state,
    const kf::Tools<DIM_S, SCALAR, OPTIONS>&                                                 tools,
    const std::list<GeneralNoiseWrapper<SCALAR, OPTIONS>>&                                  noise_sources,
    const DATA_FUNC<DIM_S, SCALAR, OPTIONS>&                                                 data_extraction_func,
    const std::string&                                                                     data_directory){
  // Set up necessary data containers
  const Eigen::Index                                                            traj_length = reference_trajectory.rows();
  Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS> state_vector;
  std::vector<SCALAR>                                                           time_vec;
  Eigen::Matrix<SCALAR, 1, Eigen::Dynamic, OPTIONS>                             plotting_data;
  std::vector<SCALAR>                                                           plotting_data_vec;
  std::vector<std::vector<SCALAR>>                                              csv_data;
  std::vector<std::string>                                                      headers;

  /// All sources enabled
  std::for_each(noise_sources.cbegin(), noise_sources.cend(),
                [](const GeneralNoiseWrapper<SCALAR, OPTIONS>& it) -> void { helpers::setNoiseEnabled<SCALAR, OPTIONS>(it); });

  // Set up state vector
  state_vector.resize(traj_length, Eigen::NoChange);
  state_vector.template topRows<1>()                                     = nominal_initial_state;
  state_vector.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = reference_trajectory;

  // Run lincov
  kf::runLinCov<DIM_S, VERSION, SCALAR, OPTIONS>(state_vector, tools);

  // Extract plotting data
  plotting_data = data_extraction_func(state_vector);

  // Convert to std vector
  time_vec.resize(traj_length);
  plotting_data_vec.resize(traj_length);
  for(Eigen::Index row_it = 0; row_it < traj_length; ++row_it){
    time_vec[row_it]          = state_vector(row_it, DIM_S::TIME_IND);
    plotting_data_vec[row_it] = plotting_data[row_it];
  }

  // Store headers and data for the "Total" error data
  headers.emplace_back(CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::TOTAL]);
  csv_data.emplace_back(plotting_data_vec);

  // One source enabled at a time
  std::for_each(noise_sources.cbegin(), noise_sources.cend(),
                [](const GeneralNoiseWrapper<SCALAR, OPTIONS>& it) -> void { helpers::setNoiseDisabled<SCALAR, OPTIONS>(it); });
  // Contributions from initial covariance conditions
  state_vector.template topRows<1>().template middleCols<DIM_S::LINCOV::AUG_COV_LEN>(DIM_S::LINCOV::AUG_COV_START_IND).setZero();

  const auto noise_sources_end = noise_sources.cend();
  for(auto noise_source_it = noise_sources.cbegin(); noise_source_it != noise_sources_end; ++noise_source_it){
    helpers::setNoiseEnabled<SCALAR, OPTIONS>(*noise_source_it);
    // Run lincov
    kf::runLinCov<DIM_S, kf::Versions(VERSION | kf::Versions::RUNNING_ERROR_BUDGET), SCALAR, OPTIONS>(state_vector, tools);
    // Extract plotting data
    plotting_data = data_extraction_func(state_vector);
    // Convert to std
    for(Eigen::Index row_it = 0; row_it < traj_length; ++row_it){
      plotting_data_vec[row_it] = plotting_data[row_it];
    }

    // Get relevant info for current noise source and store in header and data vectors
    headers.emplace_back(helpers::getNoiseName<SCALAR, OPTIONS>(*noise_source_it));
    csv_data.emplace_back(plotting_data_vec);
  }

  // Contributions from initial covariance conditions
  state_vector.template topRows<1>().template middleCols<DIM_S::LINCOV::AUG_COV_LEN>(DIM_S::LINCOV::AUG_COV_START_IND) =
    nominal_initial_state.template middleCols<DIM_S::LINCOV::AUG_COV_LEN>(DIM_S::LINCOV::AUG_COV_START_IND);

  // Run lincov
  kf::runLinCov<DIM_S, kf::Versions(VERSION | kf::Versions::RUNNING_ERROR_BUDGET), SCALAR, OPTIONS>(state_vector, tools);

  // Extract plotting data
  plotting_data = data_extraction_func(state_vector);

  // Convert to std vector
  for(Eigen::Index row_it = 0; row_it < traj_length; ++row_it){
    plotting_data_vec[row_it] = plotting_data[row_it];
  }

  // Add final vector information to headers and CSV data vectors
  headers.emplace_back(CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::INIT_COVARIANCE]);
  csv_data.emplace_back(plotting_data_vec);

  // Save error budget data to file
  makePDVGErrorBudgetCSV(time_vec, headers, csv_data, data_directory);
}

/**
 * @brief Make error budget CSV file
 *
 * @tparam SCALAR: Data type utilized in plotting
 * @param time: The vector containing time-steps of the data, written to first column
 * @param headers: Headers to be used in the CSV file
 * @param data: Vectors of each part of the data to be written to file
 * @param data_directory: Directory to write the file to
 */
template <typename SCALAR>
void makePDVGErrorBudgetCSV(const std::vector<SCALAR>&      time,
                            const std::vector<std::string>& headers,
                            const Vector2D<SCALAR>&         data,
                            const std::string&              data_directory){
  // Create CSV file
  std::ofstream file;
  std::filesystem::path filename;
  filename = std::filesystem::path(data_directory) /
    (CSVNames::FileNames::ERROR_BUDGET + CSVNames::FileSuffix::DEFAULT_FILE_SUFFIX);
  file.open(filename.string(), std::ios::out);

  if (file.is_open()){
    file << std::fixed << std::setprecision(CSV_PRECISION);

    // Write headers
    file << CSVHeaders::ERROR_BUDGET[CSVHeaders::ERROR_BUDGET_IDXS::TIME] << ",";
    for (size_t i = 0; i < headers.size()-1; i++){
      file << headers[i] << ",";
    }
    file << headers[headers.size()-1] << "\n";

    // Write data
    auto data_size = data.size();
    for (size_t time_it = 0; time_it < time.size(); time_it++){
      file << time[time_it] << ",";
      for (size_t data_it = 0; data_it < data_size-1; data_it++){
        file << data[data_it][time_it] << ",";
      }
      file << data[data_size-1][time_it] << "\n";
    }
    file.close();
  } else {
    throw std::runtime_error("Error opening file: " + filename.string());
  }
}

/**
 * @brief Prepare data for writing to CSV, MC/LC combined overload
 *
 * @tparam DIM_S: BasicModelDim used to access all parts of state vectors
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param mc_state_vectors: Monte Carlo state vectors
 * @param lincov_state_vector: LinCov state vectors
 * @param mappings: A helper object that maps one state vector to another
 * @param truth_to_plot: Functor to map data to truth values
 * @param nav_to_plot: Functor to map data to nav values
 * @param names: List of name pair/tuples that indicate the desired data types and data starting indices
 * @param plot_types: Boolean array indicating what plot types were requested
 * @param data_directory: Directory to store the data in
 */
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
void makeAllStatisticsCSV(
    const std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::MC::FULL_STATE_LEN, OPTIONS>>& mc_state_vectors,
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>&          lincov_state_vector,
    const kf::map::MappingsBasePtr<DIM_S, SCALAR, OPTIONS>&                                       mappings,
    const std::function<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DISP_DIM, OPTIONS>(
      const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DIM, OPTIONS>&)>&                  truth_to_plot,
    const std::function<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::ERROR_DIM, OPTIONS>(
      const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::NAV_DIM, OPTIONS>&)>&                    nav_to_plot,
    const NameVector&                                                                             names,
    const std::array<bool, 6>&                                                                    plot_types,
    const std::string&                                                                            data_directory){
  // Get time vector
  const size_t sim_len = lincov_state_vector.rows();
  std::vector<SCALAR> time(sim_len);
  for(size_t time_it = 0; time_it < sim_len; ++time_it){
    time[time_it] = lincov_state_vector(time_it, DIM_S::TIME_IND);
  }

  // For each index that is true, call the appropriate function with that index
  if (plot_types[PlotTypeInds::TRUTH_DISP_PLOTS]){
    makeTruthNavErrorCSV<DIM_S, DIM_S::TRUTH_DISP_DIM, SCALAR, OPTIONS>(
      PlotTypeInds::TRUTH_DISP_PLOTS,
      data_directory,
      names,
      time,
      mappings,
      lincov_state_vector,
      mc_state_vectors);
  }

  if (plot_types[PlotTypeInds::TRUTH_DISP_OFF_REF_PLOTS]){
    makeTruthNavErrorCSV<DIM_S, DIM_S::TRUTH_DISP_DIM, SCALAR, OPTIONS>(
      PlotTypeInds::TRUTH_DISP_OFF_REF_PLOTS,
      data_directory,
      names,
      time,
      mappings,
      lincov_state_vector,
      mc_state_vectors);
  }

  if (plot_types[PlotTypeInds::NAV_DISP_PLOTS]){
    makeTruthNavErrorCSV<DIM_S, DIM_S::ERROR_DIM, SCALAR, OPTIONS>(
      PlotTypeInds::NAV_DISP_PLOTS,
      data_directory,
      names,
      time,
      mappings,
      lincov_state_vector,
      mc_state_vectors);
  }

  if (plot_types[PlotTypeInds::NAV_DISP_OFF_REF_PLOTS]){
    makeTruthNavErrorCSV<DIM_S, DIM_S::ERROR_DIM, SCALAR, OPTIONS>(
      PlotTypeInds::NAV_DISP_OFF_REF_PLOTS,
      data_directory,
      names,
      time,
      mappings,
      lincov_state_vector,
      mc_state_vectors);
  }

  if (plot_types[PlotTypeInds::EST_ERROR_PLOTS]){
    makeTruthNavErrorCSV<DIM_S, DIM_S::ERROR_DIM, SCALAR, OPTIONS>(
      PlotTypeInds::EST_ERROR_PLOTS,
      data_directory,
      names,
      time,
      mappings,
      lincov_state_vector,
      mc_state_vectors);
  }

  if (plot_types[PlotTypeInds::STATE_PLOTS]){
    makeStateCSVKickoff<DIM_S, SCALAR, OPTIONS>(
      data_directory,
      names,
      time,
      mappings,
      truth_to_plot,
      nav_to_plot,
      lincov_state_vector,
      mc_state_vectors);
  }
}

/**
 * @brief Prepare data for writing to CSV, LC-only version overload
 *
 * @tparam DIM_S: BasicModelDim used to access all parts of state vectors
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param lincov_state_vector: LinCov state vectors
 * @param mappings: A helper object that maps one state vector to another
 * @param names: List of name pair/tuples that indicate the desired data types and data starting indices
 * @param plot_types: Boolean array indicating what plot types were requested
 * @param data_directory: Directory to store the data in
 */
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
void makeAllStatisticsCSV(
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>& lincov_state_vector,
    const kf::map::MappingsBasePtr<DIM_S, SCALAR, OPTIONS>&                              mappings,
    const NameVector&                                                                    names,
    const std::array<bool, 4>&                                                           plot_types,
    const std::string&                                                                   data_directory){
  // Get time vector
  const size_t sim_len = lincov_state_vector.rows();
  std::vector<SCALAR> time(sim_len);
  for(size_t time_it = 0; time_it < sim_len; ++time_it){
    time[time_it] = lincov_state_vector(time_it, DIM_S::TIME_IND);
  }

  // For each index that is true, call the appropriate function with that index
  if (plot_types[PlotTypeInds::TRUTH_DISP_PLOTS]){
    makeTruthNavErrorCSV<DIM_S, DIM_S::TRUTH_DISP_DIM, SCALAR, OPTIONS>(
      PlotTypeInds::TRUTH_DISP_PLOTS,
      data_directory,
      names,
      time,
      mappings,
      lincov_state_vector);
  }

  if (plot_types[PlotTypeInds::NAV_DISP_PLOTS]){
    makeTruthNavErrorCSV<DIM_S, DIM_S::ERROR_DIM, SCALAR, OPTIONS>(
      PlotTypeInds::NAV_DISP_PLOTS,
      data_directory,
      names,
      time,
      mappings,
      lincov_state_vector);
  }

  if (plot_types[PlotTypeInds::EST_ERROR_PLOTS]){
    makeTruthNavErrorCSV<DIM_S, DIM_S::ERROR_DIM, SCALAR, OPTIONS>(
      PlotTypeInds::EST_ERROR_PLOTS,
      data_directory,
      names,
      time,
      mappings,
      lincov_state_vector);
  }

  if (plot_types[PlotTypeInds::STATE_PLOTS]){
    makeStateCSVKickoff<DIM_S, SCALAR, OPTIONS>(
      data_directory,
      names,
      time,
      mappings,
      nullptr,  // No truth or nav with LinCov only, don't provide functors
      nullptr,
      lincov_state_vector);
  }
}

/**
 * @brief Prepare data for making truth, nav, or error-related CSV files
 *
 * @tparam DIM_S: Dimensions for MC and LC state vectors (BasicModelDim)
 * @tparam SUB_DIM: Sub-dimension value used to generalize across different plot types
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param plot_type_index: Index of plot type for choosing how to create and manipulate data
 * @param data_dir: Directory to store the data in
 * @param names: List of names that indicate the desired data types
 * @param time: Vector for simulation time values
 * @param mappings: A helper object that maps one state vector to another
 * @param mc_state_vectors: Monte Carlo state vectors
 * @param lincov_state_vector: LinCov state vectors
 */
template<typename DIM_S, Eigen::Index SUB_DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void makeTruthNavErrorCSV(
    const Eigen::Index                                                                            plot_type_index,
    const std::string&                                                                            data_dir,
    const NameVector&                                                                             names,
    const std::vector<SCALAR>&                                                                    time,
    const kf::map::MappingsBasePtr<DIM_S, SCALAR, OPTIONS>&                                       mappings,
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>&          lincov_state_vector,
    const std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::MC::FULL_STATE_LEN, OPTIONS>>& mc_state_vectors){
  using MCRuns = std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, SUB_DIM, OPTIONS>>;
  using CovData = std::vector<Eigen::Matrix<SCALAR, SUB_DIM, SUB_DIM, OPTIONS>>;

  // Make flags to account for possible function calls
  bool mc_only = (plot_type_index == PlotTypeInds::NAV_DISP_OFF_REF_PLOTS) ||
                 (plot_type_index == PlotTypeInds::TRUTH_DISP_OFF_REF_PLOTS);
  bool lc_only = mc_state_vectors.empty();

  // Set up possible vectors
  MCRuns mc_runs_full = {};
  CovData mc_full = {};
  CovData lc_full;
  const size_t sim_len = lincov_state_vector.rows();
  // Enclose scope to save memory on avg state vectors
  {
    // One or the other of these will be used
    // avg/ref_truth/nav_state variables are used to extract Monte Carlo runs
    std::unique_ptr<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DIM, OPTIONS>> avg_truth_state;
    std::unique_ptr<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::NAV_DIM, OPTIONS>>   avg_nav_state;
    std::unique_ptr<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DIM, OPTIONS>> ref_truth_state;
    std::unique_ptr<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::NAV_DIM, OPTIONS>>   ref_nav_state;

    // Get different data depending on the plot type
    switch(plot_type_index){
      case PlotTypeInds::TRUTH_DISP_PLOTS:
        if (!lc_only){
          avg_truth_state =
            std::make_unique<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DIM, OPTIONS>>(
              kf::math::approxMeanTruthStateTrajectory<DIM_S, SCALAR, OPTIONS>(mc_state_vectors, mappings));
          mc_runs_full = kf::math::findTruthStateDispersion<DIM_S, SCALAR, OPTIONS>(mc_state_vectors,
                                                                                    *avg_truth_state,
                                                                                    mappings);
        }
        lc_full = kf::math::truthStateDispersionCovariance<DIM_S, SCALAR, OPTIONS>(lincov_state_vector);
        break;

      case PlotTypeInds::TRUTH_DISP_OFF_REF_PLOTS:
        if (!lc_only){
          ref_truth_state =
            std::make_unique<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DIM, OPTIONS>>();
          (*ref_truth_state).resize(sim_len, Eigen::NoChange);

          for(Eigen::Index time_it = 0; time_it < Eigen::Index(sim_len); ++time_it){
            (*ref_truth_state).row(time_it) =
              mappings->mapRefTruth(
                lincov_state_vector.template block<1, DIM_S::REF_DIM>(time_it, DIM_S::REF_START_IND));
          }

          mc_runs_full = kf::math::findTruthStateDispersion<DIM_S, SCALAR, OPTIONS>(mc_state_vectors,
                                                                                    *ref_truth_state,
                                                                                    mappings);
        }
        break;

      case PlotTypeInds::NAV_DISP_PLOTS:
        if (!lc_only){
          avg_nav_state =
            std::make_unique<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::NAV_DIM, OPTIONS>>(
              kf::math::approxMeanNavStateTrajectory<DIM_S, SCALAR, OPTIONS>(mc_state_vectors, mappings));
          mc_runs_full = kf::math::findNavStateDispersion<DIM_S, SCALAR, OPTIONS>(mc_state_vectors,
                                                                                  *avg_nav_state,
                                                                                  mappings);
        }
        lc_full = kf::math::navStateDispersionCovariance<DIM_S, SCALAR, OPTIONS>(lincov_state_vector);
        break;

      case PlotTypeInds::NAV_DISP_OFF_REF_PLOTS:
        if (!lc_only){
          ref_nav_state =
            std::make_unique<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::NAV_DIM, OPTIONS>>();
          (*ref_nav_state).resize(sim_len, Eigen::NoChange);

          for(Eigen::Index time_it = 0; time_it < Eigen::Index(sim_len); ++time_it){
            (*ref_nav_state).row(time_it) =
              mappings->mapRefTruth(
                lincov_state_vector.template block<1, DIM_S::REF_DIM>(time_it, DIM_S::REF_START_IND));
          }
          mc_runs_full = kf::math::findNavStateDispersion<DIM_S, SCALAR, OPTIONS>(mc_state_vectors,
                                                                                  *ref_nav_state,
                                                                                  mappings);
        }
        break;

      case PlotTypeInds::EST_ERROR_PLOTS:
        if (!lc_only){
          mc_runs_full = kf::math::findErrorStates<DIM_S, SCALAR, OPTIONS>(mc_state_vectors, mappings);
        }
        lc_full = kf::math::errorStateCovariance<DIM_S, SCALAR, OPTIONS>(lincov_state_vector, mappings);
        break;
    }
  }

  // Extract vector for determining Monte Carlo 3-sigma bounds
  if (!lc_only){
    mc_full = kf::math::approxStateDispersionCovariance<SUB_DIM, SCALAR, OPTIONS>(mc_runs_full);
  }

  // Make some changes to vectors and write data out
  const size_t names_len = names.size();
  const size_t num_sims  = mc_runs_full.size();
  for(size_t name_it = 0; name_it < names_len; ++name_it){
    const Eigen::Index num_states =     std::get<0>(names[name_it].second);
    const std::string  data_type_name = names[name_it].first;
    Eigen::Index       start_ind;

    // Get the data's starting index from the 'names' data structure
    switch(plot_type_index){
      case PlotTypeInds::EST_ERROR_PLOTS:
        // Continue
      case PlotTypeInds::NAV_DISP_PLOTS:
        // Continue
      case PlotTypeInds::NAV_DISP_OFF_REF_PLOTS:
        start_ind = std::get<3>(names[name_it].second);
        break;
      case PlotTypeInds::TRUTH_DISP_PLOTS:
        // Continue
      case PlotTypeInds::TRUTH_DISP_OFF_REF_PLOTS:
        start_ind = std::get<2>(names[name_it].second);
        break;
      default:
        start_ind = std::get<3>(names[name_it].second);
        break;
    }

    // Create vectors to store data
    std::unique_ptr<Vector2D<SCALAR>> lc_bounds_data;
    std::unique_ptr<Vector2D<SCALAR>> mc_bounds_data;
    std::unique_ptr<Vector3D<SCALAR>> mc_run_data;

    // Convert data to std::vectors for CSV writing functions
    if (!lc_only && !mc_only){
      // Extract LinCov and Monte Carlo data
      mc_run_data =
        std::make_unique<Vector3D<SCALAR>>(
          num_states,
          Vector2D<SCALAR>(num_sims, std::vector<SCALAR>(sim_len)));

      mc_bounds_data = std::make_unique<Vector2D<SCALAR>>(num_states, std::vector<SCALAR>(sim_len));
      lc_bounds_data = std::make_unique<Vector2D<SCALAR>>(num_states, std::vector<SCALAR>(sim_len));

      for(size_t time_it = 0; time_it < sim_len; ++time_it){
        for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it){
          // Store all MC runs
          for(size_t sim_it = 0; sim_it < num_sims; ++sim_it){
            (*mc_run_data)[dim_it][sim_it][time_it] =
              mc_runs_full[sim_it](time_it, start_ind + dim_it);
          }

          // Make 3-sigma calculations
          (*mc_bounds_data)[dim_it][time_it] =
            SCALAR(3) * std::sqrt(mc_full[time_it](start_ind + dim_it, start_ind + dim_it));
          (*lc_bounds_data)[dim_it][time_it] =
            SCALAR(3) * std::sqrt(lc_full[time_it](start_ind + dim_it, start_ind + dim_it));
        }
      }
      // Make files with uncertainty data and additional files with Monte Carlo runs
      makeThreeSigmaCSV(plot_type_index, sim_len, time,
                        data_dir, data_type_name,
                        *lc_bounds_data, *mc_bounds_data);
      makeMonteCarloRunsCSV(plot_type_index, sim_len, time,
                          data_dir, data_type_name, *mc_run_data);
    } else if (mc_only){
      // Extract Monte Carlo data only
      mc_run_data =
        std::make_unique<Vector3D<SCALAR>>(
          num_states,
          Vector2D<SCALAR>(num_sims, std::vector<SCALAR>(sim_len)));

      mc_bounds_data = std::make_unique<Vector2D<SCALAR>>(num_states, std::vector<SCALAR>(sim_len));

      for(size_t time_it = 0; time_it < sim_len; ++time_it){
        for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it){
          // Store all MC runs
          for(size_t sim_it = 0; sim_it < num_sims; ++sim_it){
            (*mc_run_data)[dim_it][sim_it][time_it] =
              mc_runs_full[sim_it](time_it, start_ind + dim_it);
          }

          // Make 3-sigma calculations
          (*mc_bounds_data)[dim_it][time_it] =
            SCALAR(3) * std::sqrt(mc_full[time_it](start_ind + dim_it, start_ind + dim_it));
        }
      }
      // Make files with uncertainty data and additional files with Monte Carlo runs
      makeThreeSigmaCSV(plot_type_index, sim_len, time,
                        data_dir, data_type_name,
                        {}, *mc_bounds_data);
      makeMonteCarloRunsCSV(plot_type_index, sim_len, time,
                            data_dir, data_type_name, *mc_run_data);
    } else {
      // Extract LinCov data only
      lc_bounds_data = std::make_unique<Vector2D<SCALAR>>(num_states, std::vector<SCALAR>(sim_len));

      for(size_t time_it = 0; time_it < sim_len; ++time_it){
        for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it){
          // Make 3-sigma calculations
          (*lc_bounds_data)[dim_it][time_it] =
            SCALAR(3) * std::sqrt(lc_full[time_it](start_ind + dim_it, start_ind + dim_it));
        }
      }
      // Make files with LinCov-only uncertainty data
      makeThreeSigmaCSV(plot_type_index, sim_len, time,
                        data_dir, data_type_name,
                        *lc_bounds_data, {});
    }
  }
}

/**
 * @brief Prepare data for making truth, nav, or error-related CSV files
 *
 * @tparam DIM_S: Dimensions for MC and LC state vectors (BasicModelDim)
 * @tparam SUB_DIM: Sub-dimension value used to generalize across different plot types
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param data_dir: Directory to store the data in
 * @param names: List of names that indicate the desired data types
 * @param time: Vector for simulation time values
 * @param mappings: A helper object that maps one state vector to another
 * @param truth_to_plot: Functor to map data to truth values
 * @param nav_to_plot: Functor to map data to nav values
 * @param lincov_state_vector: LinCov state vectors
 * @param mc_state_vectors: Monte Carlo state vectors
 */
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
void makeStateCSVKickoff(
    const std::string&                                                                             data_dir,
    const NameVector&                                                                              names,
    const std::vector<SCALAR>&                                                                     time,
    const kf::map::MappingsBasePtr<DIM_S, SCALAR, OPTIONS>&                                        mappings,
    const std::function<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DISP_DIM, OPTIONS>(
      const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DIM, OPTIONS>&)>&                   truth_to_plot,
    const std::function<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::ERROR_DIM, OPTIONS>(
      const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::NAV_DIM, OPTIONS>&)>&                     nav_to_plot,
    const Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::LINCOV::FULL_STATE_LEN, OPTIONS>&           lincov_state_vector,
    const std::vector<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::MC::FULL_STATE_LEN, OPTIONS>>&  mc_state_vectors){
  // Add flag for checking if only LinCov provided
  const bool lc_only = mc_state_vectors.empty();
  // Create avg_truth and avg_nav state plot matrices
  std::unique_ptr<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DISP_DIM, OPTIONS>> truth_plot;
  std::unique_ptr<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::ERROR_DIM, OPTIONS>>      nav_plot;

  // If Monte Carlo data is used, extract truth and nav data
  if (!lc_only){
    truth_plot =
      std::make_unique<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DISP_DIM, OPTIONS>>();
    nav_plot =
      std::make_unique<Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DISP_DIM, OPTIONS>>();
    Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::TRUTH_DIM, OPTIONS> avg_truth_state;
    Eigen::Matrix<SCALAR, Eigen::Dynamic, DIM_S::NAV_DIM, OPTIONS>   avg_nav_state;
    avg_truth_state =
      kf::math::approxMeanTruthStateTrajectory<DIM_S, SCALAR, OPTIONS>(mc_state_vectors, mappings);
    avg_nav_state =
      kf::math::approxMeanNavStateTrajectory<DIM_S, SCALAR, OPTIONS>(mc_state_vectors, mappings);
    *truth_plot = truth_to_plot(avg_truth_state);
    *nav_plot = nav_to_plot(avg_nav_state);
  }

  const size_t names_len = names.size();
  const size_t sim_len = lincov_state_vector.rows();
  for(size_t name_it = 0; name_it < names_len; ++name_it){
    // Get information for data extraction from 'names' vector
    const Eigen::Index num_states      = std::get<0>(names[name_it].second);
    const Eigen::Index ref_start_ind   = std::get<1>(names[name_it].second);
    const Eigen::Index truth_start_ind = std::get<2>(names[name_it].second);
    const Eigen::Index nav_start_ind   = std::get<3>(names[name_it].second);
    const bool         has_ref         = -1 != ref_start_ind;
    bool               has_truth       = -1 != truth_start_ind;
    bool               has_nav         = -1 != nav_start_ind;

    std::unique_ptr<Vector2D<SCALAR>> sub_ref;
    std::unique_ptr<Vector2D<SCALAR>> sub_truth_avg;
    std::unique_ptr<Vector2D<SCALAR>> sub_nav_avg;

    // Truth and nav data is not included with LinCov-only plots, so set flags accordingly
    if (lc_only){
      has_truth = false;
      has_nav = false;
    }

    // Extract data corresponding to the flags set above
    if (has_ref){
      sub_ref =
        std::make_unique<Vector2D<SCALAR>>(num_states, std::vector<SCALAR>(sim_len));
    }
    if (has_truth){
      sub_truth_avg =
        std::make_unique<Vector2D<SCALAR>>(num_states, std::vector<SCALAR>(sim_len));
    }
    if (has_nav){
      sub_nav_avg =
        std::make_unique<Vector2D<SCALAR>>(num_states, std::vector<SCALAR>(sim_len));
    }

    // Convert data to std::vectors for CSV writing functions
    for(size_t time_it = 0; time_it < sim_len; ++time_it){
      for(Eigen::Index dim_it = 0; dim_it < num_states; ++dim_it){
        // Make reference vector
        if(has_ref){
          (*sub_ref)[dim_it][time_it] =
            lincov_state_vector(time_it, DIM_S::REF_START_IND + ref_start_ind + dim_it);
        }

        // Make truth vector
        if(has_truth){
          (*sub_truth_avg)[dim_it][time_it] = (*truth_plot)(time_it, truth_start_ind + dim_it);
        }

        // Make navigation vector
        if(has_nav){
          (*sub_nav_avg)[dim_it][time_it] = (*nav_plot)(time_it, nav_start_ind + dim_it);
        }
      }
    }

    // Make CSVs
    if (has_ref){
      makeStateCSV(sim_len, time, data_dir, names[name_it].first,
                    CSVHeaders::STATE_REF_FLAG, *sub_ref);
    }
    if (has_truth){
      makeStateCSV(sim_len, time, data_dir, names[name_it].first,
                    CSVHeaders::STATE_TRUTH_FLAG, *sub_truth_avg);
    }
    if (has_nav){
      makeStateCSV(sim_len, time, data_dir, names[name_it].first,
                    CSVHeaders::STATE_NAV_FLAG, *sub_nav_avg);
    }
  }
}

/**
 * @brief Enable noise in the noise object depending on its type
 *
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param noise: Noise object
 */
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
void helpers::setNoiseEnabled(const GeneralNoiseWrapper<SCALAR, OPTIONS>& noise) noexcept{
  switch(noise.index()){
    case 0:
      std::get<0>(noise)->setEnabled();
      break;
    case 1:
      std::get<1>(noise)->setEnabled();
      break;
    case std::variant_npos:
      assert(false);
      break;
    default:
      assert(false);
      break;
  };
}

/**
 * @brief Disable noise in the noise object depending on its type
 *
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param noise: Noise object
 */
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
void helpers::setNoiseDisabled(const GeneralNoiseWrapper<SCALAR, OPTIONS>& noise) noexcept{
  switch(noise.index()){
    case 0:
      std::get<0>(noise)->setDisabled();
      break;
    case 1:
      std::get<1>(noise)->setDisabled();
      break;
    case std::variant_npos:
      assert(false);
      break;
    default:
      assert(false);
      break;
  };
}

/**
 * @brief Get the noise object name
 *
 * @tparam SCALAR: Data type utilized in plotting
 * @tparam OPTIONS: Options for Eigen Matrix behavior
 * @param noise: Noise object
 */
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
const std::string& helpers::getNoiseName(const GeneralNoiseWrapper<SCALAR, OPTIONS>& noise) noexcept{
  switch(noise.index()){
    case 0:
      return std::get<0>(noise)->getName();
      break;
    case 1:
      return std::get<1>(noise)->getName();
      break;
    case std::variant_npos:
      assert(false);
      break;
    default:
      assert(false);
      break;
  };
  assert(false);
  return std::get<0>(noise)->getName();
}
}  // namespace csv
}  // namespace planner_interface
#endif  // PLANNER_INTERFACE__MAKE_STATISTICS_CSV_HPP_
