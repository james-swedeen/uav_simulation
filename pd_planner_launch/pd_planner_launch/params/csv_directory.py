"""
This module utilizes and defines constants used for creating a data directory
to store LinCov, Monte Carlo, and PDVG results
"""
from pathlib import Path
import pd_planner_launch.params.scenario_params as par
import re, shutil
from datetime import datetime
import planner_interface.csv_tools as csv_params

# Top-level save directory for storing CSV data
CSV_DATA_DIRECTORY:           str = str(Path.home() / 'test_data')
CSV_DATA_DIRECTORY_HEADLESS: str = str(Path.home() / 'test_data_headless')

class StateListener():
  def __init__(self):

    # 10,000 lines equates to just over 1MB of data with a precision of 6
    self.row_limit = 10000000  # How many lines of CSV can be written before a new file is made
    self.precision = 6         # Number of fixed sig-figs after decimal point in files

def create_csv_directory():
  """
  Using the provided data directory in the CSV_DATA_DIRECTORY_LC_MC variable,
  create the directory structure used by the lincov_interface node system
  for saving LinCov and Monte Carlo data
  """
  # Take the data directory variable and create top level directory
  top_level_dir = Path(CSV_DATA_DIRECTORY)

  # Clear all existing data from the directory and remake. Doing this eliminates data mismatch issues.
  if top_level_dir.exists():
    shutil.rmtree(str(top_level_dir))

  top_level_dir.mkdir(parents=True, exist_ok=True)

  for plot_type in csv_params.DIRECTORY_NAMES:
    # Make plot type directory
    plot_type_dir: Path = top_level_dir / plot_type
    plot_type_dir.mkdir(parents=True, exist_ok=True)

    for data_type in csv_params.FILE_NAMES:
      # Make data type directory
      data_type_dir: Path = plot_type_dir / data_type
      data_type_dir.mkdir(parents=True, exist_ok=True)

      # Make mc_runs sub-directory (except in state folder)
      if plot_type is not csv_params.DIRECTORY_NAMES[0]:
        data_type_dir /= csv_params.MC_RUNS_DIR_NAME
        data_type_dir.mkdir(parents=True, exist_ok=True)

def create_csv_directory_headless() -> str:
  """
  Creates a folder to store headless simulation run data in.
  Intended for utilization with the state_listener_csv_node in
  the planner_interface package
  """

  # Make top-level directory.
  dir = Path(CSV_DATA_DIRECTORY_HEADLESS)
  dir.mkdir(parents=True, exist_ok=True)

  # Get number of most recent run (if folders exist)
  max_num = 0
  found_valid = False
  for path in dir.glob('./*'):
    if path is not dir:  # As long as this isn't the parent directory
      run_number = int(path.name.split('_')[-1])
      max_num = max(max_num, run_number)
      found_valid = True

  # Create new folder with next run number
  if not found_valid:
    run_number_string = 'data_' + f'{0:05}'
  else:
    run_number_string = 'data_' + f'{(max_num + 1):05}'
  dir = dir / run_number_string
  dir.mkdir(parents=True, exist_ok=True)

  return str(dir)

def collect_test_info(dir: str) -> None:
  """
  Creates a small README with some testing information

  ==============================
  Formatted as:
  Tester: <name>
  Date: <date>
  Test Info:
    - Straight line: y/n
    - Curved line: y/n
    - GPS-Denied Areas: y/n
    - Position sensor: y/n
  Reason for test: <reason>
  ==============================

  Inputs:
  - dir: Data directory for current simulation (i.e. .../test_data_headless/data_024)
  """
  max_category_width = 15     # Greatest width of any category in example (not including colon)
  max_subcategory_width = 16  # Greatest width of any subcategory in example (not including colon)
  tab_width = 2               # Tab width for indentation
  line_width = 100            # Max allowable line width for testing reason

  format_max_cat = f"%-{max_category_width}s"
  format_max_subcat = f"%-{max_subcategory_width}s"
  tab = " "*tab_width

  # Input variables
  tester_name: str
  test_reason: str
  use_categories: list[str] = ['Straight Line', 'Curved Line', 'GPS-Denied Areas', 'Position sensor']

  # Other text variables
  date = datetime.now()
  use_cat_responses: list[str] = ['']*len(use_categories)

  # Get the type of test from the input parameters
  print("\033[93m=== PLEASE ENTER DATA INFO ===")
  print("Press [Enter] to skip README creation\033[0m")
  print(f"Enter the tester's name:\n>>> ", end='')
  tester_name = input()
  if tester_name == '':
      return

  # Iterate over all test info categories desired and collect input
  for item in range(0, len(use_categories)):
    print(f'Is this test using {use_categories[item]}? (y/n)\n>>> ', end='')
    response = input()

    # Verify input is valid
    while not re.match(r"\b(y|n|Y|N)\b", response):
      print(f"ERROR: Invalid input ({response})")
      print(f'Is this test using {use_categories[item]}? (y/n)\n>>> ', end='')
      response = input()
    use_cat_responses[item] = response.lower()

  print(f"Enter the reason for testing:\n>>> ", end='')
  test_reason = input()

  # Write all to file now
  with open(dir + "/README.txt", 'w') as file:
    file.write(format_max_cat % "Date" + ": ")
    file.write(str(date) + "\n")
    file.write(format_max_cat % "Tester" + ": ")
    file.write(tester_name + "\n\n")

    # Write test type info
    file.write(format_max_cat % "Test Info: ")
    file.write("\n")
    for item in range(0, len(use_categories)):
      file.write(f"{tab}- {format_max_subcat % use_categories[item]}: {use_cat_responses[item]}\n")

    # Write reason for testing
    file.write("\n" + format_max_cat % "Reason for testing: ")
    file.write("\n")

    # Split up reason for testing to prevent lines longer than line_width
    n_lines = len(test_reason) // line_width
    index = 0
    prev_index = 0
    test_reason_split = test_reason.split()
    for i in range(0, n_lines):
      total = 0
      while total < line_width:
        total += len(test_reason_split[index]) + 1  # Add one to include spaces
        index += 1

      # Write all to file for previous index that didn't violate length condition
      file.write(' '.join(test_reason_split[prev_index:index]) + '\n')
      prev_index = index

    file.write(' '.join(test_reason_split[index:]) + '\n')
    file.close()
