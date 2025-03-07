"""
MCPlotterMenu handles menu inputs, navigation, and selection
management for the MCPlotter class. After menuing operations
are finished, this class provides information about the plots
the user has selected, at which point the MCPlotter takes over
and produces the selected plots from data in a particular location
"""

import os
import re

class MCPlotterMenu:
  # Define public constants
  STATE_AUTO_MODE = 0
  STATE_MANUAL_MODE = 1
  STATE_STARTUP = 2
  STATE_PLOTTING = 3

  def __init__(self, plot_dict: dict) -> None:
    """
    Form variables for generating menu via input from MCPlotter
    """
    # Extract dictionary information
    self._k_name_list   = plot_dict['_k_name_list']
    self._group_dict    = plot_dict['_group_dict']
    self._header_list   = plot_dict['_header_list']
    self._g_name_len    = plot_dict['_g_name_len']
    self._header_len    = plot_dict['_header_len']

    # Define arrays to store selection data
    self._g_vec:    list[int] = []
    self._solo_vec: list[int] = []

    # Create states for runtime operation
    self.state = MCPlotterMenu.STATE_STARTUP
    self._return_state = MCPlotterMenu.STATE_AUTO_MODE
    self._success = False

    # Colored text
    self.WARNING_COLOR = '\033[93m'
    self.ENDC = '\033[0m'
    self.GREEN = '\033[92m'
    self.CYAN = '\033[96m'


  # === "PRIVATE" FUNCTIONS ===
  def _clear_screen(self) -> None:
    """Clear screen for better readability"""
    os.system('clear')

  def _get_input(self, message_func, error_msg: str) -> str:
    """
    Get input from user after displaying short info message for current menu

    Inputs:
    - message_func: A function alias used to print out appropriate message for the
                    current menu state
    - error_msg: Error message to print if user gave invalid input

    Outputs:
    - input(): Returns the user's input
    """
    # Clear the screen if the last message wasn't a "plotting complete" message
    if not self._success:
      self._clear_screen()
    else:
      self._success = False

    # Print provided context menu for user
    message_func()

    # Print error if invalid input was detected
    if error_msg is not None:
      print(f"{self.WARNING_COLOR}Error: {error_msg}{self.ENDC}")
    print(">>> ", end='')
    return input()

  def _print_startup_message(self) -> None:
    """Print context menu for the startup menu state"""
    print("=== MODE SELECTION ===")
    print("Select the mode you would like to use (required input shown as [<option>]):")
    print("[a] Automatic: Generate a PDF of all the data in the selected folder")
    print("[m] Manual   : Select what to plot from a list of available data\n")
    print("[e] End program\n")

  def _get_startup_input(self) -> None:
    """
    Get input from user regarding which plotting mode to use
    """
    # Get input until valid and change state correspondingly
    good_input = False
    error_msg = None
    while not good_input:
      response = self._get_input(self._print_startup_message, error_msg)

      # Make sure user chose a valid option. If so, change menu state to selected option
      if response == 'a':
        self.state = MCPlotterMenu.STATE_AUTO_MODE
        good_input = True
      elif response == 'm':
        self.state = MCPlotterMenu.STATE_MANUAL_MODE
        good_input = True
      elif response == 'e':
        print("Quitting...")
        exit()
      else:
        error_msg = f"{self.WARNING_COLOR}Invalid input. Please choose between [a], [m], or [e]{self.ENDC}"

  def _print_auto_mode_message(self) -> None:
    """Print context menu for confirming automatic mode"""
    print("=== AUTO MODE CONFIRMATION ===")
    print("Are you sure you want to continue?")
    print("Depending on the data, this could take a significant amount of time.")
    print("[y] Yes, continue")
    print("[b] Return to mode selection\n")
    print("[e] End program\n")

  def _get_auto_mode_input(self) -> None:
    """
    Get input from user about being sure to continue with auto mode
    """

    # Get input until valid and change state correspondingly
    error_msg = None
    good_input = False
    while not good_input:
      response = self._get_input(self._print_auto_mode_message, error_msg)

      # Validate user input
      if response == 'y':
        # Move forward with generating all plots
        self.state = MCPlotterMenu.STATE_PLOTTING
        self._return_state = MCPlotterMenu.STATE_AUTO_MODE

        # Fill plotting vectors
        self._g_vec = range(0, self._g_name_len-1)
        self._solo_vec = None

        good_input = True
      elif response == 'b':
        # Back up to start menu
        self.state = MCPlotterMenu.STATE_STARTUP
        good_input = True
      elif response == 'e':
        # Quit
        print("Quitting...")
        exit()
      else:
        error_msg = f"{self.WARNING_COLOR}Invalid input. Please choose between [y], [b], or [e]{self.ENDC}"

  def _print_manual_mode_message(self) -> None:
    """Print manual mode message with all available options, delineated by header groups"""
    # Print group header and options
    print("=== SELECTION OPTIONS ===")
    header_count = 0
    for name in range(0, self._g_name_len):
      print(f"[g{name}]", end='')

      # Print individual header and options
      hlen = len(self._group_dict[self._k_name_list][name])
      for header in range(0, hlen):
        print(f"\t[{header_count}] {self._header_list[header_count]}")
        header_count += 1

    print("\nChoose what to plot using the values shown in brackets on the left of each option.")
    print("- Separate inputs by commas (>>> g1, g2, 1, 4, 7)")
    print("- Select range by using hyphens (>>> g1-g4, 1-4)\n")
    print("[b] Return to mode selection")
    print("[e] End program\n")

  def _get_manual_mode_input(self) -> None:
    """Print menu for manual mode"""

    good_input = False
    error_msg = None
    while not good_input:
      response = self._get_input(self._print_manual_mode_message, error_msg)
      bad_input = False

      # Validate user input
      if response == 'e':
        # End program
        print("Quitting...")
        exit()

      elif response == 'b':
        # Return to mode selection
        good_input = True
        self.state = MCPlotterMenu.STATE_STARTUP

      # If the user input matches any valid response for selection
      elif re.match(r"^[0-9g,\- ]*$", response):
        # Remove any whitespace and split by commas
        sresponse = response.replace(' ', '')
        sresponse = sresponse.split(',')

        # Parse each selection given by user
        for option in sresponse:
          # If a range selection was made
          if '-' in option:
            opt = option.split('-')

            # Should expect there are only 2 numbers
            # If there is more than one hyphen or less than 2 parts, not good input
            if len(opt) != 2 or '' in opt:
              bad_input = True
              error_msg = f"Invalid range selection ({option})"
              break

            # If group selection
            contains_g = [True if 'g' in o else False for o in opt]
            if True in contains_g:
              # Handle range of groups of headers
              num_list = []

              # Iterate over expected two numbers
              for selection in range(0, len(opt)):
                # Check to make sure numbers are valid
                opt[selection] = opt[selection].replace('g', '')
                (error_msg, bad_input, vector) = \
                  self._add_selection(option=option, opt=opt[selection], vector=self._g_vec, max_len=self._g_name_len-1, type='g')
                num_list = num_list + vector

                if bad_input:
                  break

              # If input is good, add range of groups to selected group list
              if not bad_input:
                num_list = sorted(num_list)
                for n in range(num_list[0], num_list[1]+1):
                  if n not in self._g_vec:
                    self._g_vec.append(n)

            else:
              # Handle range of individual headers
              num_list = []

              # Iterate over expected two numbers
              for selection in range(0, len(opt)):
                # Check to make sure numbers are valid
                (error_msg, bad_input, vector) = \
                  self._add_selection(option=option, opt=opt[selection], vector=self._solo_vec, max_len=self._header_len-1, type='h')
                num_list = num_list + vector

                if bad_input:
                  break

              # If input is good, add range of headers to selected headers list
              if not bad_input:
                num_list = sorted(num_list)
                for n in range(num_list[0], num_list[1]+1):
                  if n not in self._solo_vec:
                    self._solo_vec.append(n)

          else:
            # Handle individual selection
            if 'g' in option:
              # Handle individual group selection
              gcount = sum([True for c in option if c == 'g'])
              if gcount > 1:
                bad_input = True
                error_msg = f"Invalid selection ({option}). More than 2 \'g\' characters."
                break

              opt = option.replace('g', '')

              # Check to make sure numbers are valid
              (error_msg, bad_input, vector) = \
                self._add_selection(option=option, opt=opt, vector=self._g_vec, max_len=self._g_name_len-1, type='g')
              self._g_vec = self._g_vec + vector

            else:
              # Handle individual header selection
              opt = option

              # Check to make sure numbers are valid
              (error_msg, bad_input, vector) = \
                self._add_selection(option=option, opt=opt, vector=self._solo_vec, max_len=self._header_len-1, type='h')
              self._solo_vec = self._solo_vec + vector

        if bad_input == False:
          good_input = True
          self.state = MCPlotterMenu.STATE_PLOTTING
          self._return_state = MCPlotterMenu.STATE_MANUAL_MODE

      else:
        error_msg = "Invalid input"


  def _get_error(self, input: str, bad_value: str, type: str) -> str:
      """
      Print from a selection of errors depending on type provided

      Args:
      - input: Input from user deemed to be improper for an attempted selection
      - bad_value: Individual component of input considered an error
      - type: Character indicating what part of the selection went wrong
      """
      if type == 'n':
        # If it is a numeric issue
        return f"Failed to add option. ({bad_value}) is not numeric"
      elif type == 'h':
        # If it is a header selection issue
        return f"Invalid individual selection from input ({input}). Header number ({bad_value}) does not exist"
      elif type == 'g':
        # If it is a group selection issue
        return f"Invalid group selection from input ({input}). Group number ({bad_value}) does not exist"
      else:
        print("Invalid type option in _get_error")
        exit()

  def _add_selection(self, option: str, opt: str, vector: list, max_len: int, type: str) -> tuple([str, bool, list]):
    """
    Iterate over user input and add their selected graphs to a list

    Args:
    - option: The user's input for the current selection cycle
    - opt: A (potentially) shortened substring of 'option' used to isolate numbers if a group
      selection was made.
    - vector: List of numbers of selected plots to make
    - max_len: Max number of states in the files. Can't select a higher header num than the ones
      that exist in the files
    - type: Character indicating if the current selection being processed is for group selection
      ('g') or for individual header selection ('h')
    """
    add_vector = []   # List of column indices of data to plot
    error_msg = ''
    bad_input = False
    if opt.isnumeric():
      num = int(opt)
      if num > max_len or num < 0:
          error_msg = self._get_error(input=option, bad_value=opt, type=type)
          bad_input = True

      elif num not in vector:
        add_vector.append(num)
    else:
      error_msg = self._get_error(input=None, bad_value=opt, type='n')
      bad_input = True

    return (error_msg, bad_input, add_vector)

  def _reset(self):
    """Clear leftover selections from last plot cycle"""
    self._g_vec = []
    self._solo_vec = []


  # === PUBLIC FUNCTIONS ===
  def run(self) -> tuple([list, list, int]):
    """
    In a while loop, ask user for input and perform plotting operations

    Returns:
    - A tuple of two lists and an integer. The first list enumerates numbers of groups of headers
      that the user selected. The second list does the same for individual headers. The integer
      is a flag indicating what mode was selected by the user, either automatic or manual
    """
    self.state = MCPlotterMenu.STATE_STARTUP
    self._reset()

    while True:
      if self.state == MCPlotterMenu.STATE_STARTUP:
        self._get_startup_input()

      elif self.state == MCPlotterMenu.STATE_AUTO_MODE:
        self._get_auto_mode_input()

      elif self.state == MCPlotterMenu.STATE_MANUAL_MODE:
        self._get_manual_mode_input()

      elif self.state == MCPlotterMenu.STATE_PLOTTING:
        # Give control back to MCPlotter for the actual plotting
        return (self._g_vec, self._solo_vec, self._return_state)

      else:
        print("Something wonky happened...")
        exit()

  def status_update(self, current: int, total: int) -> None:
    """
    Display a progress bar

    Args:
    - current: Number of plots that have been generated so far
    - total: Total number of plots to be generated
    """
    self._clear_screen()
    print("Progress: [", end='')

    bar_size = 40

    # Print appropriate number of '#'
    char_per_complete = total / bar_size
    num_chars = int(current/char_per_complete)
    percentage = int((current / total) * 100)
    prog = '#'*num_chars
    print(f'{self.GREEN}{prog:<40}{self.ENDC}', end='')
    print(f'] {percentage:>3}%')


  def print_success(self, filepath: str = None) -> None:
    """
    Print a message on operation success

    Args:
    - filepath: A string providing the location of the saved PDF. This is displayed to the user
    """
    self._clear_screen()
    if filepath is not None:
      print(f"{self.GREEN}=== PDF CREATION SUCCESSFUL ===")
      print(f"File saved to: {self.CYAN} \'{filepath}\'{self.ENDC}\n")
    else:
      print(f"{self.GREEN}=== OPERATION SUCCESSFUL ==={self.ENDC}")
    self._success = True
