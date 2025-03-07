"""ch3_derivatives_tests.py: Implements some basic tests for the dirivatives funciton."""

import os
import pickle
from typing import cast

import numpy as np
from mav_sim.chap3.mav_dynamics import StateIndices, derivatives
from mav_sim.chap3.mav_dynamics_euler import derivatives_euler
from mav_sim.tools import types

IND = StateIndices()

class DynamicsResults:
    """Stores the results for the dynamics"""
    def __init__(self) -> None:
        self.state: types.NP_MAT
        self.forces_moments: types.ForceMoment
        self.xdot: types.NP_MAT

def generate_euler_tests(num_test: int = 100) -> list[DynamicsResults]:
    """Generates tests for the euler dynamics
    """
    # Generate the results list
    results: list[DynamicsResults] = []

    # Generate the results for each test
    for _ in range(num_test):
        # Generate the inputs
        result = DynamicsResults()
        result.state = np.random.randn(12,1)*100.
        result.forces_moments = cast(types.ForceMoment, np.random.randn(6,1)*10.)

        # Generate the output
        result.xdot = derivatives_euler(state=result.state, forces_moments=result.forces_moments)
        results.append(result)

    return results

def generate_quat_tests(num_test: int = 100) -> list[DynamicsResults]:
    """Generates tests for the quaternion dynamics
    """
    # Generate the results list
    results: list[DynamicsResults] = []

    # Generate the results for each test
    for _ in range(num_test):
        # Generate the inputs
        result = DynamicsResults()
        result.state = np.random.randn(13,1)*100.
        result.forces_moments = cast(types.ForceMoment, np.random.randn(6,1)*10.)

        # Ensure that the quaternion is a unit quaternion
        result.state[IND.QUAT] = result.state[IND.QUAT]/np.linalg.norm(result.state[IND.QUAT])

        # Generate the output
        result.xdot = derivatives(state=result.state, forces_moments=result.forces_moments)
        results.append(result)

    return results

def save_tests(tests_euler: list[DynamicsResults], tests_quat: list[DynamicsResults]) -> None:
    """Save off the tests inside a pickle file
    """
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch03_test_archive.pkl"
        ),
        "wb",
    ) as file:
        data = {}
        data["euler"] = tests_euler
        data["quat"] = tests_quat
        pickle.dump(data, file)

def generate_tests() -> None:
    """ Generates and saves the tests
    """
    # Generate tests
    tests_euler = generate_euler_tests()
    tests_quat = generate_quat_tests()

    # Save tests
    save_tests(tests_euler=tests_euler, tests_quat=tests_quat)

def euler_derivatives_test(tests: list[DynamicsResults], threshold: float = 1e-4) -> bool:
    """Test the Euler derivatives, display invalid results
    """
    # Evaluate the results
    print("\nStarting derivatives_euler test")
    success = True
    for test in tests:
        # Generate the output
        xdot = derivatives_euler(state=test.state, forces_moments=test.forces_moments)

        # Compare the output with the result
        diff = np.linalg.norm(xdot - test.xdot)

        if diff > threshold:
            print("\n\nFailed test!")
            print("state = \n", test.state, ", forces/moments = \n", test.forces_moments, ", expected: \n", test.xdot, ", received: \n", xdot)
            success = False

    # Indicate success
    if success:
        print("Passed test on derivatives_euler")
    return success

def quat_derivatives_test(tests: list[DynamicsResults], threshold: float = 1e-4) -> bool:
    """Test the Euler derivatives, display invalid results
    """
    # Evaluate the results
    print("\nStarting derivatives test")
    success = True
    for test in tests:
        # Generate the output
        xdot = derivatives(state=test.state, forces_moments=test.forces_moments)

        # Compare the output with the result
        diff = np.linalg.norm(xdot - test.xdot)

        if diff > threshold:
            print("\n\nFailed test!")
            print("state = \n", test.state, ", forces/moments = \n", test.forces_moments, ", expected: \n", test.xdot, ", received: \n", xdot)
            success = False

    # Indicate success
    if success:
        print("Passed test on derivatives")
    return success

def run_tests()->None:
    """Runs all of the tests
    """
    # Load the tests
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch03_test_archive.pkl"
        ),
        "rb",
    ) as file:
        data = pickle.load(file)

        # Run the tests
        euler_derivatives_test(tests=data["euler"])
        quat_derivatives_test(tests=data["quat"])

if __name__ == "__main__":
    # Generate tests
    generate_tests()

    # Run tests
    run_tests()
