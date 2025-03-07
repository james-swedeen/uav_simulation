"""ch5_derivatives_tests.py: Implements some basic tests for the chapter 5 partial."""


import os
import pickle
from typing import Any, cast

import numpy as np
from mav_sim.chap3.mav_dynamics import IND
from mav_sim.chap5.compute_models import compute_ss_model
from mav_sim.chap5.trim import (
    trim_objective_fun,
    variable_bounds,
    velocity_constraint,
    velocity_constraint_partial,
)
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.tools import types


#############  Test structure definitions ########################
class VelocityConstraintTest:
    """Stores a test for the velocity constraint"""
    def __init__(self) -> None:
        # Inputs
        self.x: types.NP_MAT
        self.Va_desired: float

        # Output
        self.out: float

    def __str__(self) -> str:
        """Outputs the object in a nice format"""
        out = "Inputs:\nx:\n" + str(self.x) + \
              "\nVa_desired: " + str(self.Va_desired) + \
              "\nOutput: " + str(self.out)
        return out

class VelocityConstraintPartialTest:
    """Stores a test for the velocity constraint partial"""
    def __init__(self) -> None:
        # Inputs
        self.x: types.NP_MAT

        # Output
        self.out: list[float]

    def __str__(self) -> str:
        """Outputs the object in a nice format"""
        out = "Inputs:\nx:\n" + str(self.x) + \
              "\nOutput\n: " + str(self.out)
        return out

class VariableBoundsTest:
    """Stores a test for the Variable bounds"""
    def __init__(self) -> None:
        # Inputs:
        self.state0: types.NP_MAT
        self.eps: float

        # Outputs:
        self.lb: list[float]
        self.ub: list[float]

    def __str__(self) -> str:
        """Outputs the object in a nice format"""
        out = "Inputs:\nstate0:\n" + str(self.state0) + \
              "\neps: " + str(self.eps) + \
              "\nOutput\nlb: " + str(self.lb) + \
              "\nub: " + str(self.ub)
        return out

class TrimObjectiveFunTest:
    """Stores a test for the trim_objective_fun"""
    def __init__(self) -> None:
        # Inputs:
        self.x: types.NP_MAT
        self.Va: float
        self.gamma: float
        self.R: float
        self.psi_weight: float

        # Outputs
        self.out: float

    def __str__(self) -> str:
        """Outputs the object in a nice format"""
        out = "Inputs:x:\n" + str(self.x) + \
              "\nVa: " + str(self.Va) + \
              "\ngamma: " + str(self.gamma) + \
              "\nR: " + str(self.R) + \
              "\npsi_weight: " + str(self.psi_weight) + \
              "\nOutput\nout: " + str(self.out)
        return out

#############  Auto test generation ######################
def generate_tests()-> None:
    """Generates and saves a pickle file with the randomly generated tests"""
    # Generate tests
    data: dict[str,Any] = {}
    data["vel_constraint"] = generate_velocity_constraint_tests()
    data["vel_constraint_partial"] = generate_velocity_constraint_partial_tests()
    data["variable_bounds"] = generate_variable_bounds_tests()
    data["trim_objective_fun"] = generate_trim_objective_fun_tests()

    # Save the tests
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch05_test_archive.pkl"
        ),
        "wb",
    ) as file:
        pickle.dump(data, file)

def generate_velocity_constraint_tests(num_tests: int = 1000) -> list[VelocityConstraintTest]:
    """Generates random test data and stores the results"""
    tests: list[VelocityConstraintTest] = []

    # Generate the results for each test
    for _ in range(num_tests):
        # Generate the state
        test = VelocityConstraintTest()
        test.x = np.random.randn(16,1)*100.
        test.Va_desired = np.random.randn()*30.

        # Generate the output
        test.out = velocity_constraint(x=test.x, Va_desired=test.Va_desired)

        # Store the test
        tests.append(test)

    return tests

def generate_velocity_constraint_partial_tests(num_tests: int = 1000) -> list[VelocityConstraintPartialTest]:
    """Generates random test data and stores the results"""
    tests: list[VelocityConstraintPartialTest] = []

    # Generate the results for each test
    for _ in range(num_tests):
        # Generate the state
        test = VelocityConstraintPartialTest()
        test.x = np.random.randn(16,1)*100.

        # Generate the output
        test.out = velocity_constraint_partial(x=test.x)

        # Store the test
        tests.append(test)

    return tests

def generate_variable_bounds_tests(num_tests: int = 1000) -> list[VariableBoundsTest]:
    """Generates random test data and stores the results"""
    tests: list[VariableBoundsTest] = []

    # Generate the results for each test
    for _ in range(num_tests):
        # Generate the state
        test = VariableBoundsTest()
        test.state0 = np.random.randn(12,1)*100.
        test.eps = np.random.randn()

        # Generate the output
        test.lb, test.ub = variable_bounds(state0=test.state0, eps=test.eps)

        # Store the test
        tests.append(test)

    return tests

def generate_trim_objective_fun_tests(num_tests: int = 1000) -> list[TrimObjectiveFunTest]:
    """Generates random test data and stores the results"""
    tests: list[TrimObjectiveFunTest] = []

    # Generate the results for each test
    for k in range(num_tests):
        # Generate the state
        test = TrimObjectiveFunTest()
        test.x = np.random.randn(16,1)*100.
        test.Va = np.random.randn()*30.
        test.gamma = np.random.randn()*np.pi/2.
        test.R = np.random.randn()*1000.
        test.psi_weight = np.random.randn()*1.e6

        # Set R to infinity for straight line
        if k % 3 == 1:
            test.R = np.inf

        if k % 5 == 1:
            test.gamma = 0.

        # Generate the output
        test.out = trim_objective_fun(x=test.x, Va=test.Va, gamma=test.gamma, R=test.R, psi_weight=test.psi_weight)

        # Store the test
        tests.append(test)

    return tests

#############  auto test run ######################
def run_auto_tests()->None:
    """Runs all of the auto-generated tests"""
    # Load the tests
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "ch05_test_archive.pkl"
        ),
        "rb",
    ) as file:
        data = pickle.load(file)

        # Run the tests
        velocity_constraint_test(tests=data["vel_constraint"])
        velocity_constraint_partial_test(tests=data["vel_constraint_partial"])
        variable_bounds_test(tests=data["variable_bounds"])
        trim_objective_fun_test(tests=data["trim_objective_fun"])

def velocity_constraint_test(tests: list[VelocityConstraintTest], threshold: float = 1e-4) -> bool:
    """Runs the test for the velocity constraint"""
    # Evaluate the results
    print("\nStarting velocity_constraint test")
    success = True
    for test in tests:
        # Generate the output
        out = velocity_constraint(x=test.x, Va_desired=test.Va_desired)

        # Compare the output with the result
        diff = np.linalg.norm(out-test.out)
        if diff > threshold:
            print("\n\nFailed test!")
            print("test:\n", test, "\nactual out: ", out)
            success = False

    # Indicate success
    if success:
        print("Passed test on velocity_constraint")
    return success

def velocity_constraint_partial_test(tests: list[VelocityConstraintPartialTest], threshold: float = 1e-4) -> bool:
    """Runs the test for the velocity constraint partial"""
    # Evaluate the results
    print("\nStarting velocity_constraint_partial test")
    success = True
    for test in tests:
        # Generate the output
        out = velocity_constraint_partial(x=test.x)

        # Compare the output with the result
        diff = np.linalg.norm(np.array([out])-np.array([test.out]) )
        if diff > threshold:
            print("\n\nFailed test!")
            print("test:\n", test, "\nactual out: ", out)
            success = False

    # Indicate success
    if success:
        print("Passed test on velocity_constraint_partial")
    return success

def variable_bounds_test(tests: list[VariableBoundsTest], threshold: float = 1e-4) -> bool:
    """Runs the test for the variable_bounds"""
    # Evaluate the results
    print("\nStarting variable_bounds test")
    success = True
    for test in tests:
        # Generate the output
        lb, ub = variable_bounds(state0=test.state0, eps=test.eps)

        # Compare the output with the result
        diff = 0.
        for low in zip(test.lb, lb):
            if np.isinf(low[0]):
                if not np.isinf(low[1]):
                    diff += 10.*threshold
            else:
                diff += cast(float, np.linalg.norm(low[0]-low[1]) )
        for low in zip(test.ub, ub):
            if np.isinf(low[0]):
                if not np.isinf(low[1]):
                    diff += 10.*threshold
            else:
                diff += cast(float, np.linalg.norm(low[0]-low[1]) )

        if diff > threshold:
            print("\n\nFailed test!")
            print("test:\n", test, "\nactual lb: ", lb, "\nactual ub: ", ub)
            success = False

    # Indicate success
    if success:
        print("Passed test on variable_bounds")
    return success

def trim_objective_fun_test(tests: list[TrimObjectiveFunTest], threshold: float = 1.) -> bool:
    """Runs the test for the trim_objective_fun"""
    # Evaluate the results
    print("\nStarting trim_objective_fun test")
    success = True
    for test in tests:
        # Generate the output
        out = trim_objective_fun(x=test.x, Va=test.Va, gamma=test.gamma, R=test.R, psi_weight=test.psi_weight)

        # Compare the output with the result
        diff = np.abs(out-test.out)

        if diff > threshold:
            print("\n\nFailed test!")
            print("test:\n", test, "\nactual out: ", str(out))
            success = False

    # Indicate success
    if success:
        print("Passed test on trim_objective_fun")
    return success

if __name__ == "__main__":
    #generate_tests()
    run_auto_tests()
