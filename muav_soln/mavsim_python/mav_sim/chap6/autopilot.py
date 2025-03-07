"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
        12/21 - GND
"""
from typing import Optional

import mav_sim.parameters.control_parameters as AP
import numpy as np
from mav_sim.chap6.pd_control_with_rate import PDControlWithRate
from mav_sim.chap6.pi_control import PIControl
from mav_sim.chap6.tf_control import TFControl
from mav_sim.message_types.msg_autopilot import MsgAutopilot
from mav_sim.message_types.msg_delta import MsgDelta
from mav_sim.message_types.msg_state import MsgState

# from mav_sim.tools.transfer_function import TransferFunction
from mav_sim.tools.wrap import saturate, wrap


class SequentialLoopClosureParameters:
    """Defines all of the autopilot parameters for initialization of the sequential
       loop closure
    """
    def __init__(self) -> None:
        # Roll from aileron
        self.roll_kp=AP.roll_kp         # Proportional gain
        self.roll_kd=AP.roll_kd         # Derivative gain
        self.roll_limit=np.radians(45)  # Limit on roll angle (radians)

        # Course from roll
        self.course_kp=AP.course_kp         # Proportional gain
        self.course_ki=AP.course_ki         # Integral gain
        self.course_limit=np.radians(30)    # Course angle limit (radians)

        # Yaw damper
        self.yaw_k=AP.yaw_damper_kr         # k: TF control gain
        self.yaw_n0=0.0                     # n0: numerator constant
        self.yaw_n1=1.0                     # n1: numerator scaling of s
        self.yaw_d0=AP.yaw_damper_p_wo      # d0: denominator constant
        self.yaw_d1=1                       # d1: denominator scaling of s

        # Pitch from elevator
        self.pitch_kp=AP.pitch_kp           # Proportional gain
        self.pitch_kd=AP.pitch_kd           # Derivative gain
        self.pitch_limit=np.radians(45)     # Limit on pitch angle (radians)

        # Altitude from pitch
        self.alt_kp=AP.altitude_kp          # Proportional gain
        self.alt_ki=AP.altitude_ki          # Integral gain
        self.alt_limit=np.radians(30)       # Altitude change limit TODO: Change from radians

        # Airspeed from throttle
        self.va_kp=AP.airspeed_throttle_kp  # Proportional gain
        self.va_ki=AP.airspeed_throttle_ki  # Integral gain
        self.va_limit=1.0                   # Airspeed limit


class Autopilot:
    """Creates an autopilot for controlling the mav to desired values
    """
    def __init__(self, ts_control: float, gains: Optional[SequentialLoopClosureParameters] = None) -> None:
        """Initialize the lateral and longitudinal controllers

        Args:
            ts_control: time step for the control
        """
        # Initialize gains
        if gains is None:
            gains = SequentialLoopClosureParameters()

        # instantiate lateral-directional controllers
        self.roll_from_aileron = PDControlWithRate( # Section 6.1.1.1
                        kp=gains.roll_kp,
                        kd=gains.roll_kd,
                        limit=np.radians(45))
        self.course_from_roll = PIControl( # Section 6.1.1.2
                        kp=gains.course_kp,
                        ki=gains.course_ki,
                        Ts=ts_control,
                        limit=gains.course_limit)

        self.yaw_damper = TFControl( # Note that this is not in the book. This is a replacement for yawDamper function on page 111
                        k=gains.yaw_k,
                        n0=gains.yaw_n0,
                        n1=gains.yaw_n1,
                        d0=gains.yaw_d0,
                        d1=gains.yaw_d1,
                        Ts=ts_control)

        # instantiate longitudinal controllers
        self.pitch_from_elevator = PDControlWithRate( # Section 6.1.2.1
                        kp=gains.pitch_kp,
                        kd=gains.pitch_kd,
                        limit=gains.pitch_limit)
        self.altitude_from_pitch = PIControl( # Section 6.1.2.2
                        kp=gains.alt_kp,
                        ki=gains.alt_ki,
                        Ts=ts_control,
                        limit=gains.alt_limit)
        self.airspeed_from_throttle = PIControl( # Section 6.1.2.3
                        kp=gains.va_kp,
                        ki=gains.va_ki,
                        Ts=ts_control,
                        limit=gains.va_limit)
        self.commanded_state = MsgState()

    def update(self, cmd: MsgAutopilot, state: MsgState) -> tuple[MsgDelta, MsgState]:
        """Given a state and autopilot command, compute the control to the mav

        Args:
            cmd: command to the autopilot
            state: current state of the mav

        Returns:
            delta: low-level flap commands
            commanded_state: the state being commanded
        """

        # lateral autopilot
        chi_c = wrap(cmd.course_command, state.chi)
        phi_c = saturate( # course hold loop, last equation in 6.1.1.2 with addition of feedforward term
            cmd.phi_feedforward + self.course_from_roll.update(chi_c, state.chi), # course_from_roll is a PI_Control
            -np.radians(30), np.radians(30)) # Values for saturation +/- 30 degrees

        delta_a = self.roll_from_aileron.update(phi_c, state.phi, state.p) # Section 6.1.1.1 (last equation),
                                                                           # roll_from_aileron is a PDControlWithRate

        delta_r = self.yaw_damper.update(state.r) # yaw_damper is instance of TFControl, variation on Section 6.1.1.4 equations

        # longitudinal autopilot
        # saturate the altitude command
        altitude_c = saturate(cmd.altitude_command, state.altitude - AP.altitude_zone, # Saturate command altitude to be
                            state.altitude + AP.altitude_zone)                         # close to current altitude

        theta_c = self.altitude_from_pitch.update(altitude_c, state.altitude) # Section 6.1.2.2 (last equation),
                                                                              # altitude_from_pitch is PIControl

        delta_e = self.pitch_from_elevator.update(theta_c, state.theta, state.q) # Section 6.1.2.1 (last equation),
                                                                                 # pitch_from_elevator is a PDControlWithRate

        delta_t = self.airspeed_from_throttle.update(cmd.airspeed_command, state.Va) # Section 6.1.2.3 (last equation),
                                                                                     # airspeed_from_throttle is PIControl
        delta_t = saturate(delta_t, 0.0, 1.0)

        # construct control outputs and commanded states
        delta = MsgDelta(elevator=delta_e,
                         aileron=delta_a,
                         rudder=delta_r,
                         throttle=delta_t)
        self.commanded_state.altitude = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = phi_c
        self.commanded_state.theta = theta_c
        self.commanded_state.chi = cmd.course_command
        return delta, self.commanded_state.copy()
