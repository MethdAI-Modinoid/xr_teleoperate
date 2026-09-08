"""
Robot-less arm controller for headset-only recording.

`G1_29_ArmController` talks DDS to a physical G1 and blocks in its constructor until a
`LowState_` message arrives::

    while not self.lowstate_buffer.GetData():
        time.sleep(0.1)
        logger_mp.warning("[G1_29_ArmController] Waiting to subscribe dds...")

With no robot and no simulator publishing that topic, this never returns. That single loop
(and its twin in `Dex3_1_Controller`) is the only thing preventing a Meta headset plus a PC
from recording episodes on its own -- everything else in the pipeline (televuer's hand
tracking, the Pinocchio/CasADi IK, dex-retargeting, EpisodeWriter) is robot-independent.

This module supplies a drop-in replacement that keeps the arm state in memory instead. See
`../../XR_SWAP_PLAN.md` work item A.

WHAT IS AND IS NOT SIMULATED
----------------------------
This is deliberately NOT a dynamics simulation. It reproduces the real controller's
*command path* -- the 250 Hz loop that velocity-clips the commanded target -- and treats the
result as the achieved state. It does not model tracking error, gravity sag, or compliance.

That choice matters because `G1_29_ArmIK.solve_ik` uses whatever `get_current_dual_arm_q()`
returns as BOTH its warm start and its smoothness anchor (`init_data` / `var_q_last`), so
this class's output feeds directly back into the recorded trajectory. Two notes on why this
particular model is the honest one:

  * In normal operation the clip does not bind. The limit is 20-30 rad/s at a 1/250 s tick,
    i.e. up to 0.08 rad per tick, or ~0.67 rad per 30 Hz frame -- far more than a human hand
    moves between frames. So the state converges to the commanded target within a frame, and
    feeding it back is equivalent to `solve_ik(..., current_lr_arm_motor_q=None)`, which is
    that function's own documented fallback. We are not inventing behaviour.
  * The clip still binds on a pathological IK jump, which is exactly when it should, and
    keeps the recorded trajectory to something a real G1 could execute.

Inventing a fake tracking lag instead would damp the recorded trajectory by an amount with
no physical justification, and would be indistinguishable from a real problem later.
"""

import threading
import time
from enum import IntEnum

import numpy as np

import logging_mp

logger_mp = logging_mp.getLogger(__name__)

# Mirrors G1_29_JointIndex / G1_29_Num_Motors in robot_arm.py. Duplicated rather than
# imported because that module imports unitree_sdk2py at module scope, which is precisely
# the dependency a robot-less station should not need.
G1_29_NUM_MOTORS = 35


class G1_29_JointArmIndex(IntEnum):
    """Full-body motor indices of the 14 arm joints, left arm first -- the order every
    `*_dual_arm_*` array in this codebase uses."""
    kLeftShoulderPitch = 15
    kLeftShoulderRoll = 16
    kLeftShoulderYaw = 17
    kLeftElbow = 18
    kLeftWristRoll = 19
    kLeftWristPitch = 20
    kLeftWristyaw = 21

    kRightShoulderPitch = 22
    kRightShoulderRoll = 23
    kRightShoulderYaw = 24
    kRightElbow = 25
    kRightWristRoll = 26
    kRightWristPitch = 27
    kRightWristYaw = 28


class G1_29_VirtualArmController:
    """
    In-memory stand-in for `G1_29_ArmController`.

    Implements the public surface `teleop_hand_and_arm.py` actually uses:
    `speed_gradual_max`, `speed_instant_max`, `get_current_motor_q`,
    `get_current_dual_arm_q`, `get_current_dual_arm_dq`, `ctrl_dual_arm`, and
    `ctrl_dual_arm_go_home`. Constructor signature matches so it is a drop-in.

    Args:
        motion_mode: accepted for signature compatibility; no effect (there is no robot to
            put into a motion mode).
        simulation_mode: when True, skip velocity clipping entirely, matching the real
            controller's own behaviour in simulation.
        control_dt: command-loop period, matching the real controller's 1/250 s.
    """

    def __init__(self, motion_mode: bool = False, simulation_mode: bool = False,
                 control_dt: float = 1.0 / 250.0):
        logger_mp.info("Initialize G1_29_VirtualArmController (no robot, no DDS)...")

        self.motion_mode = motion_mode
        self.simulation_mode = simulation_mode
        self.control_dt = control_dt

        self.q_target = np.zeros(14)
        self.tauff_target = np.zeros(14)

        # Achieved state. Zero is the G1's URDF zero pose (arms hanging at its sides), which
        # is a reasonable, well-defined starting configuration.
        self._q = np.zeros(14)
        self._dq = np.zeros(14)

        self.arm_velocity_limit = 20.0
        self._speed_gradual_max = False
        self._gradual_start_time = None
        self._gradual_time = None

        self.ctrl_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._running = True

        self.publish_thread = threading.Thread(target=self._ctrl_motor_state, daemon=True)
        self.publish_thread.start()

        logger_mp.info("Initialize G1_29_VirtualArmController OK!")

    # -- the command loop -------------------------------------------------------------

    def clip_arm_q_target(self, target_q, velocity_limit):
        """Identical to the real controller's: scale the whole step down uniformly so no
        joint exceeds `velocity_limit`, preserving the motion's direction."""
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        motion_scale = np.max(np.abs(delta)) / (velocity_limit * self.control_dt)
        return current_q + delta / max(motion_scale, 1.0)

    def _ctrl_motor_state(self):
        """Mirror of the real `_ctrl_motor_state`, minus the DDS write: clip the commanded
        target, then adopt it as the achieved state."""
        while self._running:
            start_time = time.time()

            with self.ctrl_lock:
                arm_q_target = np.asarray(self.q_target, dtype=float).copy()

            if self.simulation_mode:
                clipped = arm_q_target
            else:
                clipped = self.clip_arm_q_target(arm_q_target,
                                                 velocity_limit=self.arm_velocity_limit)

            with self._state_lock:
                # Finite-difference velocity, so get_current_dual_arm_dq() is real data
                # rather than a hardcoded zero.
                self._dq = (clipped - self._q) / self.control_dt
                self._q = clipped

            if self._speed_gradual_max is True:
                t_elapsed = start_time - self._gradual_start_time
                self.arm_velocity_limit = 20.0 + (10.0 * min(1.0, t_elapsed / 5.0))

            time.sleep(max(0, self.control_dt - (time.time() - start_time)))

    # -- public API -------------------------------------------------------------------

    def ctrl_dual_arm(self, q_target, tauff_target):
        """Set the target q & tau for the 14 arm motors."""
        with self.ctrl_lock:
            self.q_target = np.asarray(q_target, dtype=float).copy()
            self.tauff_target = np.asarray(tauff_target, dtype=float).copy()

    def get_current_dual_arm_q(self):
        """Current q of the 14 arm motors, left arm first."""
        with self._state_lock:
            return self._q.copy()

    def get_current_dual_arm_dq(self):
        """Current dq of the 14 arm motors, left arm first."""
        with self._state_lock:
            return self._dq.copy()

    def get_current_motor_q(self):
        """
        Current q of ALL body motors, as a length-35 array indexed by full-body motor id.

        Only the 14 arm entries are ever non-zero here: a robot-less station has no legs or
        waist to report, and nothing drives them. Provided because the `dex1`/`controller`
        recording branches call `.tolist()` on this.
        """
        full = np.zeros(G1_29_NUM_MOTORS)
        with self._state_lock:
            for idx, joint in enumerate(G1_29_JointArmIndex):
                full[int(joint)] = self._q[idx]
        return full

    def ctrl_dual_arm_go_home(self):
        """Drive both arms back to the zero pose, waiting for them to arrive."""
        logger_mp.info("[G1_29_VirtualArmController] ctrl_dual_arm_go_home start...")
        with self.ctrl_lock:
            self.q_target = np.zeros(14)
        tolerance = 0.05
        for _ in range(100):
            if np.all(np.abs(self.get_current_dual_arm_q()) < tolerance):
                logger_mp.info("[G1_29_VirtualArmController] both arms have reached the home position.")
                return
            time.sleep(0.05)
        logger_mp.warning("[G1_29_VirtualArmController] go_home timed out before reaching zero.")

    def speed_gradual_max(self, t: float = 5.0):
        """Ramp the arm velocity limit to its maximum over `t` seconds."""
        self._gradual_start_time = time.time()
        self._gradual_time = t
        self._speed_gradual_max = True

    def speed_instant_max(self):
        """Raise the arm velocity limit to maximum immediately."""
        self.arm_velocity_limit = 30.0

    def close(self):
        """Stop the command loop."""
        self._running = False
        if self.publish_thread.is_alive():
            self.publish_thread.join(timeout=1.0)
